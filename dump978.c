//
// Copyright 2015, Oliver Jowett <oliver@mutability.co.uk>
//

// This file is free software: you may copy, redistribute and/or modify it  
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your  
// option) any later version.  
//
// This file is distributed in the hope that it will be useful, but  
// WITHOUT ANY WARRANTY; without even the implied warranty of  
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License  
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "uat.h"
#include "uat_decode.h"
#include "fec/rs.h"

void *rs_uplink;
void *rs_adsb_short;
void *rs_adsb_long;

void make_atan2_table();
void read_from_stdin();
int process_buffer(uint8_t *input, int len, uint64_t offset);
int decode_adsb_frame(uint64_t timestamp, uint8_t *input);
int decode_uplink_frame(uint64_t timestamp, uint8_t *input);

#define UPLINK_POLY 0x187
#define ADSB_POLY 0x187

static int raw_mode = 0;

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "-raw"))
        raw_mode = 1;

    rs_adsb_short = init_rs_char(8, /* gfpoly */ ADSB_POLY, /* fcr */ 120, /* prim */ 1, /* nroots */ 12, /* pad */ 225);
    rs_adsb_long  = init_rs_char(8, /* gfpoly */ ADSB_POLY, /* fcr */ 120, /* prim */ 1, /* nroots */ 14, /* pad */ 207);
    rs_uplink     = init_rs_char(8, /* gfpoly */ UPLINK_POLY, /* fcr */ 120, /* prim */ 1, /* nroots */ 20, /* pad */ 163);

    make_atan2_table();
    read_from_stdin();
    return 0;
}

static void dump_raw_message(char updown, uint8_t *data, int len)
{
    int i;

    fprintf(stdout, "%c", updown);
    for (i = 0; i < len; ++i) {
        fprintf(stdout, "%02x", data[i]);
    }

    fprintf(stdout, ";\n");
}


uint16_t atan2_table[65536]; // contains value [0..65536) -> [0, 2*pi)
#define iqphase(i,q) (atan2_table[i*256 + q])

void make_atan2_table()
{
    int i,q;

    for (i = 0; i < 256; ++i) {
        for (q = 0; q < 256; ++q) {
            double d_i = (i - 127.5);
            double d_q = (q - 127.5);
            double ang = atan2(d_q, d_i) + M_PI; // atan2 returns [-pi..pi], normalize to [0..2*pi]
            double scaled_ang = round(32768 * ang / M_PI);
            
            iqphase(i,q) = (scaled_ang < 0 ? 0 : scaled_ang > 65535 ? 65535 : (uint16_t)scaled_ang);
        }
    }
}


// 36 bit sync word
#define ADSB_SYNC_WORD   0xEACDDA4E2UL
#define UPLINK_SYNC_WORD 0x153225B1DUL
#define SYNC_MASK 0xFFFFFFFFFUL

#define SYNC_LENGTH (36)
#define ADSB_FRAME_LENGTH (272+112)
#define UPLINK_FRAME_LENGTH ((576+160)*6)

void read_from_stdin()
{
    uint8_t buffer[65536*4];
    int n;
    int used = 0;
    uint64_t offset = 0;
    
    while ( (n = read(0, buffer+used, sizeof(buffer)-used)) > 0 ) {
        int processed;

        used += n;
        processed = process_buffer(buffer, used, offset);
        used -= processed;
        offset += processed;
        if (used > 0)
            memmove(buffer, buffer+processed, used);
    }
}

int find_average_dphi(uint8_t *input, uint64_t pattern, int16_t *center, int32_t *separation)
{
    int i;

    int32_t dphi_zero_total = 0;
    int zero_bits = 0;
    int32_t dphi_one_total = 0;
    int one_bits = 0;

    for (i = 0; i < 36; ++i) {
        uint16_t phi0 = iqphase(input[i*4+0], input[i*4+1]);
        uint16_t phi1 = iqphase(input[i*4+2], input[i*4+3]);
        int16_t delta_phi = phi1 - phi0;

        if (pattern & (1UL << (35-i))) {
            ++one_bits;
            dphi_one_total += delta_phi;
        } else {
            ++zero_bits;
            dphi_zero_total += delta_phi;
        }
    }

    dphi_zero_total /= zero_bits;
    dphi_one_total /= one_bits;

    *separation = dphi_one_total - dphi_zero_total;
    *center = (dphi_zero_total + dphi_one_total) / 2;

#if 0    
    fprintf(stdout, "zeroes %.0fkHz ones %.0fkHz separation %.0fkHz center %.0fkHz\n",
            dphi_zero_total * 2083334.0 / 65536 / 1000,
            dphi_one_total * 2083334.0 / 65536 / 1000,
            *separation * 2083334.0 / 65536 / 1000,
            *center * 2083334.0 / 65536 / 1000);
#endif

    // sanity check - 20kHz to 1MHz separation
    if (*separation < (65536 * (20e3 / 2083334.0)) || *separation > (65536 * (1000e3 / 2083334.0)))
        return 0;
    else
        return 1;
}

int process_buffer(uint8_t *input, int len, uint64_t offset)
{
    uint16_t last_phi = iqphase(input[0], input[1]);
    uint64_t sync0 = 0, sync1 = 0;
    int i;    

    // We expect samples at twice the UAT bitrate.
    // We look at phase difference between pairs of adjacent samples, i.e.
    //  sample 1 - sample 0   -> sync0
    //  sample 2 - sample 1   -> sync1
    //  sample 3 - sample 2   -> sync0
    //  sample 4 - sample 3   -> sync1
    // ...
    //
    // We accumulate bits into two buffers, sync0 and sync1.
    // Then we compare those buffers to the expected 36-bit sync word that
    // should be at the start of each UAT frame. When (if) we find it,
    // that tells us which sample to start decoding from.

    // Stop when we run out of samples for a full sync word plus max-sized frame;
    // our caller will pass the remaining data back to us as the
    // start of the buffer next time. This means we don't need to maintain
    // state between calls.
    for (i = 2; i+(SYNC_LENGTH+UPLINK_FRAME_LENGTH+3)*4 < len; i += 4) {
        uint16_t phi0 = iqphase(input[i+0], input[i+1]);
        uint16_t phi1 = iqphase(input[i+2], input[i+3]);
        int16_t dphi0 = phi0 - last_phi;  // don't need to worry about wrapping at +/-pi, the width of the datatype does it for us
        int16_t dphi1 = phi1 - phi0;

        last_phi = phi1;

        // accumulate sync words
        sync0 = ((sync0 << 1) | (dphi0 < 0 ? 0 : 1));
        sync1 = ((sync1 << 1) | (dphi1 < 0 ? 0 : 1));

        //fprintf(stdout, "%09lx %09lx\n", sync0 & SYNC_MASK, sync1 & SYNC_MASK);


        // see if we have a valid sync word
        if ((sync0 & SYNC_MASK) == ADSB_SYNC_WORD) {
            int skip = decode_adsb_frame(offset+i+2, input+i+2);
            if (skip) {
                i = i + 2 + skip - 4;
                continue;
            }                
        } else if ((sync0 & SYNC_MASK) == UPLINK_SYNC_WORD) {
            int skip = decode_uplink_frame(offset+i+2, input+i+2);
            if (skip) {
                i = i + 2 + skip - 4;
                continue;
            }
        }

        if ((sync1 & SYNC_MASK) == ADSB_SYNC_WORD) {
            int skip = decode_adsb_frame(offset+i+4, input+i+4);
            if (skip) {
                i = i + 4 + skip - 4;
                continue;
            }                
        } else if ((sync1 & SYNC_MASK) == UPLINK_SYNC_WORD) {
            int skip = decode_uplink_frame(offset+i+4, input+i+4);
            if (skip) {
                i = i + 4 + skip - 4;
                continue;
            }                
        }
    }

    return i;
}

void decode_latlng(int lat, int lng, double *wgs_lat, double *wgs_lng)
{
    if (lat <= 0x400000) {
        // 1st quadrant
        *wgs_lat = lat * 360.0 / 16777216.0;
    } else {
        // 4th quadrant
        *wgs_lat = -90 + (lat & 0x3fffff) * 360.0 / 16777216.0;
    }
    
    *wgs_lng = lng * 360.0 / 16777216.0;
    if (*wgs_lng > 180.0)
        *wgs_lng -= 360.0;        
} 

int decode_adsb_frame(uint64_t timestamp, uint8_t *input)
{
    int i;
    uint8_t framedata[ADSB_FRAME_LENGTH/8];
    int16_t average_dphi;
    int32_t separation;
    struct uat_adsb_mdb mdb;
    int n_corrected;

    if (!find_average_dphi(input-36*4, ADSB_SYNC_WORD, &average_dphi, &separation)) {
        return 0;
    }

    memset(framedata, 0, sizeof(framedata));

    for (i = 0; i < ADSB_FRAME_LENGTH; ++i) {
        uint16_t phi0 = iqphase(input[i*4+0], input[i*4+1]);
        uint16_t phi1 = iqphase(input[i*4+2], input[i*4+3]);
        int16_t dphi = phi1 - phi0;
        
        if (dphi > average_dphi)
            framedata[i/8] |= (1 << (7 - (i&7)));
    }

    if ((framedata[0] >> 3) == 0) {
        // "Basic UAT ADS-B Message": 144 data bits, 96 FEC bits
        n_corrected = decode_rs_char(rs_adsb_short, framedata, NULL, 0);
        if (n_corrected < 0)
            return 0;

        dump_raw_message('-', framedata, 144/8);
        if (!raw_mode) {
            uat_decode_adsb_mdb(framedata, &mdb);
            
            fprintf(stdout,
                    "%.6f   Basic UAT MDB received\n"
                    "=============================================\n",
                    timestamp / 2083334.0 / 2);
            uat_display_adsb_mdb(&mdb, stdout);
            fprintf(stdout, "=============================================\n\n");
        }

        return (144+96)*4;
    }
    
    // "Long UAT ADS-B Message": 272 data bits, 112 FEC bits
    n_corrected = decode_rs_char(rs_adsb_long, framedata, NULL, 0);
    if (n_corrected < 0)
        return 0;

    dump_raw_message('-', framedata, 272/8);
    if (!raw_mode) {
        uat_decode_adsb_mdb(framedata, &mdb);
        
        fprintf(stdout,
                "%.6f   Long UAT MDB received\n"
                "=============================================\n",
                timestamp / 2083334.0 / 2);
        uat_display_adsb_mdb(&mdb, stdout);
        fprintf(stdout, "=============================================\n\n");
    }

    return (272+112)*4;
}

void decode_fisb_apdu(uint8_t *pdu, int length)
{
    // I don't have a spec for this :(
    // Mostly cargo-culted from inspection of the lone-star-adsb decoder

    int id = ((pdu[0] & 0x1f) << 6) | ((pdu[1] & 0xfc) >> 2);
    fprintf(stdout,
            "     === FIS-B APDU ===\n"
            "      Product:\n"
            "       A:%d G:%d P:%d S:%d T:%d ID:%d\n"
            "       Hour: %d Min: %d\n",
            (pdu[0] & 0x80) ? 1 : 0,
            (pdu[0] & 0x40) ? 1 : 0,
            (pdu[0] & 0x20) ? 1 : 0,
            (pdu[1] & 0x02) ? 1 : 0,
            ((pdu[1] & 0x01) << 1) | ((pdu[2] & 0x80) >> 7),
            id,

            (pdu[2] & 0x7c) >> 2,
            ((pdu[2] & 0x03) << 4) | ((pdu[3] & 0xf0) >> 4));

    // TODO
}

void decode_uplink_app_data(uint8_t *blockdata)
{
    int i = 8;
    while (i < 422) {
        int length = (blockdata[i] << 1) | (blockdata[i+1] >> 7);
        int type = (blockdata[i+1] & 7);
        if (length == 0) {
            fprintf(stdout, 
                    "    (%d bytes trailing)\n",
                    422 - i);
            break; // no more data
        }

        fprintf(stdout,
                "   === INFORMATION FRAME ===\n"
                "    Start offset:      %d\n"
                "    Length:            %d\n"
                "    Type:              %d\n",
                i, length, type);

        if (i + 2 + length > 422) {
            fprintf(stdout, "    (length exceeds available data, halting decode)\n");
            break;
        } else {
            int j;

            fprintf(stdout, "    Data:              ");
            for (j = 0; j < length; ++j)
                fprintf(stdout, "%02x", blockdata[i + 2 + j]);
            fprintf(stdout, "\n");

            if (type == 0) {
                // FIS-B APDU
                decode_fisb_apdu(blockdata + i + 2, length);
            }
        }

        i = i + length + 2;
    }
}

void decode_uplink_mdb(uint8_t *blockdata)
{
    fprintf(stdout, 
            "   === UPLINK MDB ===\n");

    if (blockdata[5] & 1) {
        // position valid
        int lat = (blockdata[0] << 15) | (blockdata[1] << 7) | (blockdata[2] >> 1);
        int lng = ((blockdata[2] & 1) << 23) | (blockdata[3] << 15) | (blockdata[4] << 7) | (blockdata[5] >> 1);
        double wgs_lat = 0, wgs_lng = 0;

        decode_latlng(lat, lng, &wgs_lat, &wgs_lng);
        fprintf(stdout,
                "    GS Latitude:       %+3.3f (%d)\n"
                "    GS Longitude:      %+3.3f (%d)\n",
                wgs_lat, lat,
                wgs_lng, lng);
    }

    fprintf(stdout,
            "    UTC coupled:       %s\n"
            "    App data valid:    %s\n"
            "    Slot ID:           %d\n"
            "    TIS-B Site ID:     %d\n",
            (blockdata[6] & 0x80) ? "yes" : "no",
            (blockdata[6] & 0x20) ? "yes" : "no",
            (blockdata[6] & 0x1f),
            (blockdata[7] >> 4));

    if (blockdata[6] & 0x20)
        decode_uplink_app_data(blockdata);
}   

int decode_uplink_frame(uint64_t timestamp, uint8_t *input)
{
    int i, block;
    uint8_t framedata[UPLINK_FRAME_LENGTH/8];
    int16_t average_dphi;
    int32_t separation;
    uint8_t deinterleaved[432];

    if (!find_average_dphi(input-36*4, UPLINK_SYNC_WORD, &average_dphi, &separation)) {
        return 0;
    }

    memset(framedata, 0, sizeof(framedata));

    for (i = 0; i < UPLINK_FRAME_LENGTH; ++i) {
        uint16_t phi0 = iqphase(input[i*4+0], input[i*4+1]);
        uint16_t phi1 = iqphase(input[i*4+2], input[i*4+3]);
        int16_t dphi = phi1 - phi0;
        
        if (dphi > average_dphi)
            framedata[i/8] |= (1 << (7 - (i&7)));
    }

    // deinterleave blocks
    for (block = 0; block < 6; ++block) {
        uint8_t blockdata[92];
        int n_corrected;
        
        for (i = 0; i < 92; ++i)
            blockdata[i] = framedata[i * 6 + block];

        n_corrected = decode_rs_char(rs_uplink, blockdata, NULL, 0);
        if (n_corrected < 0)
            return 0;

        memcpy (deinterleaved + 72*block, blockdata, 72); // drop the trailing ECC part
    }

    dump_raw_message('+', deinterleaved, 72*6);
    if (!raw_mode) {
        fprintf(stdout,
                "%.6f   Uplink MDB received\n"
                "=============================================\n",
                timestamp / 2083334.0 / 2);
        //decode_uplink_mdb(deinterleaved);
        fprintf(stdout, "=============================================\n\n");
    }

    return UPLINK_FRAME_LENGTH*4;
}



