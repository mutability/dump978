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
int process_buffer(uint16_t *phi, int len, uint64_t offset);
int decode_adsb_frame(uint64_t timestamp, uint16_t *phi);
int decode_uplink_frame(uint64_t timestamp, uint16_t *phi);

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

static void dump_raw_message(char updown, uint8_t *data, int len, int rs_errors)
{
    int i;

    fprintf(stdout, "%c", updown);
    for (i = 0; i < len; ++i) {
        fprintf(stdout, "%02x", data[i]);
    }

    if (rs_errors)
        fprintf(stdout, ";rs=%d", rs_errors);
    fprintf(stdout, ";\n");
}


uint16_t iqphase[65536]; // contains value [0..65536) -> [0, 2*pi)

void make_atan2_table()
{
    unsigned i,q;
    union {
        uint8_t iq[2];
        uint16_t iq16;
    } u;

    for (i = 0; i < 256; ++i) {
        for (q = 0; q < 256; ++q) {
            double d_i = (i - 127.5);
            double d_q = (q - 127.5);
            double ang = atan2(d_q, d_i) + M_PI; // atan2 returns [-pi..pi], normalize to [0..2*pi]
            double scaled_ang = round(32768 * ang / M_PI);

            u.iq[0] = i;
            u.iq[1] = q;
            iqphase[u.iq16] = (scaled_ang < 0 ? 0 : scaled_ang > 65535 ? 65535 : (uint16_t)scaled_ang);
        }
    }
}


// 36 bit sync word
#define ADSB_SYNC_WORD   0xEACDDA4E2UL
#define UPLINK_SYNC_WORD 0x153225B1DUL

#define SYNC_BITS (36)

#define SHORT_FRAME_DATA_BITS (144)
#define SHORT_FRAME_BITS (SHORT_FRAME_DATA_BITS+96)
#define SHORT_FRAME_DATA_BYTES (SHORT_FRAME_DATA_BITS/8)
#define SHORT_FRAME_BYTES (SHORT_FRAME_BITS/8)

#define LONG_FRAME_DATA_BITS (272)
#define LONG_FRAME_BITS (LONG_FRAME_DATA_BITS+112)
#define LONG_FRAME_DATA_BYTES (LONG_FRAME_DATA_BITS/8)
#define LONG_FRAME_BYTES (LONG_FRAME_BITS/8)

#define UPLINK_BLOCK_DATA_BITS (576)
#define UPLINK_BLOCK_BITS (UPLINK_BLOCK_DATA_BITS+160)
#define UPLINK_BLOCK_DATA_BYTES (UPLINK_BLOCK_DATA_BITS/8)
#define UPLINK_BLOCK_BYTES (UPLINK_BLOCK_BITS/8)

#define UPLINK_FRAME_BLOCKS (6)
#define UPLINK_FRAME_DATA_BITS (UPLINK_FRAME_BLOCKS * UPLINK_BLOCK_DATA_BITS)
#define UPLINK_FRAME_BITS (UPLINK_FRAME_BLOCKS * UPLINK_BLOCK_BITS)
#define UPLINK_FRAME_DATA_BYTES (UPLINK_FRAME_DATA_BITS/8)
#define UPLINK_FRAME_BYTES (UPLINK_FRAME_BITS/8)

static void convert_to_phi(uint16_t *buffer, int n)
{
    int i;

    for (i = 0; i < n; ++i)
        buffer[i] = iqphase[buffer[i]];
}

void read_from_stdin()
{
    char buffer[65536*2];
    int n;
    int used = 0;
    uint64_t offset = 0;
    
    while ( (n = read(0, buffer+used, sizeof(buffer)-used)) > 0 ) {
        int processed;

        convert_to_phi((uint16_t*) (buffer+(used&~1)), ((used&1)+n)/2);

        used += n;
        processed = process_buffer((uint16_t*) buffer, used/2, offset);
        used -= processed * 2;
        offset += processed;
        if (used > 0) {
            memmove(buffer, buffer+processed*2, used);
        }
    }
}

// maximum number of bit errors to permit in the sync word
#define MAX_SYNC_ERRORS 2

// check that there is a valid sync word starting at 'phi'
// that matches the sync word 'pattern'. Place the dphi
// threshold to use for bit slicing in '*center'. Return 1
// if the sync word is OK, 0 on failure
int check_sync_word(uint16_t *phi, uint64_t pattern, int16_t *center)
{
    int i;
    int32_t dphi_zero_total = 0;
    int zero_bits = 0;
    int32_t dphi_one_total = 0;
    int one_bits = 0;
    int error_bits;

    // find mean dphi for zero and one bits;
    // take the mean of the two as our central value

    for (i = 0; i < 36; ++i) {
        int16_t dphi = phi[i*2+1] - phi[i*2];

        if (pattern & (1UL << (35-i))) {
            ++one_bits;
            dphi_one_total += dphi;
        } else {
            ++zero_bits;
            dphi_zero_total += dphi;
        }
    }

    dphi_zero_total /= zero_bits;
    dphi_one_total /= one_bits;

    *center = (dphi_one_total + dphi_zero_total) / 2;

    // recheck sync word using our center value
    error_bits = 0;
    for (i = 0; i < 36; ++i) {
        int16_t dphi = phi[i*2+1] - phi[i*2];

        if (pattern & (1UL << (35-i))) {
            if (dphi < *center)
                ++error_bits;
        } else {
            if (dphi >= *center)
                ++error_bits;
        }
    }

    //fprintf(stdout, "check_sync_word: center=%.0fkHz, errors=%d\n", *center * 2083334.0 / 65536 / 1000, error_bits);

    return (error_bits <= MAX_SYNC_ERRORS);
}

int process_buffer(uint16_t *phi, int len, uint64_t offset)    
{
    uint64_t sync0 = 0, sync1 = 0;
    int lenbits;
    int bit;

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

    // Stop when we run out of remaining samples for a max-sized frame.
    // Arrange for our caller to pass the trailing data back to us next time;
    // ensure we don't consume any partial sync word we might be part-way
    // through. This means we don't need to maintain state between calls.

    // We actually only look for the first CHECK_BITS of the sync word here.
    // If there's a match, the frame demodulators will derive a center offset
    // from the full word and then use that to re-check the sync word. This
    // lets us grab a few more marginal frames.

    // 18 seems like a good tradeoff between recovering more frames
    // and excessive false positives
#define CHECK_BITS 18
#define CHECK_MASK ((1UL<<CHECK_BITS)-1)
#define CHECK_ADSB (ADSB_SYNC_WORD >> (SYNC_BITS-CHECK_BITS))
#define CHECK_UPLINK (UPLINK_SYNC_WORD >> (SYNC_BITS-CHECK_BITS))

    lenbits = len/2 - ((SYNC_BITS-CHECK_BITS) + UPLINK_FRAME_BITS);
    for (bit = 0; bit < lenbits; ++bit) {
        int16_t dphi0 = phi[bit*2+1] - phi[bit*2];
        int16_t dphi1 = phi[bit*2+2] - phi[bit*2+1];

        sync0 = ((sync0 << 1) | (dphi0 > 0 ? 1 : 0));
        sync1 = ((sync1 << 1) | (dphi1 > 0 ? 1 : 0));

        if (bit < CHECK_BITS)
            continue; // haven't fully populated sync0/1 yet

        // see if we have (the start of) a valid sync word
        // It would be nice to look at popcount(expected ^ sync) 
        // so we can tolerate some errors, but that turns out
        // to be very expensive to do on every sample

        if ((sync0 & CHECK_MASK) == CHECK_ADSB) {
            int startbit = bit-CHECK_BITS+1;
            int skip = decode_adsb_frame(offset+startbit*2, phi+startbit*2);
            if (skip) {
                bit = startbit + skip;
                continue;
            }                
        } else if ((sync0 & CHECK_MASK) == CHECK_UPLINK) {
            int startbit = bit-CHECK_BITS+1;
            int skip = decode_uplink_frame(offset+startbit*2, phi+startbit*2);
            if (skip) {
                bit = startbit + skip;
                continue;
            }
        }

        if ((sync1 & CHECK_MASK) == CHECK_ADSB) {
            int startbit = bit-CHECK_BITS+1;
            int skip = decode_adsb_frame(offset+startbit*2+1, phi+startbit*2+1);
            if (skip) {
                bit = startbit + skip;
                continue;
            }
        } else if ((sync1 & CHECK_MASK) == CHECK_UPLINK) {
            int startbit = bit-CHECK_BITS+1;
            int skip = decode_uplink_frame(offset+startbit*2+1, phi+startbit*2+1);
            if (skip) {
                bit = startbit + skip;
                continue;
            }                
        }
    }

    return (bit - CHECK_BITS)*2;
}

// demodulate 'bytes' bytes from samples at 'phi' into 'frame',
// using 'center_dphi' as the bit slicing threshold
static void demod_frame(uint16_t *phi, uint8_t *frame, int bytes, int16_t center_dphi)
{
    while (--bytes >= 0) {
        uint8_t b = 0;
        if ((int16_t)(phi[1] - phi[0]) > center_dphi) b |= 0x80;
        if ((int16_t)(phi[3] - phi[2]) > center_dphi) b |= 0x40;
        if ((int16_t)(phi[5] - phi[4]) > center_dphi) b |= 0x20;
        if ((int16_t)(phi[7] - phi[6]) > center_dphi) b |= 0x10;
        if ((int16_t)(phi[9] - phi[8]) > center_dphi) b |= 0x08;
        if ((int16_t)(phi[11] - phi[10]) > center_dphi) b |= 0x04;
        if ((int16_t)(phi[13] - phi[12]) > center_dphi) b |= 0x02;
        if ((int16_t)(phi[15] - phi[14]) > center_dphi) b |= 0x01;
        *frame++ = b;
        phi += 16;
    }
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

int decode_adsb_frame(uint64_t timestamp, uint16_t *phi)
{
    uint8_t framedata[LONG_FRAME_BYTES];
    uint8_t short_framedata[SHORT_FRAME_BYTES];
    int16_t center_dphi;
    struct uat_adsb_mdb mdb;
    int n_corrected;

    if (!check_sync_word(phi, ADSB_SYNC_WORD, &center_dphi))
        return 0;

    demod_frame(phi + SYNC_BITS*2, framedata, LONG_FRAME_BYTES, center_dphi);

    // Try decoding as a Long UAT. Copy the data first in case we have to fall back
    // to Basic UAT.
    memcpy(short_framedata, framedata, SHORT_FRAME_BYTES);
    n_corrected = decode_rs_char(rs_adsb_long, framedata, NULL, 0);
    if (n_corrected >= 0 && n_corrected <= 7 && (framedata[0]>>3) != 0) {
        dump_raw_message('-', framedata, LONG_FRAME_DATA_BYTES, n_corrected);
        if (!raw_mode) {
            uat_decode_adsb_mdb(framedata, &mdb);
            
            fprintf(stdout,
                    "%.6f   Long UAT MDB received\n"
                    "=============================================\n",
                    timestamp / 2083334.0);
            uat_display_adsb_mdb(&mdb, stdout);
            fprintf(stdout, "=============================================\n\n");
        }

        fflush(stdout);
        return (SYNC_BITS+LONG_FRAME_BITS);
    }

    // Retry as Basic UAT
    n_corrected = decode_rs_char(rs_adsb_short, short_framedata, NULL, 0);
    if (n_corrected >= 0 && n_corrected <= 6 && (short_framedata[0]>>3) == 0) {
        dump_raw_message('-', short_framedata, SHORT_FRAME_DATA_BYTES, n_corrected);
        if (!raw_mode) {
            uat_decode_adsb_mdb(short_framedata, &mdb);
            
            fprintf(stdout,
                    "%.6f   Basic UAT MDB received\n"
                    "=============================================\n",
                    timestamp / 2083334.0);
            uat_display_adsb_mdb(&mdb, stdout);
            fprintf(stdout, "=============================================\n\n");
        }

        fflush(stdout);
        return (SYNC_BITS+SHORT_FRAME_BITS);
    }

    // Nope.
    return 0;
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

int decode_uplink_frame(uint64_t timestamp, uint16_t *phi)
{
    int i, block;
    uint8_t interleaved[UPLINK_FRAME_BYTES];
    int16_t center_dphi;
    uint8_t deinterleaved[UPLINK_FRAME_DATA_BYTES];
    int total_corrected = 0;

    if (!check_sync_word(phi, UPLINK_SYNC_WORD, &center_dphi))
        return 0;

    demod_frame(phi + SYNC_BITS*2, interleaved, UPLINK_FRAME_BYTES, center_dphi);

    // deinterleave blocks
    for (block = 0; block < UPLINK_FRAME_BLOCKS; ++block) {
        uint8_t blockdata[UPLINK_BLOCK_BYTES];
        int n_corrected;
        
        for (i = 0; i < UPLINK_BLOCK_BYTES; ++i)
            blockdata[i] = interleaved[i * UPLINK_FRAME_BLOCKS + block];

        n_corrected = decode_rs_char(rs_uplink, blockdata, NULL, 0);
        if (n_corrected < 0 || n_corrected > 10) {
            return 0;
        }

        memcpy (deinterleaved + UPLINK_BLOCK_DATA_BYTES*block, blockdata, UPLINK_BLOCK_DATA_BYTES); // drop the trailing ECC part
        total_corrected += n_corrected;
    }

    dump_raw_message('+', deinterleaved, UPLINK_FRAME_DATA_BYTES, total_corrected);
    if (!raw_mode) {
        fprintf(stdout,
                "%.6f   Uplink MDB received\n"
                "=============================================\n",
                timestamp / 2083334.0 / 2);
        //decode_uplink_mdb(deinterleaved);
        fprintf(stdout, "=============================================\n\n");
    }

    fflush(stdout);
    return (UPLINK_FRAME_BITS+SYNC_BITS);
}



