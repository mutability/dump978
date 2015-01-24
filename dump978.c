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

void make_atan2_table();
void read_from_stdin();
int process_buffer(uint8_t *input, int len, uint64_t offset);
int decode_adsb_frame(uint8_t *input);
int decode_uplink_frame(uint8_t *input);

int main(int argc, char **argv)
{
    make_atan2_table();
    read_from_stdin();
    return 0;
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
    for (i = 2; i+(SYNC_LENGTH+UPLINK_FRAME_LENGTH+1)*4 < len; i += 4) {
        uint16_t phi0 = iqphase(input[i+0], input[i+1]);
        uint16_t phi1 = iqphase(input[i+2], input[i+3]);
        int16_t dphi0 = phi0 - last_phi;  // don't need to worry about wrapping at +/-pi, the width of the datatype does it for us
        int16_t dphi1 = phi1 - phi0;

#if 0
        {
            // express dphi as the effective carrier deviation in Hz
            // dphi = 32768 implies 0.5 cycles/sample = 1041667Hz

            double deviation_0 = (2083334.0 * dphi0 / 65536);
            double deviation_1 = (2083334.0 * dphi1 / 65536);
            fprintf(stdout,
                    "%4d  %02x %02x %02x %02x   %+8.0f %1d    %+8.0f %1d\n",
                    i, input[i+0], input[i+1], input[i+2], input[i+3], 
                    deviation_0, (dphi0 < 0) ? 0 : 1,
                    deviation_1, (dphi1 < 0) ? 0 : 1);
        }
#endif

        last_phi = phi1;

        // accumulate sync words
        sync0 = ((sync0 << 1) | (dphi0 < 0 ? 0 : 1));
        sync1 = ((sync1 << 1) | (dphi1 < 0 ? 0 : 1));

        //fprintf(stdout, "%09lx %09lx\n", sync0 & SYNC_MASK, sync1 & SYNC_MASK);


        // see if we have a valid sync word
        if ((sync0 & SYNC_MASK) == ADSB_SYNC_WORD) {
            if ((sync1 & SYNC_MASK) == ADSB_SYNC_WORD) {
                // Both offset 0 and 1 are candidates, TODO: find the one with better correlation and use that
                fprintf(stdout, "%9.6f adsb   0+1  ", (offset+i) / 2 / 2083334.0);
            } else {           
                fprintf(stdout, "%9.6f adsb   0    ", (offset+i) / 2 / 2083334.0);
            }
            i += decode_adsb_frame(input+i+2);
        } else if ((sync1 & SYNC_MASK) == ADSB_SYNC_WORD) {
            if ((sync0 & SYNC_MASK)>>1 == ADSB_SYNC_WORD>>1) {
                // Both offset 1 and 2 are candidates, TODO: find the one with better correlation and use that
                fprintf(stdout, "%9.6f adsb   1+2  ", (offset+i) / 2 / 2083334.0);
            } else {
                fprintf(stdout, "%9.6f adsb   1    ", (offset+i) / 2 / 2083334.0);
            }

            i += decode_adsb_frame(input+i+4);
        } else if ((sync0 & SYNC_MASK) == UPLINK_SYNC_WORD) {
            if ((sync1 & SYNC_MASK) == UPLINK_SYNC_WORD) {
                // Both offset 0 and 1 are candidates, TODO: find the one with better correlation and use that
                fprintf(stdout, "%9.6f uplink 0+1  ", (offset+i) / 2 / 2083334.0);
            } else {           
                fprintf(stdout, "%9.6f uplink 0    ", (offset+i) / 2 / 2083334.0);
            }
            i += decode_uplink_frame(input+i+2);
        } else if ((sync1 & SYNC_MASK) == UPLINK_SYNC_WORD) {
            if ((sync0 & SYNC_MASK)>>1 == UPLINK_SYNC_WORD>>1) {
                // Both offset 1 and 2 are candidates, TODO: find the one with better correlation and use that
                fprintf(stdout, "%9.6f uplink 1+2  ", (offset+i) / 2 / 2083334.0);
            } else {
                fprintf(stdout, "%9.6f uplink 1    ", (offset+i) / 2 / 2083334.0);
            }

            i += decode_uplink_frame(input+i+4);
        }
    }

    //fprintf(stdout, "done %d\n", i);
    return i;
}

int find_average_dphi(uint8_t *input, uint64_t pattern, int16_t *center)
{
    int i;

    int32_t dphi_zero_total = 0;
    int zero_bits = 0;
    int32_t dphi_one_total = 0;
    int one_bits = 0;

    int32_t separation;

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

    separation = dphi_one_total - dphi_zero_total;
    *center = (dphi_zero_total + dphi_one_total) / 2;
    fprintf(stdout, "zeroes %.0fkHz ones %.0fkHz separation %.0fkHz center %.0fkHz\n",
            dphi_zero_total * 2083334.0 / 65536 / 1000,
            dphi_one_total * 2083334.0 / 65536 / 1000,
            separation * 2083334.0 / 65536 / 1000,
            *center * 2083334.0 / 65536 / 1000);

    // sanity check - 20kHz to 1MHz separation
    if (separation < (65536 * (20e3 / 2083334.0)) || separation > (65536 * (1000e3 / 2083334.0)))
        return 0;
    else
        return 1;
}

const char *address_qualifiers[8] = {
    "ICAO, from aircraft",
    "Reserved (1)",
    "ICAO, rebroadcast by TIS-B",
    "Non-ICAO TIS-B track file",
    "Vehicle",
    "Fixed Beacon",
    "Reserved (6)",
    "Reserved (7)"
};

const char *nic_types[16] = {
    "unknown integrity, >20NM",
    "RNP-10/RNP-5, 8-20NM",
    "RNP-4, 4-8NM",
    "RNP-2, 2-4NM",
    "RNP-1, 1-2NM",
    "RNP-0.5, 0.6-1NM",
    "RNP-0.3, 0.2-0.6NM",
    "RNP-0.1, 0.1-0.2NM",
    "RNP-0.05, <0.1NM",
    "Rc 25-75m, VPL 37.5-112m",
    "Rc 7.5-25m, VPL 11-37.5m",
    "Rc <7.5m, VPL <11m",
    "reserved (12)",
    "reserved (13)",
    "reserved (14)",
    "reserved (15)"
};

const char *airground_state[4] = {
    "Airborne, subsonic",
    "Airborne, supersonic",
    "On ground",
    "Reserved (3)"
};

void decode_header(uint8_t *framedata)
{
    fprintf(stdout,
            " === HEADER ===\n"
            "  MDB type code:     %d\n"
            "  Address qualifier: %d (%s)\n"
            "  Address:           %06x\n",
            framedata[0]>>3,
            framedata[0] & 7,
            address_qualifiers[framedata[0] & 7],
            (framedata[1] << 16) | (framedata[2] << 8) | framedata[3]);
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

void decode_state_vector_common(uint8_t *framedata)
{
    int lat, lng, alt, nic, ag, hvel, vvel;

    fprintf(stdout,
            " === STATE VECTOR ===\n");

    lat = (framedata[4] << 15) | (framedata[5] << 7) | (framedata[6] >> 1);
    lng = ((framedata[6] & 1) << 23) | (framedata[7] << 15) | (framedata[8] << 7) | (framedata[9] >> 1);            
    alt = (framedata[10] << 4) | ((framedata[11] & 0xf0) >> 4);
    nic = (framedata[11] & 15);
            
    if (lat == 0 && lng == 0 && nic == 0) {
        fprintf(stdout,
                "  Latitude:          unavailable\n"
                "  Longitude:         unavailable\n");
    } else {
        double wgs_lat = 0, wgs_lng = 0;
        decode_latlng(lat, lng, &wgs_lat, &wgs_lng);
        fprintf(stdout,
                "  Latitude:          %+3.1f (%d)\n"
                "  Longitude:         %+3.1f (%d)\n",
                wgs_lat, lat,
                wgs_lng, lng);
    }
    
    if (alt == 0)
        fprintf(stdout,
                "  Altitude:          unavailable\n");
    else
        fprintf(stdout,
                "  Altitude:          %d ft %s (%d)\n",
                alt * 25 - 1025,
                (framedata[9] & 1) ? "geometric" : "barometric",
                alt);
    
    fprintf(stdout,
            "  NIC:               %s (%d)\n",
            nic_types[nic], nic);
    
    
    ag = framedata[12] >> 6;
    fprintf(stdout,
            "  Air/ground state:  %s (%d)\n",
            airground_state[ag], ag);

    hvel = ((framedata[12] & 0x1f) << 17) | (framedata[13] << 9) | (framedata[14] << 1) | ((framedata[15] & 0x80) >> 7);
    vvel = ((framedata[15] & 0x7f) << 4) | ((framedata[16] & 0xf0) >> 4);

    if (ag & 2) {
        int raw_gs = (hvel >> 11) & 0x7ff;
        int raw_track = hvel & 0x7ff;
        double len, wid;

        if (raw_gs == 0) {
            fprintf(stdout,
                    "  Ground speed:      unavailable (%d)\n",
                    raw_gs);
        } else {
            int gs = ((raw_gs & 0x3ff) - 1);
            fprintf(stdout,
                    "  Ground speed:      %d kts (%d)\n",
                    gs, raw_gs);
        } 

        switch ((raw_track & 0x600) >> 9) {
        case 0:
            fprintf(stdout,
                    "  Track/heading:    unavailable (%d)\n",
                    raw_track);
            break;
        case 1:
            fprintf(stdout,
                    "  True track angle: %d (%d)\n",
                    (raw_track & 0x1ff) * 360 / 512,
                    raw_track);
            break;
        case 2:
            fprintf(stdout,
                    "  Magnetic heading: %d (%d)\n",
                    (raw_track & 0x1ff) * 360 / 512,
                    raw_track);
            break;
        case 3:
            fprintf(stdout,
                    "  True heading:     %d (%d)\n",
                    (raw_track & 0x1ff) * 360 / 512,
                    raw_track);
            break;
        }
        
        switch ((vvel >> 7) & 15) {
        case 0: len = 15; wid = 11.5; break;
        case 1: len = 15; wid = 23; break;
        case 2: len = 25; wid = 28.5; break;
        case 3: len = 25; wid = 34; break;
        case 4: len = 35; wid = 33; break;
        case 5: len = 35; wid = 38; break;
        case 6: len = 45; wid = 39.5; break;
        case 7: len = 45; wid = 45; break;
        case 8: len = 55; wid = 45; break;
        case 9: len = 55; wid = 52; break;
        case 10: len = 65; wid = 59.5; break;
        case 11: len = 65; wid = 67; break;
        case 12: len = 75; wid = 72.5; break;
        case 13: len = 75; wid = 80; break;
        case 14: len = 85; wid = 80; break;
        case 15: len = 85; wid = 90; break;
        }

        fprintf(stdout,
                "  Length/width:     %.0fm/%.1fm (%d)\n"
                "  Position offset:  %s\n",
                len, wid, vvel,
                (vvel & 0x40) ? "applied" : "not applied");
    } else {
        // airborne
        int raw_ns = (hvel >> 11) & 0x7ff;
        int raw_ew = hvel & 0x7ff;        
        int supersonic = (ag & 1);
        int ns_vel;
        int ew_vel;
        int track;
        int speed;

        if (raw_ns == 0) {
            ns_vel = 0;
            fprintf(stdout,
                    "  N/S velocity:      unavailable\n");
        } else {
            ns_vel = ((raw_ns & 0x3ff) - 1) * (supersonic ? 4 : 1) * ((raw_ns & 0x400) ? -1 : 1);
            fprintf(stdout,
                    "  N/S velocity:      %d kts (%d)\n", ns_vel, raw_ns);
        }

        if (raw_ew == 0) {
            ew_vel = 0;
            fprintf(stdout,
                    "  E/W velocity:      unavailable\n");
        } else {
            ew_vel = ((raw_ew & 0x3ff) - 1) * (supersonic ? 4 : 1) * ((raw_ew & 0x400) ? -1 : 1);
            fprintf(stdout,
                    "  E/W velocity:      %d kts (%d)\n", ew_vel, raw_ew);
        }
        
        if (ns_vel != 0 || ew_vel != 0) {
            track = (int) (90 - atan2(ns_vel, ew_vel) * 180 / M_PI);
            if (track < 0)
                track += 360;
            speed = (int) sqrt(ns_vel * ns_vel + ew_vel * ew_vel);
            fprintf(stdout,
                    "  Track:             %d\n"
                    "  Speed:             %d kts\n",
                    track, speed);
        } else {
            track = 0;
            speed = 0;
            fprintf(stdout,
                    "  Track:             indeterminate\n"
                    "  Speed:             0 kts\n");
        }

        if ((vvel & 0x1ff) == 0) {
            fprintf(stdout,
                    "  Vertical rate:     unavailable\n");
        } else {
            int vrate = ((vvel & 0x1ff) - 1) * 64 * ((vvel & 0x200) ? -1 : 1);
            if (vvel & 0x400)
                fprintf(stdout,
                        "  Vert rate (baro):  %d ft/min (%d)\n",
                        vrate, vvel);
            else
                fprintf(stdout,
                        "  Vert rate (geo):   %d ft/min (%d)\n",
                        vrate, vvel);
        }
    }        
}

void decode_state_vector(uint8_t *framedata)
{
    int addr_qual = framedata[0] & 7;
    
    if (addr_qual == 0 || addr_qual == 1 || addr_qual == 4 || addr_qual == 5) {
        // ADS-B
        decode_state_vector_common(framedata);
        fprintf(stdout,
                "  UTC coupled:       %s\n",
                (framedata[16] & 0x08) ? "yes" : "no");
    } else if (addr_qual == 2 || addr_qual == 3) {
        // TIS-B
        decode_state_vector_common(framedata);
        fprintf(stdout,
                "  TIS-B Site ID:     %d\n",
                (framedata[16] & 0x0f));
    }
}

char base40_alphabet[40] = "0123456789ABCDEFGHIJKLMNOPQRTSUVWXYZ ...";
void decode_emitter_callsign(uint8_t *framedata) {
    char buf[9];    
    uint16_t v;
    int emitter;

    v = (framedata[17]<<8) | (framedata[18]);
    emitter = (v/1600) % 40;
    buf[0] = base40_alphabet[(v/40) % 40];
    buf[1] = base40_alphabet[v % 40];
    v = (framedata[19]<<8) | (framedata[20]);
    buf[2] = base40_alphabet[(v/1600) % 40];
    buf[3] = base40_alphabet[(v/40) % 40];
    buf[4] = base40_alphabet[v % 40];
    v = (framedata[21]<<8) | (framedata[22]);
    buf[5] = base40_alphabet[(v/1600) % 40];
    buf[6] = base40_alphabet[(v/40) % 40];
    buf[7] = base40_alphabet[v % 40];
    buf[8] = 0;

    fprintf(stdout,
            "  Emitter category:  %d\n"
            "  Callsign:          %s\n",
            emitter,
            buf);
}

void decode_mode_status(uint8_t *framedata)
{
    fprintf(stdout,
            " === MODE STATUS ===\n");
    decode_emitter_callsign(framedata);

    fprintf(stdout,
            "  Emergency status:  %d\n"
            "  UAT version:       %d\n"
            "  SIL:               %d\n",
            framedata[23] >> 5,
            (framedata[23] >> 2) & 7,
            framedata[23] & 3);

    fprintf(stdout,
            "  Transmit MSO:      %d\n",
            framedata[24] >> 2);

    fprintf(stdout,
            "  NACp:              %d\n"
            "  NACv:              %d\n"
            "  NICbaro:           %d\n",
            framedata[25] >> 4,
            (framedata[25] >> 1) & 7,
            framedata[25] & 1);

    fprintf(stdout,
            "  Capability codes:  %s %s\n"
            "  Operational modes: %s %s %s\n"
            "  True/Mag:          %s\n"
            "  CSID:              %s\n",
            (framedata[26] & 0x80) ? "+CDTI" : "-cdti",
            (framedata[26] & 0x40) ? "+ACAS" : "-acas",

            (framedata[26] & 0x20) ? "+ACASRA" : "-acasra",
            (framedata[26] & 0x10) ? "+IDENT" : "-ident",
            (framedata[26] & 0x08) ? "+ATC" : "-atc",

            (framedata[26] & 0x04) ? "Magnetic" : "True",
            
            (framedata[26] & 0x02) ? "normal callsign" : "alternate callsign");        
}

void decode_aux_state_vector(uint8_t *framedata)
{
    int alt = (framedata[29] << 4) | ((framedata[30] & 0xf0) >> 4);

    fprintf(stdout,
            " === AUX STATE VECTOR ===\n");


    if (alt == 0)
        fprintf(stdout,
                "  Secondary alt:     unavailable\n");
    else
        fprintf(stdout,
                "  Secondary alt:     %d ft %s (%d)\n",
                alt * 25 - 1025,
                (framedata[9] & 1) ? "barometric" : "geometric",
                alt);
}   
                                         
int decode_adsb_frame(uint8_t *input)
{
    int i;
    uint8_t framedata[ADSB_FRAME_LENGTH/8];
    int16_t average_dphi;
    int mdb_type;

    if (!find_average_dphi(input-36*4, ADSB_SYNC_WORD, &average_dphi)) {
        fprintf(stdout, " (abandoned)\n");
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

    mdb_type = (framedata[0] >> 3);
    if (mdb_type == 0) {
        // "Basic UAT ADS-B Message": 144 data bits, 96 FEC bits
        fprintf(stdout, "  Basic UAT: ");
        for (i = 0; i < 144/8; i++)
            fprintf(stdout, "%02X", framedata[i]);

        fprintf(stdout, " | ");
        for (i = 144/8; i < (144+96)/8; i++)
            fprintf(stdout, "%02X", framedata[i]);

        fprintf(stdout, "\n");

        decode_header(framedata);
        decode_state_vector(framedata);

        return (144+96)*4;
    }
    
    // "Long UAT ADS-B Message": 272 data bits, 112 FEC bits
    fprintf(stdout, "  Long UAT:  ");
    for (i = 0; i < 272/8; i ++)
        fprintf(stdout, "%02X", framedata[i]);
    
    fprintf(stdout, " | ");
    for (i = 272/8; i < (272+112)/8; i++)
        fprintf(stdout, "%02X", framedata[i]);
    
    fprintf(stdout, "\n");

    decode_header(framedata);
    if (mdb_type <= 10)
        decode_state_vector(framedata);
    if (mdb_type == 1 || mdb_type == 2 || mdb_type == 5 || mdb_type == 6)
        decode_aux_state_vector(framedata);
    if (mdb_type == 1 || mdb_type == 3)
        decode_mode_status(framedata);
    
    return (272+112)*4;
}

void decode_fisb_apdu(uint8_t *pdu, int length)
{
    fprintf(stdout,
            "     === FIS-B APDU ===\n");
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
                "    GS Latitude:       %+3.1f (%d)\n"
                "    GS Longitude:      %+3.1f (%d)\n",
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

int decode_uplink_frame(uint8_t *input)
{
    int i, block;
    uint8_t framedata[UPLINK_FRAME_LENGTH/8];
    int16_t average_dphi;
    uint8_t deinterleaved[432];

    if (!find_average_dphi(input-36*4, UPLINK_SYNC_WORD, &average_dphi)) {
        fprintf(stdout, " (abandoned)\n");
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
        
        fprintf(stdout, "  Uplink %c:  ", ('A' + block));        
        for (i = 0; i < 72; ++i) {
            fprintf(stdout, "%02x", framedata[i * 6 + block]);
            blockdata[i] = framedata[i * 6 + block];
        } 
        fprintf(stdout, " | ");
        for (i = 72; i < 92; ++i) {
            fprintf(stdout, "%02x", framedata[i * 6 + block]);
            blockdata[i] = framedata[i * 6 + block];
        }
        fprintf(stdout, "\n");

        // XXX here we should do error correction on blockdata

        memcpy (deinterleaved + 72*block, blockdata, 72); // drop the trailing ECC part
    }
    
    decode_uplink_mdb(deinterleaved);
    return UPLINK_FRAME_LENGTH*4;
}



