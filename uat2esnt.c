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
#include <math.h>

#include "uat.h"
#include "uat_decode.h"
#include "reader.h"

static void checksum_and_send(uint8_t *esnt_frame);

static int encode_altitude(int valid, int ft)
{
    int i;

    if (!valid)
        return 0;

    i = (ft + 1000) / 25;
    if (i < 0) i = 0;
    if (i > 0x7FF) i = 0x7FF;

    return (i & 0x000F) | 0x0010 | ((i & 0x07F0) << 1);
}

static int encode_ground_movement(int valid, int speed_kt)
{
    if (!valid)
        return 0;
    if (speed_kt > 175)
        return 124;
    if (speed_kt > 100)
        return (speed_kt - 100) / 5 + 108;
    if (speed_kt > 70)
        return (speed_kt - 70) / 2 + 93;
    if (speed_kt > 15)
        return (speed_kt - 15) + 38;
    if (speed_kt > 2)
        return (speed_kt - 2) * 2 + 11;
    if (speed_kt == 2)
        return 12;
    if (speed_kt == 1)
        return 8;
    return 1;
}

static int encode_ground_track(int valid, int track)
{
    if (!valid)
        return 0;
    return 0x80 | (track * 128 / 360);
}

static int encode_air_velocity(int valid, int vel, int supersonic)
{
    int sign;

    if (!valid)
        return 0;

    if (vel < 0) {
        sign = 0x0400;
        vel = -vel;
    } else {
        sign = 0;
    }
    
    if (supersonic)
        vel = vel / 4;
    
    ++vel;
    if (vel > 1023)
        vel = 1023;

    return vel | sign;
}

static int encode_vert_rate(int valid, int rate)
{
    int sign;

    if (!valid)
        return 0;
    
    if (rate < 0) {
        sign = 0x200;
        rate = -rate;
    } else {
        sign = 0;
    }

    rate = (rate / 64) + 1;
    if (rate >= 511)
        rate = 511;

    return rate | sign;
}

static double cprMod(double a, double b) {
    double res = fmod(a, b);
    if (res < 0) res += b;
    return res;
}

static int cprNL(double lat)
{
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}

static int cprN(double lat, int odd)
{
    int nl = cprNL(lat) - (odd ? 1 : 0);
    if (nl < 1)
        nl = 1;
    return nl;
}

static int encode_cpr_lat(double lat, double lon, int odd, int surface)
{
    int NbPow = (surface ? 1<<19 : 1<<17);

    double Dlat = 360.0 / (odd ? 59 : 60);
    int YZ = floor(NbPow * cprMod(lat, Dlat) / Dlat + 0.5);

    return YZ & 0x1FFFF; // always a 17-bit field
}

static int encode_cpr_lon(double lat, double lon, int odd, int surface)
{
    int NbPow = (surface ? 1<<19 : 1<<17);

    double Dlat = 360.0 / (odd ? 59 : 60);
    int YZ = floor(NbPow * cprMod(lat, Dlat) / Dlat + 0.5);

    double Rlat = Dlat * (1.0 * YZ / NbPow + floor(lat / Dlat));
    double Dlon = (360.0 / cprN(Rlat, odd));
    int XZ = floor(NbPow * cprMod(lon, Dlon) / Dlon + 0.5);

    return XZ & 0x1FFFF; // always a 17-bit field
}

static void generate_esnt_ground_position(int metype,
                                          uint32_t address,                                          
                                          int raw_lat,
                                          int raw_lon,
                                          int raw_movement,
                                          int raw_track,
                                          int odd)
{
    uint8_t esnt_frame[14];

    esnt_frame[0] = (18 << 3) | (5);   // DF 18, TIS-B relay of ADS-B message with other address
    esnt_frame[1] = (address & 0xFF0000) >> 16;
    esnt_frame[2] = (address & 0x00FF00) >> 8;
    esnt_frame[3] = (address & 0x0000FF);
    esnt_frame[4] = (metype << 3) | ((raw_movement & 0x70) >> 4);   // ES type, ground movement bits
    esnt_frame[5] = ((raw_movement & 0x0F) << 4) | ((raw_track & 0xF0) >> 4);  // ground movement, track
    esnt_frame[6] = ((raw_track & 0x0F) << 4) | (odd ? 0x04 : 0x00) | ((raw_lat >> 15) & 0x03); // track, time, odd, lat
    esnt_frame[7] = ((raw_lat >> 7) & 0xFF); // lat
    esnt_frame[8] = ((raw_lat << 1) & 0xFE) | ((raw_lon >> 16) & 0x01); // lat, lon
    esnt_frame[9] = ((raw_lon >> 8) & 0xFF); // lon
    esnt_frame[10] = (raw_lon & 0xFF); // lon
    esnt_frame[11] = 0; // crc
    esnt_frame[12] = 0; // crc
    esnt_frame[13] = 0; // crc
    checksum_and_send(esnt_frame);
}

static void generate_esnt_air_position(int metype,
                                       uint32_t address,                                          
                                       int raw_alt,
                                       int raw_lat,
                                       int raw_lon,
                                       int odd)
{
    uint8_t esnt_frame[14];

    esnt_frame[0] = (18 << 3) | (5);   // DF 18, TIS-B relay of ADS-B message with other address
    esnt_frame[1] = (address & 0xFF0000) >> 16;
    esnt_frame[2] = (address & 0x00FF00) >> 8;
    esnt_frame[3] = (address & 0x0000FF);
    esnt_frame[4] = (metype << 3); // metype, SAF=0, FS=normal/airborne
    esnt_frame[5] = ((raw_alt >> 4) & 0xFF);    // altitude
    esnt_frame[6] = ((raw_alt << 4) & 0xF0) | (odd ? 0x04 : 0x00) | ((raw_lat >> 15) & 0x03); // alt, time, odd, lat
    esnt_frame[7] = ((raw_lat >> 7) & 0xFF); // lat
    esnt_frame[8] = ((raw_lat << 1) & 0xFE) | ((raw_lon >> 16) & 0x01); // lat, lon
    esnt_frame[9] = ((raw_lon >> 8) & 0xFF); // lon
    esnt_frame[10] = (raw_lon & 0xFF); // lon
    esnt_frame[11] = 0; // crc
    esnt_frame[12] = 0; // crc
    esnt_frame[13] = 0; // crc
    checksum_and_send(esnt_frame);
}

static void generate_esnt_air_velocity(int metype,
                                       int mesub,
                                       uint32_t address,
                                       int raw_ns_vel,
                                       int raw_ew_vel,
                                       int raw_vert_rate)
{
    uint8_t esnt_frame[14];

    esnt_frame[0] = (18 << 3) | (5);   // DF 18, TIS-B relay of ADS-B message with other address
    esnt_frame[1] = (address & 0xFF0000) >> 16;
    esnt_frame[2] = (address & 0x00FF00) >> 8;
    esnt_frame[3] = (address & 0x0000FF);
    esnt_frame[4] = (metype << 3) | mesub; // metype, mesub
    esnt_frame[5] = ((raw_ew_vel >> 8) & 0x07); // intent change = 0, IFR = 0, NUCr = 0, E/W vel
    esnt_frame[6] = (raw_ew_vel & 0xFF); // E/W vel
    esnt_frame[7] = ((raw_ns_vel >> 3) & 0xFF); // N/S vel
    esnt_frame[8] = ((raw_ns_vel << 5) & 0xE0) | ((raw_vert_rate >> 6) & 0x1F); // N/S vel, VR source, VR
    esnt_frame[9] = ((raw_vert_rate << 2) & 0xFC);  // VR
    esnt_frame[10] = 0; // gnss alt difference (TODO)
    esnt_frame[11] = 0; // crc
    esnt_frame[12] = 0; // crc
    esnt_frame[13] = 0; // crc
    checksum_and_send(esnt_frame);
}

// Generator polynomial for the Mode S CRC:
#define MODES_GENERATOR_POLY 0xfff409U

// CRC values for all single-byte messages;
// used to speed up CRC calculation.
static uint32_t crc_table[256];

static void initCrcTables()
{
    int i;
    for (i = 0; i < 256; ++i) {
        uint32_t c = i << 16;
        int j;
        for (j = 0; j < 8; ++j) {
            if (c & 0x800000)
                c = (c<<1) ^ MODES_GENERATOR_POLY;
            else
                c = (c<<1);
        }

        crc_table[i] = c & 0x00ffffff;
    }
}

static uint32_t checksum(uint8_t *message, int n)
{
    uint32_t rem = 0;
    int i;

    for (i = 0; i < n; ++i) {
        rem = (rem << 8) ^ crc_table[message[i] ^ ((rem & 0xff0000) >> 16)];
        rem = rem & 0xffffff;
    }

    return rem;
}

static void checksum_and_send(uint8_t *esnt_frame)
{
    int j;
    uint32_t rem = checksum(esnt_frame, 11);

    esnt_frame[11] ^= (rem & 0xFF0000) >> 16;
    esnt_frame[12] ^= (rem & 0x00FF00) >> 8;
    esnt_frame[13] ^= (rem & 0x0000FF);

    fprintf(stdout, "*");
    for (j = 0; j < 14; j++)
        fprintf(stdout, "%02X", esnt_frame[j]);
    fprintf(stdout, ";\n");
    fflush(stdout);
}

static void generate_esnt(struct uat_adsb_mdb *mdb)
{
    if (!mdb->sv_valid)
        return; // nothing useful.

    // Process SV:
    
    if (!mdb->sv.position_valid) {
        generate_esnt_air_position(0,
                                   mdb->hdr.address,
                                   encode_altitude(mdb->sv.altitude_valid, mdb->sv.altitude),
                                   0, 0, 0);
    } else if (mdb->sv.airground_state == AG_GROUND) {
        generate_esnt_ground_position(8,
                                      mdb->hdr.address,
                                      encode_cpr_lat(mdb->sv.lat, mdb->sv.lon, 0, 1),
                                      encode_cpr_lon(mdb->sv.lat, mdb->sv.lon, 0, 1),
                                      encode_ground_movement(mdb->sv.speed_valid, mdb->sv.speed),
                                      encode_ground_track(mdb->sv.track_valid, mdb->sv.track),
                                      0);

        generate_esnt_ground_position(8,
                                      mdb->hdr.address,
                                      encode_cpr_lat(mdb->sv.lat, mdb->sv.lon, 1, 1),
                                      encode_cpr_lon(mdb->sv.lat, mdb->sv.lon, 1, 1),
                                      encode_ground_movement(mdb->sv.speed_valid, mdb->sv.speed),
                                      encode_ground_track(mdb->sv.track_valid, mdb->sv.track),
                                      1);
    } else {
        generate_esnt_air_position(18,
                                   mdb->hdr.address,
                                   encode_altitude(mdb->sv.altitude_valid, mdb->sv.altitude),
                                   encode_cpr_lat(mdb->sv.lat, mdb->sv.lon, 0, 0),
                                   encode_cpr_lon(mdb->sv.lat, mdb->sv.lon, 0, 0),
                                   0);
        generate_esnt_air_position(18,
                                   mdb->hdr.address,
                                   encode_altitude(mdb->sv.altitude_valid, mdb->sv.altitude),
                                   encode_cpr_lat(mdb->sv.lat, mdb->sv.lon, 1, 0),
                                   encode_cpr_lon(mdb->sv.lat, mdb->sv.lon, 1, 0),
                                   1);

        if (mdb->sv.ns_vel_valid || mdb->sv.ew_vel_valid || mdb->sv.vert_rate_valid) {
            int supersonic = (mdb->sv.airground_state == AG_SUPERSONIC);
            generate_esnt_air_velocity(19,
                                       supersonic ? 2 : 1,
                                       mdb->hdr.address,
                                       encode_air_velocity(mdb->sv.ns_vel_valid, mdb->sv.ns_vel, supersonic),
                                       encode_air_velocity(mdb->sv.ew_vel_valid, mdb->sv.ew_vel, supersonic),
                                       encode_vert_rate(mdb->sv.vert_rate_valid, mdb->sv.vert_rate));
        }
    }                                       

    // XXX process MS for callsign and squawk

}

static void handle_frame(frame_type_t type, uint8_t *frame, int len, void *extra)
{
    if (type == UAT_DOWNLINK) {
        struct uat_adsb_mdb mdb;
        uat_decode_adsb_mdb(frame, &mdb);
        generate_esnt(&mdb);
    }
}        

int main(int argc, char **argv)
{
    struct dump978_reader *reader;
    int framecount;

    initCrcTables();

    reader = dump978_reader_new(0,0);
    if (!reader) {
        perror("dump978_reader_new");
        return 1;
    }
    
    while ((framecount = dump978_read_frames(reader, handle_frame, NULL)) > 0)
        ;

    if (framecount < 0) {
        perror("dump978_read_frames");
        return 1;
    }

    return 0;
}

