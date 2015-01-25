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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include <unistd.h>
#include <time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>

#include "uat.h"
#include "uat_decode.h"

#define NON_ICAO_ADDRESS 0x1000000U

struct aircraft {
    struct aircraft *next;
    uint32_t address;

    uint32_t messages;
    time_t last_seen;
    time_t last_seen_pos;

    int position_valid : 1;
    int altitude_valid : 1;
    int track_valid : 1;
    int speed_valid : 1;
    int vert_rate_valid : 1;

    airground_state_t airground_state;
    char callsign[9];
    char squawk[9];

    // if position_valid:
    double lat;
    double lon;

    // if altitude_valid:
    int32_t altitude; // in feet
    
    // if track_valid:
    uint16_t track;

    // if speed_valid:
    uint16_t speed; // in kts

    // if vert_rate_valid:
    int16_t vert_rate; // in ft/min
};        

static struct aircraft *aircraft_list;
static time_t NOW;
static const char *json_dir;

static struct aircraft *find_aircraft(uint32_t address)
{
    struct aircraft *a;
    for (a = aircraft_list; a; a = a->next)
        if (a->address == address)
            return a;
    return NULL;
}

static struct aircraft *find_or_create_aircraft(uint32_t address)
{
    struct aircraft *a = find_aircraft(address);
    if (a)
        return a;

    a = calloc(1, sizeof(*a));
    a->address = address;
    a->airground_state = AIRGROUND_RESERVED;

    a->next = aircraft_list;
    aircraft_list = a;

    return a;
}

static void expire_old_aircraft()
{
    struct aircraft *a, **last;    
    for (last = &aircraft_list, a = *last; a; a = *last) {
        if ((NOW - a->last_seen) > 300) {
            *last = a->next;
            free(a);
        } else {
            last = &a->next;
        }
    }
}

static uint32_t message_count;

static void process_mdb(struct uat_adsb_mdb *mdb)
{
    struct aircraft *a;
    uint32_t addr;
    
    ++message_count;

    switch (mdb->hdr.address_qualifier) {
    case ADSB_ICAO:
    case TISB_ICAO:        
        addr = mdb->hdr.address;
        break;

    default:
        addr = mdb->hdr.address | NON_ICAO_ADDRESS;
        break;
    }
   
    a = find_or_create_aircraft(addr);
    a->last_seen = NOW;
    ++a->messages;
    
    // copy state into aircraft
    if (mdb->sv_valid) {
        a->airground_state = mdb->sv.airground_state;

        if (mdb->sv.position_valid) {
            a->position_valid = 1;
            a->lat = mdb->sv.lat;
            a->lon = mdb->sv.lon;
            a->last_seen_pos = NOW;
        }
        
        if (mdb->sv.altitude_valid) {
            a->altitude_valid = 1;
            a->altitude = mdb->sv.altitude;
        }

        if (mdb->sv.track_valid) {
            a->track_valid = 1;
            a->track = mdb->sv.track;
        }

        if (mdb->sv.speed_valid) {
            a->speed_valid = 1;
            a->speed = mdb->sv.speed;
        }

        if (mdb->sv.vert_rate_valid) {
            a->vert_rate_valid = 1;
            a->vert_rate = mdb->sv.vert_rate;
        }
    }
    
    if (mdb->ms_valid) {
        if (mdb->ms.callsign[0]) {
            if (mdb->ms.callsign_id)
                strcpy(a->callsign, mdb->ms.callsign);
            else
                strcpy(a->squawk, mdb->ms.callsign);
        }
    }

    if (mdb->auxsv_valid) {
        if (mdb->auxsv.sec_altitude_valid) {
            // only use secondary if no primary is available
            if ((!mdb->sv_valid && !a->altitude_valid) ||
                (mdb->sv_valid && !mdb->sv.altitude_valid)) {
                a->altitude_valid = 1;
                a->altitude = mdb->auxsv.sec_altitude;
            }
        }
    }
}

static int write_receiver_json(const char *dir)
{
    char path[PATH_MAX];
    FILE *f;

    if (snprintf(path, PATH_MAX, "%s/receiver.json", dir) >= PATH_MAX)
        return 0;

    if (!(f = fopen(path, "w"))) {
        perror("fopen(receiver.json)");
        return 0;
    }

    fprintf(f,
            "{\n"
            "  \"version\" : \"dump978-uat2json\",\n"
            "  \"refresh\" : 1000,\n"
            "  \"history\" : 0\n"
            "}\n");
    fclose(f);
    return 1;
}

static int write_aircraft_json(const char *dir)
{
    char path[PATH_MAX];
    FILE *f;
    struct aircraft *a;

    if (snprintf(path, PATH_MAX, "%s/aircraft.json", dir) >= PATH_MAX)
        return 0;

    if (!(f = fopen(path, "w"))) {
        perror("fopen(aircraft.json)");
        return 0;
    }

    fprintf(f,
            "{\n"
            "  \"now\" : %u,\n"
            "  \"messages\" : %u,\n"
            "  \"aircraft\" : [\n",
            (unsigned)NOW,
            message_count);
    

    for (a = aircraft_list; a; a = a->next) {
        if (a != aircraft_list)
            fprintf(f, ",\n");
        fprintf(f,
                "    {\"hex\":\"%s%06x\"",
                (a->address & NON_ICAO_ADDRESS) ? "~" : "",
                a->address & 0xFFFFFF);

        if (a->squawk[0])
            fprintf(f, ",\"squawk\":\"%s\"", a->squawk);
        if (a->callsign[0])
            fprintf(f, ",\"flight\":\"%s\"", a->callsign);
        if (a->position_valid)
            fprintf(f, ",\"lat\":%.6f,\"lon\":%.6f,\"seen_pos\":%u", a->lat, a->lon, (unsigned) (NOW - a->last_seen_pos));        
        if (a->altitude_valid)
            fprintf(f, ",\"altitude\":%d", a->altitude);
        if (a->vert_rate_valid)
            fprintf(f, ",\"vert_rate\":%d", a->vert_rate);
        if (a->track_valid)
            fprintf(f, ",\"track\":%u", a->track);
        if (a->speed_valid)
            fprintf(f, ",\"speed\":%u", a->speed);
        fprintf(f, ",\"messages\":%u,\"seen\":%u,\"rssi\":0}",
                a->messages, (unsigned) (NOW - a->last_seen));
    }

    fprintf(f,
            "\n  ]\n"
            "}\n");
    fclose(f);
    return 1;
}
    
static void periodic_work()
{
    static time_t next_write;
    if (NOW >= next_write) {
        expire_old_aircraft();
        write_aircraft_json(json_dir);
        next_write = NOW + 1;
    }
}

#define SHORT_FRAME_SIZE (144/8)
#define LONG_FRAME_SIZE (272/8)

static void handle_frame(uint8_t *frame, int len)
{
    struct uat_adsb_mdb mdb;

    if (len != SHORT_FRAME_SIZE && len != LONG_FRAME_SIZE) {
        fprintf(stderr, "odd frame size: %d\n", len);
        return;
    }

    if (len == SHORT_FRAME_SIZE && (frame[0]>>3) != 0) {
        fprintf(stderr, "short frame with non-zero type\n");
        return;
    }

    if (len == LONG_FRAME_SIZE && (frame[0]>>3) == 0) {
        fprintf(stderr, "long frame with zero type\n");
        return;
    }

    uat_decode_adsb_mdb(frame, &mdb);
    uat_display_adsb_mdb(&mdb, stdout);    
    process_mdb(&mdb);
}                                                        

static int hexbyte(char *buf)
{
    int i;
    char c;

    c = buf[0];
    if (c >= '0' && c <= '9')
        i = (c - '0');
    else if (c >= 'a' && c <= 'f')
        i = (c - 'a' + 10);
    else if (c >= 'A' && c <= 'F')
        i = (c - 'A' + 10);
    else
        return -1;

    i <<= 4;
    c = buf[1];
    if (c >= '0' && c <= '9')
        return i | (c - '0');
    else if (c >= 'a' && c <= 'f')
        return i | (c - 'a' + 10);
    else if (c >= 'A' && c <= 'F')
        return i | (c - 'A' + 10);
    else
        return -1;
}

static int process_input(char *buf, int len)
{
    char *p = buf;

    for (;;) {
        char *newline;

        if (p >= buf+len)
            return len;

        newline = memchr(p, '\n', buf+len-p);
        if (newline == NULL)
            return p-buf;
        
        if (*p == '-') {           
            uint8_t frame[LONG_FRAME_SIZE];
            uint8_t *out = frame;

            ++p;
            while (p < newline) {
                int byte;
                
                if (p[0] == ';' || p[0] == '\r') {
                    handle_frame(frame, out-frame);
                    break;
                }

                if (out >= frame+sizeof(frame)) {
                    fprintf(stderr, "downlink message is too long\n");
                    break;
                }
                
                byte = hexbyte(p);
                if (byte < 0) {
                    fprintf(stderr, "bad hexbyte in downlink message: %c%c\n", p[0], p[1]);
                    break;
                }
                
                *out++ = byte;
                p += 2;
            }
        }
            
        p = newline+1;
    }
}

static void read_loop()
{
    char buf[4096];
    int used = 0;

    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

    for (;;) {
        fd_set readset, writeset, excset;
        struct timeval timeout;
        ssize_t bytes_read;
        int consumed;

        FD_ZERO(&readset);
        FD_ZERO(&writeset);
        FD_ZERO(&excset);
        FD_SET(0, &readset);
        FD_SET(0, &excset);
        timeout.tv_sec = 0;
        timeout.tv_usec = 500000;

        select(1, &readset, &writeset, &excset, &timeout);
        bytes_read = read(0, buf+used, sizeof(buf)-used);
        if (bytes_read == 0)
            return;

        if (bytes_read < 0 && errno != EAGAIN && errno != EINTR && errno != EWOULDBLOCK) {
            perror("read");
            return;
        }

        NOW = time(NULL);

        if (bytes_read > 0) {
            used += bytes_read;
            consumed = process_input(buf, used);

            if (used == sizeof(buf) && consumed == 0) {
                fprintf(stderr, "line too long, ditching input\n");
                used = 0;
            } else {
                used -= consumed;
                if (used > 0)
                    memmove(buf, buf+consumed, used);
            }
        }

        periodic_work();
    }
}                    

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
                "Syntax: %s <dir>\n"
                "\n"
                "Reads UAT messages on stdin.\n"
                "Periodically writes aircraft state to <dir>/aircraft.json\n"
                "Also writes <dir>/receiver.json once on startup\n",
                argv[0]);
        return 1;
    }

    json_dir = argv[1];

    if (!write_receiver_json(json_dir)) {
        fprintf(stderr, "Failed to write receiver.json - check permissions?\n");
        return 1;
    }
    read_loop();
    write_aircraft_json(json_dir);
    return 0;
}
