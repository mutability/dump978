// Part of dump978, a UAT decoder.
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

#include <math.h>

#include "uat.h"
#include "uat_decode.h"

void uat_decode_hdr(uint8_t *frame, struct uat_hdr *hdr)
{
    hdr->mdb_type = (frame[0] >> 3) & 0x1f;
    hdr->address_qualifier = (address_qualifier_t) (frame[0] & 0x07);
    hdr->address = (frame[1] << 16) | (frame[2] << 8) | frame[3];
}

static const char *address_qualifier_names[8] = {
    "ICAO address via ADS-B",
    "reserved (national use)",
    "ICAO address via TIS-B",
    "TIS-B track file address",
    "Vehicle address",
    "Fixed ADS-B Beacon Address",
    "reserved (6)",
    "reserved (7)"
};    

void uat_display_hdr(const struct uat_hdr *hdr, FILE *to)
{
    fprintf(to,
            "HDR:\n"
            " MDB Type:          %d\n"
            " Address:           %06X (%s)\n",
            hdr->mdb_type, 
            hdr->address,
            address_qualifier_names[hdr->address_qualifier]);
}

static double dimensions_widths[16] = {
    11.5, 23, 28.5, 34, 33, 38, 39.5, 45, 45, 52, 59.5, 67, 72.5, 80, 80, 90
};

void uat_decode_sv(uint8_t *frame, struct uat_sv *sv)
{
    static struct uat_sv zero_sv;
    uint32_t raw_lat, raw_lon, raw_alt;

    *sv = zero_sv;

    sv->nic = (frame[11] & 15);

    raw_lat = (frame[4] << 15) | (frame[5] << 7) | (frame[6] >> 1);
    raw_lon = ((frame[6] & 0x01) << 23) | (frame[7] << 15) | (frame[8] << 7) | (frame[9] >> 1);
    
    if (sv->nic != 0 || raw_lat != 0 || raw_lon != 0) {
        sv->position_valid = 1;
        sv->lat = raw_lat * 360.0 / 16777216.0;
        if (sv->lat > 90)
            sv->lat -= 180;
        sv->lon = raw_lon * 360.0 / 16777216.0;
        if (sv->lon > 180)
            sv->lon -= 360;
    }

    raw_alt = (frame[10] << 4) | ((frame[11] & 0xf0) >> 4);
    if (raw_alt != 0) {
        sv->altitude_valid = 1;
        sv->altitude = (raw_alt - 1) * 25 - 1000;
        sv->altitude_type = (frame[9] & 1) ? GEO : BARO;
    }
    
    sv->airground_state = (frame[12] >> 6) & 0x03;

    switch (sv->airground_state) {
    case AIRBORNE_SUBSONIC:
    case AIRBORNE_SUPERSONIC:
        {
            int raw_ns, raw_ew, raw_vvel;
            
            raw_ns = ((frame[12] & 0x1f) << 6) | ((frame[13] & 0xfc) >> 2);        
            if ((raw_ns & 0x3ff) != 0) {
                sv->ns_vel_valid = 1;
                sv->ns_vel = ((raw_ns & 0x3ff) - 1);
                if (raw_ns & 0x400)
                    sv->ns_vel = 0 - sv->ns_vel;
                if (sv->airground_state == AIRBORNE_SUPERSONIC)
                    sv->ns_vel *= 4;
            }
            
            raw_ew = ((frame[13] & 0x03) << 9) | (frame[14] << 1) | ((frame[15] & 0x80) >> 7);
            if ((raw_ew & 0x3ff) != 0) {
                sv->ew_vel_valid = 1;
                sv->ew_vel = ((raw_ew & 0x3ff) - 1);
                if (raw_ew & 0x400)
                    sv->ew_vel = 0 - sv->ew_vel;
                if (sv->airground_state == AIRBORNE_SUPERSONIC)
                    sv->ew_vel *= 4;
            }
            
            if (sv->ns_vel_valid && sv->ew_vel_valid) {
                if (sv->ns_vel != 0 || sv->ew_vel != 0) {
                    sv->track_valid = 1;
                    sv->track_type = AIRBORNE_TRACK;
                    sv->track = (uint16_t)(360 + 90 - atan2(sv->ns_vel, sv->ew_vel) * 180 / M_PI) % 360;
                }
                
                sv->speed_valid = 1;
                sv->speed = (int) sqrt(sv->ns_vel * sv->ns_vel + sv->ew_vel * sv->ew_vel);
            }

            raw_vvel = ((frame[15] & 0x7f) << 4) | ((frame[16] & 0xf0) >> 4);
            if ((raw_vvel & 0x1ff) != 0) {
                sv->vert_rate_valid = 1;
                sv->vert_rate_source = (raw_vvel & 0x400) ? BARO : GEO;
                sv->vert_rate = ((raw_vvel & 0x1ff) - 1) * 64;
                if (raw_vvel & 0x200)
                    sv->vert_rate = 0 - sv->vert_rate;
            }                
        }
        break;

    case GROUND:
        {
            int raw_gs, raw_track;

            raw_gs = ((frame[12] & 0x1f) << 6) | ((frame[13] & 0xfc) >> 2);
            if (raw_gs != 0) {
                sv->speed_valid = 1;
                sv->speed = ((raw_gs & 0x3ff) - 1);
            }

            raw_track = ((frame[13] & 0x03) << 9) | (frame[14] << 1) | ((frame[15] & 0x80) >> 7);
            switch ((raw_track & 0x0600)>>9) {
            case 1: sv->track_valid = 1; sv->track_type = GROUND_TRACK; break;
            case 2: sv->track_valid = 1; sv->track_type = GROUND_MAG_HEADING; break;
            case 3: sv->track_valid = 1; sv->track_type = GROUND_TRUE_HEADING; break;
            }

            if (sv->track_valid)
                sv->track = (raw_track & 0x1ff) * 360 / 512;

            sv->lengthwidth_valid = 1;
            sv->length = 15 + 10 * ((frame[15] & 0x38) >> 3);
            sv->width = dimensions_widths[(frame[15] & 0x78) >> 3];
            sv->position_offset = (frame[15] & 0x04) ? 1 : 0;
        }
        break;

    case AIRGROUND_RESERVED:
        // nothing
        break;
    }
    
    if ((frame[0] & 7) == 2 || (frame[0] & 7) == 3) {
        sv->utc_coupled = 0;
        sv->tisb_site_id = (frame[16] & 0x0f);
    } else {
        sv->utc_coupled = (frame[16] & 0x08) ? 1 : 0;
        sv->tisb_site_id = 0;
    }
}

void uat_display_sv(const struct uat_sv *sv, FILE *to)
{
    fprintf(to,
            "SV:\n"
            " NIC:               %u\n",
            sv->nic);

    if (sv->position_valid)
        fprintf(to,
                " Latitude:          %+.4f\n"
                " Longitude:         %+.4f\n",
                sv->lat,
                sv->lon);

    if (sv->altitude_valid)
        fprintf(to,
                " Altitude:          %d ft (%s)\n",
                sv->altitude,
                sv->altitude_type == BARO ? "barometric" : "geometric");

    if (sv->ns_vel_valid)
        fprintf(to,
                " N/S velocity:      %d kt\n",
                sv->ns_vel);

    if (sv->ew_vel_valid)
        fprintf(to,
                " E/W velocity:      %d kt\n",
                sv->ew_vel);

    if (sv->track_valid) {
        switch (sv->track_type) {
        case AIRBORNE_TRACK:
            fprintf(to,
                    " Track:             %u\n",
                    sv->track);
            break;
        case GROUND_TRACK:
            fprintf(to,
                    " Ground track:      %u\n",
                    sv->track);
            break;
        case GROUND_MAG_HEADING:
            fprintf(to,
                    " Ground heading:    %u (magnetic)\n",
                    sv->track);
            break;
        case GROUND_TRUE_HEADING:
            fprintf(to,
                    " Ground heading:    %u (true)\n",
                    sv->track);
            break;
        }
    }

    if (sv->speed_valid)
        fprintf(to,
                " Speed:             %u kt\n",
                sv->speed);
        
    if (sv->vert_rate_valid)
        fprintf(to,
                " Vertical rate:     %d ft/min (%s)\n",
                sv->vert_rate,
                sv->vert_rate_source == BARO ? "barometric" : "geometric");
        
    if (sv->lengthwidth_valid)
        fprintf(to,
                " Dimensions:        %.1fm L x %.1fm W%s\n",
                sv->length, sv->width,
                sv->position_offset ? " (position offset applied)" : "");
    
    fprintf(to,
            " UTC coupling:      %s\n"
            " TIS-B site ID:     %u\n",
            sv->utc_coupled ? "yes" : "no",
            sv->tisb_site_id);
}

static char base40_alphabet[40] = "0123456789ABCDEFGHIJKLMNOPQRTSUVWXYZ  ..";
void uat_decode_ms(uint8_t *frame, struct uat_ms *ms)
{
    uint16_t v;
    int i;

    v = (frame[17]<<8) | (frame[18]);
    ms->emitter_category = (v/1600) % 40;
    ms->callsign[0] = base40_alphabet[(v/40) % 40];
    ms->callsign[1] = base40_alphabet[v % 40];
    v = (frame[19]<<8) | (frame[20]);
    ms->callsign[2] = base40_alphabet[(v/1600) % 40];
    ms->callsign[3] = base40_alphabet[(v/40) % 40];
    ms->callsign[4] = base40_alphabet[v % 40];
    v = (frame[21]<<8) | (frame[22]);
    ms->callsign[5] = base40_alphabet[(v/1600) % 40];
    ms->callsign[6] = base40_alphabet[(v/40) % 40];
    ms->callsign[7] = base40_alphabet[v % 40];
    ms->callsign[8] = 0;

    // trim trailing spaces
    for (i = 7; i >= 0; --i) {
        if (ms->callsign[i] == ' ')
            ms->callsign[i] = 0;
        else
            break;
    }

    ms->emergency_status = (frame[23] >> 5) & 7;
    ms->uat_version = (frame[23] >> 2) & 7;
    ms->sil = (frame[23] & 3);
    ms->transmit_mso = (frame[24] >> 2) & 0x3f;
    ms->nac_p = (frame[25] >> 4) & 15;
    ms->nac_v = (frame[25] >> 1) & 7;
    ms->nic_baro = (frame[25] & 1);
    ms->has_cdti = (frame[26] & 0x80 ? 1 : 0);
    ms->has_acas = (frame[26] & 0x40 ? 1 : 0);
    ms->acas_ra_active = (frame[26] & 0x20 ? 1 : 0);
    ms->ident_active = (frame[26] & 0x10 ? 1 : 0);
    ms->atc_services = (frame[26] & 0x08 ? 1 : 0);
    ms->heading_type = (frame[26] & 0x04 ? MAGNETIC : TRUE);
    ms->callsign_id = (frame[26] & 0x02 ? 1 : 0);
}

static const char *emitter_category_names[40] = {
    "No information",                          // 0
    "Light <= 7000kg",
    "Medium Wake 7000-34000kg",
    "Medium Wake 34000-136000kg",
    "Medium Wake High Vortex 34000-136000kg",  // 4
    "Heavy >= 136000kg",
    "Highly Maneuverable",
    "Rotorcraft",
    "reserved (8)",                            // 8
    "Glider/Sailplane",
    "Lighter than air",
    "Parachutist / sky diver",
    "Ultra light / hang glider / paraglider",  // 12
    "reserved (13)",
    "UAV",
    "Space / transatmospheric",
    "reserved (16)",                           // 16
    "Emergency vehicle",
    "Service vehicle",
    "Point obstacle",
    "Cluster obstacle",                        // 20
    "Line obstacle",
    "reserved (22)",
    "reserved (23)",
    "reserved (24)",
    "reserved (25)",
    "reserved (26)",
    "reserved (27)",
    "reserved (28)",
    "reserved (29)",
    "reserved (30)",
    "reserved (31)",
    "reserved (32)",
    "reserved (33)",
    "reserved (34)",
    "reserved (35)",
    "reserved (36)",
    "reserved (37)",
    "reserved (38)",
    "reserved (39)"
};    

static const char *emergency_status_names[8] = {
    "No emergency",
    "General emergency",
    "Lifeguard / Medical emergency",
    "Minimum fuel",
    "No communications",
    "Unlawful interference",
    "Downed aircraft",
    "reserved"
};

void uat_display_ms(const struct uat_ms *ms, FILE *to)
{
    fprintf(to,
            "MS:\n"
            " Emitter category:  %s\n"
            " Callsign:          %s%s\n"
            " Emergency status:  %s\n"
            " UAT version:       %u\n"
            " SIL:               %u\n"
            " Transmit MSO:      %u\n"
            " NACp:              %u\n"
            " NACv:              %u\n"
            " NICbaro:           %u\n"
            " Capabilities:      %s%s\n"
            " Active modes:      %s%s%s\n"
            " Target track type: %s\n",
            emitter_category_names[ms->emitter_category],
            ms->callsign[0] ? ms->callsign : "unavailable", ms->callsign_id ? "" : " (alternative)",
            emergency_status_names[ms->emergency_status],
            ms->uat_version,
            ms->sil,
            ms->transmit_mso,
            ms->nac_p,
            ms->nac_v,
            ms->nic_baro,
            ms->has_cdti ? "CDTI " : "", ms->has_acas ? "ACAS " : "",
            ms->acas_ra_active ? "ACASRA " : "", ms->ident_active ? "IDENT " : "", ms->atc_services ? "ATC " : "",
            ms->heading_type == MAGNETIC ? "magnetic heading" : "true heading");
}

void uat_decode_auxsv(uint8_t *frame, struct uat_auxsv *auxsv)
{
    int raw_alt = (frame[29] << 4) | ((frame[30] & 0xf0) >> 4);
    if (raw_alt != 0) {
        auxsv->sec_altitude_valid = 1;
        auxsv->sec_altitude = (raw_alt - 1) * 25 - 1000;
        auxsv->sec_altitude_type = (frame[9] & 1) ? BARO : GEO;
    } else {
        auxsv->sec_altitude_valid = 0;
    }
}    


void uat_display_auxsv(const struct uat_auxsv *auxsv, FILE *to)
{
    fprintf(to,
            "AUXSV:\n");

    if (auxsv->sec_altitude_valid)
        fprintf(to,
                " Sec. altitude:     %d ft (%s)\n",                
                auxsv->sec_altitude,
                auxsv->sec_altitude_type == BARO ? "barometric" : "geometric");
    else
        fprintf(to,
                " Sec. altitude:     unavailable\n");
}

void uat_decode_adsb_mdb(uint8_t *frame, struct uat_adsb_mdb *mdb)
{
    uat_decode_hdr(frame, &mdb->hdr);   
    if (mdb->hdr.mdb_type <= 10) {
        mdb->sv_valid = 1;
        uat_decode_sv(frame, &mdb->sv);
    } else {
        mdb->sv_valid = 0;
    }
    
    switch (mdb->hdr.mdb_type) {
    case 0: // HDR SV
    case 4: // HDR SV (TC+0) (TS)
    case 7: // HDR SV reserved
    case 8: // HDR SV reserved
    case 9: // HDR SV reserved
    case 10: // HDR SV reserved
        mdb->sv_valid = 1;
        mdb->ms_valid = mdb->auxsv_valid = 0;
        uat_decode_sv(frame, &mdb->sv);
        break;

    case 1: // HDR SV MS AUXSV
        mdb->sv_valid = mdb->ms_valid = mdb->auxsv_valid = 1;
        uat_decode_sv(frame, &mdb->sv);
        uat_decode_ms(frame, &mdb->ms);
        uat_decode_auxsv(frame, &mdb->auxsv);
        break;

    case 2: // HDR SV AUXSV
    case 5: // HDR SV (TC+1) AUXSV
    case 6: // HDR SV (TS) AUXSV
        mdb->sv_valid = mdb->auxsv_valid = 1;
        mdb->ms_valid = 0;
        uat_decode_sv(frame, &mdb->sv);
        uat_decode_auxsv(frame, &mdb->auxsv);
        break;

    case 3: // HDR SV MS (TS)
        mdb->sv_valid = mdb->ms_valid = 1;
        mdb->auxsv_valid = 0;
        uat_decode_sv(frame, &mdb->sv);
        uat_decode_ms(frame, &mdb->ms);
        break;

    default:
        mdb->sv_valid = mdb->ms_valid = mdb->auxsv_valid = 0;
        break;
    }
}

void uat_display_adsb_mdb(const struct uat_adsb_mdb *mdb, FILE *to)
{
    uat_display_hdr(&mdb->hdr, to);
    if (mdb->sv_valid)
        uat_display_sv(&mdb->sv, to);
    if (mdb->ms_valid)
        uat_display_ms(&mdb->ms, to);
    if (mdb->auxsv_valid)
        uat_display_auxsv(&mdb->auxsv, to);
}
