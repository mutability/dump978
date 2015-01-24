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

#ifndef UAT_DECODE_H
#define UAT_DECODE_H

#include <stdint.h>
#include <stdio.h>

#include "uat.h"

void uat_decode_hdr(uint8_t *frame, struct uat_hdr *hdr);
void uat_display_hdr(const struct uat_hdr *hdr, FILE *to);
void uat_decode_sv(uint8_t *frame, struct uat_sv *sv);
void uat_display_sv(const struct uat_sv *sv, FILE *to);
void uat_decode_ms(uint8_t *frame, struct uat_ms *ms);
void uat_display_ms(const struct uat_ms *ms, FILE *to);
void uat_decode_auxsv(uint8_t *frame, struct uat_auxsv *auxsv);
void uat_display_auxsv(const struct uat_auxsv *auxsv, FILE *to);
void uat_decode_adsb_mdb(uint8_t *frame, struct uat_adsb_mdb *mdb);
void uat_display_adsb_mdb(const struct uat_adsb_mdb *mdb, FILE *to);

#endif
