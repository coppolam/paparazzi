/*
 * Copyright (C) Pascal Brisset, Antoine Drouin (2008), Kirk Scheper (2016)
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/traffic_info/traffic_info.c"
 * @author Kirk Scheper
 * Information relative to the other aircrafts.
 * Keeps track of other aircraft in airspace
 */

#include "modules/multi/traffic_info.h"

#include "generated/airframe.h"     // AC_ID
#include "generated/flight_plan.h"  // NAV_MSL0

#include "subsystems/datalink/datalink.h"
#include "pprzlink/dl_protocol.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "math/pprz_geodetic_utm.h"

#include "subsystems/gps.h"

uint8_t acs_idx;
uint8_t the_acs_id[NB_ACS_ID];
struct ac_info_ the_acs[NB_ACS];

void traffic_info_init(void)
{
  memset(the_acs_id, 0, NB_ACS_ID);

  the_acs_id[0] = 0;  // ground station
  the_acs_id[AC_ID] = 1;
  the_acs[the_acs_id[AC_ID]].ac_id = AC_ID;
  acs_idx = 2;
}

int parse_acinfo(){
  set_ac_info(DL_ACINFO_ac_id(dl_buffer),
    DL_ACINFO_utm_east(dl_buffer),
    DL_ACINFO_utm_north(dl_buffer),
    DL_ACINFO_alt(dl_buffer)*10 + NAV_MSL0, // hack because ground station sends hmsl
    DL_ACINFO_utm_zone(dl_buffer),
    DL_ACINFO_course(dl_buffer),
    DL_ACINFO_speed(dl_buffer),
    DL_ACINFO_climb(dl_buffer),
    DL_ACINFO_itow(dl_buffer));

  return 1;
}

struct ac_info_ *get_ac_info(uint8_t _id)
{
  return &the_acs[the_acs_id[_id]];
}

void set_ac_info(uint8_t id, uint32_t utm_east, uint32_t utm_north, uint32_t alt, uint8_t utm_zone, uint16_t course,
                uint16_t gspeed, uint16_t climb, uint32_t itow)
{
  if (acs_idx < NB_ACS) {
    if (id > 0 && the_acs_id[id] == 0) {    // new aircraft id
      the_acs_id[id] = acs_idx++;
      the_acs[the_acs_id[id]].ac_id = id;
    }
    uint16_t my_zone = UtmZoneOfLlaLonDeg(gps.lla_pos.lon);
    if (utm_zone == my_zone) {
      the_acs[the_acs_id[id]].utm.east = utm_east;
      the_acs[the_acs_id[id]].utm.north = utm_north;
      the_acs[the_acs_id[id]].utm.alt = alt;
      the_acs[the_acs_id[id]].utm.zone = utm_zone;
      the_acs[the_acs_id[id]].course = course;
      the_acs[the_acs_id[id]].gspeed = gspeed;
      the_acs[the_acs_id[id]].climb = climb;
      the_acs[the_acs_id[id]].itow = itow;
    } else { // store other uav in utm extended zone
      struct UtmCoor_i utm = {.east = utm_east, .north = utm_north, .alt = alt, .zone = my_zone};
      struct LlaCoor_i lla;
      lla_of_utm_i(&lla, &utm);

      set_ac_info_lla(id, lla.lat, lla.lon, lla.alt, course, gspeed, climb, itow);
    }
  }
}

void set_ac_info_lla(uint8_t id, int32_t lat, int32_t lon, int32_t alt,
                     int16_t course, uint16_t gspeed, int16_t climb, uint32_t itow)
{

  if (acs_idx < NB_ACS) {
    if (id > 0 && the_acs_id[id] == 0) {
      the_acs_id[id] = acs_idx++;
      the_acs[the_acs_id[id]].ac_id = id;
    }

    struct LlaCoor_i lla = {.lat = lat, .lon = lon, .alt = alt};
    struct UtmCoor_i utm;
    utm.zone = UtmZoneOfLlaLonDeg(gps.lla_pos.lon);   // use current zone as reference, i.e zone extend

    utm_of_lla_i(&utm, &lla);

    the_acs[the_acs_id[id]].utm.east = lat;
    the_acs[the_acs_id[id]].utm.north = lon;
    the_acs[the_acs_id[id]].utm.alt = alt;
    //UTM_COPY(the_acs[the_acs_id[id]].utm, utm);
    the_acs[the_acs_id[id]].course = course;
    the_acs[the_acs_id[id]].gspeed = gspeed;
    the_acs[the_acs_id[id]].climb = climb;
    the_acs[the_acs_id[id]].itow = itow;
  }
}
