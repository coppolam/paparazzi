/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * Steven_guided_ctrl.h
 *
 *  Created on: Sep 11, 2017
 *      Author: steven <stevenhelm@live.nl>
 */

#include "math/pprz_algebra_int.h"
#include "navigation.h"
#include "autopilot.h"
#include "../../firmwares/rotorcraft/guidance/guidance_h.h"
#include "../../firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/airframe.h"
#include "std.h"
#include "Steven_guided_ctrl.h"



void guided_ctrl_init(void){

}

void guided_ctrl_per(void){

}

bool setForwardVelocity(float velx){
	bool temp = true;
	temp &= guidance_v_set_guided_z(-1.0);
	temp &= guidance_h_set_guided_vel(velx,0);
	return !temp;
}

bool stopFlying(void){
	bool temp = true;
	temp &= guidance_v_set_guided_z(-1.0);
	temp &= guidance_h_set_guided_vel(0.0,0.0);
	return !temp;
}

bool goLand(void){
	bool temp = true;
	temp &= guidance_v_set_guided_vz(0.05);
	temp &= guidance_v_set_guided_z(0.0);
	temp &= guidance_h_set_guided_vel(0.0,0.0);
	return !temp;
}
