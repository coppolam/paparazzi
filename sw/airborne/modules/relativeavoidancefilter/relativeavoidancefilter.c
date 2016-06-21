/*
 * Copyright (C) Mario Coppola
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
 * @file "modules/relativeavoidancefilter/relativeavoidancefilter.c"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#include "relativeavoidancefilter.h"

#define PSISEARCH 15.0 		// Search grid for psi_des
#define NUAVS 5				// Maximum expected number of drones
#define MAF_SIZE_POS 1  	// Moving Average Filter size;
#define MAF_SIZE_VEL 1  	// Moving Average Filter size;

#ifndef INS_INT_VEL_ID
#define INS_INT_VEL_ID ABI_BROADCAST
#endif



ekf_filter ekf[NUAVS-1]; 	// EKF structure
btmodel model[NUAVS-1];  	// Bluetooth model structure 
int IDarray[NUAVS-1]; 		// Array of IDs of other MAVs
int8_t srcstrength[NUAVS-1];// Source strength
uint32_t now_ts[NUAVS-1]; 	// Time of last received message from each MAV

int nf; 					// Number of filters registered
float psi_des, v_des; 		// psi_des = desired course w.r.t. north, v_des = magnitude of velocity
float vx_des, vy_des;		// Desired velocities in NED frame
float RSSIarray[NUAVS-1];	// Recorded RSSI values (so they can all be sent)
float magprev;				// Previous magnitude from 0,0 (for simulated wall detection)

float ccvec[NUAVS-1][4];
float x_est[NUAVS-1][MAF_SIZE_POS], y_est[NUAVS-1][MAF_SIZE_POS];
float vx_est[NUAVS-1][MAF_SIZE_VEL], vy_est[NUAVS-1][MAF_SIZE_VEL];

float ownVx_old;
float ownVy_old;
float trackedVx_old;
float trackedVy_old;

struct FloatVect3 vel_body;   // body frame velocity from sensors

//  AbiBindMsgVELOCITY_ESTIMATE(INS_INT_VEL_ID, &vel_est_ev, vel_est_cb);
static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi);

static abi_event vel_est_ev;
static void vel_est_cb(uint8_t sender_id, uint32_t stamp, float x, float y, float z, float noise);


static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi)
{
	int i= -1; // Initialize index
	printf("message received with rssi %d for drone %d number %d \n", rssi, ac_id, i);
	// If it's a new ID we start a new EKF for it
	// TODO: aircraft ID are now hard coded here, make it more general (set in airframe file?)
	if ((!array_find_int(NUAVS-1, IDarray, ac_id, &i)) && (nf < NUAVS-1) && ( (ac_id== 200) || (ac_id == 201) || (ac_id == 202) ) ) { //check if a new aircraft ID is present, continue
		IDarray[nf] = ac_id;
		srcstrength[nf] = source_strength;
		ekf_filter_new(&ekf[nf]); // Initialize an ekf filter for each target tracker

		// Set up the Q and R matrices and all the rest.
		fmat_scal_mult(9,9, ekf[nf].Q, pow(0.5,2), ekf[nf].Q);
		fmat_scal_mult(8,8, ekf[nf].R, pow(0.2,2), ekf[nf].R);
		ekf[nf].Q[0]   = 0.01;
		ekf[nf].Q[9+1] = 0.01;
		ekf[nf].R[0]   = pow(RSSINOISE,2.0);
		ekf[nf].X[0]   = 1.0; // Initial positions cannot be zero or else you'll divide by zero
		ekf[nf].X[1]   = 1.0;
		ekf[nf].dt     = 0.2; // Initial assumption (STDMA code runs at 5Hz)
		model[nf].Pn   = -63.0 - 8.0 + (float)source_strength; // -63 is from calibration with antenna that had 8dB power, so the source_strength recalibrates
		model[nf].gammal = 2.0;
		nf++;
	}

	// If we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == (NUAVS-1)) ) {
		RSSIarray[i] = rssi; //logging

		if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) // only in guided mode (flight) (take off in NAV)
		{
			// Update the time between messages
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6);
			now_ts[i] = get_sys_time_usec();
			
			// Get the aircraft info for that ID
			struct ac_info_ * ac = get_ac_info(ac_id);
			float angle, trackedVx, trackedVy;
			
			deg2rad   (((float)ac->course)/10.0, &angle); // decidegrees
			polar2cart(((float)ac->gspeed)/100.0, angle, &trackedVx, &trackedVy); // get x and y velocities (cm/s)
			
			// Bind received values in case of errors
			float ownVx = vel_body.x; //From optical flow
			float ownVy = vel_body.y;
			
			// Bind velocitiesto a known maximum to avoid occasional errors
			keepbounded(&ownVx,-2.0,2.0);
			keepbounded(&ownVy,-2.0,2.0);
			keepbounded(&trackedVx,-2.0,2.0);
			keepbounded(&trackedVy,-2.0,2.0);
			
			// See UtmCoor for north pos in z
			// Construct measurement vector for EKF using the latest data obtained for each case
			float Y[8];
			Y[0] = (float)rssi;// + rand_normal(0.0, 5.0); // Bluetooth RSSI measurement
			//Y[1] = stateGetSpeedNed_f()->x + rand_normal(0.0, 0.2); // next three are updated in the periodic function
			//Y[2] = stateGetSpeedNed_f()->y + rand_normal(0.0, 0.2);
			Y[1] = ownVx;
			Y[2] = ownVy;
			Y[3] = 0.0; //stateGetNedToBodyEulers_f()->psi; // get own body orientation
			Y[4] = trackedVx;  //Velocity tracked from other drone
			Y[5] = trackedVy;	
			Y[6] = 0.0;        //other drone heading towards north (hardcoded to be 0)
			Y[7] = (-(float)ac->utm.alt/1000.0) - (stateGetPositionNed_f()->z) + rand_normal(0.0, 0.2); //utm.alt is in mm above 0
			
			// Run the steps of the EKF
			ekf_filter_predict(&ekf[i], &model[i]);
			ekf_filter_update (&ekf[i], Y);
				
			/*
			 * Xvector: dotx_other and doty_other are expressed in own bodyframe
			 * X = [x y dotx_own doty_own dotx_other doty_other psi_own psi_other dh]
			 * Yvector: dotx_other and dotyother are not rotated yet
			 * Y = [RSSI dotx_own doty_own psi_own dotx_other doty_other psi_other dh]
			 */
			// Moving average filter to state (MAF_SIZE of 1, average with direct last measurment)
			ccvec[i][0] = movingaveragefilter(x_est[i],  MAF_SIZE_POS, ekf[i].X[0]);
			ccvec[i][1] = movingaveragefilter(y_est[i],  MAF_SIZE_POS, ekf[i].X[1]);				
			ccvec[i][2] = movingaveragefilter(vx_est[i], MAF_SIZE_VEL, ekf[i].X[4]);
			ccvec[i][3] = movingaveragefilter(vy_est[i], MAF_SIZE_VEL, ekf[i].X[5]);
						
		}
		else { // Initial estimate is towards the initial direction of flight (filter of other drone!)
			ekf[i].X[0] = -stateGetPositionNed_f()->x; // Initial positions cannot be zero or else you'll divide by zero
			ekf[i].X[1] = -stateGetPositionNed_f()->y;
		}
	}
};


static void vel_est_cb(uint8_t sender_id __attribute__((unused)),
                       uint32_t stamp,
                       float x, float y, float z,
                       float noise __attribute__((unused))) {

	//TODO make more generic
	static float vel_body_x_buf[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	static float vel_body_y_buf[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


	vel_body.x =  movingaveragefilter(&vel_body_x_buf, 12, x);
	vel_body.y = movingaveragefilter(&vel_body_y_buf, 12, y);

    // vel_body.x = x;
	// vel_body.y = y;
}


bool alternate;
static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
{	
	// Store the relative localization data
	uint8_t i;

	if (nf == 2) {
		if (alternate) {
			alternate = false;
			i = 1;
		}
		else {
			alternate = true;
			i = 0;
		}
	}
	else { //only data on first
		i = 0;
	}
	
	pprz_msg_send_RAFILTERDATA(trans, dev, AC_ID,
		&IDarray[i],			     // ID or filtered aircraft number
		&RSSIarray[i], 		    	 // Received ID and RSSI
		&srcstrength[i],		     // Source strength
		&ekf[i].X[0], &ekf[i].X[1],  // x and y pos
		&ekf[i].X[2], &ekf[i].X[3],  // Own vx and vy
		&ekf[i].X[4], &ekf[i].X[5],  // Received vx and vy
		&ekf[i].X[6], &ekf[i].X[7],  // Orientation own , orientation other
		&ekf[i].X[8], 			     // Height separation
		&vx_des, &vy_des);		     // Commanded velocities
};

void rafilter_init(void)
{
	randomgen_init();    	// Initialize the random generator (for simulation purposes)
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	
	alternate = 0;
	nf 		  = 0; 		   	// Number of active filters
	psi_des   = 0.0;	   	// Initialize
	v_des     = V_NOMINAL; 	// Initial desired velocity
	magprev   = 3.0; 	   	// Just a random high value
	
	for (int i = 0; i < NUAVS-1; i++) {
		fmat_make_zeroes( x_est[i],  1, MAF_SIZE_POS );
		fmat_make_zeroes( y_est[i],  1, MAF_SIZE_POS );
		fmat_make_zeroes( vx_est[i], 1, MAF_SIZE_POS );
		fmat_make_zeroes( vy_est[i], 1, MAF_SIZE_POS );
	}

	// Subscribe to the ABI RSSI messages
	AbiBindMsgRSSI(ABI_BROADCAST,  &rssi_ev,    bluetoothmsg_cb);
	AbiBindMsgVELOCITY_ESTIMATE(ABI_BROADCAST, &vel_est_ev, vel_est_cb);

	// Send out the filter data
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RAFILTERDATA, send_rafilterdata);
};


void rafilter_periodic(void)
{	
	// FAKE MESSAGE for testing purposes from the 0.0 position if a BT source is not available!
	// float d = sqrt(pow(stateGetPositionEnu_f()->x,2)+pow(stateGetPositionEnu_f()->y,2));
	// AbiSendMsgRSSI(1, 2, 2,  (int)(-63 -2*10*log10(d)));

	/*********************************************
		Sending speed directly between drones
	*********************************************/
	//float velx = stateGetSpeedNed_f()->x;
	//float vely = stateGetSpeedNed_f()->y;
	float velx = vel_body.x;
	float vely = vel_body.y;
		
	float spd, crs_vel_body, crs_bodyaxis;
	crs_bodyaxis = stateGetNedToBodyEulers_f()->psi;
	
	// Convert Course to the proper format
	cart2polar(velx, vely, &spd, &crs_vel_body); 	// Get the total speed and course
	wrapTo2Pi(&crs_vel_body); 					    // Wrap to 2 Pi since the sent result is unsigned
	wrapTo2Pi(&crs_bodyaxis); 					    // Wrap to 2 Pi since the sent result is unsigned
	float crs_vel_earth = - crs_vel_body + crs_bodyaxis;
	wrapTo2Pi(&crs_vel_earth); 					    // Wrap to 2 Pi since the sent result is unsigned

	int32_t course = (int32_t)(crs_vel_body*(1e7)); 	// Typecast crs into a int32_t type integer with proper unit (see gps.course in gps.h)
	// int32_t course = (int32_t)(crs*(1e7)); 	// Typecast crs into a int32_t type integer with proper unit (see gps.course in gps.h)

	uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(course) / 1e7) / 2)) & 0x7FF) <<
	                        21; 									  // bits 31-21 course (where the magnitude is pointed at)
	multiplex_speed |= (((uint32_t)(spd*100)) & 0x7FF) << 10;         // bits 20-10 speed in cm/s
	multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);        // bits 9-0 z velocity in cm/s

	int16_t alt = (int16_t)(gps.hmsl / 10); 							// height in dm

	DOWNLINK_SEND_GPS_SMALL(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &multiplex_speed, &gps.lla_pos.lat,
                          &gps.lla_pos.lon, &alt);                     // Messages throught USB bluetooth dongle to other drones


	/*********************************************
		Relative Avoidance Behavior
	*********************************************/

	if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED) {
		float cc[nf][6];

		// Pos in X and Y just for arena border detection!
		float posx = stateGetPositionNed_f()->x; 
		float posy = stateGetPositionNed_f()->y;

		bool flagglobal = false; 		// Null assumption
		bool wallgettingcloser = false; // Null assumption

		// Wall detection algorithm
		if (sqrt(pow(posx,2) + pow(posy,2)) > magprev) {
			wallgettingcloser = true;
		}
		magprev = sqrt(pow(posx,2) + pow(posy,2));

		// Change direction to avoid wall
		if ( ((abs(posx) > (ASIDE-0.5)) || (abs(posy) > (ASIDE-0.5))) && wallgettingcloser) {
			//Equivalent to PID with gain 1 towards center. This is only to get the direction anyway.
			cart2polar(-posx,-posy, &v_des, &psi_des);
			v_des = V_NOMINAL;
		}
		else {
			
			polar2cart(v_des, psi_des, &vx_des, &vy_des); // vx_des & vy_des = desired velocity
			uint8_t i;
			for ( i = 0; i < nf; i++ ) { // nf is amount of filters running
				float dist = sqrt(pow(ekf[i].X[0],2) + pow(ekf[i].X[1],2));
				float eps = 1.0*ASIDE*tan(1.7/2) - MAVSIZE - ASIDE;
				
				// cc = collisioncone
				collisioncone_update(cc[i], ccvec[i][0], ccvec[i][1], ccvec[i][2], ccvec[i][3], dist+MAVSIZE+eps);	// x y xdot_0 ydot_0 (characteristics collision cones)
				
				if ( collisioncone_checkdanger( cc[i], vx_des, vy_des )) {
					flagglobal = true; 		// We could be colliding!
					// TODO: Change flagglobal name
				}
			}
	
			if (flagglobal) { 		// If the desired velocity doesn't work, then let's find the next best thing according to VO
				v_des = V_NOMINAL;
				collisioncone_findnewcmd(cc, &v_des, &psi_des, PSISEARCH, nf); // Go 15 degrees clockwise until save direction in found
			}
			
		}

		polar2cart(v_des, psi_des, &vx_des, &vy_des);  // new desired speed
		autopilot_guided_move_ned(vx_des, vy_des, 0.0, 0.0);  //send to guided mode

	}

};
