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
 * @file "modules/relativelocalizationfilter/relativelocalizationfilter.c"
 * @author Mario Coppola
 * Relative Localization Filter for collision avoidance between drones
 */

#include <time.h>
#include <pthread.h>
#include "relativelocalizationfilter.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/multi/traffic_info.h"
#include "modules/stdma/stdma.h"
#include "modules/decawave/Serial/Serial_Communication.h"
#include "subsystems/datalink/bluegiga.h"

#include "pprzlink/pprz_transport.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

#ifndef NUAVS
#define NUAVS 5				// Maximum expected number of drones
#endif

#ifndef INS_INT_VEL_ID
#define INS_INT_VEL_ID ABI_BROADCAST
#endif

#ifndef UWB_LOCALIZATION
#define UWB_LOCALIZATION
#endif
int IDarray[NUAVS-1]; 		// Array of IDs of other MAVs
uint32_t now_ts[NUAVS-1]; 	// Time of last received message from each MAV
int nf;						// Number of filters registered
ekf_filter ekf[NUAVS-1]; 	// EKF structure
float rangearray[NUAVS-1];	// Recorded RSSI values (so they can all be sent)
struct EnuCoor_f current_pos;
struct EnuCoor_f current_speed;
int counter = 0;

//char rlFileName[50] = "/data/ftp/internal_000/rlLogFile1.csv";

/** The file pointer */
static FILE *rlFileLogger = NULL;
char* rlconcat(const char *s1, const char *s2);

static pthread_mutex_t ekf_mutex;

#define RLLOG 0
/*
#ifdef RSSI_LOCALIZATION
int8_t srcstrength[NUAVS-1];// Source strength
float RSSIarray[NUAVS-1];	// Recorded RSSI values (so they can all be sent)
btmodel model[NUAVS-1];  	// Bluetooth model structure 

static abi_event rssi_ev;
static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi);

static void bluetoothmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, int8_t source_strength, int8_t rssi)
{ 
	int i = -1; // Initialize the index of all tracked drones (-1 for null assumption of no drone found).

	// Check if a new aircraft ID is present, if it's a new ID we start a new EKF for it.
	if (( !array_find_int(NUAVS-1, IDarray, ac_id, &i))  // If yes, a new drone is found.
		   && (nf < NUAVS-1))  // If yes, the amount of drones does not exceed the maximum.
	{
		IDarray[nf] = ac_id; 				// Store ID in an array (logging purposes)
		srcstrength[nf] = source_strength;  // Store source strength in an array (logging purposes)
		ekf_filter_new(&ekf[nf]); 			// Initialize an EKF filter for the newfound drone

		// Set up the Q and R matrices and all the rest
		// Weights are based on:
		// Coppola et al, "On-board Communication-based Relative Localization for Collision Avoidance in Micro Air Vehicle teams", 2017
		fmat_scal_mult(EKF_N,EKF_N, ekf[nf].Q, pow(0.5,2.0), ekf[nf].Q);
		fmat_scal_mult(EKF_M,EKF_M, ekf[nf].R, pow(SPEEDNOISE,2.0), ekf[nf].R);
		ekf[nf].Q[0]   	   = 0.01; // Reccomended 0.01 to give this process a high level of trust
		ekf[nf].Q[EKF_N+1] = 0.01;
		ekf[nf].R[0]   = pow(RSSINOISE,2.0);
			
		// Initialize the states
		// Initial position cannot be zero or the filter will divide by zero on initialization
		ekf[i].X[0] = 1.0; // Relative position North
		ekf[i].X[1] = 1.0; // Relative position East
		// The other variables can be initialized at 0
		ekf[i].X[2] = 0.0; // Own Velocity North
		ekf[i].X[3] = 0.0; // Own Velocity East
		ekf[i].X[4] = 0.0; // Relative velocity North
		ekf[i].X[5] = 0.0; // Relative velocity East
		ekf[i].X[6] = 0.0; // Height difference

		ekf[nf].dt       = 0.2;  // Initial assumption for time difference between messages (STDMA code runs at 5Hz)
		model[nf].Pn     = -63;  // Expected RSSI at 1m (based on experience)
		model[nf].gammal = 2.0;	 // Expected Space-loss parameter (based on free space assumption)
		nf++; 					 // Number of filter is present is increased
	}
	// Else, if we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == (NUAVS-1)) )
	{
		RSSIarray[i] = (float)rssi; // Store RSSI in array (for logging purposes)

		// Get own velocities
		float ownVx = stateGetSpeedEnu_f()->y; // Velocity North in NED
		float ownVy = stateGetSpeedEnu_f()->x; // Velocity East in NED
		// Bind to realistic amounts to avoid occasional spikes/NaN/inf errors
		keepbounded(&ownVx,-2.0,2.0);
		keepbounded(&ownVy,-2.0,2.0);

		// Make the filter only in Guided mode (flight).
		// This is because it is best for the filter should only start once the drones are in motion, 
		// otherwise it might diverge while drones are not moving.
		if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
		{
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6); // Update the time between messages
			
			// Get the velocity in NED for the tracked aircraft
			float trackedVx, trackedVy;
			polar2cart(acInfoGetGspeed(ac_id), acInfoGetCourse(ac_id), &trackedVx, &trackedVy); // get North and East velocities (m/s)
			// As for own velocity, bind to realistic amounts to avoid occasional spikes/NaN/inf errors
			keepbounded(&trackedVx,-2.0,2.0);
			keepbounded(&trackedVy,-2.0,2.0);

			// Construct measurement vector Y for EKF using the latest data obtained.
			// Y = [RSSI owvVx ownVy trackedVx trackedVy dh], EKF_M = 6 as defined in discreteekf.h
			float Y[EKF_M];
			Y[0] = (float)rssi; 	//RSSI measurement
			Y[1] = ownVx; 	   		// Own velocity North (NED frame)
			Y[2] = ownVy;			// Own velocity East  (NED frame)
			Y[3] = trackedVx;  		// Velocity of other drone Norht (NED frame)
			Y[4] = trackedVy;		// Velocity of other drone East  (NED frame)
			Y[5] = acInfoGetPositionUtm_f(ac_id)->alt - stateGetPositionEnu_f()->z;  // Height difference
			
			// Run the steps of the EKF, but only if velocity difference is significant (to filter out minimal noise)
			if (  sqrt( pow(Y[1]-Y[3],2) + pow(Y[2]-Y[4],2) ) > 0.05 )
			{
				ekf_filter_predict(&ekf[i], &model[i]); // Prediction step of the EKF 
				ekf_filter_update(&ekf[i], Y);	// Update step of the EKF
			}
		}

		now_ts[i] = get_sys_time_usec();  // Store latest time

	}
};
#else //if UWB_LOCALIZATION
*/

int cnt;

static abi_event uwb_ev;
static void uwbmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh);

static void uwbmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh) 
{


	int i = -1; // Initialize the index of all tracked drones (-1 for null assumption of no drone found).

	// Check if a new aircraft ID is present, if it's a new ID we start a new EKF for it.
	if (( !array_find_int(NUAVS-1, IDarray, ac_id, &i))  // If yes, a new drone is found.
		   && (nf < NUAVS-1))  // If yes, the amount of drones does not exceed the maximum.
	{
		pthread_mutex_lock(&ekf_mutex);
		IDarray[nf] = ac_id; 				// Store ID in an array (logging purposes)
		ekf_filter_new(&ekf[nf]); 			// Initialize an EKF filter for the newfound drone

		// Set up the Q and R matrices and all the rest
		// Weights are based on:
		// Coppola et al, "On-board Communication-based Relative Localization for Collision Avoidance in Micro Air Vehicle teams", 2017
		fmat_scal_mult(EKF_N, EKF_N, ekf[nf].Q, pow(0.3,2.0), ekf[nf].Q);
		fmat_scal_mult(EKF_M, EKF_M, ekf[nf].R, pow(0.1,2.0), ekf[nf].R);
		ekf[nf].Q[0]   	   = 0.01; // Reccomended 0.01 to give this process a high level of trust
		ekf[nf].Q[EKF_N+1] = 0.01;
		ekf[nf].R[0]   = 0.2;
		ekf[nf].P[EKF_N*2+2] = 0.1;
		ekf[nf].P[EKF_N*3+3] = 0.1;
		ekf[nf].P[EKF_N*4+4] = 0.1;
		ekf[nf].P[EKF_N*5+5] = 0.1;
		ekf[nf].P[EKF_N*5+5] = 0.1;

		// Initialize the states
		// Initial position cannot be zero or the filter will divide by zero on initialization
		ekf[nf].X[0] = 0.0; // Relative position North
		ekf[nf].X[1] = 2.5; // Relative position East
		// The other variables can be initialized at 0
		ekf[nf].X[2] = 0.0; // Own Velocity North
		ekf[nf].X[3] = 0.0; // Own Velocity East
		ekf[nf].X[4] = 0.0; // Velocity other North
		ekf[nf].X[5] = 0.0; // Velocity other East
		ekf[nf].X[6] = 0.0; // Height difference
		ekf[nf].X[7] = 0.0; // Bias

		ekf[nf].dt  = 0.1;  // Initial assumption for time difference between messages (STDMA code runs at 5Hz)
		nf++; 			 	// Number of filter is present is increased
		pthread_mutex_unlock(&ekf_mutex);
	}
	// Else, if we do recognize the ID, then we can update the measurement message data
	else if ((i != -1) || (nf == (NUAVS-1)) )
	{
		rangearray[i] = range; // Store RSSI in array (for logging purposes)

		// Get own velocities
		float ownVx = stateGetSpeedEnu_f()->y; // Velocity North in NED
		float ownVy = stateGetSpeedEnu_f()->x; // Velocity East in NED
		// Bind to realistic amounts to avoid occasional spikes/NaN/inf errors
		keepbounded(&ownVx,-2.0,2.0);
		keepbounded(&ownVy,-2.0,2.0);

		//if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
		if(stateGetPositionEnu_f()->z > 1.0)
		{
		// Make the filter only in Guided mode (flight).
		// This is because it is best for the filter should only start once the drones are in motion, 
		// otherwise it might diverge while drones are not moving.
			
			ekf[i].dt = (get_sys_time_usec() - now_ts[i])/pow(10,6); // Update the time between messages
			pthread_mutex_lock(&ekf_mutex);
			// As for own velocity, bind to realistic amounts to avoid occasional spikes/NaN/inf errors
			keepbounded(&trackedVx,-2.0,2.0);
			keepbounded(&trackedVy,-2.0,2.0);
			
			// Construct measurement vector Y for EKF using the latest data obtained.
			// Y = [RSSI owvVx ownVy trackedVx trackedVy dh], EKF_M = 6 as defined in discreteekf.h
			float Y[EKF_M];
			Y[0] = range; 	        //RSSI measurement
			Y[1] = ownVx; 	   		// Own velocity North (NED frame)
			Y[2] = ownVy;			// Own velocity East  (NED frame)
			Y[3] = trackedVx;  		// Velocity of other drone Norht (NED frame)
			Y[4] = trackedVy;		// Velocity of other drone East  (NED frame)
			Y[5] = trackedh - stateGetPositionEnu_f()->z;  // Height difference
			// Run the steps of the EKF, but only if velocity difference is significant (to filter out minimal noise)		
			ekf_filter_predict(&ekf[i]); // Prediction step of the EKF
			ekf_filter_update(&ekf[i], Y);	// Update step of the EKF
		}
		else
		{
			ekf[i].X[0] = 0.0; // Relative position North
			ekf[i].X[1] = 2.5; // Relative position East
			// The other variables can be initialized at 0
			ekf[i].X[2] = 0.0; // Own Velocity North
			ekf[i].X[3] = 0.0; // Own Velocity East
			ekf[i].X[4] = 0.0; // Velocity other North
			ekf[i].X[5] = 0.0; // Velocity other East
			ekf[i].X[6] = 0.0; // Height difference
			ekf[i].X[7] = 0.0; // Bias
		}
	}
	pthread_mutex_unlock(&ekf_mutex);
	if(RLLOG){
		current_speed = *stateGetSpeedEnu_f();
		current_pos = *stateGetPositionEnu_f();

		if(rlFileLogger!=NULL){
			pthread_mutex_lock(&ekf_mutex);
			fprintf(rlFileLogger,"%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
					counter,
					i,
					(float)(now_ts[i]/pow(10,6)),
					ekf[i].dt,
					current_pos.y,
					current_pos.x,
					-current_pos.z,
					current_speed.y,
					current_speed.x,
					-current_speed.z,
					range,
					trackedVx,
					trackedVy,
					trackedh,
					ekf[i].X[0],
					ekf[i].X[1],
					ekf[i].X[2],
					ekf[i].X[3],
					ekf[i].X[4],
					ekf[i].X[5],
					ekf[i].X[6],
					ekf[i].X[7]);
			counter++;
			pthread_mutex_unlock(&ekf_mutex);
		}
	}
	now_ts[i] = get_sys_time_usec();  // Store latest time
};
//#endif


#ifdef PPRZ_MSG_ID_RLFILTER

static void send_rafilterdata(struct transport_tx *trans, struct link_device *dev)
{
	// To avoid overflowing, it is best to send the data of each tracked drone separately.
	// To do so, we can cycle through the filters at each new timestep.

	cnt++;
	if (cnt >= nf)
		cnt = 0;
	//printf("rangearray[cnt] is: %f, cnt is: %i\n",rangearray[cnt],cnt);

	pprz_msg_send_RLFILTER(
		trans, dev, AC_ID,			 // Standard stuff
		&IDarray[cnt],			     		 // ID of the tracked UAV in question
		&rangearray[cnt], 		    	 // Received ID and RSSI
		&ekf[cnt].X[0], &ekf[cnt].X[1],  // Relative position [North, East]
		&ekf[cnt].X[2], &ekf[cnt].X[3],  // Own velocity [North, East]
		&ekf[cnt].X[4], &ekf[cnt].X[5],  // Relative velocity of other drone [North, East]
		&ekf[cnt].X[6]				 // Height separation [Down]
		);

			 
};
#endif

void relativelocalizationfilter_init(void)
{



	time_t rawtime;
	struct tm * timeinfo;


	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	char time[30];
	strftime(time,sizeof(time),"%Y-%m-%d-%X",timeinfo);

	//printf ( "Current local time and date: %s", asctime (timeinfo) );
	char* temp = rlconcat("/data/ftp/internal_000/rlLogFile_",time);
	char* rlFileName=rlconcat(temp,".txt");


	if(RLLOG){
		rlFileLogger = fopen(rlFileName,"w");
		if (rlFileLogger!=NULL){
			fprintf(rlFileLogger,"msg_count,AC_ID,time,dt,own_x,own_y,own_z,own_vx,own_vy,own_vz,Range,track_vx_meas,track_vy_meas,track_z_meas,kal_x,kal_y,kal_vx,kal_vy,kal_oth_vx,kal_oth_vy,kal_rel_h,kal_bias\n");
		}
	}
	array_make_zeros_int(NUAVS-1, IDarray); // Clear out the known IDs
	nf = 0; // Number of active filters upon initialization
/*
	#ifdef RSSI_LOCALIZATION
	AbiBindMsgRSSI(ABI_BROADCAST, &rssi_ev, bluetoothmsg_cb); // Subscribe to the ABI RSSI messages
	#elseif UWB_LOCALIZATION
	#endif
	*/
	AbiBindMsgUWB(ABI_BROADCAST, &uwb_ev, uwbmsg_cb); // Subscribe to the ABI RSSI messages

	#ifdef PPRZ_MSG_ID_RLFILTER
	cnt = 0;
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RLFILTER, send_rafilterdata); // Send out the filter data
	#endif
};

void relativelocalizationfilter_periodic(void)
{	
	#ifdef RSSI_LOCALIZATION
	/*********************************************
		Sending speed directly between drones
	 *********************************************/
	// Convert course to the proper format (NED)
	float spd, crs;
	cart2polar(stateGetSpeedEnu_f()->y, stateGetSpeedEnu_f()->x, &spd, &crs); // Get the total speed and course
	wrapTo2Pi(&crs); 

	int32_t course = (int32_t)(crs*(1e7)); // Typecast crs into a int32_t type integer with proper unit (see gps.course in gps.h)
	uint32_t multiplex_speed = (((uint32_t)(floor(DeciDegOfRad(course) / 1e7) / 2)) & 0x7FF) <<
			21; 									  // bits 31-21 course (where the magnitude is pointed at)
	multiplex_speed |= (((uint32_t)(spd*100)) & 0x7FF) << 10;         // bits 20-10 speed in cm/s
	multiplex_speed |= (((uint32_t)(-gps.ned_vel.z)) & 0x3FF);        // bits 9-0 z velocity in cm/s
	int16_t alt = (int16_t)(stateGetPositionEnu_f()->z*100.0);
	#endif

	// Use this for communication via the AR drones + Bluetooth dongle
	#ifdef BLUEGIGA_DONGLE
	DOWNLINK_SEND_GPS_SMALL(extra_pprz_tp, EXTRA_DOWNLINK_DEVICE, &multiplex_speed, &gps.lla_pos.lat, &gps.lla_pos.lon, &alt);
	#elif BLUETOOTH_UART
	// Message through USB bluetooth dongle to other drones
	DOWNLINK_SEND_GPS_SMALL(stdma_trans, bluegiga_p, &multiplex_speed, &gps.lla_pos.lat, &gps.lla_pos.lon, &alt);
	#endif
}

char* rlconcat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1)+strlen(s2)+1);//+1 for the zero-terminator
    //in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}
