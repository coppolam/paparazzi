/*
 * Copyright (C) Steven van der Helm
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
 * @file "modules/relativelocalizationfilter/relative_localization_no_north.c"
 * @author Steven van der Helm
 * Range based relative localization filter without north dependency
 */

#include <time.h>
#include <pthread.h>
#include "relative_localization_no_north.h"
#include "fmatrix.h"
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
struct NedCoor_f current_accel;
struct FloatRates current_rates;
struct FloatEulers current_eulers;
int counter = 0;

//char rlFileName[50] = "/data/ftp/internal_000/rlLogFile1.csv";

/** The file pointer */
static FILE *rlFileLogger = NULL;
char* rlconcat(const char *s1, const char *s2);

static pthread_mutex_t ekf_mutex;

#define RLLOG 1

PRINT_CONFIG_VAR(EKF_XZERO)

void initNewEkfFilter(ekf_filter *filter){
	float Pval[EKF_N] = {16,16,16,16,16,16,16,16,16};
	// inputs: a1x, a1y, a2x, a2y, r1, r2
	float Qval[EKF_L] = {pow(2,2),pow(2,2),pow(2,2),pow(2,2),pow(0.05,2),pow(0.05,2)};
	// measurements: range, h1, h2, u1, v1, u2, v2
	float Rval[EKF_M] = {pow(0.1,2),pow(0.1,2),pow(0.1,2),pow(0.2,2),pow(0.2,2),pow(0.2,2),pow(0.2,2)};
	fmat_make_zeros(filter->X,EKF_N,1);
	filter->X[0]=EKF_XZERO;
	filter->X[1]=EKF_YZERO;
	filter->X[2]=-1;
	filter->X[3]=-1;
	fmat_make_diagonal(EKF_N,filter->P,Pval);
	fmat_make_diagonal(EKF_L,filter->Q,Qval);
	fmat_make_diagonal(EKF_M,filter->R,Rval);
	filter->dt = 0.1;


}



int cnt;

static abi_event uwb_ev;
static void uwbmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh);

static void uwbmsg_cb(uint8_t sender_id __attribute__((unused)), 
	uint8_t ac_id, float range, float trackedVx, float trackedVy, float trackedh) 
{
	static float oldtime = 0.0;




	int i = -1; // Initialize the index of all tracked drones (-1 for null assumption of no drone found).

	// Check if a new aircraft ID is present, if it's a new ID we start a new EKF for it.
	if (( !array_find_int(NUAVS-1, IDarray, ac_id, &i))  // If yes, a new drone is found.
		   && (nf < NUAVS-1))  // If yes, the amount of drones does not exceed the maximum.
	{
		pthread_mutex_lock(&ekf_mutex);
		IDarray[nf] = ac_id; 				// Store ID in an array (logging purposes)
		ekf_filter_new(&ekf[nf]); 			// Initialize an EKF filter for the newfound drone

		initNewEkfFilter(&ekf[nf]);
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
		float ownh = stateGetPositionEnu_f()->z;
		// Bind to realistic amounts to avoid occasional spikes/NaN/inf errors
		keepbounded(&ownVx,-2.0,2.0);
		keepbounded(&ownVy,-2.0,2.0);

		//if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
		if(ownh > 0.5)
		{
		// Make the filter only in Guided mode (flight).
		// This is because it is best for the filter should only start once the drones are in motion, 
		// otherwise it might diverge while drones are not moving.
			float input[EKF_L] = {0,0,0,0,0,0};
			float measurements[EKF_M] = {range,-ownh,trackedh,ownVx,ownVy,trackedVx,trackedVy};
			float dt = (get_sys_time_usec() - now_ts[i])/pow(10,6); // Update the time between messages
			updateEkfFilter(&ekf[i],input,measurements,dt);
		}
		else
		{
			/*
			ekf[i].X[0] = 0.0; // Relative position North
			ekf[i].X[1] = 2.5; // Relative position East
			// The other variables can be initialized at 0
			ekf[i].X[2] = 0.0; // Own Velocity North
			ekf[i].X[3] = 0.0; // Own Velocity East
			ekf[i].X[4] = 0.0; // Velocity other North
			ekf[i].X[5] = 0.0; // Velocity other East
			ekf[i].X[6] = 0.0; // Height difference
			//ekf[i].X[7] = 0.0; // Bias
			*/
		}
	}
	pthread_mutex_unlock(&ekf_mutex);
	if(RLLOG){
		current_speed = *stateGetSpeedEnu_f();
		current_pos = *stateGetPositionEnu_f();
		current_accel = *stateGetAccelNed_f();
		current_rates = *stateGetBodyRates_f();
		current_eulers = *stateGetNedToBodyEulers_f();

		if(rlFileLogger!=NULL){
			pthread_mutex_lock(&ekf_mutex);
			fprintf(rlFileLogger,"%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
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
					current_accel.x,
					current_accel.y,
					current_accel.z,
					current_eulers.phi,
					current_eulers.theta,
					current_eulers.psi,
					current_rates.p,
					current_rates.q,
					current_rates.r,
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
					ekf[i].X[7],
					ekf[i].X[8],
					ekf[i].X[9]);
			counter++;
			pthread_mutex_unlock(&ekf_mutex);
		}
	}
	now_ts[i] = get_sys_time_usec();  // Store latest time
	if(((get_sys_time_usec()/pow(10,6))-oldtime)>1){
		oldtime = get_sys_time_usec()/pow(10,6);
		printf("Current estimate of EKF is x,y,z1,z2,u1,v1,u2,v2,gam: %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
				ekf[i].X[0],
				ekf[i].X[1],
				ekf[i].X[2],
				ekf[i].X[3],
				ekf[i].X[4],
				ekf[i].X[5],
				ekf[i].X[6],
				ekf[i].X[7],
				ekf[i].X[8],
				ekf[i].X[9]);
	}
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
			fprintf(rlFileLogger,"msg_count,AC_ID,time,dt,own_x,own_y,own_z,own_vx,own_vy,own_vz,own_ax,own_ay,own_az,own_phi,own_theta,own_psi,own_p,own_q,own_r,Range,track_vx_meas,track_vy_meas,track_z_meas,kal_x,kal_y,kal_h1,kal_h2,kal_u1,kal_v1,kal_u2,kal_v2,kal_gamma\n");
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
