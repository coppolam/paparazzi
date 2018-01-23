#include "uwb_logger.h"
#include <pthread.h>
#include <time.h>
#include "subsystems/abi.h"
//#include <stdlib.h>
#include <stdio.h>
//#include <stdint.h>
#include <stdbool.h>
//#include <pthread.h>
#include "state.h"
#include "generated/flight_plan.h"        // reference lla NAV_XXX0
#include "../../math/pprz_algebra.h"

//Fixed point conversions
#include "../../math/pprz_algebra.h"
#include "../../math/pprz_algebra_int.h"

#ifndef UWB_LOGGER_GPS_ID
#define UWB_LOGGER_GPS_ID GPS_MULTI_ID
#endif
#ifndef UWB_LOGGER_OPTICFLOW_ID
#define UWB_LOGGER_OPTICFLOW_ID 1
#endif


char* uwbstrconcat(const char *s1, const char *s2);


#define UWB_LOGGER true
static FILE *UWBFileLogger = NULL;

static abi_event uwb_log_ev;
static abi_event uwb_gps_ev;
static abi_event uwb_opticflow_ev;

//static pthread_mutex_t uwb_logger_mutex;

static struct LtpDef_i ltp_def;

static struct NedCoor_i gps_ned_vel_cm_s;      ///< speed NED in cm/s
static struct NedCoor_f gps_ned_vel_cm_s_f;      ///< speed NED in cm/s
static struct NedCoor_i gps_ned_pos_cm;
static struct NedCoor_i gps_ned_pos_cm_f;

static struct FloatVect3 optic_vel_m_s_f;

void uwb_logger_init(void){
	struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
	llh_nav0.lat = NAV_LAT0;
	llh_nav0.lon = NAV_LON0;
	/* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
	llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

	ltp_def_from_lla_i(&ltp_def, &llh_nav0);

	AbiBindMsgUWB_NDI(ABI_BROADCAST, &uwb_log_ev, logEvent);
	AbiBindMsgGPS(UWB_LOGGER_GPS_ID, &uwb_gps_ev, uwb_gps_cb);
	AbiBindMsgVELOCITY_ESTIMATE(UWB_LOGGER_OPTICFLOW_ID, &uwb_opticflow_ev, uwb_optic_vel_cb);
	if(UWB_LOGGER){
		time_t rawtime;
		struct tm * timeinfo;


		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		char time[30];
		strftime(time,sizeof(time),"%Y-%m-%d-%X",timeinfo);

		//printf ( "Current local time and date: %s", asctime (timeinfo) );
		char* temp = uwbstrconcat("/data/ftp/internal_000/UWBLogFile_",time);
		char* UWBFileName=uwbstrconcat(temp,".txt");
		UWBFileLogger = fopen(UWBFileName,"w");
		if (UWBFileLogger!=NULL){
			fprintf(UWBFileLogger,"msg_count,time,dt,"
					"state_x,state_y,state_z,"
					"state_vx,state_vy,state_vz,"
					"state_ax,state_ay,state_az,"
					"state_phi,state_theta,state_psi,"
					"state_p,state_q,state_r,"
					"gps_x,gps_y,gps_z,"
					"gps_vx,gps_vy,gps_vz,"
					"optic_vx,optic_vy,optic_vz,"
					"Range,track_vx_meas,track_vy_meas,track_z_meas,"
					"kal_x,kal_y,kal_h1,kal_h2,kal_u1,kal_v1,kal_u2,kal_v2,kal_gamma\n");
		}
	}

}

void logEvent(uint8_t sender_id __attribute__((unused)),float time, float dt,float range, float trackedVx, float trackedVy, float trackedh, float xin, float yin, float h1in, float h2in, float u1in, float v1in, float u2in, float v2in, float gammain){
	static int counter = 0;
	struct EnuCoor_f current_speed = *stateGetSpeedEnu_f();
	struct EnuCoor_f current_pos = *stateGetPositionEnu_f();
	struct NedCoor_f current_accel = *stateGetAccelNed_f();
	struct FloatRates current_rates = *stateGetBodyRates_f();
	struct FloatEulers current_eulers = *stateGetNedToBodyEulers_f();
	if(UWB_LOGGER){


		if(UWBFileLogger!=NULL){
			//pthread_mutex_lock(&uwb_logger_mutex);
			fprintf(UWBFileLogger,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
					counter,
					time,
					dt,
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
					gps_ned_pos_cm_f.x/100.f,
					gps_ned_pos_cm_f.y/100.f,
					gps_ned_pos_cm_f.z/100.f,
					gps_ned_vel_cm_s_f.x/100.f,
					gps_ned_vel_cm_s_f.y/100.f,
					gps_ned_vel_cm_s_f.z/100.f,
					optic_vel_m_s_f.x,
					optic_vel_m_s_f.y,
					optic_vel_m_s_f.z,
					range,
					trackedVx,
					trackedVy,
					trackedh,
					xin,
					yin,
					h1in,
					h2in,
					u1in,
					v1in,
					u2in,
					v2in,
					gammain);
			//pthread_mutex_unlock(&uwb_logger_mutex);
			counter++;
		}

	}


}

extern void uwb_gps_cb(uint8_t sender_id __attribute__((unused)),uint32_t time, struct GpsState *gps_s){
	gps_ned_vel_cm_s = gps_s->ned_vel;
	VECT3_ASSIGN(gps_ned_vel_cm_s_f,gps_ned_vel_cm_s.x,gps_ned_vel_cm_s.y,gps_ned_vel_cm_s.z);
	ned_of_ecef_point_i(&gps_ned_pos_cm, &ltp_def, &gps_s->ecef_pos);
	VECT3_ASSIGN(gps_ned_pos_cm_f,gps_ned_pos_cm.x,gps_ned_pos_cm.y,gps_ned_pos_cm.z);
}

void uwb_optic_vel_cb(uint8_t sender_id __attribute__((unused)),uint32_t time,float vx, float vy, float vz, float noise){
	VECT3_ASSIGN(optic_vel_m_s_f,vx,vy,vz);
}

char* uwbstrconcat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1)+strlen(s2)+1);//+1 for the zero-terminator
    //in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}
