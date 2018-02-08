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
// Butterworth filter
#include "filters/low_pass_filter.h"
#include "../../boards/bebop/actuators.h"
#if UWB_LOG_NDI
#include "../uwb_control/uwb_follower_control.h"
#endif
#include "../../subsystems/ins/ins_int.h"

#ifndef UWB_LOGGER_GPS_ID
#define UWB_LOGGER_GPS_ID GPS_MULTI_ID
#endif
#ifndef UWB_LOGGER_OPTICFLOW_ID
#define UWB_LOGGER_OPTICFLOW_ID 1
#endif

#define UWB_LOWPASS_CUTOFF_FREQUENCY_YAWR 8
#define UWB_LOWPASS_CUTOFF_FREQUENCY_AX 8
#define UWB_LOWPASS_CUTOFF_FREQUENCY_AY 8


Butterworth2LowPass uwb_butter_yawr;
Butterworth2LowPass uwb_butter_ax;
Butterworth2LowPass uwb_butter_ay;


char* uwbstrconcat(const char *s1, const char *s2);

#define NUM_UWB_LOGGERS 2
#define UWB_LOGGER true
static FILE *UWBFileLogger[NUM_UWB_LOGGERS];

static abi_event uwb_log_ev;
static abi_event uwb_gps_ev;
static abi_event uwb_opticflow_ev;
static abi_event uwb_sonar_ev;

//static pthread_mutex_t uwb_logger_mutex;

static struct LtpDef_i ltp_def;

struct NedCoor_i uwb_gps_ned_vel_cm_s;      ///< speed NED in cm/s
struct NedCoor_f uwb_gps_ned_vel_cm_s_f;      ///< speed NED in cm/s
struct NedCoor_i uwb_gps_ned_pos_cm;
struct NedCoor_i uwb_gps_ned_pos_cm_f;

struct FloatVect3 uwb_optic_vel_m_s_f;

float uwb_sonarheight = 0.0;
float uwb_baroheight = 0.0;

float uwb_smooth_yawr = 0.0;
float uwb_smooth_ax = 0.0;
float uwb_smooth_ay = 0.0;




void uwb_logger_init(void){
	init_butterworth_2_low_pass(&uwb_butter_yawr, UWB_LOWPASS_CUTOFF_FREQUENCY_YAWR, 1./PERIODIC_FREQUENCY, 0.0);
	init_butterworth_2_low_pass(&uwb_butter_ax, UWB_LOWPASS_CUTOFF_FREQUENCY_AX, 1./PERIODIC_FREQUENCY, 0.0);
	init_butterworth_2_low_pass(&uwb_butter_ay, UWB_LOWPASS_CUTOFF_FREQUENCY_AY, 1./PERIODIC_FREQUENCY, 0.0);
	struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
	llh_nav0.lat = NAV_LAT0;
	llh_nav0.lon = NAV_LON0;
	/* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
	llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

	ltp_def_from_lla_i(&ltp_def, &llh_nav0);

	AbiBindMsgUWB_NDI(ABI_BROADCAST, &uwb_log_ev, logEvent);
	AbiBindMsgGPS(UWB_LOGGER_GPS_ID, &uwb_gps_ev, uwb_gps_cb);
	AbiBindMsgVELOCITY_ESTIMATE(UWB_LOGGER_OPTICFLOW_ID, &uwb_opticflow_ev, uwb_optic_vel_cb);
	AbiBindMsgAGL(ABI_BROADCAST, &uwb_sonar_ev,uwb_sonar_height_cb);
	if(UWB_LOGGER){
		for(int i=0; i<NUM_UWB_LOGGERS;i++){
			time_t rawtime;
			struct tm * timeinfo;


			time ( &rawtime );
			timeinfo = localtime ( &rawtime );
			char time[30];
			strftime(time,sizeof(time),"%Y-%m-%d-%X",timeinfo);

			char buf[100];
			sprintf(buf,"/data/ftp/internal_000/UWBLogFile_%d_%s",i,time);

			char *UWBFileName = &buf;
			UWBFileLogger[i] = fopen(UWBFileName,"w");
			fprintf(UWBFileLogger[i],"msg_count,time,dt,"
					"state_x,state_y,state_z,"
					"state_vx,state_vy,state_vz,"
					"state_ax,state_ay,state_az,"
					"smooth_ax,smooth_ay,"
					"state_phi,state_theta,state_psi,"
					"state_p,state_q,state_r,"
					"smooth_r,"
					"gps_x,gps_y,gps_z,"
					"gps_vx,gps_vy,gps_vz,"
					"optic_vx,optic_vy,optic_vz,"
					"sonar_z,baro_z,"
					"Range,track_vx,track_vy,track_z,track_ax,track_ay,track_r,"
					"kal_x,kal_y,kal_h1,kal_h2,kal_u1,kal_v1,kal_u2,kal_v2,kal_gamma,"
					"rpm1,rpm2,rpm3,rpm4,rpm_ref1,rpm_ref2,rpm_ref3,rpm_ref4"
#if UWB_LOG_NDI
					",vcom1,vcom2,vcom1_cap,vcom2_cap"
#endif
					"\n");

		}
	}

}

void logEvent(uint8_t sender_id __attribute__((unused)),uint8_t ac_id, float time, float dt,float range, float trackedVx, float trackedVy, float trackedh, float trackedAx,float trackedAy,float trackedYawr,  float xin, float yin, float h1in, float h2in, float u1in, float v1in, float u2in, float v2in, float gammain){
	static int counter = 0;
	struct EnuCoor_f current_speed = *stateGetSpeedEnu_f();
	struct EnuCoor_f current_pos = *stateGetPositionEnu_f();
	struct NedCoor_f current_accel = *stateGetAccelNed_f();
	struct FloatRates current_rates = *stateGetBodyRates_f();
	struct FloatEulers current_eulers = *stateGetNedToBodyEulers_f();
	if(UWB_LOGGER){


		//pthread_mutex_lock(&uwb_logger_mutex);
		fprintf(UWBFileLogger[ac_id],"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"
#if UWB_LOG_NDI
				",%f,%f,%f,%f"
#endif
				"\n",
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
				uwb_smooth_ax,
				uwb_smooth_ay,
				current_eulers.phi,
				current_eulers.theta,
				current_eulers.psi,
				current_rates.p,
				current_rates.q,
				current_rates.r,
				uwb_smooth_yawr,
				uwb_gps_ned_pos_cm_f.x/100.f,
				uwb_gps_ned_pos_cm_f.y/100.f,
				uwb_gps_ned_pos_cm_f.z/100.f,
				uwb_gps_ned_vel_cm_s_f.x/100.f,
				uwb_gps_ned_vel_cm_s_f.y/100.f,
				uwb_gps_ned_vel_cm_s_f.z/100.f,
				uwb_optic_vel_m_s_f.x,
				uwb_optic_vel_m_s_f.y,
				uwb_optic_vel_m_s_f.z,
				-uwb_sonarheight,
				-ins_int.baro_z,
				range,
				trackedVx,
				trackedVy,
				trackedh,
				trackedAx,
				trackedAy,
				trackedYawr,
				xin,
				yin,
				h1in,
				h2in,
				u1in,
				v1in,
				u2in,
				v2in,
				gammain,
				(float)actuators_bebop.rpm_obs[0],
				(float)actuators_bebop.rpm_obs[1],
				(float)actuators_bebop.rpm_obs[2],
				(float)actuators_bebop.rpm_obs[3],
				(float)actuators_bebop.rpm_ref[0],
				(float)actuators_bebop.rpm_ref[1],
				(float)actuators_bebop.rpm_ref[2],
				(float)actuators_bebop.rpm_ref[3]
#if UWB_LOG_NDI
,ndihandle.commands[0],
ndihandle.commands[1],
ndihandle.commandscap[0],
ndihandle.commandscap[1]
#endif
		);
		//pthread_mutex_unlock(&uwb_logger_mutex);
		counter++;


	}


}

void uwb_sonar_height_cb(uint8_t __attribute__((unused)) sender_id, float distance){
	uwb_sonarheight = distance;
}

extern void uwb_gps_cb(uint8_t sender_id __attribute__((unused)),uint32_t time, struct GpsState *gps_s){
	uwb_gps_ned_vel_cm_s = gps_s->ned_vel;
	VECT3_ASSIGN(uwb_gps_ned_vel_cm_s_f,uwb_gps_ned_vel_cm_s.x,uwb_gps_ned_vel_cm_s.y,uwb_gps_ned_vel_cm_s.z);
	ned_of_ecef_point_i(&uwb_gps_ned_pos_cm, &ltp_def, &gps_s->ecef_pos);
	VECT3_ASSIGN(uwb_gps_ned_pos_cm_f,uwb_gps_ned_pos_cm.x,uwb_gps_ned_pos_cm.y,uwb_gps_ned_pos_cm.z);
}

void uwb_optic_vel_cb(uint8_t sender_id __attribute__((unused)),uint32_t time,float vx, float vy, float vz, float noise){
	VECT3_ASSIGN(uwb_optic_vel_m_s_f,vx,vy,vz);
}

void uwb_logger_event(void){
	//uwb_smooth_ax = update_butterworth_2_low_pass(&uwb_butter_ax,stateGetAccelNed_f()->x);
	//uwb_smooth_ax = update_butterworth_2_low_pass(&uwb_butter_ay,stateGetAccelNed_f()->y);
	uwb_smooth_ax = stateGetAccelNed_f()->x;
	uwb_smooth_ay = stateGetAccelNed_f()->y;
	uwb_smooth_yawr = update_butterworth_2_low_pass(&uwb_butter_yawr,stateGetBodyRates_f()->r);
}

char* uwbstrconcat(const char *s1, const char *s2)
{
	char *result = malloc(strlen(s1)+strlen(s2)+1);//+1 for the zero-terminator
	//in real code you would check for errors in malloc here
	strcpy(result, s1);
	strcat(result, s2);
	return result;
}
