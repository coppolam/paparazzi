#include <time.h>
#include <math.h>
#include "uwb_follower_control.h"
#include "../relativelocalizationfilter/fmatrix.h"
#include <pthread.h>
#include "../../firmwares/rotorcraft/guidance/guidance_h.h"
#include "../../firmwares/rotorcraft/guidance/guidance_v.h"
#include "generated/flight_plan.h"
#include "../../firmwares/rotorcraft/autopilot_guided.h"
#include "navigation.h"
#include "../relativelocalizationfilter/relative_localization_no_north.h"
#include "../loggers/uwb_logger.h"

#define NDI_METHOD 1
// method 0 is first order approximation, no acceleration or yaw rate used
// method 1 is first order approximation, acceleration and yaw rate used, but yaw rate not taken into account in integral

#define NDI_MOST_RECENT ndihandle.data_entries-1

#define UWB_NDI_LOGGER false
static FILE *NDIFileLogger = NULL;


bool ndi_following_leader = false;
bool ndi_run_computation = true;

static pthread_mutex_t uwb_ndi_mutex;

void cleanNdiValues(float tcur);
float accessCircularFloatArrElement(float *arr, int index);
float computeNdiFloatIntegral(float* ndiarr, float curtime);
void bindNorm(void);
char* strconcat(const char *s1, const char *s2);

ndihandler ndihandle = {.delay = 5,
		.tau_x=3,
		.tau_y = 3,
		.Kp = -1.5,
		.Ki = 0,
		.Kd = -3,
		.data_start = 0,
		.data_end = 0,
		.data_entries = 0,
		.maxcommand = 1.5};

static abi_event uwb_ndi_ev;

extern void uwb_ndi_follower_init(void){
	AbiBindMsgUWB_NDI(ABI_BROADCAST, &uwb_ndi_ev, addNdiValues);
	if(UWB_NDI_LOGGER){
		time_t rawtime;
		struct tm * timeinfo;


		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		char time[30];
		strftime(time,sizeof(time),"%Y-%m-%d-%X",timeinfo);

		//printf ( "Current local time and date: %s", asctime (timeinfo) );
		char* temp = strconcat("/data/ftp/internal_000/NDILogFile_",time);
		char* NDIFileName=strconcat(temp,".txt");
		NDIFileLogger = fopen(NDIFileName,"w");
		if (NDIFileLogger!=NULL){
			fprintf(NDIFileLogger,"msg_count,time,dt,own_x,own_y,own_z,own_vx,own_vy,own_vz,own_ax,own_ay,own_az,own_phi,own_theta,own_psi,own_p,own_q,own_r,Range,track_vx_meas,track_vy_meas,track_z_meas,kal_x,kal_y,kal_h1,kal_h2,kal_u1,kal_v1,kal_u2,kal_v2,kal_gamma,vcom1,vcom2,vcom1_cap,vcom2_cap\n");
		}
	}
}


void addNdiValues(uint8_t sender_id __attribute__((unused)),float time, float dt,float range, float trackedVx, float trackedVy, float trackedh, float trackedAx, float trackedAy, float trackedYawr, float xin, float yin, float h1in, float h2in, float u1in, float v1in, float u2in, float v2in, float gammain){

	struct EnuCoor_f current_speed = *stateGetSpeedEnu_f();
	struct EnuCoor_f current_pos = *stateGetPositionEnu_f();
	struct NedCoor_f current_accel = *stateGetAccelNed_f();
	struct FloatRates current_rates = *stateGetBodyRates_f();
	struct FloatEulers current_eulers = *stateGetNedToBodyEulers_f();
	static int counter = 0;
	float t = time;
	pthread_mutex_lock(&uwb_ndi_mutex);
	if(ndihandle.data_entries==NDI_PAST_VALS){
		ndihandle.data_entries--;
		ndihandle.data_start = (ndihandle.data_start+1)%NDI_PAST_VALS;
	}
	ndihandle.xarr[ndihandle.data_end] = xin;
	ndihandle.yarr[ndihandle.data_end] = yin;
	ndihandle.u1arr[ndihandle.data_end] = u1in;
	ndihandle.v1arr[ndihandle.data_end] = v1in;
	ndihandle.u2arr[ndihandle.data_end] = u2in;
	ndihandle.v2arr[ndihandle.data_end] = v2in;
	ndihandle.r1arr[ndihandle.data_end] = uwb_smooth_yawr;
	ndihandle.r2arr[ndihandle.data_end] = trackedYawr;
	ndihandle.ax1arr[ndihandle.data_end] = uwb_smooth_ax;
	ndihandle.ay1arr[ndihandle.data_end] = uwb_smooth_ay;
	ndihandle.ax2arr[ndihandle.data_end] = trackedAx;
	ndihandle.ay2arr[ndihandle.data_end] = trackedAy;
	ndihandle.gamarr[ndihandle.data_end] = gammain;
	ndihandle.tarr[ndihandle.data_end] = t;
	ndihandle.data_end = (ndihandle.data_end+1)%NDI_PAST_VALS;
	ndihandle.data_entries++;
	if(UWB_NDI_LOGGER){


		if(NDIFileLogger!=NULL){
			fprintf(NDIFileLogger,"%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
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
					gammain,
					ndihandle.commands[0],
					ndihandle.commands[1],
					ndihandle.commandscap[0],
					ndihandle.commandscap[1]);
			counter++;
		}

	}
	pthread_mutex_unlock(&uwb_ndi_mutex);
}

void cleanNdiValues(float tcur){
	pthread_mutex_lock(&uwb_ndi_mutex);
	int curentries = ndihandle.data_entries;
	for(int i=0;i<curentries;i++){
		if((tcur-ndihandle.tarr[ndihandle.data_start])>ndihandle.delay){
			ndihandle.data_start = (ndihandle.data_start+1)%NDI_PAST_VALS;
			ndihandle.data_entries--;

		}
	}
	pthread_mutex_unlock(&uwb_ndi_mutex);
	return;
}

extern void printCircularFloatArr(float* arr){
	printf("Array: {");
	for(int i=0; i<ndihandle.data_entries;i++){
		printf("%.1f, ",accessCircularFloatArrElement(arr,i));
	}
	printf("}\n");
}

float accessCircularFloatArrElement(float *arr, int index){
	float value;
	pthread_mutex_lock(&uwb_ndi_mutex);
	int realindex = (ndihandle.data_start+index)%NDI_PAST_VALS;
	value = arr[realindex];
	pthread_mutex_unlock(&uwb_ndi_mutex);
	return value;
}

bool startNdiTracking(void){
	ndi_following_leader = true;
	return false;
}

bool stopNdiTracking(void){
	ndi_following_leader = false;
	return false;
}


//TODO check behavior when array is empty!
void calcNdiCommands(void){
	if(ndi_run_computation){
		float curtime = get_sys_time_usec()/pow(10,6);
		cleanNdiValues(curtime);
		if(ndihandle.data_entries>0){
#if(NDI_METHOD==0)
			float oldx = accessCircularFloatArrElement(ndihandle.xarr,0);
			float oldy = accessCircularFloatArrElement(ndihandle.yarr,0);

			//float oldu1 = accessCircularFloatArrElement(ndihandle.u1arr,0);
			//float oldv1 = accessCircularFloatArrElement(ndihandle.v1arr,0);
			//struct EnuCoor_f current_speed = *stateGetSpeedEnu_f();
			//float newu1 = current_speed.y;
			//float newv1 = current_speed.x;
			float newu1 = accessCircularFloatArrElement(ndihandle.u1arr,NDI_MOST_RECENT);
			float newv1 = accessCircularFloatArrElement(ndihandle.v1arr,NDI_MOST_RECENT);
			float oldu2 = accessCircularFloatArrElement(ndihandle.u2arr,0);
			float oldv2 = accessCircularFloatArrElement(ndihandle.v2arr,0);
			oldx = oldx - computeNdiFloatIntegral(ndihandle.u1arr,curtime);
			oldy = oldy - computeNdiFloatIntegral(ndihandle.v1arr,curtime);

			float Minv[4];
			fmat_make_zeros(Minv,2,2);
			fmat_assign(0,0,2,Minv,-ndihandle.tau_x);
			fmat_assign(1,1,2,Minv,-ndihandle.tau_y);

			float l[2];
			/*fmat_make_zeros(l,2,1);
			fmat_assign(0,0,1,l,newu1/ndihandle.tau_x);
			fmat_assign(1,0,1,l,newv1/ndihandle.tau_y);*/
			l[0]=newu1/ndihandle.tau_x;
			l[1]=newv1/ndihandle.tau_y;

			float oldxed = oldu2 - newu1;
			float oldyed = oldv2 - newv1;

			float v[2];
			v[0] = ndihandle.Kp * oldx + ndihandle.Kd * oldxed;
			v[1] = ndihandle.Kp * oldy + ndihandle.Kd * oldyed;

			float sig[2];
			sig[0] = v[0]-l[0];
			sig[1] = v[1]-l[1];

			fmat_mult(2,2,1,ndihandle.commands,Minv,sig);
			bindNorm();
#else if(NDI_METHOD == 1)
	float oldx = accessCircularFloatArrElement(ndihandle.xarr,0);
	float oldy = accessCircularFloatArrElement(ndihandle.yarr,0);

	//float oldu1 = accessCircularFloatArrElement(ndihandle.u1arr,0);
	//float oldv1 = accessCircularFloatArrElement(ndihandle.v1arr,0);
	//struct EnuCoor_f current_speed = *stateGetSpeedEnu_f();
	//float newu1 = current_speed.y;
	//float newv1 = current_speed.x;
	float newu1 = accessCircularFloatArrElement(ndihandle.u1arr,NDI_MOST_RECENT);
	float newv1 = accessCircularFloatArrElement(ndihandle.v1arr,NDI_MOST_RECENT);
	float newax1 = accessCircularFloatArrElement(ndihandle.ax1arr,NDI_MOST_RECENT);
	float neway1 = accessCircularFloatArrElement(ndihandle.ay1arr,NDI_MOST_RECENT);
	float newr1 = accessCircularFloatArrElement(ndihandle.r1arr,NDI_MOST_RECENT);
	float oldu2 = accessCircularFloatArrElement(ndihandle.u2arr,0);
	float oldv2 = accessCircularFloatArrElement(ndihandle.v2arr,0);
	float oldax2 = accessCircularFloatArrElement(ndihandle.ax2arr,0);
	float olday2 = accessCircularFloatArrElement(ndihandle.ay2arr,0);
	float oldr2 = accessCircularFloatArrElement(ndihandle.r2arr,0);
	oldx = oldx - computeNdiFloatIntegral(ndihandle.u1arr,curtime);
	oldy = oldy - computeNdiFloatIntegral(ndihandle.v1arr,curtime);

	float Minv[4];
	fmat_make_zeros(Minv,2,2);
	fmat_assign(0,0,2,Minv,-ndihandle.tau_x);
	fmat_assign(1,1,2,Minv,-ndihandle.tau_y);

	float l[2];
	/*fmat_make_zeros(l,2,1);
				fmat_assign(0,0,1,l,newu1/ndihandle.tau_x);
				fmat_assign(1,0,1,l,newv1/ndihandle.tau_y);*/
	//l[0]=newu1/ndihandle.tau_x;
	//l[1]=newv1/ndihandle.tau_y;
	l[0] = (newu1-newr1*newr1*oldx - newr1*ndihandle.tau_x*newv1+oldax2*ndihandle.tau_x + 2 * newr1*ndihandle.tau_x*oldv2)/ndihandle.tau_x;
	l[1] = (newv1 - newr1*newr1*oldy + newr1*ndihandle.tau_y*newu1 + olday2*ndihandle.tau_y - 2*newr1*ndihandle.tau_y*oldu2)/ndihandle.tau_y;

	float oldxed = oldu2 - newu1;
	float oldyed = oldv2 - newv1;

	float v[2];
	v[0] = ndihandle.Kp * oldx + ndihandle.Kd * oldxed;
	v[1] = ndihandle.Kp * oldy + ndihandle.Kd * oldyed;

	float sig[2];
	sig[0] = v[0]-l[0];
	sig[1] = v[1]-l[1];

	fmat_mult(2,2,1,ndihandle.commands,Minv,sig);
	bindNorm();

#endif
		}
		else{
			pthread_mutex_lock(&uwb_ndi_mutex);
			ndihandle.commands[0] = 0;
			ndihandle.commands[1] = 0;
			pthread_mutex_unlock(&uwb_ndi_mutex);
		}

	}
	else{
		pthread_mutex_lock(&uwb_ndi_mutex);
		ndihandle.commands[0] = 0;
		ndihandle.commands[1] = 0;
		pthread_mutex_unlock(&uwb_ndi_mutex);
	}
}

/*bool ndi_follow_init(void){
	uwb_use_gps = false;
	uwb_use_opticflow = true;
	uwb_use_sonar = true;
	uwb_send_onboard = true;
	return false;
}*/

//TODO: Trapeoidal integration, and not simply discarding values before tcur-delay, but linearly interpolating to tcur-delay
float computeNdiFloatIntegral(float* ndiarr, float curtime){
	float integval = 0;
	float dt;
	for(int i=0; i<ndihandle.data_entries-1;i++){
		dt = accessCircularFloatArrElement(ndihandle.tarr,i+1)-accessCircularFloatArrElement(ndihandle.tarr,i);
		integval += dt * accessCircularFloatArrElement(ndiarr,i);
	}
	dt = curtime - accessCircularFloatArrElement(ndihandle.tarr,NDI_MOST_RECENT);
	integval += dt * accessCircularFloatArrElement(ndiarr,NDI_MOST_RECENT);

	return integval;
}

void bindNorm(void){
	pthread_mutex_lock(&uwb_ndi_mutex);
	float normcom = sqrt(ndihandle.commands[1]*ndihandle.commands[1] + ndihandle.commands[0]*ndihandle.commands[0]);
	if(normcom>ndihandle.maxcommand){
		ndihandle.commandscap[0] = ndihandle.commands[0] * ndihandle.maxcommand/normcom;
		ndihandle.commandscap[1] = ndihandle.commands[1] * ndihandle.maxcommand/normcom;
	}
	else{
		ndihandle.commandscap[0] = ndihandle.commands[0];
		ndihandle.commandscap[1] = ndihandle.commands[1];
	}
	pthread_mutex_unlock(&uwb_ndi_mutex);

}

void printNdiVars(void){
	/*
	printf("xe "); printCircularFloatArr(ndihandle.xarr);
	printf("ye "); printCircularFloatArr(ndihandle.yarr);
	printf("u1 "); printCircularFloatArr(ndihandle.u1arr);
	printf("v1 "); printCircularFloatArr(ndihandle.v1arr);
	printf("u2 "); printCircularFloatArr(ndihandle.u2arr);
	printf("v2 "); printCircularFloatArr(ndihandle.v2arr);
	printf("t "); printCircularFloatArr(ndihandle.tarr);*/
	printf("NDI commands: %f, %f\n",ndihandle.commands[0],ndihandle.commands[1]);
}

bool ndi_follow_leader(void){
	bool temp = true;
	temp &= guidance_v_set_guided_z(-NDI_FLIGHT_HEIGHT);
	temp &= guidance_h_set_guided_vel(ndihandle.commands[0],ndihandle.commands[1]);
	return !temp;
}







#define TRAJ_EPS 0.5
#define TRAJ_LENGTH 4

uint8_t traj_targetindex = 0;

//static uint8_t trajectory[TRAJ_LENGTH] = {WP_t0,WP_t1,WP_t2,WP_t3};
static uint8_t trajectory[TRAJ_LENGTH] = {WP_t3,WP_t2,WP_t1,WP_t0};

void findNearestWaypoint(void);
void getNextTargetWaypoint(void);

bool initialiseTrajectory(void){
	findNearestWaypoint();
	return false; // false means in the flight plan that the function executed succesfully.
}

bool flyTrajectory(void){
	getNextTargetWaypoint();
	bool temp = true;
	float eastcoord = waypoint_get_x(trajectory[traj_targetindex]);
	float northcoord = waypoint_get_y(trajectory[traj_targetindex]);
	temp &= autopilot_guided_goto_ned(northcoord,eastcoord,-NDI_FLIGHT_HEIGHT,0);
	return false; // False means in the flight plan that the function executed succesfully
}

void getNextTargetWaypoint(void){
	//printf("target waypoint id %d, distance: %f\n", traj_targetindex,get_dist2_to_waypoint(trajectory[traj_targetindex]));
	if(get_dist2_to_waypoint(trajectory[traj_targetindex])<TRAJ_EPS*TRAJ_EPS){
		traj_targetindex = (traj_targetindex+1)%TRAJ_LENGTH;
	}
}

void findNearestWaypoint(void){
	uint8_t nearestwpindex = 0;
	float nearestwpdist = get_dist2_to_waypoint(trajectory[nearestwpindex]);
	float testdist;
	for (uint8_t i=1;i<TRAJ_LENGTH;i++){
		testdist = get_dist2_to_waypoint(trajectory[i]);
		if(testdist<nearestwpdist){
			nearestwpdist = testdist;
			nearestwpindex = i;
		}
	}
	traj_targetindex = nearestwpindex;
	return;
}

bool flyToCentre(void){
	bool temp = true;
	temp &= autopilot_guided_goto_ned(0,0,-NDI_FLIGHT_HEIGHT,0);
	return !temp;
}

char* strconcat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1)+strlen(s2)+1);//+1 for the zero-terminator
    //in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}

