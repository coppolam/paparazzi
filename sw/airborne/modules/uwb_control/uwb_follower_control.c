#include <time.h>
#include <math.h>
#include "uwb_follower_control.h"
#include "../relativelocalizationfilter/fmatrix.h"


#define NDI_MOST_RECENT ndihandle.data_entries-1

bool ndi_following_leader = false;
bool ndi_run_computation = true;

void cleanNdiValues(float tcur);
float accessCircularFloatArrElement(float *arr, int index);
float computeNdiFloatIntegral(float* ndiarr, float curtime);
void bindNorm(void);

ndihandler ndihandle = {.delay = 3,
		.tau_x=5,
		.tau_y = 5,
		.Kp = -2,
		.Ki = -0.2,
		.Kd = -5,
		.data_start = 0,
		.data_end = 0,
		.data_entries = 0,
		.maxcommand = 1};

static abi_event uwb_ndi_ev;

extern void uwb_ndi_follower_init(void){
	AbiBindMsgUWB_NDI(ABI_BROADCAST, &uwb_ndi_ev, addNdiValues);
}


void addNdiValues(uint8_t sender_id __attribute__((unused)),float x, float y, float u1, float v1, float u2, float v2){
	float t = get_sys_time_usec()/pow(10,6);
	if(ndihandle.data_entries==NDI_PAST_VALS){
		ndihandle.data_entries--;
		ndihandle.data_start = (ndihandle.data_start+1)%NDI_PAST_VALS;
	}
	ndihandle.xarr[ndihandle.data_end] = x;
	ndihandle.yarr[ndihandle.data_end] = y;
	ndihandle.u1arr[ndihandle.data_end] = u1;
	ndihandle.v1arr[ndihandle.data_end] = v1;
	ndihandle.u2arr[ndihandle.data_end] = u2;
	ndihandle.v2arr[ndihandle.data_end] = v2;
	ndihandle.tarr[ndihandle.data_end] = t;
	ndihandle.data_end = (ndihandle.data_end+1)%NDI_PAST_VALS;
	ndihandle.data_entries++;
}

void cleanNdiValues(float tcur){
	int curentries = ndihandle.data_entries;
	for(int i=0;i<curentries;i++){
		if((tcur-ndihandle.tarr[ndihandle.data_start])>ndihandle.delay){
			ndihandle.data_start = (ndihandle.data_start+1)%NDI_PAST_VALS;
			ndihandle.data_entries--;

		}
	}
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
	int realindex = (ndihandle.data_start+index)%NDI_PAST_VALS;
	return arr[realindex];
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
		float oldx = accessCircularFloatArrElement(ndihandle.xarr,0);
		float oldy = accessCircularFloatArrElement(ndihandle.yarr,0);

		float oldu1 = accessCircularFloatArrElement(ndihandle.u1arr,0);
		float oldv1 = accessCircularFloatArrElement(ndihandle.v1arr,0);
		float oldu2 = accessCircularFloatArrElement(ndihandle.u2arr,0);
		float oldv2 = accessCircularFloatArrElement(ndihandle.v2arr,0);
		oldx = oldx - computeNdiFloatIntegral(ndihandle.u1arr,curtime);
		oldy = oldy - computeNdiFloatIntegral(ndihandle.v1arr,curtime);

		float Minv[4];
		fmat_make_zeros(Minv,2,2);
		fmat_assign(0,0,2,Minv,-ndihandle.tau_x);
		fmat_assign(1,1,2,Minv,-ndihandle.tau_y);

		float l[2];
		fmat_make_zeros(l,2,1);
		fmat_assign(0,0,1,l,oldu1/ndihandle.tau_x);
		fmat_assign(1,0,1,l,oldv1/ndihandle.tau_y);

		float oldxed = oldu2 - oldu1;
		float oldyed = oldv2 - oldv1;

		float v[2];
		v[0] = ndihandle.Kp * oldx + ndihandle.Kd * oldxed;
		v[1] = ndihandle.Kp * oldy + ndihandle.Kd * oldyed;

		float sig[2];
		sig[0] = v[0]-l[0];
		sig[1] = v[1]-l[1];

		fmat_mult(2,2,1,ndihandle.commands,Minv,sig);
		bindNorm();
	}
	else{
		ndihandle.commands[0] = 0;
		ndihandle.commands[1] = 0;
	}
}

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
	float normcom = sqrt(ndihandle.commands[1]*ndihandle.commands[1] + ndihandle.commands[0]*ndihandle.commands[0]);
	if(normcom>ndihandle.maxcommand){
		ndihandle.commands[0] = ndihandle.commands[0] * ndihandle.maxcommand/normcom;
		ndihandle.commands[1] = ndihandle.commands[1] * ndihandle.maxcommand/normcom;
	}

}

void printNdiVars(void){
	printf("xe "); printCircularFloatArr(ndihandle.xarr);
	printf("ye "); printCircularFloatArr(ndihandle.yarr);
	printf("u1 "); printCircularFloatArr(ndihandle.u1arr);
	printf("v1 "); printCircularFloatArr(ndihandle.v1arr);
	printf("u2 "); printCircularFloatArr(ndihandle.u2arr);
	printf("v2 "); printCircularFloatArr(ndihandle.v2arr);
	printf("t "); printCircularFloatArr(ndihandle.tarr);
	printf("NDI commands: %f, %f\n",ndihandle.commands[0],ndihandle.commands[1]);
}
