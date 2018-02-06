#ifndef UWB_FOLLOWER_CONTROL_H
#define UWB_FOLLOWER_CONTROL_H

#include <std.h>
// // ABI messages
#include "subsystems/abi.h"



#define NDI_FLIGHT_HEIGHT 1.5
#define NDI_PAST_VALS 200

typedef struct ndihandler{
	float delay;
	float tau_x;
	float tau_y;
	float wn_x;
	float wn_y;
	float eps_y;
	float eps_x;
	float Kp;
	float Ki;
	float Kd;
	float xarr[NDI_PAST_VALS];
	float yarr[NDI_PAST_VALS];
	float u1arr[NDI_PAST_VALS];
	float v1arr[NDI_PAST_VALS];
	float u2arr[NDI_PAST_VALS];
	float v2arr[NDI_PAST_VALS];
	float r1arr[NDI_PAST_VALS];
	float r2arr[NDI_PAST_VALS];
	float ax1arr[NDI_PAST_VALS];
	float ay1arr[NDI_PAST_VALS];
	float ax2arr[NDI_PAST_VALS];
	float ay2arr[NDI_PAST_VALS];
	float tarr[NDI_PAST_VALS];
	float gamarr[NDI_PAST_VALS];
	int data_start;
	int data_end;
	int data_entries;
	float commands[2];
	float commandscap[2];
	float maxcommand;
} ndihandler;

extern ndihandler ndihandle;

extern bool startNdiTracking(void);
extern bool stopNdiTracking(void);
extern bool ndi_follow_leader(void);

extern void addNdiValues(uint8_t sender_id __attribute__((unused)),uint8_t ac_id,float time, float dt,float range, float trackedVx, float trackedVy, float trackedh, float trackedAx, float trackedAy, float trackedYawr, float xin, float yin, float h1in, float h2in, float u1in, float v1in, float u2in, float v2in, float gammain);

extern void uwb_ndi_follower_init(void);

extern void calcNdiCommands(void);
//extern bool ndi_follow_init(void);

extern void printCircularFloatArr(float* arr);

extern void printNdiVars(void);

extern bool initialiseTrajectory(void);
extern bool flyTrajectory(void);
extern bool flyToCentre(void);
extern bool flyToWaypoint(uint8_t wp_id);










#endif
