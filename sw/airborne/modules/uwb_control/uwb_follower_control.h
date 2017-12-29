#ifndef UWB_FOLLOWER_CONTROL_H
#define UWB_FOLLOWER_CONTROL_H

#include <std.h>
// // ABI messages
#include "subsystems/abi.h"


#define NDI_PAST_VALS 20

typedef struct ndihandler{
	float delay;
	float tau_x;
	float tau_y;
	float Kp;
	float Ki;
	float Kd;
	float xarr[NDI_PAST_VALS];
	float yarr[NDI_PAST_VALS];
	float u1arr[NDI_PAST_VALS];
	float v1arr[NDI_PAST_VALS];
	float u2arr[NDI_PAST_VALS];
	float v2arr[NDI_PAST_VALS];
	float tarr[NDI_PAST_VALS];
	int data_start;
	int data_end;
	int data_entries;
	float commands[2];
	float maxcommand;
} ndihandler;

extern ndihandler ndihandle;

extern bool startNdiTracking(void);
extern bool stopNdiTracking(void);

extern void addNdiValues(uint8_t sender_id __attribute__((unused)), float x, float y, float u1, float v1, float u2, float v2);

extern void uwb_ndi_follower_init(void);

extern void calcNdiCommands(void);

extern void printCircularFloatArr(float* arr);

extern void printNdiVars(void);











#endif
