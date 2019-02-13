#ifndef UWB_FOLLOWER_CONTROL_H
#define UWB_FOLLOWER_CONTROL_H

#include <std.h>

#define NDI_FLIGHT_HEIGHT 1.5 // Flight at 1.5m unless otherwise specified.
#define NDI_PAST_VALS 200 // Store the last 200 values in order to compute the control

typedef struct ndihandler {
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


extern bool startNdiTracking(void);
extern bool stopNdiTracking(void);

extern bool hover_guided(void);
extern bool ndi_follow_leader(void);

extern void uwb_follower_control_init(void);
extern void uwb_follower_control_periodic(void);

#endif /*UWB_FOLLOWER_CONTROL*/
