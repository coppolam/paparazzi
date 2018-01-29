#ifndef UWB_STATE_CONTROL_H
#define UWB_STATE_CONTROL_H

#include <std.h>

extern bool uwb_use_gps;
extern bool uwb_use_opticflow;
extern bool uwb_use_sonar;
extern bool uwb_send_onboard;

extern void uwb_state_control_init(void);
extern bool switchOnGPSandSendGPS(void);
extern bool switchOnGPSandSendONBOARD(void);
extern bool switchOffGPSandSendGPS(void);
extern bool switchOffGPSandSendONBOARD(void);

#endif
