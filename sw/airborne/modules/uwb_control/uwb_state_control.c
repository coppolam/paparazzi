#include "uwb_state_control.h"

bool uwb_use_gps;
bool uwb_use_opticflow;
bool uwb_use_sonar;
bool uwb_send_onboard;



void uwb_state_control_init(void){
#if UWB_USE_GPS
	uwb_use_gps = true;
#else
	uwb_use_gps = false;
#endif
#if UWB_USE_OPTICFLOW
	uwb_use_opticflow = true;
#else
	uwb_use_opticflow = false;
#endif
#if UWB_USE_SONAR
	uwb_use_sonar = true;
#else
	uwb_use_sonar = false;
#endif
#if UWB_SEND_ONBOARD
	uwb_send_onboard = true;
#else
	uwb_send_onboard=false;
#endif
	PRINT_CONFIG_VAR(UWB_USE_GPS);
}


bool switchOnGPSandSendGPS(void){
	uwb_use_gps = true;
	uwb_use_opticflow = false;
	uwb_use_sonar = false;
	uwb_send_onboard = false;
	return false;
}
bool switchOnGPSandSendONBOARD(void){
	uwb_use_gps = true;
	uwb_use_opticflow = false;
	uwb_use_sonar = false;
	uwb_send_onboard = true;
	return false;
}
bool switchOffGPSandSendGPS(void){
	uwb_use_gps = false;
	uwb_use_opticflow = true;
	uwb_use_sonar = true;
	uwb_send_onboard = false;
	return false;
}
bool switchOffGPSandSendONBOARD(void){
	uwb_use_gps = false;
	uwb_use_opticflow = true;
	uwb_use_sonar = true;
	uwb_send_onboard = true;
	return false;
}

