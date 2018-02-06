#ifndef UWB_LOGGER_H
#define UWB_LOGGER_H

#include <std.h>
#include "../../subsystems/gps.h"


extern struct NedCoor_i uwb_gps_ned_vel_cm_s;      ///< speed NED in cm/s
extern struct NedCoor_f uwb_gps_ned_vel_cm_s_f;      ///< speed NED in cm/s
extern struct NedCoor_i uwb_gps_ned_pos_cm;
extern struct NedCoor_i uwb_gps_ned_pos_cm_f;

extern struct FloatVect3 uwb_optic_vel_m_s_f;

extern float uwb_sonarheight;

extern float uwb_smooth_yawr;
extern float uwb_smooth_ax;
extern float uwb_smooth_ay;

extern void uwb_logger_init(void);

extern void logEvent(uint8_t sender_id __attribute__((unused)),uint8_t ac_id, float time, float dt,float range, float trackedVx, float trackedVy, float trackedh, float trackedAx, float trackedAy,float trackedYawr,float xin, float yin, float h1in, float h2in, float u1in, float v1in, float u2in, float v2in, float gammain);
extern void uwb_gps_cb(uint8_t sender_id __attribute__((unused)),uint32_t time, struct GpsState *gps_s);
extern void uwb_optic_vel_cb(uint8_t sender_id __attribute__((unused)),uint32_t time,float vx, float vy, float vz, float noise);
extern void uwb_sonar_height_cb(uint8_t __attribute__((unused)) sender_id, float distance);
extern void uwb_logger_event(void);
#endif
