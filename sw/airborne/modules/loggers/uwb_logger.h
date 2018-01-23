#ifndef UWB_LOGGER_H
#define UWB_LOGGER_H

#include <std.h>
#include "../../subsystems/gps.h"





extern void uwb_logger_init(void);

extern void logEvent(uint8_t sender_id __attribute__((unused)),float time, float dt,float range, float trackedVx, float trackedVy, float trackedh, float xin, float yin, float h1in, float h2in, float u1in, float v1in, float u2in, float v2in, float gammain);
extern void uwb_gps_cb(uint8_t sender_id __attribute__((unused)),uint32_t time, struct GpsState *gps_s);
extern void uwb_optic_vel_cb(uint8_t sender_id __attribute__((unused)),uint32_t time,float vx, float vy, float vz, float noise);

#endif
