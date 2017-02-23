/*
 * Copyright (C) 2016 - IMAV 2016
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file modules/computer_vision/flight_plan_guided.c
 * @author IMAV 2016
 */

#include "modules/flight_plan_guided/flight_plan_guided.h"
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>


#include "mcu_periph/uart.h"
/*
#include "modules/stereocam/stereocam.h"
#include "modules/stereocam/stereoprotocol.h"
#include "modules/stereocam/stereocam2state/stereocam2state.h"
*/

// start and stop modules
#include "generated/modules.h"

#ifdef INS_BARO_AGL_OFFSET
#define LEGS_HEIGHT INS_BARO_AGL_OFFSET
#else
#define LEGS_HEIGHT 0.2
#endif

int32_t counter = 0;

float wanted_heading;

#define NOM_FLIGHT_ALT 1.7  // nominal flight altitude
float nom_flight_alt; // nominal flight altitude

#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"

//abi for range sensors
static abi_event range_sensors_ev;
static void range_sensors_cb(uint8_t sender_id,
        int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left,int16_t range_bottom, int16_t range_top);
struct range_finders_ range_finders;
static void range_sensors_cb(uint8_t sender_id,
                             int16_t range_front, int16_t range_right, int16_t range_back, int16_t range_left,int16_t range_bottom, int16_t range_top)
{
  static int32_t front_wall_detect_counter = 0;
  static const int32_t max_sensor_range = 2000;

    // save range finders values
    range_finders.front = range_front;
    range_finders.right = range_right;
    range_finders.left = range_left;
    range_finders.back = range_back;
    range_finders.top = range_top;
    range_finders.bottom = range_bottom;
    uint16_t tel_buf[4] = {0,range_right, 0 , range_left};
     uint8_t length = 4;

   /* DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &length, &(length), length,
  		  tel_buf);*/
}

//abi for stereocam
static abi_event stereocam_obstacle_ev;
static void stereocam_obstacle_cb(uint8_t sender_id, float heading, float range);
float stereo_distance;
void stereocam_obstacle_cb(uint8_t sender_id, float heading, float range)
{
	stereo_distance = range;
	DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
}

static abi_event avoidance_turn_angle_ev;
static void avoidance_turn_angle_cb(uint8_t sender_id, float angle, bool trigger);
 float turn_angle;
 float turn_trigger;
static void avoidance_turn_angle_cb(uint8_t sender_id, float angle, bool trigger)
{
	turn_trigger = trigger;
	turn_angle = angle;
}




static abi_event agl_ev;
static float filtered_agl = LEGS_HEIGHT;
static void agl_cb(uint8_t sender_id, float agl);

static void agl_cb(uint8_t sender_id, float agl)
{
  filtered_agl = filtered_agl * 0.9 + agl * 0.1;
}



void flight_plan_guided_init(void)
{
  nom_flight_alt = NOM_FLIGHT_ALT;
  AbiBindMsgAGL(1, &agl_ev, agl_cb); // ABI to the altitude above ground level
  AbiBindMsgRANGE_SENSORS(ABI_BROADCAST, &range_sensors_ev, range_sensors_cb);
  AbiBindMsgSTEREOCAM_OBSTACLE(ABI_BROADCAST, &stereocam_obstacle_ev, stereocam_obstacle_cb);
  AbiBindMsgAVOIDANCE_TURN_ANGLE(ABI_BROADCAST, &avoidance_turn_angle_ev, avoidance_turn_angle_cb);
}


/* Kill throttle */
uint8_t KillEngines(void)
{
  autopilot_set_motors_on(FALSE);

  return false;
}


/* Start throttle */
uint8_t StartEngines(void)
{
  autopilot_set_motors_on(TRUE);

  return false;
}


/* Reset the altitude reference to the current GPS alt if GPS is used */
uint8_t ResetAlt(void) {ins_reset_altitude_ref(); return false;}

bool TakeOff(float climb_rate)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_v_set_guided_vz(-climb_rate);
  guidance_h_set_guided_body_vel(0, 0);

  return false;
}


bool WaitUntilAltitude(float altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z < altitude) { return true; }

  return false;
}


bool WaitUntilSpeedOrAltitude(float speed, float fail_altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z > fail_altitude) { return false; }
  if (stateGetSpeedEnu_f()->z < speed) { return true; }

  return false;
}

float specialtimer = 0;

bool ResetSpecialTimer(void)
{
  specialtimer = 0;
  return false;
}

bool WaitUntilTimerOrAltitude(float sec, float fail_altitude)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z > fail_altitude) { return false; }
  specialtimer += 1.0f / ((float)NAV_FREQ);
  if (specialtimer < sec) { return true; }

  return false;
}


bool RotateToHeading(float heading)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_h_set_guided_heading(heading);
  return false;
}


bool WaitforHeading(float heading)
{
  guidance_h_set_guided_heading_rate(1.);
  if (fabs(heading - stateGetNedToBodyEulers_f()->psi) < 0.1) {
    guidance_h_set_guided_heading(heading);
    return false;
  }

  return true;
}


uint8_t Hover(float alt)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }
  // Horizontal velocities are set to zero
  guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
  guidance_h_set_guided_body_vel(0, 0);
  guidance_v_set_guided_z(-alt);

  return false;
}

/* Move forward */
uint8_t MoveForward(float vx)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(vx, 0);
  }
  return false;
}

/* Move Right */
uint8_t MoveRight(float vy)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(0, vy);
  }
  return false;
}

void stereo_force_field(float *vel_body_x, float distance_stereo, float avoid_inner_border, float avoid_outer_border,
                        float tinder_range, float min_vel_command, float max_vel_command)
{
  static const int16_t max_sensor_range = 2.0f;

  float difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_x_command = *vel_body_x;

  // Balance avoidance command for front direction (sideways)
  if (distance_stereo > max_sensor_range) {
    //do nothing
  } else if (distance_stereo < avoid_inner_border) {
    avoid_x_command -= max_vel_command;
  } else if (distance_stereo < avoid_outer_border) {
    // Linear
    avoid_x_command -= (max_vel_command - min_vel_command) *
                       (avoid_outer_border - distance_stereo)
                       / difference_inner_outer;
  } else {
    if (distance_stereo > tinder_range) {
      avoid_x_command += max_vel_command;
    }
  }

  *vel_body_x = avoid_x_command;
}




void range_sensor_force_field(float *vel_body_x, float *vel_body_y, float *vel_body_z, int16_t avoid_inner_border, int16_t avoid_outer_border,
    int16_t tinder_range, float min_vel_command, float max_vel_command)
{
  static const int16_t max_sensor_range = 2000;

  int16_t difference_inner_outer = avoid_outer_border - avoid_inner_border;

  // Velocity commands
  float avoid_x_command = *vel_body_x;
  float avoid_y_command = *vel_body_y;
  float avoid_z_command = *vel_body_z;

  // Balance avoidance command for y direction (sideways)
  if (range_finders.right < 1 || range_finders.right > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.right < avoid_inner_border){
    avoid_y_command -= max_vel_command;
  } else if (range_finders.right < avoid_outer_border) {
    // Linear
    avoid_y_command -= (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.right)
        / (float)difference_inner_outer;
  } else {}

  if (range_finders.left < 1 || range_finders.left > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.left < avoid_inner_border){
    avoid_y_command += max_vel_command;
  } else if (range_finders.left < avoid_outer_border) {
    // Linear
    avoid_y_command += (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.left)
        / (float)difference_inner_outer;
  } else {}

  // balance avoidance command for x direction (forward/backward)
  if (range_finders.front < 1 || range_finders.front > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.front < avoid_inner_border){
    avoid_x_command -= max_vel_command;
  } else if (range_finders.front < avoid_outer_border) {
    // Linear
    avoid_x_command -= (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.front)
        / (float)difference_inner_outer;
  } else if(range_finders.front > tinder_range){
      avoid_x_command += max_vel_command;
  } else {}


  if (range_finders.back < 1 || range_finders.back > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.back < avoid_inner_border){
    avoid_x_command += max_vel_command;
  } else if (range_finders.back < avoid_outer_border) {
    // Linear
    avoid_x_command += (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.back)
        / (float)difference_inner_outer;
  } else {}


/*  if (range_finders.top < 1 || range_finders.top > max_sensor_range)
  {
    //do nothing
  } else if(range_finders.top < 600){
    avoid_z_command += max_vel_command;
  } else if (range_finders.top < 800) {
    // Linear
    avoid_z_command += (max_vel_command - min_vel_command) *
        ((float)avoid_outer_border - (float)range_finders.top)
        / (float)difference_inner_outer;
  } else {
	    if (distance_stereo > 1200) {
	      avoid_x_command -= max_vel_command;
	    }
  }*/

  *vel_body_x = avoid_x_command;
  *vel_body_y = avoid_y_command;
  *vel_body_z = avoid_z_command;

}

bool avoid_wall(float vel_body_x_command)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {

    stereo_force_field(&vel_body_x_command, distance_stereo, 0.80f, 1.2, 5.0f , 0.0f, -0.2f);
    MoveForward(vel_body_x_command);

  }
  return true;

}

bool avoid_wall_and_sides(float vel_body_x_command)
{
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (autopilot_mode == AP_MODE_GUIDED) {

	float vel_body_y_command = 0.0f;
	float vel_body_z_command = 0.0f;

    stereo_force_field(&vel_body_x_command, distance_stereo, 0.8f, 1.2, 5.0f , 0.0f, -0.3f);
    range_sensor_force_field(&vel_body_x_command, &vel_body_y_command, &vel_body_z_command, 1000, 1200, 9000 , 0.0f, 0.3f);

    guidance_v_set_guided_z(-1.5);
    //guidance_v_set_guided_vz(vel_body_z_command);
    guidance_h_set_guided_body_vel(vel_body_x_command, vel_body_y_command);

    DOWNLINK_SEND_VELOCITY_COMMANDS(DefaultChannel, DefaultDevice, &vel_body_x_command, &vel_body_y_command, &vel_body_z_command);

/*
    if(range_finders.top<2000)
    {
    float reset_height = stateGetPositionEnu_f()->z - (float)(range_finders.top - range_finders.bottom)/1000;
    guidance_v_set_guided_z(reset_height);
    }
*/
  //  guidance_h_set_guided_body_vel(0.2,0.2);
  }
  return true;

}

bool change_h_mode(uint8_t mode)
{
  guidance_h_mode_changed(mode);
  return false;
}

bool change_v_mode(uint8_t mode)
{
  guidance_v_mode_changed(mode);
  return false;
}


bool RotateToHeading_ATT(float new_heading, float trim_phi, float trim_theta)
{
  struct Int32Eulers cmd;

  if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    cmd.phi = ANGLE_BFP_OF_REAL(trim_phi); //trim?
    cmd.theta = ANGLE_BFP_OF_REAL(trim_theta);
    cmd.psi = ANGLE_BFP_OF_REAL(new_heading);

    stabilization_attitude_set_rpy_setpoint_i(&cmd);
    stabilization_attitude_run(autopilot_in_flight);
  }
  return false;
}

bool ResetAngles_ATT(float current_heading)
{
  struct Int32Eulers cmd;
  if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    cmd.phi = ANGLE_BFP_OF_REAL(0.0f);
    cmd.theta = ANGLE_BFP_OF_REAL(0.0f);
    cmd.psi = ANGLE_BFP_OF_REAL(current_heading);

    stabilization_attitude_set_rpy_setpoint_i(&cmd);
    stabilization_attitude_run(autopilot_in_flight);
  }
  return false;
}

bool WaitforHeadingCondition(float heading)
{
  if (fabs(heading - stateGetNedToBodyEulers_f()->psi) < 0.2) {
    return false;
  }

  return true;
}

bool wait_for_mode(uint8_t mode)
{
	  if (guidance_h.mode == mode) {
		  return false;
	  }

	  return true;


}

bool reset_counter()
{
	counter = 0;
	  return false;

}


bool wait_counter(int32_t end_counter)
{
	counter++;
	  if (counter>end_counter) {
		  return false;
	  }

	  return true;


}

bool kill_mode()
{
	autopilot_set_mode(AP_MODE_KILL);
	return true;
}

