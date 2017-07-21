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
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>
#include "math/pprz_algebra_float.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"


#include "mcu_periph/uart.h"
/*
#include "modules/stereocam/stereocam.h"
#include "modules/stereocam/stereoprotocol.h"
#include "modules/stereocam/stereocam2state/stereocam2state.h"
*/

float distance_thres_logic;
int32_t turn_counter;
float hover_wait_sec;
float forward_speed;
float seconds_of_bad_quality;
bool first_occurance_bad_quality = TRUE;
float seconds_of_bad_quality_thres;
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

#include "subsystems/datalink/telemetry.h"
#include "subsystems/abi.h"

float distance_stereo;


//abi for stereocam
static abi_event stereocam_obstacle_ev;
static void stereocam_obstacle_cb(uint8_t sender_id, float heading, float range,float quality);
float distance_stereo;
float quality;
void stereocam_obstacle_cb(uint8_t UNUSED(sender_id), float UNUSED(heading), float range, float flow_quality)
{
  distance_stereo = range;
  quality = flow_quality;
 // DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
}

static abi_event avoidance_turn_angle_ev;
static void avoidance_turn_angle_cb(uint8_t sender_id, float angle, bool trigger);
float turn_angle;
float turn_trigger;
static void avoidance_turn_angle_cb(uint8_t UNUSED(sender_id), float angle, bool trigger)
{
  turn_trigger = trigger;
  turn_angle = angle;
}

struct FloatVect3 vel_body_FF;
static abi_event forcefield_velocity_ev;
static void forcefield_velocity_cb(uint8_t sender_id, float vel_body_x_FF, float vel_body_y_FF, float vel_body_z_FF);
static void forcefield_velocity_cb(uint8_t UNUSED(sender_id), float vel_body_x_FF, float vel_body_y_FF, float vel_body_z_FF)
{
	//  DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);

  vel_body_FF.x = vel_body_x_FF;
  vel_body_FF.y = vel_body_y_FF;
  vel_body_FF.z = vel_body_z_FF;

 // DOWNLINK_SEND_FORCEFIELD_VELOCITY(DefaultChannel, DefaultDevice,&vel_body_x_FF,&vel_body_y_FF,&vel_body_z_FF);

}



static abi_event agl_ev;
static float filtered_agl = LEGS_HEIGHT;
static void agl_cb(uint8_t sender_id, float agl);

static void agl_cb(uint8_t UNUSED(sender_id), float agl)
{
  filtered_agl = filtered_agl * 0.9 + agl * 0.1;
}


void flight_plan_guided_init(void)
{

   distance_stereo = 2.0f;
   distance_thres_logic =  1.4f;
   turn_counter = 3;
   hover_wait_sec = 1.5f;
   forward_speed = 0.1f;

   seconds_of_bad_quality = 0;
   seconds_of_bad_quality_thres = 4.0f;


  nom_flight_alt = NOM_FLIGHT_ALT;
  AbiBindMsgAGL(1, &agl_ev, agl_cb); // ABI to the altitude above ground level
  AbiBindMsgSTEREOCAM_OBSTACLE(ABI_BROADCAST, &stereocam_obstacle_ev, stereocam_obstacle_cb);
  AbiBindMsgAVOIDANCE_TURN_ANGLE(ABI_BROADCAST, &avoidance_turn_angle_ev, avoidance_turn_angle_cb);
  AbiBindMsgFORCEFIELD_VELOCITY(ABI_BROADCAST, &forcefield_velocity_ev, forcefield_velocity_cb);
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
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  guidance_v_set_guided_vz(-climb_rate);
  guidance_h_set_guided_body_vel(0, 0);

  return false;
}


bool WaitUntilAltitude(float altitude)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z < altitude) { return true; }

  return false;
}


bool WaitUntilSpeedOrAltitude(float speed, float fail_altitude)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

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
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (stateGetPositionEnu_f()->z > fail_altitude) { return false; }
  specialtimer += 1.0f / ((float)NAV_FREQ);
  if (specialtimer < sec) { return true; }

  return false;
}


bool RotateToHeading(float heading)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

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
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }
  // Horizontal velocities are set to zero
  guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
  guidance_h_set_guided_body_vel(0, 0);
  guidance_v_set_guided_z(-alt);

  return false;
}

/* Move forward */
uint8_t MoveForward(float vx)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (autopilot.mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(vx, 0);
  }
  return false;
}

/* Move Right */
uint8_t MoveRight(float vy)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (autopilot.mode == AP_MODE_GUIDED) {
    guidance_h_set_guided_body_vel(0, vy);
  }
  return false;
}




bool avoid_wall(float vel_body_x_command)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (autopilot.mode == AP_MODE_GUIDED) {

    //TODO: DO stereo range also in range modules
    //stereo_force_field(&vel_body_x_command, distance_stereo, 0.80f, 1.2, 5.0f , 0.0f, -0.2f);
    MoveForward(vel_body_x_command);

  }
  return true;

}


/* Wraps an angle in radians between -PI and +PI */
float wrapToPi_guided(float ang)
{
	if (ang > M_PI) {
		while (ang > M_PI) {
			ang = ang - 2*M_PI;
		}
	}
	else if (ang < -M_PI) {
		while (ang < -M_PI) {
			ang = ang + 2*M_PI;
		}
	}
	return ang;
}

float first_time_bad_quality;

int32_t counter_checkoccurances =0;
bool avoid_wall_and_sides(float vel_body_x_command)
{
  if (autopilot.mode != AP_MODE_GUIDED) { return true; }

  if (autopilot.mode == AP_MODE_GUIDED) {

      if (quality <= 100)
	  vel_body_x_command = vel_body_x_command*quality/100.0f + 0.05;
      if(quality==0){


    	  if (first_occurance_bad_quality){
        	//  printf("counter_checkoccurances: %d\n", counter_checkoccurances);
        	  if(counter_checkoccurances<4){
                  counter_checkoccurances = 0;

        	  }else{

              counter_checkoccurances = 0;
        	  seconds_of_bad_quality =0;

    		  first_time_bad_quality=(float)get_sys_time_msec()/1000;
    		  first_occurance_bad_quality = FALSE;
        	  }
    	  }
    	  float current_time_bad_quality = (float)get_sys_time_msec()/1000;
    	  seconds_of_bad_quality = current_time_bad_quality - first_time_bad_quality;
    	//  printf("bad quality amount of time: %f\n",seconds_of_bad_quality);
       }
      else
      {
    	  counter_checkoccurances ++;
    	  //printf("counter_checkoccurances: %d\n", counter_checkoccurances);
    	  seconds_of_bad_quality =0;
    	  first_occurance_bad_quality = TRUE;
      }

/*      if (seconds_of_bad_quality>seconds_of_bad_quality_thres)
      {printf("should turn now?\n"); }*/



    vel_body_x_command += vel_body_FF.x;
    float vel_body_y_command = vel_body_FF.y;

    //TODO: remove old functions
    //stereo_force_field(&vel_body_x_command, distance_stereo, 0.8f, 1.2, 5.0f , 0.0f, -0.3f);
    //range_sensor_force_field(&vel_body_x_command, &vel_body_y_command, &vel_body_z_command, 1000, 1200, 9000 , 0.0f, 0.3f);

    guidance_v_set_guided_z(-nom_flight_alt);
    guidance_h_set_guided_body_vel(vel_body_x_command, vel_body_y_command);

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


bool RotateToHeading_ATT(float new_heading, float trim_phi, float trim_theta, bool in_flight)
{
  struct Int32Eulers cmd;

  if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    cmd.phi = ANGLE_BFP_OF_REAL(trim_phi); //trim?
    cmd.theta = ANGLE_BFP_OF_REAL(trim_theta);
    cmd.psi = ANGLE_BFP_OF_REAL(new_heading);

    stabilization_attitude_set_rpy_setpoint_i(&cmd);
    stabilization_attitude_run(in_flight);
  }
  return false;
}

bool ResetAngles_ATT(float current_heading, bool in_flight)
{
  struct Int32Eulers cmd;
  if (guidance_h.mode == GUIDANCE_H_MODE_ATTITUDE) {
    cmd.phi = ANGLE_BFP_OF_REAL(0.0f);
    cmd.theta = ANGLE_BFP_OF_REAL(0.0f);
    cmd.psi = ANGLE_BFP_OF_REAL(current_heading);

    stabilization_attitude_set_rpy_setpoint_i(&cmd);
    stabilization_attitude_run(in_flight);
  }
  return false;
}

bool WaitforHeadingCondition(float heading)
{
  if (fabs(heading - stateGetNedToBodyEulers_f()->psi) < 0.01) {
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
  if (counter > end_counter) {
    return false;
  }

  return true;

}

bool kill_mode()
{
  autopilot_set_mode(AP_MODE_KILL);
  return true;
}

