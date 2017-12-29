#include "../../subsystems/navigation/waypoints.h"
#include "navigation.h"
#include <math.h>
#include <std.h>
#include "uwb_leader_control.h"
#include "generated/flight_plan.h"

#define TRAJ_EPS 0.4
#define TRAJ_LENGTH 8

uint8_t traj_targetindex = 0;

static uint8_t trajectory[TRAJ_LENGTH] = {WP_t0,WP_t1,WP_t2,WP_t3,WP_t4,WP_t5,WP_t6,WP_t7};

void findNearestWaypoint(void);
void getNextTargetWaypoint(void);

bool initialiseTrajectory(void){
	findNearestWaypoint();
	return false; // false means in the flight plan that the function executed succesfully.
}

bool flyTrajectory(void){
	getNextTargetWaypoint();
	NavGotoWaypoint(trajectory[traj_targetindex]);
	return false; // False means in the flight plan that the function executed succesfully
}

void getNextTargetWaypoint(void){
	if(get_dist2_to_waypoint(trajectory[traj_targetindex])<TRAJ_EPS*TRAJ_EPS){
		traj_targetindex = (traj_targetindex+1)%TRAJ_LENGTH;
	}
}

void findNearestWaypoint(void){
	uint8_t nearestwpindex = 0;
	float nearestwpdist = get_dist2_to_waypoint(trajectory[nearestwpindex]);
	float testdist;
	for (uint8_t i=1;i<TRAJ_LENGTH;i++){
		testdist = get_dist2_to_waypoint(trajectory[i]);
		if(testdist<nearestwpdist){
			nearestwpdist = testdist;
			nearestwpindex = i;
		}
	}
	traj_targetindex = nearestwpindex;
	return;
}
