#include "collisioncone.h"


int collisioncone_assess( float *cc,
	float relx, float rely,
	float ownvx, float ownvy,
	float relvx, float relvy,
	float radius)
{
	collisioncone_update(cc, relx, rely, relvx, relvy, radius);
	return collisioncone_checkdanger(cc, ownvx, ownvy);
}

/*
	Updated the coordinates of the triangle for the collision cone of an obstacle.
*/
void collisioncone_update( float *cc,
	float relx, float rely,
	float relvx, float relvy,
	float radius)
{
	float range, bearing;
	cart2polar(relx, rely, &range, &bearing);

	// The other two edges are symmetrical about a diagonal extending along the localization bearing.
	cc[0] = 0.0;
	cc[1] = 0.0;

	// This is the edge extending upwards to the right (+y)
	cc[2] = 5.0; 							   // x_body
	cc[3] = 5.0*tan(atan(radius/(1.0*range))); // y_body

	// This is the edge exteding upwards to the left (-y)
	cc[4] = cc[2]; 	 // x_body
	cc[5] = -cc[3];  // y_body

	shape_rotateatorigin(cc, 6, bearing);
	shape_shift(cc, 6, relvx, relvy);

}

/*
	Updated the coordinates of the triangle for the collision cone of an obstacle.
	The second one is adjusted to be the fusion of its original self and the first one.
	It could speed things up a little provided that multiple cones share a similar area and that obstacles are not moving, such that the origins of the triangle are shared.
*/
void collisioncone_fuse( float *cc0, float *cc1)
{

	// 2*PI is added everywhere to avoid the issue where a boundary is in between -ang and +ang
	if ((atan2(cc1[3], cc1[2]) + 2*M_PI > atan2(cc0[3], cc0[2]) + 2*M_PI) 
		&& (atan2(cc1[3], cc1[2]) + 2*M_PI < atan2(cc0[5], cc0[4]) + 2*M_PI)) {
		cc1[2] = cc0[2];
		cc1[3] = cc0[3];
	}

	if ((atan2(cc1[5], cc1[4]) + 2*M_PI > atan2(cc0[3], cc0[2]) + 2*M_PI)
		&& (atan2(cc1[5], cc1[4]) + 2*M_PI < atan2(cc0[5], cc0[4]) + 2*M_PI)) {
		cc1[4] = cc0[4];
		cc1[5] = cc0[5];
	}
}


bool collisioncone_checkdanger( float *cc, float ownvx, float ownvy)
{
	float vv[2];
	vv[0] = ownvx;
	vv[1] = ownvy;

	// Check if the current velocity is ok.
	// If point is in area then we get a flag
	return shape_checkifpointinarea(cc, 6, vv);
};


void collisioncone_findnewcmd( float cc[2][6], 
	float *v_des, float *psi_des, 
	float psisearch, int nfilters )
{
	int i;
	int count = 1;
	// int ng = 1;
	bool flag = true;
	float psi_add;
	deg2rad(psisearch, &psi_add);

	float psi0 = *psi_des;
	float vx, vy;

	while (*v_des <= 2.0) {
		polar2cart(*v_des, *psi_des, &vx, &vy);

		for (i = 0; i < nfilters; i++) { // Check if we succeed
			flag = collisioncone_checkdanger(cc[i], vx, vy);
			if (flag)
				break;
		}

		if(!flag) // No issues found
			return;

		*psi_des = psi0 + (count * psi_add);
		wrapTo2Pi(psi_des);

		// ng = ng * -1;
		count++;
			
		if (count >= (2*M_PI)/psi_add) {
			*v_des = *v_des + *v_des;
			count = 1;
		}
	}
	
	// Failed to find a solution, so just stop this time
	*v_des = 0.0;
};

// float collisioncone_expansionangle( float range, float R, float e )
// {
// 	return atan((range+(2*R)+e)/range);
// }

float movingaveragefilter(float *vec, int size, float newelement)
{
	array_shiftleft(vec, size, 1);
	vec[size-1] = newelement;
	float out = array_sum(size, vec)/(float)size;
	vec[size-1] = out;
	return out;
}