#ifndef COLLISIONCONE_H
#define COLLISIONCONE_H

#include "math.h"
#include "shape.h"
#include "coordinateconversions.h"


int collisioncone_assess( float *cc,
	float relx, float rely,
	float ownvx, float ownvy,
	float relvx, float relvy,
	float radius);

void collisioncone_update( float *cc,
	float relx, float rely,
	float relvx, float relvy,
	float radius);

void collisioncone_fuse( float *cc0, float *cc1 );

bool collisioncone_checkdanger( float *cc, 
	float ownvx, float ownvy);

void collisioncone_findnewcmd(	float cc[][6], 
	float *v_des, float *psi_des,
	float psisearch, int nfilters );
	
// float collisioncone_expansionangle ( float range, float R, float e );
float movingaveragefilter(float *array, int size, float newelement);

#endif