/*
 * Copyright (C) Steven van der Helm / Mario Coppola
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/relativelocalizationfilter/discreteekf.h"
 * @author Steven van der Helm / Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 */

#ifndef DISCRETEEKF_NO_NORTH_H
#define DISCRETEEKF_NO_NORTH_H

#define UWB_LOCALIZATION

#include "fmatrix.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include <stdbool.h>

#define EKF_N 9
#define EKF_M 7
#define EKF_L 6

enum ekf_statein{x12,y12,z1,z2,u1,v1,u2,v2,gam};
enum ekf_input{u1dm,v1dm,u2dm,v2dm,r1m,r2m};

/*
 * Continuous time state transition equation
 * state is: {x_rel,y_rel,h1,h2,u1,v1,u2,v2,gamma}
 */
static inline void ekf_fsym(float *statein,float *input,float *output){
	output[0] =  input[r1m]*statein[y12] - statein[u1] + statein[u2] * cos(statein[gam]) - statein[v2] * sin(statein[gam]);
	output[1] = -input[r1m]*statein[x12] - statein[v1] + statein[u2] * sin(statein[gam]) + statein[v2] * cos(statein[gam]);
	output[2] = 0;
	output[3] = 0;
	output[4] = input[u1dm] + input[r1m] * statein[v1];
	output[5] = input[v1dm] - input[r1m] * statein[u1];
	output[6] = input[u2dm] + input[r2m] * statein[v2];
	output[7] = input[v2dm] - input[r2m] * statein[u2];
	output[8] = input[r2m] - input[r1m];
}

static inline void ekf_Fx(float *statein,float *input,float *output){
	fmat_make_zeros(output,EKF_N,EKF_N);
	fmat_assign(0,1,EKF_N,output,input[r1m]);
	fmat_assign(0,4,EKF_N,output,-1);
	fmat_assign(0,6,EKF_N,output,cos(statein[gam]));
	fmat_assign(0,7,EKF_N,output,-sin(statein[gam]));
	fmat_assign(0,8,EKF_N,output,-statein[v2] * cos(statein[gam]) - statein[u2] * sin(statein[gam]));

	fmat_assign(1,0,EKF_N,output,-input[r1m]);
	fmat_assign(1,5,EKF_N,output,-1);
	fmat_assign(1,6,EKF_N,output,sin(statein[gam]));
	fmat_assign(1,7,EKF_N,output,cos(statein[gam]));
	fmat_assign(1,8,EKF_N,output,statein[u2]*cos(statein[gam]) - statein[v2] * sin(statein[gam]));

	fmat_assign(4,5,EKF_N,output,input[r1m]);
	fmat_assign(5,4,EKF_N,output,-input[r1m]);
	fmat_assign(6,7,EKF_N,output,input[r2m]);
	fmat_assign(7,6,EKF_N,output,-input[r2m]);
}

static inline void ekf_G(float *statein,float *input,float *output){
	fmat_make_zeros(output,EKF_N,EKF_L);
	fmat_assign(0,4,EKF_L,output,statein[y12]);
	fmat_assign(1,4,EKF_L,output,-statein[x12]);
	fmat_assign(4,0,EKF_L,output,1);
	fmat_assign(4,4,EKF_L,output,statein[v1]);
	fmat_assign(5,1,EKF_L,output,1);
	fmat_assign(5,4,EKF_L,output,-statein[u1]);
	fmat_assign(6,2,EKF_L,output,1);
	fmat_assign(6,4,EKF_L,output,statein[v2]);
	fmat_assign(7,3,EKF_L,output,1);
	fmat_assign(7,4,EKF_L,output,-statein[u2]);
	fmat_assign(8,4,EKF_L,output,-1);
	fmat_assign(8,5,EKF_L,output,1);
}

/*
 * Measurement equation, measures range, height1, height2, u1, v1, u2, v2
 */
static inline void ekf_hsym(float *statein,float *output){
	output[0] = pow(pow((statein[z1]-statein[z2]),2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5);
	output[1] = statein[z1];
	output[2] = statein[z2];
	output[3] = statein[u1];
	output[4] = statein[v1];
	output[5] = statein[u2];
	output[6] = statein[v2];
}

static inline void ekf_Hx(float *statein, float *output){
	fmat_make_zeros(output,EKF_M,EKF_N);
	fmat_assign(0,0,EKF_N,output,statein[x12]/(pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5)));
	fmat_assign(0,1,EKF_N,output,statein[y12]/(pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5)));
	fmat_assign(0,2,EKF_N,output,(2*statein[z1]-2*statein[z2])/(2*pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5)));
	fmat_assign(0,3,EKF_N,output,-(2*statein[z1]-2*statein[z2])/(2*pow(pow(statein[z1]-statein[z2],2.0)+pow(statein[x12],2.0)+pow(statein[y12],2.0),0.5)));
	fmat_assign(1,2,EKF_N,output,1);
	fmat_assign(2,3,EKF_N,output,1);
	fmat_assign(3,4,EKF_N,output,1);
	fmat_assign(4,5,EKF_N,output,1);
	fmat_assign(5,6,EKF_N,output,1);
	fmat_assign(6,7,EKF_N,output,1);

}

typedef struct ekf_filter {
	/* Jacobian of state				*/
	float Phi[EKF_N*EKF_N];
	/* noise input matrix				*/
	float Gamma[EKF_N*EKF_L];
	/* measurement vector			*/
	float *Zm_in;
  /* Jacobian of state				*/
  float Fx[EKF_N*EKF_N];
  /* noise input matrix				*/
  float G[EKF_N*EKF_L];
  /* input vector					*/
  float *U;
  /* state difference				*/
  float dX[EKF_N];
  /* state                           */
  float X[EKF_N];
  /* state prediction                */
  float Xp[EKF_N];
  /* measurement prediction          */
  float Zp[EKF_M];
  /* state covariance matrix         */
  float P[EKF_N*EKF_N];
  /* process covariance noise        */
  float Q[EKF_N*EKF_N];
  /* measurement covariance noise    */
  float R[EKF_M*EKF_M];
  /* jacobian of the measure wrt X   */
  float H[EKF_N*EKF_M];

  /* Temp matrices */
  float tmp1[EKF_N*EKF_N];
  float tmp2[EKF_N*EKF_N];
  float tmp3[EKF_N*EKF_N];
  float tmp4[EKF_N*EKF_N];

  float dt;


} ekf_filter;

extern void ekf_set_vals(ekf_filter *filter, float *input, float *measurements, float dt);

extern void linear_filter(float* X, float dt, float *dX, float* A);


extern void linear_measure(float*X, float* Y, float *H);

extern void ekf_filter_new(ekf_filter* filter);

extern void ekf_filter_setup(
					ekf_filter *filter,
					float* Q,
					float* R,
          float t);

extern void ekf_filter_reset(ekf_filter *filter);


extern void ekf_filter_predict(ekf_filter *filter);


extern void ekf_filter_update(ekf_filter *filter);

extern void ekf_filter_get_state(ekf_filter* filter, float *X, float* P);

extern void updateEkfFilter(ekf_filter *filter, float *input, float *measurements, float dt);


#endif /* EKF_H */
