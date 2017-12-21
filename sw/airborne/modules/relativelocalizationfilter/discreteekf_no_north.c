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
 * @file "modules/relativelocalizationfilter/discreteekf.c"
 * @author Steven van der Helm / Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 */

#include "discreteekf_no_north.h"


/* Creates a basic Extended Kalman filter with
Zero intial state
P, Q, R as identity matrices
*/
void ekf_filter_new(ekf_filter *filter)
{
	fmat_make_identity(filter->P, EKF_N); // Make identity matrix
	fmat_make_identity(filter->Q, EKF_N); // Make identity matrix
	fmat_make_identity(filter->R, EKF_M); // Make identity matrix
	fmat_make_zeros(filter->X, EKF_N, 1); // Initial state
	filter->dt = 1; // Unitary time difference
}

/* Externally sets up the Q and R matrices of the EKF */
void ekf_filter_setup(
	ekf_filter *filter,
					float* Q,
					float* R,
					float t)
{
	memcpy(filter->Q, Q, EKF_N * EKF_N * sizeof(float));
	memcpy(filter->R, R, EKF_M * EKF_M * sizeof(float));
	filter->dt = t;
}

/* Resets the state voidector and covariance matrices of the EKF */
void ekf_filter_reset(ekf_filter *filter)
{
	fmat_make_identity(filter->P, EKF_N); // Make identity matrix
	fmat_make_zeros(filter->X, EKF_N, 1); // Initial state
}

/* Extract the latest state vector and the covariance matrix of the EKF*/
void ekf_filter_get_state(ekf_filter* filter, float *X, float* P){
	memcpy(X, filter->X, EKF_N * sizeof(float));
	memcpy(P, filter->P, EKF_N * EKF_N * sizeof(float));
}



extern void ekf_set_vals(ekf_filter *filter, float *input, float *measurements, float dt){
	filter->U = input;
	filter->Zm_in = measurements;
	filter->dt = dt;
}

/* Perform the prediction step
	PREDICT:
		Predict state
			x_p = f(x);
			A = Jacobian of f(x)

		Predict P
			P = A * P * A' + Q;

		Predict measure
			z_p = h(x_p)
			H = Jacobian of h(x)
*/
void ekf_filter_predict(ekf_filter* filter) {
	ekf_fsym(filter->X,filter->U,filter->dX);
	ekf_Fx(filter->X,filter->U,filter->Fx);
	ekf_G(filter->X,filter->U,filter->G);
	fmat_scal_mult(EKF_N,1,filter->dX,filter->dt,filter->dX);
	fmat_add(EKF_N,1,filter->Xp,filter->X,filter->dX);

	c2d(EKF_N,EKF_N,EKF_L,filter->Fx,filter->G,filter->dt,filter->Phi,filter->Gamma);

	//fmat_print(EKF_N,EKF_L,filter->Gamma);
	//fmat_print(EKF_N,EKF_N,filter->Phi);

	fmat_mult(EKF_N, EKF_N, EKF_N, filter->tmp1, filter->Phi, filter->P); // tmp1 <- Phi*P
	fmat_transpose(EKF_N, EKF_N, filter->tmp2, filter->Phi); // tmp2 <- Phi'
	fmat_mult(EKF_N, EKF_N, EKF_N, filter->tmp3, filter->tmp1, filter->tmp2); // tmp3 <- Phi*P*Phi'

	fmat_mult(EKF_N,EKF_L,EKF_L,filter->tmp1,filter->Gamma,filter->Q); // tmp1 <- Gamma*Q
	fmat_transpose(EKF_N, EKF_L, filter->tmp2, filter->Gamma); // tmp2 <- Gamma'
	fmat_mult(EKF_N, EKF_L, EKF_N, filter->tmp4, filter->tmp1, filter->tmp2); // tmp4 <- Gamma*Q*Gamma'

	fmat_add(EKF_N, EKF_N, filter->P, filter->tmp3, filter->tmp4); // Phi*P*Phi' + Gamma*Q*Gamma'
	//fmat_print(EKF_N,EKF_N,filter->P);

	ekf_hsym(filter->Xp,filter->Zp);
	ekf_Hx(filter->Xp,filter->H);

}
//#endif

/* Perform the update step
	UPDATE:
		Get Kalman Gain
			P12 = P * H';
			K = P12/(H * P12 + R);

		Update x
			x = x_p + K * (z - z_p);

		Update P
			P = (eye(numel(x)) - K * H) * P;
*/
void ekf_filter_update(ekf_filter* filter) {

	/*  E = H * P * H' + R */
	fmat_transpose(EKF_M, EKF_N, filter->tmp1, filter->H); // tmp1 <- H'
	fmat_mult(EKF_N, EKF_N, EKF_M, filter->tmp2, filter->P, filter->tmp1); // tmp2 <- P*H'
	fmat_mult(EKF_M, EKF_N, EKF_M, filter->tmp1, filter->H, filter->tmp2); // tmp1 <- H*P*H'
	fmat_add(EKF_M, EKF_M, filter->tmp2, filter->tmp1, filter->R); // tmp2 <- E = H*P*H' + R

	/* Get Kalman gain K = P * H' * inv(E) */
	fmat_inverse(EKF_M, filter->tmp1, filter->tmp2); // tmp1 <- inv(E)
	fmat_transpose(EKF_M, EKF_N, filter->tmp2, filter->H); // tmp2 <- H'
	fmat_mult(EKF_N, EKF_N, EKF_M, filter->tmp3, filter->P, filter->tmp2);  // tmp3 <- P*H'
	fmat_mult(EKF_N, EKF_M, EKF_M, filter->tmp2, filter->tmp3, filter->tmp1); // tmp2 <- K = P*H'*inv(E)

	/* P = P - K * H * P */
	fmat_mult(EKF_N, EKF_M, EKF_N, filter->tmp1, filter->tmp2, filter->H); // tmp1 <- K*H
	fmat_mult(EKF_N, EKF_N, EKF_N, filter->tmp3, filter->tmp1, filter->P); // tmp3 <- K*H*P
	fmat_sub(EKF_N, EKF_N, filter->P, filter->P, filter->tmp3); // P = P (1 - K*H)

	/*  X = Xp + K * err */
	fmat_sub(EKF_M, 1, filter->tmp1, filter->Zm_in, filter->Zp); // err = Y - Yp
	fmat_mult(EKF_N, EKF_M, 1, filter->tmp3, filter->tmp2, filter->tmp1); // K*err
	fmat_add(EKF_N, 1, filter->X, filter->Xp, filter->tmp3); // X = Xp + K*err

}

/* Linearized (Jacobian) filter function */
void linear_filter(float* X, float dt, float *dX, float* A)
{

	/* dX */
	// Make a zero vector
	fmat_make_zeros(dX,EKF_N,1);
	dX[0] = -(X[2] - X[4])*(dt);
	dX[1] = -(X[3] - X[5])*(dt);

	/* F'(x) */
	// Make an identity matrix
	fmat_make_identity(A,EKF_N);
	A[0*EKF_N+2] = -dt;
	A[0*EKF_N+4] =  dt;

	A[1*EKF_N+3] = -dt;
	A[1*EKF_N+5] =  dt;
};

/* Linearized (Jacobian) measure function */
void linear_measure(float*X, float* Y, float *H)
{
	int row, col;

	// RSSI measurement
	Y[0] = sqrt(pow(X[0],2.0) + pow(X[1],2.0) + pow(X[6],2.0));

	// x velocity of i (north)
	Y[1] = X[2];

	// y velocity of i (east)
	Y[2] = X[3];

	// x velocity of j (east)
	Y[3] = X[4];

	// y velocity of j (east)
	Y[4] = X[5];

	// Height difference
	Y[5] = X[6];

	// Generate the Jacobian Matrix
	for (row = 0 ; row < EKF_M ; row++ )
	{
		for (col = 0 ; col < EKF_N ; col++ )
		{
			// x, y, and z pos columns are affected by the RSSI
			if ((row == 0) && (col == 0 || col == 1 || col == 6 )) {
				H[ row*EKF_N+col ] = X[col]/sqrt((pow(X[0],2.0) + pow(X[1],2.0) + pow(X[6],2.0)));
			}

			// All other values are 1
			else if (((row == 1) && (col == 2)) ||
				((row == 2) && (col == 3)) ||
				((row == 3) && (col == 4)) ||
				((row == 4) && (col == 5)) ||
				((row == 5) && (col == 6)))
			{
				H[ row*EKF_N+col ] = 1.0;
			}

			else {
				H[ row*EKF_N+col ] = 0.0;
			}
		}
	}

};

void updateEkfFilter(ekf_filter *filter, float *input, float *measurements, float dt){
	ekf_set_vals(filter,input,measurements,dt);
	ekf_filter_predict(filter);
	ekf_filter_update(filter);
}
