#include "discreteekf.h"

/* Creates a basic Extended Kalman filter with 
Zero intial state
P, Q, R as identity matrices 
*/
void ekf_filter_new(ekf_filter *filter)
{
	fmat_make_identity(filter->P, N); // Make identity matrix
	fmat_make_identity(filter->Q, N); // Make identity matrix
	fmat_make_identity(filter->R, M); // Make identity matrix
	fmat_make_zeroes(filter->X, N, 1); // Initial state
	filter->dt = 1; // Unitary time difference
}

/* Externally sets up the Q and R matrices of the EKF */
void ekf_filter_setup(
	ekf_filter *filter,
					float* Q,
					float* R,
					float t)
{
	memcpy(filter->Q, Q, N * N * sizeof(float));
	memcpy(filter->R, R, M * M * sizeof(float));
	filter->dt = t;
}

/* Resets the state voidector and covariance matrices of the EKF */
void ekf_filter_reset(ekf_filter *filter)
{
	fmat_make_identity(filter->P, N); // Make identity matrix
	fmat_make_zeroes(filter->X, N, 1); // Initial state
}

/* Extract the latest state vector and the covariance matrix of the EKF*/
void ekf_filter_get_state(ekf_filter* filter, float *X, float* P){
	memcpy(X, filter->X, N * sizeof(float));
	memcpy(P, filter->P, N * N * sizeof(float));
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
void ekf_filter_predict(ekf_filter* filter, btmodel* model) {

	// Fetch dt, dX and A given the current state X and input u
	linear_filter(filter->X, filter->dt, filter->tmp1, filter->tmp3);

	// Get state prediction Xp = X + dX
	fmat_add(N, 1, filter->Xp, filter->X, filter->tmp1); 

	// Get measurement prediction Zp based on Xp and get Jacobian H
	linear_measure(filter->Xp, filter->Zp, filter->H, model);

	//P = A * P * A' + Q
	fmat_mult(N, N, N, filter->tmp1, filter->tmp3, filter->P); // A*P
	fmat_transpose(N, N, filter->tmp2, filter->tmp3); // A'
	fmat_mult(N, N, N, filter->tmp3, filter->tmp1, filter->tmp2); // A*P*A'
	fmat_add(N, N, filter->P, filter->tmp3, filter->Q); // A*P*A' + Q

}

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
void ekf_filter_update(ekf_filter* filter, float *y) {

	/*  E = H * P * H' + R */
	fmat_transpose(M, N, filter->tmp1, filter->H); // H'
	fmat_mult(N, N, M, filter->tmp2, filter->P, filter->tmp1); // P*H'
	fmat_mult(M, N, M, filter->tmp1, filter->H, filter->tmp2); // H*P*H'
	fmat_add(M, M, filter->tmp2, filter->tmp1, filter->R); // E = H*P*H' + R

	/* Get Kalman gain K = P * H' * inv(E) */
	fmat_inverse(M, filter->tmp1, filter->tmp2); // inv(E)
	fmat_transpose(M, N, filter->tmp2, filter->H); // H'
	fmat_mult(N, N, M, filter->tmp3, filter->P, filter->tmp2);  // P*H'
	fmat_mult(N, M, M, filter->tmp2, filter->tmp3, filter->tmp1); // K = P*H'*inv(E)

	/* P = P - K * H * P */
	fmat_mult(N, M, N, filter->tmp1, filter->tmp2, filter->H); // K*H
	fmat_mult(N, N, N, filter->tmp3, filter->tmp1, filter->P); // K*H*P
	fmat_sub(N, N, filter->P, filter->P, filter->tmp3); //P = P (1 - K*H)
	
	/*  X = X + K * err */
	fmat_sub(M, 1, filter->tmp1, y, filter->Zp); // err = Z - Zp
	fmat_mult(N, M, 1, filter->tmp3, filter->tmp2, filter->tmp1); // K*err
	fmat_add(N, 1, filter->X, filter->Xp, filter->tmp3); // X = X + K*err

}

/* Linearized (Jacobian) filter function */
void linear_filter(float* X, float dt, float *dX, float* A)
{

	/* dX */
	// Make a zero vector
	fmat_make_zeroes(dX,N,1);
	dX[0] = -(X[2] - X[4])*(dt);
	dX[1] = -(X[3] - X[5])*(dt);
	
	/* F'(x) */
	// Make an identity matrix
	fmat_make_identity(A,N);
	A[0*N+2] = -dt;
	A[0*N+4] =  dt;

	A[1*N+3] = -dt;
	A[1*N+5] =  dt;
};

/* Linearized (Jacobian) measure function */
void linear_measure(float*X, float* Y, float *H, btmodel *model)
{
	int row, col;
	float Pn = model->Pn;
	float gamma = model->gammal;

	// RSSI measurement
	Y[0] = Pn - (10.0 * gamma * log10(sqrt(pow(X[0],2.0) + pow(X[1],2.0) + pow(X[8],2.0))));

	// x velocity of i wrt i body frame
	Y[1] = X[2];

	// y velocity of i wrt i body frame
	Y[2] = X[3];

	// Orientation of i wrt north
	Y[3] = X[6];

	// x velocity of j wrt j body frame
	Y[4] = cos( X[6] - X[7] ) * X[4] - sin( X[6] - X[7] ) * X[5];

	// y velocity of j wrt  j body frame
	Y[5] = sin( X[6] - X[7] ) * X[4] + cos( X[6] - X[7] ) * X[5];

	// Orientation of j wrt north
	Y[6] = X[7];

	// Height difference
	Y[7] = X[8];

	// Generate the Jacobian Matrix
	for (row = 0 ; row < M ; row++ )
	{
		for (col = 0 ; col < N ; col++ )
		{
			if ((row == 0) && (col == 0 || col == 1 || col == 8 ))
				H[ row*N+col ] = (-gamma*10/log(10))*(X[col]/(pow(X[0],2.0) + pow(X[1],2.0) + pow(X[8],2.0)));
			
			else if (((row == 1) && (col == 2)) ||
				((row == 2) && (col == 3)) ||
				((row == 3) && (col == 6)) ||
				((row == 6) && (col == 7)) ||
				((row == 7) && (col == 8)))
			{ 
				H[ row*N+col ] = 1.0;
			}

			else if ((row == 4) && (col == 4))
				H[ row*N+col ] = cos(X[6]-X[7]);
			else if ((row == 4) && (col == 5))
				H[ row*N+col ] = -sin(X[6]-X[7]);
			else if ((row == 4) && (col == 6))
				H[ row*N+col ] = X[4]*sin(X[7]-X[6]) - X[5] * cos(X[7] - X[6]);
			else if ((row == 4) && (col == 7))
				H[ row*N+col ] = X[4]*sin(X[6]-X[7]) + X[5] * cos(X[6] - X[7]);
			

			else if ((row == 5) && (col == 4))
				H[ row*N+col ] = sin(X[6]-X[7]);
			else if ((row == 5) && (col == 5))
				H[ row*N+col ] = cos(X[6]-X[7]);
			else if ((row == 5) && (col == 6))
				H[ row*N+col ] = X[4]*cos(X[7]-X[6]) + X[5] * sin(X[7] - X[6]);
			else if ((row == 5) && (col == 7))
				H[ row*N+col ] = -X[4]*cos(X[6]-X[7]) + X[5] * sin(X[6] - X[7]);

			else 
				H[ row*N+col ] = 0.0;
		}
	}

};
