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
 * @file "modules/relativelocalizationfilter/fmatrix.c"
 * @author Steven van der Helm / Mario Coppola
 * C matrix functions
 */

#include "fmatrix.h"
#include <math.h>


void fmat_make_ones(float *matrix, int row, int col){
	int i,j;
	for(i = 0 ; i < row; i++)
	{
		for(j = 0 ; j < col; j++)
		{
			matrix[i*col+j] = 1.0;
		}
	}
}

void fmat_make_diagonal(int n, float *matrix, float *diagvalue){
	fmat_make_zeros(matrix,n,n);
	for(int row=0;row<n;row++){
		for (int col=0;col<n;col++){
			if(row==col)
				fmat_assign(row,col,n,matrix,diagvalue[row]);
		}
	}
}

void fmat_assign(int row, int col, int n_col, float *matrix, float value){
	matrix[row*n_col+col] = value;
}

/* Copy inmat of size [m x n]  into outmat of size [m x n] */
void fmat_copy(int m, int n, float *inmat, float *outmat){
	{
		int j;
		for ( j = 0; j < n*m; j++ )
		{
			outmat[j] = inmat[j];
		}
	}
}

void fmat_invmult(int n1, int n2, float *a, float *b, float *outmat){
	float temp[n1*n1];
	fmat_inverse(n1, temp,a);
	fmat_mult_cop(n1,n1,n2,outmat,temp,b);
}

int fmat_max_i(int a, int b){
	int value = a<b ? b : a;
	return value;
}


float fmat_max_f(float a, float b){
	float value = a<b ? b : a;
	return value;
}

float fmat_log_2(float x){
	{
		float value;

		if ( x == 0.0 )
		{
			value = - 1.0E+30;
		}
		else
		{
			value = log ( fabs ( x ) ) / log ( 2.0 );
		}

		return value;
	}
}

float fmat_norm_li( int m, int n, float *a ){
	float row_sum;
	float value;

	value = 0.0;

	for ( int row = 0; row < m; row++ )
	{
		row_sum = 0.0;
		for ( int col = 0; col < n; col++ )
		{
			row_sum = row_sum + fabs ( a[row*m+col] );
		}
		value = fmat_max_f( value, row_sum );
	}
	return value;
}

void fmat_expm(int n, float *a, float *expa){
	float a_norm;
	float c;
	float d[n*n];
	int ee;
	int k;
	//const float one = 1.0;
	int p;
	const int q = 6;
	int s;
	float t;
	float x[n*n];
	float a2[n*n];
	//float temp[n*n];

	fmat_copy ( n, n, a, a2 );

	a_norm = fmat_norm_li ( n, n, a2 );

	ee = ( int ) ( fmat_log_2 ( a_norm ) ) + 1;

	s = fmat_max_i( 0, ee + 1 );

	t = 1.0 / pow ( 2.0, s );

	fmat_scal_mult( n, n, a2, t, a2 );

	fmat_copy ( n, n, a2, x );

	c = 0.5;

	fmat_make_identity ( expa, n);

	fmat_add_scal_mult ( n, n, expa, expa, c, a2);

	fmat_make_identity ( d, n);

	fmat_add_scal_mult ( n, n, d, d, -c, a2);



	p = 1;

	for ( k = 2; k <= q; k++ )
	{
		c = c * ( float ) ( q - k + 1 ) / ( float ) ( k * ( 2 * q - k + 1 ) );

		fmat_mult_cop(n, n, n, x, x, a2);

		fmat_add_scal_mult(n,n,expa,expa,c,x);

		if ( p )
		{
			fmat_add_scal_mult(n,n,d,d,c,x);
		}
		else
		{
			fmat_add_scal_mult(n,n,d,d,-c,x);
		}

		p = !p;
	}

	/*
	   E -> inverse(D) * E
	 */
	fmat_invmult(n,n,d,expa,expa);



	/*
	   E -> E^(2*S)
	 */
	 for ( k = 1; k <= s; k++ )
	 {
		 fmat_mult_cop(n,n,n,expa,expa,expa);
	 }


}

/* Function to add two matrices to eachother */
void fmat_add(int n_row, int n_col, float* r, float* a, float* b) {
	int row, col, ridx;
	for (row = 0; row<n_row; row++)
	{
		for (col = 0; col<n_col; col++)
		{
			ridx = row * n_col + col;
			r[ridx] = a[ridx] + b[ridx];
		}
	}
}

/* Function to subtract two matrices from eachother */
void fmat_sub(int n_row, int n_col, float* r, float* a, float* b) {
	int row, col, ridx;
	for (row = 0; row<n_row; row++)
	{
		for (col = 0; col<n_col; col++)
		{
			ridx = row * n_col + col;
			r[ridx] = a[ridx] - b[ridx];
		}
	}
}

/* Function to obtain the transpose of a matrix */
void fmat_transpose(int n_row, int n_col, float* r, float* a) {
	int row, col, ridx, aidx;
	for (row = 0; row<n_row; row++)
	{
		for (col = 0; col<n_col; col++)
		{
			aidx = row * n_col + col;
			ridx = col * n_row + row;
			r[ridx] = a[aidx];
		}
	}
}

/* Function to multiply a matrix by a scalar value */
void fmat_scal_mult(int n_row, int n_col, float* r, float k, float* a) {
	int row, col, ridx;
	for (row = 0; row < n_row; row++)
	{
		for (col = 0; col < n_col; col++)
		{
			ridx = row * n_col + col;
			if (a[ridx] != 0.0)
				r[ridx] = k * a[ridx];
			else
				r[ridx] = 0.0;
		}
	}
}



/* Add a scalar value to a matrix */
void fmat_add_scal_mult(int n_row, int n_col, float* r, float*a, float k, float* b) {
	int row, col, ridx;
	for (row = 0; row < n_row; row++)
	{
		for (col = 0; col < n_col; col++)
		{
			ridx = row * n_col + col;
			r[ridx] = a[ridx] + k * b[ridx];
		}
	}
}

/* Multiply two matrices with eachother */
void fmat_mult(int n_rowa, int n_cola, int n_colb, float* r, float* a, float* b) {
	int row, col, k, ridx, aidx, bidx;
	for (row = 0; row < n_rowa; row++)
	{
		for (col = 0; col < n_colb; col++)
		{
			ridx = col + row * n_colb;
			r[ridx] =0.;
			for (k=0; k < n_cola; k++)
			{
				aidx = k + row * n_cola;
				bidx = col + k * n_colb;
				r[ridx] += a[aidx] * b[bidx];
			}
		}
	}

}

/* Multiply two matrices with eachother, and use a temporary array to store result. This allows
 * One to store the resulting matrix in one of the input matrices for consecutive multiplications
 * (e.g. when doing matrix exponentiation), at the cost of some copy overhead. */
void fmat_mult_cop(int n_rowa, int n_cola, int n_colb, float* r, float* a, float* b) {
	float temp[n_rowa*n_colb];
	int row, col, k, ridx, aidx, bidx;
	for (row = 0; row < n_rowa; row++)
	{
		for (col = 0; col < n_colb; col++)
		{
			ridx = col + row * n_colb;
			temp[ridx] =0.;
			for (k=0; k < n_cola; k++)
			{
				aidx = k + row * n_cola;
				bidx = col + k * n_colb;
				temp[ridx] += a[aidx] * b[bidx];
			}
		}
	}
	fmat_copy(n_rowa,n_colb,temp,r);
}

#ifndef ARM_COMPILER

/* Print the matrix */
void fmat_print(int n_row, int n_col, float* a)
{
	int row, col, ridx;
	for (row = 0; row < n_row; row++)
	{
		for (col = 0; col < n_col; col++)
		{
			ridx = row * n_col + col;
			printf("%2.4f\t", a[ridx]);
		}
		printf("\n");
	}
	printf("\n");
}

/* Print the matrix
void fmat_print(int n_row, int n_col, double* a)
{
	int row, col, ridx;
	for (row = 0; row < n_row; row++)
	{
		for (col = 0; col < n_col; col++)
		{
			ridx = row * n_col + col;
			printf("%2.2f\t", a[ridx]);
		}
		printf("\n");
	}
	printf("\n");
}
 */

#endif

/* Calculate inverse of matrix

Compliments of:
https://www.quora.com/How-do-I-make-a-C++-program-to-get-the-inverse-of-a-matrix-100-X-100

Algorithm verified with Matlab
 */
void fmat_inverse(int n, float* matinv, float *mat)
{
	int i, j, k, row, col;
	float t;

	float a[2*n*n];

	/* Append an ide1ntity matrix on the right of the original matrix */
	for(row = 0; row < n; row++)
	{
		for(col = 0; col < 2*n; col++)
		{
			if (col < n)
			{
				a[ row*2*n+col ] = mat[row*n + col];
			}
			else if( (col >= n) && (col == row+n))
			{
				a[ row*2*n+col ] = 1.0;
			}
			else
			{
				a[ row*2*n+col ] = 0.0;
			}
		}
	}

	/* Do the inversion */
	for( i = 0; i < n; i++)
	{
		// Store diagonal variable (temp)
		t = a[ i*2*n + i ];

		for(j = i; j < 2*n; j++)
		{
			// Divide by the diagonal value
			a[ i*2*n + j ] = a[ i*2*n + j ]/t;
		}

		for(j = 0; j < n; j++)

		{
			if( i!=j )
			{
				t = a[ j*2*n + i ];

				for( k = 0; k < 2*n; k++)
				{
					a[j*2*n + k] = a[j*2*n + k] - t*a[i*2*n + k];
				}

			}

		}

	}

	/* Cut out the identity, which has now moved to the left side */
	for(row = 0 ; row < n ; row++ )
	{
		for(col = n; col < 2*n; col++ )
		{
			matinv[row * n + col - n] = a[row*2*n + col];
		}

	}

};

/* Make a matrix of zeros */
void fmat_make_zeros(float *matrix, int row, int col)
{
	int i,j;
	for(i = 0 ; i < row; i++)
	{
		for(j = 0 ; j < col; j++)
		{
			matrix[i*col+j] = 0.0;
		}
	}
};

/* Make a matrix of zeros */
void fmat_make_zeroes(float *matrix, int row, int col)
{
	int i,j;
	for(i = 0 ; i < row; i++)
	{
		for(j = 0 ; j < col; j++)
		{
			matrix[i*col+j] = 0.0;
		}
	}
};

/* Make an identity matrix */
void fmat_make_identity(float *matrix, int n)
{
	int i,j;
	for(i = 0 ; i < n; i++)
	{
		for(j = 0 ; j < n; j++)
		{
			if (i == j)
			{
				matrix[i*n+j] = 1.0;
			}
			else
			{
				matrix[i*n+j] = 0.0;
			}
		}
	}
};

void extractPhiGamma(int n_row, int n_colA, int n_colB, float *inmat, float *phi, float *gamma){
	int totalsize = n_row+n_colB;
	for(int row = 0; row<totalsize;row++){
		for (int col = 0; col<totalsize;col++){
			if(row<n_row && col<n_colA){
				phi[col+row*n_colA] = *inmat;
			}
			else if(row<n_row && col>=n_colA){
				gamma[(col-n_colA)+row*n_colB] = *inmat;
			}
			inmat++;
		}
	}
}

void combineMatrices(int n_row, int n_colA, int n_colB, float *A, float *B, float *combmat){
	int totalsize = n_row+n_colB;
	for (int row =0; row<totalsize; row++){
		for (int col = 0; col<totalsize;col++){
			if ((row<n_row) && col<n_colA){
				combmat[col+row*totalsize]=*A;
				A++;
			}
			else if ((row<n_row) && (col>=n_colA) ){
				combmat[col+row*totalsize]= *B;
				B++;
			}
			else{
				combmat[col+row*totalsize]=0.0;
			}
		}
	}
}

void c2d(int n_row, int n_colA, int n_colB, float *A, float *B, float dt, float *phi, float *gamma){
	int totalsize = n_row+n_colB;
	float combmat[totalsize*totalsize];
	float expm[totalsize*totalsize];
	fmat_scal_mult(n_row,n_colA,A,dt,A);
	fmat_scal_mult(n_row,n_colB,B,dt,B);
	combineMatrices(n_row,n_colA,n_colB,A,B,combmat);
	fmat_expm(totalsize,combmat,expm);
	extractPhiGamma(n_row,n_colA,n_colB,expm,phi,gamma);
}

