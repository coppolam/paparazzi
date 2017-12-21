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
 * @file "modules/relativelocalizationfilter/fmatrix.h"
 * @author Steven van der Helm / Mario Coppola
 * C matrix functions
 */

#ifndef FMATRIX_H
#define FMATRIX_H

#ifndef ARM_COMPILER
#include <stdio.h> // needed for the printf statements
#endif /*ARM_COMPILER*/

extern void fmat_add(int n_row, int n_col, float* r, float* a, float* b);
extern void fmat_sub(int n_row, int n_col, float* r, float* a, float* b);
extern void fmat_transpose(int n_row, int n_col, float* r, float* a);

extern void fmat_scal_mult(int n_row, int n_col, float* r, float k, float* a);
extern void fmat_add_scal_mult(int n_row, int n_col, float* r, float*a, float k, float* b);
extern void fmat_mult(int n_rowa, int n_cola, int n_colb, float* r, float* a, float* b);
extern void fmat_mult_cop(int n_rowa, int n_cola, int n_colb, float* r, float* a, float* b);
extern void fmat_invmult(int n1, int n2, float *a, float *b, float *outmat);
extern void fmat_inverse(int n, float* inv, float* a);
extern void fmat_make_zeros(float *matrix, int row, int col);
extern void fmat_make_ones(float *matrix, int row, int col);
extern void fmat_make_identity(float *matrix, int n);
extern void fmat_copy(int m, int n, float *inmat, float *outmat);
extern float fmat_norm_li( int m, int n, float *a );
extern float fmat_max_f (float a, float b);
extern int fmat_max_i(int a, int b);
extern float fmat_log_2(float x);
extern void fmat_expm(int n, float *a, float *expa);
extern void fmat_assign(int row, int col, int n_col,  float *matrix, float value);
extern void fmat_make_diagonal(int n, float *matrix, float *diagvalue);

extern void extractPhiGamma(int n_row, int n_colA, int n_colB, float *inmat, float *phi, float *gamma);
extern void combineMatrices(int n_row, int n_colA, int n_colB, float *A, float *B, float *combmat);
extern void c2d(int n_row, int n_colA, int n_colB, float *A, float *B, float dt, float *phi, float *gamma);

#ifndef ARM_COMPILER
extern void fmat_print(int n_row, int n_col, float* a);
#endif

#endif /* FMATRIX_H */
