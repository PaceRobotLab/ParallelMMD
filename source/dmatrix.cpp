#include "brMemalloc.h"
#include "linalg.h"
#include "dmatrix.h"
#include <math.h>

dMatrix::dMatrix(int nrows, int ncols)
{
	this->nrows = nrows;
	this->ncols = ncols;
	a = alloc2Ddouble(ncols, nrows);
}

dMatrix::~dMatrix()
{
//	free2D(a);
}

dMatrix dMatrix::inverse()
{
	dMatrix ia = dMatrix(nrows, ncols);
	double** temp = alloc2Ddouble(ncols, nrows);
	int i, j, n = nrows;
	for(i=0;i<n;i++)
		for(j=0;j<n;j++) temp[i][j] = a[i][j];

	invert_matrix(temp, n, ia.a);
	free2D(temp);
	return ia;
}

dMatrix tpt2rot(double tilt, double dp, double dt)
{
dMatrix R = mult_matrix(eulerang(tilt, 'x'), mult_matrix(eulerang(dp, 'y'), eulerang(-(tilt+dt), 'x')));

return R;
}

dMatrix eulerang(double angle, char axis)
{
	double ca, sa;
	ca = cos(angle);
	sa = sin(angle);
	dMatrix* M = new dMatrix(3,3);
	double** R = M->get_array();

	switch(axis)
	{
	case 'x':
		R[0][0] = 1.0;
		R[1][1] = R[2][2] = ca;
		R[1][2] = -sa; R[2][1] = sa;
		break;
	case 'y':
		R[1][1] = 1.0;
		R[0][0] = R[2][2] = ca;
		R[0][2] = sa; R[2][0] = -sa;
		break;
	case 'z':
		R[2][2] = 1.0;
		R[0][0] = R[1][1] = ca;
		R[0][1] = -sa; R[1][0] = sa;
	}

	return *M;
}

dMatrix mult_matrix(dMatrix A, dMatrix B)
{
	int i,j,k,m,n,l;
	m = A.get_nrows();
	l = A.get_ncols();
	n = A.get_ncols();
	dMatrix* C = new dMatrix(m,n);
	double** a = A.get_array();
	double** b = B.get_array();
	double** c = C->get_array();

	for(i=0;i<m;i++)
	for(j=0;j<n;j++)
	{
		c[i][j] = 0;
		for(k=0;k<l;k++)
			c[i][j]+= a[i][k]*b[k][j];
	}

	return *C;
}

dMatrix *mult_matrixp(dMatrix A, dMatrix B)
{
	int i,j,k,m,n,l;
	m = A.get_nrows();
	l = A.get_ncols();
	n = A.get_ncols();
	dMatrix* C = new dMatrix(m,n);
	double** a = A.get_array();
	double** b = B.get_array();
	double** c = C->get_array();

	for(i=0;i<m;i++)
	for(j=0;j<n;j++)
	{
		c[i][j] = 0;
		for(k=0;k<l;k++)
			c[i][j]+= a[i][k]*b[k][j];
	}

	return C;
}
