#include <math.h>
#include "affine.h"
#include "CornerClass.h"
#include "brMemalloc.h"
#include "minmax.h"
#include "linalg.h"


void FindAffineMatrix(int matches[][4], int n, dMatrix U, dMatrix K)
{
	double **A,** b;
	int i,j, k;
	double x1, x2, y1, y2, dx, dy;
	double sx, sy, xc, yc;
	sx = K.a[0][0]; xc = K.a[0][2]; sy = K.a[1][1]; yc = K.a[1][2];
	A = alloc2Ddouble(3,3);
	b = alloc2Ddouble(2,3);

	int dmatches[4][4] = {{147,99,84,68}, {249,170,190,132},{160,139,104,107},{306,141,243,99}};


	// Estimating Eigenvalues of A'A

	for(k=0;k<n;k++)
	{
		x1 = (double)(sx*matches[k][0] + xc);
		y1 = (double)(sy*matches[k][1] + yc);
		x2 = (double)(sx*matches[k][2] + xc);
		y2 = (double)(sy*matches[k][3] + yc);
		dx = x2 - x1;
		dy = y2 - y1;

		A[0][0] += x1*x1;
		A[0][1] += x1*y1;
		A[0][2] += x1;
		A[1][1] += y1*y1;
		A[1][2] += y1;
		b[0][0] += dx*x1;
		b[1][0] += dx*y1;
		b[2][0] += dx;
		b[0][1] += dy*x1;
		b[1][1] += dy*y1;
		b[2][1] += dy;
	}

	A[2][2] = n;
	A[1][0] = A[0][1];
	A[2][0] = A[0][2];
	A[2][1] = A[1][2];
	
	gaussj(A, 3, b, 2);

	U.set2identity();
	for(i=0;i<2;i++)for(j=0;j<3;j++)U.a[i][j]+= b[j][i];
	U = mult_matrix(K.inverse(),mult_matrix(U, K));
}


void FindAffineMatrix(int matches[][4], int n, dMatrix U)
{
	double **A,** b;
	int i,j, k;
	double x1, x2, y1, y2, dx, dy;
	i=2, j=3;
	A = alloc2Ddouble(3,3);
	b = alloc2Ddouble(i,j);
	double* p = b[0];
	int dmatches[4][4] = {{147,99,84,68}, {249,170,190,132},{160,139,104,107},{306,141,243,99}};


	// Estimating Eigenvalues of A'A

	for(k=0;k<n;k++)
	{
		x1 = (double)(matches[k][0]);
		y1 = (double)(matches[k][1]);
		x2 = (double)(matches[k][2]);
		y2 = (double)(matches[k][3]);
		dx = x2 - x1;
		dy = y2 - y1;

		A[0][0] += x1*x1;
		A[0][1] += x1*y1;
		A[0][2] += x1;
		A[1][1] += y1*y1;
		A[1][2] += y1;
		b[0][0] += dx*x1;
		b[1][0] += dx*y1;
		b[2][0] += dx;
		b[0][1] += dy*x1;
		b[1][1] += dy*y1;
		b[2][1] += dy;
	}

	A[2][2] = n;
	A[1][0] = A[0][1];
	A[2][0] = A[0][2];
	A[2][1] = A[1][2];
	
	gaussj(A, 3, b, 2);

	U.set2identity();
	for(i=0;i<2;i++)for(j=0;j<3;j++)U.a[i][j]+= b[j][i];

}

void FindAffineMatrix(int matches[][4], int n, dMatrix U, int* w)
{
	double **A,** b;
	int i,j, k,m=0;
	double x1, x2, y1, y2, dx, dy;
	i=2, j=3;
	A = alloc2Ddouble(3,3);
	b = alloc2Ddouble(i,j);
	double* p = b[0];
	int dmatches[4][4] = {{147,99,84,68}, {249,170,190,132},{160,139,104,107},{306,141,243,99}};


	// Estimating Eigenvalues of A'A

	for(k=0;k<n;k++)
	if(w[k] == 1)
	{
		x1 = (double)(matches[k][0]);
		y1 = (double)(matches[k][1]);
		x2 = (double)(matches[k][2]);
		y2 = (double)(matches[k][3]);
		dx = x2 - x1;
		dy = y2 - y1;

		A[0][0] += x1*x1;
		A[0][1] += x1*y1;
		A[0][2] += x1;
		A[1][1] += y1*y1;
		A[1][2] += y1;
		b[0][0] += dx*x1;
		b[1][0] += dx*y1;
		b[2][0] += dx;
		b[0][1] += dy*x1;
		b[1][1] += dy*y1;
		b[2][1] += dy;
		m++;
	}

	A[2][2] = m;
	A[1][0] = A[0][1];
	A[2][0] = A[0][2];
	A[2][1] = A[1][2];
	
	gaussj(A, 3, b, 2);

	U.set2identity();
	for(i=0;i<2;i++)for(j=0;j<3;j++)U.a[i][j]+= b[j][i];

}


float FindAffineMatrix3(int matches[][4], int n, dMatrix U, float* error)
{
// This function computes 
//	1.	Affine matrix of the form
//
//			U = [cos(alpha)	sin(alpha)	u;
//				-sin(alpha)	cos(alpha)	v;
//					0			0		1];
//
//	2.	total error, i.e sum((x2-x1)^2+(y2-y2)^2)
//
//	and alternatevely, the error vector
	
	int k;
	float mx1, my1, mx2, my2, x1[400], y1[400], x2[400],y2[400],u,v, s, c;
	double alpha;
	
//	Estimating mean of the point matches coordinates

	mx1 = my1 = mx2 = my2 = 0.0f;

	for(k=0;k<n;k++)
	{
		mx1+= matches[k][0];
		my1+= matches[k][1];
		mx2+= matches[k][2];
		my2+= matches[k][3];
	}
	mx1/=n;
	my1/=n;
	mx2/=n;
	my2/=n;

	for(k=0;k<n;k++)
	{
		x1[k] = matches[k][0] - mx1;
		y1[k] = matches[k][1] - my1;
		x2[k] = matches[k][2] - mx2;
		y2[k] = matches[k][3] - my2;
	}

//	Estimating alpha
	float t1 = 0, t2 = 0;
	for(k=0;k<n;k++)
	{
		t1+= x2[k]*y1[k] - x1[k]*y2[k];
		t2+= x1[k]*x2[k] + y1[k]*y2[k];
	}

	if(t2 == 0)		//this shoild never happen!!
		alpha = 0.0;
	else
		alpha = atan(t1/t2);

//	Estimating u and v
	c = float(cos(alpha));
	s = float(sin(alpha));
	u = mx2 - mx1*c - my1*s;
	v = my2 + mx1*s - my1*c;

	U.a[0][0] = c;	U.a[0][1] = s;	U.a[0][2] = u;
	U.a[1][0] = -s;	U.a[1][1] = c;	U.a[1][2] = v;
	U.a[2][0] =	U.a[2][1] = 0.0;	U.a[2][2] = 1.0;

//	Calculating total error
	float dx, dy, err = 0;

	if(error == 0)
		for(k=0;k<n;k++)
		{
			dx = matches[k][2] - matches[k][0]*c - matches[k][1]*s - u;
			dy = matches[k][3] + matches[k][0]*s - matches[k][1]*c - v;
			err+= dx*dx + dy*dy;
		}
	else
		for(k=0;k<n;k++)
		{
			dx = matches[k][2] - matches[k][0]*c - matches[k][1]*s - u;
			dy = matches[k][3] + matches[k][0]*s - matches[k][1]*c - v;
			error[k] = dx*dx + dy*dy;
			err+= error[k];
		}

	return err/n;
}


void affine_error(int matches[][4], float* err, int n, double** U, float thr, int* w, int* sum_w)
{
	int x1, y1, x2, y2, i;
	float dx, dy;
	float a[6];

	for(i=0;i<3;i++)
	{
		a[i] = (float)U[0][i];
		a[i+3] = (float) U[1][i];
	}

	for(i=0;i<n;i++)
	{
	  x1 = matches[i][0];
	  y1 = matches[i][1];
	  x2 = matches[i][2];
	  y2 = matches[i][3];
	  dx = x2 - a[0]*x1 - a[1]*y1 - a[2];
	  dy = y2 - a[3]*x1 - a[4]*y1 - a[5];
	  err[i] = dx*dx + dy*dy;
	}

	if(w != 0)
	{
		(*sum_w) = 0;
		for(i=0;i<n;i++)
		if(err[i]<thr)
		{
			w[i] = 1;
			(*sum_w)++;
		}
		else
			w[i] = 0;
	}
}

