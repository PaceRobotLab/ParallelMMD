#include <math.h>
#include "homography.h"
#include "CornerClass.h"
#include "brMemalloc.h"
#include "minmax.h"
#include "linalg.h"

void FindHomography(void* vpcl, dMatrix U)
{
	double P[3][3], **A,**H, eig[9];
	int i,j, k, nrot, ind1, n;
	int x1, x2, y1, y2, r;
	CORNER_LIST* cl = (CORNER_LIST*)vpcl;
	A = alloc2Ddouble(9,9);
	H = alloc2Ddouble(9,9);
	P[2][2] = 1;

	// Estimating Eigenvalues of A'A
	k = 0;
	n = cl->N;
	CORNER_M* pcm;

	for(i=0;i<n;i++)
	{
		pcm = &(cl->Corners[i]);
		if(pcm->index2>=0)
		{
			x1 = pcm->x;
			x2 = x1 + pcm->dx;
			y1 = pcm->y;
			y2 = y1 + pcm->dy;
			P[0][0] = x1*x1;
			P[0][1] = P[1][0] = x1*y1;
			P[0][2] = P[2][0] = x1;
			P[1][1] = y1*y1;
			P[1][2] = P[2][1] = y1;

			for(i=0;i<3;i++)
			for(j=0;j<3;j++)
			{
				r = x2*x2+y2*y2;
				A[i][j] = A[i+3][j+3] = P[i][j];
				A[i][j+6] = A[i+6][j] = -x2*P[i][j];
				A[i+3][j+6] = A[i+6][j+3] = -y2*P[i][j];
				A[i+6][j+6] = r*P[i][j];
			}
		}
	}

	jacobi(A, 9, eig, H, &nrot);
//	minim(eig, 9, &ind1);
	ind1 = 0;
	double mn = eig[0];
	for(i=1;i<9;i++)
		if(eig[i]<mn)
		{
			mn = eig[i];
			ind1 = i;
		}

	i=ind1;
	U.a[0][0] = H[0][i];
	U.a[0][1] = H[1][i];
	U.a[0][2] = H[2][i];
	U.a[1][0] = H[3][i];
	U.a[1][1] = H[4][i];
	U.a[1][2] = H[5][i];
	U.a[2][0] = H[6][i];
	U.a[2][1] = H[7][i];
	U.a[2][2] = H[8][i];
}

void FindHomography(int matches[][4], int n,dMatrix U)
{
	double P[3][3], **A,**H, eig[9];
	int i,j, k, nrot, ind1;
	double x1, x2, y1, y2, r, s=1;
	A = alloc2Ddouble(9,9);
	H = alloc2Ddouble(9,9);
	P[2][2] = 1;

	int dmatches[4][4] = {{147,99,84,68}, {249,170,190,132},{160,139,104,107},{306,141,243,99}};
	// scaling
	double xc1, yc1, fx1, fy1, xc2, yc2, fx2, fy2, mx1, my1, mx2, my2;
	xc1 = xc2 = yc1 = yc2 = fx1 = fy1 = fx2 = fy2 = mx1 = my1 = mx2 = my2 = 0.0;
	for(i=0;i<n;i++)
	{
		xc1+= matches[i][0];
		yc1+= matches[i][1];
		xc2+= matches[i][2];
		yc2+= matches[i][3];

		mx1+= matches[i][0]*matches[i][0];
		my1+= matches[i][1]*matches[i][1];
		mx2+= matches[i][2]*matches[i][2];
		my2+= matches[i][3]*matches[i][3];
	}
		
	xc1/= n;
	yc1/= n;
	xc2/= n;
	yc2/= n;

	fx1 = 0.5*(mx1/n - xc1*xc1);
	fy1 = 0.5*(my1/n - yc1*yc1);
	fx2 = 0.5*(mx2/n - xc2*xc2);
	fy2 = 0.5*(my2/n - yc2*yc2);

	dMatrix K1(3,3), K2(3,3), iK1(3,3), iK2(3,3);
	iK1.a[0][0] = fx1; iK1.a[0][2] = xc1; iK1.a[1][1] = fy1; iK1.a[1][2] = yc1; iK1.a[2][2] = 1.0;
	iK2.a[0][0] = fx2; iK2.a[0][2] = xc2; iK2.a[1][1] = fy2; iK2.a[1][2] = yc2; iK2.a[2][2] = 1.0;
	K1 = iK1.inverse();
	K2 = iK2.inverse();




	// Estimating Eigenvalues of A'A

	for(k=0;k<n;k++)
	{
		x1 = (matches[k][0] - xc1)/fx1;
		y1 = (matches[k][1] - yc1)/fy1;
		x2 = (matches[k][2] - xc2)/fx2;
		y2 = (matches[k][3] - yc2)/fy2;
		P[0][0] = x1*x1;
		P[0][1] = P[1][0] = x1*y1;
		P[0][2] = P[2][0] = x1;
		P[1][1] = y1*y1;
		P[1][2] = P[2][1] = y1;

		for(i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			r = x2*x2+y2*y2;
			A[i+3][j+3] += P[i][j];
			A[i+6][j]   += -x2*P[i][j];
			A[i+6][j+3] += -y2*P[i][j];
			A[i+6][j+6] += r*P[i][j];
		}
	}

	for(i=0;i<3;i++)
	for(j=0;j<3;j++)
	{
		A[i][j] = A[i+3][j+3];
		A[i][j+6] = A[i+6][j];
		A[i+3][j+6] = A[i+6][j+3];
	}

	
	jacobi(A, 9, eig, H, &nrot);
	//	minim(eig, 9, &ind1);
	ind1 = 0;
	double mn = eig[0];
	for(i=1;i<9;i++)
		if(eig[i]<mn)
		{
			mn = eig[i];
			ind1 = i;
		}

	i=ind1;
	double dU[3][3];
	dU[0][0] = H[0][i];
	dU[0][1] = H[1][i];
	dU[0][2] = H[2][i];
	dU[1][0] = H[3][i];
	dU[1][1] = H[4][i];
	dU[1][2] = H[5][i];
	dU[2][0] = H[6][i];
	dU[2][1] = H[7][i];
	dU[2][2] = H[8][i];

	double dtr = det3x3(dU);
	if(dtr<0)
	{
		dtr = -dtr;
		for(i=0;i<3;i++)for(j=0;j<3;j++)dU[i][j]=-dU[i][j];
	}

	double c = pow(dtr, 1.0/3.0);
	for(i=0;i<3;i++)for(j=0;j<3;j++)U.a[i][j]= dU[i][j] = dU[i][j]/c;
	U = mult_matrix(iK1,mult_matrix(U, K2));

}

void FindHomography(int matches[][4], int n, dMatrix U, dMatrix K1, dMatrix K2)
{
	double P[3][3], **A,**H, eig[9];
	int i,j, k, nrot, ind1;
	double x1, x2, y1, y2, r;
	double sx1, sy1, sx2, sy2, xc1, yc1, xc2, yc2;
	sx1 = K1.a[0][0]; xc1 = K1.a[0][2]; sy1 = K1.a[1][1]; yc1 = K1.a[1][2];
	sx2 = K2.a[0][0]; xc2 = K2.a[0][2]; sy2 = K2.a[1][1]; yc2 = K2.a[1][2];
	A = alloc2Ddouble(9,9);
	H = alloc2Ddouble(9,9);
	P[2][2] = 1;

	int dmatches[4][4] = {{147,99,84,68}, {249,170,190,132},{160,139,104,107},{306,141,243,99}};


	// Estimating Eigenvalues of A'A

	for(k=0;k<n;k++)
	{
		x1 = (double)(sx1*matches[k][0] + xc1);
		y1 = (double)(sy1*matches[k][1] + yc1);
		x2 = (double)(sx2*matches[k][2] + xc2);
		y2 = (double)(sy2*matches[k][3] + yc2);
		P[0][0] = x1*x1;
		P[0][1] = P[1][0] = x1*y1;
		P[0][2] = P[2][0] = x1;
		P[1][1] = y1*y1;
		P[1][2] = P[2][1] = y1;

		for(i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			r = x2*x2+y2*y2;
			A[i+3][j+3] += P[i][j];
			A[i+6][j]   += -x2*P[i][j];
			A[i+6][j+3] += -y2*P[i][j];
			A[i+6][j+6] += r*P[i][j];
		}
	}

	for(i=0;i<3;i++)
	for(j=0;j<3;j++)
	{
		A[i][j] = A[i+3][j+3];
		A[i][j+6] = A[i+6][j];
		A[i+3][j+6] = A[i+6][j+3];
	}

	
	jacobi(A, 9, eig, H, &nrot);
	//	minim(eig, 9, &ind1);
	ind1 = 0;
	double mn = eig[0];
	for(i=1;i<9;i++)
		if(eig[i]<mn)
		{
			mn = eig[i];
			ind1 = i;
		}

	i=ind1;
	double dU[3][3];
	dU[0][0] = H[0][i];
	dU[0][1] = H[1][i];
	dU[0][2] = H[2][i];
	dU[1][0] = H[3][i];
	dU[1][1] = H[4][i];
	dU[1][2] = H[5][i];
	dU[2][0] = H[6][i];
	dU[2][1] = H[7][i];
	dU[2][2] = H[8][i];

	double dtr = det3x3(dU);
	if(dtr<0)
	{
		dtr = -dtr;
		for(i=0;i<3;i++)for(j=0;j<3;j++)dU[i][j]=-dU[i][j];
	}

	double c = pow(dtr, 1.0/3.0);
	for(i=0;i<3;i++)for(j=0;j<3;j++)U.a[i][j]= dU[i][j] = dU[i][j]/c;

	U = mult_matrix(K1.inverse(),mult_matrix(U, K2));

	double err3 = fabs(U.a[0][2]) + fabs(U.a[1][2]);
	if(err3 > 20)
	{
		err3 = err3;
	}
}


void homographic_warping(unsigned char** src, unsigned char** dst, float** M, int x, int y)
{
	int i, j, i1, j1;
	float x1, y1;

	for(i=0;i<y;i++)
	for(j=0;j<x;j++)
	{
		bilinear_mapping(M, j, i, &x1, &y1);
		i1 = int(y1+0.5f);
		j1 = int(x1+0.5f);
		if(i1>=0 && i1<y && j1>=0 && j1<x)
			dst[i][j] = src[i1][j1];
	}
}

void bilinear_mapping(float** M, int x, int y, float* x1, float* y1)
{
	float X, Y, Z;
	X = M[0][0]*x+M[0][1]*y+M[0][2];
	Y = M[1][0]*x+M[1][1]*y+M[1][2];
	Z = M[2][0]*x+M[2][1]*y+M[2][2];

	*x1 = X/Z;
	*y1 = Y/Z;
}

void bilinear_mapping(float** M, float x, float y, float* x1, float* y1)
{
	float X, Y, Z;
	X = M[0][0]*x+M[0][1]*y+M[0][2];
	Y = M[1][0]*x+M[1][1]*y+M[1][2];
	Z = M[2][0]*x+M[2][1]*y+M[2][2];

	*x1 = X/Z;
	*y1 = Y/Z;
}
	
void bilinear_mapping(double** M, int x, int y, float* x1, float* y1)
{
	float X, Y, Z;
	X = float(M[0][0]*x+M[0][1]*y+M[0][2]);
	Y = float(M[1][0]*x+M[1][1]*y+M[1][2]);
	Z = float(M[2][0]*x+M[2][1]*y+M[2][2]);

	*x1 = X/Z;
	*y1 = Y/Z;
}

void bilinear_mapping(double** M, float x, float y, float* x1, float* y1)
{
	float X, Y, Z;
	X = float(M[0][0]*x+M[0][1]*y+M[0][2]);
	Y = float(M[1][0]*x+M[1][1]*y+M[1][2]);
	Z = float(M[2][0]*x+M[2][1]*y+M[2][2]);

	*x1 = X/Z;
	*y1 = Y/Z;
}


void bilinear_mapping(float M[3][3], int x, int y, float* x1, float* y1)
{
	float X, Y, Z;
	X = M[0][0]*x+M[0][1]*y+M[0][2];
	Y = M[1][0]*x+M[1][1]*y+M[1][2];
	Z = M[2][0]*x+M[2][1]*y+M[2][2];

	*x1 = X/Z;
	*y1 = Y/Z;
}
	
void bilinear_mapping(double M[3][3], int x, int y, float* x1, float* y1)
{
	float X, Y, Z;
	X = float(M[0][0]*x+M[0][1]*y+M[0][2]);
	Y = float(M[1][0]*x+M[1][1]*y+M[1][2]);
	Z = float(M[2][0]*x+M[2][1]*y+M[2][2]);

	*x1 = X/Z;
	*y1 = Y/Z;
}

float homography_error(int matches[][4], float* err, int n, double** U)
{
	int x1, y1, x2, y2, i;
	float xc, yc, error = 0;

	for(i=0;i<n;i++)
	{
	  x1 = matches[i][0];
	  y1 = matches[i][1];
	  x2 = matches[i][2];
	  y2 = matches[i][3];
	  bilinear_mapping(U,x1, y1, &xc, &yc);
	  err[i] = (x2-xc)*(x2-xc) + (y2-yc)*(y2-yc);
	  error+= err[i];
	}
	return error;
}

dMatrix& scaleMatrix(dMatrix& Mc, float s)
{
	Mc.a[0][2] = Mc.a[0][2]*s;
	Mc.a[1][2] = Mc.a[1][2]*s;
	Mc.a[2][0] = Mc.a[2][0]/s;
	Mc.a[2][1] = Mc.a[2][1]/s;
	return Mc;
}

void scaleMatrix(dMatrix& Mc, dMatrix& Mr, float s)
{
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			Mr.a[i][j] = Mc.a[i][j];

	Mr.a[0][2] = Mr.a[0][2]*s;
	Mr.a[1][2] = Mr.a[1][2]*s;
	Mr.a[2][0] = Mr.a[2][0]/s;
	Mr.a[2][1] = Mr.a[2][1]/s;
}


