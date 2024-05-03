#include "Candidate.h"
#include "homography.h"
#include "stdio.h"
#include <math.h>


Candidate::Candidate(int n): N(n)
{
	int k = (N-1)/2;
	float c1, c2;
	c1 = 1/float(N);
	c2 = 3/float(k*(k+1)*(2*k+1));
	FILE* Fp = fopen("e:\\users\\mdt\\pred.txt", "wt");

	for(int i = 0; i<N; i++)
	{
		T[0][i] = c1;
		T[1][i] = (i-k)*c2;
		fprintf(Fp, "%f ", T[1][i]);
	}

	fprintf(Fp, "%f \n", T[0][0]);
	fclose(Fp);

	x_predicted = y_predicted = 0;
	current_frame = 0;
}

void Candidate::Update(int xi, int yi, dMatrix M)
{
	// Updates the current frame, the position vectors, and the velocity parameters
	int i,j;
	float x1 = 0, y1 = 0;
	FILE* Fp;
	static fg = 1;
	double error;
	Fp = fopen("e:\\users\\mdt\\pred.txt", "at");


	if(current_frame < N)
	{
		for(i=0;i<current_frame;i++)
		{
			bilinear_mapping(M.a, x[i], y[i], &x1, &y1);
			x[i] = int(x1+.5f);
			y[i] = int(y1+.5f);
		}
		x[current_frame] = xi;
		y[current_frame] = yi;

	}
	else
	{
		error = PredictionError(xi, yi);
		for(i=0;i<N;i++)
		{
			bilinear_mapping(M.a, x[i], y[i], &x1, &y1);
			x[i] = int(x1+.5f);
			y[i] = int(y1+.5f);
		}

		i = current_frame%N;
		x[i] = xi;
		y[i] = yi;

		a[0] = a[1] = b[0] = b[1] = 0.0f;
		for(i=0; i<N;i++)
		{
			j = (i+1+current_frame)%N;
			a[0]+= T[0][i]*x[j];
			a[1]+= T[1][i]*x[j];
			b[0]+= T[0][i]*y[j];
			b[1]+= T[1][i]*y[j];
			fprintf(Fp, "%4d %4d   ", x[j], y[j]);
		}
		bilinear_mapping(M.a, x_predicted, y_predicted, &x1, &y1);
		fprintf(Fp, "%6.2f ", error);
		PredictNewPosition();
		fprintf(Fp, "%4d %4d \n", x_predicted, y_predicted);
	}
	fclose(Fp);
	current_frame++;

}

void Candidate::PredictNewPosition()
{
	int k = (N-1)/2;

	x_predicted = int(a[0] + a[1]*(k+1) +.5f);
	y_predicted = int(b[0] + b[1]*(k+1) + .5f);
}

void Candidate::UpdateNewPosition(dMatrix M)
{
	// Updates the current frame, the position vectors, and the velocity parameters
	float* x1, *y1;
	bilinear_mapping(M.a, x_predicted, y_predicted, x1, y1);
	x_predicted = int(*x1+.5f);
	y_predicted = int(*y1+.5f);
}

double Candidate::PredictionError(int x1, int y1)
{
//	It is assumed that the true position has to lie somewhere between the current position
//	x[4], y[4] and predticted position x_predicted, y_predicted.
//	So the error is determined as a minimal distance between the point (x1,y1) and the line
//	determined by (x[4], y[4]) and (x_predicted, y_predicted).

	double t, error;
//	First, compute t

	double d[2], dp[2], da[2], aa, ap;

	dp[0] = double(x_predicted - x[4]);
	dp[1] = double(y_predicted - y[4]);
	da[0] = double(a[1]);
	da[1] = double(a[2]);

	aa = da[0]*da[0] + da[1]*da[1];
	ap = da[0]*dp[0] + da[1]*dp[1];

	t = ap/aa;
	if(t<0)
		t=0.0;
	else if(t>1)
		t = 1.0;

	d[0] = dp[0] - t*da[0];
	d[1] = dp[1] - t*da[1];

	error = sqrt(d[0]*d[0] + d[1]*d[1]);
	return error;

}

int Candidate::FindBestPrediction(int* xc, int* yc, int n)
{
	double pe, error = 100000.0;
	int i, ind;
	
	for(i=0; i<n; i++)
	{
		pe = PredictionError(xc[i], yc[i]);
		if( error > pe )
		{
			error = pe;
			ind = i;
		}
	}

	return ind;
}
			


Candidate::~Candidate()
{
}