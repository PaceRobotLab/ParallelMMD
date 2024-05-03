//	Class Candidate.h

#ifndef CANDIDATE_H
	#define CANDIDATE_H

#include "dmatrix.h"

class Candidate
{
public:
	int		current_frame;
	int		N;				// Number of frames used for filtering
	int		x[10], y[10];
	float	T[2][10];		// Prediction Matrix
	float	a[2], b[2];
	int		x_predicted, y_predicted;

	Candidate(int N);
	~Candidate();
	void Update(int x, int y, dMatrix M);
	void PredictNewPosition();
	void UpdateNewPosition(dMatrix M);
	double PredictionError(int x1, int y1);
	int FindBestPrediction(int* xc, int* yc, int n);

};

#endif