#include "dmatrix.h"

void FindAffineMatrix(int matches[][4], int n, dMatrix U);
void FindAffineMatrix(int matches[][4], int n, dMatrix U, int* w);
void FindAffineMatrix(int matches[][4], int n, dMatrix U, dMatrix K);
float FindAffineMatrix3(int matches[][4], int n, dMatrix U, float* error);

void affine_error(int matches[][4], float* err, int n, double** U, float thr = 0, int* w = 0, int* sum_w = 0);