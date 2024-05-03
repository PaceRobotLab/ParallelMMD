#include "dmatrix.h"


void FindHomography(void* , dMatrix U); // void->CORNER_LIST
void FindHomography(int matches[][4], int n, dMatrix U);
void FindHomography(int matches[][4], int n,dMatrix U, dMatrix K1, dMatrix K2);
void homographic_warping(unsigned char** src, unsigned char** dst, float** M, int x, int y);
void bilinear_mapping(float** M, int x, int y, float* x1, float* y1);
void bilinear_mapping(double** M, int x, int y, float* x1, float* y1);
void bilinear_mapping(float** M, float x, float y, float* x1, float* y1);
void bilinear_mapping(double** M, float x, float y, float* x1, float* y1);
void bilinear_mapping(float M[3][3], int x, int y, float* x1, float* y1);
void bilinear_mapping(double M[3][3], int x, int y, float* x1, float* y1);
float homography_error(int matches[][4], float* err, int n, double** U);
dMatrix& scaleMatrix(dMatrix& Mc, float s);
void scaleMatrix(dMatrix& Mc, dMatrix& Mr, float s);