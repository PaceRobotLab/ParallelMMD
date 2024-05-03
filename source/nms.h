#include "cv.h"
#include "CornerClass.h"

void FindLocalMax(int** s1,unsigned char** s2, int* xc, int* yc, int* N, int x, int y, int win = 3);
void FindLocalMax(unsigned char** s1, int* xc, int* yc, int* val, int* N, int x, int y, int win = 3, int thr = 0);
void FindLocalMax(unsigned char** s1, unsigned char** mask, int* xc, int* yc, int* val, int* N, int x, int y, int win, int thr);
void FindMaxima(unsigned char* p, unsigned char** tmpi, int x, int y, int wx, int wy, int thr, int* xc, int* yc, int* val, int* N);
void FindMaxima(unsigned char* p, unsigned char** mask, unsigned char** tmpi, int x, int y, int wx, int wy, int thr, int* xc, int* yc, int* val, int* N);
void sFindMaxima(short* p, unsigned short scale, unsigned char** tmpi, int x, int y, int wx, int wy, int thr, int* xc, int* yc, int* val, int* N);
unsigned char ilocal_max(int**, int, int, int);
unsigned char ulocal_max(unsigned char**, int, int, int);
void ffind_local_max(float**, unsigned char**, int, int, int window =3);
unsigned char flocal_max(float**, int, int, int);
bool ulocal_max1(unsigned char** func, unsigned char** mask, int x, int i, int j);
bool ulocal_max(unsigned char** func, unsigned char** mask, int x, int ic, int jc, int win);
void FindLocalMax(unsigned char** s1, int* xc, int* yc, int* val, int* N, int x, int y, CRectangle* Rect, int win, int thr);
