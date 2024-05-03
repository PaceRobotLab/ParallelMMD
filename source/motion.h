#ifndef MOTION_H
#define MOTION_H

#include "defs.h"
#include "dmatrix.h"

void iplBackgroundUpdate(unsigned char* bgd, unsigned char* im1, unsigned char* im2, dMatrix H, int x, int y);
void brSubtract(Image<unsigned char>* pIm1, Image<unsigned char>* pIm2, Image<unsigned char>* pImdif);
void brImageDifference(Image<unsigned char>* pIm1, Image<unsigned char>* pIm2, Image<unsigned char>* pImdif, int threshold);
void brBlur(Image<unsigned char>* pIm1, Image<unsigned char>* pIm2, tRectangle* pRect, int threshold, int dx = 0, int dy = 0);
void FindGlobalBox(Image<unsigned char>* pImage, tRectangle* pBox);


#endif