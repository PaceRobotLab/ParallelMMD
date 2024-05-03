#include "cv.h"

void FindMaxima(unsigned char* p, unsigned char** tmpi, int x, int y, int wx, int wy, int thr)
{
	IplImage *srcImage, *dstImage;
	int xc[50], yc[50], N;
  
	srcImage = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                  
	dstImage = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                 

	srcImage->imageData = (char *)p;
	dstImage->imageData = (char *)tmpi[0];

	iplBlur(srcImage, dstImage, x, y, 0, 0);
	iplThreshold(dstImage, dstImage, thr);
	FindLocalMax(tmpi, xc, yc, &N, x, y, 3);

}

