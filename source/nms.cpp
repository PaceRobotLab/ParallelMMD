#include "nms.h"
#include <memory.h>
#include "cv.h"
//#include "cvaux.h"
#include <math.h>
#define __min(a,b) ((a)<(b))?(a):(b)
#define __max(a,b) ((a)>(b))?(a):(b)

void FindLocalMax(int** s1,unsigned char** s2, int* xc, int* yc, int* N, int x, int y, int win)
{
int i,j, k;
k = 0;
int y1=y-win, x1=x-win;

for(i=win;i<y1;i++)
for(j=win;j<x1;j++)
{
  s2[i][j] = ilocal_max(s1, i, j, win);
  if(s2[i][j]>0)
  {
	  xc[k] = j;
	  yc[k] = i;
	  k++;
  }
}

*N = k;
  
for(i=win;i<y1;i++)
for(j=win;j<x1;j++)
 if(s2[i][j]==1){
  s2[i-1][j-1]=2;
  s2[i-1][j]=3;
  s2[i-1][j+1]=4;

  s2[i][j-1]=5;
  s2[i][j+1]=6;

  s2[i+1][j-1]=7;
  s2[i+1][j]=8;
  s2[i+1][j+1]=9;}
}

void FindLocalMax(unsigned char** s1, int* xc, int* yc, int* val, int* N, int x, int y, int win, int thr)
{
int i,j, k, kt, dx, dy, r = 400;
k = kt = 0;
int y1=y-win, x1=x-win;
int xt[100], yt[100], valt[100], c[100];
memset((void*) c, 1, 100*sizeof(int));

for(i=win;i<y1;i++)
for(j=win;j<x1;j++)
{
  if(s1[i][j] > thr && ulocal_max(s1, i, j, win))
  {
	  xt[kt] = j;
	  yt[kt] = i;
	  valt[kt]= s1[i][j];
	  kt++;
  }
}

for(i=0;i<kt;i++)
if(c[i])
	for(j=i+1; j<kt; j++)
	if(c[j])
	{
		dx = xt[i] - xt[j];
		dy = yt[i] - yt[j];
		if( dx*dx + dy*dy < r )
		{
			c[j] = 0;
			xt[i] = (valt[i]*xt[i] + valt[j]*xt[j])/(valt[i]+valt[j]);
			yt[i] = (valt[i]*yt[i] + valt[j]*yt[j])/(valt[i]+valt[j]);
			valt[i] = __max(valt[i], valt[j]);
		}
	}

for(i=0;i<kt;i++)
if(c[i])
{
	xc[k] = xt[i];
	yc[k] = yt[i];
	val[k]= valt[i];
	k++;
}

*N = k;
}

void FindLocalMax(unsigned char** s1, int* xc, int* yc, int* val, int* N, int x, int y, CRectangle* Rect, int win, int thr)
{
int i,j, k, kt, dx, dy, r = 400;
k = kt = 0;
int x1 = Rect->x_min + win, y1 = Rect->y_min + win, x2 = Rect->x_max - win, y2 = Rect->y_max - win;
int xt[100], yt[100], valt[100], c[100];
memset((void*) c, 1, 100*sizeof(int));

for(i=y1; i<y2; i++)
for(j=x1 ;j<x2; j++)
{
  if(s1[i][j] > thr && ulocal_max(s1, i, j, win))
  {
	  xt[kt] = j;
	  yt[kt] = i;
	  valt[kt]= s1[i][j];
	  kt++;
  }
}

for(i=0;i<kt;i++)
if(c[i])
	for(j=i+1; j<kt; j++)
	if(c[j])
	{
		dx = xt[i] - xt[j];
		dy = yt[i] - yt[j];
		if( dx*dx + dy*dy < r )
		{
			c[j] = 0;
			xt[i] = (valt[i]*xt[i] + valt[j]*xt[j])/(valt[i]+valt[j]);
			yt[i] = (valt[i]*yt[i] + valt[j]*yt[j])/(valt[i]+valt[j]);
		}
	}

for(i=0;i<kt;i++)
if(c[i])
{
	xc[k] = xt[i];
	yc[k] = yt[i];
	k++;
}

*N = k;
}

void FindLocalMax(unsigned char** s1, unsigned char** mask, int* xc, int* yc, int* val, int* N, int x, int y, int win, int thr)
{
int i,j, k, kt, dx, dy, r = 400;
k = kt = 0;
int y1=y-win, x1=x-win;
int xt[1000], yt[1000], valt[1000], c[1000];
memset((void*) c, 1, 100*sizeof(int));

for(i=win;i<y1;i++)
for(j=win;j<x1;j++)
{
  if(mask[i][j])
  if(ulocal_max(s1, mask, x, i, j, win) )
  {
	  xt[kt] = j;
	  yt[kt] = i;
	  valt[kt]= s1[i][j];
	  kt++;
  }
}

for(i=0;i<kt;i++)
if(c[i])
	for(j=i+1; j<kt; j++)
	if(c[j])
	{
		dx = xt[i] - xt[j];
		dy = yt[i] - yt[j];
		if( dx*dx + dy*dy < r )
		{
			c[j] = 0;
			xt[i] = (valt[i]*xt[i] + valt[j]*xt[j])/(valt[i]+valt[j]);
			yt[i] = (valt[i]*yt[i] + valt[j]*yt[j])/(valt[i]+valt[j]);
		}
	}

for(i=0;i<kt;i++)
if(c[i])
{
	xc[k] = xt[i];
	yc[k] = yt[i];
	k++;
}

*N = k;
}


unsigned char ilocal_max(int **s1, int i, int j, int win)
{                              //s1 - response matrix;
int s0;
unsigned char inp=0;

if(s1[i][j]>0){
	s0 = s1[i][j];
	if(win==2){
		if(
			(s0>s1[i-1][j+2]) &&
			(s0>s1[i][j+1]) &&
			(s0>s1[i][j+2]) &&
			(s0>s1[i+1][j-1]) &&
			(s0>s1[i+1][j]) &&
			(s0>s1[i+1][j+1]) &&
			(s0>s1[i+1][j+2]) &&
			(s0>s1[i+2][j-2]) &&
			(s0>s1[i+2][j-1]) &&
			(s0>s1[i+2][j]) &&
			(s0>s1[i+2][j+1]) &&
			(s0>s1[i+2][j+2]) &&
			(s0>=s1[i-2][j-2]) &&
			(s0>=s1[i-2][j-1]) &&
			(s0>=s1[i-2][j]) &&
			(s0>=s1[i-2][j+1]) &&
			(s0>=s1[i-2][j+2]) &&
			(s0>=s1[i-1][j-2]) &&
			(s0>=s1[i-1][j-1]) &&
			(s0>=s1[i-1][j]) &&
			(s0>=s1[i-1][j+1]) &&
			(s0>=s1[i][j-2]) &&
			(s0>=s1[i][j-1]) &&
			(s0>=s1[i+1][j-2]) )inp = 1;
		}
	else if(win==3){
		if(
			(s0>s1[i-1][j+3]) &&
			(s0>s1[i-1][j+2]) &&
			(s0>s1[i][j+1]) &&
			(s0>s1[i][j+2]) &&
			(s0>s1[i][j+3]) &&
			(s0>s1[i+1][j-1]) &&
			(s0>s1[i+1][j]))
		if(
			(s0>s1[i+1][j+1]) &&
			(s0>s1[i+1][j+2]) &&
			(s0>s1[i+1][j+3]) &&
			(s0>s1[i+2][j-2]) &&
			(s0>s1[i+2][j-1]) &&
			(s0>s1[i+2][j]) )
		if(
			(s0>s1[i+2][j+1]) &&
			(s0>s1[i+2][j+2]) &&
			(s0>s1[i+3][j-1]) &&
			(s0>s1[i+3][j])&&
			(s0>s1[i+3][j+1])){
		if((s0>=s1[i-3][j-1]) &&
			(s0>=s1[i-3][j]) &&
			(s0>=s1[i-3][j+1]) &&
			(s0>=s1[i-2][j-2]) &&
			(s0>=s1[i-2][j-1]) &&
			(s0>=s1[i-2][j]) &&
			(s0>=s1[i-2][j+1]) &&
			(s0>=s1[i-2][j+2]) &&
			(s0>=s1[i-1][j-3]) &&
			(s0>=s1[i-1][j-2]) &&
			(s0>=s1[i-1][j-1]) &&
			(s0>=s1[i-1][j]) &&
			(s0>=s1[i-1][j+1]) &&
			(s0>=s1[i][j-3]) &&
			(s0>=s1[i][j-2]) &&
			(s0>=s1[i][j-1]) &&
			(s0>=s1[i+1][j-3]) &&
			(s0>=s1[i+1][j-2]) )inp = 1;}
		}
	}
return inp;
}

unsigned char ulocal_max(unsigned char** s1, int i, int j, int win)
{                              //s1 - response matrix;
int s0;
unsigned char inp=0;

if(s1[i][j]>0){
	s0 = s1[i][j];
	if(win==2){
		if(
			(s0>s1[i-1][j+2]) &&
			(s0>s1[i][j+1]) &&
			(s0>s1[i][j+2]) &&
			(s0>s1[i+1][j-1]) &&
			(s0>s1[i+1][j]) &&
			(s0>s1[i+1][j+1]) &&
			(s0>s1[i+1][j+2]) &&
			(s0>s1[i+2][j-2]) &&
			(s0>s1[i+2][j-1]) &&
			(s0>s1[i+2][j]) &&
			(s0>s1[i+2][j+1]) &&
			(s0>s1[i+2][j+2]) &&
			(s0>=s1[i-2][j-2]) &&
			(s0>=s1[i-2][j-1]) &&
			(s0>=s1[i-2][j]) &&
			(s0>=s1[i-2][j+1]) &&
			(s0>=s1[i-2][j+2]) &&
			(s0>=s1[i-1][j-2]) &&
			(s0>=s1[i-1][j-1]) &&
			(s0>=s1[i-1][j]) &&
			(s0>=s1[i-1][j+1]) &&
			(s0>=s1[i][j-2]) &&
			(s0>=s1[i][j-1]) &&
			(s0>=s1[i+1][j-2]) )inp = 1;
		}
	else if(win==3){
		if(
			(s0>s1[i-1][j+3]) &&
			(s0>s1[i-1][j+2]) &&
			(s0>s1[i][j+1]) &&
			(s0>s1[i][j+2]) &&
			(s0>s1[i][j+3]) &&
			(s0>s1[i+1][j-1]) &&
			(s0>s1[i+1][j]))
		if(
			(s0>s1[i+1][j+1]) &&
			(s0>s1[i+1][j+2]) &&
			(s0>s1[i+1][j+3]) &&
			(s0>s1[i+2][j-2]) &&
			(s0>s1[i+2][j-1]) &&
			(s0>s1[i+2][j]) )
		if(
			(s0>s1[i+2][j+1]) &&
			(s0>s1[i+2][j+2]) &&
			(s0>s1[i+3][j-1]) &&
			(s0>s1[i+3][j])&&
			(s0>s1[i+3][j+1])){
		if((s0>=s1[i-3][j-1]) &&
			(s0>=s1[i-3][j]) &&
			(s0>=s1[i-3][j+1]) &&
			(s0>=s1[i-2][j-2]) &&
			(s0>=s1[i-2][j-1]) &&
			(s0>=s1[i-2][j]) &&
			(s0>=s1[i-2][j+1]) &&
			(s0>=s1[i-2][j+2]) &&
			(s0>=s1[i-1][j-3]) &&
			(s0>=s1[i-1][j-2]) &&
			(s0>=s1[i-1][j-1]) &&
			(s0>=s1[i-1][j]) &&
			(s0>=s1[i-1][j+1]) &&
			(s0>=s1[i][j-3]) &&
			(s0>=s1[i][j-2]) &&
			(s0>=s1[i][j-1]) &&
			(s0>=s1[i+1][j-3]) &&
			(s0>=s1[i+1][j-2]) )inp = 1;}
		}
	}
return inp;
}

void ffind_local_max(float** s1,unsigned char** s2, int x, int y, int win)
{
int i,j;
int y1=y-win, x1=x-win;

for(i=win;i<y1;i++)
for(j=win;j<x1;j++)
  s2[i][j] = flocal_max(s1, i, j, win);

}

unsigned char flocal_max(float **s1, int i, int j, int win)
{                              //s1 - response matrix;
int k,l,i2,j2;
float s0;
unsigned char inp=0;
if(s1[i][j]>0)
 {
 inp=1;
 i2=i+win;
 j2=j+win;
 s0=s1[i][j];
 for(k = i-win;k<=i2; k++)
 for(l = j-win;l<=j2; l++)
  if((k!=i)||(l!=j))
	{
	if(s0<s1[k][l])
	 {inp=0;
	 goto Konec;}
	}
  Konec:;
 }
return inp;
}




//	Clustering

void FindMaxima(unsigned char* p, unsigned char** tmpi, int x, int y, int wx, int wy, int thr, int* xc, int* yc, int* val, int* N)
{
	IplImage *srcImage, *dstImage;

	srcImage=cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
  
//	srcImage = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	dstImage = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                 

	srcImage->imageData = (char *)p;
	dstImage->imageData = (char *)tmpi[0];

    cvSmooth(srcImage, dstImage,CV_BLUR,wx,wy,0,0);
    cvSmooth(dstImage, dstImage,CV_BLUR,wx,wy,0,0);
//	iplBlur(srcImage, dstImage, wx, wy, 0, 0);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);
//	iplThreshold(dstImage, dstImage, thr);

	FindLocalMax(tmpi, xc, yc, val, N, x, y, 3, thr);

}

void sFindMaxima(short* p, unsigned short scale, unsigned char** tmpi, int x, int y, int wx, int wy, int thr, int* xc, int* yc, int* val, int* N)
{
	IplImage *srcImage, *dstImage;
  
	srcImage = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_16S,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_16S, "GRAY",                      
//        "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	dstImage = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                 

	srcImage->imageData = (char*)p;
	dstImage->imageData = (char*)tmpi[0];
	for(int i = 0; i<x*y; i++)
		tmpi[0][i] = (unsigned char)(p[i]/scale);

	cvSmooth(dstImage, dstImage,CV_BLUR,wx, wy, wx/2, wy/2);
	cvSmooth(dstImage, dstImage,CV_BLUR,wx, wy, wx/2, wy/2);

//	iplBlur(dstImage, dstImage, wx, wy, wx/2, wy/2);
//	iplBlur(dstImage, dstImage, wx, wy, wx/2, wy/2);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);
//	iplThreshold(dstImage, dstImage, thr);

	FindLocalMax(tmpi, xc, yc, val, N, x, y, 3, thr);

}

void FindMaxima(unsigned char* p, unsigned char** mask, unsigned char** tmpi, int x, int y, int wx, int wy, int thr, int* xc, int* yc, int* val, int* N)
{
	IplImage *srcImage, *dstImage, *maskImage;
  
	srcImage = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	dstImage = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                 
	maskImage = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                 

	srcImage->imageData = (char *)p;
	dstImage->imageData = (char *)tmpi[0];
	maskImage->imageData = (char *)mask[0];

    cvSmooth(srcImage, dstImage,CV_BLUR,wx,wy,0,0);
    cvSmooth(dstImage, dstImage,CV_BLUR,wx,wy,0,0);
//	iplBlur(srcImage, dstImage, wx, wy, 0, 0);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);
//	iplBlur(dstImage, dstImage, wx, wy, 0, 0);

    cvThreshold(dstImage, maskImage, thr,255,CV_THRESH_BINARY);

//	iplThreshold(dstImage, maskImage, thr);

	FindLocalMax(tmpi, mask, xc, yc, val, N, x, y, 3, thr);

}


bool ulocal_max1(unsigned char** func, unsigned char** mask, int x, int i, int j)
{
	if(mask[i][j] == 0)
		return false;

	int win = 1;
	unsigned char* p0 = func[i]+j;
	unsigned char* p1 = func[i-win]+j-win;
	unsigned char* p2 = mask[i-win]+j-win;
	unsigned char dif;
	int offset = x - 2*win;

	// potentially use short tmp = c0 - *p1, you can use mmx here ?!

//	First raw;
	dif = *p0 - *p1;

	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}

	dif = *p0 - *p1;

	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}

	dif = *p0 - *p1;

	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}
//	End first raw

	p1+=offset;
	p2+=offset;

//	Second raw
	dif = *p0 - *p1;
	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}

	p2++; p1++;

	dif = *p0 - *p1;
	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}
//	End second raw

	p1+=offset;
	p2+=offset;

//	Third raw;
	dif = *p0 - *p1;
	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}

	dif = *p0 - *p1;
	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}

	dif = *p0 - *p1;
	if(dif > 0)
	{
		*p2++ = 0;
		p1++;
	}
	else if(dif == 0)
	{
		*p2++ = 1;
		p1++;
	}
	else
	{
		mask[i][j] = 0;
		return false;
	}
//	End third raw
	
return true;
}


bool ulocal_max(unsigned char** func, unsigned char** mask, int x, int ic, int jc, int win)
{
	if(mask[ic][jc] == 0)
		return false;

	int i, j, dw = 2*win+1;
	unsigned char* p0 = func[ic]+jc;
	unsigned char* p1 = func[ic-win]+jc-win;
	unsigned char* p2 = mask[ic-win]+jc-win;
	unsigned char dif;
	int offset = x - 2*win;

	for(i=0; i<win; i++)
	{
		for(j=0; j < dw; j++)
		{
			dif = *p0 - *p1;
			if(dif > 0)
			{
				*p2++ = 0;
				p1++;
			}
			else if(dif == 0)
			{
				*p2++ = 1;
				p1++;
			}
			else
			{
				mask[ic][jc] = 0;
				return false;
			}
		}
		p1+=offset;
		p2+=offset;
	}

	for(j=0; j < win; j++)
	{
		dif = *p0 - *p1;
		if(dif > 0)
		{
			*p2++ = 0;
			p1++;
		}
		else if(dif == 0)
		{
			*p2++ = 1;
			p1++;
		}
		else
		{
			mask[ic][jc] = 0;
			return false;
		}
	}
	p1++;
	p2++;
	for(j=0; j < win; j++)
	{
		dif = *p0 - *p1;
		if(dif > 0)
		{
			*p2++ = 0;
			p1++;
		}
		else if(dif == 0)
		{
			*p2++ = 1;
			p1++;
		}
		else
		{
			mask[ic][jc] = 0;
			return false;
		}
	}
	p1+=offset;
	p2+=offset;

	for(i=0; i<win; i++)
	{
		for(j=0; j < dw; j++)
		{
			dif = *p0 - *p1;
			if(dif > 0)
			{
				*p2++ = 0;
				p1++;
			}
			else if(dif == 0)
			{
				*p2++ = 1;
				p1++;
			}
			else
			{
				mask[ic][jc] = 0;
				return false;
			}
		}
		p1+=offset;
		p2+=offset;
	}
	return true;
}
