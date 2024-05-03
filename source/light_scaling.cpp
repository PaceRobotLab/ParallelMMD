#include "light_scaling.h"
#include "homography.h"
#define __min(a,b) ((a)<(b))?(a):(b)
#define __max(a,b) ((a)>(b))?(a):(b)

void find_min_ROI(dMatrix M, int width, int height, int* roi)
{
	float x1,x2,x3,x4,y1,y2,y3,y4;
	bilinear_mapping(M.a, 0, 0, &x1, &y1);
	bilinear_mapping(M.a, width, 0, &x2, &y2);
	bilinear_mapping(M.a, 0, height, &x3, &y3);
	bilinear_mapping(M.a, width, height, &x4, &y4);

	roi[0] = __max(0, int(__max(x1,x3)));
	roi[1] = __max(0, int(__max(y1,y2)));
	roi[2] = __min(width, int(__min(x2,x4)));
	roi[3] = __min(height, int(__min(y3,y4)));
}

float find_scale(IplImage* IplFirst, IplImage* IplSecond, IplImage* IplBGD, dMatrix M)
{
	int i,j;
	//double coeffs[3][3];
	CvMat *cvM = cvCreateMat(3,3,CV_32FC1); // 32 bit floats

	for(i=0;i<3;i++)for(j=0;j<3;j++) cvSetReal2D(cvM,i,j,M.a[i][j]);
		//coeffs[i][j] = M.a[i][j];
   

	cvWarpPerspective(IplFirst, IplBGD, cvM,  CV_INTER_NN, cvRealScalar(0));
	int roi[4],s1,s2,x;
	char *p1, *p2,*im2,*bgd;
	im2 = IplSecond->imageData;
	bgd = IplBGD->imageData;
	x = IplBGD->width;
	s1 = s2 = 0;
	find_min_ROI(M, IplBGD->width, IplBGD->height, roi);
	for(i=roi[1]; i<roi[3]; i++)
	{
		p1 = bgd+i*x+roi[0];
		p2 = im2+i*x+roi[0];
		for(j=roi[0]; j<roi[2]; j++)
		{
			s1+= *p1++;
			s2+= *p2++;
		}
	}
	
	return	s1/float(s2);
}

void scaled_difference(IplImage* IplFirst, IplImage* IplSecond, IplImage* IplBGD, 
					   IplImage* IplResult, dMatrix M, float s)
{
	int i, j, n, roi[4],w,h;
	w = IplFirst->width;
	h = IplFirst->height;
	n = w*h;
	unsigned char *im2, *bgd, *res;
	im2 = (unsigned char*)IplSecond->imageData;
	res = (unsigned char*)IplResult->imageData;
	bgd = (unsigned char*)IplBGD->imageData;

	//double coeffs[3][3];
	//for(i=0;i<3;i++)for(j=0;j<3;j++)coeffs[i][j] = M.a[i][j];
	CvMat *cvM = cvCreateMat(3,3,CV_64FC1); // 32 bit floats

	for(i=0;i<3;i++)for(j=0;j<3;j++) cvSetReal2D(cvM,i,j,M.a[i][j]);

	find_min_ROI(M, IplBGD->width, IplBGD->height, roi);

	//iplMultiplySScale(IplSecond, IplResult, 255);
	cvConvertScale(IplSecond, IplResult, 255, 0);
//	memcpy(im2, res, n);
	memcpy( bgd, res, n ); 

	cvWarpPerspective(IplFirst, IplBGD, cvM, CV_INTER_LINEAR, cvRealScalar(0));
	cvAbsDiff(IplBGD, IplResult, IplResult);
	// dml cvAbsDiff(IplBGD, IplSecond, IplResult);

/*	for(i=0;i<=roi[1]+10;i++)
	{
		p1 = res + i*w;
		for(j=0;j<w;j++)
			*p1++ = 0;
	}

	for(i=roi[3]-10;i<h;i++)
	{
		p1 = res + i*w;
		for(j=0;j<w;j++)
			*p1++ = 0;
	}

	for(i=roi[1]+1; i<roi[3]-1; i++)
	{
		p1 = res + i*w;
		for(j=0;j<=roi[0]+10;j++)
			*p1++ = 0;

		p1 = res + i*w + roi[2];
		for(j=roi[2]-10; j<w; j++)
			*p1++ = 0;
	}
*/
}
