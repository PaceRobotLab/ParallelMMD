#include "cmotion_detection.h"
#include "light_scaling.h"
#include "brMemalloc.h"
#include "median.h"
#include "homography.h"
#include <math.h>
#include <stdio.h>

CMotionDetection::CMotionDetection(int x, int y)
{
	width = x;
	height = y;
	scale = 0.0f;
	g1 = new unsigned char [x*y];
	g2 = new unsigned char [x*y];
	gb = new unsigned char [x*y];
	gt = new unsigned char [x*y];

	gx = new short [x*y];
	gy = new short [x*y];
	gs = new short [x*y];
	memset((void*)gx, x*y, sizeof(short));
	memset((void*)gy, x*y, sizeof(short));

	motion_mask = alloc2Duchar(x,y);
	median_motion_mask = alloc2Duchar(x,y);

	Im1 = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//        "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//        x, y, NULL, NULL, NULL, NULL);                  
	Im2 = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  

//	
	G1 = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	G1->imageData = (char *)g1;

	G2 = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	G2->imageData = (char *)g2;
	Gb = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	Gb->imageData = (char *)gb;

	Gs = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_16S,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_16S, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	Gs->imageData = (char *)gs;

//
	Gx = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_16S,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_16S, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	Gx->imageData = (char*)gx;

	Gy = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_16S,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_16S, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	Gy->imageData = (char*)gy;

	Gt = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	Gt->imageData = (char*)gt;

	MM = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	MM->imageData = (char*)motion_mask[0];

	MedMM = cvCreateImageHeader(cvSize(x,y),IPL_DEPTH_8U,1);
//	iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
//         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
//         x, y, NULL, NULL, NULL, NULL);                  
	MedMM->imageData = (char*)median_motion_mask[0];
	center.x = width/2;
	center.y = height/2;
}


int CMotionDetection::compute_motion_mask(unsigned char* im1, unsigned char* im2, 
										  dMatrix M, int threshold, float radius)
{
/*	FILE* pFile1 = fopen(".\\g1.raw", "wb");
	FILE* pFile2 = fopen(".\\g2.raw", "wb");
	FILE* pFileb = fopen(".\\gb.raw", "wb");
	FILE* pFilet = fopen(".\\gt.raw", "wb");
	FILE* pFilem1= fopen(".\\gm1.raw", "wb");
	FILE* pFilem2= fopen(".\\gm2.raw", "wb");
	FILE* pFileE= fopen(".\\edgels.txt", "wt");
*/
	int i = 0, n = width*height, grad_tot = 0;//, x, y;
	unsigned char* mm = motion_mask[0];//, *tmp;
	float   grad_thr = 175.0f;
/*	if(scale == 0)
	{
		this->im1 = im1;
		Im1->imageData = (char*)im1;
		iplFixedFilter(Im1, G1, IPL_GAUSSIAN_5x5);
	}
	else
	{
		tmp = g1;
		g1 = g2;
		g2 = tmp;
		G1->imageData = (char*)g1;
		G2->imageData = (char*)g2;
	//	memcpy(g1, g2, n);
	}
*/

	this->im1 = im1;
	Im1->imageData = (char*)im1;
	//iplFixedFilter(Im1, G1, IPL_GAUSSIAN_5x5);
	cvSmooth(Im1,G1,CV_GAUSSIAN,5,5);
	Im2->imageData = (char*)im2;
	//iplFixedFilter(Im2, G2, IPL_GAUSSIAN_5x5);
	cvSmooth(Im2,G2,CV_GAUSSIAN,5,5);

//	cvAbsDiff(Im1, Im2, Gt);

	scale = 1.0f;//find_scale(G1, G2, Gb, M);		//equalize images with regards to lighting
	scaled_difference(G1, G2, Gb, Gt, M, scale);
/*	fwrite((void*)g1, sizeof(unsigned char), width*height, pFile1);
	fwrite((void*)g2, sizeof(unsigned char), width*height, pFile2);
	fwrite((void*)gb, sizeof(unsigned char), width*height, pFileb);
	fwrite((void*)gt, sizeof(unsigned char), width*height, pFilet);
	fclose(pFile1);
	fclose(pFile2);
	fclose(pFileb);
	fclose(pFilet);
*/
	int N = 0, thr = n/10;
	cvSobel(Gb, Gx, 1, 0, 3);
	cvSobel(Gb, Gy, 0, 1, 3);

	int j, sum = 0, hp[320],vp[240];

	memset((void*)hp,0,320*sizeof(int));// size built in
	memset((void*)vp,0,240*sizeof(int));

    cvThreshold(Gt, MM, threshold,255,CV_THRESH_BINARY);
//	iplThreshold(Gt, MM, threshold);
//	fwrite((void*)mm, sizeof(unsigned char), width*height, pFilem1);

	for(i=0;i<n;i++)
	if(mm[i])
	{
		grad_tot = int(cvSqrt(gx[i]*gx[i] + gy[i]*gy[i])/8);
		if(gt[i] - radius*grad_tot < threshold)
			mm[i] = 0;
		else
			mm[i] = mm[i] + 1 - 1;
	}

//	iplMedianFilter(MM, MedMM, 3, 3, 1, 1);
//	iplOpen(MM, MM, 1);
	float cx, cy;
	int roi[4];
	find_min_ROI(M, width, height, roi);

	for(i=roi[1]+10;i<roi[3]-10;i++)
	for(j=roi[0]+10;j<roi[2]-10;j++)
	if(motion_mask[i][j])
	{
		sum++;
		vp[i]++;
		hp[j]++;
	}


	if(sum>20)
	{
		center.x = GetMedianFromCountingSort(sum/2, hp);
		center.y = GetMedianFromCountingSort(sum/2, vp);
	}
	else
	{
		bilinear_mapping(M.a, center.x, center.y, &cx, &cy);
		center.x = int(cx+.5);
		center.y = int(cy+.5);
//		this->im1 = im1;
//		Im1->imageData = (char*)im1;
//		iplFixedFilter(Im1, G2, IPL_GAUSSIAN_5x5);
	}

	cvCircle( MM, cvPoint(cvRound(center.x),cvRound(center.y)), 5, CV_RGB(128,128,128), 1);

//	fwrite((void*)mm, sizeof(unsigned char), width*height, pFilem2);
//	fclose(pFilem1);
//	fclose(pFilem2);
	return sum;

}


int CMotionDetection::CompareHomographies(unsigned char* im1, unsigned char* im2, dMatrix M0, dMatrix M1, int threshold, float radius)
{
	int i = 0, n = width*height, grad_tot = 0;//, x, y;
	unsigned char* mm = motion_mask[0];//, *tmp;
	float grad_thr = 175.0f;
	int N0, N1;

/*	if(scale == 0)
	{
		this->im1 = im1;
		Im1->imageData = (char*)im1;
		iplFixedFilter(Im1, G1, IPL_GAUSSIAN_5x5);
		scale = 1.0f;
	}
	else
	{
		tmp = g1;
		g1 = g2;
		g2 = tmp;
		G1->imageData = (char*)g1;
		G2->imageData = (char*)g2;
	}
*/
	this->im1 = im1;
	Im1->imageData = (char*)im1;
//	iplFixedFilter(Im1, G1, IPL_GAUSSIAN_5x5);
	cvSmooth(Im1, G1, CV_GAUSSIAN,5,5);

	Im2->imageData = (char*)im2;
//	iplFixedFilter(Im2, G2, IPL_GAUSSIAN_5x5);
	cvSmooth(Im1, G1, CV_GAUSSIAN,5,5);

	N0 = N1 = 0;
	int j, hp[160],vp[120];
	memset((void*)hp,0,160*sizeof(int));
	memset((void*)vp,0,120*sizeof(int));

	scaled_difference(G1, G2, Gb, Gt, M0, scale);
//	iplThreshold(Gt, MM, threshold);
	cvThreshold(Gt, MM, threshold,255,CV_THRESH_BINARY);

	for(i=5;i<height-5;i++)
	for(j=5;j<width-5;j++)
	if(motion_mask[i][j])
	{
		N0++;
		vp[i]++;
		hp[j]++;
	}

	scaled_difference(G1, G2, Gb, Gt, M1, scale);
//	iplThreshold(Gt, MM, threshold);
		cvThreshold(Gt, MM, threshold,255,CV_THRESH_BINARY);

	for(i=5;i<height-5;i++)
	for(j=5;j<width-5;j++)
	if(motion_mask[i][j])
	{
		N1++;
		vp[i]++;
		hp[j]++;
	}

//	It assumes that M0 is homography matrix obtained from the camera
//	and M1 is the homography matrix obtained from the point matches
//	and accepts M1 only it is better than M0

	if(N1 < 0.95*N0 - 5)
		return 1;
	else
		return 0;
}


CMotionDetection::~CMotionDetection()
{
delete g1;
delete g2;
delete gb;
delete gt;
delete gx;
delete gy;
delete gs;
free2D(motion_mask);
free2D(median_motion_mask);
}
