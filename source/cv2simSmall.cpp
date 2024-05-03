// cv2simSmall.cpp : Defines the entry point for the console application.
//
// ***** Camera Image  To  Simulation Image comparison *********
// ***** This version first tested June/July 2006      *********
//
// 
// Compare a camera image to a simulation view that contains basically
// the same scene and say what the change in camera viewpoint for the 
// simulation should be to line up with the camea view
//
// Uses the Corner Library developed by Miroslav Trajkovic
// Uses the OpenCv library for everything else.
// Still contains quite a bit of code other than the Corner Library which needs to be
// cleaned out and done better
//
// A note on Image formats:
// Opencv like Ipl treats an image as a one-dimensional array
// The Corner library needs two dimensional arrays. So I convert between
// by allocating an array to hold to addresses of the start of rows in the oneD array.
//
// This is done slightly differently in different locations and should be cleaned up.
//
// A note on Matrix formats:
// Opencv now uses the CvMat type for matrices (and images).
// The corner library uses dMatrix (dMatrix.cpp/.h) and also just regular 2D arrays.
// I switch between the two when necessary.
//
// IN THIS VERSION (June 9 2008)
// This just demonstrates the version that worked back in June/July 2006 for an input
// consisting of a camera image of a book and an Ogre world model camera image of the book.
// some care was taken that they be similar, but not identical. This is Book Example 1.
//
// To run the program:
// Click on one of the image windows, and every time the program pauses, press a key to continue.
// It will show you:
// 1. The two grey level images
// 2. The corners located at the most detailed level in each image
// 3. The affine transformation matrix when the corners are matched
// 4. The first image warped by the transformation (so you can see how close it is to the second)
// 5. The resultation areas of differences between the images (should be minimal)
// 
// Damian Lyons Fordham RCV Lab 2008

#include "cv.h"
#include "highgui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <memory.h>

#ifdef _EiC
#define WIN32
#endif

#define min(a,b) ((a)<(b))?(a):(b)

// These are all the corner library inclides    
#include "CornerClass.h"
#include "dmatrix.h"
#include "Candidate.h"

#include "robust_estimators.h"

#include "homography.h"
#include "linalg.h"
#include "imagealign.h"
#include "cmotion_detection.h"

#define WAITFORUSER {printf("\nPRESS a key to continue.....\n"); cvWaitKey( 0 );}

/*
 Find and return connected components in image
*/
int connectedcomponents(IplImage *src, CvSeq** contourPtr)
{
 
        IplImage* dst = cvCreateImage( cvGetSize(src), 8, 3 ); // for display purposes
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* contour = 0;
		int ret=0;

        //cvThreshold( src, src, 1, 255, CV_THRESH_BINARY );
        //cvNamedWindow( "Source", 1 );
        //cvShowImage( "Source", src );

        cvFindContours( src, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        cvZero( dst );

		*contourPtr = contour; // start of contour list
		double sizeThreshold = 0.001 * ((double)src->width*src->height); // ratio of image size as size threshold
        printf("Size threshold is %lf\n",sizeThreshold);
		for( ; contour != 0; contour = contour->h_next )
        {
			CvRect box = cvBoundingRect(contour,1);
			float aspRatio = 1.0;//((float)box.height)/((float)box.width);
			double area =  fabs(cvContourArea(contour));
			if (area > sizeThreshold && aspRatio>0.8 && aspRatio< 1.2 ){
				CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
				/* replace CV_FILLED with 1 to see the outlines */
				cvDrawContours( dst, contour, color, color, -1, CV_FILLED, 8 );
				printf("Size  is %lf aspect is %lf \n",area, aspRatio);
				ret++;
			}
        }

		printf("There are a total of %d components above min size & shape.\n",ret);
        cvShowImage( "Components", dst );
        WAITFORUSER

		cvReleaseImage(&dst);

		return ret;
}




/*
 This routine takes as input an image at three levels of resolution (img1..img3)
 and outputs a corner list for each one (pCLO..pCL2)
*/

void compareCam2Sim(unsigned char **img1, 
					unsigned char **img2, 
					unsigned char **img3, 
					unsigned char **res, // result image for display
					int Width, int Height,
					CORNER_LIST *pCL0,CORNER_LIST *pCL1,CORNER_LIST *pCL2 )
{
 CCorner			*pCCorner, *pCCornerQuart, *pCCornerHalf;

 unsigned char*		corner_data;
 
 int				thr[2] = {0, 10};

 int    image_data_size = Width*Height;

 corner_data = new unsigned char [image_data_size];
	
 pCCorner = new CCorner(Width, Height, MIC); 
 pCCornerHalf = new CCorner(Width/2, Height/2, MIC);
 pCCornerQuart = new CCorner(Width/4, Height/4, MIC);
	
 // full resolution
 pCCorner->DetectCorners(img1, thr);
 pCCorner->CreateCornerImage(img1,res);// show the result on the image
 CreateCornerList(pCCorner, pCL0);

//half resolution
 pCCornerHalf->DetectCorners(img2, thr);
 //pCCornerHalf->CreateCornerImage(img2,res);
 CreateCornerList(pCCornerHalf, pCL1);

//quarter resolution
 pCCornerQuart->DetectCorners(img3, thr);
 //pCCornerQuart->CreateCornerImage(img3,res);
 CreateCornerList(pCCornerQuart, pCL2);

 return;

}

/* 

  Helper routine to make a 2D image from a 1D image.
  IPL and the Corner Library use different image formats.

  This routine allocates memory and makes a 2D image from an IPL 1D image.
  Its done as a special case ..
  it only works for 8 bit, 1 channel (>1 channel multiply width by num channels)
  Note that the Alloc2Duchar routine does something similar .. clean this up!
*/

unsigned char **iplImage2twoDImage(unsigned char *p_Image1D, int width, int height)
{int i;

  unsigned char **p_Image2D = new unsigned char *[height]; // allocate space for row address list

  p_Image2D[0] = p_Image1D;
   
  for (i=height-1;i>=0;i--)
		p_Image2D[i] = &(p_Image1D[i*width]); // make the row address list, pointing into the 1D array

  return p_Image2D;
}
/*
   Difference detection
   Find areas of difference between the warped real and the simulted views
   Are they compact or not?
   Produce a list of compact regions or a fail
*/
int cv2simDifferenceMask(IplImage *imgA, IplImage *imgB, IplImage *imgDiff, int threshold,
					 int nMatches, int matches[][4], dMatrix *Mr)
{int i,j,k,w, id, jd; double eSum=0.0;
	// create the warp mask

	IplImage* bw_mask = cvCreateImage(
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1); // match mask
	IplImage* bw_diff1 = cvCreateImage(
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1); // match masked image
	IplImage* bw_diff2 = cvCreateImage(
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1); // warp masked image

	 threshold=5;
	 cvSmooth(imgA,imgA,CV_GAUSSIAN,5,5);
	 cvSmooth(imgB,imgB,CV_GAUSSIAN,5,5);	
	 cvSmooth(imgA,imgA,CV_GAUSSIAN,5,5);
	 cvSmooth(imgB,imgB,CV_GAUSSIAN,5,5);	
	//cvThreshold(imgA,imgA, threshold,255,CV_THRESH_BINARY);
	//cvThreshold(imgB,imgB, threshold,255,CV_THRESH_BINARY);

	cvAbsDiff(imgA, imgB, bw_diff1);
	//cvThreshold(imgDiff,imgDiff, threshold,255,CV_THRESH_BINARY);

	cvShowImage( "motionAB",bw_diff1);
	printf("This is the direct difference of warped A and B \n");
	WAITFORUSER
  
	cvCopy(bw_diff1,bw_mask); // to show the matches

	// (1,0) is in the unwarped, warped image coordines imgA
	// (3,2) is in the original image imgB

	// Mark the image for diagnosis
	CvScalar sw,sb;
	sw.val[0]=255;sb.val[0]=0;
	for (k=0; k<nMatches; k++) {
		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2],sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2],sb);
		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2]+1,sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2]+1,sw);
		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2]-1,sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2]-1,sw);
	}
	printf("This is the set of matches used to calculate the mask on the image difference\n");
	cvShowImage( "motionAB",bw_mask);

	WAITFORUSER

	// calculate the affine transform error per match and sum of error
	dMatrix src(1,3), *dst; int it, jt;
	double mWeights[1000],mSum=0, dist;
	for (k=0; k<nMatches; k++) {
        src.a[0][0] = matches[k][1]; 
	    src.a[0][1] = matches[k][0];
		src.a[0][2] = 1.0;
		dst = mult_matrixp(src,*Mr);
		it=dst->a[0][0]; jt=dst->a[0][1];
		dist  = (matches[k][3]-it)*(matches[k][3]-it)
					       +(matches[k][2]-jt)*(matches[k][2]-jt);
		if (dist>0) mWeights[k] = 1.0/sqrt(dist);
		else mWeights[k]=0;

		mSum += mWeights[k];
	}
	printf("Sum of error = %lf\n",mSum);

	// calculate the sum of gaussians
	CvScalar s,q, r; 
	char *imgptr= bw_mask->imageData;
	int *matchptr=0;
	// get the image data
	int height    = bw_mask->height;
	int width     = bw_mask->width;
	int step      = bw_mask->widthStep;
	int channels  = bw_mask->nChannels;
	
	FILE *mlog = fopen(".//mlogfile.csv","w");

	double sx=5.0, sy=5.0, eGain=1000;
    for (j=0; j<height; j++)
		for (i=0; i<width; i++) 
			for (k=0; k<channels; k++) {
				eSum=0.0; 
				for (w=0; w<nMatches; w++) {
					matchptr = &(matches[w][0]);
  					id =(i-matchptr[3]); jd = (j-matchptr[2]);
					eSum += (mWeights[w]/mSum)*exp((double) -(id*id/(2.0*sx*sx) + jd*jd/(2.0*sy*sy)));
				}
				eSum *=eGain;
		
				imgptr[j*step+i*channels+k] = min(255,(int)(eSum * 255.0));
				fprintf(mlog,"%d ",imgptr[j*step+i*channels+k]);
				if (i<width-1) fprintf(mlog,","); else fprintf(mlog,"\n");
			}

    fclose(mlog);
    printf("This is the mask based on imgA\n");
	cvShowImage( "motionAB",bw_mask);
	WAITFORUSER

    cvThreshold(bw_mask,bw_mask, 10 ,255,CV_THRESH_BINARY_INV);

    printf("This is the mask thresholded based on imgA\n");
	cvShowImage( "motionAB",bw_mask);
	WAITFORUSER
	
    cvSetZero(imgDiff); // clear the img difference array
	cvSetZero(bw_diff2);
	cvCopy(bw_diff1,bw_diff2,bw_mask); // copy across only the (inverted) match mask areas
	cvCopy(bw_diff2,imgDiff,imgA); // filter black 'unwarped' area
	printf("And this is the direct subtracted image masked by bw_mask AND imgA \n");
	cvShowImage( "motionAB",imgDiff);
	WAITFORUSER

    printf("And this is the thresholded subtracted image \n");
	threshold = 50;
    cvThreshold(imgDiff,imgDiff, threshold,255,CV_THRESH_BINARY);
	cvShowImage( "motionAB",imgDiff);
	WAITFORUSER

	cvReleaseImage(&bw_mask);
	cvReleaseImage(&bw_diff1);
	cvReleaseImage(&bw_diff2);
		
	return 0;
}
/*
 Use multiplication instead of binarymask
*/

int cv2simDifference(IplImage *imgA, IplImage *imgB, IplImage *imgDiff, int threshold,
					 int nMatches, int matches[][4], dMatrix *Mr, int scale)
{int i,j,k,w, id, jd; double eSum=0.0;
	// create the warp mask

	IplImage* bw_mask = cvCreateImage(
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1); // match mask
	IplImage* bw_diff1 = cvCreateImage(
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1); // match masked image
	IplImage* bw_diff2 = cvCreateImage(
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1); // warp masked image
	 
//	printf("This is A\n"); cvShowImage( "motionAB",imgA);
//	WAITFORUSER
//	printf("This is B\n"); cvShowImage( "motionAB",imgB);
//	WAITFORUSER
		
	 //cvEqualizeHist(imgA,imgA);
	 //cvEqualizeHist(imgB,imgB);

	cvCopy(imgA,bw_mask); // Show warped image with matched points
	CvScalar sw,sb;
	sw.val[0]=255;sb.val[0]=0;
	for (k=0; k<nMatches; k++) {
		for (i=0; i<4; i++) matches[k][i] /= scale;

		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2],sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2],sb);
		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2]+1,sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2]+1,sw);
		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2]-1,sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2]-1,sw);
	}
	printf("This is the set of matches used to calculate the mask on the image difference\n");
	cvShowImage( "motionAB",bw_mask);
	WAITFORUSER

	 cvSmooth(imgA,imgA,CV_GAUSSIAN,5,5);
	 cvSmooth(imgB,imgB,CV_GAUSSIAN,5,5);	
	 cvSmooth(imgA,imgA,CV_GAUSSIAN,5,5);
	 cvSmooth(imgB,imgB,CV_GAUSSIAN,5,5);	

	 cvAbsDiff(imgA, imgB, bw_diff1);

    printf("This is the direct difference of warped A and B \n");
 	cvShowImage( "motionAB",bw_diff1);
	WAITFORUSER

	threshold *= 1;
    printf("And this is the thresholded (%d) subtracted image \n", threshold);
    cvThreshold(bw_diff1,bw_diff1, threshold,255,CV_THRESH_BINARY);	
	cvShowImage( "motionAB",bw_diff1);
    WAITFORUSER
  
	cvAbsDiff(imgA, imgB, bw_diff1); // restore
	cvCopy(bw_diff1,bw_mask); // to show the matches

	// (1,0) is in the unwarped, warped image coordines imgA
	// (3,2) is in the original image imgB

	// Mark the image for diagnosis
	//CvScalar sw,sb;
	sw.val[0]=255;sb.val[0]=0;
	for (k=0; k<nMatches; k++) {
		for (i=0; i<4; i++) matches[k][i] /= scale;

		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2],sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2],sb);
		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2]+1,sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2]+1,sw);
		cvSet2D(bw_mask,matches[k][3]+1,matches[k][2]-1,sw);
		cvSet2D(bw_mask,matches[k][3],  matches[k][2]-1,sw);
	}
	printf("This is the set of matches used to calculate the mask on the image difference\n");
	cvShowImage( "motionAB",bw_mask);

	WAITFORUSER

	// calculate the affine transform error per match and sum of error
	dMatrix src(1,3), *dst; int it, jt;
	double mWeights[1000],mSum=0, dist;
	for (k=0; k<nMatches; k++) {
        src.a[0][0] = matches[k][1]; 
	    src.a[0][1] = matches[k][0];
		src.a[0][2] = 1.0;
		dst = mult_matrixp(src,*Mr);
		it=dst->a[0][0]; jt=dst->a[0][1];
		dist  = (matches[k][3]-it)*(matches[k][3]-it)
					       +(matches[k][2]-jt)*(matches[k][2]-jt);
		if (dist>0) mWeights[k] = 1.0/sqrt(dist);
		else mWeights[k]=0;

		mSum += mWeights[k];
	}
	printf("Sum of error = %lf\n",mSum);

	// calculate the sum of gaussians
	CvScalar s,q, r; 

	cvZero(bw_diff2);
	cvZero(bw_mask);

	char *maskptr= bw_mask->imageData; // mask image, for diagnostic display
	char *img1ptr= bw_diff1->imageData; // difference image by subtraction
	char *img2ptr= bw_diff2->imageData; // result of mask mediated subtraction

	FILE *mlog = fopen(".//mlogfile.csv","w");
	int height    = bw_mask->height;
	int width     = bw_mask->width;
	int step      = bw_mask->widthStep;
	int channels  = bw_mask->nChannels; 
	int offset=0, newmask, newval;
	double sx=5.0, sy=5.0, eGain=50;
    for (j=0; j<height; j++)
		for (i=0; i<width; i++) 
			for (k=0; k<channels; k++) {
				eSum=0.0; 
				for (w=0; w<nMatches; w++) {
  					id =(i-matches[w][2]); jd = (j-matches[w][3]);
					eSum += (mWeights[w]/mSum)*exp((double) -(id*id/(2.0*sx*sx) + jd*jd/(2.0*sy*sy)));
				}
				eSum *=eGain;
			
				offset = j*step+i*channels+k;
				newmask = (int)(eSum * 255.0);
				if (newmask>255) newmask=255; else if (newmask<0) newmask=0;

				if (newmask>0)
				 newval =  (int)((double)img1ptr[offset]/(double)maskptr[offset]);
				else newval = img1ptr[offset];

				if (newval>255) newval=255; else if (newval<0) newval=0;

				img2ptr[offset] = newval; //min( 255, max(1,newval));
				maskptr[offset] = newmask;

				if (i%2==0) 	fprintf(mlog,"%d,",(unsigned char)maskptr[offset]);
				if (i==width-1) fprintf(mlog,"0\n");
				
			}


    printf("This is the mask based on imgA\n");
	cvShowImage( "Components",bw_mask);
	fclose(mlog);

	WAITFORUSER

    //cvThreshold(bw_mask,bw_mask, 10 ,255,CV_THRESH_BINARY_INV);
    printf("This is the mask mediated image difference\n");
	cvShowImage( "motionAB",bw_diff2);
	WAITFORUSER

	
    cvSetZero(imgDiff); // clear the img difference array
	cvCopy(bw_diff2,imgDiff,imgB); // filter black 'unwarped' area

	printf("And this is the image difference mediated by bw_mask AND imgA \n");
	cvShowImage( "motionAB",imgDiff);
	WAITFORUSER

    printf("And this is the thresholded subtracted image \n");
	threshold = 100;
    cvThreshold(imgDiff,imgDiff, threshold,255,CV_THRESH_BINARY);
	cvShowImage( "motionAB",imgDiff);
	WAITFORUSER

	cvReleaseImage(&bw_mask);
	cvReleaseImage(&bw_diff1);
	cvReleaseImage(&bw_diff2);

		
	return 0;
}
/*

  Main for the Camera Image  To  Simulation Image comparison program
  This is overly long and should be more modular.

  dml June 2008.
*/


int main( int argc, char** argv )
{
   //File names for the book example one test sequence
	// BookImages1 0000 real, 0001 simulation : Small differences
	// BookImages2 0001 real, 0000 simulation : Larger difference and out of view
	// BookImages2 0009 real, 0008 simulation : Larger difference and new real object

  //   char* filename1 = (char*)"../BookImages2/name_0001.bmp"; 
  //   char* filename2 = (char*)"../BookImages2/name_0000.bmp"; 

   //char* filename1 = (char*)"BookImages2\\name_0009.bmp"; 
   //char* filename2 = (char*)"BookImages2\\name_0008.bmp"; 

   //char* filename1 = (char*)"BookImages1\\name_0000.bmp"; 
   //char* filename2 = (char*)"BookImages1\\name_0001.bmp"; 

  //char* filename1 = (char*)"LabWallImages/20081013Blabwall_0004.bmp"; 
  //char* filename2 = (char*)"LabWallImages/tilt_up.bmp"; /* left_3 is closest */
 
  //char* filename1 = (char*)"../LabWallImages/20081013Blabwall_0001.bmp"; 
  //   char* filename1 = (char*)"../LabWallImages/20081013Blabwall_0001.bmp"; 
  //char* filename2 = (char*)"../LabWallImages/ogreshot1.bmp"; /* left_3 is closest */

    //     char* filename1 = (char *)"bigimage.bmp";
    //  char* filename2 = (char*)"LabWallImages/boxtest3.bmp"; /* left_3 is closest */
 
  char* filename1 = (char*)"../LabWallImages/box_0007.bmp"; 
  char* filename2 = (char*)"../LabWallImages/left_3.bmp"; /* left_3 is closest */
 
 
 CORNER_LIST* pCL0 = new CORNER_LIST; // for image A all resolutions, most to least
 CORNER_LIST* pCL1 = new CORNER_LIST;
 CORNER_LIST* pCL2 = new CORNER_LIST;

 CORNER_LIST* pCL3 = new CORNER_LIST; // for image B all resolutions, most to least
 CORNER_LIST* pCL4 = new CORNER_LIST;
 CORNER_LIST* pCL5 = new CORNER_LIST;
 int g_threshold=60; // should use the same throughout

 IplImage* imgA = cvLoadImage( filename1, 1 );

 printf("image A size: %d, %d\n",imgA->width, imgA->height);
 
 cvNamedWindow( "imageA", 1 );
 cvShowImage( "imageA", imgA );// Show camera image

WAITFORUSER


 IplImage* imgB = cvLoadImage( filename2, 20 );
 
 cvNamedWindow( "imageB", 1 );
 cvShowImage( "imageB", imgB ); // Show Ogre image
 printf("Showing the Real Camera Image and the Ogre Simulated Image..\n");
 printf("F1=%s, F2=%s\n",filename1,filename2);

 //make a bunch of windows for later...

 cvNamedWindow( "cornerA",  100 );
 cvNamedWindow( "cornerB",  200 );
 cvNamedWindow( "motionAB", 300 );

 cvNamedWindow( "Components", 400 );


 // IPL images at 3 resolutions for the Real Camera Image

 IplImage* bw_imgA = cvCreateImage(
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1);



 IplImage* bw_img_halfA = cvCreateImage(
	   cvSize(bw_imgA->width/2,bw_imgA->height/2),IPL_DEPTH_8U,1);

 IplImage* bw_img_quartA = cvCreateImage(
	   cvSize(bw_imgA->width/4,bw_imgA->height/4),IPL_DEPTH_8U,1);

 cvCvtColor(imgA, bw_imgA, CV_BGR2GRAY); // flatten to B and W
 //cvEqualizeHist(bw_imgA,bw_imgA);
 cvScale(bw_imgA, bw_imgA, 1, -50);//-80); // added this for the lab wall ogre image 101408
  cvSmooth(bw_imgA,bw_imgA,CV_GAUSSIAN,5,5); // lab wall real image to omany edges?

 cvPyrDown(bw_imgA, bw_img_halfA);

 cvPyrDown(bw_img_halfA, bw_img_quartA);
 
 cvShowImage("imageA",bw_img_halfA);


 // IPL images at 3 resolutions for the Ogre image
 
 IplImage* bw_imgB = cvCreateImage(
	   cvSize(imgB->width,imgB->height),IPL_DEPTH_8U,1);

 IplImage* bw_img_halfB = cvCreateImage(
	   cvSize(bw_imgB->width/2,bw_imgB->height/2),IPL_DEPTH_8U,1);

 IplImage* bw_img_quartB = cvCreateImage(
	   cvSize(bw_imgB->width/4,bw_imgB->height/4),IPL_DEPTH_8U,1);

 cvCvtColor(imgB, bw_imgB, CV_BGR2GRAY); // flatten to B and W
 //cvEqualizeHist(bw_imgB,bw_imgB);
 // cvScale(bw_imgB, bw_imgB, 1, 50); // added this for the lab wall ogre image 101408

 cvPyrDown(bw_imgB, bw_img_halfB);

 cvPyrDown(bw_img_halfB, bw_img_quartB);
 
 cvShowImage("imageB",bw_img_halfB);



  // Need to translate the images to the corner library format, real camera first

 unsigned char **uimgA = iplImage2twoDImage((unsigned char *)bw_imgA->imageData,
	                                                bw_imgA->width,bw_imgA->height);

 unsigned char **uimgA2 = iplImage2twoDImage((unsigned char *)bw_img_halfA->imageData,
													bw_img_halfA->width,bw_img_halfA->height);

 unsigned char **uimgA4 = iplImage2twoDImage((unsigned char *)bw_img_quartA->imageData,
													bw_img_quartA->width,bw_img_quartA->height);

 // Some diagnostic imagery 

 IplImage* bw_result = cvCreateImage(
	   cvSize(bw_imgA->width,bw_imgA->height),IPL_DEPTH_8U,1); // Used for displaying results

 IplImage* bw_half_result = cvCreateImage(
	   cvSize(bw_img_halfA->width,bw_img_halfA->height),IPL_DEPTH_8U,1); // Used for displaying results


 unsigned char **uresult = iplImage2twoDImage((unsigned char *)bw_result->imageData,
													bw_result->width,bw_result->height);
 
 
  cvAbsDiff(bw_imgB, bw_imgA, bw_result);

  printf("This is the direct difference of unwarped A and B \n");
  cvShowImage( "motionAB",bw_result);
  WAITFORUSER

  printf("And this is the thresholded (%d) subtracted image \n",g_threshold);
  cvThreshold(bw_result,bw_result, g_threshold,255,CV_THRESH_BINARY);	
  cvShowImage( "motionAB",bw_result);
  WAITFORUSER
 
 
 // Corners for real camera image at all resolutions

 cvCopy(bw_imgA, bw_result); // to make diagostic display easier to understand
 compareCam2Sim(uimgA, uimgA2,uimgA4, uresult,
	              bw_imgA->width, bw_imgA->height,
				  pCL0, pCL1, pCL2);

 printf("Corners high resolution for Image A: Real Camera\n");
   
 cvShowImage( "cornerA", bw_result ); // show these to the user

 //WAITFORUSER

 // Translate Ogre images to the corner library format

 unsigned char **uimgB = iplImage2twoDImage((unsigned char *)bw_imgB->imageData,
	                                                bw_imgB->width,bw_imgB->height);

 unsigned char **uimgB2 = iplImage2twoDImage((unsigned char *)bw_img_halfB->imageData,
													bw_img_halfB->width,bw_img_halfB->height);

 unsigned char **uimgB4 = iplImage2twoDImage((unsigned char *)bw_img_quartB->imageData,
													bw_img_quartB->width,bw_img_quartB->height);

 // Corners at all resolutions for these images

 cvCopy(bw_imgB, bw_result); // makes corner display easier to understand
 compareCam2Sim(uimgB, uimgB2,uimgB4, uresult,
	              bw_imgB->width, bw_imgB->height,
				  pCL3, pCL4, pCL5);

 printf("Corners at highest resolution for Image B: Ogre Image\n");
   
 cvShowImage( "cornerB", bw_result ); // show these
 
 //WAITFORUSER

 // Now do the matching


 int scale=1;
 int matches[1000][4], nMatches; // image to image match list

 dMatrix Mc(3,3), // Real camera motions if known
	     Mr(3,3); // calculated camera motions


 Mc.set2identity(); // Assume no motion was intended

   //Mc = ComputeHomography(*ptz1, *ptz2); // if you know the two locations use this
   //scaleMatrix(Mc, 0.5f);

 Mr.set2identity();

 printf("Calculating Mr from the two high resolution images....\n");
  
 Mr = AllignImageAffineM(pCL0, pCL3, &Mc, 1, scale, 1, matches, &nMatches); // will print values of Mr
 
 
 //WAITFORUSER


 // Show the effect of the calculated transform
//#define BWHALF 
 
#ifdef BWHALF
 scale =2;
#else
 scale=1;
#endif

 printf("Warping A to B at scale %d\n",scale);

 // Need to go from the corner library's dMatrix to OpenCv's Cvmat
 CvMat *cvM = cvCreateMat(3,3,CV_64FC1); 
 for(int j=0;j<2;j++) Mr.a[j][2] /= scale;
 for(int i=0;i<3;i++) {
	 for(int j=0;j<3;j++) 
	 { cvSetReal2D(cvM,i,j,(Mr.a[i][j]) ); printf("%lf ",Mr.a[i][j]); } 
	  printf("\n");
	 }

 IplImage* bw_imgAW = cvCreateImage( // create an image to show the result of warping
	   cvSize(imgA->width,imgA->height),IPL_DEPTH_8U,1);

 IplImage* bw_img_halfAW = cvCreateImage( // create an image to show the result of warping
	   cvSize(bw_img_halfA->width,bw_img_halfA->height),IPL_DEPTH_8U,1);



 
#ifdef BWHALF
 cvWarpPerspective( bw_img_halfA, bw_img_halfAW, cvM); // Warp A 
#else
 cvWarpPerspective( bw_imgA, bw_imgAW, cvM); // Warp A 
#endif

 #ifdef BWHALF
 cvShowImage( "motionAB", bw_img_halfAW );
 #else
 cvShowImage( "motionAB", bw_imgAW ); // replace A with warped A
#endif

 WAITFORUSER
#ifdef BWHALF
 cvShowImage( "cornerA", bw_img_halfAW ); // replace A with warped A
 #else
 cvShowImage( "cornerA", bw_imgAW ); // replace A with warped A
#endif

#ifdef BWHALF
cv2simDifference(bw_img_halfAW, bw_img_halfB, bw_half_result, 
				 g_threshold, nMatches, matches, &Mr, scale);
#else
cv2simDifference(bw_imgAW, bw_imgB, bw_result, 
				 g_threshold, nMatches, matches, &Mr, scale);
#endif

#ifdef BWHALF
 cvShowImage( "motionAB", bw_half_result ); // show the motion image
#else
 cvShowImage( "motionAB", bw_result ); // show the motion image
#endif
 printf("Showing result of detection in motionAB\n");
 WAITFORUSER

			// do connected components on this image
 int numBlobs=0;
 CvSeq *blobs = 0, *blobPtr;
#ifdef BWHALF
 numBlobs = connectedcomponents(bw_half_result, &blobs);
#else
 numBlobs = connectedcomponents(bw_result, &blobs);
#endif



	// Release all the many images used

    cvReleaseImage( &imgA );
    cvReleaseImage( &bw_imgA );
    cvReleaseImage( &bw_img_halfA );
    cvReleaseImage( &bw_img_quartA );
	cvReleaseImage( &bw_imgAW );

    cvReleaseImage( &imgB );
    cvReleaseImage( &bw_imgB );
    cvReleaseImage( &bw_img_halfB );
    cvReleaseImage( &bw_img_quartB );

	free(uimgA);
	free(uimgA2);
	free(uimgA4);

	free(uimgB);
	free(uimgB2);
	free(uimgB4);

    cvDestroyWindow("cornerA");
    cvDestroyWindow("cornerB");
    cvDestroyWindow("motionAB");
    cvDestroyWindow("imageA");
    cvDestroyWindow("imageB");


	return 0;

}
