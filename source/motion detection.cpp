#include "../Handoff/stdafx.h" // dml 6/15

#include "tracker.h"
#include "median.h"
#include "brConvolv.h"
#include "label.h"
#include "brMemalloc.h"
#include "robust_estimators.h"
#include "motion.h"
#include "homography.h"
#include "linalg.h"
#include <math.h>
#include <time.h>
#include <memory.h>
#include "mil.h"

#include "ipl.h"
#include "cv.h"

// Globals set by the BITE GUI

#ifndef _NON_ALLEGIANT // dml 6/15
#include "../Handoff/handoff.h"
#else
#include "NonAllegiant.h"
#endif

#include "../Handoff/BITEDialog.h" // dml 6/15


extern BITEDialog biteDialog;
FILE* pFileVreme = fopen("D:\\users\\kori\\vreme.txt", "wt");
FILE* pFileOF = fopen("D:\\users\\kori\\of.txt", "wt");
clock_t start, end;



int Nc = 0;
//FILE* pFile = fopen("D:\\users\\kori\\Box.txt","wt");
FILE* pFile = fopen("D:\\users\\kori\\trace.txt","wt");
FILE* pFilePTZ = fopen("D:\\users\\kori\\PTZ.txt","wt");

int g_ncounter=0;
int f_counter = 10;

void CTracker::TrackTarget()
{
	if( tracker_type == ColorTracker)
		TrackTargetColor();//TestTrackContinuosMCTarget();//
	else
		TrackTargetMC();
}

void CTracker::TrackTargetColor()
{
	int i = 500, DeltaPanTiltTicks[2], move_camera_flag;
	int nXDelta, nYDelta, nXVel, nYVel, nCam; 
	dMatrix Q(3,3), R(3,3), iQ(3,3);
	CPTZRadians ptz;
	Q.a[0][0] = pCamPar->lfFx;
	Q.a[1][1] = pCamPar->lfFy;
	Q.a[0][2] = pCamPar->plfPrincipalPoint[0];
	Q.a[1][2] = pCamPar->plfPrincipalPoint[1];

	//pCPTZController->getCameraPosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks));
	ptz_old = ptz_current;
//	pCPTZController->getRepetitivePosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks)); // dml 6/14 REP UPDATE
//    TRACE("pos=%f \n",ptz.m_xTicks);
//	ptz_current = ptz;		mdt 07/25/00

	float tilt_angle, DeltaPanTilt[2], temp = float(0.89*BR_PI_OVER_2);
	tilt_angle = (float)ptz.m_yTicks;
//	move_camera_flag = startTracking(ptz, POI);
	move_camera_flag = startContinuosMCTracking(POI);

//	TestRegistration(ptz_current, ptz);

	if (nTargetLabel>=0) // targetlabel is -1 if no target was seen, else a target was seen
		move_camera_flag = 1; // dml 2/15/00, 'cos vel control demands it
	else
		move_camera_flag=0;

	move_camera_flag = 1;

	if(move_camera_flag == 1)
	{
		nXDel = nXDelta = 4*POI[0] - Width/2;
		nYDel = nYDelta = 4*POI[1] - Height/2;
		
		pCPTZController->CalculateCameraVelocity(ptz.m_zTicks=170,nXDelta,nYDelta, &nXVel, &nYVel);
		nXDel=nXVel;
		nYDel=nYVel;
		pCPTZController->setCameraVelocity(nXVel, nYVel,0);

		POI[0] = Width/8;	// update the POI so that target is back in the centre after
		POI[1] = Height/8;	// camera moved
	}
	else 
			pCPTZController->setCameraVelocity(0,0,0);

}
 
void CTracker::TrackTargetMC()
{
	int i = 500, DeltaPanTiltTicks[2], move_camera_flag, nCam;
	CPTZRadians ptz;
	dMatrix Q(3,3), R(3,3), iQ(3,3);
	Q.a[0][0] = pCamPar->lfFx;
	Q.a[1][1] = pCamPar->lfFy;
	Q.a[0][2] = pCamPar->plfPrincipalPoint[0];
	Q.a[1][2] = pCamPar->plfPrincipalPoint[1];

	ptz_old = ptz_current;
	pCPTZController->getCameraNumber(&nCam);
	pCPTZController->setCameraNumber(nCam);
	pCPTZController->getCameraPosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks));
//	pCPTZController->getRepetitivePosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks)); // dml 6/14 REP UPDATE
    TRACE("pos=%f \n",ptz.m_xTicks);

	float tilt_angle, DeltaPanTilt[2], temp = float(0.89*BR_PI_OVER_2);
	tilt_angle = (float)ptz.m_yTicks;
	ptz_current = ptz;

	move_camera_flag = startColorTracking(ptz, POI);

	nXDel=POI[0]; nYDel=POI[1];
//	move_camera_flag = 0;	//This only fon NonAllegiant - mdt
	
/*	if(move_camera_flag == 1 && f_counter < 10)
	{
		f_counter++;
		pCPTZController->getCameraNumber(&nCam);
		pCPTZController->setCameraNumber(nCam);
		pCPTZController->getCameraPosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks));
		fprintf(pFilePTZ,"%8.6f %8.6f %4d \n",(ptz.m_xTicks), (ptz.m_yTicks), (ptz.m_zTicks));
		fwrite(secondImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);
		if(f_counter == 10)
			fclose(pFileImage);
	}
*/
	if(move_camera_flag == 1)
	{
//		nFirstFlag = true;	commented for continuos tracker - mdt
		// ZOOMING ENABLED OR NOT 
//		CameraControl(tilt_angle, POI, DeltaPanTilt, DeltaPanTiltTicks, &(ptz.m_zTicks) );  // Zooming implemented
 		CameraControl(tilt_angle, POI, DeltaPanTilt, DeltaPanTiltTicks);		//No Zoom

		ptz.m_xTicks+=DeltaPanTilt[0]; 
		ptz.m_yTicks+=DeltaPanTilt[1]; 
		if(ptz.m_yTicks> temp)
		{
			tilt_angle = tilt_angle;
		}
		pCPTZController->setCameraPositionRadians(ptz.m_xTicks, ptz.m_yTicks, 170/*ptz.m_zTicks*/);
		POI[0] = Width/8;	// update the POI so that target is back in the centre after
		POI[1] = Height/8;	// camera moved
		SetRectangle(&SR, 0, 0, 0, 0);
	}
}


void CTracker::TrackContinuosMCTarget()
{
	int i = 500, DeltaPanTiltTicks[2], move_camera_flag;
	CPTZRadians ptz;
	int nXDelta, nYDelta, nXVel, nYVel, nCam; 

	dMatrix Q(3,3), R(3,3), iQ(3,3);
	Q.a[0][0] = pCamPar->lfFx;
	Q.a[1][1] = pCamPar->lfFy;
	Q.a[0][2] = pCamPar->plfPrincipalPoint[0];
	Q.a[1][2] = pCamPar->plfPrincipalPoint[1];

	ptz_old = ptz_current;
	pCPTZController->getCameraNumber(&nCam);
	//Sleep(1000);
	pCPTZController->setCameraNumber(nCam);
	pCPTZController->getCameraPosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks));
    TRACE("pos=%f \n",ptz.m_xTicks);

	float tilt_angle, DeltaPanTilt[2], temp = float(0.89*BR_PI_OVER_2);
	tilt_angle = (float)ptz.m_yTicks;
	ptz_current = ptz;

	move_camera_flag = startContinuosMCTracking(ptz, POI);

	nXDel=POI[0]; nYDel=POI[1];

	if(move_camera_flag == 1)
	{
//		nFirstFlag = true;	commented for continuos tracker - mdt
		// ZOOMING ENABLED OR NOT 
//		CameraControl(tilt_angle, POI, DeltaPanTilt, DeltaPanTiltTicks, &(ptz.m_zTicks) );  // Zooming implemented
 		CameraControl(tilt_angle, POI, DeltaPanTilt, DeltaPanTiltTicks);		//No Zoom

		ptz.m_xTicks+=DeltaPanTilt[0]; 
		ptz.m_yTicks+=DeltaPanTilt[1]; 
		if(ptz.m_yTicks> temp)
		{
			tilt_angle = tilt_angle;
		}
		pCPTZController->setCameraPositionRadians(ptz.m_xTicks, ptz.m_yTicks, 170/*ptz.m_zTicks*/);
		POI[0] = Width/8;	// update the POI so that target is back in the centre after
		POI[1] = Height/8;	// camera moved
		SetRectangle(&SR, 0, 0, 0, 0);
	}
}


void CTracker::SelectTarget(brVSpace* pViewSpace, int nCameraID, int nPanMax, int nTiltMax, int nZoomMax, int nZoomMin, 
							int nTiltTicks)
{
	float tilt_angle, DeltaPanTilt[2];
	int DeltaPanTiltTicks[2], motion_flag = 0;
	CPTZRadians ptz;

	
	if(pCPTZController == NULL)
		pCPTZController = new brPTZController(pViewSpace);	// initialize camera controler
	pCPTZController->setCameraNumber(nCameraID);

	getCameraInfo(pViewSpace, nCameraID, nPanMax, nTiltMax, nZoomMax, nZoomMin);
	pCPTZController->getCameraPosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks));
	tilt_angle = ptz.m_yTicks;
//	tilt_angle = TiltTicksToRadians(nTiltTicks);
	current_fx = current_fy = ZoomTicksToFocalLength(ptz.m_zTicks);

  	// DISPLAY ACTIVATION
	SelectMilDisplay();		//temporary change - mdt


	if(IMAGE_TYPE == 0)
	{
		// Grab the first image
		GrabImage(firstImage[0]);
		subsampleImage(firstImage[0], firstImage_coarse);
		int N = 320*240;

		while(motion_flag == 0)
		{
			// Grab the second image
			GrabImage(secondImage[0]);
			subsampleImage(secondImage[0], secondImage_coarse);

			ComputeForegroundImage(firstImage_coarse, secondImage_coarse, foregroundImage[2]);
			motion_flag = IsMotion(foregroundImage[2]);
			ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);

			// DISPLAY 
			foregroundImage[0]->m_PutImage(MIL_params.MilImageDisp); // display related
		} // end while

		computeTorsoHeadBox(tilt_angle, foregroundImage[2], motion_flag, &TorsoHead, POI);

	}//end if
	else
	{
		// Grab the first image
		// BMP add - 02/21
		if (BMPflag==false) 
		GrabImage_Color(firstImage_color, firstImage[0]);
		else 
		ReadBMPImage(firstImage_color, firstImage[0]);

		subsampleImage(firstImage[0], firstImage_coarse);
		int N = 320*240;

		while(motion_flag == 0)
		{
			// Grab the second image
				if (BMPflag==false) 
		GrabImage_Color(secondImage_color, secondImage[0]);
		else 
		ReadBMPImage(secondImage_color, secondImage[0]);

	
			subsampleImage(secondImage[0], secondImage_coarse);
			ComputeForegroundImage(firstImage_coarse, secondImage_coarse, foregroundImage[2]);
			ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);
			motion_flag = IsMotion(foregroundImage[2]);
			
			// DISPLAY 
			//secondImage_color->m_PutImage_Color(MIL_params.MilImageDispColor);
			DisplayBMP(secondImage_color->m_GetImage()); // dml 6/15
			// secondImage[0]->m_PutImage(MIL_params.MilImageDisp);
			// foregroundImage[0]->m_PutImage(MIL_params.MilImageDisp);

		} // end while

		computeTorsoHeadBox(tilt_angle, foregroundImage[2], motion_flag, &TorsoHead, POI); 


		// DISPLAY 
		// resultImage_color->m_PutImage_Color(MIL_params.MilImageDispColor);


	}//end else

		// comment added by esc: July 29, 1999
	// MOVE OF THE CAMERA to center the moving person after detection
	// No zoom adaptation at this point
//	CameraControl(tilt_angle, POI, &(ptz->m_yTicks), &dpan);
	CameraControl(tilt_angle, POI, DeltaPanTilt, DeltaPanTiltTicks);
	CPTZRadians(ptz.m_xTicks+=DeltaPanTilt[0], ptz.m_yTicks+=DeltaPanTilt[1], ptz.m_zTicks);
	pCPTZController->setCameraPositionRadians(ptz.m_xTicks, ptz.m_yTicks, ptz.m_zTicks);
	pCPTZController->getCameraPosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks));
}


// comment added by esc: July 27, 1999
// startTracking(float tilt_angle) is operating when the camera is fixed
// and checks if the moving object is not close to the image boundary
// if No the flag move_camera remains = 0  
// if yes the flag move_camera switches to 1 and the program
// left the function startTracking(float tilt_angle)

int CTracker::startTracking(CPTZRadians ptz, int* cPOI)
{
	int move_camera_flag;

	if( tracker_type == ColorTracker)
		move_camera_flag = startColorTracking(ptz, POI);
	else
		move_camera_flag = startContinuosMCTracking(ptz, POI);

	return move_camera_flag;
}

int CTracker::startColorTracking(CPTZRadians ptz, int* cPOI)
{
	milImage<MonochromeImage>	*tempImage_fine;
	tRectangle				PlotBox;
	Image<int>	*tempImage_coarse;
	float tilt_angle = ptz.m_yTicks;
	// Grab the first image
	int move_camera_flag = 0, motion_flag = 0;
	int N = 320*240,i, j;
	tRectangle	Torso_fine;
	unsigned char* p_img;
	dMatrix	Mr(3,3);
	unsigned char *p_src, *p_dst;
	CORNER_LIST* tempCL;
	
	if(nFirstFlag)
	{
		nFirstFlag = 0;
		subsampleImage(firstImage[0], firstImage[1], 2);
		ColorMotion(firstImage_color, firstImage[1], pCCornerHalf, pCL0);
	}
	else
	{

		subsampleImage(secondImage[0], secondImage[1], 2);
		ColorMotion(secondImage_color, secondImage[1], pCCornerHalf, pCL1);
//		pCPTZController->setCameraVelocity(0, 0,0);
		Mr = AllignImage(pCL1, pCL0,1);
		Mr.a[0][2]*=2; Mr.a[1][2]*=2;
		Mr.a[2][0]/=2; Mr.a[2][1]/=2;
		background_update(bgdImage[0]->p_Image2D, firstImage[0]->p_Image2D, secondImage[0]->p_Image2D, Mr, Width, Height);
		for(i=0;i<Height;i++)
		for(j=0;j<Width;j++)
			foregroundImage[0]->p_Image2D[i][j] = abs(bgdImage[0]->p_Image2D[i][j] - secondImage[0]->p_Image2D[i][j]);

		p_dst = resultImage_color->m_GetImage();
		p_src = foregroundImage[0]->m_GetImage();
		for(i=0;i<N;i++) // display an inverted b/w motion picture
		{
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src++;
		}
		DisplayBMP(resultImage_color->m_GetImage()); // dml 6/15/00
//		DisplayBMP(secondImage_color->m_GetImage());
		tempCL = pCL0;
		pCL0 = pCL1;
		pCL1 = tempCL;
		tempImage_fine = firstImage[0];
		tempImage_coarse = firstImage_coarse;
		firstImage[0] = secondImage[0];
		firstImage_coarse = secondImage_coarse;
		secondImage[0] = tempImage_fine;
		secondImage_coarse = tempImage_coarse;

	}//end else

	return 1;
}


int CTracker::startMCTracking(CPTZRadians ptz, int* cPOI)
{
	tRectangle		PlotBox;
	unsigned char *	p_src, *p_dst;
	int move_camera_flag = 0, motion_flag = 0;
	int Ndiv3 = 320*240, N= Ndiv3 *3 ,i = 100;
	float tilt_angle = ptz.m_yTicks;


	if(nFirstFlag)
	{
//		firstImage_color->p_Image2D[0] = p_image_buffer + image_data_size*((current_frame++)%10);
		if (BMPflag==false) 
		GrabImage_Color(firstImage_color, firstImage[0]);
		else 
		ReadBMPImage(firstImage_color, firstImage[0]);
//		memcpy(p_image_buffer + image_data_size*((current_frame++)%10), firstImage[0]->p_Image2D[0], image_data_size); 
		subsampleImage(firstImage[0], firstImage_coarse);

//		secondImage_color->p_Image2D[0] = p_image_buffer + image_data_size*((current_frame++)%10);
		if (BMPflag==false) 
		GrabImage_Color(secondImage_color, secondImage[0]);
		else 
		ReadBMPImage(secondImage_color, secondImage[0]);

//		memcpy(p_image_buffer + image_data_size*((current_frame++)%10), secondImage[0]->p_Image2D[0], image_data_size); 
		subsampleImage(secondImage[0], secondImage_coarse);
		nFirstFlag=false;

		nTargetLabel =
			TrackMCSegment(firstImage_coarse, secondImage_coarse, foregroundImage[2], tilt_angle, &Torso);
	}
	else
	{
//		secondImage_color->p_Image2D[0] = p_image_buffer + image_data_size*((current_frame++)%10);
		if (BMPflag==false) 
		GrabImage_Color(secondImage_color, secondImage[0]);
		else 
		ReadBMPImage(secondImage_color, secondImage[0]);

//		memcpy(p_image_buffer + image_data_size*((current_frame++)%10), secondImage[0]->p_Image2D[0], image_data_size); 
		subsampleImage(secondImage[0], secondImage_coarse);
		nTargetLabel =
			TrackMCSegment(firstImage_coarse, secondImage_coarse, foregroundImage[2], tilt_angle, &Torso);
	}

	SetRectangle(&BBox, (Torso.x_min+2)/4, (Torso.y_min+2)/4, (Torso.x_max+2)/4, (Torso.y_max+2)/4);
	p_src = secondImage_color->m_GetImage();	
	ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);

	if(nTargetLabel > 0) // a moving target was found
	{
		motion_flag = 1;
		POI[0] = (BBox.x_min + BBox.x_max)/2;
		POI[1] = (BBox.y_min + BBox.y_max)/2;
		move_camera_flag = IsMoveCameraTorsoHeadBox(&BBox, motion_flag);
	}
	else // either no target or a static target was found
	{
		if(nTargetLabel == 0) // a stationary target was found
		{
			// dont expand motionframe here, so you can still see old target
			motion_flag = 1;
			move_camera_flag = 0;
			POI[0] = (BBox.x_min + BBox.x_max)/2;
			POI[1] = (BBox.y_min + BBox.y_max)/2;
		}
		else // no target was found
		{
			motion_flag = 0;
			move_camera_flag = 0;
		}
	}

		// ALWAYS switch frames dml 3/25/00
		// if you don't you can get two difference regions
		// of the same person (one from the old ref image and one from the new)

	int image_display = 1;
	if (biteDialog.EnableMILWin) image_display = 0; // dml 6/15/00
	
	p_dst = resultImage_color->m_GetImage();
	if(image_display == 0)
	{
		p_src = foregroundImage[0]->m_GetImage();
		
		for(i=0;i<Ndiv3;i++) // display an inverted b/w motion picture
		{
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src++;
		}
	}
	else
	{
		memcpy( p_dst, p_src, N); // display the original image
	}

		//RGB2HSI(firstImage_color, rgY_Image); // dml 12/2/99
		//ClassifyPixelsFromModel(rgY_Image, resultImage_color, &TargetColor);
		// dml 2/00 Commented out the two lines above to improve speed for demos

	//resultImage_color->m_PutImage_Color( MIL_params.MilImageDispColor );
	

	// dml 12/2/99 putting matrox display calls under this allows
	// them to stay up slightly longer! less flashing

	// Put matrox calls in this IF statement to have them draw only when 
	// real data is available
	if (motion_flag)
	{
		CopyRectangle(&SR, &PlotBox);

		//MgraColor(M_DEFAULT,M_RGB888(255,0,0)); 

		//MgraLine(M_DEFAULT,MIL_params.MilImageDispColor,4*POI[0],4*POI[1],0,0);		// Lines for the visualisation
		//MgraLine(M_DEFAULT,MIL_params.MilImageDispColor,4*POI[0],4*POI[1],319,0);	// of the POI


		//MgraRect(M_DEFAULT,MIL_params.MilImageDispColor, 4*PlotBox.x_min,4*PlotBox.y_min, 4*PlotBox.x_max,4*PlotBox.y_max);

//		#ifdef _DEBUG // only in the debug version
			//MgraColor(M_DEFAULT,M_RGB888(0,0,250)); 
			CopyRectangle(&BBox, &PlotBox, 4);
		
			PlotRectangle(resultImage_color, PlotBox, 1); // dml 6/15
			//MgraRect(M_DEFAULT,MIL_params.MilImageDispColor, 4*PlotBox.x_min,4*PlotBox.y_min, 4*PlotBox.x_max,4*PlotBox.y_max);

			//MgraColor(M_DEFAULT,M_RGB888(0,250, 0)); 
			CopyRectangle(&TorsoHead, &PlotBox, 4);
			
			PlotRectangle(resultImage_color, PlotBox, 1); // dml 6/15
			//MgraRect(M_DEFAULT,MIL_params.MilImageDispColor, 4*PlotBox.x_min,4*PlotBox.y_min, 4*PlotBox.x_max,4*PlotBox.y_max);
//		#endif
	}

	DisplayBMP(resultImage_color->m_GetImage()); // dml 6/15/00

	return move_camera_flag;
}



void CTracker::startTracking()
{
	// Grab the first image
	int move_camera = 0;
	int N = 320*240,i = 1000;
	

	while(move_camera == 0)
	{
		i--;
		//	GrabAndDisplayImage(firstImage[0]);	//changed by mdt
		GrabImage(firstImage[0]);
		subsampleImage(firstImage[0], firstImage_coarse);
	
		// Grab the second image
		GrabImage(secondImage[0]);
		subsampleImage(secondImage[0], secondImage_coarse);
		ComputeForegroundImage(firstImage_coarse, secondImage_coarse, foregroundImage[2]);
		ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);
		foregroundImage[0]->m_PutImage(MIL_params.MilImageDisp);
		move_camera = IsMoveCamera(POI);
	}
}

void CTracker::startTrackingDebug(void* pFile)
{
	// Grab the first image
	int move_camera = 0;
	int N = 320*240,i = 1000;
	

	while(move_camera == 0)
	{
		i--;
		GrabImageDebug(firstImage[0], pFile);
		subsampleImage(firstImage[0], firstImage_coarse);
	
		// Grab the second image
		GrabImage(secondImage[0]);
	
		subsampleImage(secondImage[0], secondImage_coarse);
		ComputeForegroundImage(firstImage_coarse, secondImage_coarse, foregroundImage[2]);
		ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);
		foregroundImage[0]->m_PutImage(MIL_params.MilImageDisp);
		MgraLine(M_DEFAULT,MIL_params.MilImageDisp,4*POI[0],4*POI[1],0,0);		// Lines for the visualisation
		MgraLine(M_DEFAULT,MIL_params.MilImageDisp,4*POI[0],4*POI[1],239,0);	// of the POI
		move_camera = IsMoveCamera(POI);
	}
}

// Modified by esc, July 29 - 1999
// Bug Min=Max initialization fixed 
// the output of ComputeForegroundImage() is the image 
// ComputeForegroundImage which contains only the biggest 
// independent moving objects labeled as 1, 
// the rest is labeled to 0 

void CTracker::ComputeForegroundImage(char ** Image1, char** Image2, char** ForegroundImage)
{
	int i,j, N=Height*Width/16, Min, Max, Median;
	int MedianLocation = int(N/2);
	int x = Width/4, y=Height/4, **tp;
	int *p_img1, *p_img2, *p_imgdif;
	p_img1 = (int*)Image1->m_GetImage();
	p_img2 = (int*)Image2->m_GetImage();
	p_imgdif = (int*)differenceImage_coarse->m_GetImage();
	unsigned char* p_fgd = ForegroundImage->m_GetImage();

	Min = Max = abs((*p_img2 - *p_img1)/16);
	
	//	Compute Difference image and find it's min and max 
	for(i=0;i<N;i++)
	{
		j = p_img2[i] - p_img1[i];
		j = int(abs(j/16));
		p_imgdif[i] = j;
	
		if(Min>j)
			Min = j;
		else
			if(Max<j)
				Max = j;
	}


	//	Compute histogram of difference image
	//  comment added by esc July 29, 1999 
	//  the parameter threshold is used to separate pixels belonging to moving
	//  from others 
	GenerateHistogramFromImage(differenceImage_coarse, 512, histogram, Min, Max);
	Median = GetMedianFromCountingSort(MedianLocation, histogram);
	float sigma =1.4826f*Median;
	float threshold = 2.5f*sigma;
	if(threshold < DIFERENCE_THRESHOLD)
		threshold = DIFERENCE_THRESHOLD;



	// comment added by esc July 29, 1999 
	// differenceImage_coarse->p_Image2D  is binarized to be used for 
	// noise filtering and by the label() function below 
	for(i=N-1;i>=0;i--)
	{
		if(abs(p_imgdif[i])>threshold)
			(p_imgdif[i]) = 1;
		else
			(p_imgdif[i]) = 0;
	}

//	NoiseFiltering(differenceImage_coarse, foregroundImage[2]);
	//	NOISE FILTERING FUNCTION: START DEBUG 
	tp = differenceImage_coarse->p_Image2D;
	int count;
	for(i=1; i<y-1; i++)
	for(j=1; j<x-1; j++)
	{
		if(tp[i][j]>0)
		{
			count = tp[i-1][j-1] + tp[i-1][j] + tp[i-1][j+1] + tp[i][j-1] + tp[i][j+1] + tp[i+1][j-1] + tp[i+1][j] + tp[i+1][j+1];
			if(count>2)
				ForegroundImage->p_Image2D[i][j] = 1;
			else
				ForegroundImage->p_Image2D[i][j] = 0;
		}
		else
			ForegroundImage->p_Image2D[i][j] = 0;
//		ForegroundImage->p_Image2D[i][j] = tp[i][j];
	}
	//	ForegroundImage->p_Image[i] = (differenceImage_coarse->p_Image[i]);
	//	END FILTERING FUNCTION - END DEBUG



	int nbobj, index = 0;
//	Find connected components labeled now from 2 to nbobj+1
	label(ForegroundImage->p_Image2D, 2, y-1, 2, x-1, 3, &nbobj);

//	Allocate memory for objects
	int objects[100];
	for(i=0; i<100; i++)objects[i] = 0;

//	Counting number of pixels for each and every object
	for(i=2; i<y-2; i++)
	for(j=2; j<x-2; j++)
	if(ForegroundImage->p_Image2D[i][j]>0)
		objects[ForegroundImage->p_Image2D[i][j]]++;

//	Find the bigest object (in terms of number of pixels) 
	Max = 20;
	for(i=0;i<100;i++)
	if(objects[i]>Max)
	{
		Max = objects[i];
		index = i;
	}

//	Label the biggest object to 1 and all the others to 0 (background)

	if(index>1)
	{
		index++;
		index--;
	}

	for(i=0;i<N;i++)
	if(p_fgd[i] > 0)
	{
		if(p_fgd[i] != index)
			p_fgd[i] = 0;
		else p_fgd[i] = 2;
	}
}
	
void CTracker::ExpandMonochromeImage(Image<MonochromeImage>* coarseImage, milImage<MonochromeImage>* fineImage)
{
int i, j, k, x, y, i1, j1, scale = 4;
unsigned char **fine, **coarse;
coarse = coarseImage->p_Image2D;
fine = fineImage->p_Image2D;
int debug_count = 0;
k = 0; x = Width/4; y = Height/4;

/*	for(i=0;i<y;i++)
		for(i1=0;i1<scale;i1++)
			for(j=0;j<x;j++)
				for(j1=0;j1<scale;j1++)
				{
					fine[k++] = coarse[i][j];
					if(coarse[i][j]>0)
						debug_count++;
				}
*/
	for(i=0;i<y;i++)
	for(j=0;j<x;j++)
		for(i1=0;i1<scale;i1++)
		for(j1=0;j1<scale;j1++)
			fine[scale*i+i1][scale*j+j1] = MAX_GREY_VALUE*coarse[i][j];
	
}


void CTracker::ExpandColorImage(Image<unsigned char>* coarse_Image, Image<unsigned char>* fine_color_Image)
{
int i, j, k, x, y, i1, j1, i2, j2, scale = 4;
unsigned char **fine;
unsigned char ** p_fgd2D;
k = 0; x = Width/4; y = Height/4;

p_fgd2D = coarse_Image->p_Image2D;
fine = fine_color_Image->p_Image2D;

// initialization to 0

memset(fine[0],255, sizeof(unsigned char)*Width*Height*3);

for(i=0;i<y;i++)
for(j=0;j<x;j++)
if(p_fgd2D[i][j] > 0)
// if(i%2==0 && j%2==0 )
  {
	  for(i1=0;i1<scale;i1++)
		for(j1=0;j1<scale;j1++)
		{ 
			i2 = scale*i+i1;
			j2 = 3*(scale*j+j1);

			fine[i2][j2] = BodyColor.R;
			fine[i2][j2+1] = BodyColor.G;
			fine[i2][j2+2] = BodyColor.B; 

	//		1D implementation available but commented
	//		k = 3*i2 *Width + 3*j2 
	/*		
			i2 = scale*i+i1;
			j2 = scale*j+j1;
			k = 3*(i2*Width + j2);
			fine[0][k] = BodyColor.R;
			fine[0][k+1] = BodyColor.G;
			fine[0][k+2] = BodyColor.B;	*/
	
		} 
  }

}



// Comment added by esc; July 20, 1999
// IsMotion is first initializing motion_flag to 0 when called
// Then, IsMotion is testing the size of the motion to decide 
// if it is really motion
int CTracker::IsMotion(Image<MonochromeImage>* ForegroundImage)
{
//	x = Width/scale;
int i,j,scale=4,x = ForegroundImage->m_GetRows(), y = ForegroundImage->m_GetCols();
unsigned char** tp = ForegroundImage->p_Image2D;
int i1,i2,j1,j2;
//int threshold = int(MOTION_THRESHOLD*(1-2*DeadBand)*(1-2*DeadBand)*x*y);
int threshold = int(MOTION_THRESHOLD*x*y*100/(MIN_DIST*MIN_DIST));
int motion_flag = 0;

i1 = int(DeadBand*y);
i2 = y - i1;
j1 = int(DeadBand*x);
j2 = x - j1;

motion_size_private = 0;

//for(i=i1; i<i2; i++)
//for(j=j1; j<j2; j++)
for(i=0; i<y; i++)
for(j=0; j<x; j++)	
if(tp[i][j])
	motion_size_private+=5;//=tp[i][j];

if(motion_size_private>threshold)
	motion_flag = 1;		

return motion_flag;
}


void CTracker::computePOI(Image<MonochromeImage>* ForegroundImage, int* cPOI)
{
	int i, j, height, width, MedianLocation = 0;
	height = foregroundImage[2]->m_GetCols();
	width  = foregroundImage[2]->m_GetRows();

	for(i=0;i<width;i++)hist_x[i] = 0;
	for(i=0;i<height;i++)hist_y[i] = 0;

//	for(i=SR.y_min; i<= SR.y_max; i++)
//	for(j=SR.x_min; j<= SR.x_max; j++)
	for(i=0; i< height; i++)
	for(j=0; j< width; j++)
		if(foregroundImage[2]->p_Image2D[i][j] > 0)
		{
			MedianLocation++;
			hist_x[j]++;
			hist_y[i]++;
		}

	MedianLocation/=2;
	cPOI[0] = GetMedianFromCountingSort(MedianLocation, hist_x);
	cPOI[1] = GetMedianFromCountingSort(MedianLocation, hist_y);
}

void CTracker::computeROI()
{
//	Given a POI this method computes a Region of Interest (which is a rectangle,
//	centered in the POI. This is possible as it is assumed (at this point) that 
//	the moving object is symetric. Finding ROI is therefore equivalent to find the 
//	width and height of the rectangle.


}


int CTracker::IsMoveCamera(int* cPOI)
{
	int scale=4,x = Width/scale, y = Height/scale;
	int motion_flag, move_camera = 0;
	int i1,i2,j1,j2;
	i1 = int(DeadBand*y);
	i2 = y - i1;
	j1 = int(DeadBand*x);
	j2 = x - j1;

	motion_flag = IsMotion(foregroundImage[2]);
	if(motion_flag == 1)
	{
		computePOI(foregroundImage[2], POI);
		if(cPOI[0]<j1 || cPOI[0]>j2 || cPOI[1]<i1 || cPOI[1]>i2)
			move_camera = 1;
	}
	return move_camera;
}


unsigned char CTracker::IsZoom(double tilt, int* PofI)
{
	unsigned char ret_value = 0;
	int move_camera = 0;

	double	pi[2], f, pan = 0.0, Pw[3] = {0, 4.5, 0.0};
	pi[0] = double(PofI[0]); pi[1] = double(PofI[1]);

	WorldPoint_from_ImagePoint(pi, pan, tilt, Pw);
	tilt = TiltAngle_from_WorldPoint(Pw);
	f = fabs(FocalLengthFromDistance(tilt, Pw[2]));

	if( fabs((f-current_fx)/current_fx) < 0.1)
	{
		move_camera = 1;
		ret_value = 1;
	}
	
	return ret_value;
}



int CTracker::IsMoveCameraTorsoHeadBox(tRectangle* THBox, int motion_flag)
{
	int scale=4,x = Width/scale, y = Height/scale;
	int i1,i2,j1,j2;
	i1 = int(DeadBand*y);
	i2 = y - i1;
	j1 = int(DeadBand*x);
	j2 = x - j1;
	int move_camera_flag = 0;

	if(motion_flag == 1)
	{
		if(THBox->x_min<j1 || THBox->x_max>j2 || THBox->y_min<i1 || THBox->y_max>i2)
		{
			move_camera_flag = 1;
//			computePOI(foregroundImage[2], POI);
		}
	}
	return move_camera_flag;
}


void CTracker::PlotRectangle(tRectangle R, int type)
{
	unsigned char **fine;

	if(IMAGE_TYPE == 0)
		fine = foregroundImage[0]->p_Image2D;
	else
		fine = resultImage_color->p_Image2D;

	int i,x_min, x_max, y_min, y_max, cl = 1,i1,j1,j2;
	x_min = 4*R.x_min;
	y_min = 4*R.y_min;
	x_max = 4*R.x_max;
	y_max = 4*R.y_max;
	//fprintf(pFile,"%4d  %4d  %4d  %4d  \n", R.x_min, R.x_max, R.y_min, R.y_max);

	if(IMAGE_TYPE == 0)
	{
		switch(type){
		case 1:
			for(i=x_min;i<=x_max;i++)
				fine[y_min][i] = fine[y_max][i] = cl;
			for(i=y_min;i<=y_max;i++)
				fine[i][x_min] = fine[i][x_max] = cl;
			break;
		case 2:
			for(i=x_min;i<=x_max;i+=2)
				fine[y_min][i] = fine[y_max][i] = cl;
			for(i=y_min;i<=y_max;i+=2)
				fine[i][x_min] = fine[i][x_max] = cl;
			break;
		}
	}
	else
	{
		for(i=x_min;i<=x_max;i++)
		{
			i1 = 3*i;
			fine[y_min][i1] = fine[y_max][i1++] = cl;
			fine[y_min][i1] = fine[y_max][i1++] = cl;
			fine[y_min][i1] = fine[y_max][i1] = cl;
		}
		for(i=y_min;i<=y_max;i++)
		{
			j1 = 3*x_min; j2 = 3*x_max;
			fine[i][j1++] = fine[i][j2++] = cl;
			fine[i][j1++] = fine[i][j2++] = cl;
			fine[i][j1++] = fine[i][j2++] = cl;
		}
	}

}

void CTracker::PlotRectangle(milImage<unsigned char>* pImage, tRectangle R, int type)
{
	unsigned char **fine = pImage->p_Image2D;

	int i,x_min, x_max, y_min, y_max, cl = 1,i1,j1,j2;
	x_min = R.x_min;
	y_min = R.y_min;
	x_max = R.x_max;
	y_max = R.y_max;
	//fprintf(pFile,"%4d  %4d  %4d  %4d  \n", R.x_min, R.x_max, R.y_min, R.y_max);

	if(IMAGE_TYPE == 0)
	{
		switch(type){
		case 1:
			for(i=x_min;i<=x_max;i++)
				fine[y_min][i] = fine[y_max][i] = cl;
			for(i=y_min;i<=y_max;i++)
				fine[i][x_min] = fine[i][x_max] = cl;
			break;
		case 2:
			for(i=x_min;i<=x_max;i+=2)
				fine[y_min][i] = fine[y_max][i] = cl;
			for(i=y_min;i<=y_max;i+=2)
				fine[i][x_min] = fine[i][x_max] = cl;
			break;
		}
	}
	else
	{
		for(i=x_min;i<=x_max;i++)
		{
			i1 = 3*i;
			fine[y_min][i1] = fine[y_max][i1++] = cl;
			fine[y_min][i1] = fine[y_max][i1++] = cl;
			fine[y_min][i1] = fine[y_max][i1] = cl;
		}
		for(i=y_min;i<=y_max;i++)
		{
			j1 = 3*x_min; j2 = 3*x_max;
			fine[i][j1++] = fine[i][j2++] = cl;
			fine[i][j1++] = fine[i][j2++] = cl;
			fine[i][j1++] = fine[i][j2++] = cl;
		}
	}

}

void CTracker::PlotRectangle(unsigned char **fine, tRectangle R, int type)
{

	int i,x_min, x_max, y_min, y_max, cl = 1,i1,j1,j2;
	x_min = R.x_min;
	y_min = R.y_min;
	x_max = R.x_max;
	y_max = R.y_max;
	//fprintf(pFile,"%4d  %4d  %4d  %4d  \n", R.x_min, R.x_max, R.y_min, R.y_max);

	if(IMAGE_TYPE == 0)
	{
		switch(type){
		case 1:
			for(i=x_min;i<=x_max;i++)
				fine[y_min][i] = fine[y_max][i] = cl;
			for(i=y_min;i<=y_max;i++)
				fine[i][x_min] = fine[i][x_max] = cl;
			break;
		case 2:
			for(i=x_min;i<=x_max;i+=2)
				fine[y_min][i] = fine[y_max][i] = cl;
			for(i=y_min;i<=y_max;i+=2)
				fine[i][x_min] = fine[i][x_max] = cl;
			break;
		}
	}
	else
	{
		for(i=x_min;i<=x_max;i++)
		{
			i1 = 3*i;
			fine[y_min][i1] = fine[y_max][i1++] = cl;
			fine[y_min][i1] = fine[y_max][i1++] = cl;
			fine[y_min][i1] = fine[y_max][i1] = cl;
		}
		for(i=y_min;i<=y_max;i++)
		{
			j1 = 3*x_min; j2 = 3*x_max;
			fine[i][j1++] = fine[i][j2++] = cl;
			fine[i][j1++] = fine[i][j2++] = cl;
			fine[i][j1++] = fine[i][j2++] = cl;
		}
	}

}


void CTracker::computeBBox(float tilt_angle, int* PofI)
{
	double fx, fy, x0, y0, xn, yn, Xc, Yc=6 /* distance from the ceiling to the POI */, Zc;
	double s0, c0, dX, dY, dZ, Z;
	int x[4], y[4], x_min, x_max, y_min, y_max;
	s0 = sin(tilt_angle);
	c0 = cos(tilt_angle);

	fx = fy = current_fy/4.0;
	x0 = pCamPar->plfPrincipalPoint[0]/4.0;
	y0 = pCamPar->plfPrincipalPoint[1]/4.0;
	dX = 1.0;
	dY = 3.0;
	dZ = 0.1;
	//	1 - First compute normalized coordinates of the point of interest
	xn = PofI[0] - x0;
	yn = PofI[1] - y0;

	//	2 - Compute Xc & Zc
	Zc = Yc*(fy*c0 - yn*s0)/(yn*c0 + fy*s0);
	Xc = xn*(Yc*s0 + Zc*c0)/fx;

	//	

	Z = (Yc-dY)*s0 +(Zc-dZ)*c0;
	x[0] = int(fx*(Xc-dX)/Z + x0);	y[0] = int(fy*((Yc-dY)*c0 - (Zc-dZ)*s0)/Z + y0);

	Z = (Yc+dY)*s0 +(Zc-dZ)*c0;
	x[1] = int(fx*(Xc+dX)/Z + x0);	y[1] = int(fy*((Yc+dY)*c0 - (Zc-dZ)*s0)/Z + y0);

	Z = (Yc-dY)*s0 +(Zc+dZ)*c0;
	x[2] = int(fx*(Xc-dX)/Z + x0);	y[2] = int(fy*((Yc-dY)*c0 - (Zc+dZ)*s0)/Z + y0);

	Z = (Yc+dY)*s0 +(Zc+dZ)*c0;
	x[3] = int(fx*(Xc+dX)/Z + x0);	y[3] = int(fy*((Yc+dY)*c0 - (Zc+dZ)*s0)/Z + y0);

	x_min = __min(__min(x[0],x[1]), __min(x[2],x[3]));
	y_min = __min(__min(y[0],y[1]), __min(y[2],y[3]));
	x_max = __max(__max(x[0],x[1]), __max(x[2],x[3]));
	y_max = __max(__max(y[0],y[1]), __max(y[2],y[3]));
	BBox.x_min = __max(0,x_min);
	BBox.y_min = __max(0,y_min);
	BBox.x_max = __min(Width/4-1,x_max);
	BBox.y_max = __min(Height/4-1,y_max);
}

// comment added by esc: July 29, 1999
// The output of this function is a POI
// based on the center of the Torso+head box 
void CTracker::computeTorsoHeadBox(float tilt_angle, Image<MonochromeImage>* ForegroundImage, int motion_flag, tRectangle* THBox, int* PofI)
{
	// Compute Blackboard model TorsoHead Box. dZ = 0;
	int tPOI[2];

	if(motion_flag == 1)
	{
		computePOI(ForegroundImage, tPOI);
		computeBodyBox(foregroundImage[2], &BodyBox);
		HeadTop[0] = tPOI[0];
		HeadTop[1] = BodyBox.y_min;
		double fx, fy, x0, y0, xn, yn, Yt=3.0, Xc, Yc, Zc;
		double s0, c0, dY, dX, Z;
		int x[4], y[4];
		s0 = sin(tilt_angle);
		c0 = cos(tilt_angle);

		fx = fy = current_fy/4.0;
		x0 = pCamPar->plfPrincipalPoint[0]/4.0;
		y0 = pCamPar->plfPrincipalPoint[1]/4.0;
		dX = 0.5;
		dY = 1.5;
	
		//	1 - First compute normalized coordinates of the point of interest
		xn = HeadTop[0] - x0;
		yn = HeadTop[1] - y0;

		//	2 - Compute Xc & Zc
		Zc = Yt*(fy*c0 - yn*s0)/(yn*c0 + fy*s0);
		Xc = xn*(Yt*s0 + Zc*c0)/fx;
		Yc = Yt + 1.5;

		//	

		Z = (Yc-dY)*s0 + Zc*c0;
		x[0] = int(fx*(Xc-dX)/Z + x0);	y[0] = int(fy*((Yc-dY)*c0 - Zc*s0)/Z + y0);

		Z = (Yc+dY)*s0 + Zc*c0;
		x[1] = int(fx*(Xc+dX)/Z + x0);	y[1] = int(fy*((Yc+dY)*c0 - Zc*s0)/Z + y0);

		THBox->x_min = x[0]; // BodyBox.x_min;//
		THBox->y_min = y[0];
		THBox->x_max = x[1]; // BodyBox.x_max;//
		THBox->y_max = y[1];
		PofI[0] = (THBox->x_min + THBox->x_max)/2;
		PofI[1] = (THBox->y_min + THBox->y_max)/2;
	}
}

void CTracker::computeTorsoBox(float tilt_angle, Image<MonochromeImage>* ForegroundImage,tRectangle* TBox, int* tPOI)
{
// Compute Blackboard model TorsoHead Box. dZ = 0;
int motion_flag = IsMotion(foregroundImage[2]);
if(motion_flag == 1)
{
//	computePOI(foregroundImage[2], tPOI);
	computeBodyBox(foregroundImage[2], &BodyBox);
	HeadTop[0] = tPOI[0];
	HeadTop[1] = BodyBox.y_min;
	double fx, fy, x0, y0, xn, yn, Yt=3.0, Xc, Yc, Zc;
	double s0, c0, dY, dX, Z;
	int x[4], y[4];
	s0 = sin(tilt_angle);
	c0 = cos(tilt_angle);

	fx = fy = current_fy/4.0;
	x0 = pCamPar->plfPrincipalPoint[0]/4.0;
	y0 = pCamPar->plfPrincipalPoint[1]/4.0;
	dX = 0.5;
	dY = 1.0;
	
	//	1 - First compute normalized coordinates of the point of interest
	xn = HeadTop[0] - x0;
	yn = HeadTop[1] - y0;

	//	2 - Compute Xc & Zc
	Zc = Yt*(fy*c0 - yn*s0)/(yn*c0 + fy*s0);
	Xc = xn*(Yt*s0 + Zc*c0)/fx;
	Yc = Yt + 2.0;

	//	

	Z = (Yc-dY)*s0 + Zc*c0;
	x[0] = int(fx*(Xc-dX)/Z + x0);	y[0] = int(fy*((Yc-dY)*c0 - Zc*s0)/Z + y0);

	Z = (Yc+dY)*s0 + Zc*c0;
	x[1] = int(fx*(Xc+dX)/Z + x0);	y[1] = int(fy*((Yc+dY)*c0 - Zc*s0)/Z + y0);

	TBox->x_min = x[0]; // BodyBox.x_min;//
	TBox->y_min = y[0];
	TBox->x_max = x[1]; // BodyBox.x_max;//
	TBox->y_max = y[1];
//	fprintf(pFile,"%4d  %4d  %4d  %4d  %4d  %4d  \n", x[0],x[1],y[0],y[1], tPOI[0], tPOI[1]);
}
}


void CTracker::computeTorsoBox(float tilt_angle, tRectangle* TBox, int* HeadTop)
{
	double fx, fy, x0, y0, xn, yn, Yt=3.0, Xc, Yc, Zc;
	double s0, c0, dY, dX, Z;
	int x[4], y[4];
	s0 = sin(tilt_angle);
	c0 = cos(tilt_angle);

	fx = fy = current_fy/4.0;
	x0 = pCamPar->plfPrincipalPoint[0]/4.0;
	y0 = pCamPar->plfPrincipalPoint[1]/4.0;
	dX = 0.5;
	dY = 1.0;
	
	//	1 - First compute normalized coordinates of the point of interest
	xn = HeadTop[0] - x0;
	yn = HeadTop[1] - y0;

	//	2 - Compute Xc & Zc
	Zc = Yt*(fy*c0 - yn*s0)/(yn*c0 + fy*s0);
	Xc = xn*(Yt*s0 + Zc*c0)/fx;
	Yc = Yt + 2.0;

	//	

	Z = (Yc-dY)*s0 + Zc*c0;
	x[0] = int(fx*(Xc-dX)/Z + x0);	y[0] = int(fy*((Yc-dY)*c0 - Zc*s0)/Z + y0);

	Z = (Yc+dY)*s0 + Zc*c0;
	x[1] = int(fx*(Xc+dX)/Z + x0);	y[1] = int(fy*((Yc+dY)*c0 - Zc*s0)/Z + y0);

	TBox->x_min = x[0]; // BodyBox.x_min;//
	TBox->y_min = y[0];
	TBox->x_max = x[1]; // BodyBox.x_max;//
	TBox->y_max = y[1];
//	fprintf(pFile,"%4d  %4d  %4d  %4d  %4d  %4d  \n", x[0],x[1],y[0],y[1], tPOI[0], tPOI[1]);
}


void CTracker::getMotionImprovedOptimalBBox(int dx, int dy, int* PofI)
{
//	BBox is given by (x_min, y_min) and (x_max, y_max)
//	First, we will compute the search area based on BBox and dx, dy
//	This is the intersection of the box (x_min - dx, y_min - dy) - 
//	(x_max + dx, y_max + dy) and the entire image
	
	unsigned char **fine;
	fine = foregroundImage[2]->p_Image2D;

	int box_width, box_height, i, j, i1, i2, j1, j2, k, l, n, m;
	box_width = BBox.x_max - BBox.x_min + 1;
	box_height = BBox.y_max - BBox.y_min + 1;
	i1 = __max(BBox.y_min-dy, 0);
	i2 = __min(BBox.y_max+dy, Height/4) - box_height;
	j1 = __max(BBox.x_min-dx, 0);
	j2 = __min(BBox.x_max+dx, Width/4) - box_width;

	n = i2 - i1 + 1;
	m = j2 - j1 + 1;
	int Ver[100][100];
	
	// Horizontal pass
/*	for(i=0; i<n; i++)
	{
		Hor[i][0] = fine[i1+i][j1];
		for(j=1; j<box_width; j++)
			Hor[i][0]+= fine[i1+i][j1+j];

		for(j=1; j<m; j++)
			Hor[i][j] = Hor[i][j-1] + fine[i1+i][j1+j+box_width-1] - fine[i1+i][j1+j-1];
	}

	// Vertical Pass
	for(j=0; j<m; j++)
	{
		Ver[0][j] = Hor[0][j];
		for(i=1; i<box_height; i++)
			Ver[0][j]+= Hor[i][j];

		for(i=1; i<n; i++)
			Ver[i][j] = Ver[i][j-1] + Hor[i][j+box_height-1] - Hor[i][j-1];
	}
*/
	for(i=0;i<n;i++)
	for(j=0;j<m;j++)
	{
		Ver[i][j] = 0;
		for(k=0; k<box_height; k++)
		for(l=0; l<box_width; l++)
			Ver[i][j]+= fine[i+i1+k][j+j1+l];
	}

	int i_max, j_max, Ver_max = Ver[0][0];
	i_max = j_max = 0;

	for(i=0; i<n; i++)
	for(j=0; j<m; j++)
	if(Ver_max<Ver[i][j])
	{
		i_max = i;
		j_max = j;
	}
	if(Ver_max>10)
	{
		PofI[0] = i1 + i_max;
		PofI[1] = j1 + j_max;
	}
}


void CTracker:: computeBodyBox(Image<MonochromeImage>* ForegroundImage, tRectangle* BB)
{
	unsigned char** p_fgd = foregroundImage[2]->p_Image2D;
	int i, j, x_min, x_max, y_min, y_max, width = Width/4, height = Height/4;
	x_min = y_min = 1000;
	x_max = y_max = -1000;

	for(i=0; i<height; i++)
	for(j=0; j<width; j++)
	if(p_fgd[i][j]>0)
	{
		if(x_min>j)
			x_min = j;
		if(y_min>i)
			y_min = i;
		if(x_max<j)
			x_max = j;
		if(y_max<i)
			y_max = i;
	}
	
	BodyBox.x_min = x_min;
	BodyBox.y_min = y_min;
	BodyBox.x_max = x_max;
	BodyBox.y_max = y_max;

}

void CTracker:: computeHeadTop(Image<MonochromeImage>* ForegroundImage, int*HT)
{
	unsigned char** p_fgd = ForegroundImage->p_Image2D;
	int i, j, x_head, y_head, width = ForegroundImage->m_GetRows(), height = ForegroundImage->m_GetCols();
	y_head = 1000;

	for(i=0; i<height; i++)
	for(j=0; j<width; j++)
	if(p_fgd[i][j]>0)
	{
		if(y_head>i)
		{
			y_head = i;
			x_head = j;
		}
	}
	HT[0] = x_head;
	HT[1] = y_head;
}

void CTracker::ComputePersonColor(Image<MonochromeImage> mask_Image_coarse, tRectangle)
{

}

double CTracker::WorldDistanceBetweenImagePoints(double* p1, double* p2, double tilt)
{
double P1[3] = {0, 4.5, 0}, P2[3] = {0, 4.5, 0};
RelativeWorldPoint_from_ImagePoint(p1, tilt, P1);
RelativeWorldPoint_from_ImagePoint(p2, tilt, P2);
double d = sqrt((P1[0]-P2[0])*(P1[0]-P2[0]) + (P1[2]-P2[2])*(P1[2]-P2[2]));
return d;
}


void CTracker::ComputeMovingPixels(Image<int>* Image1, Image<int>* Image2, Image<MonochromeImage>* ForegroundImage)
{
	int i,j, N=Height*Width/16, Min, Max, Median;
	int MedianLocation = int(N/2);
	int x = Width/4, y=Height/4, **tp;
	int *p_img1, *p_img2, *p_imgdif;
	p_img1 = (int*)Image1->m_GetImage();
	p_img2 = (int*)Image2->m_GetImage();
	p_imgdif = (int*)differenceImage_coarse->m_GetImage();
	unsigned char* p_fgd = ForegroundImage->m_GetImage();

	Min = Max = abs((*p_img2 - *p_img1)/16);
	
	//	Compute Difference image and find it's min and max 
	for(i=0;i<N;i++)
	{
		j = p_img2[i] - p_img1[i];
		j = int(abs(j/16));
		p_imgdif[i] = j;
	
		if(Min>j)
			Min = j;
		else
			if(Max<j)
				Max = j;
	}


	//	Compute histogram of difference image
	//  comment added by esc July 29, 1999 
	//  the parameter threshold is used to separate pixels belonging to moving
	//  from others 
	GenerateHistogramFromImage(differenceImage_coarse, 512, histogram, Min, Max);
	Median = GetMedianFromCountingSort(MedianLocation, histogram);
	float sigma =1.4826f*Median;
	float threshold = 2.5f*sigma;
	if(threshold < DIFERENCE_THRESHOLD)
		threshold = DIFERENCE_THRESHOLD;



	// comment added by esc July 29, 1999 
	// differenceImage_coarse->p_Image2D  is binarized to be used for 
	// noise filtering and by the label() function below 
	for(i=N-1;i>=0;i--)
	{
		if(abs(p_imgdif[i])>threshold)
			(p_imgdif[i]) = 1;
		else
			(p_imgdif[i]) = 0;
	}

//	NoiseFiltering(differenceImage_coarse, foregroundImage[2]);
	//	NOISE FILTERING FUNCTION: START DEBUG 
	tp = differenceImage_coarse->p_Image2D;
	int count;
	for(i=1; i<y-1; i++)
	for(j=1; j<x-1; j++)
	{
		if(tp[i][j]>0)
		{
			count = tp[i-1][j-1] + tp[i-1][j] + tp[i-1][j+1] + tp[i][j-1] + tp[i][j+1] + tp[i+1][j-1] + tp[i+1][j] + tp[i+1][j+1];
			if(count>2)
				ForegroundImage->p_Image2D[i][j] = 1;
			else
				ForegroundImage->p_Image2D[i][j] = 0;
		}
		else
			ForegroundImage->p_Image2D[i][j] = 0;
//		ForegroundImage->p_Image2D[i][j] = tp[i][j];
	}
	//	ForegroundImage->p_Image[i] = (differenceImage_coarse->p_Image[i]);
	//	END FILTERING FUNCTION - END DEBUG
}


dMatrix CTracker::AllignImage(CPTZRadians ptz_old, CPTZRadians ptz_new, CORNER_LIST* pCL0, CORNER_LIST* pCL1)
{
	int i, j, n, matches[1000][4];
	dMatrix M(3,3), Q(3,3), R1(3,3), R(3,3), iQ(3,3), Mr(3,3);
	
	float** Mm = alloc2Dfloat(3,3);
	float** iM = alloc2Dfloat(3,3);
	double fx, fy, xc, yc;
	int inliers[1000];
	CORNER_M* pCm;
	double tilt, dpan, dtilt;

	fx = Q.a[0][0] = 350.0;//pCamPar->lfFx;
	fy = Q.a[1][1] = 350.0;//pCamPar->lfFy;
	xc = Q.a[0][2] = pCamPar->plfPrincipalPoint[0];
	yc = Q.a[1][2] = pCamPar->plfPrincipalPoint[1];
	Q.a[2][2] = 1.0;

	iQ.a[0][0] = 1/fx;
	iQ.a[1][1] = 1/fy;
	iQ.a[0][2] = -xc/fx;
	iQ.a[1][2] = -yc/fy;
	iQ.a[2][2] = 1.0;

	tilt  = ptz_old.m_yTicks;
	dtilt = ptz_new.m_yTicks - tilt;
	dpan  = ptz_new.m_xTicks - ptz_old.m_xTicks;

	R = tpt2rot(tilt, dpan, dtilt);
	M = mult_matrix(Q, mult_matrix(R, iQ));

	if (((CButton *)biteDialog.GetDlgItem(IDC_REG_ON))->GetCheck()!=1)	
		return M;

	for(i=0;i<3;i++)for(j=0;j<3;j++)Mm[i][j] = float(M.a[i][j]);
	inv3x3(Mm, iM);
	int r = 60;
//	MatchRotCorners(pCL0, pCL1, Mm, Width, Height, 5);
	MatchCorners(pCL0, pCL1, Width, Height, r);

	n = 0;
	for(i=0;i<pCL0->N;i++)
	{
	  pCm = &(pCL0->Corners[i]);
	  if(pCm->index2>0)
	  {
		matches[n][0] = pCm->x;
		matches[n][1] = pCm->y;
		matches[n][2] = pCm->x + pCm->dx;
		matches[n][3] = pCm->y + pCm->dy;
		n++;
	  }
	}

	if(n>10)
		AffineLMedS(matches, n, Mr, inliers, 3, 0.45f, 0.999f);
	else
		return M;

	return Mr;
}

dMatrix CTracker::AllignImage(CORNER_LIST* pCL0, CORNER_LIST* pCL1, int reg, int scale)
{
	int i, j, n, matches[1000][4];
	dMatrix M(3,3), Mr(3,3);
	M.set2identity();

	int inliers[1000];
	CORNER_M* pCm;

	MatchCorners(pCL0, pCL1, Width, Height, 90/scale);

	n = 0;
	for(i=0;i<pCL0->N;i++)
	{
	  pCm = &(pCL0->Corners[i]);
	  if(pCm->index2>0)
	  {
		matches[n][0] = pCm->x;
		matches[n][1] = pCm->y;
		matches[n][2] = pCm->x + pCm->dx;
		matches[n][3] = pCm->y + pCm->dy;
		n++;
	  }
	}

	if(n>4)
	{
		if(reg==0)
			TranslationLMedS(matches, n, Mr, inliers);
		else if(reg==1)
			Affine3RANSAC(matches, n, Mr, inliers, 2, 0.66f, 0.999f, 16.0f/sqrt(scale));
		else
			HomographyRANSAC(matches, n, Mr, inliers, 4, 0.4f, 0.999f, 16.0f/sqrt(scale));
	}
	else
		Mr.set2identity();

	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen("d:\\users\\kori\\a.txt", "at");
		fprintf(pFilem, "%d \n", n);

		for(i=0;i<n;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);

		for(i=0;i<3;i++)
			fprintf(pFilem, "%7.4f  %7.4f  %7.4f \n", Mr.a[i][0], Mr.a[i][1], Mr.a[i][2]);
		fclose(pFilem);
	}

	return Mr;
}

dMatrix CTracker::AllignImageAffine(CORNER_LIST* pCL0, CORNER_LIST* pCL1, dMatrix* A, int reg, int scale, int ret_matrix)
{
	int i, j, n, matches[1000][4];
	dMatrix M(3,3), Mr(3,3);
	M.set2identity();

	int inliers[1000];
	CORNER_M* pCm;

	n = MatchCornersAffine(pCL0, pCL1, A, matches, 15);

	if(n>4)
	{
		if(reg==0)
			TranslationLMedS(matches, n, Mr, inliers);
		else if(reg==1)
			Affine3RANSAC(matches, n, Mr, inliers, 2, 0.66f, 0.999f, 16.0f/sqrt(scale), ret_matrix);
		else
			HomographyRANSAC(matches, n, Mr, inliers, 4, 0.4f, 0.999f, 16.0f/sqrt(scale));
	}
	else
		Mr.set2identity();

	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen("d:\\users\\kori\\a.txt", "at");
		fprintf(pFilem, "%d \n", n);

		for(i=0;i<n;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);

		for(i=0;i<3;i++)
			fprintf(pFilem, "%7.4f  %7.4f  %7.4f \n", Mr.a[i][0], Mr.a[i][1], Mr.a[i][2]);
		fclose(pFilem);
	}

	return Mr;
}

int CTracker::startContinuosMCTracking(CPTZRadians ptz, int* cPOI)
{
	milImage<MonochromeImage>	*tempImage_fine;
	milImage<unsigned char>		*tempImage_coarse8U, *tempImage_half;
	tRectangle					PlotBox;
	unsigned char				*p_src, *p_dst;
	int							move_camera_flag = 0, motion_flag = 0;
	int							Ndiv3 = 320*240, N= Ndiv3 *3 ,i = 100,j;
	float						tilt_angle = ptz.m_yTicks;
	int							thr[2] = {0, 300}, scale = 2;
	CORNER_LIST					*tempCL;
	dMatrix						Mr(3,3), Ms(3,3);
	float**						Mt=alloc2Dfloat(3,3);
	float**						iM=alloc2Dfloat(3,3);
	IplImage			*IplFirstFull, *IplSecondFull, *IplFirstHalf, *IplSecondHalf,*IplFirstCoarse, *IplSecondCoarse;
  
	IplFirstFull = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width, Height, NULL, NULL, NULL, NULL);                  
	IplSecondFull = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width, Height, NULL, NULL, NULL, NULL);                 
	IplFirstHalf = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/2, Height/2, NULL, NULL, NULL, NULL);                  
	IplSecondHalf = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/2, Height/2, NULL, NULL, NULL, NULL);                 
	IplFirstCoarse = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/4, Height/4, NULL, NULL, NULL, NULL);                  
	IplSecondCoarse = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/4, Height/4, NULL, NULL, NULL, NULL);                 

	IplFirstFull->imageData = (char *)firstImage[0]->p_Image2D[0];
	IplSecondFull->imageData = (char *)secondImage[0]->p_Image2D[0];
	IplFirstHalf->imageData = (char *)firstImage[1]->p_Image2D[0];
	IplSecondHalf->imageData = (char *)secondImage[1]->p_Image2D[0];
	IplFirstCoarse->imageData = (char *)firstImage[2]->p_Image2D[0];
	IplSecondCoarse->imageData = (char *)secondImage[2]->p_Image2D[0];


	if(nFirstFlag)
	{
		if (BMPflag==false) 
		GrabImage_Color(firstImage_color, firstImage[0]);
		else 
		ReadBMPImage(firstImage_color, firstImage[0]);
		subsampleImage(firstImage[0], firstImage[1], 2);
		subsampleImage(firstImage[1], firstImage[2], 2);

		if (BMPflag==false) 
		GrabImage_Color(secondImage_color, secondImage[0]);
		else 
		ReadBMPImage(secondImage_color, secondImage[0]);

		subsampleImage(secondImage[0], secondImage[1]);
		subsampleImage(secondImage[1], secondImage[2]);

		nFirstFlag=false;

		nTargetLabel =
			TrackMCSegment(firstImage_coarse, secondImage_coarse, foregroundImage[2], tilt_angle, &Torso);

		pCCorner->DetectCorners(secondImage[2]->p_Image2D, thr);
		CreateCornerList(pCCorner, pCL0);
		pCCorner->DetectCorners(secondImage[1]->p_Image2D, thr);
		CreateCornerList(pCCorner, pCL2);

		tempImage_fine = firstImage[0];
		tempImage_half = firstImage[1];
		tempImage_coarse8U = firstImage[2];
		firstImage[0] = secondImage[0];
		firstImage[1] = secondImage[1];
		firstImage[2] = secondImage[2];
		secondImage[0] = tempImage_fine;
		secondImage[1] = tempImage_half;
		secondImage[2] = tempImage_coarse8U;
	}
	else
	{
		if (BMPflag==false) 
			GrabImage_Color(secondImage_color, secondImage[0]);
		else 
			ReadBMPImage(secondImage_color, secondImage[0]);

		pCCorner->DetectCorners(secondImage[2]->p_Image2D, thr);
		CreateCornerList(pCCorner, pCL1);
		pCCorner->DetectCorners(secondImage[1]->p_Image2D, thr);
		CreateCornerList(pCCorner, pCL3);

		if(ptz.m_xTicks == ptz_old.m_xTicks && ptz.m_yTicks == ptz.m_yTicks && ptz.m_zTicks == ptz_old.m_zTicks)
			Mr.set2identity();
		else
		{
			Mr = AllignImage(pCL1, pCL0,1, 4);
/*			Mr.a[0][2]*=scale;
			Mr.a[1][2]*=scale;
			Mr.a[2][0]/=scale;
			Mr.a[2][1]/=scale;
			pCCorner->DetectCorners(secondImage[1]->p_Image2D, thr);
			CreateCornerList(pCCorner, pCL3);
			Mr = AllignImageAffine(pCL3, pCL2, &Mr, 1, scale);
			Mr.a[0][2]*=scale;
			Mr.a[1][2]*=scale;
			Mr.a[2][0]/=scale;
			Mr.a[2][1]/=scale;
*/		}

/*		background_update(bgdImage[0]->p_Image2D, firstImage[0]->p_Image2D, secondImage[0]->p_Image2D, Mr, Width, Height);
		Ms = update_homography(Mr, 4.0f);// This needs to be corrected
		subsampleImage(secondImage[0], secondImage_coarse);
		background_update(bgdImage_coarse->p_Image2D, firstImage_coarse->p_Image2D, secondImage_coarse->p_Image2D, Ms, Width/4, Height/4);
*/		
		iplBackgroundUpdate(bgdImage[2]->p_Image2D[0], firstImage[2]->p_Image2D[0], secondImage[2]->p_Image2D[0], Mr.inverse(), Width/4, Height/4);
		CopyRectangle(&Torso, &PlotBox);
		Mr.a[0][2]*=4;
		Mr.a[1][2]*=4;
		Mr.a[2][0]/=4;
		Mr.a[2][1]/=4;
		Torso_update(&Torso, Mr.inverse());

		if (((CButton *)biteDialog.GetDlgItem(IDC_SAVE))->GetCheck()==1)	
		{
			FILE* pFileImage = fopen("D:\\users\\kori\\Images.raw", "wb");
			fwrite(firstImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);
			fwrite(secondImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);
			fclose(pFileImage);
		}

		nTargetLabel =
			TrackMCSegment(bgdImage_coarse, secondImage_coarse, foregroundImage[2], tilt_angle, &Torso);
// Debuging part
		for(i=0;i<Height;i++)
		for(j=0;j<Width;j++)
			foregroundImage[0]->p_Image2D[i][j] = abs(secondImage[0]->p_Image2D[i][j] - bgdImage[0]->p_Image2D[i][j]);
// end debugging part

		tempCL = pCL0;
		pCL0 = pCL1;
		pCL1 = tempCL;

		tempCL = pCL2;
		pCL2 = pCL3;
		pCL3 = tempCL;

		tempImage_fine = firstImage[0];
		tempImage_half = firstImage[1];
		tempImage_coarse8U = firstImage[2];
		firstImage[0] = secondImage[0];
		firstImage[1] = secondImage[1];
		firstImage[2] = secondImage[2];
		secondImage[0] = tempImage_fine;
		secondImage[1] = tempImage_half;
		secondImage[2] = tempImage_coarse8U;
	}

	SetRectangle(&BBox, Torso.x_min/4, Torso.y_min/4, Torso.x_max/4, Torso.y_max/4);
	p_src = secondImage_color->m_GetImage();
//	ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);

	if(nTargetLabel >= 0) // a moving target was found
	{
		motion_flag = 1;
		POI[0] = (BBox.x_min + BBox.x_max)/2;
		POI[1] = (BBox.y_min + BBox.y_max)/2;
		if(POI[0]<39 || POI[0]>41 || POI[1]<29 || POI[1]>39) move_camera_flag = 1;
	}
	else // either no target or a static target was found
	{
		if(nTargetLabel == 0) // a stationary target was found
		{
			// dont expand motionframe here, so you can still see old target
			motion_flag = 1;
			move_camera_flag = 0;
			POI[0] = (BBox.x_min + BBox.x_max)/2;
			POI[1] = (BBox.y_min + BBox.y_max)/2;
		}
		else // no target was found
		{
			motion_flag = 0;
			move_camera_flag = 0;
		}
	}

		// ALWAYS switch frames dml 3/25/00
		// if you don't you can get two difference regions
		// of the same person (one from the old ref image and one from the new)

	int image_display = 0;
	if (biteDialog.EnableMILWin) image_display = 0; // dml 6/15/00
	
	p_dst = resultImage_color->m_GetImage();
	if(image_display == 0)
	{
		p_src = foregroundImage[0]->m_GetImage();
		
		for(i=0;i<Ndiv3;i++) // display an inverted b/w motion picture
		{
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src++;
		}
	}
	else
	{
		memcpy( p_dst, p_src, N); // display the original image
	}
	if (motion_flag)
	{
		CopyRectangle(&SR, &PlotBox);

			CopyRectangle(&Torso, &PlotBox);
		
			PlotRectangle(resultImage_color, PlotBox, 1); // dml 6/15
			//MgraRect(M_DEFAULT,MIL_params.MilImageDispColor, 4*PlotBox.x_min,4*PlotBox.y_min, 4*PlotBox.x_max,4*PlotBox.y_max);

			//MgraColor(M_DEFAULT,M_RGB888(0,250, 0)); 
			CopyRectangle(&TorsoHead, &PlotBox, 4);
			
			PlotRectangle(resultImage_color, PlotBox, 1); // dml 6/15
			//MgraRect(M_DEFAULT,MIL_params.MilImageDispColor, 4*PlotBox.x_min,4*PlotBox.y_min, 4*PlotBox.x_max,4*PlotBox.y_max);
//		#endif
	}

	DisplayBMP(resultImage_color->m_GetImage()); // dml 6/15/00
	return move_camera_flag;
}

int CTracker::startContinuosMCTracking(int* cPOI)
{
	milImage<MonochromeImage>	*tempImage_fine;
	milImage<unsigned char>		*tempImage_coarse8U, *tempImage_half;
	tRectangle					PlotBox, SR;
	unsigned char				*p_src, *p_dst;
	int							move_camera_flag = 0, motion_flag = 0;
	int							Ndiv3 = 320*240, N= Ndiv3 *3 ,i = 100,j;
	int							thr[2] = {0, 300}, scale = 2;
	CORNER_LIST					*tempCL;
	dMatrix						Mr(3,3), Ms(3,3);
	float**						Mt=alloc2Dfloat(3,3);
	float**						iM=alloc2Dfloat(3,3);
	IplImage			*IplFirstFull, *IplSecondFull, *IplFirstHalf, *IplSecondHalf,*IplFirstCoarse, *IplSecondCoarse,
						*IplForegroundFull, *IplForegroundHalf, *IplForegroundCoarse;
  
	IplFirstFull = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width, Height, NULL, NULL, NULL, NULL);                  
	IplSecondFull = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width, Height, NULL, NULL, NULL, NULL);                 
	IplFirstHalf = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/2, Height/2, NULL, NULL, NULL, NULL);                  
	IplSecondHalf = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/2, Height/2, NULL, NULL, NULL, NULL);                 
	IplFirstCoarse = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/4, Height/4, NULL, NULL, NULL, NULL);                  
	IplSecondCoarse = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/4, Height/4, NULL, NULL, NULL, NULL);                 

	IplForegroundFull = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width, Height, NULL, NULL, NULL, NULL);                  
	IplForegroundHalf = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/2, Height/2, NULL, NULL, NULL, NULL);                 
	IplForegroundCoarse = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/4, Height/4, NULL, NULL, NULL, NULL);                  

	IplFirstFull->imageData    = (char *)firstImage[0]->p_Image2D[0];
	IplSecondFull->imageData   = (char *)secondImage[0]->p_Image2D[0];
	IplFirstHalf->imageData    = (char *)firstImage[1]->p_Image2D[0];
	IplSecondHalf->imageData   = (char *)secondImage[1]->p_Image2D[0];
	IplFirstCoarse->imageData  = (char *)firstImage[2]->p_Image2D[0];
	IplSecondCoarse->imageData = (char *)secondImage[2]->p_Image2D[0];
	
	IplForegroundFull->imageData   = (char *)foregroundImage[0]->p_Image2D[0];
	IplForegroundHalf->imageData   = (char *)firstImage[1]->p_Image2D[0];
	IplForegroundCoarse->imageData = (char *)foregroundImage[2]->p_Image2D[0];


	if(nFirstFlag)
	{
		if (BMPflag==false) 
		GrabImage_Color(firstImage_color, firstImage[0]);
		else 
		ReadBMPImage(firstImage_color, firstImage[0]);
		subsampleImage(firstImage[0], firstImage[1], 2);
		subsampleImage(firstImage[1], firstImage[2], 2);

		if (BMPflag==false) 
		GrabImage_Color(secondImage_color, secondImage[0]);
		else 
		ReadBMPImage(secondImage_color, secondImage[0]);

		subsampleImage(secondImage[0], secondImage[1], 2);
		subsampleImage(secondImage[1], secondImage[2], 2);

		nFirstFlag=false;

		nTargetLabel =
			TrackMCSegment(firstImage[2], secondImage[2], foregroundImage[2], 0, &Torso);

		pCCornerCoarse->DetectCorners(secondImage[2]->p_Image2D, thr);
		CreateCornerList(pCCornerCoarse, pCL0);
		pCCornerHalf->DetectCorners(secondImage[1]->p_Image2D, thr);
		CreateCornerList(pCCornerHalf, pCL2);

		tempImage_fine = firstImage[0];
		tempImage_half = firstImage[1];
		tempImage_coarse8U = firstImage[2];
		firstImage[0] = secondImage[0];
		firstImage[1] = secondImage[1];
		firstImage[2] = secondImage[2];
		secondImage[0] = tempImage_fine;
		secondImage[1] = tempImage_half;
		secondImage[2] = tempImage_coarse8U;
	}
	else
	{
		if (BMPflag==false) 
			GrabImage_Color(secondImage_color, secondImage[0]);
		else 
			ReadBMPImage(secondImage_color, secondImage[0]);

		subsampleImage(secondImage[0], secondImage[1], 2);
		subsampleImage(secondImage[1], secondImage[2], 2);

		pCCornerCoarse->DetectCorners(secondImage[2]->p_Image2D, thr);
		CreateCornerList(pCCornerCoarse, pCL1);

		Mr = AllignImage(pCL1, pCL0,1, 4);
		Mr.a[0][2]*=scale;
		Mr.a[1][2]*=scale;
		Mr.a[2][0]/=scale;
		Mr.a[2][1]/=scale;
		pCCornerHalf->DetectCorners(secondImage[1]->p_Image2D, thr);
		CreateCornerList(pCCornerHalf, pCL3);
		Mr = AllignImageAffine(pCL3, pCL2, &Mr, 1, scale, 1);

//		Scale Mr, so that it corresponds to the low resolution homography
		Mr.a[0][2]/=scale;
		Mr.a[1][2]/=scale;
		Mr.a[2][0]*=scale;
		Mr.a[2][1]*=scale;
		iplBackgroundUpdate(bgdImage[2]->p_Image2D[0], firstImage[2]->p_Image2D[0], secondImage[2]->p_Image2D[0], Mr.inverse(), Width/4, Height/4);
		CopyRectangle(&Torso, &PlotBox);

//		Scale Mr back
		scale = 4;
		Mr.a[0][2]*=scale;
		Mr.a[1][2]*=scale;
		Mr.a[2][0]/=scale;
		Mr.a[2][1]/=scale;

		Torso_update(&Torso, Mr.inverse());
		SetRectangle(&SR, 10, 10, Width-10, Height-10);

		double vreme;
		MappTimer(M_TIMER_RESET, &vreme);
		nTargetLabel =
			TrackMCSegment(firstImage[0], secondImage[0], bgdImage[0], foregroundImage[0], Mr, &Torso, &SR);
		
		pCand->Update( (Torso.x_min+Torso.x_max)/2, (Torso.y_min+Torso.y_max)/2, Mr.inverse());
//			TrackMCSegment(bgdImage[2], secondImage[2], foregroundImage[2], 0, &Torso);
		MappTimer(M_TIMER_READ, &vreme);
		TRACE("vreme=%f \n",vreme);
// Debuging part
/*		int pw[3] = {1, 2, 4};
		i=0;
		scale = pw[i];
		Mr.a[0][2]/=scale;
		Mr.a[1][2]/=scale;
		Mr.a[2][0]*=scale;
		Mr.a[2][1]*=scale;
		iplBackgroundUpdate(bgdImage[i]->p_Image2D[0], firstImage[i]->p_Image2D[0], secondImage[i]->p_Image2D[0], Mr.inverse(), Width/scale, Height/scale);
		brImageDifference(bgdImage[i], secondImage[i], foregroundImage[i], 50);
		brBlur(foregroundImage[i], foregroundImage[i], &Torso, 40);
*/
//		brSubtract(bgdImage[0], secondImage[0], foregroundImage[0]);
		static fl = 1;

		if (((CButton *)biteDialog.GetDlgItem(IDC_SAVE))->GetCheck()==1 && fl == 1)	
		{
			FILE* pFileImage = fopen("D:\\users\\kori\\Images.raw", "wb");
//			fwrite(firstImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);
//			fwrite(secondImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);
//			fwrite(bgdImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);
			fwrite(foregroundImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);
			fclose(pFileImage);
			fl = 0;
		}

// end debugging part

		tempCL = pCL0;
		pCL0 = pCL1;
		pCL1 = tempCL;

		tempCL = pCL2;
		pCL2 = pCL3;
		pCL3 = tempCL;

		tempImage_fine = firstImage[0];
		tempImage_half = firstImage[1];
		tempImage_coarse8U = firstImage[2];
		firstImage[0] = secondImage[0];
		firstImage[1] = secondImage[1];
		firstImage[2] = secondImage[2];
		secondImage[0] = tempImage_fine;
		secondImage[1] = tempImage_half;
		secondImage[2] = tempImage_coarse8U;
	}

	SetRectangle(&BBox, Torso.x_min/4, Torso.y_min/4, Torso.x_max/4, Torso.y_max/4);
	p_src = secondImage_color->m_GetImage();	
//	iplResize(IplForegroundCoarse, IplForegroundFull, 4, 1, 4, 1, IPL_INTER_NN);
//	ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);

	if(nTargetLabel >= 0) // a moving target was found
	{
		motion_flag = 1;
		POI[0] = (BBox.x_min + BBox.x_max)/2;
		POI[1] = (BBox.y_min + BBox.y_max)/2;
		move_camera_flag = 1;
	}
	else // no target was found
	{
		motion_flag = 0;
		move_camera_flag = 1;
	}

	// ALWAYS switch frames dml 3/25/00
	// if you don't you can get two difference regions
	// of the same person (one from the old ref image and one from the new)

	int image_display = 1;
//	if (biteDialog.EnableMILWin) image_display = 0; // dml 6/15/00
	if (((CButton *)biteDialog.GetDlgItem(IDC_MOTION_DISPLAY))->GetCheck()==0)	
		image_display = 1;
	else
		image_display = 0;


	
	p_dst = resultImage_color->m_GetImage();
	if(image_display == 0)
	{
		p_src = foregroundImage[0]->m_GetImage();
		
		for(i=0;i<Ndiv3;i++) // display an inverted b/w motion picture
		{
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src++;
		}
	}
	else
	{
		memcpy( p_dst, p_src, N); // display the original image
	}

	if (motion_flag)
	{
		CopyRectangle(&SR, &PlotBox);

			CopyRectangle(&Torso, &PlotBox);
		
			PlotRectangle(resultImage_color, PlotBox, 1); // dml 6/15
			//MgraRect(M_DEFAULT,MIL_params.MilImageDispColor, 4*PlotBox.x_min,4*PlotBox.y_min, 4*PlotBox.x_max,4*PlotBox.y_max);

			//MgraColor(M_DEFAULT,M_RGB888(0,250, 0)); 
			CopyRectangle(&TorsoHead, &PlotBox, 4);
			
			PlotRectangle(resultImage_color, PlotBox, 1); // dml 6/15
			//MgraRect(M_DEFAULT,MIL_params.MilImageDispColor, 4*PlotBox.x_min,4*PlotBox.y_min, 4*PlotBox.x_max,4*PlotBox.y_max);
//		#endif
	}

	DisplayBMP(resultImage_color->m_GetImage()); // dml 6/15/00
	return move_camera_flag;
}


void background_update(unsigned char** bgd, unsigned char** im1, unsigned char** im2, dMatrix H, int x, int y)
{
	int i, j, i1, j1;
	float x1, y1, M[3][3];
	for(i=0;i<3;i++)for(j=0;j<3;j++)M[i][j] = float(H.a[i][j]);

	for(i=0;i<y;i++)
	for(j=0;j<x;j++)
	{
		bilinear_mapping(M, j, i, &x1, &y1);
		i1 = int(y1+0.5f);
		j1 = int(x1+0.5f);
		if(i1<0 || i1>=y || j1<0 || j1>=x)
			bgd[i][j] = im2[i][j];
		else
			bgd[i][j] = im1[i1][j1];
	}
}

void iplBackgroundUpdate(unsigned char* bgd, unsigned char* im1, unsigned char* im2, dMatrix H, int x, int y)
{
	IplImage *IplFirst, *IplBGD;
  
	IplFirst = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                  
	IplBGD = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                 

	IplFirst->imageData = (char *)im1;
	IplBGD->imageData = (char *)bgd;

	int i,j;
	double coeffs[3][3];
	for(i=0;i<3;i++)for(j=0;j<3;j++)coeffs[i][j] = H.a[i][j];

	memcpy( bgd, im2, x*y); 

	iplWarpPerspective(IplFirst, IplBGD, coeffs, 0, IPL_INTER_NN);

}


void background_update(int** bgd, int** im1, int** im2, dMatrix H, int x, int y)
{
	int i, j, i1, j1;
	float x1, y1,M[3][3];
	for(i=0;i<3;i++)for(j=0;j<3;j++)M[i][j] = float(H.a[i][j]);

	for(i=0;i<y;i++)
	for(j=0;j<x;j++)
	{
		bilinear_mapping(M, j, i, &x1, &y1);
		i1 = int(y1+0.5f);
		j1 = int(x1+0.5f);
		if(i1<0 || i1>=y || j1<0 || j1>=x)
			bgd[i][j] = im2[i][j];
		else
			bgd[i][j] = im1[i1][j1];
	}
}

void CTracker::Torso_update(tRectangle* pTorso, float** M)
{
	int i, j, i1, j1;
	int x1, y1, dx, dy;
	dx = pTorso->x_max-pTorso->x_min;
	dy = pTorso->y_max-pTorso->y_min;
	float xc, yc;

	bilinear_mapping(M, (pTorso->x_min+pTorso->x_max)/2.0f, (pTorso->y_min+pTorso->y_max)/2.0f, &xc, &yc);
	x1 = int(xc - dx/2.0f);
	y1 = int(yc - dy/2.0f);

	SetRectangle(pTorso, x1, y1, x1+dx, y1+dy);
}

void CTracker::Torso_update(tRectangle* pTorso, dMatrix M)
{
	int i, j, i1, j1;
	int x1, y1, dx, dy;
	dx = pTorso->x_max-pTorso->x_min;
	dy = pTorso->y_max-pTorso->y_min;
	float xc, yc;

	bilinear_mapping(M.a, (pTorso->x_min+pTorso->x_max)/2.0f, (pTorso->y_min+pTorso->y_max)/2.0f, &xc, &yc);
	x1 = int(xc - dx/2.0f);
	y1 = int(yc - dy/2.0f);

	SetRectangle(pTorso, x1, y1, x1+dx, y1+dy);
}

dMatrix update_homography(dMatrix A, float s)
{
	dMatrix B(3,3);

	for(int i=0;i<3;i++)for(int j=0;j<3;j++)B.a[i][j] = A.a[i][j];

	B.a[0][2]/= s;
	B.a[1][2]/= s;
	B.a[2][0]*= s;
	B.a[2][1]*= s;

	return B;
}

void CTracker::TestRegistration(CPTZRadians ptz1, CPTZRadians ptz2)
{
	unsigned char	*p_src, *p_dst;
	unsigned char   *pci, ** corner_image = alloc2Duchar(Width/4, Height/4);
	int				i, j, scale=2;
	int				thr[2] = {0, 200};
	dMatrix			Mr(3,3), Ms(3,3);
	float**			Mt = alloc2Dfloat(3,3);
	float**			iMt = alloc2Dfloat(3,3);
	int				Ndiv3 = 320*240, N= Ndiv3*3;
	FILE* pFileImage = fopen("D:\\users\\kori\\Images.raw", "wb");
   	Image<unsigned char>		*bgdImage_s;
   	Image<unsigned char>		*firstImage_s;
   	Image<unsigned char>		*secondImage_s;
   	Image<unsigned char>		*foregroundImage_s;
	CCorner						*pCCorner_s;
	if(scale == 2)
	{
		bgdImage_s = bgdImage[1];
		firstImage_s = firstImage[1];
		secondImage_s = secondImage[1];
		foregroundImage_s = foregroundImage[1];
		pCCorner_s = pCCornerHalf;
	}
	else if(scale == 4)
	{
		bgdImage_s = bgdImage[2];
		firstImage_s = firstImage[2];
		secondImage_s = secondImage[2];
		foregroundImage_s = foregroundImage[2];
		pCCorner_s = pCCornerCoarse;
	}



	CvSize sizeBlock, sizeShift, sizeRange;

  /* Options for the optical flow algorithm */
  sizeBlock.width = 3;
  sizeBlock.height = 3;
  sizeShift.width = 1;
  sizeShift.height = 1;
  sizeRange.width = 25;
  sizeRange.height = 25;
  int velocity_image_width = (int)(ceil((double)Width / 12.0));
  int velocity_image_height = (int)(ceil((double)Height / 12.0 ));
  int of_size = velocity_image_width*velocity_image_height;
  float *VelocityX = new float[];
  float *VelocityY = new float[of_size];

	IplImage *IplFirst, *IplSecond, *IplVelocityX, *IplVelocityY;
  
/*	IplFirst = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/4, Height/4, NULL, NULL, NULL, NULL);                  
	IplSecond = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         Width/4, Height/4, NULL, NULL, NULL, NULL);                  
	IplVelocityX = iplCreateImageHeader(1, 0, IPL_DEPTH_32F, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         velocity_image_width, velocity_image_height, NULL, NULL, NULL, NULL);                  
	IplVelocityY = iplCreateImageHeader(1, 0, IPL_DEPTH_32F, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         velocity_image_width, velocity_image_height, NULL, NULL, NULL, NULL);                  
	IplVelocityX->imageData = (char *)VelocityX;
	IplVelocityY->imageData = (char *)VelocityY;
*/

	pCPTZController->setCameraPositionRadians(&ptz1);
	Sleep(10);
	GrabImage_Color(firstImage_color, firstImage[0]);
	subsampleImage(firstImage[0], firstImage_s, scale);
	subsampleImage(firstImage_s, firstImage[2], scale);

//	IplFirst->imageData = (char*)firstImage[2]->p_Image2D[0];

	pCCornerCoarse->DetectCorners(firstImage[2]->p_Image2D, thr);
	CreateCornerList(pCCornerCoarse, &CL0);
/*	pci = corner_image[0];
	memcpy(pci, firstImage_s->p_Image2D[0], Ndiv3/(scale*scale));
	pCCorner_s->CreateCornerImage(firstImage_s->p_Image2D, corner_image);
	fwrite(corner_image[0],sizeof(unsigned char), 240*320/(scale*scale), pFileImage);
*/
double vreme, vr1, vr2, vr3;

	pCPTZController->setCameraPositionRadians(&ptz2);
	Sleep(10);
	MappTimer(M_TIMER_RESET, &vr1);
	GrabImage_Color(secondImage_color, secondImage[0]);
	MappTimer(M_TIMER_READ, &vr1);

	MappTimer(M_TIMER_RESET, &vr2);
	subsampleImage(secondImage[0], secondImage_s, scale);
	MappTimer(M_TIMER_READ, &vr2);

	MappTimer(M_TIMER_RESET, &vr3);
	subsampleImage(secondImage_s, secondImage[2], scale);
	MappTimer(M_TIMER_READ, &vr3);

	fprintf(pFileVreme, "%f  %f  %f ", vr1, vr2, vr3);

//	IplSecond->imageData = (char*)secondImage[2]->p_Image2D[0];

//start = clock();
//	cvCalcOpticalFlowBM(IplFirst, IplSecond, sizeBlock, sizeShift, sizeRange, 0, IplVelocityX, IplVelocityY);
//end = clock();


	MappTimer(M_TIMER_RESET, &vreme);
	pCCornerCoarse->DetectCorners(secondImage[2]->p_Image2D, thr);
	CreateCornerList(pCCornerCoarse, &CL1);
	MappTimer(M_TIMER_READ, &vreme);
	fprintf(pFileVreme, "%f  ", vreme);


/*	pci = corner_image[0];
	memcpy(pci, secondImage_s->p_Image2D[0], Ndiv3/(scale*scale));
	pCCorner_s->CreateCornerImage(secondImage_s->p_Image2D, corner_image);
	fwrite(corner_image[0],sizeof(unsigned char), 240*320/(scale*scale), pFileImage);

	Mr = AllignImage(&CL1, &CL0);
	Ms = update_homography(Mr, 4.0f);
	for(i=0;i<3;i++)for(j=0;j<3;j++)Mt[i][j] = Mr.a[i][j];
	inv3x3(Mt, iMt);
	subsampleImage(secondImage[0], secondImage_coarse);
	background_update(bgdImage_coarse->p_Image2D, firstImage_coarse->p_Image2D, secondImage_coarse->p_Image2D, Ms, Width/4, Height/4);

	nTargetLabel =
		TrackMCSegment(bgdImage_coarse, secondImage_coarse, foregroundImage[2], ptz1.m_yTicks, &Torso);

	for(i=0;i<Height/4;i++)
	for(j=0;j<Width/4;j++)
		foregroundImage[2]->p_Image2D[i][j] = abs(secondImage_coarse->p_Image2D[i][j] - bgdImage_coarse->p_Image2D[i][j])+1;
	ExpandMonochromeImage(foregroundImage[2], foregroundImage[0]);
*/
/*
	//	No update
	for(i=0;i<Height;i++)
	for(j=0;j<Width;j++)
		foregroundImage[0]->p_Image2D[i][j] = abs(secondImage[0]->p_Image2D[i][j] - firstImage[0]->p_Image2D[i][j]);

	fwrite(foregroundImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);

	//	Translation - Affine Matching
	background_update(bgdImage[0]->p_Image2D, firstImage[0]->p_Image2D, secondImage[0]->p_Image2D, Mr, Width, Height);
	for(i=0;i<Height;i++)
	for(j=0;j<Width;j++)
		foregroundImage[0]->p_Image2D[i][j] = abs(secondImage[0]->p_Image2D[i][j] - bgdImage[0]->p_Image2D[i][j]);

	fwrite(foregroundImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);


	//	Homography Matching
	Mr = AllignImage(&CL1, &CL0,2);
	background_update(bgdImage[0]->p_Image2D, firstImage[0]->p_Image2D, secondImage[0]->p_Image2D, Mr, Width, Height);
	for(i=0;i<Height;i++)
	for(j=0;j<Width;j++)
		foregroundImage[0]->p_Image2D[i][j] = abs(secondImage[0]->p_Image2D[i][j] - bgdImage[0]->p_Image2D[i][j]);

	fwrite(foregroundImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);

*/
	//	Affine Matching
	dMatrix Mh(3,3);

	MappTimer(M_TIMER_RESET, &vreme);
	Mr = AllignImage(&CL1, &CL0,1, scale);
	Mr.a[0][2]*=scale;
	Mr.a[1][2]*=scale;
	Mr.a[2][0]/=scale;
	Mr.a[2][1]/=scale;
	MappTimer(M_TIMER_READ, &vreme);
	fprintf(pFileVreme, "%f  ", vreme);

	MappTimer(M_TIMER_RESET, &vreme);
	pCCorner_s->DetectCorners(firstImage_s->p_Image2D, thr);
	CreateCornerList(pCCorner_s, &CL0);
	pCCorner_s->DetectCorners(secondImage_s->p_Image2D, thr);
	CreateCornerList(pCCorner_s, &CL1);
	Mr = AllignImageAffine(&CL1, &CL0, &Mr, 1, scale);
	MappTimer(M_TIMER_READ, &vreme);
	fprintf(pFileVreme, "%f ", vreme);
	Mr.a[0][2]*=scale;
	Mr.a[1][2]*=scale;
	Mr.a[2][0]/=scale;
	Mr.a[2][1]/=scale;

	MappTimer(M_TIMER_RESET, &vreme);
//	background_update(bgdImage[0]->p_Image2D, firstImage[0]->p_Image2D, secondImage[0]->p_Image2D, Mr, Width, Height);
	iplBackgroundUpdate(bgdImage[0]->p_Image2D[0], firstImage[0]->p_Image2D[0], secondImage[0]->p_Image2D[0], Mr.inverse(), Width, Height);
//	for(i=0;i<Height;i++)
//	for(j=0;j<Width;j++)
//		foregroundImage[0]->p_Image2D[i][j] = abs(secondImage[0]->p_Image2D[i][j] - bgdImage[0]->p_Image2D[i][j]);
	brSubtract(bgdImage[0], secondImage[0], foregroundImage[0]);

	MappTimer(M_TIMER_READ, &vreme);
	fprintf(pFileVreme, "%f ", vreme);

	fwrite(foregroundImage[0]->p_Image2D[0],sizeof(unsigned char), 240*320, pFileImage);

	
	// ALWAYS switch frames dml 3/25/00
		// if you don't you can get two difference regions
		// of the same person (one from the old ref image and one from the new)
	p_src = secondImage_color->m_GetImage();	
	int image_display = 0;
	if (biteDialog.EnableMILWin) image_display = 0; // dml 6/15/00
	
	p_dst = resultImage_color->m_GetImage();
	MappTimer(M_TIMER_RESET, &vreme);
	if(image_display == 0)
	{
		p_src = foregroundImage[0]->m_GetImage();
		
		for(i=0;i<Ndiv3;i++) // display an inverted b/w motion picture
		{
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src;
			*p_dst++ = MAX_GREY_VALUE - *p_src++;
		}
	}
	else
	{
		memcpy( p_dst, p_src, N); // display the original image
	}
	MappTimer(M_TIMER_READ, &vreme);
	fprintf(pFileVreme, "%f \n", vreme);

	fclose(pFileImage);
	DisplayBMP(resultImage_color->m_GetImage()); // dml 6/15/00

	if(ptz2.m_yTicks>1.57)
		ptz2.m_yTicks-=1;
	pCPTZController->setCameraPositionRadians(&ptz2);

	Sleep(10);
}



void CTracker::TestTrackContinuosMCTarget()
{
	CPTZRadians ptz;
	ptz.m_xTicks = ptz_current.m_xTicks;
	ptz.m_xTicks = ptz_current.m_xTicks;
	int nCam; 
	static a =1;
	if(a)
	{
		pCPTZController->getCameraNumber(&nCam);
		pCPTZController->setCameraNumber(nCam);
		pCPTZController->getCameraPosition(&(ptz.m_xTicks), &(ptz.m_yTicks), &(ptz.m_zTicks));
		TRACE("pos=%f \n",ptz.m_xTicks);
		a=0;
	}

	ptz_current.m_xTicks = ptz.m_xTicks - 0.2f;
	ptz_current.m_yTicks = ptz.m_yTicks + 0.04f;
	ptz.m_zTicks = ptz_current.m_zTicks = 170;
	TestRegistration(ptz, ptz_current);
}


void CTracker::ColorMotion(milImage<unsigned char>* pImage_color, Image<unsigned char>* pImage, CCorner* pCCorner, CORNER_LIST* pCL)
{
	unsigned char* p_img;
	int N = 3*320*240;
	tRectangle	Torso_fine;

	if (BMPflag==false) 
		GrabImage_Color(pImage_color, pImage);
	else 
		ReadBMPImage(pImage_color, pImage);

	p_img = p_image_buffer + image_data_size*((current_frame++)%IMAGE_BUFFER_SIZE);

//	SetRectangle(&SR, 60, 60, 260, 180);
	SR.x_min = __max(Torso.x_min-50,2);	 SR.x_max = __min(Torso.x_max+50,318);	 
	SR.y_min = __max(Torso.y_min-50,2);	 SR.y_max = __min(Torso.y_max+50,238);	 

	int p[2];
	float BestMatch;
	// p = (x, y);
	TargetHistogram->HistogramMatching(pImage_color, &Torso, &SR, p, &BestMatch);


	if(BestMatch>0.1)
	{
//		TargetHistogram->HistogramUpdate(pImage_color, &Torso);
		POI[0] = p[0]/4;
		POI[1] = p[1]/4;
		nTargetLabel=1; // dml 3/24/00 
		}
	else nTargetLabel = -1; // no target

	CopyRectangle(&Torso, &Torso_fine, 1);
	unsigned char *p_src, *p_dst;
	p_src = pImage_color->m_GetImage();
	p_dst = resultImage_color->m_GetImage();
	memcpy( p_dst, p_src, N);

	if(BestMatch>0) 
		PlotRectangle(resultImage_color, Torso_fine, 1);
	memcpy(p_img, resultImage_color->p_Image2D[0], image_data_size);
		
	int thr[2] = {0, 400};
	pCCornerHalf->DetectCorners(pImage->p_Image2D, thr);
	CreateCornerList(pCCorner, pCL);
}

void brSubtract(Image<unsigned char>* pIm1, Image<unsigned char>* pIm2, Image<unsigned char>* pImdif)
{
	int x,y;
	x = pIm1->m_GetRows();
	y = pIm1->m_GetCols();
	IplImage *IplFirst, *IplSecond, *IplResult;
  
	IplFirst = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                  
	IplSecond = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                 
	IplResult = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                 

	IplFirst->imageData = (char *)pIm1->p_Image2D[0];
	IplSecond->imageData = (char *)pIm2->p_Image2D[0];
	IplResult->imageData = (char *)pImdif->p_Image2D[0];

//	iplSubtract(IplFirst, IplSecond, IplResult);
//	iplAbs(IplResult, IplResult);
	cvAbsDiff(IplFirst, IplSecond, IplResult);
}

void brImageDifference(Image<unsigned char>* pIm1, Image<unsigned char>* pIm2, Image<unsigned char>* pImdif, int threshold)
{
	int x,y;
	x = pIm1->m_GetRows();
	y = pIm1->m_GetCols();
	IplImage *IplFirst, *IplSecond, *IplResult;
  
	IplFirst = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                  
	IplSecond = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                 
	IplResult = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                 

	IplFirst->imageData = (char *)pIm1->p_Image2D[0];
	IplSecond->imageData = (char *)pIm2->p_Image2D[0];
	IplResult->imageData = (char *)pImdif->p_Image2D[0];

	cvAbsDiff(IplFirst, IplSecond, IplResult);
	cvThreshold(IplResult, IplResult, threshold,255,CV_THRESH_BINARY);

	iplThreshold(IplResult, IplResult, threshold);
//	iplOpen(IplResult, IplResult, 1);
}

void brBlur(Image<unsigned char>* pIm1, Image<unsigned char>* pIm2, tRectangle* pRect, int threshold, int cx, int cy)
{
	int x,y, dx, dy;
	x = pIm1->m_GetRows();
	y = pIm1->m_GetCols();
	dx = pRect->x_max - pRect->x_min;
	dy = pRect->y_max - pRect->y_min;

	IplImage *IplFirst, *IplSecond;
  
	IplFirst = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                  
	IplSecond = iplCreateImageHeader(1, 0, IPL_DEPTH_8U, "GRAY",                      
         "GRAY", IPL_DATA_ORDER_PIXEL, IPL_ORIGIN_TL, IPL_ALIGN_DWORD,             
         x, y, NULL, NULL, NULL, NULL);                 

	IplFirst->imageData = (char *)pIm1->p_Image2D[0];
	IplSecond->imageData = (char *)pIm2->p_Image2D[0];

	iplMultiplyS(IplFirst, IplFirst, 100);
	iplBlur(IplFirst, IplSecond, dx, dy, cx, cy);
//	iplThreshold(IplSecond, IplSecond, threshold);
	cvThreshold(IplSecond, IplSecond, threshold,255,CV_THRESH_BINARY);
}