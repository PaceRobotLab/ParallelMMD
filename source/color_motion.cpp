#include <math.h>
#include "tracker.h"
#include "median.h"
#include "label.h"
#include "motion.h"

#define HISTOGRAM_SIZE	512
#define COLOR_THRESHOLD	33


//FILE* pFile2 = fopen("D:\\users\\kori\\ColorDif.txt","wt");

// Comment added by esc: July 29, 1999
// This function is using the  image foregroundImage[0]->p_Image2D
// where R,G and B are stored one after the others 
// like RRRRRR..., GGGGG...,BBBBB
// as opposed to the way we usually store RGB color which is RGBRGBRGB.....
// Not usable with the rest of the implementation 
void CTracker::GetRectangleColor(tRectangle R, RGBColor* pColor)
{
	int i,j,i1,j1, r, g, b, N = 0;
	unsigned char **mask = foregroundImage[2]->p_Image2D;
	unsigned char **fine_r, **fine_g, **fine_b;
	fine_r = foregroundImage[0]->p_Image2D;
	fine_g = fine_r + Width*Height;
	fine_b = fine_g + Width*Height;

	int median_r, median_g, median_b, hist_r[HISTOGRAM_SIZE], hist_g[HISTOGRAM_SIZE], hist_b[HISTOGRAM_SIZE], 
		hist_r1[HISTOGRAM_SIZE], hist_g1[HISTOGRAM_SIZE], hist_b1[HISTOGRAM_SIZE];
	float sigma_r, sigma_g, sigma_b;

	for(i=0;i<HISTOGRAM_SIZE;i++)hist_r[i] = hist_g[i] = hist_b[i] = hist_r1[i] = hist_g1[i] = hist_b1[i] = 0;

	for(i = R.y_min; i <= R.y_max; i++)
	for(j = R.x_min; j <= R.x_max; j++)
//	if(mask[i][j] > 0)	changed by mdt 11/21/99
	{
		N++;
		for(i1 = 4*i; i1 < 4*i + 4; i1++)
		for(j1 = 4*j; j1 < 4*j + 4; j1++)
		{
			r = fine_r[i][j];
			g = fine_g[i][j];
			b = fine_b[i][j];
			hist_r[r]++;
			hist_g[g]++;
			hist_b[b]++;
		}
	}

	median_r = GetMedianAndSigmaFromCountingSort(16*N, hist_r, hist_r1, &sigma_r);
	median_g = GetMedianAndSigmaFromCountingSort(16*N, hist_g, hist_g1, &sigma_g);
	median_b = GetMedianAndSigmaFromCountingSort(16*N, hist_b, hist_b1, &sigma_b);
	pColor->R = median_r;
	pColor->G = median_g;
	pColor->B = median_b;
	pColor->sigma_R = sigma_r;
	pColor->sigma_G = sigma_g;
	pColor->sigma_B = sigma_b;
}

void CTracker::GetRGBRectangleColor(milImage<unsigned char>* ColorImage, tRectangle R, RGBColor* pColor)
{
	int i,j, r, g, b, y, N = 0;
	int r1, r2, g1, g2, b1, b2;
	int R1, G1, B1, R2, G2, B2;
	r1 = r2 = g1 = g2 = b1 = b2 = 0;
	R1 = R2 = G1 = G2 = B1 = B2 = 0;
	unsigned char **fine;
	fine = ColorImage->p_Image2D;
//	N = (R.y_max - R.y_min + 1)*(R.x_max - R.x_min + 1);

	int median_r, hist_r[512], hist_g[HISTOGRAM_SIZE], hist_b[HISTOGRAM_SIZE], 
		hist_r1[HISTOGRAM_SIZE], hist_g1[HISTOGRAM_SIZE], hist_b1[HISTOGRAM_SIZE], n;
	float mean_r, mean_g, mean_b, sigma_r, sigma_g, sigma_b, fN;

	for(i=0;i<HISTOGRAM_SIZE;i++)hist_r[i] = hist_g[i] = hist_b[i] = hist_r1[i] = hist_g1[i] = hist_b1[i] = 0;

	for(i = R.y_min; i <= R.y_max; i++)
	for(j = R.x_min; j <= R.x_max; j++)
	{
		N++;
		r = fine[i][3*j];
		g = fine[i][3*j+1];
		b = fine[i][3*j+2];
		R1+= r; G1+= g; B1+= b;
		R2+= r*r; G2+= g*g; B2+= b*b;
		n = r/32 + 8*(g/32) + 64*(b/32);
		hist_r[n]++;
//		hist_g[g/32]++;
//		hist_b[b/32]++;
		y = (r+g+b+1)/3;
		
	}

	median_r = GetMedianFromCountingSort(N/2, hist_r);
//	median_g = GetMedianFromCountingSort(N/2, hist_g);
//	median_b = GetMedianFromCountingSort(N/2, hist_b);
	b = median_r/64;
	median_r-= 64*b;
	g = median_r/8;
	r = median_r - 8*g;

/*	median_r = GetMedianAndSigmaFromCountingSort(256, hist_r, hist_r1, &sigma_r);
	median_g = GetMedianAndSigmaFromCountingSort(256, hist_g, hist_g1, &sigma_g);
	median_b = GetMedianAndSigmaFromCountingSort(256, hist_b, hist_b1, &sigma_b);
*/

	fN = float(N);
	mean_r = R1/fN;
	mean_g = G1/fN;
	mean_b = B1/fN;
	sigma_r = (float)sqrt(R2/fN - mean_r*mean_r);
	sigma_g = (float)sqrt(G2/fN - mean_g*mean_g);
	sigma_b = (float)sqrt(B2/fN - mean_b*mean_b);


	pColor->R = (unsigned char)mean_r;//int(32*r+16);
	pColor->G = (unsigned char)mean_g;//int(32*g+16);
	pColor->B = (unsigned char)mean_b;//int(32*b+16);
	pColor->sigma_R = __max(5,sigma_r);
	pColor->sigma_G = __max(5,sigma_g);
	pColor->sigma_B = __max(5,sigma_b);
}

void CTracker::Get_rgYRectangleColor(milImage<unsigned char>* ColorImage, tRectangle R, RGBColor* pColor)
{
	int i,j, r, g, y, N = 0;
	int r1, r2, g1, g2, y1, y2;
	r1 = r2 = g1 = g2 = y1 = y2 = 0;
	unsigned char **fine, p_rgY[3];
	fine = ColorImage->p_Image2D;
//	N = (R.y_max - R.y_min + 1)*(R.x_max - R.x_min + 1);

	float mean_r, mean_g, mean_y, sigma_r, sigma_g, sigma_y, fN;

	for(i = R.y_min; i <= R.y_max; i++)
	for(j = R.x_min; j <= R.x_max; j++)
	{
		N++;
		RGB2rgY(fine[i]+3*j, p_rgY);
		r = p_rgY[0];
		g = p_rgY[1];
		y = p_rgY[2];
		r1+= r; g1+= g; y1+= y;
		r2+= r*r; g2+= g*g; y2+= y*y;
	}

	fN = float(N);
	mean_r = r1/fN;
	mean_g = g1/fN;
	mean_y = y1/fN;
	sigma_r = (float)sqrt(r2/fN - mean_r*mean_r);
	sigma_g = (float)sqrt(g2/fN - mean_g*mean_g);
	sigma_y = (float)sqrt(y2/fN - mean_y*mean_y);


	pColor->R = (unsigned char)mean_r;
	pColor->G = (unsigned char)mean_g;
	pColor->B = (unsigned char)mean_y;
	pColor->sigma_R = __max(5,(unsigned char)sigma_r);
	pColor->sigma_G = __max(5,(unsigned char)sigma_g);
	pColor->sigma_B = __max(5,(unsigned char)sigma_y);
}


// This function constructs a HSI target color model from the region of 
// the bounding box R in the color image ColorIMage

void CTracker::GetHSIRectangleColor(milImage<unsigned char>* ColorImage, tRectangle R, RGBColor* pColor)
{
	int i,j, r, g, y, N = 0;
	int r1, r2, g1, g2, y1, y2;
	r1 = r2 = g1 = g2 = y1 = y2 = 0;
	unsigned char **fine, pHSI[3];
	fine = ColorImage->p_Image2D;
//	N = (R.y_max - R.y_min + 1)*(R.x_max - R.x_min + 1);

	float mean_r, mean_g, mean_y, sigma_r, sigma_g, sigma_y, fN;

	for(i = R.y_min; i <= R.y_max; i++)
	for(j = R.x_min; j <= R.x_max; j++)
	{
	

		// dml,esc 12/3/99
		// this code rejects any pixel that is saturated, since
		// those pixels have no real information about the target color

		if ( fine[i][3*j]<250 || fine[i][3*j+1]<250 || fine[i][3*j+2]<250 )
		{	
			N++;
			RGB2HSI(fine[i]+3*j, pHSI);
			r = pHSI[0];
			g = pHSI[1];
			y = pHSI[2];

			r1+= r; g1+= g; y1+= y;
			r2+= r*r; g2+= g*g; y2+= y*y;
		}
	}

	fN = float(N);
	mean_r = r1/fN;
	mean_g = g1/fN;
	mean_y = y1/fN;
	sigma_r = float(sqrt(r2/fN - mean_r*mean_r));
	sigma_g = float(sqrt(g2/fN - mean_g*mean_g));
	sigma_y = float(sqrt(y2/fN - mean_y*mean_y));


	pColor->R = (unsigned char)mean_r;
	pColor->G = (unsigned char)mean_g;
	pColor->B = (unsigned char)mean_y;
	pColor->sigma_R = __max(5,(unsigned char)sigma_r);
	pColor->sigma_G = __max(5,(unsigned char)sigma_g);
	pColor->sigma_B = __max(5,(unsigned char)sigma_y);
}

void CTracker::GetRectangleColor(milImage<unsigned char>* ColorImage, tRectangle R, RGBColor* pColor, int type)
{
	switch(type)
	{
	case RGB:
		GetRGBRectangleColor(ColorImage, R, pColor);
	case HSI: 
		GetHSIRectangleColor(ColorImage, R, pColor);
		break;
	case rgY:
		Get_rgYRectangleColor(ColorImage, R, pColor);
		break;
	}
}


unsigned char CTracker::CompareColors(nColor* pColor, unsigned char r, unsigned char g, float threshold)
{
	float a,b;
	a = (r - pColor->r)/pColor->sigma_r;
	b = (g - pColor->g)/pColor->sigma_g;

	if( a*a + b*b < threshold*threshold )
		return 1;
	else
		return 0;
}

void CTracker::ComputeColorMeanModel(Image<MonochromeImage>* ColorImage, Image<MonochromeImage>* ForegroundImage, 
									 tRectangle* TBox, RGBColor* BdyColor)
{
	int i,j,i2,j2, N=Height*Width/16, scale =4;
	int x = Width/4, y=Height/4;
	unsigned char** p_fgd2D = ForegroundImage->p_Image2D;
	int N_moving_pixels=0; 
    float float_R=0., float_G=0., float_B=0.;
	float r,g,b,R2, G2, B2;
	R2 = G2 = B2 = 0.0f;

//	for(i=0;i<y;i++)
//	 for(j=0;j<x;j++) 
	for(i=TBox->y_min;i<=TBox->y_max;i++)
	for(j=TBox->x_min;j<=TBox->x_max;j++) 
//	  if(p_fgd2D[i][j] > 0)		changed by mdt 11/21/99
	  {
		i2 = scale*i;
		j2 = 3*scale*j;

		N_moving_pixels++;
		r = (float) ColorImage->p_Image2D[i2][j2];
		g = (float) ColorImage->p_Image2D[i2][j2+1];
		b = (float) ColorImage->p_Image2D[i2][j2+2];
		float_R+= r;
		float_G+= g;
		float_B+= b;
		R2+= r*r;
		G2+= g*g;
		B2+= b*b;
	  }
       
		float_R=float_R/(float)N_moving_pixels;
		float_G=float_G/(float)N_moving_pixels;
		float_B=float_B/(float)N_moving_pixels;


		BdyColor->R=(unsigned char) float_R;
		BdyColor->G=(unsigned char) float_G;
		BdyColor->B=(unsigned char) float_B;
		BdyColor->r = float_R/(float_R+float_G+float_B);
		BdyColor->g = float_G/(float_R+float_G+float_B);
		BdyColor->sigma_R = sqrt(R2/(float)N_moving_pixels - float_R*float_R);
		BdyColor->sigma_G = sqrt(G2/(float)N_moving_pixels - float_G*float_G);
		BdyColor->sigma_B = sqrt(B2/(float)N_moving_pixels - float_B*float_B);

}

int CTracker::TrackColorSegment(Image<int>* Image1, Image<int>* Image2, Image<MonochromeImage>* ForegroundImage, 
								float tilt, tRectangle* BoundingBox, tRectangle* SearchRegion)
{
	double dist_threshold = 3.0; 

	int i,j, k, i2,j2, N=Height*Width/16, Min, Max, Median,scale = 4;
	int MedianLocation = int(N/2);
	int x = Width/4, y=Height/4, **tp;
	int *p_img1, *p_img2, *p_imgdif;

	p_img1 = (int*)Image1->m_GetImage();
	p_img2 = (int*)Image2->m_GetImage();
	p_imgdif = (int*)differenceImage_coarse->m_GetImage();
	int** p_imgdif2 = differenceImage_coarse->p_Image2D;
	unsigned char* p_fgd = ForegroundImage->m_GetImage();
	unsigned char** p_fgd2 = ForegroundImage->p_Image2D;
	int x1, x2, y1, y2;
	
	if(SearchRegion == 0)
		i = 0;
	else
		i = (SearchRegion->x_max - SearchRegion->x_min)*(SearchRegion->y_max - SearchRegion->y_min);


	if(i == 0)
	{
		x1 = y1 = 1;
		x2 = x-1;
		y2 = y-1;
	}
	else
	{
		x1 = SearchRegion->x_min;
		x2 = SearchRegion->x_max;
		y1 = SearchRegion->y_min;
		y2 = SearchRegion->y_max;
	}


		

	Min = Max = abs((*p_img2 - *p_img1)/16);

	memset(p_fgd, 0, N);	//mdt 02/14/00	set foreground to 0


	//	Compute Difference image and find it's min and max 
	//	this should be done only for the Search Region. The value for the thresholds
	//	could be computed only once when the camera moves
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

/*	for(i=N-1;i>=0;i--)
	{
		if(abs(p_imgdif[i])>threshold)
			(p_imgdif[i]) = 1;
//			p_imgdif2[i>0?i-1:0][j]=p_imgdif2[i][j] = 1;	//dml 3/23/00 added expansion		  
		else
			(p_imgdif[i]) = 0;
	}
*/

	// new code added by mdt to combine color and motion filtering
	
	for(i=0;i<y;i++)
	for(j=0;j<x;j++)
		if(abs(p_imgdif2[i][j])>threshold)
			p_imgdif2[i][j] = 1;			  
		else
			p_imgdif2[i][j] = 0;
	


//	NoiseFiltering(differenceImage_coarse, foregroundImage[2]);
	//	NOISE FILTERING FUNCTION: START DEBUG 
	tp = p_imgdif2;

	int count;
	for(i=y1; i<y2; i++)
	for(j=x1; j<x2; j++)
	{
		if(tp[i][j]>0)
		{
			count = tp[i-1][j-1] + tp[i-1][j] + tp[i-1][j+1] + tp[i][j-1] + tp[i][j+1] + tp[i+1][j-1] + tp[i+1][j] + tp[i+1][j+1];
			if(count>2)
				p_fgd2[i][j] = 1;
			else
				p_fgd2[i][j] = 0;
		}
		else
			p_fgd2[i][j] = 0;		//mdt 02/14/00		doesn't need this
//		foregroundImage[2]->p_Image2D[i][j] = tp[i][j];
	}
	//	foregroundImage[2]->p_Image[i] = (differenceImage_coarse->p_Image[i]);
	//	END FILTERING FUNCTION - END DEBUG



	int nbobj, index = 0;
//	Find connected components labeled now from 2 to nbobj+1
	// result is placed in image, all pixels in an "object" are assigned
	// to the label for that object

	label(p_fgd2, 2, y-1, 2, x-1, 3, &nbobj);


//	Allocate memory for objects

#define MAX_OBJECTS 100
#define MAX_BB 1000

	int objects[MAX_OBJECTS], mean_x[MAX_OBJECTS], mean_y[MAX_OBJECTS];

	int nBBMaxX[MAX_OBJECTS], nBBMaxY[MAX_OBJECTS], nBBMinX[MAX_OBJECTS], nBBMinY[MAX_OBJECTS]; // dml 12/2/99 bounding boxes for objs

	int nSumTargetPixels[MAX_OBJECTS]; // dml 12/02/99 number of pixels that match target color

	if (nbobj>MAX_OBJECTS-1)
	{
	    //fprintf(pFile2,"Maximum number of objects exceeded(%d>%d), clipping to max!\n",nbobj,nbobj-1);
		nbobj=MAX_OBJECTS-1;
	}

	for(i=0; i<=nbobj+2; i++) // only need to clear what objects we use, dml 12/2/99
	{
		nSumTargetPixels[i] = objects[i] = 0; //dml 12/2/99 clear sum of target pixels
		mean_x[i] = mean_y[i] = 0;
		nBBMaxX[i]=nBBMaxY[i] = 0;
		nBBMinX[i]=nBBMinY[i] = MAX_BB; //dml 12/2/99 clear BBoxes
	}
	nBBMinX[nbobj+2] = BoundingBox->x_min;
	nBBMinY[nbobj+2] = BoundingBox->y_min;
	nBBMaxX[nbobj+2] = BoundingBox->x_max;
	nBBMaxY[nbobj+2] = BoundingBox->y_max;


//	Counting number of pixels for each and every object
	for(i=2; i<y-2; i++)
	for(j=2; j<x-2; j++)
	if( (k = foregroundImage[2]->p_Image2D[i][j]) > 0) // k is object label number
	{
		i2 = scale*i;
		j2 = 3*scale*j;

		objects[k]++;
		mean_x[k]+= j;
		mean_y[k]+= i;

		if (j>nBBMaxX[k]) nBBMaxX[k]=j; // dml 12/2/99 calculate bboxes fer objects
		if (j<nBBMinX[k]) nBBMinX[k]=j;
		if (i>nBBMaxY[k]) nBBMaxY[k]=i;
		if (i<nBBMinY[k]) nBBMinY[k]=i;
	}


	// dml 12/2/99 count all color pixels in BBoxes for each object

	float		histogram_match;
	tRectangle	pRec;

	for (k=0; k<=nbobj+2; k++)
	if (nBBMinY[k]!=MAX_BB)  // skip this if its not initialized
	{
		SetRectangle(&pRec, scale*nBBMinX[k], scale*nBBMinY[k], scale*nBBMaxX[k], scale*nBBMaxY[k]);
		histogram_match = TargetHistogram->CompareCompositeHistograms(secondImage_color, &pRec);
		nSumTargetPixels[k] = int(100*histogram_match);
	}


	index = 0;
	// dml 12/2/99 
	// This loop decides which of the objects is actually the Target object
	// it is based on two criteria. It must be bigger than a certain minimum
	// size of motion pixels, and it must have the most pixels of target color


	int nMaxColorIndex = -1;   // indicates no index selected
	int nMaxMotionIndex = -1;
	int nMinMotionSize = 7;   // min size value from mdt
	int nMinColorSize  = 10;   // min size guessed at by dml

	int nMaxSumTargetPixels=nMinColorSize; // max to date of target color pixels

	for(k=0;k<nbobj+2;k++) // for all the number of objects
 	 if(objects[k]>nMinMotionSize)	
		if (nSumTargetPixels[k]>nMaxSumTargetPixels) // check for best color match
		{ 
			nMaxSumTargetPixels=nSumTargetPixels[k];
			nMaxColorIndex  = k;
		}

	k = nbobj+2;		
	if (nSumTargetPixels[k]>1.2*nMaxSumTargetPixels)	// check if the previous position is 
	{												// still a good color match (20% better than moving) dml 3/23/00
		nMaxSumTargetPixels=nSumTargetPixels[k];
		nMaxColorIndex  = k;
		//fprintf(pFile2,"Static match is the best \n");
	}

	
	

	if (nMaxColorIndex != -1 && nMaxSumTargetPixels > nMinColorSize) 
	{
		index = nMaxColorIndex; // select best colored target
		// dml 12/13/99, made up this confidence value to have something to report on GUI
		// its the number of (matched color pixels / BBox area)
		nConfidence = nMaxSumTargetPixels; /*int( 100*nMaxSumTargetPixels/
		                         ((nBBMaxX[nMaxColorIndex]-nBBMinX[nMaxColorIndex])
									*(nBBMaxY[nMaxColorIndex]-nBBMinY[nMaxColorIndex]))  ); */
		nLastTargetCount = int(float(nMaxSumTargetPixels)*0.85); // dml 2/00
		// Insist that the next target will have at least 85% of the matched pixels of
		// this target. This helps avoid mismatch problem when traget stops moving
		SetRectangle(BoundingBox, nBBMinX[index], nBBMinY[index], nBBMaxX[index], nBBMaxY[index]);
	}
	else
	{
		index = -1; // select NO target at all
	}

	
	//	Label the corresponding object to 2 and all the others to 0 (background)
	for(i=0;i<N;i++)
	if(p_fgd[i]>0)
	{
		if(p_fgd[i] != index)
			p_fgd[i] = 0;
		else p_fgd[i] = 2;
	}

	if(index == nbobj+2)
		index = 0;

 return index; // return which target label chosen
}



int CTracker::TrackMCSegment(Image<int>* Image1, Image<int>* Image2, Image<MonochromeImage>* ForegroundImage, 
								float tilt, tRectangle* Target, tRectangle* SearchRegion)
{
	double dist_threshold = 3.0; 

	int i,j, k, i2,j2, N=Height*Width/16, Min, Max, Median,scale = 4;
	int MedianLocation = int(N/2);
	int x = Width/4, y=Height/4, **tp;
	int *p_img1, *p_img2, *p_imgdif;

	p_img1 = (int*)Image1->m_GetImage();
	p_img2 = (int*)Image2->m_GetImage();
	p_imgdif = (int*)differenceImage_coarse->m_GetImage();
	int** p_imgdif2 = differenceImage_coarse->p_Image2D;
	unsigned char* p_fgd = ForegroundImage->m_GetImage();
	unsigned char** p_fgd2 = ForegroundImage->p_Image2D;
	int x1, x2, y1, y2;
	
	if(SearchRegion == 0)
		i = 0;
	else
		i = (SearchRegion->x_max - SearchRegion->x_min)*(SearchRegion->y_max - SearchRegion->y_min);


	if(i == 0)
	{
		x1 = y1 = 1;
		x2 = x-1;
		y2 = y-1;
	}
	else
	{
		x1 = SearchRegion->x_min;
		x2 = SearchRegion->x_max;
		y1 = SearchRegion->y_min;
		y2 = SearchRegion->y_max;
	}


		

	Min = Max = abs((*p_img2 - *p_img1)/16);

	memset(p_fgd, 0, N);	//mdt 02/14/00	set foreground to 0


	//	Compute Difference image and find it's min and max 
	//	this should be done only for the Search Region. The value for the thresholds
	//	could be computed only once when the camera moves
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


	for(i=0;i<y;i++)
	for(j=0;j<x;j++)
		if(abs(p_imgdif2[i][j])>threshold)
			p_imgdif2[i][j] = 1;			  
		else
			p_imgdif2[i][j] = 0;
	


//	NoiseFiltering(differenceImage_coarse, foregroundImage[2]);
	//	NOISE FILTERING FUNCTION: START DEBUG 
	tp = p_imgdif2;

	int count;
	for(i=y1; i<y2; i++)
	for(j=x1; j<x2; j++)
	{
		if(tp[i][j]>0)
		{
			count = tp[i-1][j-1] + tp[i-1][j] + tp[i-1][j+1] + tp[i][j-1] + tp[i][j+1] + tp[i+1][j-1] + tp[i+1][j] + tp[i+1][j+1];
			if(count>2)
				p_fgd2[i][j] = 1;
			else
				p_fgd2[i][j] = 0;
		}
		else
			p_fgd2[i][j] = 0;		//mdt 02/14/00		doesn't need this
//		foregroundImage[2]->p_Image2D[i][j] = tp[i][j];
	}
	//	foregroundImage[2]->p_Image[i] = (differenceImage_coarse->p_Image[i]);
	//	END FILTERING FUNCTION - END DEBUG



	int nbobj, index = 0;
//	Find connected components labeled now from 2 to nbobj+1
	// result is placed in image, all pixels in an "object" are assigned
	// to the label for that object

	label(p_fgd2, 2, y-1, 2, x-1, 3, &nbobj);


//	Allocate memory for objects

	#define MAX_OBJECTS 100
	#define MAX_BB 1000

	int objects[MAX_OBJECTS], mean_x[MAX_OBJECTS], mean_y[MAX_OBJECTS];
	tRectangle pBBox[MAX_OBJECTS]; // mdt 05/10/00 bounding boxes for objs
	int nSumTargetPixels[MAX_OBJECTS]; // dml 12/02/99 number of pixels that match target color

	if (nbobj>MAX_OBJECTS-1)
	{
	    //fprintf(pFile2,"Maximum number of objects exceeded(%d>%d), clipping to max!\n",nbobj,nbobj-1);
		nbobj=MAX_OBJECTS-1;
	}

	for(i=0; i<MAX_OBJECTS; i++) // only need to clear what objects we use, dml 12/2/99
	{
		nSumTargetPixels[i] = objects[i] = 0; //dml 12/2/99 clear sum of target pixels
		mean_x[i] = mean_y[i] = 0;
		pBBox[i].x_min = pBBox[i].y_min = MAX_BB;
		pBBox[i].x_max = pBBox[i].y_max = 0;
	}
	
	CopyRectangle(Target, &pBBox[nbobj+2]);

//	Counting number of pixels for each and every object
	for(i=2; i<y-2; i++)
	for(j=2; j<x-2; j++)
	if( (k = foregroundImage[2]->p_Image2D[i][j]) > 0) // k is object label number
	{
		i2 = scale*i;
		j2 = 3*scale*j;

		objects[k]++;
		mean_x[k]+= j;
		mean_y[k]+= i;

		if (j>pBBox[k].x_max) pBBox[k].x_max=j; // dml 12/2/99 calculate bboxes for objects
		if (j<pBBox[k].x_min) pBBox[k].x_min=j;
		if (i>pBBox[k].y_max) pBBox[k].y_max=i;
		if (i<pBBox[k].y_min) pBBox[k].y_min=i;
	}


	// dml 12/2/99 count all color pixels in BBoxes for each object

	float		histogram_match;
	int p[2];
	tRectangle	pTorso, pSearchRegion;
	CopyRectangle(Target, &pTorso);
	CopyRectangle(Target, &pSearchRegion);
	pSearchRegion.x_min = __max(0, pSearchRegion.x_min-10);	
	pSearchRegion.x_max = __min(Width, pSearchRegion.x_max+10);	
	pSearchRegion.y_min = __max(0, pSearchRegion.y_min-10);	
	pSearchRegion.y_max = __min(Height, pSearchRegion.y_max+10);	

	TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, &pSearchRegion, p, &histogram_match);
	CopyRectangle(&pTorso, &pBBox[nbobj+2]);
	nSumTargetPixels[nbobj+2] = int(100*histogram_match);

	for (k=0; k<nbobj+2; k++)
	if (objects[k]>10)  // skip this if its not initialized
	{
		CopyRectangle(&pBBox[k], &pSearchRegion, scale);
		ExpandSearchRegion(Target, &pSearchRegion);
		TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, &pSearchRegion, p, &histogram_match);
		nSumTargetPixels[k] = int(100*histogram_match);
		CopyRectangle(&pTorso, &pBBox[k]);
	}


	index = 0;
	// dml 12/2/99 
	// This loop decides which of the objects is actually the Target object
	// it is based on two criteria. It must be bigger than a certain minimum
	// size of motion pixels, and it must have the most pixels of target color


	int nMaxColorIndex = -1;   // indicates no index selected
	int nMaxMotionIndex = -1;
	int nMinMotionSize = 7;   // min size value from mdt
	int nMinColorSize  = COLOR_THRESHOLD;   // min size guessed at by dml, also, histogram match in percents

	int nMaxSumTargetPixels=nMinColorSize; // max to date of target color pixels

	for(k=0;k<nbobj+2;k++) // for all the number of objects
 	 if(objects[k]>nMinMotionSize)	
		if (nSumTargetPixels[k]>nMaxSumTargetPixels) // check for best color match
		{ 
			nMaxSumTargetPixels=nSumTargetPixels[k];
			nMaxColorIndex  = k;
		}

	k = nbobj+2;
	if (nSumTargetPixels[k]>1.2*nMaxSumTargetPixels)	// check if the previous position is 
	{												// still a good color match (20% better than moving) dml 3/23/00
		nMaxSumTargetPixels=nSumTargetPixels[k];
		nMaxColorIndex  = k;
	}

	
	

	if (nMaxColorIndex != -1 && nMaxSumTargetPixels > nMinColorSize) 
	{
		index = nMaxColorIndex; // select best colored target
		// dml 12/13/99, made up this confidence value to have something to report on GUI
		// its the number of (matched color pixels / BBox area)
		nConfidence = nMaxSumTargetPixels; /*int( 100*nMaxSumTargetPixels/
		                         ((nBBMaxX[nMaxColorIndex]-nBBMinX[nMaxColorIndex])
									*(nBBMaxY[nMaxColorIndex]-nBBMinY[nMaxColorIndex]))  ); */
		nLastTargetCount = int(float(nMaxSumTargetPixels)*0.85); // dml 2/00
		// Insist that the next target will have at least 85% of the matched pixels of
		// this target. This helps avoid mismatch problem when traget stops moving
		CopyRectangle(&pBBox[index], Target);
	}
	else
	{
		index = -1; // select NO target at all
	}

	if (Target->x_min<0)
		Target->x_min*= -1; // dml 6/27
    if (Target->y_min<0) 
		Target->y_min*= -1;

	//	Label the corresponding object to 2 and all the others to 0 (background)
	for(i=0;i<N;i++)
	if(p_fgd[i]>0)
	{
		if(p_fgd[i] != index)
			p_fgd[i] = 0;
		else p_fgd[i] = 2;
	}

	if(index == nbobj+2)
		index = 0;

 return index; // return which target label chosen
}


int CTracker::TrackMCSegment(Image<unsigned char>* Image1, Image<unsigned char>* Image2, Image<MonochromeImage>* ForegroundImage, 
								float tilt, tRectangle* Target, tRectangle* SearchRegion)
{
	double dist_threshold = 3.0; 

	int i,j, k, i2,j2, N=Height*Width/16, Min, Max, Median,scale = 4;
	int MedianLocation = int(N/2);
	int x = Width/4, y=Height/4, **tp;
	unsigned char *p_img1, *p_img2;
	int *p_imgdif;

	p_img1 = (unsigned char*)Image1->m_GetImage();
	p_img2 = (unsigned char*)Image2->m_GetImage();
	p_imgdif = (int*)differenceImage_coarse->m_GetImage();
	int** p_imgdif2 = differenceImage_coarse->p_Image2D;
	unsigned char* p_fgd = ForegroundImage->m_GetImage();
	unsigned char** p_fgd2 = ForegroundImage->p_Image2D;
	int x1, x2, y1, y2;
	
	if(SearchRegion == 0)
		i = 0;
	else
		i = (SearchRegion->x_max - SearchRegion->x_min)*(SearchRegion->y_max - SearchRegion->y_min);


	if(i == 0)
	{
		x1 = y1 = 1;
		x2 = x-1;
		y2 = y-1;
	}
	else
	{
		x1 = SearchRegion->x_min;
		x2 = SearchRegion->x_max;
		y1 = SearchRegion->y_min;
		y2 = SearchRegion->y_max;
	}


		

	Min = Max = abs((*p_img2 - *p_img1));

	memset(p_fgd, 0, N);	//mdt 02/14/00	set foreground to 0


	//	Compute Difference image and find it's min and max 
	//	this should be done only for the Search Region. The value for the thresholds
	//	could be computed only once when the camera moves
	for(i=0;i<N;i++)
	{
		j = p_img2[i] - p_img1[i];
		j = abs(j);
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


	threshold = 10.0;

	for(i=0;i<y;i++)
	for(j=0;j<x;j++)
		if(abs(p_imgdif2[i][j])>threshold)
			p_imgdif2[i][j] = 1;			  
		else
			p_imgdif2[i][j] = 0;
	


//	NoiseFiltering(differenceImage_coarse, foregroundImage[2]);
	//	NOISE FILTERING FUNCTION: START DEBUG 
	tp = p_imgdif2;

	int count;
	for(i=y1; i<y2; i++)
	for(j=x1; j<x2; j++)
	{
		if(tp[i][j]>0)
		{
			count = tp[i-1][j-1] + tp[i-1][j] + tp[i-1][j+1] + tp[i][j-1] + tp[i][j+1] + tp[i+1][j-1] + tp[i+1][j] + tp[i+1][j+1];
			if(count>2)
				p_fgd2[i][j] = 1;
			else
				p_fgd2[i][j] = 0;
		}
		else
			p_fgd2[i][j] = 0;		//mdt 02/14/00		doesn't need this
//		foregroundImage[2]->p_Image2D[i][j] = tp[i][j];
	}
	//	foregroundImage[2]->p_Image[i] = (differenceImage_coarse->p_Image[i]);
	//	END FILTERING FUNCTION - END DEBUG



	int nbobj, index = 0;
//	Find connected components labeled now from 2 to nbobj+1
	// result is placed in image, all pixels in an "object" are assigned
	// to the label for that object

	label(p_fgd2, 2, y-1, 2, x-1, 3, &nbobj);


//	Allocate memory for objects

	#define MAX_OBJECTS 100
	#define MAX_BB 1000

	int objects[MAX_OBJECTS], mean_x[MAX_OBJECTS], mean_y[MAX_OBJECTS];
	tRectangle pBBox[MAX_OBJECTS]; // mdt 05/10/00 bounding boxes for objs
	int nSumTargetPixels[MAX_OBJECTS]; // dml 12/02/99 number of pixels that match target color

	if (nbobj>MAX_OBJECTS-1)
	{
	    //fprintf(pFile2,"Maximum number of objects exceeded(%d>%d), clipping to max!\n",nbobj,nbobj-1);
		nbobj=MAX_OBJECTS-1;
	}

	for(i=0; i<MAX_OBJECTS; i++) // only need to clear what objects we use, dml 12/2/99
	{
		nSumTargetPixels[i] = objects[i] = 0; //dml 12/2/99 clear sum of target pixels
		mean_x[i] = mean_y[i] = 0;
		pBBox[i].x_min = pBBox[i].y_min = MAX_BB;
		pBBox[i].x_max = pBBox[i].y_max = 0;
	}
	
	CopyRectangle(Target, &pBBox[nbobj+2]);

//	Counting number of pixels for each and every object
	for(i=2; i<y-2; i++)
	for(j=2; j<x-2; j++)
	if( (k = foregroundImage[2]->p_Image2D[i][j]) > 0) // k is object label number
	{
		i2 = scale*i;
		j2 = 3*scale*j;

		objects[k]++;
		mean_x[k]+= j;
		mean_y[k]+= i;

		if (j>pBBox[k].x_max) pBBox[k].x_max=j; // dml 12/2/99 calculate bboxes for objects
		if (j<pBBox[k].x_min) pBBox[k].x_min=j;
		if (i>pBBox[k].y_max) pBBox[k].y_max=i;
		if (i<pBBox[k].y_min) pBBox[k].y_min=i;
	}


	// dml 12/2/99 count all color pixels in BBoxes for each object

	float		histogram_match;
	int p[2];
	tRectangle	pTorso, pSearchRegion;
	CopyRectangle(Target, &pTorso);
	CopyRectangle(Target, &pSearchRegion);
	pSearchRegion.x_min = __max(0, pSearchRegion.x_min-20);	
	pSearchRegion.x_max = __min(Width-1, pSearchRegion.x_max+20);	
	pSearchRegion.y_min = __max(0, pSearchRegion.y_min-20);	
	pSearchRegion.y_max = __min(Height-1, pSearchRegion.y_max+20);	

	TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, &pSearchRegion, p, &histogram_match);
	CopyRectangle(&pTorso, &pBBox[nbobj+2]);
	nSumTargetPixels[nbobj+2] = int(100*histogram_match);

	for (k=0; k<nbobj+2; k++)
	if (objects[k]>10)  // skip this if its not initialized
	{
		CopyRectangle(&pBBox[k], &pSearchRegion, scale);
		ExpandSearchRegion(Target, &pSearchRegion);
		TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, &pSearchRegion, p, &histogram_match);
		nSumTargetPixels[k] = int(100*histogram_match);
		CopyRectangle(&pTorso, &pBBox[k]);
	}


	index = 0;
	// dml 12/2/99 
	// This loop decides which of the objects is actually the Target object
	// it is based on two criteria. It must be bigger than a certain minimum
	// size of motion pixels, and it must have the most pixels of target color


	int nMaxColorIndex = -1;   // indicates no index selected
	int nMaxMotionIndex = -1;
	int nMinMotionSize = 7;   // min size value from mdt
	int nMinColorSize  = COLOR_THRESHOLD;   // min size guessed at by dml, also, histogram match in percents

	int nMaxSumTargetPixels=nMinColorSize; // max to date of target color pixels

	for(k=0;k<nbobj+2;k++) // for all the number of objects
 	 if(objects[k]>nMinMotionSize)	
		if (nSumTargetPixels[k]>nMaxSumTargetPixels) // check for best color match
		{ 
			nMaxSumTargetPixels=nSumTargetPixels[k];
			nMaxColorIndex  = k;
		}

	k = nbobj+2;
	if (nSumTargetPixels[k]>1.2*nMaxSumTargetPixels)	// check if the previous position is 
	{												// still a good color match (20% better than moving) dml 3/23/00
		nMaxSumTargetPixels=nSumTargetPixels[k];
		nMaxColorIndex  = k;
	}

	
	

	if (nMaxColorIndex != -1 && nMaxSumTargetPixels > nMinColorSize) 
	{
		index = nMaxColorIndex; // select best colored target
		// dml 12/13/99, made up this confidence value to have something to report on GUI
		// its the number of (matched color pixels / BBox area)
		nConfidence = nMaxSumTargetPixels; /*int( 100*nMaxSumTargetPixels/
		                         ((nBBMaxX[nMaxColorIndex]-nBBMinX[nMaxColorIndex])
									*(nBBMaxY[nMaxColorIndex]-nBBMinY[nMaxColorIndex]))  ); */
		nLastTargetCount = int(float(nMaxSumTargetPixels)*0.85); // dml 2/00
		// Insist that the next target will have at least 85% of the matched pixels of
		// this target. This helps avoid mismatch problem when traget stops moving
		CopyRectangle(&pBBox[index], Target);
	}
	else
	{
		index = -1; // select NO target at all
	}

	if (Target->x_min<0)
		Target->x_min*= -1; // dml 6/27
    if (Target->y_min<0) 
		Target->y_min*= -1;

	//	Label the corresponding object to 2 and all the others to 0 (background)
	for(i=0;i<N;i++)
	if(p_fgd[i]>0)
	{
		if(p_fgd[i] != index)
			p_fgd[i] = 0;
		else p_fgd[i] = 2;
	}

	if(index == nbobj+2)
		index = 0;

 return index; // return which target label chosen
}

int CTracker::TrackMCSegment(Image<unsigned char>* firstImage, Image<unsigned char>* secondImage, Image<unsigned char>* bgdImage, Image<MonochromeImage>* foregroundImage, 
								dMatrix Mr, tRectangle* Target, tRectangle* SearchRegion)
{
	int nbobj;
	iplBackgroundUpdate(bgdImage->p_Image2D[0], firstImage->p_Image2D[0], secondImage->p_Image2D[0], Mr.inverse(), bgdImage->m_GetRows(), bgdImage->m_GetCols());
	brImageDifference(bgdImage, secondImage, foregroundImage, 50);
	brBlur(foregroundImage, foregroundImage, Target, 40);
//	label(foregroundImage->p_Image2D, 2, Height-1, 2, Width-1, 3, &nbobj);
//	FindGlobalBox(foregroundImage, SearchRegion);


	float	hm, histogram_match[2];
	int p0[2], p1[2];
	tRectangle	pTorso, pSearchRegion;
	CopyRectangle(Target, &pTorso);
	CopyRectangle(Target, &pSearchRegion);
	pSearchRegion.x_min = __max(0, pSearchRegion.x_min-20);	
	pSearchRegion.x_max = __min(Width-1, pSearchRegion.x_max+20);	
	pSearchRegion.y_min = __max(0, pSearchRegion.y_min-20);	
	pSearchRegion.y_max = __min(Height-1, pSearchRegion.y_max+20);	

	int index = -1;
	TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, &pSearchRegion, p0, &histogram_match[0], pCand);
	TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, SearchRegion, p1, &histogram_match[1], pCand, foregroundImage->p_Image2D);//, &pSearchRegion);
//	histogram_match[0] = histogram_match[1] - 10;
	if(histogram_match[0]>histogram_match[1])
	{
		index = 0;
		hm = histogram_match[0];
	}
	else
	{
		index = 1;
		hm = histogram_match[1];
	}

	if(100*hm < COLOR_THRESHOLD)
		index = -1;
	else if(index == 0)
	{
		Target->x_min = p0[0];
		Target->x_max = p0[0] + pTorso.x_max-pTorso.x_min;
		Target->y_min = p0[1];
		Target->y_max = p0[1] + pTorso.y_max-pTorso.y_min;
	}
	else
	{
		Target->x_min = p1[0];
		Target->x_max = p1[0] + pTorso.x_max-pTorso.x_min;
		Target->y_min = p1[1];
		Target->y_max = p1[1] + pTorso.y_max-pTorso.y_min;
	}

	return index; // return which target label chosen
}


int CTracker::ColorMotionDetection(Image<unsigned char>* pIm1, Image<unsigned char>* pIm2, Image<MonochromeImage>* pFgdImage, 
								int scale, int threshold, tRectangle* Target, tRectangle* SearchRegion)
{
	int i,j,k,x,y,N;
	x = pIm1->m_GetRows();
	y = pIm1->m_GetCols();
	N = x*y;
	unsigned char* p_fgd = pFgdImage->p_Image2D[0];
 	unsigned char** p_fgd2 = pFgdImage->p_Image2D;
 	brImageDifference(pIm1, pIm2, pFgdImage, threshold);

	int nbobj, index = 0;
//	Find connected components labeled now from 2 to nbobj+1
	// result is placed in image, all pixels in an "object" are assigned
	// to the label for that object

	label(p_fgd2, 2, y-1, 2, x-1, 3, &nbobj);


//	Allocate memory for objects

	#define MAX_OBJECTS 100
	#define MAX_BB 1000

	int objects[MAX_OBJECTS], mean_x[MAX_OBJECTS], mean_y[MAX_OBJECTS];
	tRectangle pBBox[MAX_OBJECTS]; // mdt 05/10/00 bounding boxes for objs
	int nSumTargetPixels[MAX_OBJECTS]; // dml 12/02/99 number of pixels that match target color

	if (nbobj>MAX_OBJECTS-1)
	{
	    //fprintf(pFile2,"Maximum number of objects exceeded(%d>%d), clipping to max!\n",nbobj,nbobj-1);
		nbobj=MAX_OBJECTS-1;
	}

	for(i=0; i<MAX_OBJECTS; i++) // only need to clear what objects we use, dml 12/2/99
	{
		nSumTargetPixels[i] = objects[i] = 0; //dml 12/2/99 clear sum of target pixels
		mean_x[i] = mean_y[i] = 0;
		pBBox[i].x_min = pBBox[i].y_min = MAX_BB;
		pBBox[i].x_max = pBBox[i].y_max = 0;
	}
	
	CopyRectangle(Target, &pBBox[nbobj+2]);

//	Counting number of pixels for each and every object
	for(i=2; i<y-2; i++)
	for(j=2; j<x-2; j++)
	if( (k = foregroundImage[2]->p_Image2D[i][j]) > 0) // k is object label number
	{
		objects[k]++;
		mean_x[k]+= j;
		mean_y[k]+= i;

		if (j>pBBox[k].x_max) pBBox[k].x_max=j; // dml 12/2/99 calculate bboxes for objects
		if (j<pBBox[k].x_min) pBBox[k].x_min=j;
		if (i>pBBox[k].y_max) pBBox[k].y_max=i;
		if (i<pBBox[k].y_min) pBBox[k].y_min=i;
	}


	// dml 12/2/99 count all color pixels in BBoxes for each object

	float		histogram_match;
	int p[2];
	tRectangle	pTorso, pSearchRegion;
	CopyRectangle(Target, &pTorso);
	CopyRectangle(Target, &pSearchRegion);
	pSearchRegion.x_min = __max(0, pSearchRegion.x_min-20);	
	pSearchRegion.x_max = __min(Width-1, pSearchRegion.x_max+20);	
	pSearchRegion.y_min = __max(0, pSearchRegion.y_min-20);	
	pSearchRegion.y_max = __min(Height-1, pSearchRegion.y_max+20);	

	TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, &pSearchRegion, p, &histogram_match);
	CopyRectangle(&pTorso, &pBBox[nbobj+2]);
	nSumTargetPixels[nbobj+2] = int(100*histogram_match);

	for (k=0; k<nbobj+2; k++)
	if (objects[k]>10)  // skip this if its not initialized
	{
		CopyRectangle(&pBBox[k], &pSearchRegion, scale);
		ExpandSearchRegion(Target, &pSearchRegion);
		TargetHistogram->sHistogramMatching(secondImage_color, &pTorso, &pSearchRegion, p, &histogram_match);
		nSumTargetPixels[k] = int(100*histogram_match);
		CopyRectangle(&pTorso, &pBBox[k]);
	}


	index = 0;
	// dml 12/2/99 
	// This loop decides which of the objects is actually the Target object
	// it is based on two criteria. It must be bigger than a certain minimum
	// size of motion pixels, and it must have the most pixels of target color


	int nMaxColorIndex = -1;   // indicates no index selected
	int nMaxMotionIndex = -1;
	int nMinMotionSize = 7;   // min size value from mdt
	int nMinColorSize  = COLOR_THRESHOLD;   // min size guessed at by dml, also, histogram match in percents

	int nMaxSumTargetPixels=nMinColorSize; // max to date of target color pixels

	for(k=0;k<nbobj+2;k++) // for all the number of objects
 	 if(objects[k]>nMinMotionSize)	
		if (nSumTargetPixels[k]>nMaxSumTargetPixels) // check for best color match
		{ 
			nMaxSumTargetPixels=nSumTargetPixels[k];
			nMaxColorIndex  = k;
		}

	k = nbobj+2;
	if (nSumTargetPixels[k]>1.2*nMaxSumTargetPixels)	// check if the previous position is 
	{												// still a good color match (20% better than moving) dml 3/23/00
		nMaxSumTargetPixels=nSumTargetPixels[k];
		nMaxColorIndex  = k;
	}

	
	

	if (nMaxColorIndex != -1 && nMaxSumTargetPixels > nMinColorSize) 
	{
		index = nMaxColorIndex; // select best colored target
		// dml 12/13/99, made up this confidence value to have something to report on GUI
		// its the number of (matched color pixels / BBox area)
		nConfidence = nMaxSumTargetPixels; /*int( 100*nMaxSumTargetPixels/
		                         ((nBBMaxX[nMaxColorIndex]-nBBMinX[nMaxColorIndex])
									*(nBBMaxY[nMaxColorIndex]-nBBMinY[nMaxColorIndex]))  ); */
		nLastTargetCount = int(float(nMaxSumTargetPixels)*0.85); // dml 2/00
		// Insist that the next target will have at least 85% of the matched pixels of
		// this target. This helps avoid mismatch problem when traget stops moving
		CopyRectangle(&pBBox[index], Target);
	}
	else
	{
		index = -1; // select NO target at all
	}

	if (Target->x_min<0)
		Target->x_min*= -1; // dml 6/27
    if (Target->y_min<0) 
		Target->y_min*= -1;

	//	Label the corresponding object to 2 and all the others to 0 (background)
	for(i=0;i<N;i++)
	if(p_fgd[i]>0)
	{
		if(p_fgd[i] != index)
			p_fgd[i] = 0;
		else p_fgd[i] = 2;
	}

	if(index == nbobj+2)
		index = 0;

 return index; // return which target label chosen
}

/*
void CTracker::TrackColorSegment(Image<int>* Image1, Image<int>* Image2, Image<MonochromeImage>* ForegroundImage, 
								 float tilt, tRectangle* SearchRegion, float threshold)
{
	double dist_threshold = 3.0;
	int i,j, k, i2,j2, current_R, current_G, current_B, N=Height*Width/16, Min, Max, Median,scale = 4;
	int MedianLocation = int(N/2);
	int x = Width/4, y=Height/4, **tp;
	int x1 = SearchRegion->x_min, x2 = SearchRegion->x_max, y1 = SearchRegion->y_min, y2 = SearchRegion->y_max;
	int *p_img1, *p_img2, *p_imgdif;
	p_img1 = (int*)Image1->m_GetImage();
	p_img2 = (int*)Image2->m_GetImage();
	p_imgdif = (int*)differenceImage_coarse->m_GetImage();
	unsigned char* p_fgd = ForegroundImage->m_GetImage();

	Min = Max = abs((*p_img2 - *p_img1)/16);
	
	//	Compute Difference image and find it's min and max 
	for(i=0;i<N;i++)
	{
		p_fgd[i] = 0;
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
	threshold = 2.5f*sigma;
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
	for(i=y1; i<y2; i++)
	for(j=x1; j<x2; j++)
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
			foregroundImage[2]->p_Image2D[i][j] = 0;
//		foregroundImage[2]->p_Image2D[i][j] = tp[i][j];
	}
	//	foregroundImage[2]->p_Image[i] = (differenceImage_coarse->p_Image[i]);
	//	END FILTERING FUNCTION - END DEBUG

	int nbobj, index = 0;
//	Find connected components labeled now from 2 to nbobj+1
	label(foregroundImage[2]->p_Image2D, 2, y-1, 2, x-1, 3, &nbobj);

//	Allocate memory for objects
	int objects[100], mean_x[100], mean_y[100], R[100], G[100], B[100];
	for(i=0; i<100; i++)
	{
		objects[i] = 0;
		mean_x[i] = mean_y[i] = R[i] = G[i] = B[i] = 0;
	}

//	Counting number of pixels for each and every object
	for(i=2; i<y-2; i++)
	for(j=2; j<x-2; j++)
	if( (k = foregroundImage[2]->p_Image2D[i][j]) > 0)
	{
		i2 = scale*i;
		j2 = 3*scale*j;

		current_R = secondImage_color->p_Image2D[i2][j2];
		current_G = secondImage_color->p_Image2D[i2][j2+1];
		current_B = secondImage_color->p_Image2D[i2][j2+2];

		objects[k]++;
		mean_x[k]+= j;
		mean_y[k]+= i;
		R[k]+= current_R;
		G[k]+= current_G;
		B[k]+= current_B;
	}

	int	Min_Size = 20;	//to be modified, mdt
	index = 0;
	double min_color_dif = 1500.0;
	color_distance_threshold = 500.0;
	
/*
	for(i=1; i<100;i++)
	if(objects[i]>Min_Size)
	{
		lum = double(objects[i]);
		mean_x[i]/= lum;
		mean_y[i]/= lum;
		segments[i].R = double(R[i])/lum;
		segments[i].G = double(G[i])/lum;
		segments[i].B = double(B[i])/lum;
		segments[i].center_of_gravity[0] = mean_x[i];
		segments[i].center_of_gravity[1] = mean_y[i];
		dr = (AccumulatedBodyColor.R - segments[i].R);//AccumulatedBodyColor.sigma_R;
		dg = (AccumulatedBodyColor.G - segments[i].G);//AccumulatedBodyColor.sigma_G;
		db = (AccumulatedBodyColor.B - segments[i].B);//AccumulatedBodyColor.sigma_B;

		color_diference = sqrt(dr*dr + dg*dg + db*db);
		fprintf(pFile2,"%8.4f \n", color_diference);
		if(color_diference < min_color_dif)
		{
			min_color_dif = color_diference;
			index = i;
		}
//			fprintf(pFile2,"%8.4f  %3d  %6.4f %6.4f \n", min_color_dif, objects[i], segments[i].r, segments[i].g);
	}
	
//	fprintf(pFile2,"nbobj = %6d  \n", nbobj);

	if(min_color_dif > color_distance_threshold)
		index = -1;

	for(i=0;i<100;i++)
	if(objects[i]>Min_Size)
	{
		Min_Size = objects[i];
		index = i;
	}

	//	Label the corresponding object to 2 and all the others to 0 (background)
	for(i=0;i<N;i++)
	if(p_fgd[i])
	{
		if(p_fgd[i] != index)
			p_fgd[i] = 0;
		else p_fgd[i] = 2;
	}

}
*/

void CTracker::Track_OR_Build_ColorSegment(Image<int>* Image1, Image<int>* Image2, Image<MonochromeImage>* ForegroundImage, float tilt_angle)
{
	if(NoF>40)
		TrackColorSegment(firstImage_coarse, secondImage_coarse, foregroundImage[2], tilt_angle, &BBox);//, &SR, 9.0);
	else
	{
		ComputeForegroundImage(firstImage_coarse, secondImage_coarse, foregroundImage[2]);
	}
}

void CTracker::UpdateColorModel(RGBColor* InstanteniousColor, RGBColor* AverageColor, int n, double* color_distance)
{
	double mean_R, sigma_R, mean_G, sigma_G, mean_B, sigma_B, curr_R, curr_G, curr_B, dR, dG, dB;
	*color_distance = 10000.0;
	curr_R = InstanteniousColor->R;
	curr_G = InstanteniousColor->G;
	curr_B = InstanteniousColor->B;
	mean_R = AverageColor->R;
	mean_G = AverageColor->G;
	mean_B = AverageColor->B;
	sigma_R = AverageColor->sigma_R;
	sigma_G = AverageColor->sigma_G;
	sigma_B = AverageColor->sigma_B;

	if(n==1)
	{
		AverageColor->R = (unsigned char)curr_R;
		AverageColor->G = (unsigned char)curr_G;
		AverageColor->B = (unsigned char)curr_B;
		AverageColor->sigma_R = 0.0;
		AverageColor->sigma_G = 0.0;
		AverageColor->sigma_B = 0.0;
	}
	else
	{
		if( (sigma_R != 0.0) && (sigma_G != 0.0) && (sigma_B != 0.0) )
		{
			dR = (curr_R - mean_R)/sigma_R;
			dG = (curr_G - mean_G)/sigma_G;
			dB = (curr_B - mean_B)/sigma_B;
			*color_distance = sqrt(dR*dR + dG*dG +dB*dB);
		}
		AverageColor->R = (unsigned char)((n-1)*mean_R + curr_R)/n;
		AverageColor->G = (unsigned char)((n-1)*mean_G + curr_G)/n;
		AverageColor->B = (unsigned char)((n-1)*mean_B + curr_B)/n;
		AverageColor->sigma_R = sqrt(( (n-1)*(mean_R*mean_G + sigma_R*sigma_R) + curr_R*curr_G )/n - AverageColor->R*AverageColor->R);
		AverageColor->sigma_G = sqrt(( (n-1)*(mean_G*mean_G + sigma_G*sigma_G) + curr_G*curr_G )/n - AverageColor->G*AverageColor->G);
		AverageColor->sigma_B = sqrt(( (n-1)*(mean_B*mean_B + sigma_B*sigma_B) + curr_B*curr_B )/n - AverageColor->B*AverageColor->B);
	}
}

// Method added by esc  11/30/99
// Classify pixel using the color model 


void CTracker::ClassifyPixelsFromModel(RGBColor* pColor,Image<unsigned char>* fine_color_Image)
{
	int i,j;
	unsigned char **fine;	
	fine = fine_color_Image->p_Image2D;
	double sigma_R_2,sigma_G_2,sigma_B_2;

	sigma_R_2 = pColor->sigma_R * pColor->sigma_R;
	sigma_G_2 = pColor->sigma_G * pColor->sigma_G;
	sigma_B_2 = pColor->sigma_B * pColor->sigma_B; 

	for(i=0;i<Height;i++)
	 for(j=0;j<Width;j++) 	     
		if(  (double) abs((fine[i][3*j]- pColor->R)*(fine[i][3*j]- pColor->R)) < 3. * sigma_R_2 &&
			 (double) abs((fine[i][3*j+1]- pColor->G)*(fine[i][3*j+1]- pColor->G)) < 3. * sigma_G_2 &&
			 (double)  abs((fine[i][3*j+2]- pColor->B)*(fine[i][3*j+2]- pColor->B)) < 3. * sigma_B_2 )
		{
		fine[i][3*j] =	255; // pColor->R;
		fine[i][3*j+1] = 0; // pColor->G;
		fine[i][3*j+2] = 0; // pColor->B; 
		}

}


// dml 12/02/99 added function to classify one pixel ucRGB[0..2] according to pColor color model

int CTracker::ClassifyPixelsFromModel(RGBColor* pColor,unsigned char *ucRGB)
{

	double sigma_R_2,sigma_G_2,sigma_B_2;

	sigma_R_2 = pColor->sigma_R * pColor->sigma_R;
	sigma_G_2 = pColor->sigma_G * pColor->sigma_G;
	sigma_B_2 = pColor->sigma_B * pColor->sigma_B; // dml 12/02/99 should precalculate

  
	return (  (double) abs((ucRGB[0]- pColor->R)*(ucRGB[0]- pColor->R)) < 3. * sigma_R_2 &&
			  (double) abs((ucRGB[1]- pColor->G)*(ucRGB[1]- pColor->G)) < 3. * sigma_G_2 &&
			  (double) abs((ucRGB[2]- pColor->B)*(ucRGB[2]- pColor->B)) < 3. * sigma_B_2 );

}


void CTracker::ClassifyPixelsFromModel(Image<unsigned char>* fine_color_Image, Image<unsigned char>* result_color_Image, RGBColor* pColor, tRectangle* R)
{
	int i,j,x1,x2,y1,y2;
/*	if(R == 0)
	{
*/		x1 = y1 = 0;
		x2 = Width;
		y2 = Height;
/*	}
	else
	{
		x1 = R->x_min;
		x2 = R->x_max+1;
		y1 = R->y_min;
		y2 = R->y_max+1;
	}
*/
	unsigned char **fine, **result;	
	fine = fine_color_Image->p_Image2D;
	result = result_color_Image->p_Image2D;

	double sigma_R_2,sigma_G_2,sigma_B_2;

	sigma_R_2 = pColor->sigma_R * pColor->sigma_R;
	sigma_G_2 = pColor->sigma_G * pColor->sigma_G;
	sigma_B_2 = pColor->sigma_B * pColor->sigma_B; 

	for(i= y1; i< y2; i++)
	for(j= x1; j< x2; j++)	     
		if(  (double) abs((fine[i][3*j]- pColor->R)*(fine[i][3*j]- pColor->R)) < 3. * sigma_R_2 &&
			 (double) abs((fine[i][3*j+1]- pColor->G)*(fine[i][3*j+1]- pColor->G)) < 3. * sigma_G_2 &&
			 (double)  abs((fine[i][3*j+2]- pColor->B)*(fine[i][3*j+2]- pColor->B)) < 3. * sigma_B_2 )
		{
		result[i][3*j] =	255; // pColor->R;
		result[i][3*j+1] = 0; // pColor->G;
		result[i][3*j+2] = 0; // pColor->B; 
		}

}


void CTracker::RGB2rgY(Image<unsigned char>* pRGB_Image, Image<unsigned char>* p_rgY_Image, tRectangle* R)
{
	int i,j,x1,x2,y1,y2;
	unsigned char **p1 = pRGB_Image->p_Image2D;
	unsigned char **p2 = p_rgY_Image->p_Image2D;
	unsigned char *p_src, *p_dst;

	if(R == 0)
	{
		x1 = y1 = 0;
		x2 = Width;
		y2 = Height;
	}
	else
	{
		x1 = R->x_min;
		x2 = R->x_max+1;
		y1 = R->y_min;
		y2 = R->y_max+1;
	}

	for(i= y1; i< y2; i++)
	{
		p_src = p1[i] + 3*x1;
		p_dst = p2[i] + 3*x1;
		for(j= x1; j< x2; j++)
		{
			RGB2rgY(p_src, p_dst);
			p_src+=3;
			p_dst+=3;
		}
	}
}


void CTracker::RGB2rgY(unsigned char* pRGB, unsigned char* p_rgY)
{
	unsigned char *ptr, r, g;
	int Y;
	ptr = pRGB;
	r = *ptr++;
	g = *ptr++;
	Y = r + g + *ptr;
	p_rgY[0] = (256*r)/Y;
	p_rgY[1] = (256*g)/Y;
	p_rgY[2] = (Y+1)/3;
}

void CTracker::rgY2RGB(unsigned char* p_rgY, unsigned char* pRGB)
{
	unsigned char *ptr, r, g, b;
	int Y;
	ptr = p_rgY;
	r = *ptr++;
	g = *ptr++;
	b = 256 - r - g;
	Y = 3*(*ptr);

	pRGB[0] = (Y*r)/256;
	pRGB[1] = (Y*g)/256;
	pRGB[2] = (Y*b)/256;
}

void CTracker::rgY2RGB(Image<unsigned char>* p_rgY_Image, Image<unsigned char>* pRGB_Image, tRectangle* R)
{
	int i,j,x1,x2,y1,y2;
	unsigned char **p2 = pRGB_Image->p_Image2D;
	unsigned char **p1 = p_rgY_Image->p_Image2D;
	unsigned char *p_src, *p_dst;

	if(R == 0)
	{
		x1 = y1 = 0;
		x2 = Width;
		y2 = Height;
	}
	else
	{
		x1 = R->x_min;
		x2 = R->x_max+1;
		y1 = R->y_min;
		y2 = R->y_max+1;
	}

	for(i= y1; i< y2; i++)
	{
		p_src = p1[i] + 3*R->x_min;
		p_dst = p2[i] + 3*R->x_min;
		for(j= x1; j< x2; j++)
		{
			rgY2RGB(p_src, p_dst);
			p_src+=3;
			p_dst+=3;
		}
	}
}

void CTracker::RGB2HSI(unsigned char* pRGB, unsigned char* pHSI)
{
	unsigned char *ptr, r, g, b, min_rgb;
	int rg,rb;
	float i, s, h;
	ptr = pRGB;
	r = *ptr++;
	g = *ptr++;
	b = *ptr;
	rg = r - g;
	rb = r - b;
	i = (float)(r+g+b);
	min_rgb = __min(__min(r,g), b);
	s = 255 - (765*min_rgb)/i;
	h = (float)acos( (rg + rb)/(2 * sqrt( rg*rg +rb*(g-b) ) ) );
	h = float(127.5*h/M_PI);
	if(b>g)
		h = 255 - h;

	i = i/3;
	pHSI[0] = (unsigned char)(h+.495); 
	pHSI[1] = (unsigned char)(s+.495); 
	pHSI[2] = (unsigned char)(i+.495);
}

void CTracker::RGB2HSI(Image<unsigned char>* pRGB_Image, Image<unsigned char>* pHSI_Image, tRectangle* R)
{
	int i,j,x1,x2,y1,y2;
	unsigned char **p1 = pRGB_Image->p_Image2D;
	unsigned char **p2 = pHSI_Image->p_Image2D;
	unsigned char *p_src, *p_dst;

	if(R == 0)
	{
		x1 = y1 = 0;
		x2 = Width;
		y2 = Height;
	}
	else
	{
		x1 = R->x_min;
		x2 = R->x_max+1;
		y1 = R->y_min;
		y2 = R->y_max+1;
	}

	for(i= y1; i< y2; i++)
	{
		p_src = p1[i] + 3*x1;
		p_dst = p2[i] + 3*x1;
		for(j= x1; j< x2; j++)
		{
			RGB2HSI(p_src, p_dst);
			p_src+=3;
			p_dst+=3;
		}
	}
}


int CTracker::HSI2RGB(unsigned char* pHSI, unsigned char* pRGB)
{
	int h, a;
	h = pHSI[0]; 

	if( h<64 || h>=192)
		a = HSI2RGB2(pHSI,pRGB);
	else
	{
		a = HSI2RGB1(pHSI,pRGB); 
		if( a != 1)
			a = HSI2RGB2(pHSI,pRGB);
	}
	return a;
}


int CTracker::HSI2RGB1(unsigned char* pHSI, unsigned char* pRGB)
{
	// assume that min = R

	float h, s, i, r, a, p, q, D, x1, x2, tmp, ch;
	h = float(pHSI[0]*M_PI/127.5); 
	s = float(pHSI[1]/255.0); 
	i = float(pHSI[2]);

	r = i*(1-s);
	ch = float(cos(h));
	a = float(0.75)/(ch*ch);
	p = 0.5f*(3*i-r);
	q = r*r - (r-i)*(3*i + a*(r-i));

	if(q < 0)
		return 0;

	D = p*p - q;
	if(fabs(D)<1e-4)
		D = 0.0f;

	if(D<0)
		return 0;

	x1 = p - (float)sqrt(D);

	if(x1 < r)
		return 0;

	x2 = 3*i - r - x1;
	if(x2 < x1)
	{
		tmp = x1;
		x1 = x2;
		x2 = x1;
	}

	pRGB[0] = (unsigned char)(r+.5);

	if(pHSI[0]<128)
	{
		pRGB[1] = (unsigned char)(x2+.5);
		pRGB[2] = (unsigned char)(x1+.5);
	}
	else
	{
		pRGB[1] = (unsigned char)(x1+.5);
		pRGB[2] = (unsigned char)(x2+.5);
	}
	return 1;
}



int CTracker::HSI2RGB2(unsigned char* pHSI, unsigned char* pRGB)
{
	// assume that min = G or B 

	float h, s, i, r, g, a, b, c, p, q, ch, tmp, D;

	h = float(pHSI[0]*M_PI/127.5f); 
	s = pHSI[1]/255.0f; 
	i = pHSI[2];
	
	g = i*(1-s);
	ch = 2.0f*(float)cos(h);
	tmp = 1/(ch*ch);
	a = 1 - 3*tmp;
	b = g - 3*i*(1-2*tmp);
	c = 3*i*i*(1-tmp) - g*(3*i - g);

	if( fabs(a) < 1e-5)
	{
		r = -c/b;
		b = 3*i - g - r;
		pRGB[0] = (unsigned char)(r+.5);
		if(pHSI[0]<128)
		{
			pRGB[1] = (unsigned char)(b+.5);
			pRGB[2] = (unsigned char)(g+.5);
		}
		else
		{
			pRGB[1] = (unsigned char)(g+.5);
			pRGB[2] = (unsigned char)(b+.5);
		}
		return 1;
	}

	p = -b/(2*a);
	q = c/a;
	D = p*p - q;

	if(fabs(D)<1e-4)
		D = 0.0f;

	if(D<0)
		return 0;

	D = (float)sqrt(D);
	r = p - D;
	b = 3*i - g - r;

	if( (r-i)*ch < 0 )
	{
		r = p + D;
		b = 3*i - g - r;
	}

	pRGB[0] = (unsigned char)(r+.5);

	if(pHSI[0]<128)
	{
		pRGB[1] = (unsigned char)(b+.5);
		pRGB[2] = (unsigned char)(g+.5);
	}
	else
	{
		pRGB[1] = (unsigned char)(g+.5);
		pRGB[2] = (unsigned char)(b+.5);
	}
	return 1;
}

void CTracker::HSI2RGB(Image<unsigned char>* pHSI_Image, Image<unsigned char>* pRGB_Image, tRectangle* R)
{
	int i,j,x1,x2,y1,y2;
	unsigned char **p2 = pRGB_Image->p_Image2D;
	unsigned char **p1 = pHSI_Image->p_Image2D;
	unsigned char *p_src, *p_dst;

	if(R == 0)
	{
		x1 = y1 = 0;
		x2 = Width;
		y2 = Height;
	}
	else
	{
		x1 = R->x_min;
		x2 = R->x_max+1;
		y1 = R->y_min;
		y2 = R->y_max+1;
	}

	for(i= y1; i< y2; i++)
	{
		p_src = p1[i] + 3*R->x_min;
		p_dst = p2[i] + 3*R->x_min;
		for(j= x1; j< x2; j++)
		{
			HSI2RGB(p_src, p_dst);
			p_src+=3;
			p_dst+=3;
		}
	}
}

void CTracker::ConvertFromRGB(Image<unsigned char>* pRGB_Image, Image<unsigned char>* pImage, int type, tRectangle* R)
{
	switch(type)
	{
	case HSI: 
		RGB2HSI(pRGB_Image, pImage, R);
		break;
	case rgY:
		RGB2rgY(pRGB_Image, pImage, R);
		break;
	}
}

		
void CTracker::ConvertFromRGB(unsigned char* pRGB, unsigned char* p, int type)
{
	switch(type)
	{
	case HSI: 
		RGB2HSI(pRGB, p);
		break;
	case rgY:
		RGB2rgY(pRGB, p);
		break;
	}
}


void CTracker::Convert2RGB(Image<unsigned char>* pImage, Image<unsigned char>* pRGB_Image, int type, tRectangle* R)
{
	switch(type)
	{
	case HSI: 
		HSI2RGB(pImage, pRGB_Image, R);
		break;
	case rgY:
		rgY2RGB(pImage, pRGB_Image, R);
		break;
	}
}

		
void CTracker::Convert2RGB(unsigned char* p, unsigned char* pRGB, int type)
{
	switch(type)
	{
	case HSI: 
		HSI2RGB(p, pRGB);
		break;
	case rgY:
		rgY2RGB(p, pRGB);
		break;
	}
}

int CTracker::CountSimmilarColorPixelsInRectangle(Image<unsigned char>* pImage, tRectangle* R, RGBColor* pColor)
{
	int i,j,x1,x2,y1,y2,N=0;
	x1 = R->x_min;
	x2 = R->x_max+1;
	y1 = R->y_min;
	y2 = R->y_max+1;

	unsigned char **fine = pImage->p_Image2D;
	double sigma_R_2,sigma_G_2,sigma_B_2;

	sigma_R_2 = pColor->sigma_R * pColor->sigma_R;
	sigma_G_2 = pColor->sigma_G * pColor->sigma_G;
	sigma_B_2 = pColor->sigma_B * pColor->sigma_B; 

	for(i= y1; i< y2; i++)
	for(j= x1; j< x2; j++)	     
		if(  (double) abs((fine[i][3*j]- pColor->R)*(fine[i][3*j]- pColor->R)) < 3. * sigma_R_2 &&
			 (double) abs((fine[i][3*j+1]- pColor->G)*(fine[i][3*j+1]- pColor->G)) < 3. * sigma_G_2 &&
			 (double)  abs((fine[i][3*j+2]- pColor->B)*(fine[i][3*j+2]- pColor->B)) < 3. * sigma_B_2 )
			N++;

	return N;
}

int CTracker::FindTorsoBox(Image<unsigned char>* pImage, tRectangle* SearchRegion, tRectangle* TBox, RGBColor* pColor)
{
	int i,j,x1,x2,y1,y2,i1,j1,dx,dy,scale = 4;
	unsigned char **fine = pImage->p_Image2D;
	unsigned char Temp[240][320];
	double sigma_R_2,sigma_G_2,sigma_B_2;

	x1 = 4*SearchRegion->x_min;
	x2 = 4*SearchRegion->x_max;
	y1 = 4*SearchRegion->y_min;
	y2 = 4*SearchRegion->y_max;

	dx = 4*(TBox->x_max - TBox->x_min);
	dy = 4*(TBox->y_max - TBox->y_min);

	sigma_R_2 = pColor->sigma_R * pColor->sigma_R;
	sigma_G_2 = pColor->sigma_G * pColor->sigma_G;
	sigma_B_2 = pColor->sigma_B * pColor->sigma_B; 

	for(i= y1; i< y2; i++)
	{
		i1 = i-y1;
		for(j= x1; j< x2; j++)	
		{
			j1 = j-x1;
			if(  (double) abs((fine[i][3*j]- pColor->R)*(fine[i][3*j]- pColor->R)) < 3. * sigma_R_2 &&
				(double) abs((fine[i][3*j+1]- pColor->G)*(fine[i][3*j+1]- pColor->G)) < 3. * sigma_G_2 &&
				(double)  abs((fine[i][3*j+2]- pColor->B)*(fine[i][3*j+2]- pColor->B)) < 3. * sigma_B_2 )
				Temp[i1][j1] = 1;
			else
				Temp[i1][j1] = 0;
		}
	}

	int a[240][320], b[240][320];
	int ty, tx;
	ty = y2 - y1; 
	tx = x2 - x1;

	for(i=0; i<ty; i++)
	{
		a[i][0]=Temp[i][0];
		for(j=1; j<dx; j++)
			a[i][0]+= Temp[i][j];

		for(j=dx; j<tx; j++)
			a[i][j-dx+1] = a[i][j-dx] + Temp[i][j] - Temp[i][j-dx];
	}

	for(j=0;j<tx-dx;j++)
	{
		b[0][j] = a[0][j];
		for(i=1;i<dy;i++)
			b[0][j]+= a[i][j];

		for(i=dy;i<ty;i++)
			b[i-dy+1][j] = b[i-dy][j] + a[i][j] - a[i-dy][j];
	}

	int x_max, y_max, b_max = 0;

	for(i=0;i<ty-dy;i++)
	for(j=0;j<tx-dx;j++)
	if(b[i][j]>b_max)
	{
		b_max = b[i][j];
		x_max = j;
		y_max = i;
	}

	if(b_max>0)
		SetRectangle(TBox, (x_max+x1+2)/4, (y_max+y1+2)/4, (x_max+x1+2+dx)/4, (y_max+y1+2+dy)/4);
	return b_max;
}

void FindGlobalBox(Image<unsigned char>* pImage, tRectangle* pBox)
{
	int w = pImage->m_GetRows();
	int h = pImage->m_GetCols();
	int i, j, x1, y1;
	unsigned char* p = pImage->p_Image2D[0], *p1, *p2;
	int w1, w2, wt, y_min, y_max;

//	First, find y_min;
	x1=0;
	while(!(*p++))
	{
		x1++;
	}

	pBox->y_min = y_min = int(x1/w);
	w1 = x1%w;

//	Find y_max
	p = pImage->p_Image2D[0];
	y1 = w*h-1;
	while(!p[y1])
		y1--;

	pBox->y_max = y_max = int(y1/w);
	w2 = y1%w;

	if(w2<w1)
	{
		wt = w1;
		w1 = w2;
		w2 = wt;
	}

//	Find x_min and x_max

	for(i=y_min+1; i<=y_max; i++)
	{
		p1 = p2 = p + i*w;
		for(j=0;j<w1;j++)
		if(*p1++)
			w1 = j;

		for(j=w-1; j>w2; j--)
		if(p2[j])
			w2 = j;
	}

	pBox->x_min = w1;
	pBox->x_max = w2;
}


