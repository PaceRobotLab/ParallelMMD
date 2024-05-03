//#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <memory.h>
#include <assert.h>
//#include "Timer.h"
#include <iostream.h>

#include "nms.h"
#include "ColorHistogram.h"
#include "brMemalloc.h"

FILE* pFile1 = fopen("D:\\users\\kori\\Hist.txt","wt");
FILE* pFileH = fopen("D:\\users\\kori\\Target.txt","wt");
FILE* pFileP = fopen("D:\\users\\kori\\Palette.raw","wb");
FILE* pFileHb = fopen("D:\\users\\kori\\hn.dat","wb");


int compare( const void *arg1, const void *arg2 );
void RGB2HSI(unsigned char* RGB, unsigned char* HSI);
PalInt** alloc2Dpalint(int x, int y);

CColorHistogram::CColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion, int histogram_size)
{
	int i;
	height						= p_InputColorImage->m_GetCols();
	width						= p_InputColorImage->m_GetRows()/3;
	hist_match	= alloc2Dshort(width, height);
	img_disp	= alloc2Duchar(width, height);
	tmp_peaks	= alloc2Duchar(width, height);
	TempVecArraysAlloc();

//	General parameters	
	gray_tolerance				= 10;
	intensity_tolerance			= 15;
	dim1						= 32;
	dim2						= 4;
	dim3						= 4;

//	Histogram related parameters
	pCompleteHistogram			= alloc2Dint(dim1,dim2);
	pNormalizedHistogram		= alloc2Dfloat(dim1,dim2);
	p_temporaryCH				= alloc2Dint(dim1,dim2);
	p_temporaryNCH				= alloc2Dfloat(dim1,dim2);

	pGrayHistogram				= new int [dim3];
	pNormalizedGrayHistogram	= new float [dim3];
	p_temporaryGH				= new float [dim3];

	p_hist_LUT					= alloc2Duchar(dim1, dim2);
	p_gray_LUT					= new unsigned char [dim3];

	p_histogram_colors			= alloc2Dint(2,16);
	p_gray_levels				= new int[4];

	
//	Image related parameters
	pPaletteImage				= alloc2Duchar(width, height);
//	pTA							= alloc2Dintv(width, height, 16); // temporary arrays
//	pTB							= alloc2Dintv(width, height, 16);
	pPTA						= alloc2Dpalint(width, height);
	pPTB						= alloc2Dpalint(width, height);



//	Initialization
	for(i=0; i<dim3; i++)
	{
		pGrayHistogram[i] = 0;
		pNormalizedGrayHistogram[i] = 0.0f;
		p_gray_LUT[i] = 0;
	}

	for(i=0;i<50;i++) histogram_vector[i] = 0;


//	Functiomality
	CreateCompositeHSIHistogram(p_InputColorImage, HistogramRegion, pNormalizedHistogram, pNormalizedGrayHistogram, dim1, dim2, dim3);
					//	CreateColorHistogram(p_InputColorImage, HistogramRegion, pCompleteHistogram, pNormalizedHistogram, dim1, dim2);
	
	n_histogram_colors = CreateHistogramVector(pNormalizedHistogram, pNormalizedGrayHistogram, p_histogram_colors, p_gray_levels, n_histogram_colors = 15, n_gray_levels, dim1, dim2, dim3);
	tRectangle SR;
	SR.x_min= 0;SR.y_min= 0;SR.x_max= 310;SR.y_max= 230;
	float hv0 = 0.0f;
	int p[2], sim;
				//	HistogramMatching(p_InputColorImage, HistogramRegion, &SR, p, &sim);
	CreateCompositeHSIPaletteImage(p_InputColorImage, &SR, p_histogram_colors, n_histogram_colors, p_gray_levels, n_gray_levels, pPaletteImage);
 	fwrite(pPaletteImage[0],sizeof(unsigned char), 240*320, pFileP);
	fclose(pFileP);
	fwrite(&hv0, sizeof(float), 1, pFileHb);
	fwrite(histogram_vector, sizeof(float), 15, pFileHb);
	fclose(pFileHb);
//	HistogramMatching(pPaletteImage, HistogramRegion, &SR, p, &sim);
	
	FILE* Fcand	 = fopen("e:\\users\\mdt\\cand.txt", "wt");

}
 
CColorHistogram::~CColorHistogram()
{
	delete [] pNewMemv1;
	delete [] pNewMemv2;

}

void CColorHistogram::TempVecArraysAlloc()
{
//	Temporary vector arrays
	v1 = new signed short *[height];
	assert (v1);
	v2 = new signed short *[height];
	assert (v2);

	size_t iSizeOfShort = sizeof(short);
	unsigned long size = 16* width * height * iSizeOfShort + 8;
	pNewMemv1 = new char [size];
	assert (pNewMemv1);
	pNewMemv2 = new char [size];
	assert (pNewMemv2);
	// Adjust
	unsigned int iOffset = (unsigned int)pNewMemv1 % 8;
	v1[0] = (short *)((unsigned int)(8 - iOffset) + (unsigned int)pNewMemv1);
	iOffset = (unsigned int)pNewMemv2 % 8;
	v2[0] = (short *)((unsigned int)(8 - iOffset) + (unsigned int)pNewMemv2);

	for (int i = height-1; i >= 0; --i)	{
		v1[i] = &v1[0][i*width*16];
		v2[i] = &v2[0][i*width*16];
	}

	for (i=height*width*16-1; i>=0; --i)	{
		v1[0][i] = 0;
		v2[0][i] = 0;
	}
}

//	Functions that create various kinds of histograms

void CColorHistogram::CreateColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, int** pCH, float** pNCH, int dimx, int dimy)
{
	int i,j, d2, c;
	int i1 = 256/dimy;
	int j1 = 256/dimx;
	for(i=0;i<dimy;i++)
		for(j=0;j<dimx;j++)
			pCH[i][j] = 0;

	float n = (float)(SearchRegion->y_max - SearchRegion->y_min + 1)*(SearchRegion->x_max - SearchRegion->x_min + 1);
	d2 = i1*j1;
	c = d2/2;
	unsigned char** p_color_data = p_InputColorImage->p_Image2D;
	
	for(i = SearchRegion->y_min; i <= SearchRegion->y_max; i++)
	 for(j = SearchRegion->x_min; j <= SearchRegion->x_max; j++)
		AssignColor2CompleteHistogramSimple(p_color_data[i][3*j], p_color_data[i][3*j+1], pCH, dimx, dimy);

	for(i=0;i<dimy;i++)
	{
	 for(j=0;j<dimx;j++)
	 {
	  pNCH[i][j] = pCH[i][j]/n;
	  fprintf(pFile1,"%4d ", pCH[i][j]);
	 }
	 fprintf(pFile1,"\n");
	}
//	fclose(pFile1);

}

void CColorHistogram::CreateHSIColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, int** pCH, float** pNCH, int dimx, int dimy)
{
	int i,j;
	int i1 = 256/dimy;
	int j1 = 256/dimx;
	for(i=0;i<dimy;i++)
		for(j=0;j<dimx;j++)
			pCH[i][j] = 0;
	float n = (float)(SearchRegion->y_max - SearchRegion->y_min + 1)*(SearchRegion->x_max - SearchRegion->x_min + 1);

	unsigned char** p_color_data = p_InputColorImage->p_Image2D;
	unsigned char hsi[3];
	
	for(i = SearchRegion->y_min; i <= SearchRegion->y_max; i++)
	 for(j = SearchRegion->x_min; j <= SearchRegion->x_max; j++)
	 {
		RGB2HSI(&(p_color_data[i][3*j]), hsi);
		AssignColor2CompleteHistogramSimple(hsi[0], hsi[1], pCH, dimx, dimy);
	 }


	for(i=0;i<dimy;i++)
	for(j=0;j<dimx;j++)
	  pNCH[i][j] = pCH[i][j]/n;

}



void CColorHistogram::CreateCompositeHSIHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, float** pNCH, float* pNGH, int dimx, int dimy, int dim_gray)
{
	int i,j;
	int i1 = 256/dimy;
	int j1 = 256/dimx;
	int r, g, b, t2 = gray_tolerance*gray_tolerance;
	int intensity;
	int pGH[256];
	int** pCH = p_temporaryCH;

	for(i=0;i<dimy;i++)
		for(j=0;j<dimx;j++)
			pCH[i][j] = 0;

	for(i=0; i<dim_gray; i++)
		pGH[i] = 0;


	float n = (float)(SearchRegion->y_max - SearchRegion->y_min + 1)*(SearchRegion->x_max - SearchRegion->x_min + 1);


	unsigned char** p_color_data = p_InputColorImage->p_Image2D;
	unsigned char hsi[3];
	
	
	for(i = SearchRegion->y_min; i <= SearchRegion->y_max; i++)
	 for(j = SearchRegion->x_min; j <= SearchRegion->x_max; j++)
	 {
		r = p_color_data[i][3*j];
		g = p_color_data[i][3*j+1];
		b = p_color_data[i][3*j+2];
		intensity = (r+g+b+1)/3;
		if(intensity <= intensity_tolerance)
			pGH[0]++;
		else if(r*r + g*g + b*b - 3*intensity*intensity <= t2)
			pGH[(dim_gray*intensity)/256]++;
		else
		{
			RGB2HSI(&(p_color_data[i][3*j]), hsi);
			AssignColor2CompleteHistogramSimple(hsi[0], hsi[1], pCH, dimx, dimy);
		}
	 }


	for(i=0;i<dimy;i++)
	for(j=0;j<dimx;j++)
		pNCH[i][j] = pCH[i][j]/n;
	
	for(i=0;i<dim_gray;i++)
		pNGH[i] = pGH[i]/n;


	for(i=0;i<dimy;i++)
	{
	 for(j=0;j<dimx;j++)
	 {
	  fprintf(pFile1,"%4d ", pCH[i][j]);
	 }
	 fprintf(pFile1,"\n");
	}

	for(i=0;i<dim3;i++)
		fprintf(pFile1,"%4d", pGH[i]);

	fprintf(pFile1,"\n");

//	fclose(pFile1);

}

void CColorHistogram::CreateGrayColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, int** pCH, int* pGH, int dimx, int dimy, int dim_gray)
{
	int i, j;
	unsigned char** p_color_data = p_InputColorImage->p_Image2D;
	
	for(i = SearchRegion->y_min; i <= SearchRegion->y_max; i++)
	 for(j = SearchRegion->x_min; j <= SearchRegion->x_max; j++)
		AssignColor2CompleteHistogram(p_color_data[i][3*j], p_color_data[i][3*j+1], pCH, pGH, dimx, dimy, dim_gray);

	for(i=0;i<dimy;i++)
	{
	 for(j=0;j<dimx;j++)
	 {
	  fprintf(pFile1,"%4d ", pCH[i][j]);
	 }
	 fprintf(pFile1,"\n \n");
	}
	fclose(pFile1);

}

void CColorHistogram::AssignColor2CompleteHistogram(unsigned char x, unsigned char y, int** pCH, int histogram_size)
{
//	x, y  - collors to be assigned
//	pCH		- complete histogram
//	
	int d,d1,d2,i1,j1,r1,r2;
	div_t	dv;
	d  = 256/histogram_size;
	d1 = d/2;	d2 = 256-d1;

	if(x<d1) x = d1;
	if(x>d2) x = d2;
	if(y<d1) y = d1;
	if(y>d2) y = d2;

	dv = div(x-d1,d);
	j1 = dv.quot;
	r1 = dv.rem;

	dv = div(y-d1,d);
	i1 = dv.quot;
	r2 = dv.rem;

	pCH[i1][j1]+= HistogramWeights[r2][r1][0];
	pCH[i1][j1+1]+= HistogramWeights[r2][r1][1];
	pCH[i1+1][j1]+= HistogramWeights[r2][r1][2];
	pCH[i1+1][j1+1]+= HistogramWeights[r2][r1][3];
}

void CColorHistogram::AssignColor2CompleteHistogramSimple(unsigned char x, unsigned char y, int** pCH, int dimx, int dimy)
{
//	x, y  - collors to be assigned
//	pCH		- complete histogram
//	
	int i1,j1;

	i1 = (y*dimy)/256;
	j1 = (x*dimx)/256;

	pCH[i1][j1]+= 1;
}

void CColorHistogram::AssignColor2CompleteHistogram(unsigned char x, unsigned char y, int** pCH, int* pGH, int dimx, int dimy, int dim_gray)
{
//	x, y  - collors to be assigned
//	pCH		- complete histogram
//	
	int i1,j1;

	i1 = (y*dimy)/256;
	j1 = (x*dimx)/256;

	pCH[i1][j1]+= 1;
}



void CColorHistogram::HistogramUpdate(Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion)
{
	CreateCompositeHSIHistogram(p_InputColorImage, HistogramRegion, pNormalizedHistogram, pNormalizedGrayHistogram, dim1, dim2, dim3);
					//	CreateColorHistogram(p_InputColorImage, HistogramRegion, pCompleteHistogram, pNormalizedHistogram, dim1, dim2);
	
	n_histogram_colors = CreateHistogramVector(pNormalizedHistogram, pNormalizedGrayHistogram, p_histogram_colors, p_gray_levels, n_histogram_colors = 15, n_gray_levels, dim1, dim2, dim3);
}














































// Funtions that create Palette Image


void CColorHistogram::CreatePaletteImage(Image<unsigned char>* p_InputColorImage, int** p_histogram_colors, int n_histogram_colors, unsigned char** pPaletteImage)
{
	int i,j,k,u,v,x,y;
	unsigned char** p_color_data = p_InputColorImage->p_Image2D;

	//	Compute intervals for the dominant histogram colors. In the particular case (16 levels)
	//	color corresponding to (i,j) = (16*i -- 16*(i+1), 16*j -- 16*(j+1))

	int h_min[50][2], h_max[50][2];

	for(i=0; i<n_histogram_colors; i++)
	{
		h_min[i][0] = 16*p_histogram_colors[i][0];
		h_min[i][1] = 16*p_histogram_colors[i][1];
		h_max[i][0] = 16*p_histogram_colors[i][0]+16;
		h_max[i][1] = 16*p_histogram_colors[i][1]+16;
	}


	y = p_InputColorImage->m_GetCols();
	x = p_InputColorImage->m_GetRows()/3;

	for(i=0; i<y; i++)
	 for(j=0; j<x; j++)
	 {
		 u = p_color_data[i][3*j];
		 v = p_color_data[i][3*j+1];

		 for(k=0; k<n_histogram_colors; k++)
		  if(u>=h_min[k][0] && u<h_max[k][0] && v>=h_min[k][1] && v<h_max[k][1])
			pPaletteImage[i][j] = k;
		  else
			pPaletteImage[i][j] = n_histogram_colors;
	  }
}


void CColorHistogram::CreateCompositeHSIPaletteImage(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, int** p_colors, int n_colors, int* p_gray_levels, int n_gray_levels, unsigned char** pPaletteImage)
{
	static a = 1;
	int i,j;
	int i1 = 256/dim2;
	int j1 = 256/dim1;
	int r, g, b, t2 = gray_tolerance*gray_tolerance;
	int intensity;
	float n = (float)(SearchRegion->y_max - SearchRegion->y_min + 1)*(SearchRegion->x_max - SearchRegion->x_min + 1);

	unsigned char** p_color_data = p_InputColorImage->p_Image2D;
	unsigned char hsi[3];
	
	
	for(i = SearchRegion->y_min; i <= SearchRegion->y_max; i++)
	 for(j = SearchRegion->x_min; j <= SearchRegion->x_max; j++)
	 {
		r = p_color_data[i][3*j];
		g = p_color_data[i][3*j+1];
		b = p_color_data[i][3*j+2];
		intensity = (r+g+b+1)/3;
		if(intensity <= intensity_tolerance)
			pPaletteImage[i][j] = p_gray_LUT[0];
		else if(r*r + g*g + b*b - 3*intensity*intensity <= t2)
			pPaletteImage[i][j] = p_gray_LUT[(dim3*intensity)/256];
		else
		{
			RGB2HSI(&(p_color_data[i][3*j]), hsi);
			pPaletteImage[i][j] = p_hist_LUT[(hsi[1]*dim2)/256][(hsi[0]*dim1)/256];
		}
	 }

}



























// Histogram Vector

int CColorHistogram::CreateHistogramVector(float** p_NormalizedHistogram, float* pGrayHistogram, int** colors, int* gray_levels, int n_histogram_colors, int n_gray_levels, int dimx, int dimy, int dim_gray)
{
	int i, j, k, k1, k2, i1, j1;
	float temp_hist[32][32], temp_gray[32], hg[32];
	float Max = 0;

	for(i=0;i<dimy;i++)
	 for(j=0;j<dimx;j++)
	  temp_hist[i][j] = p_NormalizedHistogram[i][j];

	 for(i=0; i<dim_gray; i++)
		 temp_gray[i] = pGrayHistogram[i];

	k = k1 = k2 = 0;

	while(k<n_histogram_colors)
	{
     Max = 0;
	 for(i=0;i<dimy;i++)
	 for(j=0;j<dimx;j++)
	  if(temp_hist[i][j] > Max)
	  {
		Max = temp_hist[i][j];
		i1 = i; j1 = j;
	  }

	 for(i=0; i<dim_gray; i++)
	  if(temp_gray[i] > Max)
	  {
		Max = temp_gray[i];
		i1	= i;
		j1	= 32;
	  }

	 if(Max == 0)
	 {
	   n_histogram_colors = k;
	   break;
	 }

	 if(j1<32)
	 {
	   histogram_vector[k1]	= Max;
	   colors[k1][0]		= i1;
	   colors[k1][1]		= j1;
	   temp_hist[i1][j1]	= 0;
	   k1++;
	 }
	 else
	 {
	   hg[k2]			= Max;
	   gray_levels[k2]	= i1;
	   temp_gray[i1]	= 0;
	   k2++;
	 }
	 k++;
	}

	n_histogram_colors	= k1;
	n_gray_levels		= k2;


// Create histogram_vector and LUT's
	for(i=0;i<k2;i++)
		histogram_vector[k1+i] = hg[i];

	for(i=0;i<k1;i++)
		p_hist_LUT[colors[i][0]][colors[i][1]] = i+1;
	
	for(i=0;i<k2;i++)
		p_gray_LUT[gray_levels[i]] = i+k1+1;



//	Debugging
	float sum = 0;

	for(i=0;i<4;i++)
	{
		for(j=0;j<32;j++)
			fprintf(pFile1,"%4d", p_hist_LUT[i][j]);
		fprintf(pFile1,"\n");
	}

	for(i=0;i<4;i++)
		fprintf(pFile1,"%4d", p_gray_LUT[i]);
	fprintf(pFile1,"\n");


	for(i=0;i<k1;i++)
	{
	  fprintf(pFile1,"%8.6f  %4d %4d \n", histogram_vector[i], colors[i][0], colors[i][1]);
	}

	for(i=0;i<k2;i++)
	{
	  fprintf(pFile1,"%8.6f  %4d \n", hg[i], gray_levels[i]);
	}

	for(i=0;i<k;i++)sum+=histogram_vector[i];
	fprintf(pFile1,"%8.6f ", sum);



	fclose(pFile1);


	return n_histogram_colors;
}


void CColorHistogram::HistogramMatching(unsigned char** pPaletteImage, tRectangle* Target, tRectangle* SearchRegion, int* p, int* BestMatch)
{
	int i,j,x1,x2,y1,y2,k1,k2,dx,dy;
	unsigned char** p1 = pPaletteImage;
	x1 = SearchRegion->x_min;
	x2 = SearchRegion->x_max;
	y1 = SearchRegion->y_min;
	y2 = SearchRegion->y_max;

	dx = (Target->x_max - Target->x_min);
	dy = (Target->y_max - Target->y_min);

	int a1 = dx/2;
	int a2 = dx - a1;
	int b1 = dy/2;
	int b2 = dy - b1;
	int N = (dx+1)*(dy+1);
	int hv[50];
	hv[0]=0;
	for(i=0;i<15;i++)
		hv[i+1]=int(N*histogram_vector[i]+.5);


/*	x1 = __max(x1, a1+5);
	x2 = __min(x2, width-a1-5);
	y1 = __max(y1, b1+5);
	y2 = __min(y2, height-b1-5);
*/
	x1+=a1;
	x2-=a2;
	y1+=b1;
	y2-=b2;

	PalInt** a = pPTA;
	memset((void*)a[0], 0, 240*320*sizeof(PalInt));
	PalInt** p2 = pPTB;

	for(i=y1-b1; i<=y2+b2; i++)
	{

		for(j= -a1; j<=a2; j++)
			a[i][x1].p[p1[i][x1+j]]++;

		for(j=x1; j<x2; j++)
		{
			CopyVector(a[i][j].p, a[i][j+1].p, 16);
			k1 = p1[i][a2+j+1];
			k2 = p1[i][j-a1];
			a[i][j+1].p[k1]++;
			a[i][j+1].p[k2]--;
		}
	}

	for(j=x1;j<=x2;j++)
	{
		CopyVector(a[y1-b1][j].p, p2[y1][j].p, 16);
		for(i=-b1+1;i<=b2;i++)
			AddVector(p2[y1][j].p,a[y1+i][j].p, 16);

		for(i=y1;i<y2;i++)
		{
			CopyVector(p2[i][j].p, p2[i+1][j].p, 16);
			AddVector(p2[i+1][j].p, a[i+b2+1][j].p, 16);
			SubtractVector(p2[i+1][j].p, a[i-b1][j].p, 16);
		}
	}

	int i1 = 0, j1 = 0, Max = 0, Match;

	for(i=y1;i<=y2;i++)
	for(j=x1;j<=x2;j++)
	{
		Match = VectorIntersection(hv, p2[i][j].p, 16);
		if(Match>Max)
		{
			Max = Match;
			i1 = i;
			j1 = j;
		}
	}

	p[0] = j1;
	p[1] = i1;
	*BestMatch = Max;
	Target->x_min = p[0]-a1;
	Target->y_min = p[1]-b1;
	Target->x_max = p[0]+a2;
	Target->y_max = p[1]+b2;
}

void CColorHistogram::HistogramMatching(Image<unsigned char>* p_InputColorImage, tRectangle* Target, tRectangle* SearchRegion, int* p, float* BestMatch)
{
	int N, nBestMatch;
	CreateCompositeHSIPaletteImage(p_InputColorImage, SearchRegion, p_histogram_colors, n_histogram_colors, p_gray_levels, n_gray_levels, pPaletteImage);
	HistogramMatching(pPaletteImage, Target, SearchRegion, p, &nBestMatch);
	N = (Target->x_max - Target->x_min + 1)*(Target->y_max - Target->y_min + 1);
	*BestMatch = nBestMatch/(float)N;

}


void CColorHistogram::CopyVector(int* source, int* dest, int size)
{
	int i;
	for(i=0;i<size;i++)
		dest[i] = source[i];
}



void CColorHistogram::AddVector(int* a, int* b, int size)	 // a = a + b;
{
	int i;
	for(i=0;i<size;i++)
		a[i]+= b[i];
}


void CColorHistogram::SubtractVector(int*a, int*b, int size) // a = a - b;
{
	int i;
	for(i=0;i<size;i++)
		a[i]-= b[i];
}


void CColorHistogram::AddVectors(int* source1, int* source2, int* dest, int size)
{
	int i;
	for(i=0;i<size;i++)
		dest[i] = source1[i] + source2[i];
}


int CColorHistogram::VectorIntersection(int* vec1, int* vec2, int n)
{
	int i, ret = 0;
	for(i=0;i<n;i++)
		ret+= __min(vec1[i], vec2[i]);

	return ret;
}




float CColorHistogram::CompareHistograms(float** pNCH0, Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion)
{
	int i, j;
	float sim = 0.0f;
	if(HistogramRegion->x_max == HistogramRegion->x_min || HistogramRegion->y_max == HistogramRegion->y_min)
		return 0.0f;

	CreateHSIColorHistogram(p_InputColorImage, HistogramRegion, p_temporaryCH, p_temporaryNCH, dim1, dim2);
	
	for(i=0; i<dim2; i++)
	for(j=0; j<dim1; j++)
	{
		sim+= __min(pNCH0[i][j], p_temporaryNCH[i][j]);
	}

	return sim;
}


float CColorHistogram::CompareCompositeHistograms(Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion)
{
	int i, j;
	float sim = 0.0f;
	
	if(HistogramRegion->x_max == HistogramRegion->x_min || HistogramRegion->y_max == HistogramRegion->y_min)
		return 0.0f;

	for(i=0;i<dim3;i++) p_temporaryGH[i] = 0;

	CreateCompositeHSIHistogram(p_InputColorImage, HistogramRegion, p_temporaryNCH, p_temporaryGH, dim1, dim2, dim3);
	
	for(i=0; i<dim2; i++)
	for(j=0; j<dim1; j++)
	{
		sim+= __min(pNormalizedHistogram[i][j], p_temporaryNCH[i][j]);
	}

	for(i=0;i<dim3;i++)
		sim+= __min(pNormalizedGrayHistogram[i], p_temporaryGH[i]);

	return sim;
}



void CColorHistogram::SortHistogram(int **p_histogram_colors, OrdInt* hist_colors, int histogram_size)
{
	int i, j, d = histogram_size;
	OrdInt* ptr = &(hist_colors[0]);
 	for(i=0;i<d;i++)
	 for(j=0;j<d;j++)
	 {
	   ptr->value = p_histogram_colors[i][j];
	   ptr->i = i;
	   ptr->j = j;
	   ptr++;
	 }

	qsort((void*)hist_colors, d*d, sizeof(int), compare);
}

int compare( const void *arg1, const void *arg2 )
{
	OrdInt	s1, s2, *ps1, *ps2;
	ps1 = &s1; 
	ps2 = &s2;
	ps1 = (OrdInt*)arg1;
	ps2 = (OrdInt*)arg2;
	return ps1->value - ps2->value;
}

void RGB2HSI(unsigned char* RGB, unsigned char* HSI)
{
	unsigned char *ptr, r, g, b;
	int rg, rb, min_rgb;
	float i, s, h;
	ptr = RGB;
	r = *ptr++;
	g = *ptr++;
	b = *ptr;
	rg = r - g;
	rb = r - b;
	i = float(r+g+b);
	min_rgb = __min(__min(r,g), b);
	s = 256 - (768*min_rgb)/i;
	h = (float)acos( (rg + rb)/(2 * sqrt( rg*rg +rb*(g-b) ) ) );
	if(b > g)
		h = float(2*3.1415926535897323 - h);

	h = float(128*h/3.1415926535897323);
	i = i/3;
	HSI[0] = (unsigned char)(h+.5); 
	HSI[1] = (unsigned char)(s+.5); 
	HSI[2] = (unsigned char)(i+.5);
}


PalInt** alloc2Dpalint(int x, int y)
{
   PalInt **ary;
   int i;
   ary = new PalInt *[y];
   ary[0] = new PalInt [x*y];
   
   if ((ary==0)||(ary[0]==0)) return NULL;

   for (i=y-1;i>=0;i--)
      ary[i] = &ary[0][i*x];
   
   memset((void*)ary[0], 0, x*y*sizeof(PalInt));
   return ary;
}















//	MMX related histogram functions
void CColorHistogram::sHistogramMatching(Image<unsigned char>* p_InputColorImage, tRectangle* Target, tRectangle* SearchRegion, int* p, float* BestMatch, unsigned char** mask)
{
	int i, N, nBestMatch, wt = (Target->x_max - Target->x_min + 1), ht = (Target->y_max - Target->y_min + 1);
	short hv[50];
	N = wt*ht;
	memset((void*)hv, 0, 50*sizeof(short));

	for(i=0;i<15;i++)
		hv[i+1]=short(N*histogram_vector[i]+.5);

	CreateCompositeHSIPaletteImage(p_InputColorImage, SearchRegion, p_histogram_colors, n_histogram_colors, p_gray_levels, n_gray_levels, pPaletteImage);
//	HistogramMatching(pPaletteImage, Target, SearchRegion, p, &nBestMatch);
	sHistogramMatching(pPaletteImage, SearchRegion, wt, ht,  hv, p, &nBestMatch, mask);
	Target->x_min = p[0];
	Target->y_min = p[1];
	Target->x_max = Target->x_min + wt - 1;
	Target->y_max = Target->y_min + ht - 1;

	*BestMatch = nBestMatch/(float)N;

}

void CColorHistogram::sHistogramMatching(unsigned char** pPaletteImage, tRectangle* SearchRegion, int wt, int ht, short* hv, int* best_match_coordinates, int* BestMatch, unsigned char** mask)
{
	tRectangle nSR;
	int wx = 20, wy = 20, thr = 80, xc[100], yc[100], val[100], Nc;
	memset(xc, 0, 100*sizeof(int));
	memset(yc, 0, 100*sizeof(int));
	memset(val, 0, 100*sizeof(int));

//	Timer timer_ver, timer, timermmx;

//	timer_ver.start();
	VerticalPass(pPaletteImage, SearchRegion, wt, ht, v1);
//	timer_ver.stop();

//	timer.start();
//	HorizontalPass(v1, SearchRegion, wt, ht, v2, &nSR);
//	timer.stop();

//	timermmx.start();
	HorizontalPassMMX(v1, SearchRegion, wt, ht, v2, &nSR);
//	timermmx.stop();

	FindBestMatchMMX(v2, hv, SearchRegion, best_match_coordinates, BestMatch, mask);
	sFindMaxima(hist_match[0], (*BestMatch/255) + 1, tmp_peaks, width, height, wx, wy, thr, xc, yc, val, &Nc);

	FILE* Fcand	 = fopen("e:\\users\\mdt\\cand.txt", "at");
	fprintf(Fcand, "%4d \n", Nc);
	for(int i=0; i<Nc;i++)
		fprintf(Fcand, "%4d  %4d  %6d \n", xc[i], yc[i], val[i]);

	fclose(Fcand);
//	cout << "vertical pass = " << timer_ver.duration() << endl;
//	cout << "horizontal pass = " << timer.duration() << endl;
//	cout << "horizontal pass mmx = " << timermmx.duration() << endl;
/*	TRACE("vertical pass =  %f \n", timer_ver.duration());
	TRACE("horizontal pass =  %f \n", timer.duration());
	TRACE("horizontal pass mmx=  %f \n", timermmx.duration());
*/
}

void CColorHistogram::sHistogramMatching(Image<unsigned char>* p_InputColorImage, tRectangle* Target, tRectangle* SearchRegion, int* p, float* BestMatch, Candidate* pCand, unsigned char** mask, tRectangle* SR0)
{
	int i, N, nBestMatch, wt = (Target->x_max - Target->x_min + 1), ht = (Target->y_max - Target->y_min + 1);
	short hv[50];
	N = wt*ht;
	memset((void*)hv, 0, 50*sizeof(short));

	for(i=0;i<15;i++)
		hv[i+1]=short(N*histogram_vector[i]+.5);

	CreateCompositeHSIPaletteImage(p_InputColorImage, SearchRegion, p_histogram_colors, n_histogram_colors, p_gray_levels, n_gray_levels, pPaletteImage);
//	HistogramMatching(pPaletteImage, Target, SearchRegion, p, &nBestMatch);
	sHistogramMatching(pPaletteImage, SearchRegion, wt, ht,  hv, p, &nBestMatch, pCand, mask, SR0);
	Target->x_min = p[0];
	Target->y_min = p[1];
	Target->x_max = Target->x_min + wt - 1;
	Target->y_max = Target->y_min + ht - 1;

	*BestMatch = nBestMatch/(float)N;

}

void CColorHistogram::sHistogramMatching(unsigned char** pPaletteImage, tRectangle* SearchRegion, 
						int wt, int ht, short* hv, int* best_match_coordinates, int* BestMatch, 
						Candidate* pCand, unsigned char** mask, tRectangle* SR0)
{
	tRectangle nSR;
	int i, ind, wx = 20, wy = 20, thr = 80, xc[100], yc[100], val[100], Nc;
	memset(xc, 0, 100*sizeof(int));
	memset(yc, 0, 100*sizeof(int));
	memset(val, 0, 100*sizeof(int));

//	Timer timer_ver, timer, timermmx;

	if(SR0)
	{
		VerticalPass(pPaletteImage, SR0, wt, ht, v1);
		HorizontalPassMMX(v1, SR0, wt, ht, v2, &nSR);
	}

	VerticalPass(pPaletteImage, SearchRegion, wt, ht, v1);
	HorizontalPassMMX(v1, SearchRegion, wt, ht, v2, &nSR);

//	The values obtained by FindBestMatch are offset for -(Torso.width/2, Torso.height/2)

	FindBestMatchMMX(v2, hv, SearchRegion, best_match_coordinates, BestMatch, mask);
/*	sFindMaxima(hist_match[0], (*BestMatch/255) + 1, tmp_peaks, width, height, wx, wy, thr, xc, yc, val, &Nc);

	ind = pCand->FindBestPrediction(xc, yc, val, Nc);
	*BestMatch = val[ind];
	best_match_coordinates[0] = xc[ind];
	best_match_coordinates[1] = yc[ind];

	FILE* Fcand	 = fopen("e:\\users\\mdt\\cand.txt", "at");
	fprintf(Fcand, "%4d \n", Nc);
	fprintf(Fcand, "%4d %4d %4d %4d \n", pCand->x[4], pCand->y[4], pCand->x_predicted, pCand->y_predicted);
	for(i=0; i<Nc;i++)
		fprintf(Fcand, "%4d  %4d  %6d \n", xc[i], yc[i], val[i]);
*/
//	cout << "vertical pass = " << timer_ver.duration() << endl;
//	cout << "horizontal pass = " << timer.duration() << endl;
//	cout << "horizontal pass mmx = " << timermmx.duration() << endl;
/*	TRACE("vertical pass =  %f \n", timer_ver.duration());
	TRACE("horizontal pass =  %f \n", timer.duration());
	TRACE("horizontal pass mmx=  %f \n", timermmx.duration());
*/
}


void CColorHistogram::VerticalPass(unsigned char** pPaletteImage, tRectangle* SearchRegion, int wt, int ht, short** a)
{
	int i,j,x1,x2,y1,y2,k1,k2,dx,dy;
	unsigned char** p1 = pPaletteImage;
	short* a_tmp;
	x1 = SearchRegion->x_min;
	x2 = SearchRegion->x_max;
	y1 = SearchRegion->y_min;
	y2 = SearchRegion->y_max;

	dx = wt;
	dy = ht;

	memset((void*)a[0], 0, 240*320*16*sizeof(short));

	for(j=x1; j<=x2; j++)
	{
		a_tmp = a[y1]+16*j;
		for(i= y1; i<y1+ht; i++)
			a_tmp[ p1[i][j] ]++;

		for(i=y1; i<y2-ht; i++)
		{
			a_tmp = a[i+1]+16*j;
			CopyVector(a[i]+16*j, a_tmp, 16);
			k1 = p1[i+ht][j];
			k2 = p1[i][j];
			a_tmp[k1]++;
			a_tmp[k2]--;
		}
	}

}


void CColorHistogram::HorizontalPass(short** v1, tRectangle* SearchRegion, int wt, int ht, short** v2, tRectangle* newSearchRegion)
{
	int i,j,x1,x2,y1,y2,dx,dy;
	short* v_src, *v_dest, *v_back;
	x1 = SearchRegion->x_min;
	x2 = SearchRegion->x_max;
	y1 = SearchRegion->y_min;
	y2 = SearchRegion->y_max;

	dx = wt;
	dy = ht;

	for(i=y1; i<=y2; i++)
	{
		v_back = v_src = v1[i] + 16*x1;
		v_dest = v2[i]+16*x1;
		CopyVector(v_src, v_dest, 16);
		v_src += 16;
		for(j=1; j<wt; j++, v_src += 16)
			AddVector(v_dest, v_src, 16);

		for(j=0; j<x2-x1+1-wt; j++)
		{
			CopyVector(v_dest, v_dest+16, 16);
			v_dest += 16;
			AddVector(v_dest, v_src, 16);
			SubtractVector(v_dest, v_back, 16);
			v_back += 16;
			v_src  += 16;
		}
	}

}

void CColorHistogram::HorizontalPassMMX(short** v1, tRectangle* SearchRegion, int wt, int ht, short** v2, tRectangle* newSearchRegion)
{
	int x1 = SearchRegion->x_min;
	int x2 = SearchRegion->x_max;
	int y1 = SearchRegion->y_min;
	int y2 = SearchRegion->y_max;
	// Vectors are aligned to 8-byte boundaries already
	// Vector sizes are assumed to be 16 shorts
	unsigned char * src = reinterpret_cast<unsigned char *> (v1[y1] + 16*x1);
	unsigned char * dest = reinterpret_cast<unsigned char *> (v2[y1] + 16*x1);

	int nrows = (y2 - y1 + 1) - (ht - 1);
	int addsubcount = __max(1,x2 - x1 + 1 - wt);
	int offsetrowsrc = (width - (x2 - x1 + 1))*16*2;
	int offsetrowdest = (width - (x2 - x1 + 1 - (wt - 1)))*16*2;

	__asm {
		// Push stuff
		push ecx
		push edx
		push ebx
		push esi
		push edi
		// Pointer to source vector
		mov esi, src
		// Pointer to destination vector
		mov edi, dest

		mov edx, nrows

core_loop:
		mov ecx, esi	// Store value of esi for later

		pxor mm0, mm0
		pxor mm1, mm1
		pxor mm2, mm2
		pxor mm3, mm3

		mov ebx, wt

add_loop:
		
		paddusw mm0, [esi]
		paddusw mm1, [esi+8]
		paddusw mm2, [esi+16]
		paddusw mm3, [esi+24]

		add esi, 32

		dec ebx
		jnz add_loop

		movq [edi], mm0
		movq [edi+8], mm1
		movq [edi+16], mm2
		movq [edi+24], mm3

		add edi, 32

		mov ebx, addsubcount
		// Now add once and subtract once the rest of the way
add_subtract_loop:
		// esi points to the vector to be added, while ecx points to the vector to be
		// subtracted
		psubusw mm0, [ecx]
		psubusw mm1, [ecx+8]
		psubusw mm2, [ecx+16]
		psubusw mm3, [ecx+24]

		add ecx, 32

		paddusw mm0, [esi]
		paddusw mm1, [esi+8]
		paddusw mm2, [esi+16]
		paddusw mm3, [esi+24]

		add esi, 32

		movq [edi], mm0
		movq [edi+8], mm1
		movq [edi+16], mm2
		movq [edi+24], mm3

		add edi, 32

		dec ebx
		jnz add_subtract_loop

		// Jump over to the next row within the search region
		add esi, offsetrowsrc
		add edi, offsetrowdest
		dec edx
		jnz core_loop
		
		emms

		// Pop stuff back
		pop edx
		pop ecx
		pop ebx
		pop esi
		pop edi
	}

}


void CColorHistogram::FindBestMatch(short** v2, short* hv, tRectangle* SearchRegion, int *p, int* BestMatch)
{
	int i,j,x1,x2,y1,y2;
	x1 = SearchRegion->x_min;
	x2 = SearchRegion->x_max;
	y1 = SearchRegion->y_min;
	y2 = SearchRegion->y_max;
	int i1 = 0, j1 = 0, Max = 0, Match, Min = 0;

	memset((void*)hist_match[0], 0, width*height*sizeof(short));
	for(i=y1;i<=y2;i++)
	for(j=x1;j<=x2;j++)
	{
		Match = hist_match[i][j] = VectorIntersection(hv, v2[i]+16*j, 16);
		if(Match<Min)
			Min = Match;
		if(Match>Max)
		{
			Max = Match;
			i1 = i;
			j1 = j;
		}
	}

	p[0] = j1;
	p[1] = i1;
	*BestMatch = Max;
}

void CColorHistogram::FindBestMatchMMX(short** v2, short* hv, tRectangle* SearchRegion, int *p, int* BestMatch)
{
	int i,j,x1,x2,y1,y2;
	x1 = SearchRegion->x_min;
	x2 = SearchRegion->x_max;
	y1 = SearchRegion->y_min;
	y2 = SearchRegion->y_max;
	int i1 = 0, j1 = 0, Max = 0, Match, Min = 0;
	int dx = (x2-x1+1)/2, dy = (y2 - y1 +1 )/2;


	memset((void*)hist_match[0], 0, width*height*sizeof(short));
	// Load hv into mm0-mm3
	__asm	{
		push esi
		push edi
		mov esi, hv
		movq mm0, [esi]
		movq mm1, [esi+8]
		movq mm2, [esi+16]
		movq mm3, [esi+24]
	}

	for(i=y1;i<=y2;i++)	{
		unsigned char * src = reinterpret_cast<unsigned char *> (v2[i] + 16*x1);
		for(j=x1;j<=x2;j++)	{
			//		Match = hist_match[i][j] = VectorIntersection(hv, v2[i]+16*j, 16);
			//		unsigned char * src = reinterpret_cast<unsigned char *> (v2[i]+16*j);
			__asm	{
				mov edi, src
				// Load v2 into mm4
				movq mm4, [edi]
				// Find the min of mm0 and mm4
				movq mm7, mm0	// mm7 is going to act as the mask for mm0 > mm4
				pcmpgtw mm7, mm4

				pand mm4, mm7	// mm4 now holds the lower values of mm4
				pandn mm7, mm0	// mm7 now holds the lower values of mm0

				por mm4, mm7	// mm4 holds all the lower values of mm0 and mm4

				// Load next word of v2 into mm5
				movq mm5, [edi+8]
				// Find the min of mm1 and mm5
				movq mm7, mm1	// mm7 is going to act as the mask for mm1 > mm5
				pcmpgtw mm7, mm5

				pand mm5, mm7	// mm5 now holds the lower values of mm5
				pandn mm7, mm1	// mm7 now holds the lower values of mm1

				por mm5, mm7	// mm5 holds all the lower values of mm1 and mm5

				// Add mm4 and mm5
				paddsw mm4, mm5	// Free up mm5

				// Load next word of v2 into mm6
				movq mm6, [edi+16]
				// Find the min of mm2 and mm6
				movq mm7, mm2	// mm7 is going to act as the mask for mm2 > mm6
				pcmpgtw mm7, mm6

				pand mm6, mm7	// mm6 now holds the lower values of mm6
				pandn mm7, mm2	// mm7 now holds the lower values of mm2

				por mm6, mm7	// mm6 holds all the lower values of mm2 and mm6


				// Load next word of v2 into mm5
				movq mm5, [edi+24]
				// Find the min of mm3 and mm5
				movq mm7, mm3	// mm7 is going to act as the mask for mm3 > mm5
				pcmpgtw mm7, mm5

				pand mm5, mm7	// mm5 now holds the lower values of mm5
				pandn mm7, mm3	// mm7 now holds the lower values of mm3

				por mm5, mm7	// mm5 holds all the lower values of mm2 and mm5

				// Add mm6 and mm5
				paddsw mm6, mm5

				paddsw mm4, mm6	// mm4 contains all the additions
				// Now sum up words of mm4
				movq mm5, mm4	// Copy mm4
				pxor mm7, mm7	// For use in unpacking
				// Unpack mm4
				punpckhwd mm4, mm7	// High word unpack
				punpcklwd mm5, mm7	// Low word unpack
				paddd mm4, mm5	// mm4 has 2 dwords that still need to be added
				movq mm5, mm4	// Make a copy first
				psrlq mm5, 32	// Shift a dword
				paddd mm4, mm5	// Transfer 32 bits into the result match

				movd Match, mm4

				add src, 32
			}

			hist_match[i][j] = Match;
			if(Match<Min)
				Min = Match;
			if(Match>Max)
			{
				Max = Match;
				i1 = i;
				j1 = j;
			}
		}
	}

	p[0] = j1;
	p[1] = i1;
	*BestMatch = Max;

	__asm	{
		pop esi
		pop edi
		emms
	}
}


void CColorHistogram::FindBestMatch(short** v2, short* hv, tRectangle* SearchRegion, int *p, int* BestMatch, unsigned char** mask)
{
	if(mask == 0)
	{
		FindBestMatch(v2, hv, SearchRegion, p, BestMatch);
		return;
	}

	int i,j,x1,x2,y1,y2;
	x1 = SearchRegion->x_min;
	x2 = SearchRegion->x_max;
	y1 = SearchRegion->y_min;
	y2 = SearchRegion->y_max;
	int i1 = 0, j1 = 0, Max = 0, Match, Min = 0;

	memset((void*)hist_match[0], 0, width*height*sizeof(short));
	for(i=y1;i<=y2;i++)
	for(j=x1;j<=x2;j++)
	if(mask[i][j])
	{
		Match = hist_match[i][j] = VectorIntersection(hv, v2[i]+16*j, 16);
		if(Match<Min)
			Min = Match;
		if(Match>Max)
		{
			Max = Match;
			i1 = i;
			j1 = j;
		}
	}

	p[0] = j1;
	p[1] = i1;
	*BestMatch = Max;
}


void CColorHistogram::FindBestMatchMMX(short** v2, short* hv, tRectangle* SearchRegion, int *p, int* BestMatch, unsigned char** mask)
{
	if(mask == 0)
	{
		FindBestMatchMMX(v2, hv, SearchRegion, p, BestMatch);
		return;
	}

	int i,j,x1,x2,y1,y2;
	x1 = SearchRegion->x_min;
	x2 = SearchRegion->x_max;
	y1 = SearchRegion->y_min;
	y2 = SearchRegion->y_max;
	int i1 = 0, j1 = 0, Max = 0, Match, Min = 0;
	int dx = (x2-x1+1)/2, dy = (y2 - y1 +1 )/2;


	memset((void*)hist_match[0], 0, width*height*sizeof(short));
	// Load hv into mm0-mm3
	__asm	{
		push esi
		push edi
		mov esi, hv
		movq mm0, [esi]
		movq mm1, [esi+8]
		movq mm2, [esi+16]
		movq mm3, [esi+24]
	}

	for(i=y1;i<=y2;i++)	
	for(j=x1;j<=x2;j++)	
	if(mask[i][j])
	{
		unsigned char * src = reinterpret_cast<unsigned char *> (v2[i] + 16*j);

			//		Match = hist_match[i][j] = VectorIntersection(hv, v2[i]+16*j, 16);
			//		unsigned char * src = reinterpret_cast<unsigned char *> (v2[i]+16*j);
			__asm	{
				mov edi, src
				// Load v2 into mm4
				movq mm4, [edi]
				// Find the min of mm0 and mm4
				movq mm7, mm0	// mm7 is going to act as the mask for mm0 > mm4
				pcmpgtw mm7, mm4

				pand mm4, mm7	// mm4 now holds the lower values of mm4
				pandn mm7, mm0	// mm7 now holds the lower values of mm0

				por mm4, mm7	// mm4 holds all the lower values of mm0 and mm4

				// Load next word of v2 into mm5
				movq mm5, [edi+8]
				// Find the min of mm1 and mm5
				movq mm7, mm1	// mm7 is going to act as the mask for mm1 > mm5
				pcmpgtw mm7, mm5

				pand mm5, mm7	// mm5 now holds the lower values of mm5
				pandn mm7, mm1	// mm7 now holds the lower values of mm1

				por mm5, mm7	// mm5 holds all the lower values of mm1 and mm5

				// Add mm4 and mm5
				paddsw mm4, mm5	// Free up mm5

				// Load next word of v2 into mm6
				movq mm6, [edi+16]
				// Find the min of mm2 and mm6
				movq mm7, mm2	// mm7 is going to act as the mask for mm2 > mm6
				pcmpgtw mm7, mm6

				pand mm6, mm7	// mm6 now holds the lower values of mm6
				pandn mm7, mm2	// mm7 now holds the lower values of mm2

				por mm6, mm7	// mm6 holds all the lower values of mm2 and mm6


				// Load next word of v2 into mm5
				movq mm5, [edi+24]
				// Find the min of mm3 and mm5
				movq mm7, mm3	// mm7 is going to act as the mask for mm3 > mm5
				pcmpgtw mm7, mm5

				pand mm5, mm7	// mm5 now holds the lower values of mm5
				pandn mm7, mm3	// mm7 now holds the lower values of mm3

				por mm5, mm7	// mm5 holds all the lower values of mm2 and mm5

				// Add mm6 and mm5
				paddsw mm6, mm5

				paddsw mm4, mm6	// mm4 contains all the additions
				// Now sum up words of mm4
				movq mm5, mm4	// Copy mm4
				pxor mm7, mm7	// For use in unpacking
				// Unpack mm4
				punpckhwd mm4, mm7	// High word unpack
				punpcklwd mm5, mm7	// Low word unpack
				paddd mm4, mm5	// mm4 has 2 dwords that still need to be added
				movq mm5, mm4	// Make a copy first
				psrlq mm5, 32	// Shift a dword
				paddd mm4, mm5	// Transfer 32 bits into the result match

				movd Match, mm4

//				add src, 32
			}

		hist_match[i][j] = Match;
		if(Match<Min)
			Min = Match;
		if(Match>Max)
		{
			Max = Match;
			i1 = i;
			j1 = j;
		}
	}

	p[0] = j1;
	p[1] = i1;
	*BestMatch = Max;

	__asm	{
		pop esi
		pop edi
		emms
	}
}


void CColorHistogram::WriteHistMatch2File(char* fname, int N)
{
	FILE* pFile = fopen(fname, "wb");
	for(int i=0;i<width*height;i++)
		if(hist_match[0][i]>0)
		img_disp[0][i] = (unsigned char)((255*hist_match[0][i])/N);

	fwrite((void*)img_disp[0], sizeof(unsigned char), width*height, pFile);
	fclose(pFile);
}


void CColorHistogram::CopyVector(short* source, short* dest, int size)
{
	memcpy(dest, source, size*sizeof(short));
}


void CColorHistogram::AddVector(short* a, short* b, int size)	 // a = a + b;
{
	int i;
	for(i=0;i<size;i++)
		a[i]+= b[i];
}


void CColorHistogram::SubtractVector(short*a, short*b, int size) // a = a - b;
{
	int i;
	for(i=0;i<size;i++)
		a[i]-= b[i];
}


void CColorHistogram::AddVectors(short* source1, short* source2, short* dest, int size)
{
	int i;
	for(i=0;i<size;i++)
		dest[i] = source1[i] + source2[i];
}


short CColorHistogram::VectorIntersection(short vec1[], short vec2[], int n)
{
	short i, ret = 0;
	for(i=0;i<n;i++)
		ret+= __min(vec1[i], vec2[i]);

	return ret;
}