#include	<stdlib.h>
#include	<string.h>
#include	<math.h>
#include	"CornerClass.h"
#include	"brMemalloc.h"

#define __min(a,b) ((a)<(b))?(a):(b)

CCorner::CCorner(int width, int height, int algorithm, unsigned char* data)
{
	x = width;
	y = height;
	memset((void*)xc, 0, MAX_CORNERS*sizeof(int));
	memset((void*)yc, 0, MAX_CORNERS*sizeof(int));
	n_corner_type = algorithm;
	corner_map = alloc2Duchar(x,y,data);
	InitCornerDetector(n_corner_type);
}

void CCorner::InitCornerDetector(int algorithm)
{
	switch(algorithm)
	{
		case MIC: 
			pCMIC = new CMIC(x, y);
			break;
	}
}

void CCorner::DetectCorners(unsigned char** p_image, int* thresholds)
{
	image = p_image;
	memset((void*)corner_map[0], 0, x*y*sizeof(unsigned char));
	memset((void*)xc, 0, 1000*sizeof(int));
	memset((void*)yc, 0, 1000*sizeof(int));

	switch(n_corner_type)
	{
		case MIC: 
			pCMIC->fMIC(p_image, corner_map, xc, yc, thresholds);
			N = __min(pCMIC->GetN(), MAX_CORNERS);
			break;
	}

	
}

void CCorner::DetectCorners(unsigned char** p_image, unsigned char** p_binary_image, int scale, int* thresholds)
{

}

void CCorner::DetectCorners(unsigned char** p_image, int* thresholds, tRectangle* pRectangles, int N)
{

}

void CCorner::CreateCornerImage(unsigned char** p_image, unsigned char** corner_image)
{
	int i, j, k, tmp;
	unsigned char boja = 255, sm = 0, **tz = p_image, **sl = corner_image;

	for(k=0; k< N; k++)
	{
	  j = xc[k];
	  i = yc[k];

      tmp = tz[i-1][j-1]+tz[i-1][j]+tz[i-1][j+1]+
       		tz[i][j-1]+tz[i][j]+tz[i][j+1]+
            tz[i+1][j-1]+tz[i+1][j]+tz[i+1][j+1];
      
	  if(tmp>1500)
	  {
		  boja = 1;
		  sm = 255;
	  }
      else
	  {
		  boja = 255; 
		  sm=1;
	  }

	  sl[i-1][j-1] = sl[i-1][j] = sl[i-1][j+1] = sl[i][j-1] = sl[i][j+1] = sl[i+1][j-1] = 
	  sl[i+1][j] = sl[i+1][j+1] = boja;
	  sl[i][j] = sm;
	}
}

void CCorner::CreateColorCornerImage(unsigned char** corner_image)
{
	int i, j, j1, k;
	unsigned char **sl = corner_image;

	for(k=0; k< N; k++)
	{
	  j1 = xc[k];
	  i  = yc[k];

	  j = 3*j1;
	  sl[i-1][j-3] = sl[i-1][j] = sl[i-1][j+3] = sl[i][j-3] = sl[i][j+3] = sl[i+1][j-3] = 
	  sl[i+1][j] = sl[i+1][j+3] = 0;
	  sl[i][j] = 255;

	  j++;
	  sl[i-1][j-3] = sl[i-1][j] = sl[i-1][j+3] = sl[i][j-3] = sl[i][j+3] = sl[i+1][j-3] = 
	  sl[i+1][j] = sl[i+1][j+3] = 0;
	  sl[i][j] = 255;

	  j++;
	  sl[i-1][j-3] = sl[i-1][j] = sl[i-1][j+3] = sl[i][j-3] = sl[i][j+3] = sl[i+1][j-3] = 
	  sl[i+1][j] = sl[i+1][j+3] = 255;
	  sl[i][j] = 255;
	}
}

void AllocateCornerList(CORNER_LIST* pcl, int x, int y)
{
	pcl->corners = alloc2Dshort(x,y);
}
