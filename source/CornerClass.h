//	CornerClass.h
#ifndef CORNER_CLASS
	#define CORNER_CLASS

#include <stdio.h>
#include "mic.h"
#include "dmatrix.h"

typedef struct 
{	
	int x,y,info,Hx,Hy,H;
    int index1, index2;
	unsigned char s[5][5];
	signed char dx, dy; 
	float var;
} CORNER_M;

#define MAX_CORNERS 1000

typedef struct
{
	CORNER_M Corners[MAX_CORNERS];
	int	width;
	int height;
	short**	corners;
	int		N;	//number of corners
} CORNER_LIST;

enum{ MIC, QHarris, Harris, SUSAN, Wang};

struct tRectangle
{
	int x_min, x_max;
	int y_min, y_max;
};

class	CRectangle
{
public:
	int	x_min, x_max;
	int	y_min, y_max;

	CRectangle(){}
	CRectangle(int x1, int x2, int y1, int y2): x_min(x1), x_max(x2), y_min(y1), y_max(y2){}
	void Set(int x_min, int y_min, int x_max, int y_max)
	{
		this->x_min = x_min;
		this->y_min = y_min;
		this->x_max = x_max;
		this->y_max = y_max;
	}
};


class CCorner
{
private:
	unsigned char		**image;			// Pointer to image
	unsigned char		**corner_map;			// Corner Map.
	int					n_corner_type;
	CMIC*				pCMIC;
	void				InitCornerDetector(int);

public:
	int			N;	//Number of Corners
	int			x,y; // image size
	int			xc[MAX_CORNERS], yc[MAX_CORNERS];	//coordinates of the corners

	CCorner(int width, int height, int algorithm, unsigned char* data = 0);
	CCorner(CCorner* pCorner, int* thresholds, int algorithm);

//	methods
	void DetectCorners(unsigned char** p_image, int* thresholds);
	void DetectCorners(unsigned char** p_image, unsigned char** p_binary_image, int scale, int* thresholds);
	void DetectCorners(unsigned char** p_image, int* thresholds, tRectangle* pRectangles, int N = 1);
	void CreateCornerImage(unsigned char** p_image, unsigned char** corner_image);
	void CreateColorCornerImage(unsigned char** corner_image);

	int GetCornerType(){
		return n_corner_type;}
	unsigned char** mGetImage(){ return image;}




	~CCorner(){};
};

void DescribeCorner(unsigned char**s, int i, int j, int r, CORNER_M* ret );
void CreateCornerList(CCorner* pCorner, CORNER_LIST* c1);
void MatchCorners(CORNER_LIST* c1, CORNER_LIST* c2, int x, int y, int radius);
int	 MatchCorners(CORNER_M* pcm, CORNER_LIST* c2, CRectangle ROI, CORNER_M* p_result = 0);
int  MatchCornersTranslational(CORNER_LIST* c1, CORNER_LIST* c2, int vector[2], int matches[][4], int R);
int  MatchCornersAffine(CORNER_LIST* c1, CORNER_LIST* c2, dMatrix* M, int m[][4], int radius);
void MatchRotCorners(CORNER_LIST* c1, CORNER_LIST* c2, float** M, int x, int y, int R);
void MakeTriplets(CORNER_LIST* c1, CORNER_LIST* c2, CORNER_LIST* c3, int m[300][6]);
void PlotPointMatches(CORNER_LIST* c1, unsigned char** s);
void WriteMatches2File(CORNER_LIST* c1, FILE* pFile);
void AllocateCornerList(CORNER_LIST* pcl, int x, int y);
void MakeGreyArrow(signed char u, signed char v, int m,int n, unsigned char** a);



#endif
