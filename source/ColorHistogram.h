//	Defines a class ColorHistogram
#ifndef COLOR_HISTOGRAM_H
	#define	 COLOR_HISTOGRAM_H

#ifndef MIL_IMAGE_H
	#include "milImage.h"
#endif

#ifndef MIL_FUNCTIONS
	#include "mil_functions.h"
#endif

#include "defs.h"
#define	 IMPORTANT_COLORS	15

#include "Candidate.h"


struct	OrdInt
{
	int	value, i, j;
};

struct PalInt
{
	int p[16];
};


class CColorHistogram
{
//	The purpose of the class
//	
//	The purpose of this class is to enable template matching
//	between two consecutive images based on the histogram of
//	the given original template
//
//	For a given template in first image, we first compute 2D-histogram H,
//	using chromacity components (e.g u,v)
//	Normaly, we will be using 16 color levels, so that the histogram will
//	have 256 cells. Note however, that NOT ALL the colors will be used 
//	as the chromacity curve is not a rectangular (0<u<0.7, 0<v<0.4), and my 
//	estimate is that we will probably be using aout 100 cells only.
//	Of this 100 cells, we want to find the 15 cells (c_1,...,c_15)with the highest values
//	and that will be our histogram representation.

//	After computing histogram, we will compute 15 binary images corresponding (Ik)
//	to the fifteen cell. Let p_ij(k) denote the pixel ixj in the binary image Ik.
//	Then
//
//	p_ij(k) = 1 if  |p_ij(k)-c_k| < threshold
//	p_ij(k) = 0		otherwise


private:

	unsigned char**		pPaletteImage;
	int					height, width;
	short				**v1, **v2;
	char				*pNewMemv1, *pNewMemv2;	// For alignment
	short				**hist_match;
	unsigned char		**img_disp, **tmp_peaks;

	float				histogram_vector[50];
	int					HistogramWeights[32][32][4];
	int**				pCompleteHistogram;
	int*				pGrayHistogram;
	float**				pNormalizedHistogram;
	float*				pNormalizedGrayHistogram;
	float*				p_temporaryGH;

	int***				pTA, ***pTB;	// Temporary Arrays for histogram matching
	PalInt				**pPTA, **pPTB;
	int**				p_temporaryCH;
	float**				p_temporaryNCH;
	int					gray_tolerance;
	int					intensity_tolerance;


	int					dim1, dim2, dim3;
	int					n_histogram_colors, n_gray_levels;
	int					**p_histogram_colors, *p_gray_levels;
	unsigned char		**p_hist_LUT, *p_gray_LUT;



public:

	void	CreateColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, int** pCH, float**, int dimx, int dimy);
	void	CreateHSIColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, int** pCH, float**, int dimx, int dimy);
	void	CreateGrayColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, int** pCH, int* pGH, int dimx, int dimy, int dim_gray);
	void	HistogramUpdate(Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion);
	void	CreateCompositeHSIHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* SearchRegion, float** pNCH, float* pNGH, int dimx, int dimy, int dim_gray);
	// Given the input Image and the template computes a histogram of the template
		void	AssignColor2CompleteHistogram(unsigned char x, unsigned char y, int** pCH, int histogram_granulation);
		void	AssignColor2CompleteHistogramSimple(unsigned char x, unsigned char y, int** pCH, int dimx, int dimy);
		void	AssignColor2CompleteHistogram(unsigned char x, unsigned char y, int** pCH, int* pGH, int dimx, int dimy, int dim_gray);
		// auxiliary function for CreateColorHistogram
	void	SortHistogram(int **p_histogram_colors, OrdInt* hist_colors, int histogram_size);
	//	Sort color histogram
	int CreateHistogramVector(float** p_NormalizedHistogram, float* pGrayHistogram, int** colors, int* gray_levels, int n_histogram_colors, int n_gray_levels, int dimx, int dimy, int dim_gray);
//	int	CreateHistogramVector(float **p_CompleteHistogram, int** c, int n_histogram_colors, int dimx, int dimy);
	void CreatePaletteImage(Image<unsigned char>* p_InputColorImage, int** p_histogram_colors, int n_histogram_colors, unsigned char** pPaletteImage);
	void CreateCompositeHSIPaletteImage(Image<unsigned char>* p_InputColorImage, tRectangle* Region, int** p_colors, int n_colors, int* p_gray_levels, int n_gray_levels, unsigned char** pPaletteImage);

	
	float CompareHistograms(float** pNCH0, Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion);
	float CompareCompositeHistograms(Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion);
	void HistogramMatching(unsigned char** pPaletteImage, tRectangle* Target, tRectangle* SearchRegion, int* p, int* BestMatch);
	void HistogramMatching(Image<unsigned char>* p_InputColorImage, tRectangle* Target, tRectangle* SearchRegion, int* p, float* BestMatch);

	void sHistogramMatching(Image<unsigned char>* p_InputColorImage, tRectangle* Target, 
		tRectangle* SearchRegion, int* p, float* BestMatch, unsigned char** mask = 0);
	void sHistogramMatching(unsigned char** pPaletteImage, tRectangle* SearchRegion, int wt, int ht, 
		short* hv, int* best_match_coordinates, int* BestMatch, unsigned char** mask = 0);
	void sHistogramMatching(Image<unsigned char>* p_InputColorImage, tRectangle* Target, 
		tRectangle* SearchRegion, int* p, float* BestMatch, Candidate* pCand, unsigned char** mask = 0, tRectangle* SR0 = 0);
	void sHistogramMatching(unsigned char** pPaletteImage, tRectangle* SearchRegion, int wt, int ht, 
		short* hv, int* best_match_coordinates, int* BestMatch, Candidate* pCand, unsigned char** mask = 0, tRectangle* SR0 = 0);
	void VerticalPass(unsigned char** pPaletteImage, tRectangle* SearchRegion, int wt, int ht, short** a);
	void HorizontalPass(short** v1, tRectangle* SearchRegion, int wt, int ht, short** v2, tRectangle* newSearchRegion);
	void HorizontalPassMMX(short** v1, tRectangle* SearchRegion, int wt, int ht, short** v2, tRectangle* newSearchRegion);
	void FindBestMatch(short** v2, short* hv, tRectangle* SerachRegion, int *p, int* BestMatch);
	void FindBestMatch(short** v2, short* hv, tRectangle* SerachRegion, int *p, int* BestMatch, unsigned char** mask);
	void FindBestMatchMMX(short** v2, short* hv, tRectangle* SearchRegion, int *p, int* BestMatch);
	void FindBestMatchMMX(short** v2, short* hv, tRectangle* SerachRegion, int *p, int* BestMatch, unsigned char** mask);
	void WriteHistMatch2File(char*, int);
	void TempVecArraysAlloc();

	CColorHistogram(Image<unsigned char>* p_InputColorImage, tRectangle* HistogramRegion, int histogram_size);
	~CColorHistogram();


	//Auxiliary vector operations
	void CopyVector(int* source, int* dest, int size);
	void AddVector(int* a, int* b, int size);	 // a = a + b;
	void SubtractVector(int*a, int*b, int size); // a = a - b;
	void AddVectors(int* source1, int* source2, int* dest, int size);
	int VectorIntersection(int* vec1, int* vec2, int n);

	void CopyVector(short* source, short* dest, int size);
	void AddVector(short* a, short* b, int size);	 // a = a + b;
	void SubtractVector(short*a, short*b, int size); // a = a - b;
	void AddVectors(short* source1, short* source2, short* dest, int size);
	short VectorIntersection(short vec1[], short vec2[], int n);

	float** getNormalizedHistogram()
	{
		return pNormalizedHistogram;
	}
};

#endif
