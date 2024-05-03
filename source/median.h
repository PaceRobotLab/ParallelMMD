
void GenerateHistogramFromImage(char ** inputImage,
												    int maxHistogramSize,
												    int *histogramValues,
												    int minImageValue,
												    int maxImageValue);

int GetMedianFromCountingSort(int, int *histogram);
int GetMedianAndSigmaFromCountingSort(int N, int *histogram, int *temp_histogram, float* sigma);


/* Generate Histogram 
void GenerateHistogramFromImage(char ** inputImage,
												    int maxHistogramSize,
												    int *histogramValues,
												    int minImageValue,
												    int maxImageValue)

{
//	Some knowledge of the image histogram is required, i.e user has to provide min and max 
//	image value.
//	Array histogramValues has to be preallocated, and it has the size maxHistogramSize.
//	minImageValue corresponds to the histogramValues[0] and 
//	maxImageValue corresponds to the histogramValues[maxImageValue-minImageValue];
//	maxHistogramSize must be >= maxImageValue-minImageValue+1.
/*
	int i, N, index;//, *hist;
	char **  p_img = ( char **)inputImage->m_GetImage();
	N = inputImage->m_GetRows()*inputImage->m_GetCols();

	// set histogram to zero
	for(i=minImageValue;i<=maxImageValue;i++)
		histogramValues[i-minImageValue] = 0;
	

	for (i=0; i < N; i++)
	{
		index =  int(p_img[i]) - minImageValue;
		++ histogramValues[index];
	}
	
}*/
