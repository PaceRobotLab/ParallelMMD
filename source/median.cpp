#include "median.h"


int GetMedianFromCountingSort(int MedianLocation, int *histogram)
{
//	histogramSize = maxImageValue - minImageValuel
 /* Local Variables */
    int i,j;
    int histCum;

	i = j = 0;
	histCum = histogram[0];

	while(j==0)
	{
		i++;
		histCum+= histogram[i];
		if(histCum>MedianLocation) j = i;
	}
	
	return j-1;
}

int GetMedianAndSigmaFromCountingSort(int N, int *histogram, int *temp_histogram, float* sigma)
{
//	histogramSize = maxImageValue - minImageValuel
// temp_histogram has the same size as histogram
    
	int i, median, sig, MedianLocation = (N+1)/2;
	median = GetMedianFromCountingSort(MedianLocation, histogram);
	
	int N1 = N-median;
	temp_histogram[0] = histogram[median];
	
	for(i=1;i<=median;i++)
		temp_histogram[i] = histogram[median-i];

	for(i=1;i<N1;i++)
		temp_histogram[i]+= histogram[median+i];

	sig = GetMedianFromCountingSort(MedianLocation, histogram);
	*sigma = float(1.426*sig);

	return median;
}

