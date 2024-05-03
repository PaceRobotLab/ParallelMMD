// Robust_Estimators.cpp
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "brMemalloc.h"
#include "select.h"
#include "homography.h"
#include "affine.h"
#include "robust_estimators.h"
#include "stdio.h"

#define __min(a,b) ((a)<(b))?(a):(b)
#define __max(a,b) ((a)>(b))?(a):(b)


int compare_int( const void *arg1, const void *arg2 );

void TranslationLMedS(int matches[][4], int n, dMatrix M, int* inliers)
{
	// Allocation
	float dx[1000], dy[1000],cx,cy;
	int i,j,k;
	dMatrix iK1(3,3), K1(3,3), K2(3,3);
	iK1.a[0][0] = 120.0; iK1.a[0][2] = 160.0;
	iK1.a[1][1] = 90.0; iK1.a[1][2] = 120.0;
	iK1.a[2][2] = 1.0;

	K1 = iK1.inverse();
	K2 = K1;
	
	for(i=0;i<n;i++)
	{
		dx[i] = float(matches[i][2] - matches[i][0]);
		dy[i] = float(matches[i][3] - matches[i][1]);
	}

	cx = select(int(n/2), n, dx);
	cy = select(int(n/2), n, dy);


	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen("matches.txt", "wt");
		for(i=0;i<n;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
		fclose(pFilem);
	}

	k=0;
	for(i=0;i<n;i++)
	{
		if(dx[i]>=cx-20 && dx[i] <= cx+20 && dy[i]>=cy-20 && dy[i]<=cy+20)
		{
			inliers[i]=1;
			for(j=0;j<4;j++)matches[k][j] = matches[i][j];
//			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
			k++;
		}
		else
			inliers[i] = 0;
	}


//	FindHomography(matches, k, M, K1, K2);
	M.set2identity();
	M.a[0][2] = cx;
	M.a[1][2] = cy;

//	fclose(pFilem);
}


void HomographyLMedS(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability)
{
	// Allocation
	int ms[4][4], samples[400];
	int i,j,k,ns,idx,ind[4];
	double Mat[3][300];
	dMatrix iK1(3,3), K1(3,3), K2(3,3);
	iK1.a[0][0] = 120.0; iK1.a[0][2] = 160.0;
	iK1.a[1][1] = 90.0; iK1.a[1][2] = 120.0;
	iK1.a[2][2] = 1.0;

	K1 = iK1.inverse();
	K2 = K1;
	int aa = K1.get_ncols();
	if(aa !=3)
		aa = 3;

	float err[1000], median_error[100];
	
	
	// Estimating number of samples ns
	int p = number_of_points;
	float pr = desired_probability;
	float e = percentage_of_outliers;
	double c = 1.4826*(1+5/(n-p));
	ns = int( log(1-pr)/log(1-pow((1-e),p)) + 0.5);

	int ind1;
	float mn;
	mn = 10000;

	for(i = 0; i<ns; i++)
	{
	  random_sample(n, p, ind, samples, i);

	  for(j=0;j<p;j++)
	  {
		idx = ind[j];
		for(k=0;k<4;k++)
			ms[j][k] = matches[idx][k];
	  }
	  FindHomography(ms, p, M);
	  homography_error(matches, err, n, M.a);
	  median_error[i] = select(int(n/2)+4, n, err);
	  if(median_error[i]<mn)
	  {
	    mn = median_error[i];
	    ind1 = i;
	  }

	  for(j=0;j<3;j++)for(k=0;k<3;k++)
		  Mat[j][3*i+k] = M.a[j][k];
	}

	ind1+= 0;

	float threshold = float(2.5*c*mn);
	threshold = __max(threshold, 0.5f);
	
	for(i=0;i<3;i++)
	for(j=0;j<3;j++)
		M.a[i][j] = Mat[i][3*ind1+j];

	homography_error(matches, err, n, M.a);
	k=0;
	for(i=0;i<n;i++)
	{
		if(err[i]<threshold)
		{
			inliers[i]=1;
			for(j=0;j<4;j++)matches[k][j] = matches[i][j];
//			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
			k++;
		}
		else
			inliers[i] = 0;
	}

	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen("matches.txt", "wt");
		for(i=0;i<k;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
		fclose(pFilem);
	}

	FindHomography(matches, k, M);//, K1, K2);

//	fclose(pFilem);
}

void AffineLMedS(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability)
{
	// Allocation
	int ms[3][4], samples[400], w[1000];
	int i,j,k,ns,idx,ind[4], support0 = 0, support1, ds = 100;
	double Mat[3][3];
	float mn = 10000;

	dMatrix iK1(3,3), K1(3,3), K2(3,3);
	iK1.a[0][0] = 120.0; iK1.a[0][2] = 160.0;
	iK1.a[1][1] = 90.0; iK1.a[1][2] = 120.0;
	iK1.a[2][2] = 1.0;

	K1 = iK1.inverse();
	K2 = K1;
	
	int dbg = 1;
	if(dbg == 1)
	{
		FILE* pFilem = fopen("matches.txt", "wt");
		for(i=0;i<n;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
		fclose(pFilem);
	}
	

	float err[1000], median_error;
//	Initial
	TranslationLMedS(matches, n, M, w);
	k=0;
	for(i=0;i<n;i++)
	  if(w[i])
	  {
		for(j=0;j<4;j++)matches[k][j] = matches[i][j];
		k++;
	  }
	n = k;

	i = 0;
	while(i<5 && ds>1)
	{
		FindAffineMatrix(matches, n, M, w);
		affine_error(matches, err, n, M.a, 100.0f, w, &support1);
		ds = abs(support1 - support0);
		support0 = support1;
		i++;
	}

	mn = select(int(n/2), n, err);	

	if(mn<10) return;

    for(j=0;j<3;j++)for(k=0;k<3;k++)
	  Mat[j][k] = M.a[j][k];
	
	// Estimating number of samples ns
	int p = number_of_points;
	float pr = desired_probability;
	float e = percentage_of_outliers;
	double c = 1.4826*(1+5/(n-p));
	ns = int( log(1-pr)/log(1-pow((1-e),p)) + 0.5);
	int ind1;


	for(i = 0; i<ns; i++)
	{
	  random_sample(n, p, ind, samples, i);

	  for(j=0;j<p;j++)
	  {
		idx = ind[j];
		for(k=0;k<4;k++)
			ms[j][k] = matches[idx][k];
	  }
	  FindAffineMatrix(ms, p, M);
	  affine_error(matches, err, n, M.a);
	  median_error = select(int(n/2), n, err);
      if(median_error<mn)
	  {
	    mn = median_error;
	    for(j=0;j<3;j++)for(k=0;k<3;k++)
		  Mat[j][k] = M.a[j][k];
	  }
	}

	ind1=0;
	if(mn>100)
	{
		M.set2identity();
		return;
	}

	float threshold = float(2.5*c*mn);
	threshold = __max(threshold, 0.5f);
	threshold = __min(100, threshold);
	
	for(i=0;i<3;i++)
	for(j=0;j<3;j++)
		M.a[i][j] = Mat[i][j];

	affine_error(matches, err, n, M.a);
	k=0;

	for(i=0;i<n;i++)
	{
		if(err[i]<threshold)
		{
			inliers[i]=1;
			for(j=0;j<4;j++)matches[k][j] = matches[i][j];
//			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
			k++;
		}
		else
			inliers[i] = 0;
	}



//	FindAffineMatrix(matches, k, M);
	FindHomography(matches, k, M, K1, K2);

//	fclose(pFilem);
}





//			R	A	N	S	A	C



void HomographyRANSAC(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability, float max_error)
{
	// Allocation
	int ms[4][4], samples[400];
	int i,j,k,ns,idx,ind[4], support[100];
	double Mat[3][300];
	dMatrix iK1(3,3), K1(3,3), K2(3,3);
	iK1.a[0][0] = 120.0; iK1.a[0][2] = 160.0;
	iK1.a[1][1] = 90.0; iK1.a[1][2] = 120.0;
	iK1.a[2][2] = 1.0;

	K1 = iK1.inverse();
	K2 = K1;
	int aa = K1.get_ncols();
	if(aa !=3)
		aa = 3;

	float err[1000];
	
	
	// Estimating number of samples ns
	int p = number_of_points;
	float pr = desired_probability;
	float e = percentage_of_outliers;
	double c = 1.4826*(1+5/(n-p));
	ns = int( log(1-pr)/log(1-pow((1-e),p)) + 0.5);

	int ind1, mx;
	mx = 0;

	for(i = 0; i<ns; i++)
	{
	  random_sample(n, p, ind, samples, i);

	  for(j=0;j<p;j++)
	  {
		idx = ind[j];
		for(k=0;k<4;k++)
			ms[j][k] = matches[idx][k];
	  }
	  FindHomography(ms, p, M);
	  homography_error(matches, err, n, M.a);
	  support[i] = 0;
	  for(j=0;j<n;j++)
		  if(err[j]<max_error)(support[i])++;

	  if(support[i]>mx)
	  {
		  mx = support[i];
		  ind1 = i;
	  }

	  for(j=0;j<3;j++)for(k=0;k<3;k++)
		  Mat[j][3*i+k] = M.a[j][k];
	}



	for(i=0;i<3;i++)
	for(j=0;j<3;j++)
		M.a[i][j] = Mat[i][3*ind1+j];

	homography_error(matches, err, n, M.a);
	k=0;
	for(i=0;i<n;i++)
	{
		if(err[i]<max_error)
		{
			inliers[i]=1;
			for(j=0;j<4;j++)matches[k][j] = matches[i][j];
//			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
			k++;
		}
		else
			inliers[i] = 0;
	}

	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen("matches.txt", "wt");
		for(i=0;i<k;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
		fclose(pFilem);
	}

	FindHomography(matches, k, M);//, K1, K2);

//	fclose(pFilem);
}



void AffineRANSAC(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability, float max_error)
{
	// Allocation
	int ms[3][4], samples[3000], support;
	int i,j,k,ns,idx,ind[4];
	double Mat[3][300];
	float err[1000];
	dMatrix Mh(3,3), Ma(3,3);
	double error_h, error_a;

	dMatrix iK1(3,3), K1(3,3), K2(3,3);
	iK1.a[0][0] = 120.0; iK1.a[0][2] = 160.0;
	iK1.a[1][1] = 90.0; iK1.a[1][2] = 120.0;
	iK1.a[2][2] = 1.0;

	K1 = iK1.inverse();
	K2 = K1;

	int  mx = 0;
	// Check if identity matrix is a good solution;
	for(i=0;i<n;i++)
	if(matches[i][0] == matches[i][2] && matches[i][1] == matches[i][3])
		mx++;

	if(mx> __min(n,10))
	{
		M.set2identity();
		return;
	}
	else
		mx = 0;

	// Initialize
	FindAffineMatrix(matches, n, M);
	homography_error(matches, err, n, M.a);
    for(j=0;j<n;j++)
      if(err[j]<max_error)mx++;

	for(j=0;j<3;j++)for(k=0;k<3;k++)
	  Mat[j][k] = M.a[j][k];
	

	// Estimating number of samples ns
	int p = number_of_points;
	float pr = desired_probability;
	float e = percentage_of_outliers;
	double c = 1.4826*(1+5/(n-p));
	ns = int( log(1-pr)/log(1-pow((1-e),p)) + 0.5);
	i=0;

	while(i<ns)
	{
	  random_sample(n, p, ind, samples, i);

	  for(j=0;j<p;j++)
	  {
		idx = ind[j];
		for(k=0;k<4;k++)
			ms[j][k] = matches[idx][k];
	  }
	  FindAffineMatrix(ms, p, M);
	  homography_error(matches, err, n, M.a);

	  support = 0;
	  for(j=0;j<n;j++)
		  if(err[j]<max_error)support++;

	  if(support>mx)
	  {
		  mx = support;
		  for(j=0;j<3;j++)for(k=0;k<3;k++)
		  Mat[j][k] = M.a[j][k];
	  }
	  i++;
	}

	
	for(i=0;i<3;i++)
	for(j=0;j<3;j++)
		M.a[i][j] = Mat[i][j];

	homography_error(matches, err, n, M.a);

	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen("matches.txt", "wt");
		for(i=0;i<n;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
		fclose(pFilem);
	}

	k=0;
	for(i=0;i<n;i++)
	{
		if(err[i]<max_error)
		{
			inliers[i]=1;
			for(j=0;j<4;j++)matches[k][j] = matches[i][j];
			k++;
		}
		else
			inliers[i] = 0;
	}

	FindHomography(matches, k, Mh);//, K1, K2);
	FindAffineMatrix(matches, k, Ma);
	error_h = homography_error(matches, err, k, Mh.a);
	error_a = homography_error(matches, err, k, Ma.a);

	if(error_a > 1.1*error_h)
		M = Mh;
	else
		M = Ma;

}



void Affine3RANSAC(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability, float max_error, int ret_matrix)
{
	// Allocation
	int ms[3][4], samples[3000], support;
	int i,j,k,l,ns,idx,ind[4];
	double Mat[3][300];
	float err[1000], error;
	dMatrix Mh(3,3), Ma(3,3);
	double error_h, error_a;
	printf("Affine3 RANSCAC\n");

	dMatrix iK1(3,3), K1(3,3), K2(3,3);
	iK1.a[0][0] = 120.0; iK1.a[0][2] = 160.0;
	iK1.a[1][1] = 90.0; iK1.a[1][2] = 120.0;
	iK1.a[2][2] = 1.0;

	K1 = iK1.inverse();
	K2 = K1;

	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen(".\\a.txt", "at");
		fprintf(pFilem, "%d \n", n);

		for(i=0;i<n;i++)
			fprintf(pFilem, "%d %d %d %d \n", matches[i][0], matches[i][1], matches[i][2], matches[i][3]);
		fclose(pFilem);
	}
	
	int n1,  mx = 0;
	// Initialize
	error = FindAffineMatrix3(matches, n, M, err);
    for(j=0;j<n;j++)
      if(err[j]<max_error)mx++;

	for(j=0;j<3;j++)for(k=0;k<3;k++)
	  Mat[j][k] = M.a[j][k];
	

	// Estimating number of samples ns
	int p = number_of_points;
	float pr = desired_probability;
	float e = percentage_of_outliers;
	double c = 1.4826*(1+5/(n-p));
	ns = int( log(1-pr)/log(1-pow((1-e),p)) + 0.5);
	n1 = n*(n-1)/2;
	if(p>2)
		n1 = (n1*(n-2))/3;

	ns = __min(ns,n1);
	ns = __max(ns, 20);

	i=0; l=0;

	while(i<ns)
	{
	  random_sample(n, p, ind, samples, i);
	  if(i>0)
	  {
		  l=1;
		  for(j=0;j<p;j++)
			  l = l && (samples[p*i+j] == samples[p*i-p+j]);

		  if(l)
			  l=1;
	  }

//	 FILE* pFilem1 = fopen("matches1.txt", "wt");
	  for(j=0;j<p;j++)
	  {
		idx = ind[j];
//		fprintf(pFilem1, "%d ", idx);
		for(k=0;k<4;k++)
			ms[j][k] = matches[idx][k];
	  }
//	  fprintf(pFilem1, " \n");
//	fclose(pFilem1);

	  if((error = FindAffineMatrix3(ms, p, M, err)) < max_error)
	  {
		  l++;
		  support = 0;
		  affine_error(matches, err, n, M.a, max_error, inliers, &support);

		  if(support>mx)
		  {
			  mx = support;
			  for(j=0;j<3;j++)for(k=0;k<3;k++)
			  Mat[j][k] = M.a[j][k];
		  }
	  }
	  i++;
	}

//	check for error
	if(mx<5)
	{
		M.set2identity();
		if(dbg == 1)
		{
			FILE* pFilem = fopen("c:\\user\\mdt\\a.txt", "at");
			for(i=0;i<3;i++)
				fprintf(pFilem, "%7.4f  %7.4f  %7.4f \n", M.a[i][0], M.a[i][1], M.a[i][2]);
			fclose(pFilem);
		}
		return;
	}
	
	for(i=0;i<3;i++)
	for(j=0;j<3;j++)
		M.a[i][j] = Mat[i][j];

	affine_error(matches, err, n, M.a, max_error, inliers, &support);


	k=0;
	for(i=0;i<n;i++)
	{
		if(err[i]<max_error)
		{
			inliers[i]=1;
			for(j=0;j<4;j++)matches[k][j] = matches[i][j];
			k++;
		}
		else
			inliers[i] = 0;
	}



	
	FindHomography(matches, k, Mh);//, K1, K2);
	FindAffineMatrix(matches, k, Ma);
	error_h = homography_error(matches, err, k, Mh.a);
	error_a = homography_error(matches, err, k, Ma.a);

	if(error_a > 1.1*error_h)
		M = Mh;
	else
		M = Ma;

	if(dbg == 1)
	{
		FILE* pFilem = fopen("c:\\user\\mdt\\a.txt", "at");
		for(i=0;i<3;i++)
			fprintf(pFilem, "%7.4f  %7.4f  %7.4f \n", M.a[i][0], M.a[i][1], M.a[i][2]);
		fclose(pFilem);
	}

}



























void random_sample(int n, int sample_size, int* sample)
{
  //int tmp;
  static int a = 1;
  if(a)
  {
	  srand(time(0));
	  a=0;
  }
  int i=0, j, idx;

  for(i=0; i<sample_size; i++)
  {
	idx = rand()%n;
	j=0;
	while(j<i)
	{
	  if(idx==sample[j])
	  {
		j=0;
		idx = rand()%n;
	  }
	  else
		j++;
	}
	sample[i]=idx;
  }

/*  if(sample[0]>sample[1])
  {
	  tmp = sample[0];
	  sample[0] = sample[1];
	  sample[1] = tmp;
  }
*/
//  qsort((void*)sample, sample_size, sizeof(int), compare_int);
	sort20(sample, sample_size);
}

void urandom_sample(int n, int n_samples, int* sample)
{
  static int a = 1;
  if(a)
  {
	  srand(time(0));
	  a=0;
  }
  

  int i=0, d0=0, d1;

  for(i=0; i<n_samples; i++)
  {
	d1 = int((i*n+n)/n_samples);
	sample[i] = d0 + rand()%(d1-d0);
	d0 = d1;
  }
}

void random_sample(int n, int sample_size, int* sample, int* all_samples, int n_all_samples)
{
	//Create random sample, but check if the newly created sample already exist, and if it does, StartOver again.
	// After a new sample is created update all_samples
	int i, j, *pti;
	bool flag, tmp;
StartOver:
	random_sample(n, sample_size, sample);
	pti = all_samples;

	for(i=0;i<n_all_samples;i++)
	{
		flag = true;
		for(j=0;j<sample_size;j++)
		{
			tmp = (*pti == sample[j]);
			pti++;
			flag = flag && tmp;
		}

		if(flag == true)
			goto StartOver;
	}

	pti = &(all_samples[sample_size*n_all_samples]);

	for(j=0;j<sample_size;j++)
		*pti++ = sample[j];

}

int compare_int( const void *arg1, const void *arg2 )
{
	return (int*)arg1 - (int*)arg2;
}
