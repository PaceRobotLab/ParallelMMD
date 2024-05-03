#include "dmatrix.h"

void TranslationLMedS(int matches[][4], int n, dMatrix M, int* inliers);
void HomographyLMedS(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability);
void AffineLMedS(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability);
void HomographyRANSAC(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability, float max_error);
void AffineRANSAC(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability, float max_error);
void Affine3RANSAC(int matches[][4], int n, dMatrix M, int* inliers, int number_of_points, 
					 float percentage_of_outliers, float desired_probability, float max_error, int ret_matrix = 0);

void random_sample(int n, int n_samples, int* sample);
void random_sample(int n, int n_samples, int* sample, int* all_samples, int n_all_samples);
void urandom_sample(int n, int n_samples, int* sample);

