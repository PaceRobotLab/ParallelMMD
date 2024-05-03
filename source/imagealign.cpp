
#include "cv.h"
#include "highgui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <memory.h>

#ifdef _EiC
#define WIN32
#endif


    
#include "CornerClass.h"
#include "dmatrix.h"
#include "Candidate.h"

#include "robust_estimators.h"

#include "homography.h"
#include "linalg.h"
#include "imagealign.h"


// extended signature, to return the matches found

dMatrix AllignImageAffineM(CORNER_LIST* pCL0, CORNER_LIST* pCL1, dMatrix* A, 
						  int reg, int scale, int ret_matrix, int matches[][4], int *nMatches)
{
  int i, n, /*matches[1000][4],*/ mc[1000][2];
	float x1, y1;
	dMatrix M(3,3), Mr(3,3);
	M.set2identity();

	int inliers[1000];
//	CORNER_M* pCm;

	n = MatchCornersAffine(pCL0, pCL1, A, matches,300);
	printf("Number of matches: %d\n",n);

	if (n>1000) { printf("Too many matches in AlignAffine\n\n"); exit(0); }

	for(i=0; i<n; i++)
	{
		bilinear_mapping(A->a, matches[i][0], matches[i][1], &x1, &y1);
		mc[i][0] = matches[i][0];
		mc[i][1] = matches[i][1];
		matches[i][0] = int(x1+.5);
		matches[i][1] = int(y1+.5);
	}

	*nMatches = n; // return this value

	if(n>5)
	{
		if(reg==0)
			TranslationLMedS(matches, n, Mr, inliers);
		else if(reg==1)
			Affine3RANSAC(matches, n, Mr, inliers, 2, 0.66f, 0.999f, float( 16.0/sqrt(scale) ));
		else
			HomographyRANSAC(matches, n, Mr, inliers, 4, 0.4f, 0.999f, float( 16.0/sqrt(scale) ) );
	}
	else
		Mr.set2identity();

/*	int dbg = 0;
	if(dbg == 1)
	{
		FILE* pFilem = fopen(".\\matches.txt", "at");
		fprintf(pFilem, "%4d \n", n);

		for(i=0;i<n;i++)
			fprintf(pFilem, "%d %d %d %d %d %d\n", mc[i][0], mc[i][1], matches[i][0], matches[i][1], matches[i][2], matches[i][3]);

		for(i=0;i<3;i++)
			fprintf(pFilem, "%7.4f  %7.4f  %7.4f \n", Mr.a[i][0], Mr.a[i][1], Mr.a[i][2]);
		fclose(pFilem);
	}
*/
	    printf("Mr before Mc:\n");
		for(i=0;i<3;i++)
			printf("%7.4f  %7.4f  %7.4f \n", Mr.a[i][0], Mr.a[i][1], Mr.a[i][2]);

		return mult_matrix(Mr, *A);
}

//original signature of routine

dMatrix AllignImageAffine(CORNER_LIST* pCL0, CORNER_LIST* pCL1, dMatrix* A, 
						  int reg, int scale, int ret_matrix)
{ int matches[1000][4]; int nMatches;

  return AllignImageAffineM(pCL0, pCL1, A, reg, scale, ret_matrix, matches, &nMatches);
}


// make unrealistic assumptions about focus and focal point
dMatrix ComputeHomography(CPTZRadians ptz_old, CPTZRadians ptz_new)
{
	dMatrix M(3,3), Q(3,3), R(3,3), iQ(3,3), M1(3,3);
	double tilt, dpan, dtilt;

	Q.a[0][0] = 350.0;//pCamPar->lfFx;
	Q.a[1][1] = 350.0;//pCamPar->lfFy;
	Q.a[0][2] = 0; //pCamPar->plfPrincipalPoint[0];
	Q.a[1][2] = 0; //pCamPar->plfPrincipalPoint[1];
	Q.a[2][2] = 1.0;

	iQ = Q.inverse();

	tilt  = ptz_old.m_yTicks;
	dtilt = ptz_new.m_yTicks - tilt;
	dpan  = ptz_new.m_xTicks - ptz_old.m_xTicks;

	R = tpt2rot(tilt, dpan, dtilt);
	M = mult_matrix(Q, mult_matrix(R, iQ));
	M1 = mult_matrix(Q, mult_matrix(R.inverse(), iQ));
	return M.inverse();

}
