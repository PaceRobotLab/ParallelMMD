#ifndef CMOT_DET
#define CMOT_DET


#include "cv.h"
#include "dmatrix.h"
#define	MIN_BLOB_SIZE	20

class CMotionDetection
{
public:
	unsigned char	*im1, *im2;
	unsigned char	*g1, *g2, *gb, *gt;
	short			*gx, *gy, *gs;
	unsigned char	**motion_mask;
	unsigned char	**median_motion_mask;
	int				*g_tot;
	float			scale;
	int				width, height;
	IplImage		*Im1, *Im2, *G1, *G2, *Gb, *Gs, *Gx, *Gy, *Gt, *MM, *MedMM;
	CvPoint			center;


	CMotionDetection(int x, int y);
	~CMotionDetection();
	int compute_motion_mask(unsigned char* im1, unsigned char* im2, dMatrix M, int thr, float radius = 1);
	int CompareHomographies(unsigned char* im1, unsigned char* im2, dMatrix M0, dMatrix M1, int threshold, float radius);
};

#endif