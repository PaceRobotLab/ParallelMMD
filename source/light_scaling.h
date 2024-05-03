
#include "dmatrix.h"
#include "cv.h"

void find_min_ROI(dMatrix M, int width, int height, int* roi);
float find_scale(IplImage* IplFirst, IplImage* IplSecond, IplImage* IplBGD, dMatrix M);
void scaled_difference(IplImage* IplFirst, IplImage* IplSecond, IplImage* IplBGD, IplImage* IplResult, dMatrix M, float s);
