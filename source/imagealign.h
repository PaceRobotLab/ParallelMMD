
struct CPTZRadians {int m_xTicks; int m_yTicks; int m_zTicks;} ;

dMatrix AllignImageAffine(CORNER_LIST* pCL0, CORNER_LIST* pCL1, 
						  dMatrix* A, int reg, int scale, int ret_matrix);

dMatrix AllignImageAffineM(CORNER_LIST* pCL0, CORNER_LIST* pCL1, dMatrix* A, 
						  int reg, int scale, int ret_matrix, int matches[][4], int *nMatches);


dMatrix ComputeHomography(CPTZRadians ptz_old, CPTZRadians ptz_new);
