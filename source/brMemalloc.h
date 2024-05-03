//	Functions for 2D and 3D memory allocation
//
//	memalloc.h
//
//	Written by Miroslav D. Trajkovic

#define	__MEMALLOC_H__


int**				alloc2Dint(int x, int y, int* data);	// allocate 2D integer array
int**				alloc2Dint(int x, int y);	// allocate 2D integer array
signed char**		alloc2Dschar(int x, int y);	// allocate 2D short integer array
float**				alloc2Dfloat(int x, int y);	// allocate 2D float array
double**			alloc2Ddouble(int x, int y);	// allocate 2D double array
int***				alloc3Dint(int x, int y, int z); // allocate 3D integer array
signed char***		alloc3Dschar(int x, int y, int z);
float***			alloc3Dfloat(int x, int y, int z);
float****			alloc4Dfloat(int x, int y, int z, int u);
char**				alloc2Dchar(int x, int y, int grey_level = 0);
unsigned char**		alloc2Duchar(int x, int y, int grey_level=0);	// 2D unsigned char array
unsigned char**		alloc2Duchar(int x, int y, unsigned char* data, int grey_level=0);	// 2D unsigned char array
unsigned char***	alloc3Duchar(int x, int y, int z, int grey_level=0); // 3D unsigned char array
unsigned short**	alloc2Dushort(int x, int y);
short**				alloc2Dshort(int x, int y);
short***			alloc3Dshort(int x, int y, int z);
void				free2D(int **ptr);			// free 2D array
void				free3D(int ***ptr,int x);		// free 3D array
void				free3D(signed char*** ptr, int x);
void				free3D(float*** ptr, int x);
void				free3D(short*** ptr, int x);
void				free2D(unsigned char **ptr);			// free 2D array
void				free3D(unsigned char ***,int );		// free 3D array
void				free2D(unsigned short **ptr);			// free 2D array
void				free2D(float **ptr);
void				free2D(double **ptr);
void				free2D(signed char **ptr);
void				free2D(signed short **ptr);
void				free4D(float ****ptr, int x, int y);
