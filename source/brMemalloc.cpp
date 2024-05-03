/************************************************************************/
/*									*/
/*	Memory allocation functions		*/
/*									*/
/************************************************************************/

#include <stdlib.h>

int ** alloc2Dint(int x, int y)
{
   int  **ary,i;
   ary = new int *[y];
   ary[0] = new int [x*y];
   
   if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
      ary[i] = &ary[0][i*x];
   for (i=x*y-1;i>=0;i--)
      ary[0][i] = 0;
   return ary;
}

int ** alloc2Dint(int x, int y, int* data)
{
   int  **ary,i;
	ary = new int *[y];

   if(data == NULL)
	ary[0] = new int [x*y];
   else
	ary[0] = data;

   if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
      ary[i] = &ary[0][i*x];
   for (i=x*y-1;i>=0;i--)
      ary[0][i] = 0;
   return ary;
}

int *** alloc3Dint(int x, int y, int z)
{
   int  ***ary,i;

   ary = new int **[x];
   if (ary==0) return NULL;
   for (i=x-1;i>=0;i--) {
      ary[i] = alloc2Dint(y,z);
      if (ary[i]==0) return NULL;
   }
   return ary;
}

unsigned char ** alloc2Duchar(int x, int y, int grey_level)
{
	unsigned char  **ary;
	int	i;

	ary = new unsigned char *[y];
	ary[0] = new unsigned char [x*y];
	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
      ary[0][i] = grey_level;
   return ary;
}

unsigned char ** alloc2Duchar(int x, int y, unsigned char* data, int grey_level)
{
	unsigned char  **ary;
	int	i;

	ary = new unsigned char *[y];
	if(data == NULL)
		ary[0] = new unsigned char [x*y];
	else
		ary[0] = data;

	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
      ary[0][i] = grey_level;
   return ary;
}

char ** alloc2Dchar(int x, int y, int grey_level)
{
	char  **ary;
	int	i;

	ary = new char *[y];
	ary[0] = new char [x*y];
	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
      ary[0][i] = grey_level;
   return ary;
}


signed char ** alloc2Dschar(int x, int y)
{
	signed char  **ary;
	int	i;

	ary = new signed char *[y];
	ary[0] = new signed char [x*y];
	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
		ary[0][i] = 0;
	return ary;
}

float ** alloc2Dfloat(int x, int y)
{
	float  **ary;
	int	i;

	ary = new float *[y];
	ary[0] = new float [x*y];
	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
		ary[0][i] = 0;
	return ary;
}

double ** alloc2Ddouble(int x, int y)
{
	double  **ary;
	int	i;

	ary = new double *[y];
	ary[0] = new double [x*y];
	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
		ary[0][i] = 0;
	return ary;
}
unsigned char *** alloc3Duchar(int x, int y, int z, int grey_level)
{
	unsigned char  ***ary;
	int	i;

	ary = new unsigned char **[x];
	if (ary==0) return NULL;
	for (i=x-1;i>=0;i--) {
		ary[i] = alloc2Duchar(y,z,grey_level);
		if (ary[i]==0) return NULL;
   }
   return ary;
}

signed char *** alloc3Dschar(int x, int y, int z)
{
	signed char  ***ary;
	int i;

	ary = new signed char **[x];
	if (ary==0) return NULL;
	for (i=x-1;i>=0;i--) {
		ary[i] = alloc2Dschar(y,z);
		if (ary[i]==0) return NULL;
	}
	return ary;
}

float *** alloc3Dfloat(int x, int y, int z)
{
	float  ***ary;
	int i;

	ary = new float **[x];
	if (ary==0) return NULL;
	for (i=x-1;i>=0;i--) {
		ary[i] = alloc2Dfloat(y,z);
		if (ary[i]==0) return NULL;
	}
	return ary;
}

float **** alloc4Dfloat(int x, int y, int z, int u)
{
	float  ****ary;
	int i;

	ary = new float ***[x];
	if (ary==0) return NULL;
	for (i=x-1;i>=0;i--) {
		ary[i] = alloc3Dfloat(y,z,u);
		if (ary[i]==0) return NULL;
	}
	return ary;
}
unsigned short ** alloc2Dushort(int x, int y)
{
	unsigned short **ary;
	int  i;

	ary = new unsigned short *[y];
	ary[0] = new unsigned short [x*y];
	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
		ary[0][i] = 0;
	return ary;
}

signed short ** alloc2Dshort(int x, int y)
{
	signed short **ary;
	int  i;

	ary = new signed short *[y];
	ary[0] = new signed short [x*y];
	if ((ary==0)||(ary[0]==0)) return NULL;
	for (i=y-1;i>=0;i--)
		ary[i] = &ary[0][i*x];
	for (i=x*y-1;i>=0;i--)
		ary[0][i] = 0;
	return ary;
}


short *** alloc3Dshort(int x, int y, int z)
{
	short  ***ary;
	int i;

   ary = new short **[x];
   if (ary==0) return NULL;
   for (i=x-1;i>=0;i--) {
      ary[i] = alloc2Dshort(y,z);
      if (ary[i]==0) return NULL;
   }
   return ary;
}

void free2D(unsigned char **ptr)
{
   delete[] ptr[0];
   delete[] ptr;
}

void free2D(float **ptr)
{
	delete[] ptr[0];
	delete[] ptr;
}

void free2D(double **ptr)
{
	delete[] ptr[0];
	delete[] ptr;
}

void free2D(signed char **ptr)
{
	delete[] ptr[0];
	delete[] ptr;
}

void free2D(unsigned short **ptr)
{
	delete[] ptr[0];
	delete[] ptr;
}

void free2D(signed short **ptr)
{
	delete[] ptr[0];
	delete[] ptr;
}

void free2D(int **ptr)
{
	delete[] ptr[0];
	delete[] ptr;
}

void free3D(int ***ptr,int x)
{
	int i;

	for (i=x-1;i>=0;i--)
		free2D(ptr[i]);
	delete[] ptr;
}

void free3D(unsigned char ***ptr,int x)
{
   int i;

   for (i=x-1;i>=0;i--)
		free2D(ptr[i]);
   delete[] ptr;
}

void free3D(signed char ***ptr,int x)
{
	int i;

	for (i=x-1;i>=0;i--)
		free2D(ptr[i]);
	delete[] ptr;
}

void free3D(float ***ptr,int x)
{
	int i;

	for (i=x-1;i>=0;i--)
		free2D(ptr[i]);
	delete[] ptr;
}

void free4D(float ****ptr,int x, int y)
{
	int i;

	for (i=x-1;i>=0;i--)
		free3D(ptr[i], y);
	delete[] ptr;
}


void free3D(short ***ptr,int x)
{
	int i;

	for (i=x-1;i>=0;i--)
		free2D(ptr[i]);
	delete[] ptr;
}



