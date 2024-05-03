#include "minmax.h"

void minim(unsigned char vector[], int dim, int* index)
{
int i;
unsigned char mn;
*index = 0; mn = vector[0];
for(i=1;i<dim;i++)
if(vector[i]<mn)
	{
   mn = vector[i];
   *index = i;
   }
}

void minim(float vector[], int dim, int* index)
{
int i;
float mn;
*index = 0; mn = vector[0];
for(i=1;i<dim;i++)
if(vector[i]<mn)
	{
   mn = vector[i];
   *index = i;
   }
}

void minim(double vector[], int dim, int* index)
{
int i;
double mn;
*index = 0; mn = vector[0];
for(i=1;i<dim;i++)
if(vector[i]<mn)
	{
   mn = vector[i];
   *index = i;
   }
}

void minim(int vector[], int dim, int* index)
{
int i, mn;
*index = 0; mn = vector[0];
for(i=1;i<dim;i++)
if(vector[i]<mn)
	{
   mn = vector[i];
   *index = i;
   }
}

void maxim(unsigned char vector[], int dim, int* index)
{
int i;
unsigned char mx;
*index = 0; mx = vector[0];
for(i=1;i<dim;i++)
if(vector[i]>mx)
	{
   mx = vector[i];
   *index = i;
   }
}

void maxim(float vector[], int dim, int* index)
{
int i;
float mx;
*index = 0; mx = vector[0];
for(i=1;i<dim;i++)
if(vector[i]>mx)
	{
   mx = vector[i];
   *index = i;
   }
}

void maxim(double vector[], int dim, int* index)
{
int i;
double mx;
*index = 0; mx = vector[0];
for(i=1;i<dim;i++)
if(vector[i]>mx)
	{
   mx = vector[i];
   *index = i;
   }
}

void maxim(int vector[], int dim, int* index)
{
int i, mx;
*index = 0; mx = vector[0];
for(i=1;i<dim;i++)
if(vector[i]>mx)
	{
   mx = vector[i];
   *index = i;
   }
}