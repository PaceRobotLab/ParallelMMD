// MIC.cpp
#include	<stdlib.h>
#include	<string.h>
#include	<math.h>
#include	"brMemalloc.h"
#include	"mic.h"
#include	"nms.h"

#define __min(a,b) ((a)<(b))?(a):(b)
#define __max(a,b) ((a)>(b))?(a):(b)

CMIC::CMIC(int width, int height)
{
	x = width;
	y = height;
	s = alloc2Dint(x/4, y/4);
	v = alloc2Dint(x,y);
	v1 = alloc2Dint(x/4, y/4);
}

void CMIC::fMIC(unsigned char** image, unsigned char** p_corners, int* xc, int* yc, int* thresholds)
{
//  fMIC- Detects corners using a multiresolution algorithm.
//	It uses the original and a four times smaller size image.

	int i,j,k,l,i1=0,j1,i2,j2, x1, y1, n1=4, LowTh, HighTh;

	corner_map = p_corners;
	LowTh = thresholds[0];
	HighTh = thresholds[1];
	x1 = x/4;
	y1 = y/4;
	memset((void*)v[0], 0, x*y*sizeof(int));

	if(LowTh>0)
	{
		memset((void*)v1[0], 0, x1*y1*sizeof(int));

		for(i=0;i<y1;i++)
		{
		  j1=0;
		  for(j=0;j<x1;j++)
		  {
			  for(k=0;k<4;k++)for(l=0;l<4;l++)
				s[i][j]+=image[i1+k][j1+l];
			  j1+=4;
		  }
		  i1+=4;
		}


		for(i=2;i<y1-2;i++)
		for(j=2;j<x1-2;j++)
		{
		  v1[i][j] = FindMinVar3(s, i, j, 16*LowTh);
		  if(v1[i][j]>0)
		  {
			 i1=n1*i;
			 j1=n1*j;
			 for(k=0;k<n1;k++)
			 {
			   i2=i1+k;
			   for(l=0;l<n1;l++)
			   {
				 j2=j1+l;
				 v[i2][j2] = FindMinVarS3a(image,i2,j2,HighTh);
			   }
			 }
		  }
		}
	}
	else
	{
		for(i=4;i<y-4;i++)
		for(j=4;j<x-4;j++)
			v[i][j] = FindMinVarS3a(image,i,j,HighTh);
	}

		FindLocalMax(v, corner_map, xc, yc, &N, x, y, WIN);
}

// Corner Response functions for various windows sizes

int CMIC::FindMinVar3(int** s, int i, int j, int th)
{
int v1,v2,v3,v4;
int f,v=0,m1,m2;
int q1,q2;

f=s[i][j];
v1 = f-s[i][j+1];
v2 = f-s[i+1][j];
v3 = f-s[i][j-1];
v4 = f-s[i-1][j];
q1=v1*v1+v3*v3;
q2=v2*v2+v4*v4;
m1 = __min(q1,q2);
if(m1>th){
 v1 = f-s[i+1][j+1];
 v2 = f-s[i+1][j-1];
 v3 = f-s[i-1][j-1];
 v4 = f-s[i-1][j+1];
 q1=v1*v1+v3*v3;
 q2=v2*v2+v4*v4;
 m2 = __min(q1,q2);
 if(m2>th){
  v=__min(m1,m2);}
 }
return v;
}

int FindMinVar3(unsigned char** s, int i, int j, int th)
{
int v1,v2,v3,v4;
int f,v=0,m1,m2;
int q1,q2;

f=s[i][j];
v1 = f-s[i][j+1];
v2 = f-s[i+1][j];
v3 = f-s[i][j-1];
v4 = f-s[i-1][j];
q1=v1*v1+v3*v3;
q2=v2*v2+v4*v4;
m1 = __min(q1,q2);
if(m1>th){
 v1 = f-s[i+1][j+1];
 v2 = f-s[i+1][j-1];
 v3 = f-s[i-1][j-1];
 v4 = f-s[i-1][j+1];
 q1=v1*v1+v3*v3;
 q2=v2*v2+v4*v4;
 m2 = __min(q1,q2);
 if(m2>th){
  v=__min(m1,m2);}
 }
return v;
}

int CMIC::FindMinVar5(unsigned char** s, int i, int j, int th)
{
int v1,v2,v3,v4;
int f,v=0,m1,m2,m3,m4;
int q1,q2;

f=(int)s[i][j];
v1 = f-s[i][j+2];
v2 = f-s[i+2][j];
v3 = f-s[i][j-2];
v4 = f-s[i-2][j];
q1=v1*v1+v3*v3;
q2=v2*v2+v4*v4;
m1 = __min(q1,q2);
if(m1>th){
 v1 = f-s[i+1][j+2];
 v2 = f-s[i+2][j-1];
 v3 = f-s[i-1][j-2];
 v4 = f-s[i-2][j+1];
 q1=v1*v1+v3*v3;
 q2=v2*v2+v4*v4;
 m2 = __min(q1,q2);
 if(m2>th){
  v1 = f-s[i+2][j+1];
  v2 = f-s[i+1][j-2];
  v3 = f-s[i-2][j-1];
  v4 = f-s[i-1][j+2];
  q1=v1*v1+v3*v3;
  q2=v2*v2+v4*v4;
  m3 = __min(q1,q2);
  if(m3>th){
	m4 = __min(m1,m2);
	v = __min(m3,m4);}
  }
 }
return v;
}

int CMIC::FindMinVar5a(unsigned char** s, int i, int j, int th)
{
int f0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11; //derivative values
int d0,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11; // difference between derivatives
int r0,r1,r2,r3,r4,r5; // corner responses
int R0,R1,R2,R3,R4,R5; // intercorner responses
int R = 0;
int A,B,AB,C; //aucilary variables
float mr = 900;
int common = -s[i][j];
f0 = common + s[i][j+2];
f6 = common + s[i][j-2];
r0 = f0*f0 + f6*f6;
if(r0>th){
 f3 = common + s[i+2][j];
 f9 = common + s[i-2][j];
 r3 = f3*f3 + f9*f9;
 if(r3>th){
  f1 = common + s[i+1][j+2];
  f7 = common + s[i-1][j-2];
  r1 = f1*f1 + f7*f7;
  if(r1>th){
   f2 = common + s[i+2][j+1];
   f8 = common + s[i-2][j-1];
   r2 = f2*f2+f8*f8;
   if(r2>th){
    f4 = common + s[i+2][j-1];
    f10 = common + s[i-2][j+1];
    r4 = f4*f4 + f10*f10;
    if(r4>th){
     f5 = common + s[i+1][j-2];
     f11 = common + s[i-1][j+2];
     r5 = f5*f5 + f11*f11;

     d0 = f1-f0;
     d6 = f7 - f6;
     B = f0*d0+f6*d6;
     C = r0;
     AB = r1 - C - B;
     if(B<0 && AB>0){
      A=AB-B;
      R0 = C - B*B/A;}
     else R0 = __min(r0,r1);
     if(R0>th){
      d1 = f2-f1;
      d7 = f8-f7;
      B = f1*d1 + f7*d7;
      C = r1;
      AB = r2 - C - B;
      if(B<0 && AB>0){
       A=AB-B;
       R0 = C - B*B/A;}
      else R1 = __min(r1,r2);

      if(R1>th){
       d2 = f3-f2;
       d8 = f9-f8;
       B = f2*d2 + f8*d8;
       C = r2;
       AB = r3 - C - B;
       if(B<0 && AB>0){
        A=AB-B;
        R2 = C - B*B/A;}
       else R2 = __min(r2,r3);

       if(R2>th){
        d3 = f4-f3;
        d9 = f10-f9;
        B = f3*d3 + f9*d9;
        C = r3;
        AB = r4 - C - B;
        if(B<0 && AB>0){
         A=AB-B;
         R3 = C - B*B/A;}
        else R3 = __min(r3,r4);

        if(R3>th){
         d4 = f5-f4;
         d10 = f11-f10;
         B = f4*d4 + f10*d10;
         C = r4;
         AB = r5 - C - B;
         if(B<0 && AB>0){
          A=AB-B;
          R4 = C - B*B/A;}
         else R4 = __min(r4,r5);

         if(R4>th){
          d5 = f6-f5;
          d11 = f0-f11;
          B = f5*d5 + f11*d11;
          C = r5;
          AB = r0 - C - B;
          if(B<0 && AB>0){
           A=AB-B;
           R5 = C - B*B/A;}
          else R5 = __min(r5,r0);

          R = __min(__min(R0,R1),__min(R2,R3));
          R = __min(__min(R4,R5),R);
          }
         }
        }
       }
      }
     }
    }
   }
  }
 }
return (int)R;
}


int CMIC::FindMinVar7(unsigned char** s, int i, int j, int th)
{
int v1,v2,v3,v4;
int f,v=0,m1,m2,m3,m4;
int q1,q2;

f=(int)s[i][j];
v1 = f-s[i][j+3];
v2 = f-s[i+3][j];
v3 = f-s[i][j-3];
v4 = f-s[i-3][j];
q1=v1*v1+v3*v3;
q2=v2*v2+v4*v4;
m1 = __min(q1,q2);
if(m1>th){
 v1 = f-s[i+1][j+3];
 v2 = f-s[i+3][j-1];
 v3 = f-s[i-1][j-3];
 v4 = f-s[i-3][j+1];
 q1=v1*v1+v3*v3;
 q2=v2*v2+v4*v4;
 m2 = __min(q1,q2);
 if(m2>th){
  v1 = f-s[i+2][j+2];
  v2 = f-s[i+2][j-2];
  v3 = f-s[i-2][j-2];
  v4 = f-s[i-2][j+2];
  q1=v1*v1+v3*v3;
  q2=v2*v2+v4*v4;
  m3 = __min(q1,q2);
  if(m3>th){
	v1 = f-s[i+3][j+1];
	v2 = f-s[i+1][j-3];
	v3 = f-s[i-3][j-1];
	v4 = f-s[i-1][j+3];
	q1=v1*v1+v3*v3;
	q2=v2*v2+v4*v4;
	m4 = __min(q1,q2);
	if(m4>th){
	 v=__min(__min(m1,m2), __min(m3,m4));}
	}
  }
 }
return v;
}

int CMIC::FindMinVarS3(unsigned char** s, int i, int j, int th, int mode)
{
float a1,a2,a3,a4;
float a12,a14,a32,a34;
float f,A,B1,B2,B;
float v=0,v1,v2,v3,v4,dv;

f=s[i][j];
a1 = s[i][j+1]-f;
a2 = s[i+1][j]-f;
a3 = s[i][j-1]-f;
a4 = s[i-1][j]-f;
v1=a1*a1+a3*a3;

if(mode==Line)
 { //level 1
  if(v1>th)
  {        //level 2
	v2=a2*a2+a4*a4;
	a12 = s[i+1][j+1] - f; a32 = s[i+1][j-1] - f;
	a14 = s[i-1][j+1] - f; a34 = s[i-1][j-1] - f;
	v3 = a12*a12 + a34*a34; v4 = a14*a14+a32*a32;
	if((v2>th)&&(v3>th)&&(v4>th))
	{                //level 3
	  a12=a1*a2;
	  a14=a1*a4;
	  a32=a3*a2;
	  a34=a3*a4;
	  B1 = a12+a34-v1;
	  B2 = a14+a32-v1;

	  B = __min(B1,B2);
	  if(B<0){A=v1-v2-B;  //A should be dv+2B, but we first have to check A-B;
		if(A>0){A-=B;
		 v = v1-B*B/A;
		 if(v<th)v=0;
		 }
		else v = __min(__min(v1,v2),__min(v3,v4));
	  }
	  else v = __min(__min(v1,v2),__min(v3,v4));
	}                         //level 3
  } 								  //level 2
 }                           //level 1
else
 { //Level 1
 if(v1>th)
  {      //level 2
  v2=a2*a2+a4*a4;
  if(v2>th)
	{              //level 3
	 B1 = a1*a2+a3*a4;
	 B2 = a1*a4+a2*a3;
	 B = __min(B1,B2);

	 if(B<0){                       //level 4
	  dv = v1-v2;
	  v = (v1+v2-(int)sqrt((double)(dv*dv+4*B*B)))/2;
	  if(v<th)v=0;}//level 4

	 else v = __min(v1,v2);
	}                       //level3
 //
  }   // level2
 } // level 1

return (int)v;
}


int CMIC::FindMinVarS3a(int** s, int i, int j, int th, int mode)
{
float a1,a2,a3,a4,a5,a6,a7,a8;
float f,A1,A2,A3,A4,B1,B2,B3,B4,C1,C2,C3,C4,B;
float v=0,v1,v2,v3,v4,dv;

f=(float)s[i][j];
a1 = s[i][j+1]-f;
a2 = s[i+1][j]-f;
a3 = s[i][j-1]-f;
a4 = s[i-1][j]-f;

v1=a1*a1+a3*a3;

if(mode==Line)
 { //level 1
  if(v1>th)
  {        //level 2
	v2=a2*a2+a4*a4;
	a5 = s[i+1][j+1] - f; a6 = s[i+1][j-1] - f;
	a8 = s[i-1][j+1] - f; a7 = s[i-1][j-1] - f;
	v3 = a5*a5 + a7*a7; v4 = a6*a6+a8*a8;
	if((v2>th)&&(v3>th)&&(v4>th))
	{                //level 3
		C1 = v1; C2=v2; C3=v2; C4=v1;
		B1 = a1*a5+a3*a7-v1;
		B2 = a2*a5+a4*a7-v2;
		B3 = a2*a6+a4*a8-v2;
		B4 = a3*a6+a1*a8-v1;

	  if(B1<0){A1=v3-C1-B1;  //A should be dv+2B, but we first have to check A-B;
		if(A1>0){A1-=B1;
		 v1 = C1-B1*B1/A1;}
		}
	  if(B2<0){A2=v3-C2-B2;  //A should be dv+2B, but we first have to check A-B;
		if(A2>0){A2-=B2;
		 v2 = C2-B2*B2/A2;}
		}
	  if(B3<0){A3=v4-C3-B3;  //A should be dv+2B, but we first have to check A-B;
		if(A3>0){A3-=B3;
		 v3 = C3-B3*B3/A3;}
		}
	  if(B4<0){A4=v4-C4-B4;  //A should be dv+2B, but we first have to check A-B;
		if(A4>0){A4-=B4;
		 v4 = C4-B4*B4/A4;}
		}
	  v = __min(__min(v1,v2),__min(v3,v4));
	  if(v<th)v=0;
	}                         //level 3
  } 								  //level 2
 }                           //level 1
else
 { //Level 1
 if(v1>th)
  {      //level 2
  v2=a2*a2+a4*a4;
  if(v2>th)
	{              //level 3
	 B1 = a1*a2+a3*a4;
	 B2 = a1*a4+a2*a3;
	 B = __min(B1,B2);

	 if(B<0){                       //level 4
	  dv = v1-v2;
	  v = (v1+v2-(int)sqrt((double)(dv*dv+4*B*B)))/2;
	  if(v<th)v=0;}//level 4

	 else v = __min(v1,v2);
	}                       //level3
 //
  }   // level2
 } // level 1


return (int)v;
}

int CMIC::FindMinVarS3a(unsigned char** s, int i, int j, int th, int mode)
{
float a1,a2,a3,a4,a5,a6,a7,a8;
float f,A1,A2,A3,A4,B1,B2,B3,B4,C1,C2,C3,C4,B;
float v=0,v1,v2,v3,v4,dv;

f=s[i][j];
a1 = s[i][j+1]-f;
a2 = s[i+1][j]-f;
a3 = s[i][j-1]-f;
a4 = s[i-1][j]-f;

v1=a1*a1+a3*a3;

if(mode==Line)
 { //level 1
  if(v1>th)
  {        //level 2
	v2=a2*a2+a4*a4;
	a5 = s[i+1][j+1] - f; a6 = s[i+1][j-1] - f;
	a8 = s[i-1][j+1] - f; a7 = s[i-1][j-1] - f;
	v3 = a5*a5 + a7*a7; v4 = a6*a6+a8*a8;
	if((v2>th)&&(v3>th)&&(v4>th))
	{                //level 3
		C1 = v1; C2=v2; C3=v2; C4=v1;
		B1 = a1*a5+a3*a7-v1;
		B2 = a2*a5+a4*a7-v2;
		B3 = a2*a6+a4*a8-v2;
		B4 = a3*a6+a1*a8-v1;

	  if(B1<0){A1=v3-C1-B1;  //A should be dv+2B, but we first have to check A-B;
		if(A1>0){A1-=B1;
		 v1 = C1-B1*B1/A1;}
		}
	  if(B2<0){A2=v3-C2-B2;  //A should be dv+2B, but we first have to check A-B;
		if(A2>0){A2-=B2;
		 v2 = C2-B2*B2/A2;}
		}
	  if(B3<0){A3=v4-C3-B3;  //A should be dv+2B, but we first have to check A-B;
		if(A3>0){A3-=B3;
		 v3 = C3-B3*B3/A3;}
		}
	  if(B4<0){A4=v4-C4-B4;  //A should be dv+2B, but we first have to check A-B;
		if(A4>0){A4-=B4;
		 v4 = C4-B4*B4/A4;}
		}
	  v = __min(__min(v1,v2),__min(v3,v4));
	  if(v<th)v=0;
	}                         //level 3
  } 								  //level 2
 }                           //level 1
else
 { //Level 1
 if(v1>th)
  {      //level 2
  v2=a2*a2+a4*a4;
  if(v2>th)
	{              //level 3
	 B1 = a1*a2+a3*a4;
	 B2 = a1*a4+a2*a3;
	 B = __min(B1,B2);

	 if(B<0){                       //level 4
	  dv = v1-v2;
	  v = (v1+v2-(int)sqrt((double)(dv*dv+4*B*B)))/2;
	  if(v<th)v=0;}//level 4

	 else v = __min(v1,v2);
	}                       //level3
 //
  }   // level2
 } // level 1

if(v>0)
   if(!FindMinVar5a(s, i, j, th))
    v=0;


return (int)v;
}

int CMIC::NewFindMinVar3(int** s, int i, int j, int th, int mode)
{
int a1,a2,a3,a4;
int f,A,B1,B2,B;
int v=0,v1,v2,dv;

f=s[i][j];
a1 = s[i][j+1]-f;
a2 = s[i+1][j]-f;
a3 = s[i][j-1]-f;
a4 = s[i-1][j]-f;
v1=a1*a1+a3*a3;

if(mode==Line)
 if(v1>th)
 {
  v2=a2*a2+a4*a4;
  if(v2>th)
  {
	 B1 = v2-a1*a2-a3*a4;
	 B2 = v2-a1*a4-a2*a3;
	 B=__min(B1,B2);

	 if(B>0){A=v1-v2+B;  //A should be dv+2B, but we first have to check A-B;
	  if(A>0){A+=B;
		v = v2-(B*B)/A;}
	  }
	 else v = __min(v1,v2);
  }
 //
 }
else
 if(v1>th)
 {
  v2=a2*a2+a4*a4;
  if(v2>th)
  {
	 B1 = a1*a2+a3*a4;
	 B2 = a1*a4+a2*a3;
	 B = __min(B1,B2);

	 if(B<0){
	  dv = v1-v2;
	  v = (v1+v2+(int)sqrt((double)(dv*dv+4*B*B)))/2;}

	 else v = __min(v1,v2);
  }
 //
 }
return v;
}


