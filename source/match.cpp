#include <math.h>
#include <stdlib.h>
#include <memory.h>

#include "CornerClass.h"
#include "brMemalloc.h"
#include "draw.h"
#include "linalg.h"

#define COR_TH	0.6f
#define Xm 160
#define Ym 120
#define __min(a,b) ((a)<(b))?(a):(b)
#define __max(a,b) ((a)>(b))?(a):(b)

void DescribeCorner(unsigned char**s, int i, int j, int r, CORNER_M* ret )
{
float a,b;
unsigned char pom2;
int k,l;
ret->Hx = ret->Hy = 0;
ret->dx = ret->dy = 0;
ret->index1 = ret->index2 = -1;
a=0;b=0;
int pov = (2*r+1);
pov*= pov;

for(k=i-r;k<=i+r;k++)
for(l=j-r;l<=j+r;l++){
	pom2=s[k][l];
	ret->s[k-i+r][l-j+r] =pom2;
	a+=pom2;
	b+=pom2*pom2;}
ret->H = (int)a;
ret->var = (float)sqrt((double)(pov*b-a*a));
ret->x = j;
ret->y = i;
ret->info = 1;
}

void CreateCornerList(CCorner* pCorner, CORNER_LIST* corner_list)
{
	int i, j, k, n;
	unsigned char** s = pCorner->mGetImage();
	corner_list->width = pCorner->x;
	corner_list->height = pCorner->y;
	corner_list->corners = alloc2Dshort(pCorner->x, pCorner->y);
	corner_list->N = n = pCorner->N;

	for(k=0;k<n;k++)
	{
		i = pCorner->yc[k];
		j = pCorner->xc[k];
		DescribeCorner(s, i, j, 2 ,&(corner_list->Corners[k]));
		corner_list->corners[i][j] = k+1;
	}

}

int	 MatchCorners(CORNER_M* pcm, CORNER_LIST* c2, CRectangle ROI, CORNER_M* p_result)
{
	int i, j, k, l, x, y, newcor, i1, j1, i_min, i_max, j_min, j_max, n2;
	float corelation, newcorelation;
	short **cor2 = c2->corners;
	p_result = 0;
	int index = -1;
	x = c2->width;
	y = c2->height;
	i = pcm->y;
	j = pcm->x;
	corelation = COR_TH;
	i_min = __max(2,ROI.y_min); i_max = __min(y-2,ROI.y_max); //changed by mdt
	j_min = __max(2,ROI.x_min); j_max = __min(x-2,ROI.x_max); //mdt

	for(i1 = i_min; i1<=i_max; i1++)
	for(j1 = j_min; j1<=j_max; j1++)
	if(cor2[i1][j1])
	{
		n2 = cor2[i1][j1]-1;
		newcor =0;
		for(k=0;k<5;k++)for(l=0;l<5;l++)newcor+=pcm->s[k][l]*c2->Corners[n2].s[k][l];
		newcor*= 25; 
		newcor-=(pcm->H*c2->Corners[n2].H);
		newcorelation = (newcor/pcm->var)/c2->Corners[n2].var;

		if(newcorelation > corelation)
		{
			corelation = newcorelation;
			p_result = c2->Corners + n2;
			index = n2;
		}
	}
	return index;
}


void MatchCorners(CORNER_LIST* c1, CORNER_LIST* c2, int x1, int y1, int R)
{
int i,j,i1,j1,n1,n2,newcor,k,l, N1, N2;
int i_min, i_max, j_min, j_max;
//	p1 maps the best matches, ie. for corner i in image 1 the best match in
//  image 2 is corner p1[i] in image 2 and vice versa	

float corelation, newcorelation;
short **cor1 = c1->corners;
short **cor2 = c2->corners;
int x = c1->width, y=c1->height;

N1 = c1->N;
N2 = c2->N;
int p1[1000], p2[1000];
for(i=0;i<1000;i++){
	p1[i] = 1000;
	p2[i] = 1000;
	c1->Corners[i].index2 = -1;
	c2->Corners[i].index1 = -1;}

for(n1=0;n1<N1;n1++){
	i = c1->Corners[n1].y;
	j = c1->Corners[n1].x;
	corelation = COR_TH;
	i_min = __max(2,i-R); i_max = __min(y-2,i+R); //changed by mdt
	j_min = __max(2,j-R); j_max = __min(x-2,j+R); //mdt

	for(i1 = i_min; i1<=i_max; i1++)
	for(j1 = j_min; j1<=j_max; j1++)
	if(cor2[i1][j1]){
		n2 = cor2[i1][j1]-1;
		newcor =0;
		for(k=0;k<5;k++)for(l=0;l<5;l++)newcor+=c1->Corners[n1].s[k][l]*c2->Corners[n2].s[k][l];
		newcor*= 25; 
		newcor-=(c1->Corners[n1].H*c2->Corners[n2].H);
//		newcor = c1[n1].H*c2[n2].H + c1[n1].Hx*c2[n2].Hx + c1[n1].Hy*c2[n2].Hy;
		newcorelation = (newcor/c1->Corners[n1].var)/c2->Corners[n2].var;

		if(newcorelation > corelation){
			corelation = newcorelation;
			p1[n1] = n2;}
		}
	}

for(n2=0;n2<N2;n2++){
	i = c2->Corners[n2].y;
	j = c2->Corners[n2].x;
	corelation = COR_TH;
	i_min = __max(2,i-R); i_max = __min(y-2,i+R);
	j_min = __max(2,j-R); j_max = __min(x-2,j+R);

	for(i1 = i_min; i1<=i_max; i1++)
	for(j1 = j_min; j1<=j_max; j1++)
	if(cor1[i1][j1]){
		n1 = cor1[i1][j1]-1;
		newcor =0;
		for(k=0;k<5;k++)for(l=0;l<5;l++)newcor+=c2->Corners[n2].s[k][l]*c1->Corners[n1].s[k][l];
		newcor*= 25; // 9 = 3*3 == n^2
		newcor-=(c1->Corners[n1].H*c2->Corners[n2].H);
//		newcor = c1->Corners[n1].H*c2->Corners[n2].H + c1->Corners[n1].Hx*c2->Corners[n2].Hx + c1[n1].Hy*c2[n2].Hy;
		newcorelation = (newcor/c1->Corners[n1].var)/c2->Corners[n2].var;

		if(newcorelation > corelation){
			corelation = newcorelation;
			p2[n2] = n1;}
		}
	}

for(n1=0;n1<N1;n1++)
if((n2 = p1[n1])!=1000)
if(p2[n2]==n1){
	i = c1->Corners[n1].y;
	j = c1->Corners[n1].x;
	i1= c2->Corners[n2].y;
	j1= c2->Corners[n2].x;
	c1->Corners[n1].index2 = n2;
	c2->Corners[n2].index1 = n1;
	c1->Corners[n1].dx = (signed char)(j1-j);
	c1->Corners[n1].dy = (signed char)(i1-i);
	}
}

int MatchCornersTranslational(CORNER_LIST* c1, CORNER_LIST* c2, int vector[2], int matches[][4], int R)
{
	int i, j, k, i1, n1, x1, y1, x2, y2, xp, yp;
	int Tx = vector[0], Ty = vector[1];
	CRectangle ROI;
	n1 = c1->N;
	k = 0;

	for(i=0; i<n1; i++)
	{
		x1 = c1->Corners[i].x;
		y1 = c1->Corners[i].y;
		x2 = x1 + Tx;
		y2 = y1 + Ty;
		ROI.Set(x2-R, y2-R, x2+R, y2+R);
		j = MatchCorners(c1->Corners+i, c2, ROI);

		x2 = c2->Corners[j].x;
		y2 = c2->Corners[j].y;
		xp = x2 - Tx;
		yp = y2 - Ty;
		ROI.Set(xp-R, yp-R, xp+R, yp+R);
		i1 = MatchCorners(c2->Corners+j, c1, ROI);

		if(i1 == i)
		{
			matches[k][0] = x1;
			matches[k][1] = y1;
			matches[k][2] = x2;
			matches[k][3] = y2;
			k++;
			c1->Corners[i].index2 = j;
			c2->Corners[j].index1 = i;
			c1->Corners[i].dx = (signed char)(x2-x1);
			c1->Corners[i].dy = (signed char)(y2-y1);

		}
	}

	return k;
}


int MatchCornersAffine(CORNER_LIST* c1, CORNER_LIST* c2, dMatrix* A, int matches[][4], int R)
{
	dMatrix iA(3,3);
	iA = A->inverse();
	int i, j, k, i1, n1, x1, y1, x2, y2,xp,yp;
	n1 = c1->N;
	k = 0;
	CRectangle ROI;

	for(i=0; i<n1; i++)
	{
		x1 = c1->Corners[i].x;
		y1 = c1->Corners[i].y;
		x2 = int(A->a[0][0]*x1 + A->a[0][1]*y1 + A->a[0][2]);
		y2 = int(A->a[1][0]*x1 + A->a[1][1]*y1 + A->a[1][2]);
		ROI.Set(x2-R, y2-R, x2+R, y2+R);
		j = MatchCorners(c1->Corners+i, c2, ROI);

		x2 = c2->Corners[j].x;
		y2 = c2->Corners[j].y;
		xp = int(iA.a[0][0]*x2 + iA.a[0][1]*y2 + iA.a[0][2]);
		yp = int(iA.a[1][0]*x2 + iA.a[1][1]*y2 + iA.a[1][2]);
		ROI.Set(xp-R, yp-R, xp+R, yp+R);
		i1 = MatchCorners(c2->Corners+j, c1, ROI);

		if(i1 == i)
		{
			matches[k][0] = x1;
			matches[k][1] = y1;
			matches[k][2] = x2;
			matches[k][3] = y2;
			k++;
			c1->Corners[i].index2 = j;
			c2->Corners[j].index1 = i;
			c1->Corners[i].dx = (signed char)(x2-x1);
			c1->Corners[i].dy = (signed char)(y2-y1);

		}
		else
			c1->Corners[i].index2 = -1;
	}
	return k;
}

void MatchRotCorners(CORNER_LIST* c1, CORNER_LIST* c2, float** M, int x, int y, int R)
{
	int i,j,i1,j1,n1,n2,newcor,k,l, N1, N2;
	//	p1 maps the best matches, ie. for corner i in image 1 the best match in
	//  image 2 is corner p1[i] in image 2 and vice versa	

	float corelation, newcorelation;
	float X, Y, Z;
	float** iM = alloc2Dfloat(3,3);
	inv3x3(M, iM);
	int ic, jc;

	// Debugging
	float Md[3][3], iMd[3][3];
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
		{
			Md[i][j] = M[i][j];
			iMd[i][j]= iM[i][j];
		}
	// end debugging


	short **cor1 = c1->corners;
	short **cor2 = c2->corners;
	N1 = c1->N;
	N2 = c2->N;
	int p1[1000], p2[1000];
	for(i=0;i<1000;i++)
	{
		p1[i] = 1000;
		p2[i] = 1000;
		c1->Corners[i].index2 = -1;
		c2->Corners[i].index1 = -1;
	}

	for(n1=0;n1<N1;n1++)
	{
		i = c1->Corners[n1].y;
		j = c1->Corners[n1].x;
		if(i==100)
			i=i+0;
		corelation = COR_TH;
		X = M[0][0]*j+M[0][1]*i+M[0][2];
		Y = M[1][0]*j+M[1][1]*i+M[1][2];
		Z = M[2][0]*j+M[2][1]*i+M[2][2];
		jc = (int)(X/Z);
		ic = (int)(Y/Z);

		if(ic>R && ic <y-R && jc>R && jc<x-R)
		{
			for(i1 = ic-R; i1<=ic+R; i1++)
			for(j1 = jc-R; j1<=jc+R; j1++)
			if(cor2[i1][j1])
			{
				n2 = cor2[i1][j1]-1;
				newcor =0;
				for(k=0;k<5;k++)for(l=0;l<5;l++)newcor+=c1->Corners[n1].s[k][l]*c2->Corners[n2].s[k][l];
				newcor*= 25; 
				newcor-=(c1->Corners[n1].H*c2->Corners[n2].H);
				//		newcor = c1[n1].H*c2[n2].H + c1[n1].Hx*c2[n2].Hx + c1[n1].Hy*c2[n2].Hy;
				newcorelation = (newcor/c1->Corners[n1].var)/c2->Corners[n2].var;

				if(newcorelation > corelation)
				{
					corelation = newcorelation;
					p1[n1] = n2;
				}
			}
		}
	}


	for(n2=0;n2<N2;n2++)
	{
		i = c2->Corners[n2].y;
		j = c2->Corners[n2].x;
		corelation = COR_TH;
		X = iM[0][0]*j+iM[0][1]*i+iM[0][2];
		Y = iM[1][0]*j+iM[1][1]*i+iM[1][2];
		Z = iM[2][0]*j+iM[2][1]*i+iM[2][2];
		jc = (int)(X/Z);
		ic = (int)(Y/Z);

		if(ic>R && ic <y-R && jc>R && jc<x-R)
		{
			for(i1 = ic-R; i1<=ic+R; i1++)
			for(j1 = jc-R; j1<=jc+R; j1++)
			if(cor1[i1][j1])
			{
				n1 = cor1[i1][j1]-1;
				newcor =0;
				for(k=0;k<5;k++)for(l=0;l<5;l++)newcor+=c2->Corners[n2].s[k][l]*c1->Corners[n1].s[k][l];
				newcor*= 25; // 9 = 3*3 == n^2
				newcor-=(c1->Corners[n1].H*c2->Corners[n2].H);
				//		newcor = c1->Corners[n1].H*c2->Corners[n2].H + c1->Corners[n1].Hx*c2->Corners[n2].Hx + c1[n1].Hy*c2[n2].Hy;
				newcorelation = (newcor/c1->Corners[n1].var)/c2->Corners[n2].var;

				if(newcorelation > corelation)
				{
					corelation = newcorelation;
					p2[n2] = n1;
				}
			}
		}
	}

	for(n1=0;n1<N1;n1++)
	if((n2 = p1[n1])!=1000)
	if(p2[n2]==n1)
	{
		i = c1->Corners[n1].y;
		j = c1->Corners[n1].x;
		i1= c2->Corners[n2].y;
		j1= c2->Corners[n2].x;
		c1->Corners[n1].index2 = n2;
		c2->Corners[n2].index1 = n1;
		c1->Corners[n1].dx = (signed char)(j1-j);
		c1->Corners[n1].dy = (signed char)(i1-i);
	}
}

void MakeTriplets(CORNER_LIST* c1, CORNER_LIST* c2, CORNER_LIST* c3, int m[300][6])
{
	int i1, i2, i3, k=0;
	CORNER_M* pc;

	for(i2=0;i2<c2->N; i2++)
	{
		pc = &(c2->Corners[i2]);
		if((i1 = pc->index1) >=0  && (i3 = pc->index2) >=0)
		{
			m[k][0] = c1->Corners[i1].x;
			m[k][1] = c1->Corners[i1].y;
			m[k][2] = c2->Corners[i2].x;
			m[k][3] = c2->Corners[i2].y;
			m[k][4] = c3->Corners[i3].x;
			m[k][5] = c3->Corners[i3].y;
			k++;
		}
	}
}


void PlotPointMatches(CORNER_LIST* c1, unsigned char** s)
{
	int i, n, x, y;
	CORNER_M* pc;
	n = c1->N;
	for(i=0;i<n;i++)
	{
		pc = &(c1->Corners[i]);
		if(pc->index2>=0)
		{
			x = pc->x;
			y = pc->y;
			s[y-1][x-1]=s[y-1][x]=s[y-1][x+1]=s[y][x-1]=s[y][x+1]=s[y+1][x-1]=s[y+1][x]=s[y+1][x+1]=0;
			MakeGreyArrow(pc->dx, pc->dy, x, y, s);
		}
	}
}

void WriteMatches2File(CORNER_LIST* c1, FILE* pFile)
{
	int i, n;
	CORNER_M* pcm;
	n = c1->N;

	for(i=0;i<n;i++)
	{
		pcm = &(c1->Corners[i]);
		if(pcm->index2>= 0)
			fprintf(pFile,"%4d %4d %4d %4d \n",pcm->x, pcm->y, pcm->x+pcm->dx, pcm->y+pcm->dy);
	}
}

void MakeGreyArrow(signed char u, signed char v, int m,int n, unsigned char** a)
{
int i,x,y;
a[n][m] = 255;
unsigned char r=255;
float d,du,dv;
du = (float)abs(u);
dv = (float)abs(v);
d = (du>dv)?du: dv;

if(d==0)goto END;
du = u/d;	dv=v/d;
for(i=0;i<=d;i++){
	x=int(m+i*du+.5);
	y=int(n+i*dv+.5);
	if(x>=0 && x<Xm && y>=0 && y<Ym)
		a[y][x]=r;
//	r/=10;r*=9;
	}
END:;
}
