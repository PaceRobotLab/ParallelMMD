#include <math.h>

void MakeGreyArrow(signed char u, signed char v, int m,int n, unsigned char** a)
{
int i,x,y;
a[m][n] = 255;
unsigned char r=255;
float d,du,dv;
du = (float)abs(u);
dv = (float)abs(v);
d = (du>dv)?du: dv;
 
if(d==0)goto END;
du = u/d;	dv=v/d;
for(i=0;i<=d;i++){
	x=int(n+i*du+.5);
	y=int(m+i*dv+.5);
	a[y][x]=r;
//	r/=10;r*=9;
	}
END:;
}

