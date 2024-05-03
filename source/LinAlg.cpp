#include "linalg.h"
#include "math.h"
//#include "nrutil.h"
#include <stdlib.h>
#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}
#define TINY 1.0e-20;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0 : sqrarg*sqrarg)
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);\
	a[k][l]=h+s*(g-h*tau);
#define SGN(a) (a>0.0 ? 1 : (a==0.0 ? 0 : -1) )
#define M_PI	3.1415926
#define __min(a,b) ((a)<(b))?(a):(b)
#define __max(a,b) ((a)>(b))?(a):(b)

double sqrarg;

void gaussj(double **A, int n, double **b, int m)
{
	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	double big,dum,pivinv,temp;

	indxc=new int[n];
	indxr=new int[n];
	ipiv=new int[n];
	for (j=0;j<n;j++) ipiv[j]=0;
	for (i=0;i<n;i++) {
		big=0.0;
		for (j=0;j<n;j++)
			if (ipiv[j] != 1)
				for (k=0;k<n;k++) {
					if (ipiv[k] == 0) {
						if (fabs(A[j][k]) >= big) {
							big=fabs(A[j][k]);
							irow=j;
							icol=k;
						}
					} else if (ipiv[k] > 1) goto KRAJ;
				}
		++(ipiv[icol]);
		if (irow != icol) {
			for (l=0;l<n;l++) SWAP(A[irow][l],A[icol][l])
			for (l=0;l<m;l++) SWAP(b[irow][l],b[icol][l])
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (A[icol][icol] == 0.0) goto KRAJ;
		pivinv=1.0/A[icol][icol];
		A[icol][icol]=1.0;
		for (l=0;l<n;l++) A[icol][l] *= pivinv;
		for (l=0;l<m;l++) b[icol][l] *= pivinv;
		for (ll=0;ll<n;ll++)
			if (ll != icol) {
				dum=A[ll][icol];
				A[ll][icol]=0.0;
				for (l=0;l<n;l++) A[ll][l] -= A[icol][l]*dum;
				for (l=0;l<m;l++) b[ll][l] -= b[icol][l]*dum;
			}
	}
	for (l=n-1;l>=0;l--) {
		if (indxr[l] != indxc[l])
			for (k=0;k<n;k++)
				SWAP(A[k][indxr[l]],A[k][indxc[l]]);
	}
   delete [] ipiv;
   delete [] indxr;
   delete [] indxc;

KRAJ:;
}

void ludcmp(double **a, int n, int *indx, double *d)
{
	int i,imax,j,k;
	double big,dum,sum,temp;
	double *vv;

	vv=new double[n];
	*d=1.0;
	for (i=0;i<n;i++) {
		big=0.0;
		for (j=0;j<n;j++)
			if ((temp=fabs(a[i][j])) > big) big=temp;
		if (big == 0.0) goto KRAJ;
		vv[i]=1.0/big;
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			sum=a[i][j];
			for (k=0;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			sum=a[i][j];
			for (k=0;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=0;k<n;k++) {
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (a[j][j] == 0.0) a[j][j]=TINY;
		if (j != n) {
			dum=1.0/(a[j][j]);
			for (i=j+1;i<n;i++) a[i][j] *= dum;
		}
	}
	delete [] vv;
KRAJ:;  
}

void lubksb(double **a, int n, int *indx, double b[])
{
	int i,ii=0,ip,j;
	double sum;

	for (i=0;i<n;i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if (sum) ii=i;
		b[i]=sum;
	}
	for (i=n-1;i>=0;i--) {
		sum=b[i];
		for (j=i+1;j<n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i];
	}
}

void invert_matrix(double **a, int n, double** ia)
{
	int i, j;
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
			ia[i][j] = 0.0;
		ia[i][i] = 1.0;
	}

	gaussj(a, n, ia, n);
}



void svdcmp(double **a, int m, int n, double w[], double **v)
{
	double pythag(double a, double b);
	int flag,i,its,j,jj,k,l,nm;
	double anorm,c,f,g,h,s,scale,x,y,z,*rv1;

	rv1= new double[n];
	g=scale=anorm=0.0;
	for (i=0;i<n;i++) {
		l=i+1;
		rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i < m) {
			for (k=i;k<m;k++) scale += fabs(a[k][i]);
			if (scale) {
				for (k=i;k<m;k++) {
					a[k][i] /= scale;
					s += a[k][i]*a[k][i];
				}
				f=a[i][i];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][i]=f-g;
				for (j=l;j<n;j++) {
					for (s=0.0,k=i;k<m;k++) s += a[k][i]*a[k][j];
					f=s/h;
					for (k=i;k<m;k++) a[k][j] += f*a[k][i];
				}
				for (k=i;k<m;k++) a[k][i] *= scale;
			}
		}
		w[i]=scale *g;
		g=s=scale=0.0;
		if (i < m && i != n) {
			for (k=l;k<n;k++) scale += fabs(a[i][k]);
			if (scale) {
				for (k=l;k<n;k++) {
					a[i][k] /= scale;
					s += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a[i][l]=f-g;
				for (k=l;k<n;k++) rv1[k]=a[i][k]/h;
				for (j=l;j<m;j++) {
					for (s=0.0,k=l;k<n;k++) s += a[j][k]*a[i][k];
					for (k=l;k<n;k++) a[j][k] += s*rv1[k];
				}
				for (k=l;k<n;k++) a[i][k] *= scale;
			}
		}
		anorm=__max(anorm,(double)(fabs(w[i])+fabs(rv1[i])));
	}
	for (i=n-1;i>=0;i--) {
		if (i < n) {
			if (g) {
				for (j=l;j<n;j++)
					v[j][i]=(a[i][j]/a[i][l])/g;
				for (j=l;j<n;j++) {
					for (s=0.0,k=l;k<n;k++) s += a[i][k]*v[k][j];
					for (k=l;k<n;k++) v[k][j] += s*v[k][i];
				}
			}
			for (j=l;j<n;j++) v[i][j]=v[j][i]=0.0;
		}
		v[i][i]=1.0;
		g=rv1[i];
		l=i;
	}
	for (i=(int)__min(m,n)-1;i>=0;i--) {
		l=i+1;
		g=w[i];
		for (j=l;j<n;j++) a[i][j]=0.0;
		if (g) {
			g=1.0/g;
			for (j=l;j<n;j++) {
				for (s=0.0,k=l;k<m;k++) s += a[k][i]*a[k][j];
				f=(s/a[i][i])*g;
				for (k=i;k<m;k++) a[k][j] += f*a[k][i];
			}
			for (j=i;j<m;j++) a[j][i] *= g;
		} else for (j=i;j<m;j++) a[j][i]=0.0;
		++a[i][i];
	}
	for (k=n-1;k>=0;k--) {
		for (its=1;its<=30;its++) {
			flag=1;
			for (l=k;l>=0;l--) {
				nm=l-1;
				if ((double)(fabs(rv1[l])+anorm) == anorm) {
					flag=0;
					break;
				}
				if ((double)(fabs(w[nm])+anorm) == anorm) break;
			}
			if (flag) {
				c=0.0;
				s=1.0;
				for (i=l;i<=k;i++) {
					f=s*rv1[i];
					rv1[i]=c*rv1[i];
					if ((double)(fabs(f)+anorm) == anorm) break;
					g=w[i];
					h=pythag(f,g);
					w[i]=h;
					h=1.0/h;
					c=g*h;
					s = -f*h;
					for (j=0;j<m;j++) {
						y=a[j][nm];
						z=a[j][i];
						a[j][nm]=y*c+z*s;
						a[j][i]=z*c-y*s;
					}
				}
			}
			z=w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j=0;j<n;j++) v[j][k] = -v[j][k];
				}
				break;
			}
			if (its == 30) goto KRAJ; //nrerror("no convergence in 30 svdcmp iterations");
			x=w[l];
			nm=k-1;
			y=w[nm];
			g=rv1[nm];
			h=rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.0;
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=rv1[i];
				y=w[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				rv1[j]=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=0;jj<n;jj++) {
					x=v[jj][j];
					z=v[jj][i];
					v[jj][j]=x*c+z*s;
					v[jj][i]=z*c-x*s;
				}
				z=pythag(f,h);
				w[j]=z;
				if (z) {
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=0;jj<m;jj++) {
					y=a[jj][j];
					z=a[jj][i];
					a[jj][j]=y*c+z*s;
					a[jj][i]=z*c-y*s;
				}
			}
			rv1[l]=0.0;
			rv1[k]=f;
			w[k]=x;
		}
	}
	delete[] rv1;
KRAJ:;
}

double pythag(double a, double b)
{
	double absa,absb;
	absa=fabs(a);
	absb=fabs(b);
	if (absa > absb) return absa*sqrt(1.0+SQR(absb/absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}

void choldc(double **a, int n, double p[])
{
	int i,j,k;
	double sum;

	for (i=0;i<n;i++) {
		for (j=i;j<n;j++) {
			for (sum=a[i][j],k=i-1;k>=0;k--) sum -= a[i][k]*a[j][k];
			if (i == j) {
				if (sum <= 0.0)
					goto KRAJ;
				p[i]=sqrt(sum);
			} else a[j][i]=sum/p[i];
		}
	}
KRAJ:;
}

void cholsl(double **a, int n, double p[], double b[], double x[])
{
	int i,k;
	double sum;

	for (i=0;i<n;i++) {
		for (sum=b[i],k=i-1;k>=0;k--) sum -= a[i][k]*x[k];
		x[i]=sum/p[i];
	}
	for (i=n-1;i>=0;i--) {
		for (sum=x[i],k=i+1;k<n;k++) sum -= a[k][i]*x[k];
		x[i]=sum/p[i];
	}
}

void qrdcmp(double **a, int n, double *c, double *d, int *sing)
{
	int i,j,k;
	double scale=0.0,sigma,sum,tau;

	*sing=0;
	for (k=0;k<n-1;k++) {
		for (i=k;i<n;i++) scale=__max(scale,fabs(a[i][k]));
		if (scale == 0.0) {
			*sing=1;
			c[k]=d[k]=0.0;
		} else {
			for (i=k;i<n;i++) a[i][k] /= scale;
			for (sum=0.0,i=k;i<n;i++) sum += SQR(a[i][k]);
			sigma=SIGN(sqrt(sum),a[k][k]);
			a[k][k] += sigma;
			c[k]=sigma*a[k][k];
			d[k] = -scale*sigma;
			for (j=k+1;j<n;j++) {
				for (sum=0.0,i=k;i<n;i++) sum += a[i][k]*a[i][j];
				tau=sum/c[k];
				for (i=k;i<n;i++) a[i][j] -= tau*a[i][k];
			}
		}
	}
	d[n]=a[n][n];
	if (d[n] == 0.0) *sing=1;
}

void qrsolv(double **a, int n, double c[], double d[], double b[])
{
	void rsolv(double **a, int n, double d[], double b[]);
	int i,j;
	double sum,tau;

	for (j=0;j<n-1;j++) {
		for (sum=0.0,i=j;i<n;i++) sum += a[i][j]*b[i];
		tau=sum/c[j];
		for (i=j;i<n;i++) b[i] -= tau*a[i][j];
	}
	rsolv(a,n,d,b);
}

void rsolv(double **a, int n, double d[], double b[])
{
	int i,j;
	double sum;

	b[n] /= d[n];
	for (i=n-2;i>=0;i--) {
		for (sum=0.0,j=i+1;j<n;j++) sum += a[i][j]*b[j];
		b[i]=(b[i]-sum)/d[i];
	}
}
       
void jacobi(double **a, int n, double d[], double **v, int *nrot)
{
/*
Compute all eigenvalues and eigenvectors of a real symmetric matrix a. On
output, elements of a above the diagonal are destroyed. d returns the eigenvalues
of a. v is a matrix whose coloumns contain, on output, the normalised eigenvectors
of a. nrot returns the number of Jacobi rotations that were required.
*/
	int j,iq,ip,i;
	double tresh,theta,tau,t,sm,s,h,g,c,*b,*z;

	b=new double[n];
	z=new double[n];
	for (ip=0;ip<n;ip++) {
		for (iq=0;iq<n;iq++) v[ip][iq]=0.0;
		v[ip][ip]=1.0;
	}
	for (ip=0;ip<n;ip++) {
		b[ip]=d[ip]=a[ip][ip];
		z[ip]=0.0;
	}
	*nrot=0;
	for (i=0;i<50;i++) {
		sm=0.0;
		for (ip=0;ip<n-1;ip++) {
			for (iq=ip+1;iq<n;iq++)
				sm += fabs(a[ip][iq]);
		}
		if (sm == 0.0) {
			delete [] z;
			delete [] b;
			return;
		}
		if (i < 4)
			tresh=0.2*sm/(n*n);
		else
			tresh=0.0;
		for (ip=0;ip<n-1;ip++) {
			for (iq=ip+1;iq<n;iq++) {
				g=100.0*fabs(a[ip][iq]);
				if (i > 4 && (double)(fabs(d[ip])+g) == (double)fabs(d[ip])
					&& (double)(fabs(d[iq])+g) == (double)fabs(d[iq]))
					a[ip][iq]=0.0;
				else if (fabs(a[ip][iq]) > tresh) {
					h=d[iq]-d[ip];
					if ((double)(fabs(h)+g) == (double)fabs(h))
						t=(a[ip][iq])/h;
					else {
						theta=0.5*h/(a[ip][iq]);
						t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0) t = -t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a[ip][iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip][iq]=0.0;
					for (j=0;j<=ip-1;j++) {
						ROTATE(a,j,ip,j,iq)
					}
					for (j=ip+1;j<=iq-1;j++) {
						ROTATE(a,ip,j,j,iq)
					}
					for (j=iq+1;j<n;j++) {
						ROTATE(a,ip,j,iq,j)
					}
					for (j=0;j<n;j++) {
						ROTATE(v,j,ip,j,iq)
					}
					++(*nrot);
				}
			}
		}
		for (ip=0;ip<n;ip++) {
			b[ip] += z[ip];
			d[ip]=b[ip];
			z[ip]=0.0;
		}
	}

}

void tred2(double **a, int n, double d[], double e[])
{
	int l,k,j,i;
	double scale,hh,h,g,f;

	for (i=n-1;i>=1;i--) {
		l=i-1;
		h=scale=0.0;
		if (l > 1) {
			for (k=0;k<=l;k++)
				scale += fabs(a[i][k]);
			if (scale == 0.0)
				e[i]=a[i][l];
			else {
				for (k=0;k<=l;k++) {
					a[i][k] /= scale;
					h += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
				e[i]=scale*g;
				h -= f*g;
				a[i][l]=f-g;
				f=0.0;
				for (j=0;j<=l;j++) {
					a[j][i]=a[i][j]/h;
					g=0.0;
					for (k=0;k<=j;k++)
						g += a[j][k]*a[i][k];
					for (k=j+1;k<=l;k++)
						g += a[k][j]*a[i][k];
					e[j]=g/h;
					f += e[j]*a[i][j];
				}
				hh=f/(h+h);
				for (j=0;j<=l;j++) {
					f=a[i][j];
					e[j]=g=e[j]-hh*f;
					for (k=0;k<=j;k++)
						a[j][k] -= (f*e[k]+g*a[i][k]);
				}
			}
		} else
			e[i]=a[i][l];
		d[i]=h;
	}
	d[1]=0.0;
	e[1]=0.0;
	/* Contents of this loop can be omitted if eigenvectors not
			wanted except for statement d[i]=a[i][i]; */
	for (i=0;i<n;i++) {
		l=i-1;
		if (d[i]) {
			for (j=0;j<=l;j++) {
				g=0.0;
				for (k=0;k<=l;k++)
					g += a[i][k]*a[k][j];
				for (k=0;k<=l;k++)
					a[k][j] -= g*a[k][i];
			}
		}
		d[i]=a[i][i];
		a[i][i]=1.0;
		for (j=0;j<=l;j++) a[j][i]=a[i][j]=0.0;
	}
}

void tqli(double d[], double e[], int n, double **z)
{
	double pythag(double a, double b);
	int m,l,iter,i,k;
	double s,r,p,g,f,dd,c,b;

	for (i=1;i<n;i++) e[i-1]=e[i];
	e[n]=0.0;
	for (l=0;l<n;l++) {
		iter=0;
		do {
			for (m=l;m<n-1;m++) {
				dd=fabs(d[m])+fabs(d[m+1]);
				if ((double)(fabs(e[m])+dd) == dd) break;
			}
			if (m != l) {
				if (iter++ == 30) goto KRAJ;//nrerror("Too many iterations in tqli");
				g=(d[l+1]-d[l])/(2.0*e[l]);
				r=pythag(g,1.0);
				g=d[m]-d[l]+e[l]/(g+SIGN(r,g));
				s=c=1.0;
				p=0.0;
				for (i=m-1;i>=l;i--) {
					f=s*e[i];
					b=c*e[i];
					e[i+1]=(r=pythag(f,g));
					if (r == 0.0) {
						d[i+1] -= p;
						e[m]=0.0;
						break;
					}
					s=f/r;
					c=g/r;
					g=d[i+1]-p;
					r=(d[i]-g)*s+2.0*c*b;
					d[i+1]=g+(p=s*r);
					g=c*r-b;
					for (k=0;k<n;k++) {
						f=z[k][i+1];
						z[k][i+1]=s*z[k][i]+c*f;
						z[k][i]=c*z[k][i]-s*f;
					}
				}
				if (r == 0.0 && i >= l) continue;
				d[l] -= p;
				e[l]=g;
				e[m]=0.0;
			}
		} while (m != l);
	}
KRAJ:;
}

double det3x3(double a[][3])
{
double det;
det = a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
		+ a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
return det;
}

double det3x3(double** a)
{
double det;
det = a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
		+ a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
return det;
}

void inv3x3(float** A, float** iA)
{
	int i, j;
	float Det;
	float adjA[3][3];

	adjA[0][0] =  (A[1][1]*A[2][2]-A[2][1]*A[1][2]);
	adjA[0][1] = -(A[1][0]*A[2][2]-A[2][0]*A[1][2]);
	adjA[0][2] =  (A[1][0]*A[2][1]-A[2][0]*A[1][1]);
	adjA[1][0] = -(A[0][1]*A[2][2]-A[0][2]*A[2][1]);
	adjA[1][1] =  (A[0][0]*A[2][2]-A[2][0]*A[0][2]);
	adjA[1][2] = -(A[0][0]*A[2][1]-A[2][0]*A[0][1]);
	adjA[2][0] =  (A[0][1]*A[1][2]-A[1][1]*A[0][2]);
	adjA[2][1] = -(A[0][0]*A[1][2]-A[1][0]*A[0][2]);
	adjA[2][2] =  (A[0][0]*A[1][1]-A[1][0]*A[0][1]);

	Det = A[0][0]*adjA[0][0] + A[0][1]*adjA[0][1] + A[0][2]*adjA[0][2];

	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			iA[i][j] = adjA[j][i]/Det;
}



void cubic(double p[], double sol[], int* nreal)
{
double a,b,c,Q,R,t,A,B,Q3,sQ,rq;
a = p[1]/p[0];
b = p[2]/p[0];
c = p[3]/p[0];
Q = (a*a-3*b)/9;
R = (a*(2*a*a-9*b)+27*c)/54;
Q3 = Q*Q*Q;
rq = R*R-Q3;
if(rq<0)
	{
   /* if true, there are three real roots */
   *nreal = 3;
   sQ = sqrt(Q3);
   t = acos(R/sQ);
	sol[0] = -2*sQ*cos(t/3)-a/3;
   sol[1] = -2*sQ*cos((t+2*M_PI)/3)-a/3;
   sol[2] = -2*sQ*cos((t-2*M_PI)/3)-a/3;
   }
else{
	*nreal = 1;
	A = -SGN(R)*pow((fabs(R)+sqrt(rq)), 1.0/3.0);
   if(fabs(A)>1e-10)
   	{B = Q/A;}
   else
		{B = 0;}
   sol[0] = (A+B)-a/3;
   sol[1] = -(A+B)/2 - a/3;
   sol[2] = sqrt(0.75)*(A-B);
   }
}


#undef ROTATE
#undef TINY
#undef SWAP

