#include <stdio.h>
#include <math.h>
#include "optim.h"
#include "brMemalloc.h"
#include <stdlib.h>
#include "nrutil.h"
#include "linalg.h"
#define GOLD 1.618034
#define GLIMIT 100.0
#define TINY 1.0e-20
#define R 0.61803399
#define C (1.0-R)
#define SHFT2(a,b,c) (a)=(b);(b)=(c);
#define SHFT3(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SHFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define ITMAX 100
#define ITMAX1 200
#define CGOLD 0.3819660
#define ZEPS 1.0e-10
#define MOV3(a,b,c, d,e,f) (a)=(d);(b)=(e);(c)=(f);
#define NMAX 5000
#define GET_PSUM \
					for (j=0;j<ndim;j++) {\
					for (sum=0.0,i=1;i<=mpts;i++) sum += p[i][j];\
					psum[j]=sum;}
#define SWAP(a,b) {swap=(a);(a)=(b);(b)=swap;}
#define TOL 2.0e-4

#define __min(a,b) ((a)<(b))?(a):(b)
#define __max(a,b) ((a)>(b))?(a):(b)

int ncom;
double *pcom,*xicom,(*nrfunc)(double []);

void mnbrak(double *ax, double *bx, double *cx, double *fa, double *fb, double *fc,
	double (*func)(double))
{
	double ulim,u,r,q,fu,dum;

	*fa=(*func)(*ax);
	*fb=(*func)(*bx);
	if (*fb > *fa) {
		SHFT(dum,*ax,*bx,dum)
		SHFT(dum,*fb,*fa,dum)
	}
	*cx=(*bx)+GOLD*(*bx-*ax);
	*fc=(*func)(*cx);
	while (*fb > *fc) {
		r=(*bx-*ax)*(*fb-*fc);
		q=(*bx-*cx)*(*fb-*fa);
		u=(*bx)-((*bx-*cx)*q-(*bx-*ax)*r)/
			(2.0*SIGN(__max(fabs(q-r),TINY),q-r));
		ulim=(*bx)+GLIMIT*(*cx-*bx);
		if ((*bx-u)*(u-*cx) > 0.0) {
			fu=(*func)(u);
			if (fu < *fc) {
				*ax=(*bx);
				*bx=u;
				*fa=(*fb);
				*fb=fu;
				return;
			} else if (fu > *fb) {
				*cx=u;
				*fc=fu;
				return;
			}
			u=(*cx)+GOLD*(*cx-*bx);
			fu=(*func)(u);
		} else if ((*cx-u)*(u-ulim) > 0.0) {
			fu=(*func)(u);
			if (fu < *fc) {
				SHFT(*bx,*cx,u,*cx+GOLD*(*cx-*bx))
				SHFT(*fb,*fc,fu,(*func)(u))
			}
		} else if ((u-ulim)*(ulim-*cx) >= 0.0) {
			u=ulim;
			fu=(*func)(u);
		} else {
			u=(*cx)+GOLD*(*cx-*bx);
			fu=(*func)(u);
		}
		SHFT(*ax,*bx,*cx,u)
		SHFT(*fa,*fb,*fc,fu)
	}
}

double brent(double ax, double bx, double cx, double (*f)(double), double tol,
	double *xmin)
{
	int iter;
	double a,b,d,etemp,fu,fv,fw,fx,p,q,r,tol1,tol2,u,v,w,x,xm;
	double e=0.0;

	a=(ax < cx ? ax : cx);
	b=(ax > cx ? ax : cx);
	x=w=v=bx;
	fw=fv=fx=(*f)(x);
	for (iter=1;iter<=ITMAX;iter++) {
		xm=0.5*(a+b);
		tol2=2.0*(tol1=tol*fabs(x)+ZEPS);
		if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
			*xmin=x;
			return fx;
		}
		if (fabs(e) > tol1) {
			r=(x-w)*(fx-fv);
			q=(x-v)*(fx-fw);
			p=(x-v)*q-(x-w)*r;
			q=2.0*(q-r);
			if (q > 0.0) p = -p;
			q=fabs(q);
			etemp=e;
			e=d;
			if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
				d=CGOLD*(e=(x >= xm ? a-x : b-x));
			else {
				d=p/q;
				u=x+d;
				if (u-a < tol2 || b-u < tol2)
					d=SIGN(tol1,xm-x);
			}
		} else {
			d=CGOLD*(e=(x >= xm ? a-x : b-x));
		}
		u=(fabs(d) >= tol1 ? x+d : x+SIGN(tol1,d));
		fu=(*f)(u);
		if (fu <= fx) {
			if (u >= x) a=x; else b=x;
			SHFT(v,w,x,u)
			SHFT(fv,fw,fx,fu)
		} else {
			if (u < x) a=u; else b=u;
			if (fu <= fw || w == x) {
				v=w;
				w=u;
				fv=fw;
				fw=fu;
			} else if (fu <= fv || v == x || v == w) {
				v=u;
				fv=fu;
			}
		}
	}
//	nrerror("Too many iterations in brent");
	*xmin=x;
	return fx;
}

double golden(double ax, double bx, double cx, double (*f)(double), double tol,
	double *xmin)
{
	double f1,f2,x0,x1,x2,x3;

	x0=ax;
	x3=cx;
	if (fabs(cx-bx) > fabs(bx-ax)) {
		x1=bx;
		x2=bx+C*(cx-bx);
	} else {
		x2=bx;
		x1=bx-C*(bx-ax);
	}
	f1=(*f)(x1);
	f2=(*f)(x2);
	while (fabs(x3-x0) > tol*(fabs(x1)+fabs(x2))) {
		if (f2 < f1) {
			SHFT3(x0,x1,x2,R*x1+C*x3)
			SHFT2(f1,f2,(*f)(x2))
		} else {
			SHFT3(x3,x2,x1,R*x2+C*x0)
			SHFT2(f2,f1,(*f)(x1))
		}
	}
	if (f1 < f2) {
		*xmin=x1;
		return f1;
	} else {
		*xmin=x2;
		return f2;
	}
}

double dbrent(double ax, double bx, double cx, double (*f)(double),
	double (*df)(double), double tol, double *xmin)
{
	int iter,ok1,ok2;
	double a,b,d,d1,d2,du,dv,dw,dx,e=0.0;
	double fu,fv,fw,fx,olde,tol1,tol2,u,u1,u2,v,w,x,xm;

	a=(ax < cx ? ax : cx);
	b=(ax > cx ? ax : cx);
	x=w=v=bx;
	fw=fv=fx=(*f)(x);
	dw=dv=dx=(*df)(x);
	for (iter=1;iter<=ITMAX;iter++) {
		xm=0.5*(a+b);
		tol1=tol*fabs(x)+ZEPS;
		tol2=2.0*tol1;
		if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
			*xmin=x;
			return fx;
		}
		if (fabs(e) > tol1) {
			d1=2.0*(b-a);
			d2=d1;
			if (dw != dx) d1=(w-x)*dx/(dx-dw);
			if (dv != dx) d2=(v-x)*dx/(dx-dv);
			u1=x+d1;
			u2=x+d2;
			ok1 = (a-u1)*(u1-b) > 0.0 && dx*d1 <= 0.0;
			ok2 = (a-u2)*(u2-b) > 0.0 && dx*d2 <= 0.0;
			olde=e;
			e=d;
			if (ok1 || ok2) {
				if (ok1 && ok2)
					d=(fabs(d1) < fabs(d2) ? d1 : d2);
				else if (ok1)
					d=d1;
				else
					d=d2;
				if (fabs(d) <= fabs(0.5*olde)) {
					u=x+d;
					if (u-a < tol2 || b-u < tol2)
						d=SIGN(tol1,xm-x);
				} else {
					d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
				}
			} else {
				d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
			}
		} else {
			d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
		}
		if (fabs(d) >= tol1) {
			u=x+d;
			fu=(*f)(u);
		} else {
			u=x+SIGN(tol1,d);
			fu=(*f)(u);
			if (fu > fx) {
				*xmin=x;
				return fx;
			}
		}
		du=(*df)(u);
		if (fu <= fx) {
			if (u >= x) a=x; else b=x;
			MOV3(v,fv,dv, w,fw,dw)
			MOV3(w,fw,dw, x,fx,dx)
			MOV3(x,fx,dx, u,fu,du)
		} else {
			if (u < x) a=u; else b=u;
			if (fu <= fw || w == x) {
				MOV3(v,fv,dv, w,fw,dw)
				MOV3(w,fw,dw, u,fu,du)
			} else if (fu < fv || v == x || v == w) {
				MOV3(v,fv,dv, u,fu,du)
			}
		}
	}
//	nrerror("Too many iterations in routine dbrent");
	return 0.0;
}

void amoeba(double **p, double y[], int ndim, double ftol,
	double (*funk)(double []), int *nfunk)
{
	double amotry(double **p, double y[], double psum[], int ndim,
		double (*funk)(double []), int ihi, double fac);
	int i,ihi,ilo,inhi,j,mpts=ndim;
	double rtol,sum,swap,ysave,ytry,*psum;

	psum= new double [ndim];
	*nfunk=0;
	GET_PSUM
	for (;;) {
		ilo=0;
		ihi = y[0]>y[1] ? (inhi=1,0) : (inhi=0,1);
		for (i=0;i<=mpts;i++) {
			if (y[i] <= y[ilo]) ilo=i;
			if (y[i] > y[ihi]) {
				inhi=ihi;
				ihi=i;
			} else if (y[i] > y[inhi] && i != ihi) inhi=i;
		}
		rtol=2.0*fabs(y[ihi]-y[ilo])/(fabs(y[ihi])+fabs(y[ilo]));
		if (rtol < ftol) {
			SWAP(y[0],y[ilo])
			for (i=0;i<ndim;i++) SWAP(p[0][i],p[ilo][i])
			break;
		}
		if (*nfunk >= NMAX) //nrerror("NMAX exceeded");
		*nfunk += 2;
		ytry=amotry(p,y,psum,ndim,funk,ihi,-1.0);
		if (ytry <= y[ilo])
			ytry=amotry(p,y,psum,ndim,funk,ihi,2.0);
		else if (ytry >= y[inhi]) {
			ysave=y[ihi];
			ytry=amotry(p,y,psum,ndim,funk,ihi,0.5);
			if (ytry >= ysave) {
				for (i=0;i<mpts;i++) {
					if (i != ilo) {
						for (j=0;j<ndim;j++)
							p[i][j]=psum[j]=0.5*(p[i][j]+p[ilo][j]);
						y[i]=(*funk)(psum);
					}
				}
				*nfunk += ndim;
				GET_PSUM
			}
		} else --(*nfunk);
	}
	delete [] psum;
}

double amotry(double **p, double y[], double psum[], int ndim,
	double (*funk)(double []), int ihi, double fac)
{
	int j;
	double fac1,fac2,ytry,*ptry;

	ptry=new double[ndim];
	fac1=(1.0-fac)/ndim;
	fac2=fac1-fac;
	for (j=0;j<ndim;j++) ptry[j]=psum[j]*fac1-p[ihi][j]*fac2;
	ytry=(*funk)(ptry);
	if (ytry < y[ihi]) {
		y[ihi]=ytry;
		for (j=0;j<ndim;j++) {
			psum[j] += ptry[j]-p[ihi][j];
			p[ihi][j]=ptry[j];
		}
	}
	delete [] ptry;
	return ytry;
}

void powell(double p[], double **xi, int n, double ftol, int *iter, double *fret,
	double (*func)(double []))
{
	void linmin(double p[], double xi[], int n, double *fret,
		double (*func)(double []));
	int i,ibig,j;
	double del,fp,fptt,t,*pt,*ptt,*xit;

	pt= new double[n];
	ptt=new double[n];
	xit=new double[n];
	*fret=(*func)(p);
	for (j=0;j<n;j++) pt[j]=p[j];
	for (*iter=0;;++(*iter)) {
		fp=(*fret);
		ibig=0;
		del=0.0;
		for (i=0;i<n;i++) {
			for (j=0;j<n;j++) xit[j]=xi[j][i];
			fptt=(*fret);
			linmin(p,xit,n,fret,func);
			if (fabs(fptt-(*fret)) > del) {
				del=fabs(fptt-(*fret));
				ibig=i;
			}
		}
		if (2.0*fabs(fp-(*fret)) <= ftol*(fabs(fp)+fabs(*fret))) {
			delete [] xit;
			delete [] ptt;
			delete [] pt;
			return;
		}
		if (*iter == ITMAX1) //nrerror("powell exceeding maximum iterations.");
		for (j=0;j<n;j++) {
			ptt[j]=2.0*p[j]-pt[j];
			xit[j]=p[j]-pt[j];
			pt[j]=p[j];
		}
		fptt=(*func)(ptt);
		if (fptt < fp) {
			t=2.0*(fp-2.0*(*fret)+fptt)*sqrt(fp-(*fret)-del)-del*sqrt(fp-fptt);  // SQR = sqrt
			if (t < 0.0) {
				linmin(p,xit,n,fret,func);
				for (j=0;j<n;j++) {
					xi[j][ibig]=xi[j][n];
					xi[j][n]=xit[j];
				}
			}
		}
	}
}

void linmin(double p[], double xi[], int n, double *fret, double (*func)(double []))
{
	double brent(double ax, double bx, double cx,
		double (*f)(double), double tol, double *xmin);
	double f1dim(double x);
	void mnbrak(double *ax, double *bx, double *cx, double *fa, double *fb,
		double *fc, double (*func)(double));
	int j;
	double xx,xmin,fx,fb,fa,bx,ax;

	ncom=n;
	pcom=new double [n];
	xicom=new double [n];
	nrfunc=func;
	for (j=0;j<n;j++) {
		pcom[j]=p[j];
		xicom[j]=xi[j];
	}
	ax=0.0;
	xx=1.0;
	mnbrak(&ax,&xx,&bx,&fa,&fx,&fb,f1dim);
	*fret=brent(ax,xx,bx,f1dim,TOL,&xmin);
	for (j=0;j<n;j++) {
		xi[j] *= xmin;
		p[j] += xi[j];
	}
	delete [] xicom;
	delete [] pcom;
}

double f1dim(double x)
{
	int j;
	double f,*xt;

	xt=new double [ncom];
	for (j=0;j<ncom;j++) xt[j]=pcom[j]+x*xicom[j];
	f=(*nrfunc)(xt);
	delete [] xt;
	return f;
}

void mrqcof(double x[], double y[], double sig[], int ndata, double a[], int ia[],
	int ma, double **alpha, double beta[], double *chisq,
	void (*funcs)(double, double [], double *, double [], int))
{
/*
	int i,j,k,l,m,mfit=0;
	double ymod,wt,sig2i,dy,*dyda;

	dyda=dvector(1,ma);
	for (j=1;j<=ma;j++)
		if (ia[j]) mfit++;
	for (j=1;j<=mfit;j++) {
		for (k=1;k<=j;k++) alpha[j][k]=0.0;
		beta[j]=0.0;
	}
	*chisq=0.0;
	for (i=1;i<=ndata;i++) {
		(*funcs)(x[i],a,&ymod,dyda,ma);
		sig2i=1.0/(sig[i]*sig[i]);
		dy=y[i]-ymod;
		for (j=0,l=1;l<=ma;l++) {
			if (ia[l]) {
				wt=dyda[l]*sig2i;
				for (j++,k=0,m=1;m<=l;m++)
					if (ia[m]) alpha[j][++k] += wt*dyda[m];
				beta[j] += dy*wt;
			}
		}
		*chisq += dy*dy*sig2i;
	}
	for (j=2;j<=mfit;j++)
		for (k=1;k<j;k++) alpha[k][j]=alpha[j][k];
	free_dvector(dyda,1,ma);
   */
}

void mrqmin(double x[], double y[], double sig[], int ndata, double a[], int ia[],
	int ma, double **covar, double **alpha, double *chisq,
	void (*funcs)(double, double [], double *, double [], int), double *alamda)
{
	void covsrt(double **covar, int ma, int ia[], int mfit);
//	void gaussj(double **a, int n, double **b, int m);
/*	void mrqcof(double x[], double y[], double sig[], int ndata, double a[],
		int ia[], int ma, double **alpha, double beta[], double *chisq,
		void (*funcs)(double, double [], double *, double [], int));*/
/*	int j,k,l,m;
	static int mfit;
	static double ochisq,*atry,*beta,*da,**oneda;

	if (*alamda < 0.0) {
		atry=dvector(1,ma);
		beta=dvector(1,ma);
		da=dvector(1,ma);
		for (mfit=0,j=1;j<=ma;j++)
			if (ia[j]) mfit++;
		oneda=dmatrix(1,mfit,1,1);
		*alamda=0.001;
		mrqcof(x,y,sig,ndata,a,ia,ma,alpha,beta,chisq,funcs);
		ochisq=(*chisq);
		for (j=1;j<=ma;j++) atry[j]=a[j];
	}
	for (j=0,l=1;l<=ma;l++) {
		if (ia[l]) {
			for (j++,k=0,m=1;m<=ma;m++) {
				if (ia[m]) {
					k++;
					covar[j][k]=alpha[j][k];
				}
			}
			covar[j][j]=alpha[j][j]*(1.0+(*alamda));
			oneda[j][1]=beta[j];
		}
	}
	gaussj(covar,mfit,oneda,1);
	for (j=1;j<=mfit;j++) da[j]=oneda[j][1];
	if (*alamda == 0.0) {
		covsrt(covar,ma,ia,mfit);
		free_dmatrix(oneda,1,mfit,1,1);
		free_dvector(da,1,ma);
		free_dvector(beta,1,ma);
		free_dvector(atry,1,ma);
		return;
	}
	for (j=0,l=1;l<=ma;l++)
		if (ia[l]) atry[l]=a[l]+da[++j];
	mrqcof(x,y,sig,ndata,atry,ia,ma,covar,da,chisq,funcs);
	if (*chisq < ochisq) {
		*alamda *= 0.1;
		ochisq=(*chisq);
		for (j=0,l=1;l<=ma;l++) {
			if (ia[l]) {
				for (j++,k=0,m=1;m<=ma;m++) {
					if (ia[m]) {
						k++;
						alpha[j][k]=covar[j][k];
					}
				}
				beta[j]=da[j];
				a[l]=atry[l];
			}
		}
	} else {
		*alamda *= 10.0;
		*chisq=ochisq;
	}
   */
}

void covsrt(double **covar, int ma, int ia[], int mfit)
{
	int i,j,k;
	double swap;

	for (i=mfit+1;i<=ma;i++)
		for (j=1;j<=i;j++) covar[i][j]=covar[j][i]=0.0;
	k=mfit;
	for (j=ma;j>=1;j--) {
		if (ia[j]) {
			for (i=1;i<=ma;i++) SWAP(covar[i][k],covar[i][j])
			for (i=1;i<=ma;i++) SWAP(covar[k][i],covar[j][i])
			k--;
		}
	}
}

void LevMarq(double* x, double* y, double* a, int dim_a, int dim_f,
				 double** df, double* eps, double* f,
             void (*func)(double* x, double* a, double* f, int dim_a, int num_points),
             void (*deriv)(double* x, double* a, double** df, int dim_a, int num_points))
/*
This function performs Levenberg-Marquardt minimisation.
The variables have the following meaning:
x - independent variables
y - variables to be fitted
f - function that maps x into y

*/
{
// Initial part, declaration of variables and computation of f, df, eps
int i, j, k;
double conv, prec = 1e-3, e0=0, e1,lambda = 1e-3;;
double** alfa = alloc2Ddouble(dim_a, dim_a);
double** alfa0 = alloc2Ddouble(dim_a,dim_a);
double* beta = new double [dim_f];
double** a1 = alloc2Ddouble(1, dim_a);
// Initialy, it must be prec< conv <=1. In this way, we will ensure that
// the loop will execute at least once
conv = 0.999;
func(x,a,f,dim_a,dim_f);
for(i=0;i<dim_f;i++)
	{
   eps[i] = y[i]-f[i];
   e0+=eps[i]*eps[i];
   }
// Initialisation done
FILE* dat1, *dat2, *dat3;
dat1 = fopen("beta.txt", "w");
dat2 = fopen("alfa.txt", "w");
dat3 = fopen("alfa0.txt", "w");
while(conv>prec)
	{
   if(conv<=1)
		{
		deriv(x, a, df, dim_a, dim_f);
		for(i=0;i<dim_a;i++)
			{
	   	beta[i] = 0;
	   	for(j=0;j<dim_f;j++)beta[i]+=df[j][i]*eps[j];
         fprintf(dat1, "%10.7f \n", beta[i]);
         a1[i][0]=beta[i];
		  	}
		for(i=0;i<dim_a;i++)
      	{
			for(j=0;j<dim_a;j++)
				{
			   alfa[i][j] = 0;
   			for(k=0;k<dim_f;k++)
         		alfa[i][j]+=df[k][i]*df[k][j];
	         fprintf(dat2, "%10.7f ", alfa[i][j]);
		  		}
      	fprintf(dat2, "\n");
	   	}
	   fclose(dat1);
   	fclose(dat2);
   	}

	for(i=0;i<dim_a;i++)
	for(j=0;j<dim_a;j++)alfa0[i][j] = alfa[i][j];
	for(i=0;i<dim_a;i++)alfa0[i][i]+=lambda;

// Write alfa0
	for(i=0;i<dim_a;i++)
      {
		for(j=0;j<dim_a;j++)
         fprintf(dat3, "%10.7f ", alfa0[i][j]);
      fprintf(dat3, "\n");
	   }
   fclose(dat3);
	gaussj(alfa0,dim_a,a1,1);
	for(i=0;i<dim_a;i++)a1[0][i]+=a[i];
   func(x,a,f,dim_a,dim_f);
	e1 = 0;
	for(i=0;i<dim_f;i++)
		{
	   eps[i] = y[i]-f[i];
   	e1+=eps[i]*eps[i];
	   }

   if(e1<e0)
   	{
      conv = fabs((e1-e0)/e0);
      lambda/= 10;
      e0 = e1;
      for(i=0;i<dim_a;i++)a[i]=a1[0][i];
      }
   else
   	{
      conv = 2;
      lambda*= 10;
		}
	}

}   
#undef TOL
#undef ITMAX1
#undef C
#undef R
#undef SHFT2
#undef SHFT3
#undef ITMAX
#undef CGOLD
#undef ZEPS
#undef GOLD
#undef GLIMIT
#undef TINY
#undef SHFT
#undef MOV3
#undef NRANSI
#undef SWAP
#undef GET_PSUM
#undef NMAX
/* (C) Copr. 1986-92 Numerical Recipes Software n$<39$`95.){2p49J%#4. */
