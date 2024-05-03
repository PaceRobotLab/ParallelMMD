void amoeba(double **p, double y[], int ndim, double ftol,
	double (*funk)(double []), int *nfunk);
void powell(double p[], double **xi, int n, double ftol, int *iter, double *fret,
	double (*func)(double []));
double brent(double ax, double bx, double cx, double (*f)(double), double tol,
	double *xmin);
double dbrent(double ax, double bx, double cx, double (*f)(double),
	double (*df)(double), double tol, double *xmin);
void mrqmin(float x[], float y[], float sig[], int ndata, float a[], int ia[],
	int ma, float **covar, float **alpha, float *chisq,
	void (*funcs)(float, float [], float *, float [], int), float *alamda);
void mrqcof(double x[], double y[], double sig[], int ndata, double a[], int ia[],
	int ma, double **alpha, double beta[], double *chisq,
	void (*funcs)(double, double [], double *, double [], int));
void LevMarq(double* x, double* y, double* a, int dim_a, int dim_f,
				 double** df, double* eps, double* f,
             void (*func)(double* x, double* a, double* f, int dim_a, int dim_f),
             void (*deriv)(double* x, double* a, double** df, int dim_a, int dim_f));   
