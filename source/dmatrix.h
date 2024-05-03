#ifndef DMATRIX_H
#define DMATRIX_H

class dMatrix
{
private:
	int nrows, ncols;

public:

	dMatrix inverse();
	void set2zero()
	{
		for(int i=0;i<nrows;i++)
			for(int j=0; j<ncols; j++)
				a[i][j] = 0.0;
	}

	bool is_zero()
	{
		bool ret = true;
		for(int i=0;i<nrows;i++)
			for(int j=0; j<ncols; j++)
				if(a[i][j] != 0.0)
				{
					ret = false;
					break;
				}
		return ret;
	}

	void set2identity()
	{
		set2zero();
		for(int i=0;i<nrows;i++)a[i][i] = 1.0;
	}


    dMatrix& operator=(dMatrix A)
	{
		for(int i=0;i<nrows;i++)
			for(int j=0; j<ncols; j++)
				a[i][j] = A.a[i][j];
			return *this;
	}

	dMatrix(int nrows, int ncols);
	~dMatrix();
	int get_nrows(){return nrows;}
	int get_ncols(){return ncols;}
	double** a;	//element array
	double** get_array(){return a;}
};

dMatrix mult_matrix(dMatrix A, dMatrix B);
dMatrix *mult_matrixp(dMatrix A, dMatrix B);
dMatrix tpt2rot(double tilt, double dp, double dt);
dMatrix eulerang(double angle, char axis);

#endif