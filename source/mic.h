//	MIC.h
//	This file contains functions for the MIC corner detector
#ifndef mic_h
#define mic_h
enum{Line, Circle};
#define	WIN 3

class CMIC
{
private:
	int					**s, **v, **v1;
	int					x, y;
	int					t1, t2;		//thresholds
	unsigned char**		corner_map;
	int					N; // number of corners;

public:
	
	CMIC(int x, int y);
	~CMIC();


// methods
	void fMIC(unsigned char** p_image, unsigned char** p_corners, int* xc, int* yc, int* thresholds);
	int FindMinVar3(int**, int, int, int);
	int FindMinVar3(unsigned char**, int, int, int);
	int NewFindMinVar3(unsigned char** s, int i, int j, int th, int mode = Line);
	int NewFindMinVar3(int** s, int i, int j, int th, int mode = Line);
	int FindMinVarS3(unsigned char** s, int i, int j, int th, int mode = Line);
	int FindMinVarS3(int** s, int i, int j, int th, int mode = Line);
	int FindMinVarS3a(unsigned char** s, int i, int j, int th, int mode = Line);
	int FindMinVarS3a(int** s, int i, int j, int th, int mode = Line);
	int FindMinVar5(unsigned char**, int, int, int th=450);
	int FindMinVar5a(unsigned char**, int, int, int th=250);
	int FindMinVar7(unsigned char** s, int i, int j, int th);

	int	GetN(){return N;}
};
#endif


