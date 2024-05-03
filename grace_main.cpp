#ifdef WITH_GRACE
#include <cv.h>
#include <highgui.h>
#include "cv2sim/CornerClass.h"
#include "cv2sim\StdAfx.h"
#include <thread>
#include <vector>
#include <mutex>
#include <Windows.h>

using namespace std;
using namespace cv;

extern void compareCam2Sim(unsigned char **img1,
	unsigned char **img2,
	unsigned char **img3,
	unsigned char **res, // result image for display
	int Width, int Height,
	CORNER_LIST *pCL0, CORNER_LIST *pCL1, CORNER_LIST *pCL2);

extern dMatrix AllignImageAffineM(CORNER_LIST* pCL0, CORNER_LIST* pCL1, dMatrix* A,
	int reg, int scale, int ret_matrix, int matches[][4], int *nMatches);

extern int cv2simDifference(IplImage *imgA, IplImage *imgB, IplImage *imgDiff, int threshold,
	int nMatches, int matches[][4], dMatrix *Mr, int scale);

extern int connectedcomponents(IplImage *src, CvSeq** contourPtr);

//帧
class CV2SimSmallFrame{
public:
	CV2SimSmallFrame(){}

	void makeImage2twoDImage(const Mat& bw, int index){
		vector<uchar*>& item = twoDImage[index];
		item.resize(bw.rows);

		for (int i = 0; i < bw.rows; ++i)
			item[i] = (uchar*)bw.ptr<uchar>(i);
	}

	void process(const Mat& image, int added = 0){
		cvtColor(image, bw_img, CV_BGR2GRAY);
		if (added){
			
			bw_img += added;
		}

		

		GaussianBlur(bw_img, bw_img, cv::Size(5, 5), 0, 0, cv::BORDER_REPLICATE);
		cv::pyrDown(bw_img, bw_img_half);
		cv::pyrDown(bw_img_half, bw_img_quart);

		bw_img.copyTo(bw_result);
		makeImage2twoDImage(bw_img, 0);
		makeImage2twoDImage(bw_img_half, 1);
		makeImage2twoDImage(bw_img_quart, 2);
		makeImage2twoDImage(bw_result, 3);

		compareCam2Sim(
			&twoDImage[0][0], &twoDImage[1][0], &twoDImage[2][0], &twoDImage[3][0],
			bw_img.cols, bw_img.rows, 
			&corner_list[0], &corner_list[1], &corner_list[2]);
	}

	CORNER_LIST* getCornerList(int index){
		return &corner_list[index];
	}

	Mat& getImageW(){
		return img_w;
	}

	Mat& getBWImage(){
		return bw_img;
	}

private:
	Mat bw_img;
	Mat bw_img_half;
	Mat bw_img_quart;
	Mat bw_result;
	Mat img_w;
	CORNER_LIST corner_list[3];
	vector<uchar*> twoDImage[4];
};

struct TimeStampFrame{
	Mat frameA;
	Mat frameB;
};

Mat getSimDifference(CV2SimSmallFrame& a, CV2SimSmallFrame& b){

	int scale = 1;
	int matches[1000][4], nMatches; // image to image match list
	dMatrix Mc(3, 3),	// Real camera motions if known
			Mr(3, 3);	// calculated camera motions
	Mat result;
	int g_threshold = 60;

	Mc.set2identity();	// Assume no motion was intended
	Mr.set2identity();
	Mr = AllignImageAffineM(
		a.getCornerList(0), b.getCornerList(0), 
		&Mc, 1, scale, 1, matches, &nMatches); // will print values of Mr

	CvMat cvM;
	double data[9] = { 0 };
	cvInitMatHeader(&cvM, 3, 3, CV_64FC1, data);
	for (int j = 0; j<2; j++) 
		Mr.a[j][2] /= scale;

	for (int i = 0; i<3; i++) {
		for (int j = 0; j<3; j++)
			cvSetReal2D(&cvM, i, j, (Mr.a[i][j])); 
	}

	Mat M(&cvM);
	cv::warpPerspective(a.getBWImage(), a.getImageW(), M, a.getBWImage().size());

	a.getImageW().copyTo(result);
	cv2simDifference(
		&IplImage(a.getImageW()),
		&IplImage(b.getBWImage()), 
		&IplImage(result), g_threshold, nMatches, matches, &Mr, scale);
	return result;
}

static mutex g_multiframe_lock;
static vector<TimeStampFrame> g_multiframe_list;

void worker(){
	//主要负责数据处理

	vector<CV2SimSmallFrame> cacheSSF;
	vector<Mat> cacheResult;

	while (true){

		g_multiframe_lock.lock();
		vector<TimeStampFrame> local_list = g_multiframe_list;
		g_multiframe_list.clear();
		g_multiframe_lock.unlock();

		if (cacheSSF.size() < local_list.size()*2)
			cacheSSF.resize(local_list.size()*2);

		if (cacheResult.size() < local_list.size())
			cacheResult.resize(local_list.size());

		GraceTimer timer;
		if (local_list.size() > 0){
#pragma omp parallel for num_threads(local_list.size())
			for (int i = 0; i < local_list.size(); ++i){
				CV2SimSmallFrame& fA = cacheSSF[i * 2];
				CV2SimSmallFrame& fB = cacheSSF[i * 2 + 1];
				TimeStampFrame tsf = local_list[i];

				fA.process(tsf.frameA, -50);
				//timer.step("process frameA %.2f ms\n");

				fB.process(tsf.frameB);
				//timer.step("process frameA %.2f ms\n");

				Mat result = getSimDifference(fA, fB);
				result.copyTo(cacheResult[i]);
				//timer.step("simDifference %.2f ms\n");
			}
			timer.step(format("size=%d, fee=%%.2f ms\n", local_list.size()).c_str());

			imshow("test", cacheResult[0]);
			waitKey(1);
		}
		else{
			//printf("local_list.size = %d\n", local_list.size());
			Sleep(30);
		}
	}
}

void main(){

	VideoCapture capA("video/1a.mp4");
	VideoCapture capB("video/2a.mp4");

	Mat frameA, frameB;
	CV2SimSmallFrame fA, fB;
	int numBlobs = 0;
	CvSeq *blobs = 0, *blobPtr;
	thread work(worker);

	while (capA.read(frameA) && capB.read(frameB)){

		TimeStampFrame tsf;
		resize(frameA, tsf.frameA, Size(320, 240));
		resize(frameB, tsf.frameB, Size(320, 240));

		g_multiframe_lock.lock();
		g_multiframe_list.emplace_back(tsf);
		g_multiframe_lock.unlock();

		Sleep(100);
#if 0

		//frameA = imread("../LabWallImages/box_0007.bmp");
		//frameB = imread("../LabWallImages/left_3.bmp");
		GraceTimer timer;

		fA.process(frameA, -50);
		timer.step("process frameA %.2f ms\n");

		fB.process(frameB);
		timer.step("process frameA %.2f ms\n");

		Mat result = getSimDifference(fA, fB);
		timer.step("simDifference %.2f ms\n");

		numBlobs = connectedcomponents(&IplImage(result.clone()), &blobs);
		timer.step("connectedcomponents %.2f ms\n");

		imshow("result", result);
		imshow("A", frameA);
		imshow("B", frameB);
		waitKey();
#endif
	}
	work.join();
}
#endif