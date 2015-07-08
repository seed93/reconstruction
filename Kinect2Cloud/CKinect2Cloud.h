#ifndef CKinect2Cloud_H
#define CKinect2Cloud_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <stdio.h>
#include <time.h>
#include <iostream>
using namespace cv;
using namespace std;


#ifdef _DEBUG
#pragma comment (lib,"../lib/opencv/opencv_core300d.lib")
#pragma comment (lib,"../lib/opencv/opencv_highgui300d.lib")
#pragma comment (lib,"../lib/opencv/zlibd.lib")
#pragma comment (lib,"../lib/opencv/libpngd.lib")
#pragma comment (lib,"../lib/opencv/libjpegd.lib")
#pragma comment (lib,"../lib/opencv/libtiffd.lib")
#pragma comment (lib,"../lib/opencv/libjasperd.lib")
#pragma comment (lib,"../lib/opencv/IlmImfd.lib")
#pragma comment (lib,"../lib/opencv/ippicvmt.lib")
#pragma comment (lib,"../lib/opencv/opencv_hal300d.lib")
#pragma comment (lib,"../lib/opencv/opencv_imgcodecs300d.lib")
#pragma comment (lib,"../lib/opencv/libwebpd.lib")
#pragma comment (lib,"../lib/opencv/opencv_imgproc300d.lib")
#else
#pragma comment (lib,"../lib/opencv/opencv_core300.lib")
#pragma comment (lib,"../lib/opencv/opencv_highgui300.lib")
#pragma comment (lib,"../lib/opencv/zlib.lib")
#pragma comment (lib,"../lib/opencv/libpng.lib")
#pragma comment (lib,"../lib/opencv/libjpeg.lib")
#pragma comment (lib,"../lib/opencv/libtiff.lib")
#pragma comment (lib,"../lib/opencv/libjasper.lib")
#pragma comment (lib,"../lib/opencv/IlmImf.lib")
#pragma comment (lib,"../lib/opencv/ippicvmt.lib")
#pragma comment (lib,"../lib/opencv/opencv_hal300.lib")
#pragma comment (lib,"../lib/opencv/opencv_imgcodecs300.lib")
#pragma comment (lib,"../lib/opencv/libwebp.lib")
#pragma comment (lib,"../lib/opencv/opencv_imgproc300.lib")
#endif

class CKinect2Cloud
{
public:
	CKinect2Cloud(char filename[]);
	void run();
	void convert(Mat depthmap, Mat bk, string filename, vector<float> param, vector<int> bbox, Mat R1t_times_R);
private:
	FileStorage fs;
};


#endif