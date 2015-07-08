#ifndef SharedInclude_H
#define SharedInclude_H
// SharedInclude.h
// put common header file, libraries, defines here

// include files

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <armadillo/armadillo>
#include <stdio.h>
#include <iostream>

// namespace
using namespace std;

//import libraries
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
#pragma comment (lib,"../lib/opencv/opencv_calib3d300d.lib")
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
#pragma comment (lib,"../lib/opencv/opencv_calib3d300.lib")
#endif

// defines
#define MAX_CAM_NUM 10
#define square_(x) ((x)*(x))
#define ROUND(x) ((int)((x)+0.5))
#ifndef MAX_PATH
#define MAX_PATH 1024
#endif
#define IS_PLY
#define IS_PCL
#endif