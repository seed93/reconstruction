#ifndef SharedInclude_H
#define SharedInclude_H
// SharedInclude.h
// put common header file, libraries, defines here

// include files

#include <opencv/core.hpp>
#include <opencv/imgproc.hpp>
#include <opencv/highgui.hpp>
#include <opencv/calib3d.hpp>
#include <armadillo/armadillo>
#include <stdio.h>
#include <iostream>

// namespace
using namespace std;

//import libraries
#ifdef _DEBUG
#pragma comment (lib,"../lib/opencv/opencv_calib3d245d.lib")
#pragma comment (lib,"../lib/opencv/opencv_core245d.lib")
#pragma comment (lib,"../lib/opencv/opencv_highgui245d.lib")
#pragma comment (lib,"../lib/opencv/opencv_imgproc245d.lib")
#pragma comment (lib,"../lib/opencv/zlibd.lib")
#pragma comment (lib,"../lib/opencv/libpngd.lib")
#pragma comment (lib,"../lib/opencv/libjpegd.lib")
#pragma comment (lib,"../lib/opencv/libtiffd.lib")
#pragma comment (lib,"../lib/opencv/libjasperd.lib")
#pragma comment (lib,"../lib/opencv/IlmImfd.lib")
#else
#pragma comment (lib,"../lib/opencv/opencv_calib3d245.lib")
#pragma comment (lib,"../lib/opencv/opencv_core245.lib")
#pragma comment (lib,"../lib/opencv/opencv_highgui245.lib")
#pragma comment (lib,"../lib/opencv/opencv_imgproc245.lib")
#pragma comment (lib,"../lib/opencv/zlib.lib")
#pragma comment (lib,"../lib/opencv/libpng.lib")
#pragma comment (lib,"../lib/opencv/libjpeg.lib")
#pragma comment (lib,"../lib/opencv/libtiff.lib")
#pragma comment (lib,"../lib/opencv/libjasper.lib")
#pragma comment (lib,"../lib/opencv/IlmImf.lib")

#endif

// defines
#define MAX_CAM_NUM 10
#define square_(x) ((x)*(x))
#define ROUND(x) ((int)((x)+0.5))
#define MAX_PATH 1024
//#define IS_OUTPUT
#define IS_PLY
#define IS_PCL
#endif