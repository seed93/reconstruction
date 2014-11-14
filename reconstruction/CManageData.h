#ifndef CManageData_H
#define CManageData_H

// CManageData.h
// class CManageData
// manage all common images, masks, parameters

#include "SharedInclude.h"

struct Boundary
{
	int YL,YR,XL,XR;
	int width, height;
};

struct camera
{
	cv::Mat CamCenter;
	cv::Mat P,MatIntrinsics,MatExtrinsics;
	string image_name,background_name,mask_name;
	cv::Mat image;
	cv::Mat mask;
	Boundary bound;
	vector<int> **bucket;
	int camID;
};

class CManageData
{
public:
	vector<vector<camera>> cam;
	int m_CameraNum;							// camera number
	int m_CampairNum;
	int m_PyrmNum;								// number of pyramid levels
	cv::Size m_LowestLevelSize;						// size of lowest level image
	string m_FilePath;							// input file path
	cv::Size m_OriginSize;
	cv::Mat **imagePyrm, **maskPyrm;
	CManageData();
	~CManageData();
	bool Init(cv::FileStorage fs);
	double WindowToVec(uchar * image_ptr[], int x, int window_size, arma::vec &u);	// get pixels of a window in image and return a vec
	double WindowToVec(cv::Mat image, int x, int y, int window_size, arma::vec &u);
	bool SaveMat(cv::Mat input, char * filename);
};

#endif