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
	string image_name,mask_name;
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
	int isoutput;
	string outfilename;
	cv::Size m_OriginSize;
	cv::Mat **imagePyrm, **maskPyrm;
	CManageData();
	~CManageData();
	bool Init(cv::FileStorage fs);
	double WindowToVec(uchar * image_ptr[], int x, int window_size, arma::vec &u);	// get pixels of a window in image and return a vec
	double WindowToVec(cv::Mat image, int x, int y, int window_size, arma::vec &u)
	{
		int k = 0;
		int x3 = x*3;
		int w3 = window_size*3;
		for (int i=y; i<y+window_size; i++)
		{
			uchar *ptr = image.ptr<uchar>(i)+x3;
			for (int j=0; j<w3; j++)
				u(k++) = ptr[j];
		}
		u -= mean(u);
		double normu = norm(u);
		return normu==0?1:normu;
	};
	bool SaveMat(cv::Mat input, char * filename);
};

#endif