#ifndef CPreProcess_H
#define CPreProcess_H
// CPreProcess.h
// Class CPreProcess
// pre process images, segmentation and convert

#include "SharedInclude.h"
#include "CManageData.h"

#define SingleImgH 300
#define SingleImgW 225

class CPreProcess
{
public:
	CManageData * m_data;												// pointer of data manager
	void Process();
private:
	void CutBackground(camera &current_cam);
	void cvShowManyMats(int w, int h, vector<cv::Mat> gray, int mattype);
	void VacantMakeup(int len_0, int len_1, cv::Mat &mat_input);
	void ErodeAngDilate(cv::Mat img);
};

#endif