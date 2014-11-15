#ifndef CStereoMatching_H
#define CStereoMatching_H

// CStereoMatching.h
// class CStereoMatching
// return point clouds of all views
#include "SharedInclude.h"
#include "CManageData.h"
#define NOMATCH -10000

#ifdef _DEBUG
#pragma comment (lib,"../lib/CloudOptimization_Debug.lib")
#else
#pragma comment (lib,"../lib/CloudOptimization_Release.lib")
#endif

class __declspec(dllimport) CCloudOptimization
{
public:
	void Init(int sor_meank, double sor_stdThres, double mls_radius, CManageData *m_data);
	void InsertPoint(cv::Mat p);
	void filter(int idx);
	void run();
private:
	int m_sor_meank;
	double m_mls_radius;
	double m_sor_stdThres;
	CManageData *m_ImageData;
};

class CStereoMatching
{
public:
	CManageData * m_data;												// pointer of data manager
	CCloudOptimization * m_CloudOptimization;
	int MatchBlockRadius;												// radius of matching blocks
	double m_ws;
	int m_offset;
	cv::Mat Q,R_final,T_final;
	Boundary margin[2];
	int Verbose;
	// initial function, must be called after declaim
	void Init(CManageData * data, CCloudOptimization * CloudOptimization, int radii = 2, double ws = 0.5, int disparity_offset = 2);						
	void MatchAllLayer();
private:
	void MatchOneLayer(cv::Mat disparity[], int Pyrm_depth);
	void Rectify(int CamPair, cv::Mat &Q);		// first rectify
	void LowestLevelInitialMatch(cv::Mat image[], cv::Mat mask[], cv::Mat &disparity, bool IsZeroOne);
	void HighLevelInitialMatch(cv::Mat image[], cv::Mat mask[], cv::Mat &disparity, int Pyrm_depth, bool IsZeroOne);
	void OrderConstraint(cv::Mat disparity, bool IsZeroOne);
	void SmoothConstraint(cv::Mat disparity, bool IsZeroOne);
	template<class T>
	void UniquenessContraint(cv::Mat disparity[]);
	template<class T>
	void UniquenessContraint_(cv::Mat disparity[], bool IsZeroOne);
	void Rematch(cv::Mat image[], cv::Mat mask[], cv::Mat disparity, bool IsZeroOne);
	void DisparityRefine(cv::Mat &disparity, cv::Mat image[], int iteration, bool IsZeroOne);
	template<class T>
	void DisparityToCloud(cv::Mat disparity, cv::Mat mask, cv::Mat Q, int Pyrm_depth, bool IsZeroOne, int idx);								// disparity to point cloud
	void MedianFilter(cv::Mat &disparity, cv::Mat mask, int iteration, bool IsZeroOne);
	template<class T>
	void SetBoundary(cv::Mat disparity, cv::Mat mask, cv::Mat &boundary_L, cv::Mat &boundary_R, bool IsZeroOne);
	template<class T>
	void SetBoundary_smooth(cv::Mat disparity, cv::Mat mask, cv::Mat &boundary_L, cv::Mat &boundary_R, bool IsZeroOne);
	void FindMargin(Boundary &m, cv::Mat mask);
	void ConstructPyrm(int CamPair);
};

#endif