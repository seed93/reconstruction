#include "CReconstruction.h"

// Initial function
// parse input arguments
// initial data manger and stereo matching
bool CReconstrction::Init(char *configfile)
{
	cv::FileStorage fs(configfile, cv::FileStorage::READ);
	if (fs.isOpened() == false)
	{
		printf("cannot open file %s\n", configfile);
		return false;
	}
	fs["filepath"] >> filepath;
	if (m_ImageData.Init(fs) == false)
		return false;
	m_Matching.Init(&m_ImageData, &m_CloudOptimization,2,0.03);//org: 2,0.1
	m_CloudOptimization.Init(100,1,50,2,2.5,&m_ImageData,false);//¸Ä²ÎÊý, ETH:100 0.5 50 2 0.5, org:100,0.5,100,2,2.5 new: 100 1 50 2 2.5
	return true;
}
