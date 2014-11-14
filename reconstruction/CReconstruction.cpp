#include "CReconstruction.h"

// Initial function
// parse input arguments
// initial data manger and stereo matching
bool CReconstrction::Init(char *configfile)
{
	cv::FileStorage fs(configfile, cv::FileStorage::READ);
	fs["filepath"]>>filepath;
	if (fs.isOpened() == false)
	{
		printf("cannot open file %s\n", configfile);
		return false;
	}
	if (m_ImageData.Init(fs) == false)
		return false;
	m_PreProcess.m_data = &m_ImageData;
	m_Matching.Init(&m_ImageData, &m_CloudOptimization);
	m_CloudOptimization.Init(100,0.5,&m_ImageData);//改参数,100-0.5比较好
	return true;
}
