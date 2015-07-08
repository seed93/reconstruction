#ifndef CReconstrction_H
#define CReconstrction_H

// CReconstrction.h
// class CReconstrction
// top level class, manage all classes

#include "CStereoMatching.h"

#define USAGE_HELP "help me"

class CReconstrction
{
public:
	CManageData m_ImageData;				// data manager
	CStereoMatching m_Matching;				// stereo matching part
	CCloudOptimization m_CloudOptimization;
	string filepath;
	bool Init(char *configfile);		// Initial function, must be called after declaim
};

#endif