#include "CManageData.h"

CManageData::CManageData()
{
	imagePyrm = NULL;
	maskPyrm = NULL;
}

CManageData::~CManageData()
{
	if (imagePyrm != NULL)
	{
		for (int i=0; i<m_PyrmNum; i++)
		{
			delete [] imagePyrm[i];
			delete [] maskPyrm[i];
		}
	}
}

// Initial function
// initial camera manager
bool CManageData::Init(cv::FileStorage fs)
{
	fs["filepath"]>>m_FilePath;
	string camera_calib_name;
	fs["camera_calib_name"]>>camera_calib_name;
	int LowestLevelWidth, LowestLevelHeight;
	fs["LowestLevelWidth"]>>LowestLevelWidth;
	fs["LowestLevelHeight"]>>LowestLevelHeight;
	m_LowestLevelSize = cv::Size(LowestLevelWidth, LowestLevelHeight);
	string camID;
	fs["camID"]>>camID;
	m_CampairNum = ((int)camID.length()+1)/3;
	cam.resize(m_CampairNum);

	vector<string> imagelist, masklist, backgroundlist;
	fs["imagelist"]>>imagelist;
	fs["masklist"]>>masklist;
	fs["backgroundlist"]>>backgroundlist;
	m_CameraNum = (int)imagelist.size();
	
	cv::FileStorage f_calib(m_FilePath+camera_calib_name, cv::FileStorage::READ);
	for (int i=0; i<m_CampairNum; i++)
	{
		cam[i].resize(2);
		for (int k=0; k<2; k++)
		{
			cam[i][k].camID = camID[i*3+k]-'0';
			cam[i][k].image_name = m_FilePath+imagelist[cam[i][k].camID];
			cam[i][k].mask_name = m_FilePath+masklist[cam[i][k].camID];
			cam[i][k].background_name = m_FilePath+backgroundlist[cam[i][k].camID];
			string currentID = to_string((_Longlong)cam[i][k].camID);
			f_calib["intrinsic-"+currentID]>>cam[i][k].MatIntrinsics;
			f_calib["extrinsic-"+currentID]>>cam[i][k].MatExtrinsics;
			cv::Mat currentCenter = -cam[i][k].MatExtrinsics.colRange(0,3).t()*cam[i][k].MatExtrinsics.col(3);
			currentCenter.convertTo(cam[i][k].CamCenter, CV_32FC1);
		}
	}

	fs["PyrmNum"]>>m_PyrmNum;
	
	cv::Mat img = cv::imread(m_FilePath+masklist[0], CV_LOAD_IMAGE_GRAYSCALE);
	m_OriginSize = img.size();
	imagePyrm = new cv::Mat *[m_PyrmNum];
	maskPyrm  = new cv::Mat *[m_PyrmNum]; 
	for (int i=0; i<m_PyrmNum; i++)
	{
		imagePyrm[i] = new cv::Mat[2];
		maskPyrm[i]  = new cv::Mat[2];
	}
	
	return true;
}

double CManageData::WindowToVec(uchar * image_ptr[], int x, int window_size, arma::vec &u)
{
	int k = 0;
	for (int j = x*3; j < (window_size+x)*3; j++)
		for (int i = 0; i < window_size; i++)
			u(k++) = image_ptr[i][j];
	u -= mean(u);
	double normu = norm(u);
	return normu==0?1:normu;
}

double CManageData::WindowToVec(cv::Mat image, int x, int y, int window_size, arma::vec &u)
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
}

bool CManageData::SaveMat(cv::Mat input, char * filename)
{
	FILE *fp = fopen(filename, "wb");
	if(fp == NULL)
	{
		fprintf(stderr, "Create file %s failed...\n%s", filename);
		return false;
	}
	fwrite(&(input.rows), 4, 1, fp);
	fwrite(&(input.cols), 4, 1, fp);
	int temp = input.channels();
	fwrite(&temp, 4, 1, fp);
	temp = (int)input.elemSize();
	fwrite(&temp, 4, 1, fp);
	for (int r=0; r<input.rows; r++)
	{
		fwrite(reinterpret_cast<const char*>(input.ptr(r)), input.cols*input.elemSize(), 1, fp);
	}
	fclose(fp);
	return true;
}
