#include "CPreProcess.h"
#define SCALE 0.1
void CPreProcess::Process()
{
	cv::Mat isused = cv::Mat::zeros(1, m_data->m_CameraNum, CV_8U);
	uchar *isused_ptr = isused.ptr<uchar>(0);
#pragma omp parallel for
	for(int i=0; i<m_data->m_CampairNum; i++)
	{
		for (int j=0; j<2; j++)
		{
			if (isused_ptr[m_data->cam[i][j].camID] == 0)
			{
				CutBackground(m_data->cam[i][j]);
				isused_ptr[m_data->cam[i][j].camID] = 1;
			}
		}
	}
}

void CPreProcess::CutBackground(camera &current_cam)
{
	cv::Mat m_BackOrigin_hsv, m_FigureOrigin_hsv;
	m_BackOrigin_hsv = cv::imread(current_cam.background_name);
	m_FigureOrigin_hsv = cv::imread(current_cam.image_name);
	/////step1:resize
	cv::resize(m_FigureOrigin_hsv,	m_FigureOrigin_hsv,	cv::Size(), SCALE, SCALE);
	cv::resize(m_BackOrigin_hsv,		m_BackOrigin_hsv,		cv::Size(), SCALE, SCALE);
	cv::cvtColor(m_BackOrigin_hsv,	m_BackOrigin_hsv,		CV_BGR2HSV);
	cv::cvtColor(m_FigureOrigin_hsv,	m_FigureOrigin_hsv,	CV_BGR2HSV);

	cv::medianBlur(m_FigureOrigin_hsv, m_FigureOrigin_hsv, 3);
	vector<cv::Mat> hsv1(m_BackOrigin_hsv.channels());
	vector<cv::Mat> hsv2(m_FigureOrigin_hsv.channels());
	cv::split(m_BackOrigin_hsv,hsv1);
	cv::split(m_FigureOrigin_hsv,hsv2);

	cv::Mat hsv_figure			 = hsv2[0];
	cv::Mat hsv_s_figure		 = hsv2[1];
	cv::Mat hsv_s_background	 = hsv1[1];

	////step2:according to the range of human skin in [HSV space]
	////here get a human face only
	hsv_figure = (hsv_s_figure > 40) & (hsv_s_figure < 80) & (hsv_figure > 10) & (hsv_figure < 20);	

	/////step3:in [255-S] space
	cv::medianBlur(hsv_s_figure,		hsv_s_figure,		3);
	cv::medianBlur(hsv_s_background,	hsv_s_background,	3);

	cv::Mat hsv_s_diff;
	cv::absdiff(hsv_s_figure, hsv_s_background, hsv_s_diff);
	cv::threshold(hsv_s_diff, hsv_s_diff, 30, 255, cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
	ErodeAngDilate(hsv_s_diff);
	hsv_s_diff = 255 - hsv_s_diff;
	cv::medianBlur(hsv_s_diff,hsv_s_diff,3);

	////step4:an union of [HSV] and [255-S] space
	for (int y=0; y<hsv_s_diff.rows; y++)
	{
		uchar *p =hsv_s_diff.ptr<uchar>(y);
		uchar *q =hsv_figure.ptr<uchar>(y);
		for (int x=0; x<hsv_s_diff.cols; x++)
		{
			if ((q[x]>0) || (p[x]>0) )
				p[x] = 10;
		}
	}

	/////step5:Make up some place remained vacant
	VacantMakeup(21, 4, hsv_s_diff);

	////step6:find out the convex hull of each contour
	vector<vector<cv::Point> > contours;
	cv::findContours(hsv_s_diff, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0,0));
	vector<vector<cv::Point> > hull( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
		cv::convexHull( cv::Mat(contours[i]), hull[i], false );
	double max_area = 0;
	int max_ID = -1;
	for (int j=0; j<hull.size(); j++)
	{
		double area = cv::contourArea(hull[j]);
		if (area>max_area)
		{
			max_area = area;
			max_ID = j;
		}
	}
	hsv_s_diff.setTo(0);
	cv::drawContours( hsv_s_diff, hull, max_ID, cv::Scalar(255), CV_FILLED );

	////step8:resize back and save
	cv::resize(hsv_s_diff, hsv_s_diff, cv::Size(), 1/SCALE, 1/SCALE, CV_INTER_LINEAR);
	cv::imwrite(current_cam.mask_name, hsv_s_diff);
}

// w - Maximum number of images in a row  5 per row 
// h - Maximum number of images in a column 2 per column
// size 
void CPreProcess::cvShowManyMats(int w, int h, vector<cv::Mat> gray, int mattype)
{
	cv::Mat DispMat(  60+SingleImgH*h, 100+SingleImgW*w,mattype); 
	for(int i = 0, m = 20, n = 20; i < m_data->m_CameraNum; i++, m += (20 + SingleImgW))   
	{	
		//Align them
		if( i % w == 0 && m!= 20)  
		{  
			m = 20;  
			n+= 20 + SingleImgH;  
		}
		cv::Rect roi_rect = cv::Rect(m, n, SingleImgW, SingleImgH);
		cv::Mat roi = DispMat(roi_rect);//region of interest		
		cv::resize(gray[i], roi, cv::Size(SingleImgW,SingleImgH),0,0,CV_INTER_LINEAR);	
	}

	cv::imshow("align",DispMat);
	cv::waitKey(0);
}

void CPreProcess::VacantMakeup(int len_0, int len_1, cv::Mat &mat_input)
{
	cv::Mat kernel(cv::Size(len_0+2*len_1,1),CV_64FC1,cv::Scalar(1));
	for(int i=len_1;i<len_0+len_1;i++)
		kernel.at<uchar>(0,i) = 0;
	cv::Mat temp_input,temp_totalnum;
	mat_input.copyTo(temp_input);
	cv::filter2D(temp_input, temp_totalnum, temp_input.depth(), kernel, cv::Point(len_0/2+len_1,0), 0, cv::BORDER_DEFAULT);
	int thres = 2*(len_1-1)*10;
	for (int y=0;y<mat_input.rows;y++)
	{
		uchar *p = mat_input.ptr<uchar>(y);
		uchar *q = temp_totalnum.ptr<uchar>(y);
		for (int x=10;x<300-10;x++)
		{
			if  (p[x] == 0 && q[x] > thres)
					p[x] = 10;
		}
	}
	cv::Mat temp_totalnum2;
	cv::transpose(temp_input,temp_input);
	cv::filter2D(temp_input, temp_totalnum2, temp_input.depth(), kernel, cv::Point(len_0/2+len_1,0), 0, cv::BORDER_DEFAULT);
	cv::transpose(mat_input,mat_input);
	for (int y=0;y<mat_input.rows;y++)
	{
		uchar *p =mat_input.ptr<uchar>(y);
		uchar *q = temp_totalnum2.ptr<uchar>(y);
		for (int x=10;x<400-10;x++)
		{
			if  (p[x] == 0 && q[x] > thres)
				p[x] = 10;
		}
	}
	cv::transpose(mat_input,mat_input);
}

void CPreProcess::ErodeAngDilate(cv::Mat img)
{
	int erosion_size = 2;
	int erosion_elem = 2;
	int erosion_type;
	if (erosion_elem == 0){ erosion_type = cv::MORPH_RECT; }
	else if (erosion_elem == 1){ erosion_type = cv::MORPH_CROSS; }
	else if (erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

	cv::Mat element = cv::getStructuringElement(erosion_type,
		cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),		
		cv::Point(erosion_size, erosion_size));

	cv::morphologyEx(img, img, cv::MORPH_CLOSE, element, cv::Point(-1,-1), 2);
}