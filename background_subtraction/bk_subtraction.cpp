#include <opencv/core.hpp>
#include <opencv/imgproc.hpp>
#include <opencv/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <time.h>

// namespace
using namespace std;

#define START_ID		1
#define END_ID			24
#define BACKGROUND_ID	25
#define MAX_PATH		512
#define CROP_WIDTH		900
#define CROP_HEIGHT		1200
#define SMALL_WIDTH		150
#define SMALL_HEIGHT	200
#define EPS				1e-6

int sn[] = {13293115, 13293116, 13502663, 13502667, 13502670, 13502673, 13502680, 13502683, 13502686, 13502693};

template<class T>
inline T square(T x){return x*x;}
cv::Mat CutOneImage(const char *src_filename, cv::Mat &background_img, const char *dst_filename, bool isrotated);
void CutImagesFromList(const char *pathlist);
void help();
void MinFilter(cv::Mat src, cv::Mat &dst, cv::Mat mask, int filter_size);

//import libraries
#ifdef _DEBUG
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

int main(int Argc, char ** Argv)
{
	clock_t start = clock();
	switch(Argc)
	{
	case 1:
		CutImagesFromList("pathlist.txt");
		break;
	case 2:
		if (strlen(Argv[1]) > 4)
			if (strcmp(Argv[1]+strlen(Argv[1])-4,".txt") == 0)
				CutImagesFromList(Argv[1]);
		help();
		break;
	case 4:{
		cv::Mat current_background = cv::imread(Argv[2]);
		if (current_background.empty() == true || current_background.channels() != 3)
		{
			printf("unable to open file %s\n", Argv[2]);
			return -1;
		}
		cv::resize(current_background, current_background, cv::Size(CROP_WIDTH, CROP_HEIGHT));
		current_background.convertTo(current_background, CV_32FC3);
		current_background *= 1./255;
		CutOneImage(Argv[1], current_background, Argv[3], false);
		break;}
	default:
		help();
	}
	printf("total time: %f s\n", double(clock()-start)/1000);
	return 1;
}

cv::Mat CutOneImage(const char *src_filename, cv::Mat &background_img, const char *dst_filename, bool isrotated)
{
	clock_t start =  clock();
	cv::Mat small_cut;
	cv::Mat img = cv::imread(src_filename);
	if (img.empty() == true || img.channels() != 3)
	{
		printf("unable to open file %s\n", src_filename);
		return small_cut;
	}

	if (isrotated)
	{
		cv::flip(img,img,-1);
//		cv::imwrite(src_filename, img);
	}
	cv::Size org_size = img.size();
	cv::resize(img, img, cv::Size(CROP_WIDTH, CROP_HEIGHT));
	img.convertTo(img, CV_32FC3);
	img *= 1./255;
	int height = img.rows;
	int width   = img.cols;
	cv::Mat delta(height, width, CV_32FC1);
	cv::Mat S(height, width, CV_32FC1);
	for (int y = 0; y < height; y++)
	{
		float *img_ptr		= img.ptr<float>(y);
		float *bk_ptr		= background_img.ptr<float>(y);
		float *delta_ptr	= delta.ptr<float>(y);
		float *S_pt			= S.ptr<float>(y);
		size_t count = 0;
		for (int x = 0; x < width; x++)
		{
			float current_mean = (img_ptr[count]+img_ptr[count+1]+img_ptr[count+2])/3-float(EPS);
			float current_mean2 = (bk_ptr[count]+bk_ptr[count+1]+bk_ptr[count+2])/3-float(EPS);
			float sum1,sum2;
			delta_ptr[x]	= abs(img_ptr[count]-bk_ptr[count]);
			S_pt[x]			= square(img_ptr[count]-current_mean);
			sum1			= (img_ptr[count]-current_mean)*(bk_ptr[count]-current_mean2);
			sum2			= square(bk_ptr[count]-current_mean2);
			count++;
			delta_ptr[x]	+= abs(img_ptr[count]-bk_ptr[count]);
			S_pt[x]			+= square(img_ptr[count]-current_mean);
			sum1			+= (img_ptr[count]-current_mean)*(bk_ptr[count]-current_mean2);
			sum2			+= square(bk_ptr[count]-current_mean2);
			count++;
			delta_ptr[x]	+= abs(img_ptr[count]-bk_ptr[count]);
			S_pt[x]			+= square(img_ptr[count]-current_mean);
			sum1			+= (img_ptr[count]-current_mean)*(bk_ptr[count]-current_mean2);
			sum2			+= square(bk_ptr[count]-current_mean2);
			S_pt[x] = sum1/sqrt(S_pt[x]*sum2);
			count++;
		}
	}
	cv::Mat S2 = S < 0.4;

	vector<vector<cv::Point> > contours;
	cv::findContours(S2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0,0));
	double max_area = 0;
	int max_ID = -1;
	for (int i=0; i<contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area>max_area)
		{
			max_area = area;
			max_ID = i;
		}
	}
	cv::Mat mask(height, width, CV_8UC1);
	mask.setTo(255);
	cv::drawContours( mask, contours, max_ID, cv::Scalar(0), CV_FILLED );
	S.setTo(1, mask);
	mask = mask == 0;
	MinFilter(S, S, mask, 5);
	S2 = S < 0.6;
	mask = delta < 0.05;
	S2.setTo(0, mask);
	cv::findContours(S2, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0,0));
	max_area = 0;
	max_ID = -1;
	for (int i=0; i<contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);
		if (area>max_area)
		{
			max_area = area;
			max_ID = i;
		}
	}
	mask.setTo(0);
	cv::drawContours( mask, contours, max_ID, cv::Scalar(255), CV_FILLED );
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(6,6));
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
	element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size(15,15));
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);

	img *= 255;
	img.copyTo(small_cut, mask);
	cv::resize(small_cut, small_cut, cv::Size(SMALL_WIDTH, SMALL_HEIGHT));
	cv::resize(mask, mask, org_size);
	cv::imwrite(dst_filename, mask);
	printf("%s succeed %d ms\n", src_filename, clock()-start);
	return small_cut;
}

void CutImagesFromList(const char *pathlist)
{
	FILE *fp = fopen(pathlist, "r");
	if (fp == NULL)
	{
		printf("unable to open file %s\n", pathlist);
		return ;
	}
	char pathname[MAX_PATH], small_verify_name[MAX_PATH], cmd[MAX_PATH];
	char background_filename[MAX_PATH];
	char rotated[10];
	bool isrotated[10];
	fscanf(fp, "%s\n", rotated);
	for (int i=0; i<10; i++)
		isrotated[i] = rotated[i] == '1';

	while(!feof(fp))
	{
		fscanf(fp, "%s\n", pathname);
		cv::Mat small_verify_img(10*SMALL_HEIGHT, (END_ID-START_ID+1)*SMALL_WIDTH, CV_8UC3);
		sprintf(cmd, "mkdir %s\\mask", pathname);
		if (system(cmd) == -1)
		{
			printf("%s failed\n", cmd);
			fclose(fp);
			return ;
		}
		for (int num=0; num<10; num++)
		{
			sprintf(background_filename, "%s\\%d_%d_0.jpg", pathname, BACKGROUND_ID, sn[num]);
			cv::Mat current_background = cv::imread(background_filename);
			if (current_background.empty()==true || current_background.channels() != 3)
			{
				printf("unable to open file %s\n", background_filename);
				fclose(fp);
				return ;
			}
			if (isrotated[num])
				cv::flip(current_background, current_background, -1);
			cv::resize(current_background, current_background, cv::Size(CROP_WIDTH, CROP_HEIGHT));
			current_background.convertTo(current_background, CV_32FC3);
			current_background *= 1./255;
#pragma omp parallel for	
			for (int i=START_ID; i<=END_ID; i++)
			{
				char src_filename[MAX_PATH], dst_filename[MAX_PATH];
				sprintf(src_filename, "%s\\%d_%d_0.jpg", pathname, i, sn[num]);
				sprintf(dst_filename, "%s\\mask\\%d_%d_0.jpg", pathname, i, sn[num]);
				cv::Mat current_small_cut = CutOneImage(src_filename, current_background, dst_filename, isrotated[num]);
				if (current_small_cut.empty() == true)
					continue;
				current_small_cut.copyTo(small_verify_img(
					cv::Range(num*SMALL_HEIGHT, (num+1)*SMALL_HEIGHT),
					cv::Range((i-1)*SMALL_WIDTH, i*SMALL_WIDTH)));
			}
		}
		sprintf(small_verify_name, "%s\\verify.jpg", pathname);
		cv::imwrite(small_verify_name, small_verify_img);
	}
	fclose(fp);
}

void help()
{
	printf("Usage:\n \
			1. background_subtraction.exe (default: pathlist.txt)\n	\
			2. background_subtraction.exe pathlist.txt\n			\
			3. background_subtraction.exe image background mask\n	");
}

void MinFilter(cv::Mat src, cv::Mat &dst, cv::Mat mask, int filter_size)
{
	if ((filter_size & 1) == 0 )
	{
		printf("filter size must be even number\n");
		return ;
	}
	int radii = filter_size/2;
	int square_filter_size = square(filter_size);
	cv::Mat temp(src.size(), CV_32FC1);
	temp.setTo(1);
#pragma omp parallel for
	for( int y = radii; y < src.rows-radii; y++ )
	{
		float ** window_ptr = new float *[filter_size];
		uchar * mask_ptr = mask.ptr<uchar>(y);
		float * q = temp.ptr<float>(y);
		for (int i=-radii; i<=radii; i++)
		{
			window_ptr[i+radii] = src.ptr<float>(y+i);
		}
		for( int x = radii; x < src.cols-radii; x++ )
		{
			if (mask_ptr[x] != 255)
				continue;
			float min_value = 2;
			int x_radii = x+radii;
			for (int i=-radii; i<=radii; i++)
				for (int j=-radii; j<=radii; j++)
					if (window_ptr[i+radii][x_radii+j] < min_value)
						min_value = window_ptr[i+radii][x_radii+j];
			q[x] = min_value;
			
		}
		delete [] window_ptr;
	}
	dst = temp;
}