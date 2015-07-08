#include <stdio.h>
//#include <string>
#include <opencv2/core.hpp>

#ifdef _DEBUG
#pragma comment (lib,"../lib/opencv/opencv_core300d.lib")
#pragma comment (lib,"../lib/opencv/zlibd.lib")
#pragma comment (lib,"../lib/opencv/ippicvmt.lib")
#pragma comment (lib,"../lib/opencv/opencv_hal300d.lib")
#else
#pragma comment (lib,"../lib/opencv/opencv_core300.lib")
#pragma comment (lib,"../lib/opencv/zlib.lib")
#pragma comment (lib,"../lib/opencv/ippicvmt.lib")
#pragma comment (lib,"../lib/opencv/opencv_hal300.lib")
#endif

using namespace std;
//int sn[] = {13293115, 13293116, 13502663, 13502667, 13502670, 13502673, 13502680, 13502683, 13502686, 13502693};
#define EXPRESSION_NUM 2
#define CAMERA_NUM 10

int main(int Argc, char ** Argv)
{
	char config_file[100];
	if (Argc == 1)
		strcpy(config_file, "path.txt");
	else strcpy(config_file, Argv[1]);
	FILE *fp = fopen(config_file, "r");
	if (fp == NULL)
	{
		printf("unable to open file %s\n", config_file);
		return -1;
	}
	uchar campair_ptr[8] = 
	{0, 1,
	2, 3,
	4, 5, 
	7, 6};
	cv::Mat campair(4, 2, CV_8U);
	campair.data = campair_ptr;
	while (!feof(fp))
	{
		char input_path[512], output_path[512];
		fscanf(fp, "%s %s\n", input_path, output_path);
		char cmd[512];
		sprintf(cmd, "mkdir %s", output_path);
		system(cmd);
		for (int i=0; i<EXPRESSION_NUM; i++)
		{
			printf("%d %s %s\n", i, input_path, output_path);
			cv::FileStorage fs("config.yml", cv::FileStorage::WRITE);
			if (fs.isOpened()==false)
			{
				printf("%d open file config.yml failed\n", i);
				continue;
			}
			fs<<"filepath"<<input_path;
			char buffer[512];
			sprintf(buffer, "%s%d.ply", output_path, i+1);
			fs<<"outfilename"<<buffer;
			fs<<"isoutput"<<0;
			fs<<"camera_calib_name"<<"calib_camera.yml";
			fs<<"PyrmNum"<<4;
			fs<<"LowestLevelWidth"<<160;
			fs<<"LowestLevelHeight"<<240;
			vector<string> imagelist(CAMERA_NUM),masklist(CAMERA_NUM);
			for (int j=0; j<CAMERA_NUM; j++)
			{
				char buffer1[512];
				sprintf(buffer1, "%.4d_Cam%d.jpg", i+1, j);
				imagelist[j] = buffer1;
				masklist[j] = "mask\\" + imagelist[j];
			}
			fs<<"imagelist"<<imagelist;
			fs<<"masklist"<<masklist;
			fs<<"camID"<<campair;
			fs.release();
			sprintf(cmd, "reconstruction config.yml");
			system(cmd);
		}
	}
	fclose(fp);
}