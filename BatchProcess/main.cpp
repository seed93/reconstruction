#include <stdio.h>
//#include <string>
#include <opencv/core.hpp>

#ifdef _DEBUG
#pragma comment (lib,"../lib/opencv/opencv_core245d.lib")
#pragma comment (lib,"../lib/opencv/zlibd.lib")
#else
#pragma comment (lib,"../lib/opencv/opencv_core245.lib")
#pragma comment (lib,"../lib/opencv/zlib.lib")
#endif

using namespace std;
int sn[] = {13293115, 13293116, 13502663, 13502667, 13502670, 13502673, 13502680, 13502683, 13502686, 13502693};
#define EXPRESSION_NUM 24
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
			fs<<"LowestLevelWidth"<<90;
			fs<<"LowestLevelHeight"<<120;
			vector<string> imagelist(CAMERA_NUM),masklist(CAMERA_NUM);
			for (int j=0; j<CAMERA_NUM; j++)
			{
				char buffer1[512];
				sprintf(buffer1, "%d_%d_0.jpg", i+1, sn[j]);
				imagelist[j] = buffer1;
				masklist[j] = "mask\\" + imagelist[j];
			}
			fs<<"imagelist"<<imagelist;
			fs<<"masklist"<<masklist;
			fs<<"camID"<<"14,93,68,25,70";
			fs.release();
			sprintf(cmd, "reconstruction config.yml");
			system(cmd);
		}
	}
	fclose(fp);
}