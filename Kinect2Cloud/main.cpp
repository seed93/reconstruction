#include "CKinect2Cloud.h"

int main(int argc, char **argv) {
	char config_file[100];
	if (argc == 1)
		strcpy(config_file, "filelist.yml");
	else strcpy(config_file, argv[1]);
	CKinect2Cloud Kinect2Cloud(config_file);
	Kinect2Cloud.run();
	return 0;
}