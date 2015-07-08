#include "CKinect2Cloud.h"

CKinect2Cloud::CKinect2Cloud(char filename[])
{
	fs.open(filename, FileStorage::READ);
	if (fs.isOpened() == false)
	{
		printf("cannot open file %s\n", filename);
		return ;
	}
}

void CKinect2Cloud::run()
{
	vector<int> kinect_sn;
	fs["kinect_sn"]>>kinect_sn;
	String kinect_infoname;
	fs["kinect_info"]>>kinect_infoname;
	FileStorage fscalib(kinect_infoname, FileStorage::READ);
	if (fscalib.isOpened() == false)
	{
		printf("cannot open file %s\n", kinect_infoname.c_str());
		return ;
	}
	vector<int> bbox;
	fs["boundingbox"]>>bbox;
	for (int i=0; i<kinect_sn.size(); i++)
	{
		String currentID = to_string((_Longlong)kinect_sn[i]);
		vector<float> param;
		fscalib["DepthCameraIntrinsic-"+currentID]>>param;
		vector<String> filelist, bklist, outlist;
		fs["imagelist-"+currentID]>>filelist;
		fs["masklist-"+currentID]>>bklist;
		fs["outlist-"+currentID]>>outlist;
		Mat R1t_times_R;
		fscalib["R1t_times_R-"+currentID]>>R1t_times_R;
		for (int j=0; j<filelist.size(); j++)
		{
			Mat depthmap = imread(filelist[j], CV_LOAD_IMAGE_ANYDEPTH);
			Mat bk = imread(bklist[j], CV_LOAD_IMAGE_ANYDEPTH);
			convert(depthmap, bk, outlist[j], param, bbox, R1t_times_R);
		}
	}

}

void CKinect2Cloud::convert(Mat depthmap, Mat bk, string filename, vector<float> param, vector<int> bbox, Mat R1t_times_R)
{
 	float fx=param[0],fy=param[1],cx=param[2],cy=param[3];
	Mat cloud(depthmap.rows * depthmap.cols, 4, CV_32FC1);
	int k = 0;
	//a mask needed
	for (int y=bbox[2]; y<bbox[3]; y++)
	{
		ushort *p = depthmap.ptr<ushort>(y);
		ushort *pk = bk.ptr<ushort>(y);
		for (int x=bbox[0]; x<bbox[1]; x++)
		{
			if (p[x] == 0)
				continue;
			if (pk[x]-p[x]<= 500 || pk[x]<100)
				continue;
			float *q = cloud.ptr<float>(k);
			q[2] = (float)p[x];
			q[0] = float(x-cx)*q[2]/fx;
			q[1] = (y-cy)*q[2]/fy;
			q[3] = 1;
			k++;
		}
	}

	Mat cloud_new = cloud.rowRange(0,k)*R1t_times_R;
	FILE* fp = fopen(filename.c_str(), "wb");
	fprintf(fp, "ply\n");
	fprintf(fp, "format binary_little_endian 1.0\n");
	fprintf(fp, "element vertex %d\n", k);
	fprintf(fp, "property float x\nproperty float y\nproperty float z\n");
	fprintf(fp, "end_header\n");
	fwrite(cloud_new.data, 4, 3*k, fp);
	fclose(fp);
}