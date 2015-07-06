#include "CCloudOptimization.h"
#include "my_ply_interface.h"
#define MAX_CAM_NUM 10

pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal> ());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
vector<Eigen::Vector3f> CamCenter;
vector<vector<Eigen::Matrix3f>> R;
vector<vector<Eigen::Vector3f>> T;
cv::Mat current_img;
int current_img_width, current_img_height;
Eigen::Matrix3f current_R;
Eigen::Vector3f current_T;
void texture_color(float x, float y, float z, uchar rgb[]);
class _declspec(dllexport) CCloudOptimization
{
public:
	void Init(int sor_meank, double sor_stdThres, int sor_meank1, double sor_stdThres1, double mls_radius, CManageData *ImageData, bool isdelete_);
	void InsertPoint(cv::Mat p);
	void filter(int idx);
	void run();
private:
	int m_sor_meank;
	double m_mls_radius;
	int m_outrem_neighbor;
	double m_outrem_radius;
	double m_sor_stdThres;
	bool isdelete;
	CManageData *m_ImageData;
};

inline void reverse(float &x) { x = -x;}

void CCloudOptimization::Init(int sor_meank, double sor_stdThres, int outrem_neighbor, double outrem_radius, double mls_radius, CManageData *ImageData, bool isdelete_)
{
	m_sor_meank = sor_meank;
	m_sor_stdThres = sor_stdThres;
	m_outrem_neighbor = outrem_neighbor;
	m_outrem_radius = outrem_radius;
	m_mls_radius = mls_radius;
	m_ImageData = ImageData;
	isdelete = isdelete_;
	CamCenter.resize(m_ImageData->m_CampairNum);
	for (int i=0; i<m_ImageData->m_CampairNum; i++)
		cv::cv2eigen(m_ImageData->cam[i][0].CamCenter, CamCenter[i]);
	R.resize(m_ImageData->m_CampairNum);
	T.resize(m_ImageData->m_CampairNum);
	char cmd[MAX_PATH];
	sprintf(cmd, "mkdir tmp");
	system(cmd);
}


void CCloudOptimization::InsertPoint(cv::Mat p)
{
	cloud_in->push_back(pcl::PointXYZ(p.ptr<double>(0)[0], p.ptr<double>(1)[0], p.ptr<double>(2)[0]));
}

void CCloudOptimization::filter(int idx)
{
	R[idx].resize(2);
	T[idx].resize(2);
	cv::cv2eigen(m_ImageData->cam[idx][0].P.colRange(0,3), R[idx][0]);
	cv::cv2eigen(m_ImageData->cam[idx][1].P.colRange(0,3), R[idx][1]);
	cv::cv2eigen(m_ImageData->cam[idx][0].P.col(3), T[idx][0]);
	cv::cv2eigen(m_ImageData->cam[idx][1].P.col(3), T[idx][1]);
	if (isdelete)
	{
		m_ImageData->cam[idx][0].bucket = new vector<int> *[m_ImageData->cam[idx][0].bound.height];
		for (int i=0; i<m_ImageData->cam[idx][0].bound.height; i++)
			m_ImageData->cam[idx][0].bucket[i] = new vector<int> [m_ImageData->cam[idx][0].bound.width];
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_1step (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_in);
	sor.setMeanK (m_sor_meank);
	sor.setStddevMulThresh (m_sor_stdThres);//face 0.3
	sor.filter (*cloud_filtered);
	printf("Initial points: %d\n",cloud_in->points.size());
	/*
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	// build the filter
	outrem.setInputCloud(cloud_after_1step);
	outrem.setRadiusSearch(m_outrem_radius);
	outrem.setMinNeighborsInRadius (m_outrem_neighbor);
	// apply filter
	outrem.filter (*cloud_filtered);*/

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered;

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud_filtered);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr current_cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (m_mls_radius);			//to be changed
	ne.compute (*current_cloud_normals);
	ne.setViewPoint(-CamCenter[idx][0], -CamCenter[idx][1], -CamCenter[idx][2]);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*current_cloud_normals, *cloud_filtered, *cloud_normal);

#pragma omp parallel for
	for (int i=0; i<cloud_normal->size(); i++)
	{
		Eigen::Vector3f current_camdir = CamCenter[idx] - cloud_normal->points[i].getVector3fMap();
		Eigen::Vector3f current_direction_normed = cloud_normal->points[i].getNormalVector3fMap();
		if (current_direction_normed.dot(current_camdir) < 0)
		{
			reverse(cloud_normal->points[i].normal_x);
			reverse(cloud_normal->points[i].normal_y);
			reverse(cloud_normal->points[i].normal_z);
		}	
	}


	*cloud_normals += *cloud_normal;
 	char name[MAX_PATH];
 	pcl::io::savePLYFileBinary("tmp\\cloud_filter.ply", *cloud_normal);
	system("mesh.bat");
	current_img_width = m_ImageData->cam[idx][0].image.cols;
	current_img_height = m_ImageData->cam[idx][0].image.rows;
	MyPlyIo myPlyIo(texture_color, 'b');
	sprintf(name, "tmp\\color_%d_0.ply", idx);
	current_img = m_ImageData->cam[idx][0].image;
	current_R = R[idx][0];
	current_T = T[idx][0];
	if (myPlyIo.ReadAndWrite("tmp\\mesh_trimmer.ply", name))
		std::cout << "fail\n";
	sprintf(name, "tmp\\color_%d_1.ply", idx);
	current_img = m_ImageData->cam[idx][1].image;
	current_R = R[idx][1];
	current_T = T[idx][1];
	if (myPlyIo.ReadAndWrite("tmp\\mesh_trimmer.ply", name))
		std::cout << "fail\n";
	cloud_in->clear();
}

void CCloudOptimization::run()
{
// 	Eigen::Vector4f centroid;
// 	pcl::compute3DCentroid (*cloud,centroid);
// 	Eigen::Vector3f m_ObjectCenter = Eigen::Vector3f(centroid[0]-50, centroid[1], centroid[2]);
	
// 	for (int i=0; i<m_ImageData->m_CampairNum; i++)
// 	{
// 		boost::shared_ptr<std::vector<int> > indicesptr_temp (new std::vector<int> (cam_indices[i]));
// 		pcl::ExtractIndices<pcl::PointXYZ> extract;
// 		extract.setInputCloud (cloud);
// 		extract.setIndices (indicesptr_temp);
// 		extract.setNegative (false);
// 		extract.filter (*cloud_in);
// 		char name[100];
// 		sprintf(name, "cloud_filter%d.ply", i+10);
// 		pcl::io::savePLYFileBinary(name, *cloud_in);
// 	}

	boost::shared_ptr<std::vector<int> > indicesptr (new vector<int> );
	if (isdelete)
	{	
		//TODO：首先计算法向，假设法向正确，用法向和点到相机方向的夹角来判断所属相机
//		pcl::io::savePLYFileBinary("tmp\\bigcloud_normal.ply", *cloud_normals);
// 		pcl::PLYWriter w;
// 		w.write<pcl::PointNormal>("tmp\\bigcloud_normal.ply", *cloud_normals, false, false);
// 		system("gen_normals.bat");
// 		pcl::io::loadPLYFile("tmp\\bigcloud.ply", *cloud_normals);
// 		cout<<"read succeed"<<endl;
		std::cerr << "Cloud before deleting: " << std::endl;
		std::cerr << *cloud_normals;
		vector<pcl::PointNormal,Eigen::aligned_allocator<pcl::PointNormal>> &points = cloud_normals->points;
		//遍历所有点，判断所属视角，然后向参考相机投影
		int s1 = 0, s2 = 0;
		for (int i=0; i<cloud_normals->size(); i++)
		{
			//找到所属视角，仅第一个相机
			Eigen::Vector3f current_point = points[i].getVector3fMap();
			Eigen::Vector3f current_normal = points[i].getNormalVector3fMap();
			float max_value = FLT_MIN;
			int cam_belongedto = 0;
			for (int j=0; j<m_ImageData->m_CampairNum; j++)
			{
				Eigen::Vector3f current_camdir = CamCenter[j]-current_point;
				float current_value = current_normal.dot(current_camdir)/current_camdir.norm();
				if (max_value < current_value)
				{
					max_value = current_value;
					cam_belongedto = j;
				}
			}
			//投影
			Eigen::Vector3f current_imgPt = R[cam_belongedto][0] * current_point + T[cam_belongedto][0];
			camera &current_cam = m_ImageData->cam[cam_belongedto][0];
			Boundary &current_bound = current_cam.bound;
			int current_x = ROUND(current_imgPt[0]/current_imgPt[2])-current_bound.XL;
			int current_y = ROUND(current_imgPt[1]/current_imgPt[2])-current_bound.YL;
			if (current_x < 0 || current_x >= current_bound.width ||
				current_y < 0 || current_y >= current_bound.height){
				++s1;
				continue;
			}
			if (current_cam.mask.at<uchar>(current_y+current_bound.YL, current_x+current_bound.XL) == 0){
				++s2;
				continue;
			}
			current_cam.bucket[current_y][current_x].push_back(i);
		}
		cout<<"s1: "<<s1<<" s2: "<<s2<<endl;
		cout<<"assign succeed"<<endl;
		//对每个视角
// 		vector<pcl::Normal,Eigen::aligned_allocator<pcl::Normal>> &normal = cloud_normal->points;
// 		vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ>> &point = cloud->points;
		int MatchBlockRadius = 2;
		int window_size = 2*MatchBlockRadius+1;
		int vec_size = square_(window_size)*3;

		for (int i=0; i<m_ImageData->m_CampairNum; i++)
		{
			vector<camera> &current_cam = m_ImageData->cam[i];
			int XL = current_cam[0].bound.XL;
			int XR = current_cam[0].bound.XR;
			int YL = current_cam[0].bound.YL;
			int YR = current_cam[0].bound.YR;
			int count0 = 0;
			for (int y=YL; y<=YR; y++)
			{
				uchar *mask_ptr = current_cam[0].mask.ptr<uchar>(y);
				for (int x=XL; x<=XR; x++)
				{
					if (mask_ptr[x]!=255)
						continue;
					int count = 0;
					vector<int> &current_bucket = current_cam[0].bucket[y-YL][x-XL];
					size_t candidate_size = current_bucket.size();
					switch(candidate_size)
					{
					case 0:
						break;
					case 1:
						count++;
						indicesptr->push_back(current_bucket[0]);
						break;
					case 2:
						//如果两点法向相反，就都加进去
						if (points[current_bucket[0]].getNormalVector3fMap().dot(
							points[current_bucket[1]].getNormalVector3fMap()) < 0)
						{
							indicesptr->push_back(current_bucket[0]);
							indicesptr->push_back(current_bucket[1]);
							count+=2;
						}
						else
						{
							arma::vec vecL(vec_size), vecR(vec_size);
							double normL = m_ImageData->WindowToVec(current_cam[0].image, x-MatchBlockRadius, y-MatchBlockRadius, window_size, vecL);
							short temp_i = -1;
							double CurrentMaxValue = -1;
							for (int k=0; k<2; k++)
							{
								Eigen::Vector3f current_imgPt = R[i][1] * points[current_bucket[k]].getVector3fMap() + T[i][1];
								int current_x = ROUND(current_imgPt[0]/current_imgPt[2]);
								int current_y = ROUND(current_imgPt[1]/current_imgPt[2]);
								if ( current_cam[1].mask.at<uchar>(current_y, current_x) != 255)
								{
									count0++;
									continue;
								}
								double normR = m_ImageData->WindowToVec(current_cam[1].image, x-MatchBlockRadius, y-MatchBlockRadius, window_size, vecR);
								double CurrentValue = arma::dot(vecL, vecR)/(normR*normL);
								if (CurrentValue > CurrentMaxValue)
								{
									temp_i = k;
									CurrentMaxValue = CurrentValue;
								}
							}
							if (temp_i >= 0)
							{
								count++;
								indicesptr->push_back(current_bucket[temp_i]);
							}
						}
						break;
					default:
						//多于2个时先按距离从大到小排序
						vector<int> current_bucket_sorted_idx(candidate_size);
						vector<float> current_dist(candidate_size);
						//法向是否面朝相机，1表示是，0表示反向
						vector<bool> current_direct(candidate_size);
						//计算距离和朝向
						for (int l=0; l<candidate_size; l++)
						{
							Eigen::Vector3f direct = points[current_bucket[l]].getVector3fMap() - CamCenter[i];
							current_dist[l] = direct.norm();
							current_direct[l] = points[current_bucket[l]].getNormalVector3fMap().dot(direct) < 0;
						}
						for (int l=0; l<candidate_size; l++)
						{
							float Max_dist = 0;
							int max_idx = -1;
							for (int k=0; k<candidate_size; k++)
							{
								if (Max_dist < current_dist[k])
								{
									Max_dist = current_dist[k];
									max_idx = k;
								}
							}
							current_dist[max_idx] = 0;
							current_bucket_sorted_idx[l] = max_idx;
						}
						current_dist.clear();
						int last_idx = 0;
						for (int l=1; l<candidate_size; l++)
						{
							if (current_direct[current_bucket_sorted_idx[last_idx]] == 
								current_direct[current_bucket_sorted_idx[l]] && l!=candidate_size-1)
								continue;

							short temp_i = last_idx;
							if (last_idx + 1 < l)
							{
								//多于1个点方向相同，在这些点中只保留一个
								arma::vec vecL(vec_size), vecR(vec_size);
								double normL = m_ImageData->WindowToVec(current_cam[0].image, x-MatchBlockRadius, y-MatchBlockRadius, window_size, vecL);
								double CurrentMaxValue = -1;
								for (int k=last_idx; k<l; k++)
								{
									Eigen::Vector3f current_imgPt = R[i][1] * points[current_bucket[current_bucket_sorted_idx[k]]].getVector3fMap() + T[i][1];
									int current_x = ROUND(current_imgPt[0]/current_imgPt[2]);
									int current_y = ROUND(current_imgPt[1]/current_imgPt[2]);
									if ( current_cam[1].mask.at<uchar>(current_y, current_x) != 255)
									{
										count0++;
										continue;
									}
									double normR = m_ImageData->WindowToVec(current_cam[1].image, x-MatchBlockRadius, y-MatchBlockRadius, window_size, vecR);
									double CurrentValue = arma::dot(vecL, vecR)/(normR*normL);
									if (CurrentValue > CurrentMaxValue)
									{
										temp_i = k;
										CurrentMaxValue = CurrentValue;
									}
								}
							}
							indicesptr->push_back(current_bucket[current_bucket_sorted_idx[temp_i]]);
							count++;
							last_idx = l;
						}
					}
				}
			}
		}
		for (int j=0; j<m_ImageData->m_CampairNum; j++)
		{
			for (int i=0; i<m_ImageData->cam[j][0].bound.height; i++)
				delete [] m_ImageData->cam[j][0].bucket[i];
			delete [] m_ImageData->cam[j][0].bucket;
		}
		printf("delete over\n");
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ms_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;
	if (isdelete)
		pcl::copyPointCloud(*cloud_normals, *indicesptr, *cloud);
	else pcl::copyPointCloud(*cloud_normals, *cloud);
	//cloud_normals->clear();
	mls.setNumberOfThreads(8);
	mls.setComputeNormals (true);
	mls.setInputCloud (cloud);
	mls.setSearchRadius (m_mls_radius);
	mls.setPolynomialFit (true);
	mls.setPolynomialOrder (1);
	//	mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE );
	//	mls.setUpsamplingRadius (1);
	//	mls.setUpsamplingStepSize (0.8);
	mls.process (*cloud_ms_normals);
	cloud->clear();
	std::cerr << *cloud_ms_normals;
	pcl::PCLBase<pcl::PointXYZ>::PointIndicesPtr indices = mls.getCorrespondingIndices();
	vector<int> idx_in_orig = indices->indices;
	if (isdelete)
	{
#pragma omp parallel for
		for (int i=0; i<idx_in_orig.size(); i++)
		{
			if (cloud_ms_normals->points[i].getNormalVector3fMap().dot(
				cloud_normals->points[(*indicesptr)[idx_in_orig[i]]].getNormalVector3fMap()) < 0)
			{
				reverse(cloud_ms_normals->points[i].normal_x);
				reverse(cloud_ms_normals->points[i].normal_y);
				reverse(cloud_ms_normals->points[i].normal_z);
			}
		}
	}else{
#pragma omp parallel for
		for (int i=0; i<idx_in_orig.size(); i++)
		{
			if (cloud_ms_normals->points[i].getNormalVector3fMap().dot(
				cloud_normals->points[idx_in_orig[i]].getNormalVector3fMap()) < 0)
			{
				reverse(cloud_ms_normals->points[i].normal_x);
				reverse(cloud_ms_normals->points[i].normal_y);
				reverse(cloud_ms_normals->points[i].normal_z);
			}
		}
	}

/*
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*cloud_ms_normals,centroid);
	Eigen::Vector3f m_ObjectCenter = Eigen::Vector3f(centroid[0]-50, centroid[1], centroid[2]);

#pragma omp parallel for
	for (int i=0; i<cloud_ms_normals->size(); i++)
	{
		Eigen::Vector3f current_point = cloud_ms_normals->points[i].getVector3fMap();
		Eigen::Vector3f current_direction_normed = m_ObjectCenter-current_point;
		current_direction_normed /= current_direction_normed.norm();
		float min_value = FLT_MAX;
		Eigen::Vector3f best_camdir;
		for (int j=0; j<m_ImageData->m_CameraNum; j++)
		{
			Eigen::Vector3f current_camdir = CamCenter[j]-current_point;
			float current_value = current_direction_normed.dot(current_camdir)/current_camdir.norm();
			if (current_value < min_value)
			{
				min_value = current_value;
				best_camdir = current_camdir;
			}
		}
		if (cloud_ms_normals->points[i].getNormalVector3fMap().dot(best_camdir) < 0)
		{
			reverse(cloud_ms_normals->points[i].normal_x);
			reverse(cloud_ms_normals->points[i].normal_y);
			reverse(cloud_ms_normals->points[i].normal_z);
		}	
	}
	*/

	pcl::io::savePLYFileBinary("tmp\\bigcloud.ply", *cloud_ms_normals);
//	system("gen_normals.bat");
	//meshlab script
	system("meshlab.bat");

	//texture mapping
	char cmd[MAX_PATH];
	sprintf(cmd, "TextureStitcher.exe --in tmp\\bigmesh.ply --scans scans.txt --out %s --useKD --verbose", m_ImageData->outfilename.c_str());
	system(cmd);
}

void texture_color(float x, float y, float z, uchar rgb[])
{
	Eigen::Vector3f current_point;
	current_point[0] = x;
	current_point[1] = y;
	current_point[2] = z;
	Eigen::Vector3f current_imgPt = current_R * current_point + current_T;
	int current_x = ROUND(current_imgPt[0]/current_imgPt[2]);
	int current_y = ROUND(current_imgPt[1]/current_imgPt[2]);
	if (current_x < 0 || current_x >= current_img_width ||
		current_y < 0 || current_y >= current_img_height)
	{
		rgb[0] = 127;
		rgb[1] = 127;
		rgb[2] = 127;
		return ;
	}
	current_x *= 3;
	rgb[2] = current_img.ptr<uchar>(current_y)[current_x];
	rgb[1] = current_img.ptr<uchar>(current_y)[current_x+1];
	rgb[0] = current_img.ptr<uchar>(current_y)[current_x+2];
}