#include "CCloudOptimization.h"

#define MAX_CAM_NUM 10

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
vector<Eigen::Vector3f> CamCenter;

class _declspec(dllexport) CCloudOptimization
{
public:
	void Init(int sor_meank, double sor_stdThres, double mls_radius, CManageData *ImageData);
	void InsertPoint(cv::Mat p);
	void filter(int idx);
	void run();
private:
	int m_sor_meank;
	double m_mls_radius;
	double m_sor_stdThres;
	CManageData *m_ImageData;
};

inline void reverse(float &x) { x = -x;}


void CCloudOptimization::Init(int sor_meank, double sor_stdThres, double mls_radius, CManageData *ImageData)
{
	m_sor_meank = sor_meank;
	m_sor_stdThres = sor_stdThres;
	m_mls_radius = mls_radius;
	m_ImageData = ImageData;
	CamCenter.resize(m_ImageData->m_CampairNum);
	for (int i=0; i<m_ImageData->m_CampairNum; i++)
		cv::cv2eigen(m_ImageData->cam[i][0].CamCenter, CamCenter[i]);
}


void CCloudOptimization::InsertPoint(cv::Mat p)
{
	cloud_in->push_back(pcl::PointXYZ(p.ptr<double>(0)[0], p.ptr<double>(1)[0], p.ptr<double>(2)[0]));
}

void CCloudOptimization::filter(int idx)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_in);
	sor.setMeanK (m_sor_meank);
	sor.setStddevMulThresh (m_sor_stdThres);//face 0.3
	sor.filter (*cloud_filtered);
	printf("Initial points: %d\n",cloud_in->points.size());
// 
// 	pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter_r;
// 	filter_r.setInputCloud(cloud_filtered2);
// 	filter_r.setRadiusSearch(2);
// 	filter_r.setMinNeighborsInRadius(80);
// 	filter_r.filter(*cloud_filtered);

//	pcl::io::loadPLYFile("cloud_filter0.ply", *cloud_filtered);

// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
// 	tree->setInputCloud (cloud_filtered);
// 	std::vector<pcl::PointIndices> cluster_indices;
// 	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
// 	ec.setClusterTolerance (1);
// 	ec.setMinClusterSize (15000);
// 	ec.setMaxClusterSize (1500000);
// 	ec.setSearchMethod (tree);
// 	ec.setInputCloud (cloud_filtered);
// 	ec.extract (cluster_indices);
// 	printf("After filtering: %d\n",cloud_filtered->points.size());
// 	if (cluster_indices.size() == 0)
// 	{
// 		printf("no object found...\n");
// 		return;
// 	}
// 
// 	std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
// 	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
// 		cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
// 	cloud_cluster->width = uint32_t(cloud_cluster->points.size ());
// 	cloud_cluster->height = 1;
// 	cloud_cluster->is_dense = true;

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered;

	*cloud += *cloud_filtered;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud_filtered);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr current_cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (2.5);			//to be changed
	ne.compute (*current_cloud_normals);
	ne.setViewPoint(CamCenter[idx][0], CamCenter[idx][1], CamCenter[idx][2]);
	*cloud_normal += *current_cloud_normals;
	char name[100];
	sprintf(name, "cloud_filter%d.ply", idx);
	pcl::io::savePLYFileBinary(name, *cloud_filtered);
	cloud_in->clear();
}

void CCloudOptimization::run()
{
// 	char filename[100];
// 	for (int i=0; i<5; i++)
// 	{
// 		sprintf(filename, "cloud%d.ply", i);
// 		pcl::io::loadPLYFile(filename, cloud_in);
// 		filter();
// 	}
	
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (*cloud,centroid);
	Eigen::Vector3f m_ObjectCenter = Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
	
 	vector<vector<Eigen::Matrix3f>> R(m_ImageData->m_CampairNum);
	vector<vector<Eigen::Vector3f>> T(m_ImageData->m_CampairNum);
	for (int j=0; j<m_ImageData->m_CampairNum; j++)
	{
		R[j].resize(2);
		T[j].resize(2);
		cv::cv2eigen(m_ImageData->cam[j][0].P.colRange(0,3), R[j][0]);
		cv::cv2eigen(m_ImageData->cam[j][1].P.colRange(0,3), R[j][1]);
		cv::cv2eigen(m_ImageData->cam[j][0].P.col(3), T[j][0]);
		cv::cv2eigen(m_ImageData->cam[j][1].P.col(3), T[j][1]);
		m_ImageData->cam[j][0].bucket = new vector<int> *[m_ImageData->cam[j][0].bound.height];
		for (int i=0; i<m_ImageData->cam[j][0].bound.height; i++)
			m_ImageData->cam[j][0].bucket[i] = new vector<int> [m_ImageData->cam[j][0].bound.width];
	}
	cout<<m_ObjectCenter<<endl;
	//遍历所有点，判断所属视角，然后向参考相机投影
	vector<vector<int>> cam_indices(m_ImageData->m_CampairNum);
	for (int i=0; i<cloud->size(); i++)
	{
		//找到所属视角，仅第一个相机
		Eigen::Vector3f current_point = cloud->points[i].getVector3fMap();
		Eigen::Vector3f current_direction_normed = m_ObjectCenter-current_point;
		current_direction_normed /= current_direction_normed.norm();
		float min_value = 2;
		int cam_belongedto = 0;
		for (int j=0; j<m_ImageData->m_CampairNum; j++)
		{
			Eigen::Vector3f current_camdir = CamCenter[j]-current_point;
			float current_value = current_direction_normed.dot(current_camdir)/current_camdir.norm();
			if (min_value > current_value)
			{
				min_value = current_value;
				cam_belongedto = j;
			}
		}
		cam_indices[cam_belongedto].push_back(i);
		//投影
		Boundary &current_bound = m_ImageData->cam[cam_belongedto][0].bound;
		Eigen::Vector3f current_imgPt = R[cam_belongedto][0] * current_point + T[cam_belongedto][0];
		int current_x = ROUND(current_imgPt[0]/current_imgPt[2])-current_bound.XL;
		int current_y = ROUND(current_imgPt[1]/current_imgPt[2])-current_bound.YL;
		if (current_x < 0 || current_x >= current_bound.width || \
			current_y < 0 || current_y >= current_bound.height)
			continue;
		m_ImageData->cam[cam_belongedto][0].bucket[current_y][current_x].push_back(i);
 	}

	for (int i=0; i<m_ImageData->m_CampairNum; i++)
	{
		boost::shared_ptr<std::vector<int> > indicesptr_temp (new std::vector<int> (cam_indices[i]));
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (indicesptr_temp);
		extract.setNegative (false);
		extract.filter (*cloud_in);
		char name[100];
		sprintf(name, "cloud_filter%d.ply", i+10);
		pcl::io::savePLYFileBinary(name, *cloud_in);
	}
// 	for (int i=0; i<m_ImageData->m_CampairNum; i++)
// 	{
// 		char fname[512];
// 		sprintf(fname, "%d_pre.txt", i);
// 		FILE *fp = fopen(fname,"w");
// 		vector<camera> &current_cam = m_ImageData->cam[i];
// 		int XL = current_cam[0].bound.XL;
// 		int XR = current_cam[0].bound.XR;
// 		int YL = current_cam[0].bound.YL;
// 		int YR = current_cam[0].bound.YR;
// 		for (int y=YL; y<=YR; y++)
// 		{
// 			for (int x=XL; x<=XR; x++)
// 				fprintf(fp, "%d ", current_cam[0].bucket[y-YL][x-XL].size());
// 			fprintf(fp, "\n");
// 		}
// 		fclose(fp);
// 	}

	vector<int> indices;
	//对每个视角
	vector<pcl::Normal,Eigen::aligned_allocator<pcl::Normal>> &normal = cloud_normal->points;
	vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ>> &point = cloud->points;
	int MatchBlockRadius = 2;
	int window_size = 2*MatchBlockRadius+1;
	int vec_size = square_(window_size)*3;
	
	for (int i=0; i<m_ImageData->m_CampairNum; i++)
	{
// 		char fname[512];
// 		sprintf(fname, "%d.txt", i);
// 		FILE *fp = fopen(fname,"w");
		vector<camera> &current_cam = m_ImageData->cam[i];
		int XL = current_cam[0].bound.XL;
		int XR = current_cam[0].bound.XR;
		int YL = current_cam[0].bound.YL;
		int YR = current_cam[0].bound.YR;
//		printf("current %d\n", i);
		int count0 = 0;
		for (int y=YL; y<=YR; y++)
		{
			uchar *mask_ptr = current_cam[0].mask.ptr<uchar>(y);
			for (int x=XL; x<=XR; x++)
			{
				if (mask_ptr[x]!=255)
				{
//					fprintf(fp, "%d ", 0);
					continue;
				}
				int count = 0;
				vector<int> &current_bucket = current_cam[0].bucket[y-YL][x-XL];
				size_t candidate_size = current_bucket.size();
				switch(candidate_size)
				{
				case 0:
					break;
				case 1:
					count++;
					indices.push_back(current_bucket[0]);
					break;
				case 2:
					//如果两点法向相反，就都加进去
					if (normal[current_bucket[0]].getNormalVector3fMap().dot(		\
						normal[current_bucket[1]].getNormalVector3fMap()) < 0)
					{
						indices.push_back(current_bucket[0]);
						indices.push_back(current_bucket[1]);
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
							Eigen::Vector3f current_imgPt = R[i][1] * point[current_bucket[k]].getVector3fMap() + T[i][1];
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
							indices.push_back(current_bucket[temp_i]);
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
						Eigen::Vector3f direct = point[current_bucket[l]].getVector3fMap() - CamCenter[i];
						current_dist[l] = direct.norm();
						current_direct[l] = normal[current_bucket[l]].getNormalVector3fMap().dot(direct) < 0;
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
								Eigen::Vector3f current_imgPt = R[i][1] * point[current_bucket[current_bucket_sorted_idx[k]]].getVector3fMap() + T[i][1];
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
						indices.push_back(current_bucket[current_bucket_sorted_idx[temp_i]]);
						count++;
						last_idx = l;
					}
				}
//				fprintf(fp, "%d ", count);
			}
//			fprintf(fp, "\n");
		}
//		printf("count0: %d\n", count0);
//		fclose(fp);
	}
	


	for (int j=0; j<m_ImageData->m_CampairNum; j++)
	{
		for (int i=0; i<m_ImageData->cam[j][0].bound.height; i++)
			delete [] m_ImageData->cam[j][0].bucket[i];
		delete [] m_ImageData->cam[j][0].bucket;
	}
	boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
// 	pcl::ExtractIndices<pcl::PointXYZ> extract;
// 	extract.setInputCloud (cloud);
// 	extract.setIndices (indicesptr);
// 	extract.setNegative (false);
// 	extract.filter (*cloud_in);
	
	cout<<"cloud size: "<<cloud_in->size()<<endl<<"indices size: "<<indicesptr->size()<<endl;
	pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setNumberOfThreads(8);
	mls.setComputeNormals (true);
	mls.setInputCloud (cloud);
	mls.setIndices(indicesptr);
	mls.setSearchRadius (m_mls_radius);
	mls.setPolynomialFit (true);
	mls.setPolynomialOrder (1);
//	mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE );
//	mls.setUpsamplingRadius (1);
//	mls.setUpsamplingStepSize (0.8);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ms_normals (new pcl::PointCloud<pcl::PointNormal> ());
	mls.process (*cloud_ms_normals);
	cloud->clear();
	std::cerr << *cloud_ms_normals;
#pragma omp parallel for
	for (int i=0; i<cloud_ms_normals->size(); i++)
	{
		Eigen::Vector3f current_point = cloud_ms_normals->points[i].getVector3fMap();
		Eigen::Vector3f current_direction_normed = m_ObjectCenter-current_point;
		current_direction_normed /= current_direction_normed.norm();
		float min_value = 2;
		Eigen::Vector3f best_camdir;
		for (int j=0; j<m_ImageData->m_CampairNum; j++)
		{
			Eigen::Vector3f current_camdir = CamCenter[j]-current_point;
			float current_value = current_direction_normed.dot(current_camdir)/current_camdir.norm();
			if (min_value > current_value)
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
// 	pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
// 	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_final (new pcl::PointCloud<pcl::PointNormal> ());
// 	sor.setInputCloud (cloud_ms_normals);
// 	sor.setMeanK (m_sor_meank);
// 	sor.setStddevMulThresh (m_sor_stdThres);//face 0.3
// 	sor.filter (*cloud_final);
	printf("final points: %d\n",cloud_ms_normals->points.size());
	pcl::io::savePLYFileBinary("cloud_ms_normals.ply", *cloud_ms_normals);
}