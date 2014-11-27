#ifndef CCloudOptimization_h
#define CCloudOptimization_h
#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include <opencv/eigen.hpp>
#include "../reconstruction/CManageData.h"


#ifdef _DEBUG
#pragma comment (lib, "../../PCL/lib/pcl_common_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_kdtree_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_io_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_io_ply_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_surface_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_features_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_search_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_filters_debug.lib")
#pragma comment (lib, "../../PCL/lib/pcl_segmentation_debug.lib")
#else
#pragma comment (lib, "../../PCL/lib/pcl_common_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_kdtree_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_io_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_io_ply_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_surface_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_features_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_search_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_filters_release.lib")
#pragma comment (lib, "../../PCL/lib/pcl_segmentation_release.lib")
#endif


#endif