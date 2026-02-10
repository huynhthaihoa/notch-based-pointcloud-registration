#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/memory.h"  // for pcl::make_shared
#include "pcl/point_representation.h"

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/common/pca.h"
#include "pcl/common/centroid.h"

#include "pcl/io/pcd_io.h"

#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/statistical_outlier_removal.h"

#include "pcl/features/normal_3d.h"

#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"

#include "pcl/segmentation/extract_clusters.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

typedef pcl::Normal PointN;
typedef pcl::PointCloud<pcl::Normal> NormalPointCloud;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalPointCloudPtr;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColorPointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointCloudPtr;

//constant values
const float NULLVALUE = -999;
const float SCALEX = 0.006;
const float SCALEY = 0.1;

#endif