#include "Merge.hpp"

////////////////////////////////////////////////////////////////////////////////
/** \brief Apply a transformation matrix to a point cloud
  * \param cloud the input point cloud
  * \param transform the transformation matrix
  * \retval The transformed point cloud
  */
ColorPointCloudPtr transformPointCloud(const ColorPointCloudPtr &cloud, const Eigen::Matrix4f &transform) 
{
    ColorPointCloudPtr transformedCloud(new ColorPointCloud);
    pcl::transformPointCloud(*cloud, *transformedCloud, transform);
    return transformedCloud;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Register three point clouds using their respective transformation matrices
  * \param sidePC the side view point cloud
  * \param bottomPC the bottom view point cloud
  * \param topPC the top view point cloud
  * \param sideTransform the transformation matrix for the side view point cloud
  * \param bottomTransform the transformation matrix for the bottom view point cloud
  * \param topTransform the transformation matrix for the top view point cloud
  * \retval The registered point cloud
  */
ColorPointCloudPtr registerPointClouds(const ColorPointCloudPtr &sidePC, const ColorPointCloudPtr &bottomPC, const ColorPointCloudPtr &topPC, const Eigen::Matrix4f &sideTransform, const Eigen::Matrix4f &bottomTransform, const Eigen::Matrix4f &topTransform)
{
    ColorPointCloudPtr transformedSide = transformPointCloud(sidePC, sideTransform);
    ColorPointCloudPtr transformedBottom = transformPointCloud(bottomPC, bottomTransform);
    ColorPointCloudPtr transformedTop = transformPointCloud(topPC, topTransform);

    ColorPointCloudPtr registeredCloud(new ColorPointCloud);
    *registeredCloud += *transformedSide;
    *registeredCloud += *transformedBottom;
    *registeredCloud += *transformedTop;

    return registeredCloud;
}