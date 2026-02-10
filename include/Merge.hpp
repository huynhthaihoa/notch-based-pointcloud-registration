#ifndef REGISTRATION_HPP
#define REGISTRATION_HPP

#include "utils.hpp"

ColorPointCloudPtr transformPointCloud(const ColorPointCloudPtr &, const Eigen::Matrix4f &);
ColorPointCloudPtr registerPointClouds(const ColorPointCloudPtr &, const ColorPointCloudPtr &, const ColorPointCloudPtr &, const Eigen::Matrix4f &, const Eigen::Matrix4f &, const Eigen::Matrix4f &);
#endif