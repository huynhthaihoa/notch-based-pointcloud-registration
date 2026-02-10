#ifndef POSTPROC_HPP
#define POSTPROC_HPP

#include "utils.hpp"

Eigen::Matrix4f initAlign(const ColorPointCloudPtr &, const ColorPointCloudPtr &);
std::vector<ColorPointCloudPtr> splitNotchesByX(ColorPointCloudPtr ); 
std::vector<ColorPointCloudPtr> splitNotchesByZ(ColorPointCloudPtr ); 
std::pair<ColorPointCloudPtr, ColorPointCloudPtr> splitBottomNotchesByXZ(ColorPointCloudPtr );
std::vector<ColorPointCloudPtr> splitTopNotchesByXZ(ColorPointCloudPtr ); 
Eigen::Vector3f computePlaneNormal(const ColorPointCloudPtr &);
Eigen::Matrix4f refineAlign(const ColorPointCloudPtr &, const ColorPointCloudPtr &);
Eigen::Matrix4f computeSideTranslation(const ColorPointCloudPtr &, const ColorPointCloudPtr &);
Eigen::Matrix4f computeBottomTranslation(const ColorPointCloudPtr &, const ColorPointCloudPtr &);
Eigen::Matrix4f computeTopTranslation(const ColorPointCloudPtr &, const ColorPointCloudPtr &);

#endif