#ifndef DATA_PREPROC_HPP
#define DATA_PREPROC_HPP

#include "utils.hpp"

ColorPointCloudPtr downsamplePointCloud(const ColorPointCloudPtr &, float);
ColorPointCloudPtr denoisePointCloud(const ColorPointCloudPtr &, int, float, float, float);
ColorPointCloudPtr filterSmallXClusters(const ColorPointCloudPtr &);
std::vector<float> detectYBreakPoints(const ColorPointCloudPtr &, float);
ColorPointCloudPtr filterTop2YSegments(const ColorPointCloudPtr &, float);
ColorPointCloudPtr removeSmallComponents(const ColorPointCloudPtr &, float); 
#endif