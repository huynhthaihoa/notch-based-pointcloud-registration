#ifndef DATA_LOADER_HPP
#define DATA_LOADER_HPP

#include "utils.hpp"

ColorPointCloudPtr loadRealData(const std::string &, int, int , int );
ColorPointCloudPtr loadOriginData(const std::string &, const Eigen::Matrix4f &);

#endif