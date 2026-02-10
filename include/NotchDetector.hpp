#ifndef NOTCH_DETECTOR_HPP
#define NOTCH_DETECTOR_HPP

#include "utils.hpp"

ColorPointCloudPtr detectOriginNotch(ColorPointCloudPtr, int);
ColorPointCloudPtr detectRealNotch(ColorPointCloudPtr, int);
#endif