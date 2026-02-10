#include "NotchDetector.hpp"


////////////////////////////////////////////////////////////////////////////////
/** \brief Detect the notch region from the origin point cloud
  * \param cloud the origin point cloud
  * \param type the type of point cloud (0: MASTER_SIDE, 1: MASTER_BOTTOM/MASTER_TOP)
  * \retval The detected notch region from the origin point cloud
  */
ColorPointCloudPtr detectOriginNotch(ColorPointCloudPtr cloud, int type)
{
    ColorPointCloudPtr notch(new ColorPointCloud);

    //MASTER_SIDE
    if(type == 0)
    {
        std::sort(cloud->points.begin(), cloud->points.end(), [](const PointC& a, const PointC& b) 
        {
           return a.x < b.x;
        });

        size_t splitIndex = 0;
        float maxGap = 0.0;
        for (size_t i = 1; i < cloud->points.size(); ++i) 
        {
           float gap = cloud->points[i].x - cloud->points[i - 1].x;
           if (gap > maxGap) 
           {
               maxGap = gap;
               splitIndex = i;
           }
        }

        ColorPointCloudPtr cluster1(new ColorPointCloud);
        ColorPointCloudPtr cluster2(new ColorPointCloud);
        cluster1->points.insert(cluster1->points.end(), cloud->points.begin(), cloud->points.begin() + splitIndex);
        cluster2->points.insert(cluster2->points.end(), cloud->points.begin() + splitIndex, cloud->points.end());

        double sumZ1 = 0, sumZ2 = 0;
        for (const auto& p : *cluster1) 
          sumZ1 += p.z;
        for (const auto& p : *cluster2) 
          sumZ2 += p.z;
        double avgZ1 = sumZ1 / cluster1->size();
        double avgZ2 = sumZ2 / cluster2->size();

        ColorPointCloudPtr targetCluster = (avgZ1 < avgZ2) ? cluster1 : cluster2;
        ColorPointCloudPtr nonTargetCluster = (avgZ1 < avgZ2) ? cluster2 : cluster1;

        PointC minPt, maxPt;
        pcl::getMinMax3D(*targetCluster, minPt, maxPt);

        double radius = std::sqrt(std::pow(maxPt.x - minPt.x, 2) + std::pow(maxPt.z - minPt.z, 2)) / 2.0 * 4.0;
        double centerX = (minPt.x + maxPt.x) / 2.0;
        double centerZ = (minPt.z + maxPt.z) / 2.0;

        pcl::KdTreeFLANN<PointC> kdtree;
        kdtree.setInputCloud(targetCluster);
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        std::vector<std::pair<double, std::vector<PointC>>> ySegments;
        for (double y = minPt.y; y <= maxPt.y; y += radius) 
        {
           PointC searchPoint;
           searchPoint.y = y;
           searchPoint.x = centerX;
           searchPoint.z = centerZ;

           if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
           {
               std::vector<PointC> segmentPoints;
               for (int idx : pointIdxRadiusSearch) 
               {
                   segmentPoints.push_back(targetCluster->points[idx]);
               }
               double avg_z = 0.0;
               for (const auto& p : segmentPoints) 
               {
                   avg_z += p.z;
               }
               avg_z /= segmentPoints.size();
               ySegments.push_back({avg_z, segmentPoints});
           }
       }

        std::sort(ySegments.begin(), ySegments.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

       const std::vector<std::tuple<int, int, int>> colors = {
           {255, 0, 0},
           {0, 255, 0},
           {0, 0, 255}
       };

       for (size_t i = 0; i < std::min(size_t(3), ySegments.size()); ++i) {
           double y_center = ySegments[i].second[0].y;
           for (const auto& point : targetCluster->points) {
               if (std::abs(point.y - y_center) <= radius) {
                   PointC colored_point = point;
                   colored_point.r = std::get<0>(colors[i]);
                   colored_point.g = std::get<1>(colors[i]);
                   colored_point.b = std::get<2>(colors[i]);
                   notch->points.push_back(colored_point);
               }
           }
           for (const auto& point : nonTargetCluster->points) {
               if (std::abs(point.y - y_center) <= radius) {
                   PointC colored_point = point;
                   colored_point.r = std::get<0>(colors[i]);
                   colored_point.g = std::get<1>(colors[i]);
                   colored_point.b = std::get<2>(colors[i]);
                   notch->points.push_back(colored_point);
               }
           }
       }
    }
    else
    {
       PointC minPt, maxPt;
       pcl::getMinMax3D(*cloud, minPt, maxPt);
       double radius = (maxPt.x - minPt.x) / 2.0 * 7.0;
       double centerX = minPt.x + radius;
       double centerZ = (minPt.z + maxPt.z) / 2.0;
       std::vector<std::pair<double, int>> yCounts;
       pcl::KdTreeFLANN<PointC> kdtree;
       kdtree.setInputCloud(cloud);
       std::vector<int> pointIdxRadiusSearch;
       std::vector<float> pointRadiusSquaredDistance;
       for (double y = minPt.y; y <= maxPt.y; y += radius) 
       {
           int count = 0;
           PointC searchPoint;
           searchPoint.y = y;
           searchPoint.x = centerX;
           searchPoint.z = centerZ;
           if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
           {
              count = pointIdxRadiusSearch.size();
           }
           yCounts.push_back({y, count});
       }
       if (yCounts.size() > 2) 
       {
           yCounts.erase(yCounts.begin());
           yCounts.pop_back();
       }
       std::sort(yCounts.begin(), yCounts.end(), [](auto& a, auto& b) { return a.second < b.second; });
       std::vector<std::pair<double, pcl::RGB>> notchColors = {
           {yCounts[0].first, {255, 0, 0}},
           {yCounts[1].first, {0, 255, 0}},
           {yCounts[2].first, {0, 0, 255}},
       };
       for (auto& [y, color] : notchColors) 
       {
          for (auto& point : cloud->points) 
          {
            if (point.x >= centerX - radius && point.x <= centerX + radius && point.y >= y - radius && point.y <= y + radius) 
            {
              point.r = color.r;
              point.g = color.g;
              point.b = color.b;
              notch->push_back(point);
            }
          }
        }
    }
    
    PCL_INFO("        Notch detected with %zu points.\n", notch->points.size());

    return notch;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Detect the notch region from the real point cloud
  * \param cloud the real point cloud
  * \param type the type of point cloud (0: REAL_SIDE, 1: REAL_BOTTOM/REAL_TOP)
  * \retval The detected notch region from the ideal point cloud
  */
ColorPointCloudPtr detectRealNotch(ColorPointCloudPtr cloud, int type)
{
    ColorPointCloudPtr notch(new ColorPointCloud);
    if(type == 0)//REAL_SIDE
    {
        std::sort(cloud->points.begin(), cloud->points.end(), [](const PointC& a, const PointC& b) 
        {
            return a.z < b.z;
        });

        size_t splitIndex = 0;
        float maxGap = 0.0;
        for (size_t i = 1; i < cloud->points.size(); ++i) {
            float gap = cloud->points[i].z - cloud->points[i - 1].z;
            if (gap > maxGap) {
                maxGap = gap;
                splitIndex = i;
            }
        }

        ColorPointCloudPtr cluster1(new ColorPointCloud);
        ColorPointCloudPtr cluster2(new ColorPointCloud);
        for (size_t i = 0; i < splitIndex; i++) {
            cluster1->push_back(cloud->points[i]);
        }
        for (size_t i = splitIndex; i < cloud->points.size(); i++) {
            cluster2->push_back(cloud->points[i]);
        }
        double avgZ1 = std::accumulate(cluster1->begin(), cluster1->end(), 0.0, 
            [](double sum, const PointC& p) { return sum + p.z; }) / cluster1->size();
        double avgZ2 = std::accumulate(cluster2->begin(), cluster2->end(), 0.0, 
            [](double sum, const PointC& p) { return sum + p.z; }) / cluster2->size();

        ColorPointCloudPtr targetCluster = (avgZ1 > avgZ2) ? cluster1 : cluster2;
        ColorPointCloudPtr nonTargetCluster = (avgZ1 > avgZ2) ? cluster2 : cluster1;

        std::sort(targetCluster->points.begin(), targetCluster->points.end(), 
            [](const PointC& a, const PointC& b) {
                return a.y < b.y;
            });

        std::vector<size_t> gaps;
        for (size_t i = 1; i < targetCluster->size(); ++i) {
            if ((targetCluster->points[i].y - targetCluster->points[i - 1].y) > maxGap) {
                gaps.push_back(i);
            }
        }

        if (gaps.size() < 2) {
            gaps.clear();
            size_t thirdSize = targetCluster->size() / 3;
            gaps.push_back(thirdSize);
            gaps.push_back(2 * thirdSize);
        }

        ColorPointCloudPtr segment1(new ColorPointCloud);
        ColorPointCloudPtr segment2(new ColorPointCloud);
        ColorPointCloudPtr segment3(new ColorPointCloud);

        segment1->points.insert(segment1->points.end(), targetCluster->points.begin(), 
                            targetCluster->points.begin() + gaps[0]);
        segment2->points.insert(segment2->points.end(), targetCluster->points.begin() + gaps[0], 
                            targetCluster->points.begin() + gaps[1]);
        segment3->points.insert(segment3->points.end(), targetCluster->points.begin() + gaps[1], 
                            targetCluster->points.end());

        auto processSegmentWithExtendedRange = [&nonTargetCluster](
            ColorPointCloudPtr segment, 
            const std::vector<float>& colors) {
            
            float min_y = FLT_MAX;
            float max_y = -FLT_MAX;
            for (const auto& point : segment->points) {
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
            }

            float y_distance = max_y - min_y;
            float extended_min_y = min_y - (y_distance * 0.3f);
            float extended_max_y = max_y + (y_distance * 0.3f);

            ColorPointCloudPtr extended_segment(new ColorPointCloud);
            
            for (auto& point : segment->points) {
                PointC colored_point = point;
                colored_point.r = colors[0];
                colored_point.g = colors[1];
                colored_point.b = colors[2];
                extended_segment->points.push_back(colored_point);
            }

            for (const auto& point : nonTargetCluster->points) {
                if (point.y >= extended_min_y && point.y <= extended_max_y) {
                    PointC colored_point = point;
                    colored_point.r = colors[0];
                    colored_point.g = colors[1];
                    colored_point.b = colors[2];
                    extended_segment->points.push_back(colored_point);
                }
            }

            return extended_segment;
        };

        ColorPointCloudPtr extended_segment1 = 
            processSegmentWithExtendedRange(segment1, {255, 255, 0});
        ColorPointCloudPtr extended_segment2 = 
            processSegmentWithExtendedRange(segment2, {0, 255, 255});
        ColorPointCloudPtr extended_segment3 = 
            processSegmentWithExtendedRange(segment3, {255, 0, 255});

        *notch += *extended_segment1;
        *notch += *extended_segment2;
        *notch += *extended_segment3;
    }
    else
    {
      auto findContinuousRegions = [](const ColorPointCloudPtr& cloud, double minGap) 
      {
        std::vector<double> yValues;
        for (const auto& point : cloud->points) 
        {
            yValues.push_back(point.y);
        }
        std::sort(yValues.begin(), yValues.end());
        std::vector<std::pair<double, double>> regions;
        if (yValues.empty()) return regions;
        double region_start = yValues.front();
        double prev_y = yValues.front();
        for (size_t i = 1; i < yValues.size(); ++i) 
        {
            if (yValues[i] - prev_y > minGap) 
            {
                regions.push_back({region_start, prev_y});
                region_start = yValues[i];
            }
            prev_y = yValues[i];
        }
        regions.push_back({region_start, prev_y});
        return regions;
      };
      PointC minPt, maxPt;
      pcl::getMinMax3D(*cloud, minPt, maxPt);

      double radius = (maxPt.x - minPt.x) / 2.0 * 7.0;
      double centerX = minPt.x + radius;
      double centerZ = (minPt.z + maxPt.z) / 2.0;

      double minGap = radius * 0.5;
      auto continuousRegions = findContinuousRegions(cloud, minGap);
      continuousRegions.erase(
        std::remove_if(continuousRegions.begin(), continuousRegions.end(),
            [](const auto& region) 
            { 
              return (region.second - region.first) < 100.0; 
            }),
        continuousRegions.end()
      );

      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      kdtree.setInputCloud(cloud);

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      notch->width = 0;
      notch->height = 1;
      notch->is_dense = false;

      std::vector<std::pair<double, pcl::RGB>> notchColors;

      if(continuousRegions.size() == 1)
      {
        std::vector<std::pair<double, int>> yCounts;
        const auto& region = continuousRegions[0];
        for (double y = region.first; y <= region.second; y += radius) 
        {
          int count = 0;
          pcl::PointXYZRGB searchPoint;
          searchPoint.y = y;
          searchPoint.x = centerX;
          searchPoint.z = centerZ;

          if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
          {
            count = pointIdxRadiusSearch.size();
          }
          yCounts.push_back({y, count});
        }

        if (yCounts.size() > 2) {  
          yCounts.erase(yCounts.begin());  
          yCounts.pop_back();               
        }

        std::sort(yCounts.begin(), yCounts.end(), [](auto& a, auto& b) { return a.second < b.second; });
        notchColors = {
            {yCounts[0].first, {255, 255, 0}},
            {yCounts[1].first, {0, 255, 255}},
            {yCounts[2].first, {255, 0, 255}},
        };
      }
      else 
      {
        std::vector<std::vector<std::pair<double, int>>> region_y_counts(2);
        for (size_t region_idx = 0; region_idx < 2; ++region_idx) 
        {
          const auto& region = continuousRegions[region_idx];
          for (double y = region.first; y <= region.second; y += radius) 
          {
            int count = 0;
            pcl::PointXYZRGB searchPoint;
            searchPoint.y = y;
            searchPoint.x = centerX;
            searchPoint.z = centerZ;

            if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
            {
                count = pointIdxRadiusSearch.size();
            }
            region_y_counts[region_idx].push_back({y, count});
          }
        }
        const std::vector<pcl::RGB> colors = {{255, 255, 0}, {0, 255, 255}};
        for (size_t region_idx = 0; region_idx < 2; ++region_idx) 
        {
          auto& counts = region_y_counts[region_idx];
          if (!counts.empty()) 
          {
            std::sort(counts.begin(), counts.end(), 
                [](const auto& a, const auto& b) { return a.second < b.second; });
            counts.erase(counts.begin());
            notchColors.push_back({counts[0].first, colors[region_idx]});
          }
        }

      }
    
      std::vector<int> notch_point_counts;
      for (const auto& [y, color] : notchColors) 
      {
        int current_notch_count = 0;
        for (auto& point : cloud->points) 
        {
          if (point.x >= centerX - radius && point.x <= centerX + radius && point.y >= y - radius && point.y <= y + radius) 
          {
            pcl::PointXYZRGB colored_point = point;
            colored_point.r = color.r;
            colored_point.g = color.g;
            colored_point.b = color.b;
            notch->points.push_back(colored_point);
            notch->width++;
            current_notch_count++;
          }
        }
        notch_point_counts.push_back(current_notch_count);
      }
    
    }

    PCL_INFO("        Notch detected with %zu points.\n", notch->points.size());

    return notch;
}