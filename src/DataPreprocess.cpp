#include "DataPreprocess.hpp"

////////////////////////////////////////////////////////////////////////////////
/** \brief Downsample the master (reference) point cloud
  * \param cloud the master (reference) point cloud
  * \param leafSize the leaf size for downsampling
  * \retval The downsampled master point cloud
  */
ColorPointCloudPtr downsamplePointCloud(const ColorPointCloudPtr &cloud, float leafSize)
{
    ColorPointCloudPtr downsampledCloud(new ColorPointCloud);
    pcl::VoxelGrid<PointC> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.filter(*downsampledCloud);

    PCL_INFO("          Downsampled cloud contains %zu points.\n", downsampledCloud->points.size());

    return downsampledCloud;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Denoise the real point cloud
  * \param cloud the real point cloud
  * \param stdDevMulThresh the standard deviation multiplier threshold for outlier removal
  * \param minHeightRatio the minimum height ratio for filtering small components
  * \param ySegmentThres the threshold for filtering Y segments
  * \retval The denoised real point cloud
  */
ColorPointCloudPtr denoisePointCloud(const ColorPointCloudPtr &cloud, int meanK, float stdDevMulThresh, float minHeightRatio, float ySegmentThres)
{
    ColorPointCloudPtr denoisedCloud(new ColorPointCloud);

    if(meanK > 0 && stdDevMulThresh > 0)
    {
        // //step 1: preliminary denoise - Statistical Outlier Removal
        pcl::StatisticalOutlierRemoval<PointC> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(stdDevMulThresh);
        sor.filter(*denoisedCloud);
        PCL_INFO("          After SOR, cloud contains %zu points.\n", denoisedCloud->points.size());
    }
    else
        *denoisedCloud = *cloud;

    if(ySegmentThres >= 0)
    {
        //step 2: filter small X clusters
        denoisedCloud = filterSmallXClusters(denoisedCloud);

        //step 3: filter top 2 Y segments
        denoisedCloud = filterTop2YSegments(denoisedCloud, ySegmentThres);
    }

    //step 4: remove small components
    denoisedCloud = removeSmallComponents(denoisedCloud, minHeightRatio);

    PCL_INFO("          Denoised cloud contains %zu points.\n", denoisedCloud->points.size());

    return denoisedCloud;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Filter out clusters with small width along the X-axis
  * \param cloud the input point cloud
  * \retval The filtered point cloud with small X clusters removed
  */
ColorPointCloudPtr filterSmallXClusters(const ColorPointCloudPtr &cloud)
{
    //determine the range of x, y, z value
    PointC minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    //determine the "width threshold" (following Ox-axis)
    float widthThreshold = (maxPt.x - minPt.x) * 0.3;

    pcl::search::KdTree<PointC>::Ptr tree(new pcl::search::KdTree<PointC>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointC> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    ColorPointCloudPtr filteredCloud(new ColorPointCloud);

    for (const auto& indices : clusterIndices) {
        ColorPointCloudPtr cluster(new ColorPointCloud);
        for (int idx : indices.indices) {
            cluster->push_back(cloud->points[idx]);
        }
        PointC minC, maxC;
        pcl::getMinMax3D(*cluster, minC, maxC);
        float clusterWidth = maxC.x - minC.x;
        if (clusterWidth >= widthThreshold) {
            *filteredCloud += *cluster;
        }
    }
    return filteredCloud;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Detect break points in Y-axis based on gaps exceeding a threshold
  * \param cloud the input point cloud
  * \param threshold the gap threshold to identify break points
  * \retval A vector of Y values representing the break points
*/
std::vector<float> detectYBreakPoints(const ColorPointCloudPtr& cloud, float threshold)
{
    std::vector<float> yValues;
    for (const auto& point : cloud->points) {
        yValues.push_back(point.y);
    }
    std::sort(yValues.begin(), yValues.end());
    std::vector<std::pair<float, float>> y_diffs;
    for (size_t i = 1; i < yValues.size(); i++) {
        float diff = yValues[i] - yValues[i - 1];
        y_diffs.push_back({yValues[i], diff});
    }
    std::vector<float> breakPoints;
    for (const auto& [y, diff] : y_diffs) {
        if (diff > threshold) {
            breakPoints.push_back(y);
        }
    }
    return breakPoints;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Filter the point cloud to retain only the top 2 largest Y segments
  * \param cloud the input point cloud
  * \param threshold the gap threshold to identify Y segments
  * \retval The filtered point cloud containing only the top 2 Y segments
  */
ColorPointCloudPtr filterTop2YSegments(const ColorPointCloudPtr& cloud, float threshold) 
{
    std::vector<float> breakPoints = detectYBreakPoints(cloud, threshold);
    std::vector<std::pair<float, float>> ySegments;
    if (breakPoints.empty()) 
    {
        PointC minPt, maxPt;
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        ySegments.push_back({minPt.y, maxPt.y});
    } 
    else 
    {
        PointC minPt, maxPt;
        pcl::getMinMax3D(*cloud, minPt, maxPt);
        ySegments.push_back({minPt.y, breakPoints[0]});
        for (size_t i = 0; i < breakPoints.size() - 1; i++) 
        {
            ySegments.push_back({breakPoints[i], breakPoints[i + 1]});
        }
        ySegments.push_back({breakPoints.back(), maxPt.y});
    }
    std::vector<std::pair<float, std::pair<float, float>>> segmentLengths;
    for (const auto& segment : ySegments) 
    {
        float length = segment.second - segment.first;
        segmentLengths.push_back({length, segment});
    }
    std::sort(segmentLengths.begin(), segmentLengths.end(), [](auto& a, auto& b) 
    {
        return a.first > b.first;
    });
    if (segmentLengths.size() > 2) 
    {
        segmentLengths.resize(2);
    }
    ColorPointCloudPtr filteredCloud(new ColorPointCloud);
    for (const auto& [length, segment] : segmentLengths) 
    {
        float y_min = segment.first;
        float y_max = segment.second;
        for (const auto& point : cloud->points) 
        {
            if (point.y >= y_min && point.y <= y_max) 
            {
                filteredCloud->push_back(point);
            }
        }
    }
    return filteredCloud;
}

///////////////////////////////////////////////////////////////////////////////
/** \brief Remove small connected components from the point cloud
  * \param cloud the input point cloud
  * \param minHeightRatio the minimum height ratio to retain a component
  * \retval The filtered point cloud with small components removed
  */
ColorPointCloudPtr removeSmallComponents(const ColorPointCloudPtr& cloud, float minHeightRatio) 
{
    pcl::search::KdTree<PointC>::Ptr tree(new pcl::search::KdTree<PointC>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointC> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    std::vector<std::pair<ColorPointCloudPtr, float>> clustersWithMetrics;
    float maxHeight = 0;
    for (const auto& indices : clusterIndices) 
    {
        ColorPointCloudPtr cluster(new ColorPointCloud);
        for (const auto& idx : indices.indices) 
        {
            cluster->push_back(cloud->points[idx]);
        }
        PointC minPt, maxPt;
        pcl::getMinMax3D(*cluster, minPt, maxPt);
        float height = maxPt.y - minPt.y;
        maxHeight = std::max(maxHeight, height);

        clustersWithMetrics.push_back({cluster, height});
    }
    ColorPointCloudPtr filteredCloud(new ColorPointCloud);
    for (const auto& [cluster, height] : clustersWithMetrics) 
    {
        if (height >= maxHeight * minHeightRatio)
        {
            *filteredCloud += *cluster;
        }
    }
    return filteredCloud;
}