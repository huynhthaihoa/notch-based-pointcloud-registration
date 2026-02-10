#include "Registration.hpp"

////////////////////////////////////////////////////////////////////////////////
/** \brief Determine the transformation matrix between real and origin notches based on PCA
  * \param realNotches the real point cloud
  * \param notches the origin (reference) point cloud
  * \retval The transformation matrix between the two point clouds
  */
Eigen::Matrix4f initAlign(const ColorPointCloudPtr &realNotches, const ColorPointCloudPtr &originNotches)
{
    pcl::NormalEstimation<PointC, PointN> normalEst;
    pcl::search::KdTree<PointC>::Ptr tree(new pcl::search::KdTree<PointC>());
    
    NormalPointCloudPtr realNormal(new NormalPointCloud());
    NormalPointCloudPtr originNormal(new NormalPointCloud());
    
    normalEst.setSearchMethod(tree);
    normalEst.setKSearch(30);

    normalEst.setInputCloud(realNotches);
    normalEst.compute(*realNormal);
    
    normalEst.setInputCloud(originNotches);
    normalEst.compute(*originNormal);
    
    Eigen::Vector4f realCentroid, originCentroid;
    pcl::compute3DCentroid(*realNotches, realCentroid);
    pcl::compute3DCentroid(*originNotches, originCentroid);

    Eigen::Matrix3f realCov, originCov;
    pcl::computeCovarianceMatrixNormalized(*realNotches, realCentroid, realCov);
    pcl::computeCovarianceMatrixNormalized(*originNotches, originCentroid, originCov);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> sourceEigen(realCov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> targetEigen(originCov);
    
    Eigen::Matrix4f initialTF = Eigen::Matrix4f::Identity();
    initialTF.block<3,3>(0,0) = targetEigen.eigenvectors() * sourceEigen.eigenvectors().transpose();
    initialTF.block<3,1>(0,3) = originCentroid.head<3>() - (initialTF.block<3,3>(0,0) * realCentroid.head<3>());
    
    return initialTF;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Split detected notches into 3 clusters based on 2 largest gaps along X axis
  * \param notches the detected notch point cloud
  * \retval A pair of point cloud clusters
*/
std::vector<ColorPointCloudPtr> splitNotchesByX(ColorPointCloudPtr notches)
{
    if (notches->points.empty()) 
    {
        std::cerr << "Error: Input cloud is empty!" << std::endl;
        return {nullptr, nullptr, nullptr};
    }
    std::sort(notches->points.begin(), notches->points.end(),
              [](const PointC& a, const PointC& b) {
                  return a.x < b.x;
              });

    size_t firstIndex = 0;
    float firstGap = 0.0;

    size_t secondIndex = 0;
    float secondGap = 0.0;

    for (size_t i = 1; i < notches->points.size(); ++i) 
    {
        float gap = notches->points[i].x - notches->points[i - 1].x;
        if (gap > firstGap) 
        {
            secondGap = firstGap;
            firstGap = gap;
            secondIndex = firstIndex;
            firstIndex = i;
        } 
        else if (gap > secondGap) 
        {
            secondGap = gap;
            secondIndex = i;
        }
    }

    ColorPointCloudPtr cluster1(new ColorPointCloud);
    ColorPointCloudPtr cluster2(new ColorPointCloud);
    ColorPointCloudPtr cluster3(new ColorPointCloud);
    for (size_t i = 0; i < firstIndex; ++i) 
    {
        PointC p = notches->points[i];
        p.r = 0;
        p.g = 255;
        p.b = 255;
        cluster1->push_back(p);
    }
    for (size_t i = firstIndex; i < secondIndex; ++i) 
    {
        PointC p = notches->points[i];
        p.r = 255;
        p.g = 255;
        p.b = 0;
        cluster2->push_back(p);
    }
    for (size_t i = secondIndex; i < notches->points.size(); ++i) 
    {
        PointC p = notches->points[i];
        p.r = 255;
        p.g = 0;
        p.b = 255;
        cluster3->push_back(p);
    }

    return {cluster1, cluster2, cluster3};
}

// /////////////////////////////////////////////////////////////////////////////////
// /** \brief Split notches into 3 clusters based on 2 largest gaps along Z axis
//   * \param cluster the input point cloud cluster
//   * \retval A pair of point cloud clusters
// */
std::vector<ColorPointCloudPtr> splitNotchesByZ(ColorPointCloudPtr cluster) 
{
    std::sort(cluster->points.begin(), cluster->points.end(),
              [](const PointC& a, const PointC& b) {
                  return a.z < b.z;
              });

    size_t splitIndex1 = 0;
    float maxGap1 = 0.0;
    size_t splitIndex2 = 0;
    float maxGap2 = 0.0;

    for (size_t i = 1; i < cluster->points.size(); ++i) 
    {
        float gap = cluster->points[i].z - cluster->points[i - 1].z;
        if (gap > maxGap1) 
        {
            maxGap2 = maxGap1;
            splitIndex2 = splitIndex1;
            maxGap1 = gap;
            splitIndex1 = i;
        } 
        else if (gap > maxGap2) 
        {
            maxGap2 = gap;
            splitIndex2 = i;
        }
    }

    if (splitIndex1 > splitIndex2) 
        std::swap(splitIndex1, splitIndex2);

    ColorPointCloudPtr cluster1(new ColorPointCloud);
    ColorPointCloudPtr cluster2(new ColorPointCloud);
    ColorPointCloudPtr cluster3(new ColorPointCloud);

    for (size_t i = 0; i < splitIndex1; ++i) 
    {
        cluster1->push_back(cluster->points[i]);
    }
    for (size_t i = splitIndex1; i < splitIndex2; ++i) 
    {
        cluster2->push_back(cluster->points[i]);
    }
    for (size_t i = splitIndex2; i < cluster->points.size(); ++i) 
    {
        cluster3->push_back(cluster->points[i]);
    }

    return {cluster1, cluster2, cluster3};
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Split detected notches from real bottom point cloud into two clusters based on largest gap in the XZ plane
  * \param notches the detected notch point cloud
  * \retval A pair of point cloud clusters
*/
std::pair<ColorPointCloudPtr, ColorPointCloudPtr> splitBottomNotchesByXZ(ColorPointCloudPtr notches) 
{
    if (notches->points.empty()) 
    {
        std::cerr << "Error: Input cloud is empty!" << std::endl;
        return {nullptr, nullptr};
    }
    std::sort(notches->points.begin(), notches->points.end(),
              [](const PointC& a, const PointC& b) {
                  return (a.x == b.x) ? (a.z < b.z) : (a.x < b.x);
              });

    size_t splitIndex = 0;
    float maxDistance = 0.0;
    for (size_t i = 1; i < notches->points.size(); ++i) 
    {
        float dx = notches->points[i].x - notches->points[i - 1].x;
        float dz = notches->points[i].z - notches->points[i - 1].z;
        float distance = std::sqrt(dx * dx + dz * dz);
        if (distance > maxDistance) 
        {
            maxDistance = distance;
            splitIndex = i;
        }
    }

    ColorPointCloudPtr cluster1(new ColorPointCloud);
    ColorPointCloudPtr cluster2(new ColorPointCloud);
    for (size_t i = 0; i < splitIndex; ++i) 
    {
        PointC p = notches->points[i];
        p.r = 0;
        p.g = 0;
        p.b = 255;
        cluster1->push_back(p);
    }
    for (size_t i = splitIndex; i < notches->points.size(); ++i) 
    {
        PointC p = notches->points[i];
        p.r = 255;
        p.g = 0;
        p.b = 0;
        cluster2->push_back(p);
    }
    return {cluster1, cluster2};
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Split detected notches from real top point cloud into 3 clusters based on 2 largest gaps in the XZ plane
  * \param notches the detected notch point cloud
  * \retval A pair of point cloud clusters
*/
std::vector<ColorPointCloudPtr> splitTopNotchesByXZ(ColorPointCloudPtr notches) 
{
    if (notches->points.empty()) 
    {
        std::cerr << "Error: Input cloud is empty!" << std::endl;
        return {nullptr, nullptr};
    }
    std::sort(notches->points.begin(), notches->points.end(),
              [](const PointC& a, const PointC& b) {
                  return (a.x == b.x) ? (a.z < b.z) : (a.x < b.x);
              });

    size_t firstIndex = 0;
    float firstDistance = 0.0;

    size_t secondIndex = 0;
    float secondDistance = 0.0;

    for (size_t i = 1; i < notches->points.size(); ++i) 
    {
        float dx = notches->points[i].x - notches->points[i - 1].x;
        float dz = notches->points[i].z - notches->points[i - 1].z;
        float distance = std::sqrt(dx * dx + dz * dz);
        if (distance > firstDistance) 
        {
            secondDistance = firstDistance;
            firstDistance = distance;
            secondIndex = firstIndex;
            firstIndex = i;
        } 
        else if (distance > secondDistance) 
        {
            secondDistance = distance;
            secondIndex = i;
        }
    }

    ColorPointCloudPtr cluster1(new ColorPointCloud);
    ColorPointCloudPtr cluster2(new ColorPointCloud);
    ColorPointCloudPtr cluster3(new ColorPointCloud);
    for (size_t i = 0; i < firstIndex; ++i) 
    {
        PointC p = notches->points[i];
        p.r = 0;
        p.g = 255;
        p.b = 255;
        cluster1->push_back(p);
    }
    for (size_t i = firstIndex; i < secondIndex; ++i) 
    {
        PointC p = notches->points[i];
        p.r = 255;
        p.g = 255;
        p.b = 0;
        cluster2->push_back(p);
    }
    for (size_t i = secondIndex; i < notches->points.size(); ++i) 
    {
        PointC p = notches->points[i];
        p.r = 255;;
        p.g = 0;
        p.b = 255;
        cluster3->push_back(p);
    }
    return {cluster1, cluster2, cluster3};  
}

///////////////////////////////////////////////////////////////////////////////
/** \brief Compute the normal vector of a point cloud plane using PCA
  * \param cloud the input point cloud
  * \retval The normal vector of the plane
  */
Eigen::Vector3f computePlaneNormal(const ColorPointCloudPtr &cloud) 
{
    pcl::PCA<PointC> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    return eigenVectors.col(2);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Determine the rotation matrix to align two point cloud clusters (1 from real notches to 1 from origin notches) based on their plane normals
  * \param realClusterZ2 the real (target) point cloud cluster
  * \param originClusterZ2 the origin (reference) point cloud cluster
  * \retval The rotation matrix between the two point cloud clusters
*/
Eigen::Matrix4f refineAlign(const ColorPointCloudPtr &realClusterZ2, const ColorPointCloudPtr &originClusterZ2) 
{
    Eigen::Vector3f realNormal = computePlaneNormal(realClusterZ2);
    Eigen::Vector3f originNormal = computePlaneNormal(originClusterZ2);
    Eigen::Vector3f rotationAxis = realNormal.cross(originNormal);
    rotationAxis.normalize();
    float angle = std::acos(realNormal.dot(originNormal));
    Eigen::AngleAxisf rotation(angle, rotationAxis);
    Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();
    Eigen::Matrix4f secondTF = Eigen::Matrix4f::Identity();
    if(!rotationMatrix.array().isNaN().any())
        secondTF.block<3,3>(0,0) = rotationMatrix;
    else
        secondTF(0,0) = -1.0f; //if NaN, return reverse identity matrix
    return secondTF;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Compute translation matrix between 2 clusters from side view (real to origin) after PCA-based alignment
  * \param realCluster the real (target) point cloud cluster
  * \param originCluster the origin (reference) point cloud cluster
  * \retval The translation matrix between the two point cloud clusters (from side real view to side origin view)
  */
Eigen::Matrix4f computeSideTranslation(const ColorPointCloudPtr &realCluster, const ColorPointCloudPtr &originCluster) 
{
    Eigen::Vector4f realCentroid, originCentroid;
    pcl::compute3DCentroid(*realCluster, realCentroid);
    pcl::compute3DCentroid(*originCluster, originCentroid);
    Eigen::Vector3f translation = originCentroid.head<3>() - realCentroid.head<3>();
    Eigen::Matrix4f sideTL = Eigen::Matrix4f::Identity();
    sideTL.block<3,1>(0,3) = translation;
    return sideTL;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Compute translation matrix between 2 clusters from bottom view (real to origin) after PCA-based alignment
  * \param realCluster the real (target) point cloud cluster
  * \param originCluster the origin (reference) point cloud cluster
  * \retval The translation matrix between the two point cloud clusters (from bottom real view to bottom origin view)
  */
Eigen::Matrix4f computeBottomTranslation(const ColorPointCloudPtr &realCluster, const ColorPointCloudPtr &originCluster) 
{
    pcl::PointXYZRGB minPtR, maxPtR;
    pcl::getMinMax3D(*realCluster, minPtR, maxPtR);

    float maxX_R = maxPtR.x * 3.0;
    float minX_R = minPtR.x * 3.0;
    float minY_R = minPtR.y * 0.901;
    float maxY_R = maxPtR.y * 1.017;

    ColorPointCloudPtr filteredCluster(new ColorPointCloud);
    for (const auto& p : originCluster->points) 
    {
        if (p.x < maxX_R && p.x >= minX_R && p.y >= minY_R && p.y <= maxY_R) 
        {
            filteredCluster->points.push_back(p);
        }
    }
    if (filteredCluster->empty()) 
    {
        std::cerr << "          Error: No valid points in filteredCluster after filtering!" << std::endl;
        return Eigen::Matrix4f::Identity();
    }
    Eigen::Vector4f realCentroid, originCentroid;
    pcl::compute3DCentroid(*realCluster, realCentroid);
    pcl::compute3DCentroid(*filteredCluster, originCentroid);
    Eigen::Vector3f translation = originCentroid.head<3>() - realCentroid.head<3>();
    Eigen::Matrix4f bottomTL = Eigen::Matrix4f::Identity();
    bottomTL.block<3,1>(0,3) = translation;
    return bottomTL;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Compute translation matrix between 2 clusters from top view (real to origin) after PCA-based alignment
  * \param realCluster the real (target) point cloud cluster
  * \param originCluster the origin (reference) point cloud cluster
  * \retval The translation matrix between the two point cloud clusters (from top real view to top origin view)
  */
Eigen::Matrix4f computeTopTranslation(const ColorPointCloudPtr &realCluster, const ColorPointCloudPtr &originCluster)                                          
{
    pcl::PointXYZRGB minPtR, maxPtR;
    pcl::getMinMax3D(*realCluster, minPtR, maxPtR);
    float minX_R = minPtR.x * 0.9;
    ColorPointCloudPtr filteredCluster(new ColorPointCloud);
    for (const auto& p : originCluster->points) 
    {
        if (p.x >= minX_R) {
            filteredCluster->points.push_back(p);
        }
    }
    if (filteredCluster->empty()) 
    {
        std::cerr << "Error: No valid points in filteredCluster after filtering!" << std::endl;
        return Eigen::Matrix4f::Identity();
    }
    Eigen::Vector4f realCentroid, originCentroid;
    pcl::compute3DCentroid(*realCluster, realCentroid);
    pcl::compute3DCentroid(*filteredCluster, originCentroid);
    Eigen::Vector3f translation = originCentroid.head<3>() - realCentroid.head<3>();
    Eigen::Matrix4f topTL = Eigen::Matrix4f::Identity();
    topTL.block<3,1>(0,3) = translation;
    return topTL;
}