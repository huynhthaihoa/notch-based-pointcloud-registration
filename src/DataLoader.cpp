// #include "utils.hpp"
#include "DataLoader.hpp"

////////////////////////////////////////////////////////////////////////////////
/** \brief Load real point cloud from depth image REAL_*.tiff
  * \param filePath the file path of the depth image
  * \param r the red channel value for coloring the point cloud
  * \param g the green channel value for coloring the point cloud
  * \param b the blue channel value for coloring the point cloud
  * \retval The loaded real point cloud
  */
ColorPointCloudPtr loadRealData(const std::string &filePath, int r, int g, int b)
{
    ColorPointCloudPtr realPCColorPtr(new ColorPointCloud);
    auto realDepthMap = cv::imread(filePath, cv::IMREAD_UNCHANGED);
    auto widthDepthMap = realDepthMap.cols;
    auto heightDepthMap = realDepthMap.rows;
    auto imgPtrDepthMap = realDepthMap.ptr<float>(0);
    for(int j = 0; j < heightDepthMap; ++j)
    {
        auto step = widthDepthMap * j;
        for(int i = 0; i < widthDepthMap; ++i)
        {
            //only save point with valid depth into point cloud
            if(NULLVALUE != imgPtrDepthMap[step + i])
            {
                PointC pt(SCALEX * i, SCALEY * j, -imgPtrDepthMap[step + i], r, g, b);    //top - red
                realPCColorPtr->push_back(pt);
            }
        }
    }  

    PCL_INFO("          Cloud contains %zu points.\n", realPCColorPtr->points.size());

    return realPCColorPtr;
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load origin point cloud from depth image MASTER_*.tiff and apply initial transformation
  * \param filePath the file path of the depth image
  * \param transform the initial transformation matrix to be applied to the point cloud
  * \retval The loaded and transformed origin point cloud
  */
ColorPointCloudPtr loadOriginData(const std::string &filePath, const Eigen::Matrix4f &transform)
{
    ColorPointCloud masterPC;
    // file read
    auto masterDepthMap = cv::imread(filePath, cv::IMREAD_UNCHANGED);
    auto dataFullSize = masterDepthMap.total();
    auto imgPtr = masterDepthMap.ptr<cv::Vec3f>(0);
    for (auto i = 0; i < dataFullSize; ++i)
    {
        //remove noise points
        if(NULLVALUE != imgPtr[i][2] && NULLVALUE != imgPtr[i][1] && NULLVALUE != imgPtr[i][0])
        {
            PointC pt(imgPtr[i][2], imgPtr[i][1], imgPtr[i][0], 255, 255, 255);
            masterPC.push_back(pt);
        }
    } 
    
    ColorPointCloudPtr originPCPtr(new ColorPointCloud);
    pcl::transformPointCloud(masterPC, *originPCPtr, transform);

    PCL_INFO("          Cloud contains %zu points.\n", originPCPtr->points.size());

    return originPCPtr;
}

