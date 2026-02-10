#include "utils.hpp"
#include "DataLoader.hpp"
#include "DataPreprocess.hpp"
#include "NotchDetector.hpp"
#include "Registration.hpp"
#include "Merge.hpp"

int main(int argc, char** argv)
{
    //for origin point cloud downsampling, these are default values
    float MASTER_LEAF_SIZE = 0.05;

    //for real point cloud denoising, these are default values
    int REAL_MEAN_K = 50;
    float REAL_DEV_STD_MUL_THRES = 1.0;
    float REAL_Y_SEGMENT_THRES = 1.0;
    float REAL_MIN_HEIGHT_RATIO = 0.3; 

    if(argc > 2)
    {
        MASTER_LEAF_SIZE = std::stof(argv[1]);

        REAL_MEAN_K = std::stoi(argv[2]);
        REAL_DEV_STD_MUL_THRES = std::stof(argv[3]);
        REAL_Y_SEGMENT_THRES = std::stof(argv[4]);
        REAL_MIN_HEIGHT_RATIO = std::stof(argv[5]);
    }
    std::cout << "MASTER LEAF SIZE: " << MASTER_LEAF_SIZE << std::endl;
    std::cout << "REAL MEAN K: " << REAL_MEAN_K << std::endl;
    std::cout << "REAL DEV STD MUL THRES: " << REAL_DEV_STD_MUL_THRES << std::endl;
    std::cout << "REAL Y SEGMENT THRES: " << REAL_Y_SEGMENT_THRES << std::endl;
    std::cout << "REAL MIN HEIGHT RATIO: " << REAL_MIN_HEIGHT_RATIO << std::endl;

    //to record the parameter settings in output filenames
    char argument_info[100];
    sprintf(argument_info, "OUTPUT/args_%.2f_%.2f_%.2f_%.2f", MASTER_LEAF_SIZE, REAL_DEV_STD_MUL_THRES, REAL_Y_SEGMENT_THRES, REAL_MIN_HEIGHT_RATIO);

//STEP 1: load data
    PCL_INFO("Step 1. Loading point clouds...\n");

    PCL_INFO(" - Step 1.1. Loading origin point clouds...\n");

    //load "origin top" point cloud
    std::string originTopPath = "3D/MASTER_TOP.tiff";
    PCL_INFO("      Loading origin top point cloud from %s\n", originTopPath.c_str());
    Eigen::Matrix4f originTopTF;
    originTopTF << -1 , 0 , 0 , 7
            , 0 , -1 , 0 , 537
            , 0 , 0 , 1 , 0
            , 0 , 0 , 0 , 1;
    ColorPointCloudPtr originTopPCPtr = loadOriginData(originTopPath, originTopTF);

    //load "origin side" point cloud
    std::string originSidePath = "3D/MASTER_SIDE.tiff";
    PCL_INFO("      Loading origin side point cloud from %s\n", originSidePath.c_str());
    Eigen::Matrix4f originSideTF;
    originSideTF << 0 , 0 , 1 , 0,
              0 , -1 , 0 , 537,
              1 , 0 , 0 , -3,
              0 , 0 , 0 , 1;
    ColorPointCloudPtr originSidePCPtr = loadOriginData(originSidePath, originSideTF);

    //load "origin bottom" point cloud
    std::string originBottomPath = "3D/MASTER_BOTTOM.tiff";
    PCL_INFO("      Loading origin bottom point cloud from %s\n", originBottomPath.c_str());
    Eigen::Matrix4f originBottomTF;
    originBottomTF << 1 , 0 , 0 , 0,
                 0 , -1 , 0 , 537,
                 0 , 0 , -1 , 2,
                 0 , 0 , 0 , 1;
    ColorPointCloudPtr originBottomPCPtr = loadOriginData(originBottomPath, originBottomTF);

    PCL_INFO(" - Step 1.2. Loading real point clouds...\n");

    //load "real top" point cloud, color it red
    std::string realTopPath = "3D/REAL_TOP.tiff";
    PCL_INFO("      Loading real top point cloud from %s\n", realTopPath.c_str());
    ColorPointCloudPtr realTopPCPtr = loadRealData(realTopPath, 255, 0, 0);

    //load "real side" point cloud, color it green
    std::string realSidePath = "3D/REAL_SIDE.tiff";
    PCL_INFO("      Loading real side point cloud from %s\n", realSidePath.c_str());
    ColorPointCloudPtr realSidePCPtr = loadRealData(realSidePath, 0, 255, 0);

    //load "real bottom" point cloud, color it blue
    std::string realBottomPath = "3D/REAL_BOTTOM.tiff";
    PCL_INFO("      Loading real bottom point cloud from %s\n", realBottomPath.c_str());
    ColorPointCloudPtr realBottomPCPtr = loadRealData(realBottomPath, 0, 0, 255);
//END OF STEP 1

//STEP 2: preprocess data, including downsampling origin point clouds and denoising real point clouds
    PCL_INFO("Step 2. Preprocessing point clouds...\n");

    //downsample origin point clouds
    PCL_INFO(" - Step 2.1. Downsampling origin point clouds...\n");
    PCL_INFO("      Downsampling origin top point cloud...\n");
    ColorPointCloudPtr dsOriginTopPCPtr = downsamplePointCloud(originTopPCPtr, MASTER_LEAF_SIZE);
    char dsOriginTopFilename[512];
    sprintf(dsOriginTopFilename, "%s_dsOriginTop.pcd", argument_info);
    pcl::io::savePCDFile(dsOriginTopFilename, *dsOriginTopPCPtr, true);

    PCL_INFO("      Downsampling origin side point cloud...\n");
    ColorPointCloudPtr dsOriginSidePCPtr = downsamplePointCloud(originSidePCPtr, MASTER_LEAF_SIZE);
    char dsOriginSideFilename[512];
    sprintf(dsOriginSideFilename, "%s_dsOriginSide.pcd", argument_info);
    pcl::io::savePCDFile(dsOriginSideFilename, *dsOriginSidePCPtr, true);

    PCL_INFO("      Downsampling origin bottom point cloud...\n");
    ColorPointCloudPtr dsOriginBottomPCPtr = downsamplePointCloud(originBottomPCPtr, MASTER_LEAF_SIZE);
    char dsOriginBottomFilename[512];
    sprintf(dsOriginBottomFilename, "%s_dsOriginBottom.pcd", argument_info);
    pcl::io::savePCDFile(dsOriginBottomFilename, *dsOriginBottomPCPtr, true);

    //denoise real point clouds
    PCL_INFO(" - Step 2.2. Denoising real point clouds...\n");
    
    PCL_INFO("      Denoising real top point cloud...\n");
    ColorPointCloudPtr dsRealTopPCPtr = denoisePointCloud(realTopPCPtr, REAL_MEAN_K, REAL_DEV_STD_MUL_THRES, REAL_MIN_HEIGHT_RATIO, REAL_Y_SEGMENT_THRES);
    char dsRealTopFilename[512];
    sprintf(dsRealTopFilename, "%s_dsRealTop.pcd", argument_info);
    pcl::io::savePCDFile(dsRealTopFilename, *dsRealTopPCPtr, true);

    PCL_INFO("      Denoising real side point cloud...\n");
    ColorPointCloudPtr dsRealSidePCPtr = denoisePointCloud(realSidePCPtr, REAL_MEAN_K, REAL_DEV_STD_MUL_THRES, REAL_MIN_HEIGHT_RATIO, -1);
    char dsRealSideFilename[512];
    sprintf(dsRealSideFilename, "%s_dsRealSide.pcd", argument_info);
    pcl::io::savePCDFile(dsRealSideFilename, *dsRealSidePCPtr, true);

    PCL_INFO("      Denoising real bottom point cloud...\n");
    ColorPointCloudPtr dsRealBottomPCPtr = denoisePointCloud(realBottomPCPtr, REAL_MEAN_K, REAL_DEV_STD_MUL_THRES, REAL_MIN_HEIGHT_RATIO, REAL_Y_SEGMENT_THRES);
    char dsRealBottomFilename[512];
    sprintf(dsRealBottomFilename, "%s_dsRealBottom.pcd", argument_info);
    pcl::io::savePCDFile(dsRealBottomFilename, *dsRealBottomPCPtr, true);

//END OF STEP 2

//STEP 3: notch detection
    PCL_INFO("Step 3. Detecting notches...\n");

    //detect notches from origin point clouds
    PCL_INFO(" - Step 3.1. Detecting notches from origin point clouds...\n");

    PCL_INFO("      Detecting notch from origin side point cloud...\n");
    ColorPointCloudPtr originSideNotch = detectOriginNotch(dsOriginSidePCPtr, 0);
    char originSideNotchFilename[512];
    sprintf(originSideNotchFilename, "%s_originSideNotch.pcd", argument_info);
    pcl::io::savePCDFile(originSideNotchFilename, *originSideNotch, true);

    PCL_INFO("      Detecting notch from origin bottom point cloud...\n");
    ColorPointCloudPtr originBottomNotch = detectOriginNotch(dsOriginBottomPCPtr, 1);
    char originBottomNotchFilename[512];
    sprintf(originBottomNotchFilename, "%s_originBottomNotch.pcd", argument_info);
    pcl::io::savePCDFile(originBottomNotchFilename, *originBottomNotch, true);

    PCL_INFO("      Detecting notch from origin top point cloud...\n");
    ColorPointCloudPtr originTopNotch = detectOriginNotch(dsOriginTopPCPtr, 1);
    char originTopNotchFilename[512];
    sprintf(originTopNotchFilename, "%s_originTopNotch.pcd", argument_info);
    pcl::io::savePCDFile(originTopNotchFilename, *originTopNotch, true);

    //detect notches from real point clouds
    PCL_INFO(" - Step 3.2. Detecting notches from real point clouds...\n");

    PCL_INFO("      Detecting notch from real side point cloud...\n");
    ColorPointCloudPtr realSideNotch = detectRealNotch(dsRealSidePCPtr, 0);
    char realSideNotchFilename[512];
    sprintf(realSideNotchFilename, "%s_realSideNotch.pcd", argument_info);
    pcl::io::savePCDFile(realSideNotchFilename, *realSideNotch, true);

    PCL_INFO("      Detecting notch from real bottom point cloud...\n");
    ColorPointCloudPtr realBottomNotch = detectRealNotch(dsRealBottomPCPtr, 1);
    char realBottomNotchFilename[512];
    sprintf(realBottomNotchFilename, "%s_realBottomNotch.pcd", argument_info);
    pcl::io::savePCDFile(realBottomNotchFilename, *realBottomNotch, true);

    PCL_INFO("      Detecting notch from real top point cloud...\n");
    ColorPointCloudPtr realTopNotch = detectRealNotch(dsRealTopPCPtr, 1);
    char realTopNotchFilename[512];
    sprintf(realTopNotchFilename, "%s_realTopNotch.pcd", argument_info);
    pcl::io::savePCDFile(realTopNotchFilename, *realTopNotch, true);
//END OF STEP 3

//STEP 4: registration - estimate the transformation matrix from real point clouds to origin frame based on detected notch regions
    PCL_INFO("Step 4. Registrating...\n");

    //register side point clouds
    PCL_INFO(" - Step 4.1. Estimating transformation matrix for side point cloud...\n");

    Eigen::Matrix4f initSideTransform = initAlign(realSideNotch, originSideNotch);

    ColorPointCloudPtr initTransformedRealSideNotch = transformPointCloud(realSideNotch, initSideTransform);
    auto realSideClusterZ = splitNotchesByZ(initTransformedRealSideNotch);
    auto originSideClusterX = splitNotchesByX(originSideNotch);
    auto originSideClusterZ = splitNotchesByZ(originSideClusterX[0]);
    Eigen::Matrix4f refineSideTransform = refineAlign(realSideClusterZ[0], originSideClusterZ[0]);

    ColorPointCloudPtr refineTransformedRealSideNotch = transformPointCloud(initTransformedRealSideNotch, refineSideTransform);
    auto realSideClusterX2 = splitNotchesByX(refineTransformedRealSideNotch);
    Eigen::Matrix4f sideTranslation = computeSideTranslation(realSideClusterX2[0], originSideClusterX[0]);
    
    //The overal transformation matrix from real view to original data for side point clouds 
    Eigen::Matrix4f totalSideTransform = sideTranslation * refineSideTransform * initSideTransform;
    std::cout << "          totalSideTransform:\n" << totalSideTransform << std::endl;

    PCL_INFO(" - Step 4.2. Estimating transformation matrix for bottom point cloud...\n");
    //register bottom point clouds
    Eigen::Matrix4f initBottomTransform = initAlign(realBottomNotch, originBottomNotch);

    ColorPointCloudPtr initTransformedRealBottomNotch = transformPointCloud(realBottomNotch, initBottomTransform);
    auto realBottomClusterXZ = splitBottomNotchesByXZ(initTransformedRealBottomNotch);
    auto originBottomClustersX = splitNotchesByX(originBottomNotch);
    Eigen::Matrix4f refineBottomTransform = refineAlign(realBottomClusterXZ.first, originBottomClustersX[0]);
    
    ColorPointCloudPtr refineTransformedRealBottomNotch = transformPointCloud(initTransformedRealBottomNotch, refineBottomTransform);
    auto realBottomClusterXZ2 = splitBottomNotchesByXZ(refineTransformedRealBottomNotch);
    Eigen::Matrix4f bottomTranslation = computeBottomTranslation(realBottomClusterXZ2.first, originBottomClustersX[0]);

    //The overal transformation matrix from real view to original data for bottom point clouds
    Eigen::Matrix4f totalBottomTransform = bottomTranslation * refineBottomTransform * initBottomTransform;
    std::cout << "          totalBottomTransform:\n" << totalBottomTransform << std::endl;
    
    PCL_INFO(" - Step 4.3. Estimating transformation matrix for top point cloud...\n");
    //register top point clouds
    Eigen::Matrix4f initTopTransform = initAlign(realTopNotch, originTopNotch);
    
    ColorPointCloudPtr initTransformedRealTopNotch = transformPointCloud(realTopNotch, initTopTransform);
    auto realTopClusterXZ = splitTopNotchesByXZ(initTransformedRealTopNotch);
    auto originTopClustersX = splitNotchesByX(originTopNotch);

    Eigen::Matrix4f refineTopTransform = refineAlign(realTopClusterXZ[0], originTopClustersX[0]);

    ColorPointCloudPtr refineTransformedRealTopNotch = transformPointCloud(initTransformedRealTopNotch, refineTopTransform);
    auto realTopClusterXZ2 = splitTopNotchesByXZ(refineTransformedRealTopNotch);

    Eigen::Matrix4f topTranslation = computeTopTranslation(realTopClusterXZ2[0], originTopClustersX[0]);

    //The overal transformation matrix from real view to original data for top point clouds
    Eigen::Matrix4f totalTopTransform = topTranslation * refineTopTransform * initTopTransform;
    std::cout << "          totalTopTransform:\n" << totalTopTransform << std::endl;

//END OF STEP 4

//STEP 5: apply the estimated transformation matrices to the real point clouds to register them to the origin frame
    PCL_INFO("Step 5. Merging real point clouds to origin frame...\n");

    PCL_INFO(" - Step 5.1. Transforming and merging point clouds...\n");
    //the merged result of 3 real point cloud in original frame coordinate system
    ColorPointCloudPtr realPCMerged = registerPointClouds(
        dsRealSidePCPtr, dsRealBottomPCPtr, dsRealTopPCPtr,
        totalSideTransform, totalBottomTransform, totalTopTransform);

    ColorPointCloudPtr transformedRealBottomNotch = transformPointCloud(realBottomNotch, totalBottomTransform);
    *originBottomNotch += *transformedRealBottomNotch;
    ColorPointCloudPtr transformedRealTopNotch = transformPointCloud(realTopNotch, totalTopTransform);
    *originTopNotch += *transformedRealTopNotch;
    ColorPointCloudPtr transformedRealSideNotch = transformPointCloud(realSideNotch, totalSideTransform);
    *originSideNotch += *transformedRealSideNotch;

    PCL_INFO(" - Step 5.2. Saving merged point cloud and estimated transformation matrices...\n");
    char filename[512];
    
    sprintf(filename, "%s_mergeSideNotch.pcd", argument_info);
    pcl::io::savePCDFile(filename, *originSideNotch, true);
    
    sprintf(filename, "%s_mergeBottomNotch.pcd", argument_info);
    pcl::io::savePCDFile(filename, *originBottomNotch, true);
    
    sprintf(filename, "%s_mergeTopNotch.pcd", argument_info);
    pcl::io::savePCDFile(filename, *originTopNotch, true);

    sprintf(filename, "%s_mergeReal.pcd", argument_info);
    pcl::io::savePCDFile(filename, *realPCMerged, true);
    
    sprintf(filename, "%s_transformReal.txt", argument_info);
    std::ofstream file(filename);
    
    file << "Side Transform:\n" << totalSideTransform << "\n";
    file << "Bottom Transform:\n" << totalBottomTransform << "\n";
    file << "Top Transform:\n" << totalTopTransform << "\n";
    file.close();

//END OF STEP 5

    PCL_INFO("Process completed!");

    return 0;
}
