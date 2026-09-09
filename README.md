# Notch-Based Point Cloud Registration

- **Last update**: February 09, 2026
- **Author**: Hoa Huynh

## Overview

This is a C++ implementation of a point cloud registration solution that aligns multiple 3D point clouds by detecting and matching notch regions as stable keypoint correspondences. This approach is particularly effective for registering real sensor data to a master reference model, especially in industrial applications where distinctive geometric features (notches) are present.

**Use cases:**
- Aligning captured 3D scans to reference models
- Multi-view point cloud registration
- Geometric quality inspection and verification

![Solution Pipeline](assets/pointcloud_registration.jpg)

---

## Method Overview

The solution implements a four-stage pipeline:

1. **Data Loading & Preprocessing**: Load TIFF files and downsample/denoise point clouds
2. **Notch Detection**: Identify distinctive notch regions as keypoints in both real and reference clouds
3. **Correspondence Matching**: Find matching notches between point clouds
4. **Registration & Transformation**: Estimate transformation matrices to align clouds

This approach leverages stable geometric features (notches) rather than relying on intensity or color information, making it robust to varying lighting conditions.

---

## Project Structure

```
.
├── 3D/                          # Input directory: place TIFF files here
├── OUTPUT/                      # Output directory: generated artifacts saved here
├── assets/                      # Images and visual documentation
├── include/                     # C++ header files
│   ├── utils.hpp               # Headers, typedefs, constants
│   ├── DataLoader.hpp          # Data loading function declarations
│   ├── DataPreprocess.hpp      # Preprocessing function declarations
│   ├── NotchDetector.hpp       # Notch detection function declarations
│   ├── Registration.hpp        # Registration function declarations
│   └── Merge.hpp               # Merging function declarations
├── src/                         # C++ source files
│   ├── main.cpp                # Main entry point
│   ├── DataLoader.cpp          # Data loading implementations
│   ├── DataPreprocess.cpp      # Preprocessing implementations
│   ├── NotchDetector.cpp       # Notch detection implementations
│   ├── Registration.cpp        # Registration implementations
│   └── TransformEstimator.cpp  # Transformation estimation implementations
├── CMakeLists.txt              # CMake build configuration
└── compile.sh                  # Build script
```

---

## Installation

### System Requirements

- **OS**: Ubuntu 20.04 (tested on Ubuntu 20.04)
- **Compiler**: GCC/G++ with C++17 support
- **CMake**: 3.22.1 or higher
- **Build tool**: Make or Ninja

### Dependencies

Install the following dependencies (exact versions used in development):

| Dependency | Version | Purpose |
|-----------|---------|---------|
| OpenCV | 4.10.0 | Image and TIFF file processing |
| PCL (Point Cloud Library) | 1.15.1 | Point cloud processing |
| FLANN | 1.9.2 | Nearest neighbor search |
| Eigen | 3.3.0 | Linear algebra operations |
| Boost | 1.74.0 | C++ utilities and libraries |

#### Quick Installation (Ubuntu 20.04)

```bash
# Update package manager
sudo apt-get update

# Install system dependencies
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  libpcl-dev \
  libeigen3-dev \
  libboost-all-dev \
  libflann-dev

# Verify installations
cmake --version
opencv_version || pkg-config --modversion opencv4
pcl_viewer -v
```

**Note**: If the above packages don't match the exact versions, you may need to build specific versions from source. See links below:
- [OpenCV 4.10.0](https://github.com/opencv/opencv/releases/tag/4.10.0)
- [PCL 1.15.1](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.15.1)
- [FLANN 1.9.2](https://github.com/flann-lib/flann/releases/tag/1.9.2)
- [Eigen 3.3.0](https://gitlab.com/libeigen/eigen/-/releases/3.3.0)
- [Boost 1.74.0](https://www.boost.org/releases/1.74.0/)

---

## Building the Project

### Step 1: Make the build script executable

```bash
chmod +x compile.sh
```

### Step 2: Run the build script

```bash
./compile.sh
```

This script will:
- Run CMake to generate build files
- Compile all source files
- Link against required dependencies
- Generate the executable `pointcloud_registration` in the project root

**Troubleshooting**:
- **CMake not found**: Ensure CMake 3.22+ is installed: `sudo apt-get install cmake`
- **Missing libraries**: Run the installation commands above
- **Build fails**: Check that all dependencies are installed and their paths are accessible

---

## Usage

### Prerequisites

1. Ensure the project is built (see [Building the Project](#building-the-project))
2. Place all input TIFF files in the `3D/` directory
3. Verify the `OUTPUT/` directory exists (create if needed)

### Running with Default Parameters

```bash
./pointcloud_registration
```

**Default configuration:**
- Downsampling leaf size: 0.05 m
- Denoising neighbors: 50
- Denoising std dev multiplier: 1.0
- Y-axis segmentation threshold: 1.0 m
- Minimum height ratio: 0.3

### Running with Custom Parameters

```bash
./pointcloud_registration <arg1> <arg2> <arg3> <arg4> <arg5>
```

#### Parameter Guide

| Arg | Name | Type | Default | Range | Description |
|-----|------|------|---------|-------|-------------|
| arg1 | Downsampling Leaf Size | float | 0.05 | 0.01-0.2 | Voxel size for downsampling reference cloud (m). Smaller = more detail, slower processing |
| arg2 | Denoising Neighbors | int | 50 | 10-100 | Number of neighbors to analyze for statistical outlier removal. Increase if real cloud is very noisy |
| arg3 | Std Dev Multiplier | float | 1.0 | 0.5-2.0 | Threshold for outlier detection. Higher = more permissive (keeps more points) |
| arg4 | Y-Axis Threshold | float | 1.0 | 0.5-2.0 | Segmentation threshold for TOP/BOTTOM separation (m). Tune if misalignment occurs on vertical axis |
| arg5 | Min Height Ratio | float | 0.3 | 0.1-0.8 | Minimum relative height to filter small noise. Lower = stricter filtering |

#### Example Runs

```bash
# For noisy point clouds
./pointcloud_registration 0.05 75 1.5 1.0 0.3

# For high-precision registration
./pointcloud_registration 0.02 30 0.75 0.75 0.4

# For faster processing with less accuracy
./pointcloud_registration 0.1 50 1.5 1.2 0.2
```

### Output Artifacts

After successful execution, the following files are generated in the `OUTPUT/` directory:

| Output File | Description |
|------------|-------------|
| `args_*_dsOrigin[Bottom/Top/Side].pcd` | Downsampled reference point clouds for each view |
| `args_*_dsReal[Bottom/Top/Side].pcd` | Denoised real point clouds for each view |
| `args_*_merge[Bottom/Top/Side]Notch.pcd` | Detected notches from real cloud aligned with reference notches |
| `args_*_mergeReal.pcd` | All denoised real clouds fused into the reference coordinate system |
| `args_*_transformReal.txt` | **Transformation matrices** for REAL SIDE, BOTTOM, and TOP relative to reference frame |

**Interpreting the output:**
- `.pcd` files can be visualized with PCL tools: `pcl_viewer output.pcd`
- `transformReal.txt` contains 4×4 transformation matrices (one per line) for downstream processing

---

## Troubleshooting

### No output generated after running

- **Check**: Verify TIFF files are in the `3D/` directory
- **Check**: Ensure all files are valid TIFF format
- **Check**: Review terminal output for specific error messages
- **Try**: Run with verbose output (may require code modification)

### Misaligned or poor registration results

- **Adjust arg2 & arg3**: Increase if real cloud has excessive noise
- **Adjust arg4**: If vertical misalignment occurs (TOP/BOTTOM views)
- **Adjust arg1**: Reduce for finer detail, but expect slower processing
- **Check**: Ensure reference and real point clouds contain visible notch features

### Build errors with dependency versions

- **Verify** exact versions: `pkg-config --modversion <library>`
- **Consider** building specific versions from source (see dependency links above)
- **Check** CMakeLists.txt for version requirements

### Segmentation faults or crashes

- **Check**: Available system memory (point clouds can be large)
- **Reduce**: Decrease arg2 or increase arg1 to process smaller clouds
- **Verify**: All dependencies are correctly compiled for your system

---

## Performance

- **Processing time**: Typically 30-120 seconds depending on cloud size and parameters
- **Memory**: ~500MB-2GB depending on point cloud resolution
- **Accuracy**: Notch-based approach typically achieves sub-millimeter alignment on well-defined features
