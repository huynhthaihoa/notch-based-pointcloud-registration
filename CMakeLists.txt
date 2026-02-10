# cmake needs this line
cmake_minimum_required(VERSION 3.5)

# Define project name
project(pointcloud_registration)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "PCL library status:")
message(STATUS "    config: ${PCL_DIR}")
message(STATUS "    version: ${PCL_VERSION}")
message(STATUS "    libraries: ${PCL_LIBRARIES}")
message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")

# Declare the executable target built from your sources
add_executable(pointcloud_registration src/main.cpp src/DataLoader.cpp src/DataPreprocess.cpp src/NotchDetector.cpp src/Registration.cpp src/Merge.cpp)

# Link your application with OpenCV + PCL libraries
target_link_libraries(pointcloud_registration PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES})
