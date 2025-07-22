cmake_minimum_required(VERSION 3.15)
project(pointcloud_reconstruct)

set(CMAKE_CXX_STANDARD 17)

# 设置 PCL 安装路径
set(PCL_DIR "C:/PCL1.14.1/cmake")

find_package(PCL REQUIRED COMPONENTS 
  common
  io
  filters
  visualization
  segmentation
  surface
  features
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_reconstruct dimension.cpp)
target_link_libraries(pointcloud_reconstruct ${PCL_LIBRARIES})
