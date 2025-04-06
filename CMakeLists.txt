cmake_minimum_required(VERSION 3.10)
project(fast_lio_localization_sc_qn)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++20 -pthread -fexceptions -g -ggdb") #c++17, for parallel programming
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
find_package(OpenMP)
if(OPENMP_FOUND) #For Nano-GICP, Quatro #The best way to set proper compiler settings for using OpenMP in all platforms
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS} -fopenmp -O3 -ftree-vectorize -funroll-loops")

set(CMAKE_EXPORT_COMPILE_COMMANDS True)

string(ASCII 27 Esc)
set(RED "${Esc}[1;31m")
set(YELLOW "${Esc}[1;33m")
set(NONE "${Esc}[m")
set(BLUE "${Esc}[1;34m")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("${BLUE}select arch with ${CMAKE_SYSTEM_PROCESSOR}")
  set(CMAKE_PREFIX_PATH "/opt/ros/humble/share;/usr/lib/x86_64-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  message("${BLUE}select arch with ${CMAKE_SYSTEM_PROCESSOR}")
  set(CMAKE_PREFIX_PATH "/opt/ros/humble/share;/usr/lib/arm-linux-gnu")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch")
  message("${BLUE}select arch with ${CMAKE_SYSTEM_PROCESSOR}")
  set(CMAKE_PREFIX_PATH "/opt/ros/humble/share;/usr/lib/aarch64-linux-gnu")
else()
  message("${BLUE}unknown arch with ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    add_definitions(-DEIGEN_USE_BLAS)
    find_package(BLAS REQUIRED)
    set(BLAS_LIBRARIES "/usr/lib/x86_64-linux-gnu/openblas-openmp/libopenblas.so;-lm;-ldl")
endif()

### get packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(nano_gicp REQUIRED)
find_package(quatro REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FMT REQUIRED)
find_package(teaserpp REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(minkindr REQUIRED)
find_package(minkindr_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DFASTLIO_LO_SC_QN_PARAM_DEBUG)

if(gflags_FOUND)
  message(WARNING "gflags found: ${gflags_INCLUDE_DIRS}")
endif()
if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()

set(dependencies
  rclcpp
  rosbag2_cpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_eigen
  pcl_ros
  pcl_conversions
  message_filters
  nano_gicp
  quatro
  livox_ros_driver2
  tf2_geometry_msgs
  minkindr
  minkindr_conversions
  glog
  OpenCV
  cv_bridge
)

### get packages' headers
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${PCL_CONVERSIONS_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${FMT_INCLUDE_DIRS}
)

###########
## Build ##
###########
### main
set(COMMON_LIBS ${OpenCV_LIBS} ${PCL_LIBRARIES} ${EIGEN3_LIBS} ${BLAS_LIBRARIES} ${OpenMP_LIBS} teaserpp::teaser_registration teaserpp::teaser_io gflags glog)
add_library(${PROJECT_NAME}_modules src/Scancontext.cpp src/fast_lio_localization_sc_qn.cpp src/map_matcher.cpp)
target_link_libraries(${PROJECT_NAME}_modules ${COMMON_LIBS})

add_executable(${PROJECT_NAME}_node src/main.cpp)
add_executable(bag_extract src/bag_extract.cpp)
add_executable(kitti_bag src/kitti_bag.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
ament_target_dependencies(bag_extract ${dependencies})
ament_target_dependencies(kitti_bag ${dependencies})
target_link_libraries(${PROJECT_NAME}_node ${COMMON_LIBS} ${PROJECT_NAME}_modules)
target_link_libraries(bag_extract ${COMMON_LIBS})
target_link_libraries(kitti_bag ${COMMON_LIBS})

install(TARGETS
  ${PROJECT_NAME}_node ${PROJECT_NAME}_modules bag_extract kitti_bag
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config "include" rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_libraries(
  ${PROJECT_NAME}_modules
)

ament_export_include_directories(
  include
)

ament_package()
