cmake_minimum_required(VERSION 2.8)

project(msckf)
set (msckf_CMAKE_DIR "${msckf_SOURCE_DIR}/cmake")
set (CMAKE_MODULE_PATH ${msckf_CMAKE_DIR} ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs image_transport message_generation)

include_directories("${msckf_SOURCE_DIR}/include")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(SOURCE_FILES
#exceptions
include/exceptions/calibration_file_error.h
src/exceptions/calibration_file_error.cpp
include/exceptions/impossible_exception.h
src/exceptions/impossible_exception.cpp
include/exceptions/base_exception.h
src/exceptions/base_exception.cpp
include/exceptions/general_exception.h
src/exceptions/general_exception.cpp
#feature
#include/feature/feature_rezidualization_result.h
#src/feature/feature_rezidualization_result.cpp
include/feature/tracker.h
src/feature/tracker.cpp
include/feature/feature.h
src/feature/feature.cpp
#include/feature/feature_track.h
#src/feature/feature_track.cpp
#filter
#include/filter/filter.h
#src/filter/filter.cpp
#frame
include/frame/frame.h
src/frame/frame.cpp
#include/frame/camera_pose.h
#src/frame/camera_pose.cpp
#include/frame/camera_pose_buffer.h
#src/frame/camera_pose_buffer.cpp
#include/frame/camera_reprojection_functor.h
#src/frame/camera_reprojection_functor.cpp
#imu
#include/imu/imu_buffer.h
#src/imu/imu_buffer.cpp
#include/imu/imu_item.h
#src/imu/imu_item.cpp
#include/imu/imu_device.h
#src/imu/imu_device.cpp
#state
#include/state/body_state.h
#src/state/body_state.cpp
#include/state/filter_state.h
#src/state/filter_state.cpp
#utils
#include/utils/calibration.h
#src/utils/calibration.cpp
#include/utils/quaternion_tools.h
#src/utils/quaternion_tools.cpp
#include/utils/state_initializer.h
#src/utils/state_initializer.cpp
#include/utils/quaternion.h
#src/utils/quaternion.cpp
#include/utils/stats.h
#src/utils/stats.cpp
#include/utils/stats_node.h
#src/utils/stats_node.cpp
#include/utils/stats_timer.h
#src/utils/stats_timer.cpp
#include/utils/chi_squared_ppm.h
#algorithm
include/algorithm/residual.h
src/algorithm/residual.cpp
#others
include/msckf.h
src/msckf.cpp
#include/msckf_ros.h
#src/msckf_ros.cpp
#include/msckf_kitti.h
#src/msckf_kitti.cpp
src/main.cpp
)
add_executable(msckf src/main.cpp ${SOURCE_FILES})

find_package(Eigen3 REQUIRED)
include_directories (AFTER "${EIGEN3_INCLUDE_DIR}")

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs sensor_msgs tf2 tf2_ros roscpp
    DEPENDS eigen
)

find_package(Boost REQUIRED COMPONENTS program_options filesystem)

include_directories (AFTER "${Boost_INCLUDE_DIRS}")
target_link_libraries(msckf ${Boost_LIBRARIES})

find_package(OpenCV 3.1.0 REQUIRED)
message (STATUS "${OpenCV_INCLUDE_DIRS}")
message (STATUS "${OpenCV_LIBS}")
include_directories (AFTER "${OpenCV_INCLUDE_DIRS}")
target_link_libraries(msckf ${OpenCV_LIBS})

find_package(image_transport REQUIRED)
include_directories (AFTER ${image_transport_INCLUDE_DIRS})
target_link_libraries(msckf ${image_transport_LIBRARIES})

find_package(cv_bridge REQUIRED)
target_link_libraries(msckf ${cv_bridge_LIBRARIES})

find_package(tf2_ros REQUIRED)
target_link_libraries(msckf ${tf2_ros_LIBRARIES})

find_package(eigen_conversions REQUIRED)
target_link_libraries(msckf ${eigen_conversions_LIBRARIES})
