cmake_minimum_required(VERSION 3.5)
project(aruco_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)

install(DIRECTORY
  include
  config
  DESTINATION share/${PROJECT_NAME}
)
# add include directories
include_directories(include)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(aruco_tracker src/ArucoTracker.cpp)

ament_target_dependencies(aruco_tracker rclcpp px4_msgs sensor_msgs cv_bridge OpenCV)

target_compile_features(aruco_tracker PUBLIC c_std_99 cxx_std_17)

install(TARGETS aruco_tracker DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
