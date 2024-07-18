cmake_minimum_required(VERSION 3.5)
project(imx219)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# NOTE: we build opencv from source, hence the path /usr/local/lib
find_package(OpenCV REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(/usr/local/lib)
# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

add_executable(imx219 IMX219.cpp)
target_link_libraries(imx219 ${OpenCV_LIBS})
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
)

ament_target_dependencies(imx219 Eigen3 rclcpp sensor_msgs cv_bridge OpenCV)

target_compile_features(imx219 PUBLIC c_std_99 cxx_std_17)

install(TARGETS imx219 DESTINATION lib/${PROJECT_NAME})
target_link_libraries(imx219
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()


ament_package()