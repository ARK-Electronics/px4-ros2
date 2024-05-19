cmake_minimum_required(VERSION 3.5)
project(rtl_replacement)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(rtl_replacement main.cpp)

ament_target_dependencies(rtl_replacement Eigen3 px4_ros2 rclcpp)

target_compile_features(rtl_replacement PUBLIC c_std_99 cxx_std_17)

install(TARGETS rtl_replacement DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
