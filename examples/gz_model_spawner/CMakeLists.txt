cmake_minimum_required(VERSION 3.5)
project(gz_model_spawner)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2 REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(gz_model_spawner main.cpp)

ament_target_dependencies(gz_model_spawner Eigen3 px4_ros2 rclcpp ros_gz_interfaces)

target_compile_features(gz_model_spawner PUBLIC c_std_99 cxx_std_17)

install(TARGETS gz_model_spawner DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
