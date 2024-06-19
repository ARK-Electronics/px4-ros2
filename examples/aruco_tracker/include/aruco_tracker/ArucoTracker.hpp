#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

class ArucoNode : public rclcpp::Node{

    public:
        ArucoNode(std::string node_name): Node(node_name){
            RCLCPP_INFO(this->get_logger(), "Starting ArucoTrackerNode");
            // Define aruco tag dictionary to use
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

            // // Load camera calibration data from YAML file
            // cv::FileStorage fs("/home/oem/ARK/px4_ros2/src/px4-ros2/examples/aruco_patrik_cpp/config/usb_cam_calib.yml", cv::FileStorage::READ);
            // if (fs.isOpened()) {
            //     fs["camera_matrix"] >> camera_matrix_;
            //     fs["distortion_coefficients"] >> dist_coeffs_;
            //     fs.release();
            // } else {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file");
            // }

            // if (camera_matrix_.empty() || dist_coeffs_.empty()) {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to load camera parameters correctly.");
            // }
            // Set up QoS
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5));
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

            // Create a subscription to the image topic
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera", qos, std::bind(&ArucoNode::image_callback, this, std::placeholders::_1)
            );
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco/image", qos);
            vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		         "/fmu/out/vehicle_local_position", qos, std::bind(&ArucoNode::vehicle_local_position_callback, this, std::placeholders::_1));

            // Create a publisher to the pose topic
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", qos);
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ArucoNode::pose_pub_callback, this));

        }

    private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    // rclcpp::TimerBase::SharedPtr timer_;
    float distance_to_ground_ = {};

    // Data
	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
};