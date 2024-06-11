#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

class ArucoTrackerNode : public rclcpp::Node
{
public:
	ArucoTrackerNode();

private:
	void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_pub;

	// px4_msgs
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

	// Data
	cv::Ptr<cv::aruco::Dictionary> _dictionary;
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeffs;

	Eigen::Vector3f postionNED = {};
	float heading = {};
};

