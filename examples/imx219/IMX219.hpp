// #pragma once // Ensures the header is included only once during compilation

// #include <array> // Include the array library
// #include <memory> // Include the memory library for smart pointers
// #include <rclcpp/rclcpp.hpp> // Include the ROS2 client library
// #include <sensor_msgs/msg/image.hpp> // Include the ROS2 Image message
// #include <sensor_msgs/msg/camera_info.hpp> // Include the ROS2 CameraInfo message
// #include <cv_bridge/cv_bridge.h> // Include the cv_bridge for ROS2 and OpenCV
// #include <opencv2/opencv.hpp> // Include the OpenCV library

// // Define the CameraNode class, which inherits from rclcpp::Node
// class CameraNode : public rclcpp::Node
// {
// public:
//   CameraNode(); // Constructor

// private:
//   // Timer callback function to capture and publish images periodically
//   void timer_callback();

//   // Function to create and return a CameraInfo message
//   sensor_msgs::msg::CameraInfo create_camera_info_msg();

//   // Publishers for image and camera info messages
//   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
//   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_publisher;

//   // Timer to periodically trigger the callback function
//   rclcpp::TimerBase::SharedPtr _timer;

//   // OpenCV VideoCapture object to interface with the camera
//   cv::VideoCapture _cap;
// };

#pragma once // Ensures the header is included only once during compilation

#include <rclcpp/rclcpp.hpp> // Include the ROS2 client library
#include <sensor_msgs/msg/image.hpp> // Include the ROS2 Image message
#include <sensor_msgs/msg/camera_info.hpp> // Include the ROS2 CameraInfo message
#include <gst/gst.h> // Include the GStreamer library
#include <gst/app/gstappsink.h> // Include the GStreamer appsink

// Define the CameraNode class, which inherits from rclcpp::Node
class CameraNode : public rclcpp::Node
{
public:
	CameraNode(); // Constructor

private:
	// Function to create and return a CameraInfo message
	sensor_msgs::msg::CameraInfo create_camera_info_msg();

	// Static callback function for handling new samples from the appsink
	static GstFlowReturn new_sample(GstAppSink* sink, gpointer data);

	// Publishers for image and camera info messages
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_publisher;

	// GStreamer elements for the pipeline and appsink
	GstElement* _pipeline, *_appsink;
};
