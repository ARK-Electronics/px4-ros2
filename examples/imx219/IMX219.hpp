// #pragma once 
// #include <array>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>



// class CameraNode : public rclcpp::Node
// {
// public:
//   CameraNode();

// private:
//   void timer_callback();
//   sensor_msgs::msg::CameraInfo create_camera_info_msg();

//   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
//   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_publisher;
//   rclcpp::TimerBase::SharedPtr _timer;
//   cv::VideoCapture _cap;
// };
#pragma once 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode();

private:
  void timer_callback();
  sensor_msgs::msg::CameraInfo create_camera_info_msg();
  static GstFlowReturn new_sample(GstAppSink *sink, gpointer data);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  GstElement *_pipeline, *_appsink;
};
