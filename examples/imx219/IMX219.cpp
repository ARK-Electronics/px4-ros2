#include "IMX219.hpp"
#include <sstream>

CameraNode::CameraNode()
: Node("camera_node")
{
  // Define QoS profile
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  // Create publishers
  _image_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/image", qos_profile);
  _camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", qos_profile);

  // Timer interval is set to capture frames at approximately 30 fps
  _timer = this->create_wall_timer(
    std::chrono::milliseconds(33), // Approximately 30 fps (1000 ms / 30 fps = 33.33 ms)
    std::bind(&CameraNode::timer_callback, this));

  // GStreamer pipeline for low latency streaming
  _cap.open("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink sync=false max-buffers=1 max-lateness=0");
  // cap_.open("nvarguscamerasrc ! nvvidconv ! x264enc key-int-max=15 bitrate=2500 tune=zerolatency speed-preset=ultrafast ! video/x-h264,stream-format=byte-stream ! appsink");
  if (!_cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
  }
}

sensor_msgs::msg::CameraInfo CameraNode::create_camera_info_msg()
{
  auto camera_info_msg = sensor_msgs::msg::CameraInfo();
  camera_info_msg.header.frame_id = "camera_frame";
  camera_info_msg.width = 640;
  camera_info_msg.height = 480;

  // Intrinsic camera matrix
  camera_info_msg.k = {620.0, 0.0, 320.0, 0.0, 620.0, 240.0, 0.0, 0.0, 1.0};

  // Rectification matrix
  camera_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  // Projection matrix
  camera_info_msg.p = {620.0, 0.0, 320.0, 0.0, 0.0, 620.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};

  return camera_info_msg;
}

void CameraNode::timer_callback()
{
  cv::Mat frame;
  _cap >> frame;
  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Captured empty frame");
    return;
  }

  // Publish image
  auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  image_msg->header.stamp = this->now();
  image_msg->header.frame_id = "camera_frame";
  _image_publisher->publish(*image_msg);

  // Publish camera info
  auto camera_info_msg = create_camera_info_msg();
  camera_info_msg.header.stamp = image_msg->header.stamp;
  _camera_info_publisher->publish(camera_info_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}