// #include "IMX219.hpp"
// #include <sstream>

// CameraNode::CameraNode()
// : Node("camera_node")
// {
//   // Define QoS profile
//   auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

//   // Create publishers
//   _image_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/image", qos_profile);
//   _camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", qos_profile);

//   // Timer interval is set to capture frames at approximately 30 fps
//   _timer = this->create_wall_timer(
//     std::chrono::milliseconds(33), // Approximately 30 fps (1000 ms / 30 fps = 33.33 ms)
//     std::bind(&CameraNode::timer_callback, this));

//   // GStreamer pipeline for low latency streaming
//   _cap.open("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink sync=false max-buffers=1 max-lateness=0");
//   // cap_.open("nvarguscamerasrc ! nvvidconv ! x264enc key-int-max=15 bitrate=2500 tune=zerolatency speed-preset=ultrafast ! video/x-h264,stream-format=byte-stream ! appsink");
//   if (!_cap.isOpened()) {
//     RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
//   }
// }

// sensor_msgs::msg::CameraInfo CameraNode::create_camera_info_msg()
// {
//   auto camera_info_msg = sensor_msgs::msg::CameraInfo();
//   camera_info_msg.header.frame_id = "camera_frame";
//   camera_info_msg.width = 640;
//   camera_info_msg.height = 480;

//   // Intrinsic camera matrix
//   camera_info_msg.k = {620.0, 0.0, 320.0, 0.0, 620.0, 240.0, 0.0, 0.0, 1.0};

//   // Rectification matrix
//   camera_info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

//   // Projection matrix
//   camera_info_msg.p = {620.0, 0.0, 320.0, 0.0, 0.0, 620.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};

//   return camera_info_msg;
// }

// void CameraNode::timer_callback()
// {
//   cv::Mat frame;
//   _cap >> frame;
//   if (frame.empty()) {
//     RCLCPP_WARN(this->get_logger(), "Captured empty frame");
//     return;
//   }

//   // Publish image
//   auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
//   image_msg->header.stamp = this->now();
//   image_msg->header.frame_id = "camera_frame";
//   _image_publisher->publish(*image_msg);

//   // Publish camera info
//   auto camera_info_msg = create_camera_info_msg();
//   camera_info_msg.header.stamp = image_msg->header.stamp;
//   _camera_info_publisher->publish(camera_info_msg);
// }

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<CameraNode>());
//   rclcpp::shutdown();
//   return 0;
// }
#include "IMX219.hpp"
#include <sstream>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

CameraNode::CameraNode() : Node("camera_node") {
  // Define QoS profile
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  // Create publishers
  _image_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/image", qos_profile);
  _camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", qos_profile);

  // Timer interval is set to capture frames at approximately 30 fps
  _timer = this->create_wall_timer(
    std::chrono::milliseconds(33), // Approximately 30 fps (1000 ms / 30 fps = 33.33 ms)
    std::bind(&CameraNode::timer_callback, this));

  // Initialize GStreamer
  gst_init(nullptr, nullptr);

  // GStreamer pipeline for low latency streaming
  std::string pipeline_str = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink name=appsink sync=false max-buffers=1 drop=true";

  GError *error = nullptr;
  _pipeline = gst_parse_launch(pipeline_str.c_str(), &error);

  if (error) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse launch: %s", error->message);
    g_error_free(error);
    return;
  }

  _appsink = gst_bin_get_by_name(GST_BIN(_pipeline), "appsink");
  gst_app_sink_set_emit_signals((GstAppSink*)_appsink, true);
  gst_app_sink_set_drop((GstAppSink*)_appsink, true);
  gst_app_sink_set_max_buffers((GstAppSink*)_appsink, 1);

  GstAppSinkCallbacks callbacks = {nullptr, nullptr, new_sample, nullptr, nullptr, {nullptr}};

  gst_app_sink_set_callbacks(GST_APP_SINK(_appsink), &callbacks, (gpointer)this, nullptr);

  gst_element_set_state(_pipeline, GST_STATE_PLAYING);

  RCLCPP_INFO(this->get_logger(), "GStreamer pipeline started");
}

sensor_msgs::msg::CameraInfo CameraNode::create_camera_info_msg() {
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

void CameraNode::timer_callback() {
  // The actual frame grabbing and publishing is done in the new_sample callback
}

GstFlowReturn CameraNode::new_sample(GstAppSink *sink, gpointer data) {
  CameraNode *node = static_cast<CameraNode*>(data);

  RCLCPP_INFO(node->get_logger(), "New sample received");

  GstSample *sample = gst_app_sink_pull_sample(sink);
  if (!sample) {
    RCLCPP_WARN(node->get_logger(), "Failed to pull sample");
    return GST_FLOW_ERROR;
  }

  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    gst_sample_unref(sample);
    RCLCPP_WARN(node->get_logger(), "Failed to map buffer");
    return GST_FLOW_ERROR;
  }

  // Create a sensor_msgs::msg::Image and fill it with the data
  auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
  image_msg->header.stamp = node->now();
  image_msg->header.frame_id = "camera_frame";
  image_msg->height = 480;
  image_msg->width = 640;
  image_msg->encoding = sensor_msgs::image_encodings::BGR8;
  image_msg->is_bigendian = false;
  image_msg->step = 640 * 3; // width * number of channels
  image_msg->data.assign(map.data, map.data + map.size);

  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);

  // Publish the image message
  node->_image_publisher->publish(*image_msg);

  // Publish camera info
  auto camera_info_msg = node->create_camera_info_msg();
  camera_info_msg.header.stamp = image_msg->header.stamp;
  node->_camera_info_publisher->publish(camera_info_msg);

  RCLCPP_INFO(node->get_logger(), "Image published");

  return GST_FLOW_OK;
}

int main(int argc, char * argv[]) {
  // Patrik needs this, to export the display, Jake probably does not
  // Set the DISPLAY environment variable
  setenv("DISPLAY", ":0", 1);

  // Run xhost command to allow root access
  system("xhost +si:localuser:root");

  // Add the Xauthority entry
  system("sudo xauth add $(xauth -f ~/.Xauthority list | tail -1)");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}