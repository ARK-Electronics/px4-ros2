#include "YoloDetection.hpp"
#include <sstream>

YoloDetectionNode::YoloDetectionNode()
	: Node("yolo_detection_node")
{
	RCLCPP_INFO(this->get_logger(), "Starting YoloDetectionNode");

	// Config folder is not working, so we need to provide the full path to the model, weird
	// TODO: fix this
    std::string model_path = "/home/jetson/code/ros2_jetpack6_ws/src/px4-ros2/examples/yolo_detection/config/yolov8s.onnx";
    _yolo_interface = std::make_unique<Interface>(model_path, cv::Size(640, 480), "", true);


	// RMW QoS settings
	auto qos = rclcpp::QoS(1).best_effort();

	// Subscribers
	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
			     "/image_raw", qos, std::bind(&YoloDetectionNode::image_callback, this, std::placeholders::_1));

	// _vehicle_local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
	// 				      "/fmu/out/vehicle_local_position", qos, std::bind(&YoloDetectionNode::vehicle_local_position_callback, this, std::placeholders::_1));

	_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
				   "/camera_info", qos, std::bind(&YoloDetectionNode::camera_info_callback, this, std::placeholders::_1));


	// Publishers
	_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
			     "/image_proc", qos);
	// _target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
	// 			   "/target_pose", qos);
}
// void YoloDetectionNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
// {
// 	// if (msg->dist_bottom_valid) {
// 	//  _distance_to_ground = msg->dist_bottom;
// 	// } else {
// 	//  _distance_to_ground = NAN;
// 	// }

// 	// TODO: why is dist_bottom_valid false in sim?
// 	_distance_to_ground = msg->dist_bottom;
	
// }



void YoloDetectionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

         // YOLO inference
        std::vector<Detection> detections = _yolo_interface->runInterface(cv_ptr->image);

        // Process detections
        for (const auto &detection : detections)
        {
            // Draw detection results
            cv::rectangle(cv_ptr->image, detection.box, detection.color, 2);
            std::string label = cv::format("%s: %.2f", detection.className.c_str(), detection.confidence);
            int baseline;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            cv::putText(cv_ptr->image, label, cv::Point(detection.box.x, detection.box.y - labelSize.height),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, detection.color, 1);
        }

        // Annotate the image
        annotate_image(cv_ptr);

        // Publish image
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = cv_ptr->image;
        _image_pub->publish(*out_msg.toImageMsg().get());

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void YoloDetectionNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {
		return;
	}

	// Always update the camera matrix and distortion coefficients from the new message
	_camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();   // Use clone to ensure a deep copy
	_dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();   // Use clone to ensure a deep copy

	// Log the first row of the camera matrix to verify correct values
	RCLCPP_INFO(this->get_logger(), "Camera matrix updated:\n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
		    _camera_matrix.at<double>(0, 0), _camera_matrix.at<double>(0, 1), _camera_matrix.at<double>(0, 2),
		    _camera_matrix.at<double>(1, 0), _camera_matrix.at<double>(1, 1), _camera_matrix.at<double>(1, 2),
		    _camera_matrix.at<double>(2, 0), _camera_matrix.at<double>(2, 1), _camera_matrix.at<double>(2, 2));
	RCLCPP_INFO(this->get_logger(), "Camera Matrix: fx=%f, fy=%f, cx=%f, cy=%f",
		    _camera_matrix.at<double>(0, 0), // fx
		    _camera_matrix.at<double>(1, 1), // fy
		    _camera_matrix.at<double>(0, 2), // cx
		    _camera_matrix.at<double>(1, 2)  // cy
		   );

	// Check if focal length is zero after update
	if (_camera_matrix.at<double>(0, 0) == 0) {
		RCLCPP_ERROR(this->get_logger(), "Focal length is zero after update!");

	} else {
		RCLCPP_INFO(this->get_logger(), "Updated camera intrinsics from camera_info topic.");
	}
}

void YoloDetectionNode::annotate_image(cv_bridge::CvImagePtr image)

{
	// Annotate the image with the target position and marker size
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(2);
	stream << "Yolo Detections";
	std::string text_yolo = stream.str();


	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1;
	int thickness = 2;
	int baseline = 0;
	cv::Size textSize = cv::getTextSize(text_yolo, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;
	cv::Point textOrg((image->image.cols - textSize.width - 10), (image->image.rows - 10));
	cv::putText(image->image, text_yolo, textOrg, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 8);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<YoloDetectionNode>());
	rclcpp::shutdown();
	return 0;
}