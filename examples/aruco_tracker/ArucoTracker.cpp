#include "ArucoTracker.hpp"
#include <sstream>

ArucoTrackerNode::ArucoTrackerNode()
	: Node("aruco_tracker_node")
{
	RCLCPP_INFO(this->get_logger(), "Starting ArucoTrackerNode");

	// Define aruco tag dictionary to use
	_dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

	// RMW QoS settings
	auto qos = rclcpp::QoS(1).best_effort();

	// Subscribers
	_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
		"/camera", qos, std::bind(&ArucoTrackerNode::image_callback, this, std::placeholders::_1));

	_vehicle_local_position_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", qos, std::bind(&ArucoTrackerNode::vehicle_local_position_callback, this, std::placeholders::_1));

	_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
	    "/camera_info", qos, std::bind(&ArucoTrackerNode::camera_info_callback, this, std::placeholders::_1));


	// Publishers
	_image_pub = this->create_publisher<sensor_msgs::msg::Image>(
		"/image_proc", qos);
	_target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		"/target_pose", qos);
}
void ArucoTrackerNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	// if (msg->dist_bottom_valid) {
	//  _distance_to_ground = msg->dist_bottom;
	// } else {
	//  _distance_to_ground = NAN;
	// }

	// TODO: why is dist_bottom_valid false in sim?
	_distance_to_ground = msg->dist_bottom;
	// RCLCPP_INFO(this->get_logger(), "dist_bottom: %f", msg->dist_bottom);
}

void ArucoTrackerNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	try {
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(cv_ptr->image, _dictionary, corners, ids);
		cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

		if (!_camera_matrix.empty() && !_dist_coeffs.empty() && !std::isnan(_distance_to_ground)) {
			// Calculate marker Pose and draw axes

            std::vector<std::vector<cv::Point2f>> undistortedCorners;
            for (const auto& corner : corners) {
                std::vector<cv::Point2f> undistortedCorner;
                cv::undistortPoints(corner, undistortedCorner, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);
                undistortedCorners.push_back(undistortedCorner);
            }

			for (size_t i = 0; i < ids.size(); i++) {
				// NOTE: This calculation assumes that the marker is perpendicular to the direction of the view of the camera
				// NOTE: This calculate is derived from the pinhole camera model
				// Real world size = (pixel size / focal length) * distance to object
				// float pixel_width = cv::norm(corners[i][0] - corners[i][1]);
				float pixel_width = cv::norm(undistortedCorners[i][0] - undistortedCorners[i][1]);
				float focal_length = _camera_matrix.at<double>(0, 0);
				float marker_size = (pixel_width / focal_length) * _distance_to_ground;
				// RCLCPP_INFO(this->get_logger(), "pixel_width: %f", pixel_width);
				// RCLCPP_INFO(this->get_logger(), "focal_length: %f", focal_length);
				// RCLCPP_INFO(this->get_logger(), "marker_size: %f", marker_size);

				if (!std::isnan(marker_size) && !std::isinf(marker_size) && marker_size > 0) {

					// Calculate marker size from camera intrinsics
					// RCLCPP_INFO(this->get_logger(), "marker_size: %f", marker_size);
					float half_size = marker_size / 2.0f;
					std::vector<cv::Point3f> objectPoints = {
					    cv::Point3f(-half_size,  half_size, 0),  // top left
					    cv::Point3f( half_size,  half_size, 0),  // top right
					    cv::Point3f( half_size, -half_size, 0),  // bottom right
					    cv::Point3f(-half_size, -half_size, 0)   // bottom left
					};

					// Use PnP solver (look it up!)
					cv::Vec3d rvec, tvec;
					cv::solvePnP(objectPoints, undistortedCorners[i], _camera_matrix, cv::noArray(), rvec, tvec);
					// Annotate the image
					cv::aruco::drawAxis(cv_ptr->image, _camera_matrix, cv::noArray(), rvec, tvec, marker_size);

					double target_x = -tvec[0];
					double target_y = -tvec[1];
					double target_z = -tvec[2];

					std::ostringstream stream;
					stream << std::fixed << std::setprecision(2);
					stream << "X: "  << target_x << " Y: " << target_y << " Z: " << target_z;
					std::string text_xyz = stream.str();

					int fontFace = cv::FONT_HERSHEY_SIMPLEX;
					double fontScale = 1;
					int thickness = 2;
					int baseline = 0;
					cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
					baseline += thickness;
					cv::Point textOrg((cv_ptr->image.cols - textSize.width - 10), (cv_ptr->image.rows - 10));
					cv::putText(cv_ptr->image, text_xyz, textOrg, fontFace, fontScale, cv::Scalar(0,255,255), thickness, 8);

					// RCLCPP_INFO(this->get_logger(), "tvec: [%f, %f, %f]", tvec[0], tvec[1], tvec[2]);
					// RCLCPP_INFO(this->get_logger(), "rvec: [%f, %f, %f]", rvec[0], rvec[1], rvec[2]);

					// Publish target pose
					geometry_msgs::msg::PoseStamped target_pose;
					target_pose.header.stamp = msg->header.stamp;
					target_pose.header.frame_id = "camera_frame"; // TODO: frame_id

					// Convert to drone local frame (LFD)
					// Camera frame is RBU
					target_pose.pose.position.x = target_x;
					target_pose.pose.position.y = target_y;
					target_pose.pose.position.z = target_z;
					cv::Mat rot_mat;
					cv::Rodrigues(rvec, rot_mat);
					RCLCPP_DEBUG(this->get_logger(), "Rot mat type: %d, rows: %d, cols: %d", rot_mat.type(), rot_mat.rows, rot_mat.cols);
					// TODO: why is the rotation matrix sometimes malformed?
					if (rot_mat.type() == CV_64FC1 && rot_mat.rows == 3 && rot_mat.cols == 3) {
						cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();
						target_pose.pose.orientation.x = -quat.x;
						target_pose.pose.orientation.y = -quat.y;
						target_pose.pose.orientation.z = -quat.z;
						target_pose.pose.orientation.w = quat.w;

						_target_pose_pub->publish(target_pose);
					} else {
						RCLCPP_ERROR(this->get_logger(), "rotation matrix malformed!");
					}
				}
			}
		} else {
			// RCLCPP_ERROR(this->get_logger(), "distance to ground is NAN");
		}

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

void ArucoTrackerNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
	if (!_camera_matrix.empty() && !_dist_coeffs.empty()) {
		return;
	}

    // Always update the camera matrix and distortion coefficients from the new message
    _camera_matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();  // Use clone to ensure a deep copy
    _dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double *>(msg->d.data())).clone();  // Use clone to ensure a deep copy

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

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoTrackerNode>());
	rclcpp::shutdown();
	return 0;
}