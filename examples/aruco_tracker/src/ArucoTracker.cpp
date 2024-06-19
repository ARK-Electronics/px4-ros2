#include <rclcpp/rclcpp.hpp>
#include "aruco_tracker/ArucoTracker.hpp"

// Local position callback, not used at the moment
void ArucoNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
	if (msg->dist_bottom_valid) {
		distance_to_ground_ = msg->dist_bottom;
        // RCLCPP_INFO(this->get_logger(), "Distance to ground: %f", _distance_to_ground);
	} else {
		// distance_to_ground_ = NAN;
        distance_to_ground_ = msg->dist_bottom;
	}
}

// Image calback
void ArucoNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // Convert ROS image message to OpenCV image
		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);
		cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

        // RCLCPP_INFO(this->get_logger(), "Detected %zu markers", ids.size());

        if (!ids.empty()) {
            std::stringstream ss;
            for (const auto &id : ids) {
                ss << id << " ";
            }
            RCLCPP_INFO(this->get_logger(), "Marker IDs: %s", ss.str().c_str());
        }

		// Calculate marker Pose and draw axes
		for (size_t i = 0; i < ids.size(); i++) {
			// NOTE: This calculation assumes that the marker is perpendicular to the direction of the view of the camera
			// NOTE: This calculate is derived from the pinhole camera model
			
			float pixel_width = cv::norm(corners[i][0] - corners[i][1]);
            double horizontal_fov_imx214 = 1.204; // radians
            int image_width_imx214 = 1920;        // pixels
            int image_height_imx214 = 1080;       // pixels

            // Calculate focal length in pixels for IMX214
            double fx_imx214 = image_width_imx214 / (2.0 * tan(horizontal_fov_imx214 / 2.0));
            double fy_imx214 = fx_imx214; // Assuming square pixels

            // Principal point for IMX214 (assuming the center of the image)
            double cx_imx214 = image_width_imx214 / 2.0;
            double cy_imx214 = image_height_imx214 / 2.0;

            // Construct the camera matrix for IMX214
            cv::Mat camera_matrix_imx214 = (cv::Mat_<double>(3, 3) << fx_imx214, 0, cx_imx214, 0, fy_imx214, cy_imx214, 0, 0, 1);
            // Distrostion coefficients
            cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
            float marker_size = 0.5;
            float focal_length = camera_matrix_imx214.at<double>(0, 0);
            float marker_size_1 = (pixel_width / focal_length) * distance_to_ground_;

			if (!std::isnan(marker_size) && !std::isinf(marker_size)) {
				// RCLCPP_INFO(this->get_logger(), "marker_size: %f", marker_size);
                RCLCPP_INFO(this->get_logger(), "marker_size_1: %f", marker_size_1);

				std::vector<cv::Point3f> objectPoints;
				float half_size = marker_size / 2.0f;
				objectPoints.push_back(cv::Point3f(-half_size,  half_size, 0));  // top left
				objectPoints.push_back(cv::Point3f( half_size,  half_size, 0));  // top right
				objectPoints.push_back(cv::Point3f( half_size, -half_size, 0));  // bottom right
				objectPoints.push_back(cv::Point3f(-half_size, -half_size, 0));  // bottom left



                
                
                // Initialize rvec and tvec
				cv::Vec3d rvec, tvec;
                // Solve PnP problem
                cv::solvePnP(objectPoints, corners[i], camera_matrix_imx214, dist_coeffs, rvec, tvec);
                // Draw axes
				cv::aruco::drawAxis(cv_ptr->image, camera_matrix_imx214, dist_coeffs, rvec, tvec, marker_size);


                // Log rvec and tvec
				// RCLCPP_INFO(this->get_logger(), "tvec: [%f, %f, %f]", tvec[0], tvec[1], tvec[2]);
				// RCLCPP_INFO(this->get_logger(), "rvec: [%f, %f, %f]", rvec[0], rvec[1], rvec[2]);

                

				// Publish target pose
                
				geometry_msgs::msg::PoseStamped target_pose;
				target_pose.header.stamp = msg->header.stamp;
                rclcpp::Time _last_target_timestamp = msg->header.stamp;
				target_pose.header.frame_id = "camera_frame"; // TODO: frame_id
				target_pose.pose.position.x = tvec[0]+half_size;
				target_pose.pose.position.y = tvec[1]-half_size;
				target_pose.pose.position.z = tvec[2];
                RCLCPP_INFO(this->get_logger(),"target_pose.pose.position.x: %f", target_pose.pose.position.x);
                RCLCPP_INFO(this->get_logger(),"target_pose.pose.position.y: %f", target_pose.pose.position.y);
                RCLCPP_INFO(this->get_logger(),"target_pose.pose.position.z: %f", target_pose.pose.position.z);
				cv::Mat rot_mat;
				cv::Rodrigues(rvec, rot_mat);
				// RCLCPP_DEBUG(this->get_logger(), "Rot mat type: %d, rows: %d, cols: %d", rot_mat.type(), rot_mat.rows, rot_mat.cols);
			
				if (rot_mat.type() == CV_64FC1 && rot_mat.rows == 3 && rot_mat.cols == 3) {
					cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();
                    target_pose.pose.orientation.x = quat.x;
					target_pose.pose.orientation.y = quat.y;
					target_pose.pose.orientation.z = quat.z;
					target_pose.pose.orientation.w = quat.w;
                    RCLCPP_INFO(this->get_logger(),"target_pose.pose.orientation.x: %f", target_pose.pose.orientation.x);
                    RCLCPP_INFO(this->get_logger(),"target_pose.pose.orientation.y: %f", target_pose.pose.orientation.y);
                    RCLCPP_INFO(this->get_logger(),"target_pose.pose.orientation.z: %f", target_pose.pose.orientation.z);
                    RCLCPP_INFO(this->get_logger(),"target_pose.pose.orientation.w: %f", target_pose.pose.orientation.w);
                    pose_pub_->publish(target_pose);
                   
                    

					// RCLCPP_INFO(this->get_logger(), "Quat: [%f, %f, %f, %f]", quat.x, quat.y, quat.z, quat.w);
				} else {
					RCLCPP_ERROR(this->get_logger(), "rotation matrix malformed!");
				}
                // Publish image
                cv_bridge::CvImage out_msg;
                out_msg.header = msg->header;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = cv_ptr->image;
                image_pub_->publish(*out_msg.toImageMsg().get());
			}
		}

		

	}
    // Cv_bridge exception 
    catch (const cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
	}
}
// Main function
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);  ///< Initialize ROS 2.
    auto aruco_node = std::make_shared<ArucoNode>(
      "aruco_node");           ///< Create an instance of PublisherNode.
    rclcpp::spin(aruco_node);  ///< Enter a loop, pumping callbacks.
    rclcpp::shutdown();  
}