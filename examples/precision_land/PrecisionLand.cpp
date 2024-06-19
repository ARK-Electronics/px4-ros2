/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "precision_land/PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>

static const std::string kModeName = "Precision_Land_custom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{
	// Publish GoTo setpoint
	_goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);

	// Subscribe to VehicleLocalPosition
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	// Subscribe to target_pose
	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));
}



void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

	// Aruco pose in camera frame
	pose_aruco.position.x = msg->pose.position.x;
	pose_aruco.position.y = msg->pose.position.y;
	pose_aruco.position.z = msg->pose.position.z;
	pose_aruco.orientation.w = msg->pose.orientation.w;
	pose_aruco.orientation.x = msg->pose.orientation.x;
	pose_aruco.orientation.y = msg->pose.orientation.y;
	pose_aruco.orientation.z = msg->pose.orientation.z;

	// Camera pose in drone frame
	Eigen::Matrix3f R;
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
	Eigen::Quaternionf quat(R);
	camera_pose.position.x = -0.12-0.03;// camera case and camera position
	camera_pose.position.y = +(0.03+0.01878);// camera case and camera position
	camera_pose.position.z = 0;
	camera_pose.orientation.w = quat.w();
	camera_pose.orientation.x = quat.x();
	camera_pose.orientation.y = quat.y();
	camera_pose.orientation.z = quat.z();

	// Drone pose in world frame
	auto heading = _vehicle_local_position->heading();
	drone_pose.position.x = _vehicle_local_position->positionNed().x();
	drone_pose.position.y = _vehicle_local_position->positionNed().y();
	drone_pose.position.z = _vehicle_local_position->positionNed().z();
    drone_pose.orientation.w = cos(heading / 2.0);
    drone_pose.orientation.x = 0.0;
    drone_pose.orientation.y = 0.0;
    drone_pose.orientation.z = sin(heading / 2.0);


	// Convert to KDL::Frame
	KDL::Frame frame_drone, frame_camera, frame_aruco;
	tf2::fromMsg(drone_pose, frame_drone);
	tf2::fromMsg(camera_pose, frame_camera);
	tf2::fromMsg(pose_aruco, frame_aruco);

	// Calculate the pose of the aruco in the world frame
	KDL::Frame frame_drone_world =frame_drone;
	KDL::Frame frame_camera_world = frame_drone*frame_camera;
	KDL::Frame frame_aruco_world = frame_drone*frame_camera*frame_aruco;
	KDL::Frame frame_aruco_in_camera = frame_camera*frame_aruco;
	// Convert to geometry_msgs::Pose
	geometry_msgs::msg::Pose pose_aruco_in_camera = tf2::toMsg(frame_aruco_in_camera);
	geometry_msgs::msg::Pose pose_aruco_in_world = tf2::toMsg(frame_aruco_world);
	geometry_msgs::msg::Pose pose_drone_in_world = tf2::toMsg(frame_drone_world);
	geometry_msgs::msg::Pose pose_camera_in_world = tf2::toMsg(frame_camera_world);
	
	// Calculate the heading of the aruco in the world frame
	auto q = Eigen::Quaternionf(pose_aruco_in_world.orientation.w, pose_aruco_in_world.orientation.x, pose_aruco_in_world.orientation.y, pose_aruco_in_world.orientation.z);
	
	
	
	_last_target_timestamp = msg->header.stamp;

		if (!counter)
	{
		// Logging the poses
		RCLCPP_INFO(_node.get_logger(), "Aruco pose in camera frame: %f, %f, %f", pose_aruco_in_camera.position.x, pose_aruco_in_camera.position.y, pose_aruco_in_camera.position.z);
		RCLCPP_INFO(_node.get_logger(), "Aruco orientation in camera frame: %f,%f, %f, %f", pose_aruco_in_camera.orientation.w,pose_aruco_in_camera.orientation.x, pose_aruco_in_camera.orientation.y, pose_aruco_in_camera.orientation.z);
		
		RCLCPP_INFO(_node.get_logger(), "Aruco pose in world frame: %f, %f, %f", pose_aruco_in_world.position.x, pose_aruco_in_world.position.y, pose_aruco_in_world.position.z);
		RCLCPP_INFO(_node.get_logger(), "Aruco orientation in world frame: %f,%f, %f, %f", pose_aruco_in_world.orientation.w,pose_aruco_in_world.orientation.x, pose_aruco_in_world.orientation.y, pose_aruco_in_world.orientation.z);

		RCLCPP_INFO(_node.get_logger(), "Drone pose in world frame: %f, %f, %f", pose_drone_in_world.position.x, pose_drone_in_world.position.y, pose_drone_in_world.position.z);
		RCLCPP_INFO(_node.get_logger(), "Drone orientation in world frame: %f,%f, %f, %f", pose_drone_in_world.orientation.w,pose_drone_in_world.orientation.x, pose_drone_in_world.orientation.y, pose_drone_in_world.orientation.z);
		
		RCLCPP_INFO(_node.get_logger(), "Camera pose in world frame: %f, %f, %f", pose_camera_in_world.position.x, pose_camera_in_world.position.y, pose_camera_in_world.position.z);
		RCLCPP_INFO(_node.get_logger(), "Camera orientation in world frame: %f,%f, %f, %f", pose_camera_in_world.orientation.w,pose_camera_in_world.orientation.x, pose_camera_in_world.orientation.y, pose_camera_in_world.orientation.z);
		// Set the target position, only once in each Approach state
		_target_position = Eigen::Vector3f(pose_aruco_in_world.position.x, pose_aruco_in_world.position.y, pose_aruco_in_world.position.z);
		_target_heading = px4_ros2::quaternionToYaw(q);
		
		counter = 1;
	}

}

void PrecisionLand::onActivate()
{
	_state = State::Search;
}

void PrecisionLand::onDeactivate()
{
	// TODO:
}

// GoTo setpoint type has a default update rate of 30Hz
void PrecisionLand::updateSetpoint(float dt_s)
{
	switch (_state) {
	case State::Search: {
		RCLCPP_INFO(_node.get_logger(), "State::Search");
		// Start the search, going home for now
		// TODO: implement a search pattern
		auto home_position = Eigen::Vector3f(0.0, 0.0,_vehicle_local_position->positionNed().z() );
		auto heading = _vehicle_local_position->heading();
		_goto_setpoint->update(home_position, heading);

		// Go to Approach state if a target has been set
		if(_target_position.norm() > 0.0) {
			_state = State::Approach;
		}

        break;
	}

	case State::Approach: {
		RCLCPP_INFO(_node.get_logger(), "State::Approach");

		auto position = Eigen::Vector3f(_target_position.x(),_target_position.y(), _vehicle_local_position->positionNed().z());
		auto heading = _vehicle_local_position->heading();
		RCLCPP_INFO(_node.get_logger(), "position_calculated as target: %f, %f", _target_position.x(), _target_position.y());
		
		// RCLCPP_INFO(_node.get_logger(), "heading: %f", heading);
		RCLCPP_INFO(_node.get_logger(), "local_position: %f, %f", _vehicle_local_position->positionNed().x(), _vehicle_local_position->positionNed().y());
		

		_goto_setpoint->update(position, _target_heading);
		

		// -- Check std::absf(Position - Target < Threshold) --> State Transition
		if (positionReached(position)) {
			_state = State::Descend;
		}

		break;
	}

	case State::Descend: {
		RCLCPP_INFO(_node.get_logger(), "State::Descend");

		// TODO: check if distance to bottom is still valid
		// - if invalid stop
		// - start timeout, switch to failsafe if timed out : failsafe = normal land

		// TODO: while in failsafe (normal land) keep checking conditions to switch back into precision land

		// TODO: Z setpoint very large (thru ground).. rewrite to use direct position_setpoint instead of GoTo type
		// TODO: use parameters
		float max_h = 3;
		float max_v = 1;
		float max_heading = 180.0_deg;
		// Z target one meter below ground
		float z_target = _vehicle_local_position->positionNed().z() + _vehicle_local_position->distanceGround() + 1;

		auto position = Eigen::Vector3f(_vehicle_local_position->positionNed().x(), _vehicle_local_position->positionNed().y(), z_target);
		// auto heading = _target_heading;
		auto heading = _vehicle_local_position->heading();

		_goto_setpoint->update(position, heading, max_h, max_v, max_heading);

		// TODO: use land_detector or otherwise
		// TODO: use a paramater
		// This velocity check was not working, it switched to Finished state before the drone landed
		// float kDeltaVelocity = 0.1;
		// auto velocity = _vehicle_local_position->velocityNed();
		// bool landed = velocity.norm() < kDeltaVelocity;
    
		// If the drone is closer to the ground than 6m, switch to Approach state and update the target position
		if ((_vehicle_local_position->distanceGround()<6.0)&& check_again_6) {
			
			_state = State::Approach;
			check_again_6 = 0;
			counter=0;
		}
		// If the drone is closer to the ground than 3m, switch to Approach state and update the target position
		else if ((_vehicle_local_position->distanceGround()<3.0)&& check_again_3)
		{
			_state = State::Approach;
			check_again_3 = 0;
			counter=0;
		}
		// If the drone is closer to the ground than 0.7m, switch to Finished state
		else if (_vehicle_local_position->distanceGround()<0.7)
		{
			_state = State::Finished;
		}
		

		break;
	}

	case State::Finished: {
		RCLCPP_INFO(_node.get_logger(), "State::Finished");
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	// TODO: parameters for delta_position and delta_velocitry
	static constexpr float kDeltaPosition = 0.1f;
	static constexpr float kDeltaVelocitry = 0.1f;

	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	RCLCPP_INFO(_node.get_logger(), "delta_pos: %f", delta_pos.norm());
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < kDeltaPosition) && (velocity.norm() < kDeltaVelocitry);
}

bool PrecisionLand::headingReached(float target) const
{
	// TODO: parameter for delta heading
	static constexpr float kDeltaHeading = 5.0_deg;
	float heading = _vehicle_local_position->heading();
	return fabsf(px4_ros2::wrapPi(target - heading)) < kDeltaHeading;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
