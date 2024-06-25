/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "PrecisionLandCustom";
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

	// Subscribe to VehicleAttitude
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	// Subscribe to target_pose
	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	// Aruco pose in camera frame
	_aruco_pose.position.x = msg->pose.position.x;
	_aruco_pose.position.y = msg->pose.position.y;
	_aruco_pose.position.z = msg->pose.position.z;
	_aruco_pose.orientation.w = msg->pose.orientation.w;
	_aruco_pose.orientation.x = msg->pose.orientation.x;
	_aruco_pose.orientation.y = msg->pose.orientation.y;
	_aruco_pose.orientation.z = msg->pose.orientation.z;

	// Camera pose in drone frame
	Eigen::Matrix3f R;
	R << 0, -1, 0,
	1, 0, 0,
	0, 0, 1;
	Eigen::Quaternionf quat(R);
	_camera_pose.position.x = 0;// camera case and camera position
	_camera_pose.position.y = 0;// camera case and camera position
	_camera_pose.position.z = 0;// This is teh vertical shift, I think it can be tuned
	_camera_pose.orientation.w = quat.w();
	_camera_pose.orientation.x = quat.x();
	_camera_pose.orientation.y = quat.y();
	_camera_pose.orientation.z = quat.z();

	// Drone pose in world frame
	auto vehicle_q = _vehicle_attitude->attitude();
	_drone_pose.position.x = _vehicle_local_position->positionNed().x();
	_drone_pose.position.y = _vehicle_local_position->positionNed().y();
	_drone_pose.position.z = _vehicle_local_position->positionNed().z();
	_drone_pose.orientation.w = vehicle_q.w();
	_drone_pose.orientation.x = vehicle_q.x();
	_drone_pose.orientation.y = vehicle_q.y();
	_drone_pose.orientation.z = vehicle_q.z();

	// Convert to KDL::Frame
	KDL::Frame frame_drone, frame_camera, frame_aruco;
	tf2::fromMsg(_drone_pose, frame_drone);
	tf2::fromMsg(_camera_pose, frame_camera);
	tf2::fromMsg(_aruco_pose, frame_aruco);

	// Calculate the pose of the aruco in the world frame
	KDL::Frame frame_aruco_world = frame_drone * frame_camera * frame_aruco;

	// Convert to geometry_msgs::Pose
	geometry_msgs::msg::Pose pose_aruco_in_world = tf2::toMsg(frame_aruco_world);

	// Fetch the heading of the aruco in the world frame
	auto q = Eigen::Quaternionf(pose_aruco_in_world.orientation.w, pose_aruco_in_world.orientation.x, pose_aruco_in_world.orientation.y, pose_aruco_in_world.orientation.z);
	_target_heading = px4_ros2::quaternionToYaw(q);

	// Fetch the position of the aruco in the world frame
	auto target_position = Eigen::Vector3f(pose_aruco_in_world.position.x, pose_aruco_in_world.position.y, pose_aruco_in_world.position.z);
	_target_position = target_position;

	_last_target_timestamp = msg->header.stamp;
}

void PrecisionLand::onActivate()
{
	generateSearchWaypoints();
	// Initialize _target_position with NaN values
	_target_position.setConstant(std::numeric_limits<float>::quiet_NaN());
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

		// TODO: logic should use timestamp to detect stale data.

		// If the market has not been detected, search for it
		if (std::isnan(_target_position.x())) {
			// RCLCPP_INFO(_node.get_logger(), "Target position: %f, %f, %f", double(_target_position.x()), double(_target_position.y()), double(_target_position.z()));
			Eigen::Vector3f target_position = _search_waypoints[_search_waypoint_index];
			// Go to the next waypoint
			_goto_setpoint->update(target_position, _vehicle_local_position->heading());

			if (positionReached(target_position)) {
				_search_waypoint_index++;

				// If we have searched all waypoints, start over
				if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
					_search_waypoint_index = 0;
				}
			}
		}

		// If the marker has been detected Approach it
		else {
			RCLCPP_INFO(_node.get_logger(), "Switching to State::Approach");
			_state = State::Approach;
		}

		break;
	}

	case State::Approach: {
		_approach_altitude = _vehicle_local_position->positionNed().z();

		auto position = Eigen::Vector3f(_target_position.x(), _target_position.y(), _approach_altitude);
		auto heading = _target_heading;

		_goto_setpoint->update(position, heading);

		// -- Check std::absf(Position - Target < Threshold) --> State Transition
		if (positionReached(position)) {
			RCLCPP_INFO(_node.get_logger(), "Switching to State::Descend");
			_state = State::Descend;
		}

		break;
	}

	case State::Descend: {
		// TODO: check if distance to bottom is still valid
		// - if invalid stop
		// - start timeout, switch to failsafe if timed out : failsafe = normal land

		// TODO: while in failsafe (normal land) keep checking conditions to switch back into precision land

		// TODO: Z setpoint very large (thru ground).. rewrite to use direct position_setpoint instead of GoTo type
		// TODO: use parameters
		float max_h = 5;
		float max_v = 0.35;
		float max_heading = 90.0_deg;

		// Z target one meter below ground
		auto target_z = _vehicle_local_position->positionNed().z() + _vehicle_local_position->distanceGround() + 10;
		auto position = Eigen::Vector3f(_target_position.x(), _target_position.y(), target_z);
		auto heading = _target_heading;

		_goto_setpoint->update(position, heading, max_h, max_v, max_heading);

		// TODO: use land_detector

		// if (landed) {
		// 	RCLCPP_INFO(_node.get_logger(), "Switching to State::Finished");
		// 	_state = State::Finished;
		// }

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

void PrecisionLand::generateSearchWaypoints()
{
	// Generate paralelltrack search wayponts
	// Probably during execution the PositionReached function could have a higher threshold to make the search faster
	// The search waypoints are generated in the NED frame
	// Parameters for the search pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double start_z = _vehicle_local_position->positionNed().z();
	double width = 5.0;
	double length = 20.0;
	double spacing = 4.0;
	bool reverse = false;
	std::vector<Eigen::Vector3f> waypoints;

	// Generate waypoints
	for (double i = 0; i <= length; i += spacing) {
		// Add waypoints in reverse order to make the drone fly in a zigzag pattern
		if (reverse) {
			waypoints.push_back(Eigen::Vector3f(start_x + i, start_y + width, start_z));
			waypoints.push_back(Eigen::Vector3f(start_x + i + spacing, start_y + width, start_z));

		} else {
			waypoints.push_back(Eigen::Vector3f(start_x + i, start_y, start_z));
			waypoints.push_back(Eigen::Vector3f(start_x + i + spacing, start_y, start_z));
		}

		reverse = !reverse;
	}

	// Reverse the waypoints to make the drone end the search at the starting point of the pattern
	std::reverse(waypoints.begin(), waypoints.end());
	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	// TODO: parameters for delta_position and delta_velocitry
	static constexpr float kDeltaPosition = 0.25f;
	static constexpr float kDeltaVelocity = 0.25f;

	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < kDeltaPosition) && (velocity.norm() < kDeltaVelocity);
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();

	return 0;
}
