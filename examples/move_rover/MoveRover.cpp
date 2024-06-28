/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "MoveRover.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "MoveRoverCustom";
static const bool kEnableDebugOutput = true;
static const std::string kTopicNamespacePrefix = "/px4_2/";

using namespace px4_ros2::literals;

MoveRover::MoveRover(rclcpp::Node& node, const std::string& topic_namespace_prefix)
	: ModeBase(node, kModeName, topic_namespace_prefix)
	, _node(node)
{

	// Publish TrajectorySetpoint
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	// Subscribe to VehicleLocalPosition
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);


}



void MoveRover::onActivate()
{
	generateWaypoints();
	RCLCPP_INFO(_node.get_logger(), "Switching to State::Move");
	_state = State::Move;
}

void MoveRover::onDeactivate()
{
	// TODO:
}

void MoveRover::updateSetpoint(float dt_s)
{
	switch (_state) {
	case State::Move: {

		// Get the target position
		Eigen::Vector3f target_position = _waypoints[_waypoint_index];
		// Go to the next waypoint
		// Publisher for trajectory setpoint
		_trajectory_setpoint_msg.timestamp = _node.now().nanoseconds() / 1000;
		_trajectory_setpoint_msg.position = {target_position.x(), target_position.y(), NAN};
		_trajectory_setpoint_msg.velocity = {NAN, NAN, NAN};
		_trajectory_setpoint_msg.acceleration = {NAN, NAN, NAN};
		_trajectory_setpoint_msg.jerk = {NAN, NAN, NAN};
		_trajectory_setpoint_msg.yaw = NAN;
		_trajectory_setpoint_msg.yawspeed = NAN;
		// Publish the trajectory setpoint
		_trajectory_setpoint->update(_trajectory_setpoint_msg);


		// Check if the drone has reached the target position
		if (positionReached(target_position)) {
			_waypoint_index++;

			// If we have reached all waypoints, start over
			if (_waypoint_index >= static_cast<int>(_waypoints.size())) {
				// Switch to finished state
				_state = State::Finished;
				RCLCPP_INFO(_node.get_logger(), "Switching to State::Finished");
			}
		}




		break;
	}


	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

void MoveRover::generateWaypoints()
{
	// Generate waypoints for a figure-8 pattern
	// Parameters for the figure-8 pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double start_z = 0.0;
	double A = 5.0; // Width of the figure-8
	double B = 5.0; // Height of the figure-8
	int num_points = 10;  // Number of points in the figure-8 pattern
	std::vector<Eigen::Vector3f> waypoints;

	for (int i = 0; i <= num_points; ++i) {
		double t = 2.0 * M_PI * i / num_points; // Parametric angle
		double x = start_x + A * std::sin(t);
		double y = start_y + B * std::sin(2 * t);
		waypoints.push_back(Eigen::Vector3f(x, y, start_z));
	}

	_waypoints = waypoints;
}

bool MoveRover::positionReached(const Eigen::Vector3f& target) const
{
	// Parameters for delta_position and delta_velocitry
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
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<MoveRover>>(kModeName, kEnableDebugOutput, kTopicNamespacePrefix));
	rclcpp::shutdown();

	return 0;
}
