/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

#include <vector>

class MoveRover : public px4_ros2::ModeBase
{
public:
	explicit MoveRover(rclcpp::Node& node, const std::string& topic_namespace_prefix = "");

	// See ModeBasep
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	bool positionReached(const Eigen::Vector3f& target) const;

	enum class State {
		Move, 	// Searches for target -- TODO: optionally perform a search pattern
		Finished
	};

	// ros2
	rclcpp::Node& _node;

	// px4_ros2_cpp
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// Data
	State _state = State::Move;


	// Trajectory setpoint
	px4_msgs::msg::TrajectorySetpoint _trajectory_setpoint_msg;

	rclcpp::Time _last_target_timestamp;

	// Waypoints for Figure 8 pattern
	std::vector<Eigen::Vector3f> _waypoints;
	// Waypoint pattern generation
	void generateWaypoints();
	// Waypoint pattern index
	int _waypoint_index = 0;


};
