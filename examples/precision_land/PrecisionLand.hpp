/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Core>
#include <algorithm>

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node);

	// Subscription calbacks
	void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

	// See ModeBase
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	bool positionReached(const Eigen::Vector3f& target) const;
	bool headingReached(float target) const;

	enum class State {
		Search, // Searches for target -- TODO: optionally perform a search pattern
		Approach, // Positioning over landing target while maintaining altitude
		Descend, // Stay over landing target while descending
		Finished
	};

	State _state = State::Search;

	// Subscriptions
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _target_pose_sub;

	//////////////////
	// px4_ros2_cpp //
	//////////////////

	// Subscription
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	// Publication
	std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;

  	rclcpp::Node& _node;

  	// Data
  	Eigen::Vector3f _target_position = {};
  	float _target_heading = {};
};
