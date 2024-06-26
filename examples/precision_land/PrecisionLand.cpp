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
	_vehicle_attitude= std::make_shared<px4_ros2::OdometryAttitude>(*this);

	// Subscribe to target_pose
	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	auto q = Eigen::Quaternionf(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
	_target_heading = px4_ros2::quaternionToYaw(q);
	_last_target_timestamp = msg->header.stamp;

	auto vehicle_q = _vehicle_attitude->attitude();

	(void)vehicle_q;

    // Fetch vehicle's current heading (yaw)
    float vehicle_heading = _vehicle_local_position->heading();  // Placeholder for your method to get the heading

	// TODO: rotate the XYZ into world frame
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = Eigen::AngleAxisf(vehicle_heading, Eigen::Vector3f::UnitZ());

    auto target_position = Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    _target_position = rotation_matrix * target_position;
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

		auto current_time = _node.get_clock()->now();
		auto time_delta = rclcpp::Duration::from_seconds(0.2); // 200 milliseconds

		if ((current_time - _last_target_timestamp) > time_delta) {
			// _state = State::Approach;
			_state = State::AlignHeading;
			_align_position = _vehicle_local_position->positionNed();


			// TODO: configurable param in the case that target is not visible
			// _approach_altitude = _vehicle_local_position->positionNed().z();
		}

		break;
	}

	case State::AlignHeading: {
		RCLCPP_INFO(_node.get_logger(), "State::AlignHeading");
		RCLCPP_INFO(_node.get_logger(), "current_heading: %f", double(_vehicle_local_position->heading()));
		RCLCPP_INFO(_node.get_logger(), "target_heading: %f", double(_target_heading));


		auto heading_setpoint = _vehicle_local_position->heading() + _target_heading;
		_goto_setpoint->update(_align_position, heading_setpoint);

		if (headingReached(heading_setpoint)) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			_state = State::Approach;
		}
		break;
	}

	case State::Approach: {
		RCLCPP_INFO(_node.get_logger(), "State::Approach");

		auto target_x = _vehicle_local_position->positionNed().x() + _target_position.x();
		auto target_y = _vehicle_local_position->positionNed().y() + _target_position.y();

		auto position = Eigen::Vector3f(target_x, target_y, _approach_altitude);
		// auto heading = _target_heading;
		auto heading = _vehicle_local_position->heading() + _target_heading;

		_goto_setpoint->update(position, heading);

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
		float max_h = 0.25;
		float max_v = 0.25;
		float max_heading = 90.0_deg;
		// Z target one meter below ground
		auto target_x = _vehicle_local_position->positionNed().x() + _target_position.x();
		auto target_y = _vehicle_local_position->positionNed().y() + _target_position.y();
		auto target_z = _vehicle_local_position->positionNed().z() + _vehicle_local_position->distanceGround() + 1;

		auto position = Eigen::Vector3f(target_x, target_y, target_z);
		// auto heading = _target_heading;
		auto heading = _vehicle_local_position->heading() + _target_heading;

		_goto_setpoint->update(position, heading, max_h, max_v, max_heading);

		// TODO: use land_detector or otherwise
		// TODO: use a paramater
		float kDeltaVelocity = 0.1;
		auto velocity = _vehicle_local_position->velocityNed();
		bool landed = velocity.norm() < kDeltaVelocity;

		if (landed) {
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
	static constexpr float kDeltaPosition = 0.25f;
	static constexpr float kDeltaVelocity = 0.25f;

	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < kDeltaPosition) && (velocity.norm() < kDeltaVelocity);
}

bool PrecisionLand::headingReached(float target) const
{
	// TODO: parameter for delta heading
	static constexpr float kDeltaHeading = 10.0_deg;
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
