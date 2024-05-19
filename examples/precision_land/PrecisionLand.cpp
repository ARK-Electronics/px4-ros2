/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Core>
#include <algorithm>

static const std::string kModeName = "Precision Land";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node)
		: ModeBase(node, kModeName)
		, _node(node)
	{
		// Publish GoTo setpoint
		_goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);

		// Subscribe to VehicleLocalPosition
		_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
		// Subscribe to target_pose
		_target_pose_sub = _node.create_subscription<geometry_msgs::msg::Pose>("/target_pose",
			rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));
	}

	void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
	{
		// update target pose
	}

	void onActivate() override
	{
		_state = State::Search;
	}

	void onDeactivate() override {}

	void updateSetpoint(float dt_s) override
	{
		// setpoint type GoTo has a default update rate of 30Hz
		switch (_state) {
		case State::Search:
			{
				// Check target_pose timestamp
				break;
			}
		case State::Approach:
			{
				// Check std::absf(Position - Target < Threshold)
				break;
			}
		case State::Descend:
			{
				// Send GoTos with TargetPose
				// Handle loss of target for T_delta
				// ... fallback
				// Normal land?
				break;
			}
		case State::Finished:
			{
				// Check landed
				break;
			}
		}

		// _start_position_m = _vehicle_local_position->positionNed();


		// _vehicle_local_position->heading();
		// positionReached();
		// headingReached();
		// _goto_setpoint->update(_start_position_m);
		// _goto_setpoint->update(target_position_m, heading_target_rad);

		// _goto_setpoint->update(
		// 	target_position_m,
		// 	_spinning_heading_rad,
		// 	max_horizontal_velocity_m_s,
		// 	max_vertical_velocity_m_s,
		// 	max_heading_rate_rad_s);


	}

private:

	enum class State {
		Search, // Searches for target -- TODO: optionally perform a search pattern
		Approach, // Positioning over landing target while maintaining altitude
		Descend, // Stay over landing target while descending
		Finished
	} _state;

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

	bool positionReached(const Eigen::Vector3f& target_position_m) const
	{
		static constexpr float kPositionErrorThreshold = 0.5f; // [m]
		static constexpr float kVelocityErrorThreshold = 0.3f; // [m/s]
		const Eigen::Vector3f position_error_m = target_position_m -
				_vehicle_local_position->positionNed();
		return (position_error_m.norm() < kPositionErrorThreshold) &&
		       (_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
	}

	bool headingReached(float target_heading_rad) const
	{
		static constexpr float kHeadingErrorThreshold = 7.0_deg;
		const float heading_error_wrapped = px4_ros2::wrapPi(
				target_heading_rad - _vehicle_local_position->heading());
		return fabsf(heading_error_wrapped) < kHeadingErrorThreshold;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
