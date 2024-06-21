#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/srv/spawn_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>

class ModelSpawner : public rclcpp::Node
{
public:
	ModelSpawner() : Node("model_spawner")
	{
		// NOTE: model spawning is not supported in ros_gz
		std::string service_name = "/world/default/create";
		client_ = this->create_client<ros_gz_interfaces::srv::SpawnEntity>(service_name);

		while (!client_->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}

			RCLCPP_INFO(this->get_logger(), "Waiting for the service [%s] to appear...", service_name.c_str());
		}

		auto request = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
		request->entity_factory.name = "target";  // Set the name of the model
		request->entity_factory.allow_renaming = false;  // Set allow_renaming to false
		request->entity_factory.sdf = R"(
	<?xml version="1.0" encoding="UTF-8"?>
	<sdf version='1.9'>
	<model name='arucotag'>
	<static>true</static>
	<pose>0 0 0.001 0 0 0</pose>
	<link name='base'>
		<visual name='base_visual'>
			<geometry>
				<plane>
					<normal>0 0 1</normal>
					<size>0.5 0.5</size>
				</plane>
			</geometry>
			<material>
				<diffuse>1 1 1 1</diffuse>
				<specular>0.4 0.4 0.4 1</specular>
				<pbr>
					<metal>
						<albedo_map>model://arucotag/arucotag.png</albedo_map>
					</metal>
				</pbr>
			</material>
		</visual>
	</link>
	</model>
	</sdf>
	)";  // SDF description

		geometry_msgs::msg::Pose pose;  // Define the pose
		pose.position.x = 0.0;
		pose.position.y = 0.0;
		pose.position.z = 0.0;
		pose.orientation.w = 1.0;  // Identity quaternion (no rotation)
		request->entity_factory.pose = pose;  // Assign the pose

		request->entity_factory.relative_to = "world";  // Set relative_to to "world"

		auto result_future = client_->async_send_request(request);

		// Use shared_from_this to correctly pass the shared pointer
		if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
		    rclcpp::FutureReturnCode::SUCCESS) {
			RCLCPP_INFO(this->get_logger(), "Model spawned successfully.");

		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to call service.");
		}
	}

private:
	rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr client_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ModelSpawner>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
