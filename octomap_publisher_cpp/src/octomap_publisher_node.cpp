#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class OctomapPublisherNode : public rclcpp::Node
{
public:
    OctomapPublisherNode() : Node("octomap_publisher_node")
    {
        // Publisher to monitored planning scene
        planning_scene_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/monitored_planning_scene", 10);

        // Subscriber to /octomap_binary topic published by octomap_server_node
        octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10, std::bind(&OctomapPublisherNode::octomap_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Octomap Publisher Node has been started.");
    }

private:
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        // Create PlanningScene message
        auto planning_scene_msg = moveit_msgs::msg::PlanningScene();

        // Populate Octomap part of PlanningScene message
        planning_scene_msg.world.octomap.octomap = *msg;
        planning_scene_msg.world.octomap.header = msg->header;

        // Indicate this is a diff (incremental update)
        planning_scene_msg.is_diff = true;
        planning_scene_msg.robot_state.is_diff = true;

        // Publish PlanningScene message
        planning_scene_publisher_->publish(planning_scene_msg);
        RCLCPP_INFO(this->get_logger(), "Published updated Octomap to /monitored_planning_scene");

        // Directly apply octomap to planning scene
        apply_octomap_to_planning_scene(msg);
    }

    void apply_octomap_to_planning_scene(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg)
    {
        // Apply octomap to planning scene directly using PlanningSceneInterface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        auto collision_object = create_collision_object(octomap_msg);
        planning_scene_interface.applyCollisionObject(collision_object);
    }

    moveit_msgs::msg::CollisionObject create_collision_object(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg)
    {
        // Create CollisionObject from the octomap_msg
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "octomap_object";
        collision_object.header = octomap_msg->header;
        // Assuming octomap is appropriately handled (you may need more processing here)
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        return collision_object;
    }

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_publisher_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OctomapPublisherNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
