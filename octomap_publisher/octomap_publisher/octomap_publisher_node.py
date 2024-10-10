import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene # CollisionObject
# from moveit_ros_planning_interface import PlanningSceneInterface

class OctomapPublisherNode(Node):
    def __init__(self):
        super().__init__('octomap_publisher_node')
        
        # Publisher to monitored planning scene
        self.planning_scene_publisher = self.create_publisher(PlanningScene, '/monitored_planning_scene', 10)
        
        # Subscriber to /octomap_binary topic published by octomap_server_node
        self.octomap_subscriber = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.octomap_callback,
            10
        )
        self.get_logger().info('Octomap Publisher Node has been started.')

    def octomap_callback(self, msg):
        # Create PlanningScene message
        planning_scene_msg = PlanningScene()
        
        # Populate Octomap part of PlanningScene message
        planning_scene_msg.world.octomap.octomap = msg
        planning_scene_msg.world.octomap.header = msg.header
        
        # Indicate this is a diff (incremental update)
        planning_scene_msg.is_diff = True
        planning_scene_msg.robot_state.is_diff = True
        
        # Publish PlanningScene message
        self.planning_scene_publisher.publish(planning_scene_msg)
        self.get_logger().info('Published updated Octomap to /monitored_planning_scene')

        # # Directly apply octomap to planning scene for collision detection
        # self.apply_octomap_to_planning_scene(msg)

    # def apply_octomap_to_planning_scene(self, octomap_msg):
    #     # Apply octomap to planning scene directly using PlanningSceneInterface
    #     planning_scene_interface = PlanningSceneInterface()
    #     planning_scene_interface.applyCollisionObject(self.create_collision_object(octomap_msg))

    # def create_collision_object(self, octomap_msg):
    #     # Create CollisionObject from the octomap_msg
    #     collision_object = CollisionObject()
    #     collision_object.world.octomap.octomap = octomap_msg
    #     collision_object.world.octomap.header = octomap_msg.header
    #     collision_object.is_diff = True
    #     return collision_object

def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    octomap_publisher_node = OctomapPublisherNode()
    
    # Spin node
    rclpy.spin(octomap_publisher_node)
    
    # Shutdown
    octomap_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
