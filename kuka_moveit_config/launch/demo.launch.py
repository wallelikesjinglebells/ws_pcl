import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("kuka_kr210_arm", package_name="kuka_moveit_config")
        .robot_description(file_path="config/kuka_kr210_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/kuka_kr210_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()], # to_dict(): loads the MoveIt configuration
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory("kuka_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF from world to base_footprint
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_footprint"],
    )

    # ROS 2 control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(get_package_share_directory("kuka_moveit_config"), "config", "ros2_controllers.yaml")],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    kuka_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kuka_arm_controller", "-c", "/controller_manager"],
    )
    kuka_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kuka_hand_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            move_group_node,
            rviz_node,
            static_tf_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            kuka_arm_controller_spawner,
            kuka_hand_controller_spawner,
        ]
    )

    # moveit_config = MoveItConfigsBuilder("kuka_kr210_arm", package_name="kuka_moveit_config").to_moveit_configs()
    # return generate_demo_launch(moveit_config)