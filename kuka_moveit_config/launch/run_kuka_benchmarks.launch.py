# From moveit2_tutorials/doc/how_to_guides/benchmarking/launch/run_benchmarks.launch.py

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

# Utility function to load a YAML file
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    # Parent of IOError, OSError *and* WindowsError where available
    except EnvironmentError:
        return None

def generate_launch_description():

    # Load benchmark configuration from kuka_moveit_config package
    moveit_ros_benchmarks_config = (
        ParameterBuilder("kuka_moveit_config")
        .yaml(
            parameter_namespace="benchmark_config",
            file_path="config/ompl_benchmark.yaml",
        )
        .to_dict()
    )

    # Load robot description and planning configurations
    moveit_configs = (
        MoveItConfigsBuilder("kuka_moveit_config")
        .robot_description(file_path="config/kuka_kr210_arm.urdf.xacro") 
        .robot_description_semantic(file_path="config/kuka_kr210_arm.srdf")
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            os.path.join(
                get_package_share_directory("kuka_moveit_config"),
                "config",
                "benchmarking_moveit_cpp.yaml",
            )
        )
        .to_moveit_configs()
    )

    # Load additional OMPL pipeline configurations
    ompl_planning_pipeline_config = {
        "ompl_rrtc": {
            "planning_plugins": [
                "ompl_interface/OMPLPlanner",
            ],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    }
    
    # Load OMPL planners from kuka_moveit_config package
    ompl_planning_yaml = load_yaml(
        "kuka_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl_rrtc"].update(ompl_planning_yaml)

    # Configure the warehouse for storing benchmarks
    # sqlite_database = (
    #     get_package_share_directory("kuka_moveit_config")
    #     + "/databases/kuka_test_db.sqlite"  # Update the database path if needed
    # )

    # warehouse_ros_config = {
    #     "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #     "benchmark_config": {
    #         "warehouse": {
    #             "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #             "host": sqlite_database,
    #             "port": 33828,
    #             "scene_name": "",  # Use all scenes by default
    #             "queries_regex": ".*",
    #         },
    #     },
    # }

    moveit_ros_benchmarks_node = Node(
        name="moveit_run_benchmark",
        package="moveit_ros_benchmarks",
        executable="moveit_run_benchmark",
        output="screen",
        parameters=[
            moveit_ros_benchmarks_config,
            moveit_configs.to_dict(),
            # warehouse_ros_config, 
            ompl_planning_pipeline_config,
        ],
    )

    return LaunchDescription([moveit_ros_benchmarks_node])
