### Things to keep in mind when running **pcl_publisher_node**:
- needs Open3d: `pip install open3d`
- needs numpy<1.25.0: `pip install "numpy<1.25.0"`
- needs octomap: `sudo apt-get install ros-humble-octomap-server`

### Order:
Two commands under the same number mean either command can be run. Run all commands in seperate terminals. All commands except 2. require `source install/setup.bash` in `ws_pcl`.
1. `ros2 run pcl_publisher pcl_publisher_node ~/ws_pcl/src/pcl_publisher/resource/TrialWall_PC.ply`  
`ros2 run pcl_publisher pcl_publisher_node ~/ws_pcl/src/pcl_publisher/resource/TrialWall_KUKA_PC.ply`
2. `ros2 run octomap_server octomap_server_node --ros-args -p frame_id:=world -r /cloud_in:=/pcl`
3. `ros2 run octomap_publisher octomap_publisher_node`  
`ros2 run octomap_publisher_cpp octomap_publisher_node`
4. `ros2 launch moveit_resources_panda_moveit_config demo.launch.py`  
`ros2 launch kuka_moveit_config demo.launch.py`
5. `ros2 run moveit_ros_benchmarks moveit_run_benchmark --config ~/ws_pcl/src/kuka_moveit_config/config/ompl_benchmark.yaml`  
`ros2 launch kuka_moveit_config run_kuka_benchmarks.launch.py`

### Issues:
| Issue | Fix |
| --- | --- |
| Octomap is not seen as obstacle when motion planning: [Question on Robotics Stack Exchange](https://robotics.stackexchange.com/questions/113032/ros2-moveit-rviz-ply-file-added-to-collision-environment-as-point-cloud-via-oct) | Add (and remove) a random scene object (under "Scene Objects" tab in MotionPlanning window). Motion planning now works, the robot avoids the obstacle. Found [here](https://github.com/moveit/moveit2/issues/444). |
| Benchmarking does not find needed files: [Question on Robotics Stack Exchange](https://robotics.stackexchange.com/questions/113264/benchmarking-with-moveit-ros-benchmarks-cannot-find-configuration-files) | |

### Notes:
**Conversion of STL to PLY:**
1. Using MeshLab: File → Import Mesh → STL file
2. Filters → Sampling → Poisson-disk Sampling → Apply
3. Export as PLY

### Sources:
- [Transformation of PLY file to PointCloud2 message in pcl_publisher package](https://github.com/SebastianGrans/ROS2-Point-Cloud-Demo)
- [URDF file for KUKA KR 210](https://github.com/noshluk2/ROS2-Ultimate-guide-for-Custom-Robotic-Arms-and-Kuka-Kr210/tree/main)