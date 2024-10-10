
import sys
import os

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs

import numpy as np
import open3d as o3d

class PCLPublisher(Node):

    # Constructor
    def __init__(self):
        super().__init__('pcl_publisher_node') # Initialize node with name

        # Executable requires file path as command-line argument
        # Check that file path is provided
        assert len(sys.argv) > 1, "No ply file given."
        # Check that provided file path exists
        assert os.path.exists(sys.argv[1]), "File doesn't exist."
        # Store file path in variable
        pcl_path = sys.argv[1]

        # Open3D to read point cloud file
        pcl = o3d.io.read_point_cloud(pcl_path)
        # Convert point cloud to NumPy array
        self.points = np.asarray(pcl.points)
        # Print to console
        print(self.points.shape)

        # Translate point cloud to move out of robot body
        # translation = np.array([0.5, 1, 0])
        # self.points = self.points + translation
        
        # Create publisher that publishes PointCloud2 messages to topic 'pcl'
        self.pcl_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'pcl', 10)
        # Set timer period to 30 times per second
        timer_period = 1/30.0
        # Timer that calls self.timer_callback at interval = timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Rotation matrix, rotates the point cloud on each timer callback. 
        # self.R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi/48])

              
    # Define callback function to be called by timer            
    def timer_callback(self):
        # Rotates point cloud by multiplying with rotation matrix
        # self.points = self.points @ self.R
        # Convert numpy array into PointCloud2 message
        self.pcl = point_cloud(self.points, 'world') # Default (fixed) frame in RViz: 'map'
        # Publish the PointCloud2 object to 'pcl' topic
        self.pcl_publisher.publish(self.pcl)

# Define function to convert NumPy array to PointCloud2 message
def point_cloud(points, parent_frame):
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcl_publisher = PCLPublisher()
    rclpy.spin(pcl_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcl_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
