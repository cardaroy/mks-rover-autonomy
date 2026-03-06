import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2
import numpy as np
from std_msgs.msg import Header

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')

        # Point Cloud Subscriber
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',  # Topic name for RealSense point cloud
            self.pointcloud_callback,
            10
        )

        # IMU Subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/camera/imu',  # Topic name for RealSense IMU
            self.imu_callback,
            10
        )

        # Publisher to publish the accumulated point cloud
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/accumulated_pointcloud', 10)

        # Initialize an empty list to accumulate the point cloud
        self.accumulated_pointcloud = []

    def pointcloud_callback(self, msg: PointCloud2):
        self.get_logger().info('Received PointCloud2 data')

        # Convert ROS PointCloud2 to a list of points
        pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc_array = np.array(list(pc_data))

        # Accumulate point cloud data (add new points)
        self.accumulated_pointcloud.append(pc_array)

        # Optionally: Save to a file or process further
        # Concatenate all accumulated point clouds into a single array
        accumulated_points = np.vstack(self.accumulated_pointcloud)

        # Publish the accumulated point cloud to the new topic
        self.publish_pointcloud(accumulated_points)

        # Log the current accumulated point cloud size
        self.get_logger().info(f"Accumulated point cloud size: {accumulated_points.shape[0]} points.")

    def publish_pointcloud(self, points):
        # Create the header for the PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"  # Set this to the frame where your data is located (e.g., "base_link", "camera_link", etc.)

        # Convert the accumulated points to PointCloud2 format
        point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points)

        # Publish the message
        self.pointcloud_pub.publish(point_cloud_msg)

    def imu_callback(self, msg: Imu):
        self.get_logger().info('Received IMU data')
        # You can process the IMU data here if needed
        # For example, logging orientation or acceleration
        orientation = msg.orientation
        self.get_logger().info(f"IMU Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
