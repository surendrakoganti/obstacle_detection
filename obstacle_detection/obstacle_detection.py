"""Obstacle Detection Node for ROS 2 using LaserScan and DBSCAN clustering."""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN

from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
try:
    from tf2_ros import TransformException
except ImportError:
    TransformException = Exception

# custom_msgs must be built and sourced before usage
from custom_msgs.msg import ObstacleDetectionData, ObstacleDetectionArray

# CONSTANTS
MIN_RANGE = 0.5
MAX_RANGE= 2.5

class ObstacleDetection(Node):
    """ROS 2 Node for detecting obstacles from LiDAR scans using DBSCAN clustering."""

    def __init__(self):
        """Initialize the ObstacleDetection node, subscribers, and publishers."""
        super().__init__('obstacle_detection')

        # QoS for LiDAR scan subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
            )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback_laser,
            qos_profile
            )

        self.publisher_filtered_scan = self.create_publisher(LaserScan, 'scan_filtered', 10)
        self.publisher_obstacles = self.create_publisher(
            ObstacleDetectionArray,
            'obstacles',
            10
            )

        self.eps = 0.2
        self.min_samples = 1

        # TF2 listener for transforming points
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS 2 parameters
        self.declare_parameter('eps', self.eps)
        self.declare_parameter('min_samples', self.min_samples)

        self.get_logger().info('✅ Obstacle Detection node started and ready!')

    def callback_laser(self, data):
        """Process incoming LaserScan data, cluster obstacles, and publish results."""
        filtered_scan = self.filter_scan(data)
        self.publisher_filtered_scan.publish(filtered_scan)

        points_xy = self.extract_valid_points(filtered_scan)
        if len(points_xy) == 0:
            return

        clusters = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points_xy).labels_
        self.process_clusters(points_xy, clusters)

    def filter_scan(self, data):
        """Filter LaserScan ranges to the desired min/max range."""
        filtered_scan = LaserScan()
        filtered_scan.header = data.header
        filtered_scan.angle_min = data.angle_min
        filtered_scan.angle_max = data.angle_max
        filtered_scan.angle_increment = data.angle_increment
        filtered_scan.time_increment = data.time_increment
        filtered_scan.scan_time = data.scan_time
        filtered_scan.range_min = MIN_RANGE
        filtered_scan.range_max = MAX_RANGE

        filtered_ranges = [
            r if MIN_RANGE <= r <= MAX_RANGE else float('inf')
            for r in data.ranges
        ]

        self.eps = self.get_parameter('eps').value
        self.min_samples = self.get_parameter('min_samples').value

        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = data.intensities
        return filtered_scan

    def extract_valid_points(self, scan):
        """Return valid (finite) scan points as numpy array."""
        ranges = np.array(scan.ranges)
        valid_indices = np.where(np.isfinite(ranges))[0]

        if len(valid_indices) == 0:
            return np.array([])

        angles = scan.angle_min + valid_indices * scan.angle_increment
        xs = ranges[valid_indices] * np.cos(angles)
        ys = ranges[valid_indices] * np.sin(angles)
        points_xy = np.vstack((xs, ys)).T
        return points_xy

    def process_clusters(self, points_xy, labels):
        """Transform cluster points, calculate centroids, and publish obstacles."""
        unique_clusters = set(labels)
        obstacle_array = ObstacleDetectionArray()
        obstacle_id = 0

        for cid in unique_clusters:
            cluster_points = points_xy[labels == cid]

            points_transformed_array = self.transform_points(cluster_points)
            if not points_transformed_array:
                continue

            points_np = np.array(points_transformed_array).reshape(-1, 3)
            centroid = points_np[:, :2].mean(axis=0)
            distance_to_object = np.linalg.norm(centroid)

            left_point = points_np[np.argmax(points_np[:, 1])]
            right_point = points_np[np.argmin(points_np[:, 1])]

            obstacle_id += 1
            obstacle = ObstacleDetectionData()
            obstacle.obstacle_id = obstacle_id
            obstacle.l_x = left_point[0]
            obstacle.l_y = left_point[1]
            obstacle.r_x = right_point[0]
            obstacle.r_y = right_point[1]
            obstacle.distance = distance_to_object

            obstacle_array.obstacles.append(obstacle)

            self.get_logger().info(
                f"Cluster {cid} | "
                f"Left-most: ({left_point[0]:.2f}, {left_point[1]:.2f}) "
                f"Right-most: ({right_point[0]:.2f}, {right_point[1]:.2f}) "
                f"Distance: {distance_to_object:.2f}"
            )

        self.get_logger().info('--------------------------------')
        self.publisher_obstacles.publish(obstacle_array)

    def transform_points(self, cluster_points):
        """Transform cluster points from laser frame to base link frame."""
        points_transformed = []

        for point in cluster_points:
            point_in_source = PointStamped()
            point_in_source.header.stamp = self.get_clock().now().to_msg()
            point_in_source.header.frame_id = 'laser_frame'
            point_in_source.point.x = point[0]
            point_in_source.point.y = point[1]
            point_in_source.point.z = 0.3

            try:
                transform = self.tf_buffer.lookup_transform(
                    '4/base_link',
                    'laser_frame',
                    rclpy.time.Time()
                )
                p_transformed = do_transform_point(point_in_source, transform)
                points_transformed.append([p_transformed.point.x,
                                           p_transformed.point.y,
                                           p_transformed.point.z])
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform point: {ex}')

        return points_transformed


def main(args=None):
    """Main function to run the ObstacleDetection node."""
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetection()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
