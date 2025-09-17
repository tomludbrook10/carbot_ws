#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import math

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Create publisher for initialpose topic
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',  # AMCL subscribes to this topic
            10
        )
        
        # Timer to publish initial pose (publish once after a short delay)
        self.timer = self.create_timer(2.0, self.publish_initial_pose)
        self.published = False
        
        self.get_logger().info('Initial Pose Publisher node started')
    
    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        qw = math.cos(yaw / 2.0)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def publish_initial_pose(self):
        if self.published:
            return
        
        # Create the PoseWithCovarianceStamped message
        initial_pose_msg = PoseWithCovarianceStamped()
        
        # Set the header
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'  # Reference frame
        
        # Set the pose at origin (0, 0, 0)
        x = 0.0      # x position at origin
        y = 0.0      # y position at origin
        yaw = 0.0    # yaw at 0 radians (facing forward)
        
        initial_pose_msg.pose.pose.position = Point(x=x, y=y, z=0.0)
        initial_pose_msg.pose.pose.orientation = self.euler_to_quaternion(yaw)
        
        # Set covariance matrix (6x6 = 36 elements)
        # The covariance matrix represents uncertainty in [x, y, z, roll, pitch, yaw]
        # High covariance means the robot could be ~1 meter away from (0,0,0) with decent probability
        covariance = [0.0] * 36
        
        # Set diagonal elements (variances)
        # For ~1 meter standard deviation, variance = std_dev^2
        # Position uncertainties (x, y, z) - indices 0, 7, 14
        covariance[0] = 1.0    # x variance (1.0m standard deviation)
        covariance[7] = 1.0    # y variance (1.0m standard deviation) 
        covariance[14] = 0.0   # z variance (assuming 2D)
        
        # Orientation uncertainties (roll, pitch, yaw) - indices 21, 28, 35
        covariance[21] = 0.0   # roll variance (assuming 2D)
        covariance[28] = 0.0   # pitch variance (assuming 2D)
        covariance[35] = 0.5   # yaw variance (~40 degrees standard deviation)
        
        initial_pose_msg.pose.covariance = covariance
        
        # Publish the message
        self.publisher.publish(initial_pose_msg)
        self.published = True
        
        self.get_logger().info(f'Published initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad')
        self.get_logger().info('Initial pose at origin with ~1m covariance published successfully!')
        
        # Destroy timer since we only need to publish once
        self.timer.destroy()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = InitialPosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()