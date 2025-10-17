#!/usr/bin/env python3
"""
Simple Pure Pursuit Path Follower
Pre-computes dense path from waypoints and follows it
No Nav2 stack required - just this node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Int32, Bool
import yaml
import math
import numpy as np

class SimplePurePursuitFollower(Node):
    def __init__(self):
        super().__init__('simple_pure_pursuit_follower')
        
        # Parameters
        self.declare_parameter('waypoint_file', '/home/tom/carbot_ws/waypoints/waypoints.yaml')
        self.declare_parameter('path_resolution', 0.1)  # meters between path points
        self.declare_parameter('lookahead_distance', 0.6)  # meters
        self.declare_parameter('max_linear_velocity', 1.0)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('waypoint_tolerance', 0.1)  # meters
        self.declare_parameter('loop_path', False)
        self.declare_parameter('wheelbase', 0.178)  # for Ackermann
        
        # Get parameters
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.path_resolution = self.get_parameter('path_resolution').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.loop_path = self.get_parameter('loop_path').value
        self.wheelbase = self.get_parameter('wheelbase').value
        
        # State
        self.current_pose = None
        self.waypoints = []
        self.dense_path = []
        self.current_path_index = 0
        self.current_waypoint_index = 0
        self.waypoint_indices = []  # Indices in dense_path where waypoints are
        self.running = False
        
        # Subscribe to AMCL pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

         # Subscribers for control
        self.start_sub = self.create_subscription(
            Bool, 
            'start_navigation', 
            self.start_callback, 
            10
        )

         # Subscribers for control
        self.start_sub = self.create_subscription(
            Bool, 
            'stop_navigation', 
            self.stop_callback, 
            10
        )

        # Subscribers for contro
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/follower_status', 10)
        self.waypoint_pub = self.create_publisher(Int32, '/current_waypoint', 10)
        self.path_marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        
        # Control loop at 20 Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # Load waypoints and generate path
        if self.waypoint_file:
            self.load_waypoints()
            if self.waypoints:
                self.generate_dense_path()
                #self.visualize_path()
                self.publish_status('Path ready - waiting for pose')

    def start_callback(self, msg):
        """Handle start navigation command"""
        if msg.data:
            self.running = True
        self.get_logger().info("Start navigating")
    
    def stop_callback(self, msg):
        """Handle stop navigation command"""
        if msg.data:
            self.running = False
        self.get_logger().info("Stop navigating")
    
    def load_waypoints(self):
        """Load waypoints from YAML file"""
        try:
            with open(self.waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' in data:
                for wp in data['waypoints']:
                    x = float(wp['position']['x'])
                    y = float(wp['position']['y'])
                    self.waypoints.append((x, y))
                
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
    
    def generate_dense_path(self):
        """Generate dense path points between waypoints"""
        if len(self.waypoints) < 2:
            self.get_logger().error('Need at least 2 waypoints')
            return
        
        self.dense_path = []
        self.waypoint_indices = []
        
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]
            
            # Distance between waypoints
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dist = math.sqrt(dx**2 + dy**2)
            
            # Number of intermediate points
            num_points = max(2, int(dist / self.path_resolution))
            
            # Mark where this waypoint is in the dense path
            self.waypoint_indices.append(len(self.dense_path))
            
            # Generate intermediate points
            for j in range(num_points):
                t = j / (num_points - 1)
                x = start[0] + t * dx
                y = start[1] + t * dy
                self.dense_path.append((x, y))
        
        # Add the last waypoint
        self.waypoint_indices.append(len(self.dense_path) - 1)
        
        # Loop back to start if requested
        if self.loop_path and len(self.waypoints) > 2:
            start = self.waypoints[-1]
            end = self.waypoints[0]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dist = math.sqrt(dx**2 + dy**2)
            num_points = max(2, int(dist / self.path_resolution))
            
            for j in range(1, num_points):  # Skip first point (already have it)
                t = j / (num_points - 1)
                x = start[0] + t * dx
                y = start[1] + t * dy
                self.dense_path.append((x, y))
        
        self.get_logger().info(f'Generated dense path with {len(self.dense_path)} points')
    
    def pose_callback(self, msg):
        """Update current pose from AMCL"""
        self.current_pose = msg.pose.pose
        self.get_logger().info("Got pose")
    
    def find_lookahead_point(self):
        """Find the lookahead point on the path"""
        if not self.current_pose or not self.dense_path:
            return None
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Find closest point on path ahead of current index
        min_dist = float('inf')
        closest_idx = self.current_path_index
        
        # Search forward from current index
        for i in range(self.current_path_index, min(self.current_path_index + 50, len(self.dense_path))):
            px, py = self.dense_path[i]
            dist = math.sqrt((px - robot_x)**2 + (py - robot_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Update current index to closest point
        self.current_path_index = closest_idx
        
        # Find lookahead point
        lookahead_idx = self.current_path_index
        accumulated_dist = 0
        
        for i in range(self.current_path_index, len(self.dense_path) - 1):
            px1, py1 = self.dense_path[i]
            px2, py2 = self.dense_path[i + 1]
            segment_dist = math.sqrt((px2 - px1)**2 + (py2 - py1)**2)
            accumulated_dist += segment_dist
            
            if accumulated_dist >= self.lookahead_dist:
                lookahead_idx = i + 1
                break
        
        # Check if we've reached the end
        if lookahead_idx >= len(self.dense_path) - 1:
            if self.loop_path:
                self.current_path_index = 0
                self.current_waypoint_index = 0
                self.publish_status('Looping to start')
            else:
                self.running = False
                self.publish_status('Path complete!')
                return None
        
        # Check if we've passed a waypoint
        for idx, wp_idx in enumerate(self.waypoint_indices):
            if self.current_path_index > wp_idx and idx > self.current_waypoint_index:
                self.current_waypoint_index = idx
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}')
                wp_msg = Int32()
                wp_msg.data = self.current_waypoint_index + 1
                self.waypoint_pub.publish(wp_msg)
        
        return self.dense_path[lookahead_idx]
    
    def pure_pursuit_control(self, lookahead_point):
        """Calculate control commands using pure pursuit"""
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Get robot heading from quaternion
        q = self.current_pose.orientation
        robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        # Vector from robot to lookahead point
        dx = lookahead_point[0] - robot_x
        dy = lookahead_point[1] - robot_y
        
        # Angle to lookahead point
        target_angle = math.atan2(dy, dx)
        
        # Angle error
        angle_error = target_angle - robot_yaw
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Calculate curvature (1/radius)
        dist_to_lookahead = math.sqrt(dx**2 + dy**2)
        curvature = 2.0 * math.sin(angle_error) / dist_to_lookahead
        
        # Calculate velocities
        linear_vel = self.max_linear_vel
        
        # Slow down for sharp turns
        if abs(curvature) > 0.5:
            linear_vel *= (1.0 - 0.7 * min(abs(curvature), 1.0))
        
        # For Ackermann: convert curvature to angular velocity
        angular_vel = linear_vel * curvature
        
        # Limit angular velocity
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        
        return linear_vel, angular_vel
    
    def control_loop(self):
        """Main control loop"""

        if not self.running or not self.current_pose:
            # Stop the robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        self.get_logger().info(f'Look ahead point is ({lookahead_point[0]}, {lookahead_point[1]})')
        
        if lookahead_point is None:
            # Stop the robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Calculate control
        linear_vel, angular_vel = self.pure_pursuit_control(lookahead_point)
        
        # Publish command
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)
        
        # Debug
        self.get_logger().debug(
            f'Path idx: {self.current_path_index}/{len(self.dense_path)}, '
            f'Vel: {linear_vel:.2f}, Ang: {angular_vel:.2f}'
        )
    
    def visualize_path(self):
        """Publish path markers for visualization"""
        marker_array = MarkerArray()
        
        # Path line
        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.7
        
        for x, y in self.dense_path:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.1
            path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # Waypoint markers
        for i, (x, y) in enumerate(self.waypoints):
            wp_marker = Marker()
            wp_marker.header.frame_id = 'map'
            wp_marker.header.stamp = self.get_clock().now().to_msg()
            wp_marker.ns = 'waypoints'
            wp_marker.id = i + 1
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose.position.x = x
            wp_marker.pose.position.y = y
            wp_marker.pose.position.z = 0.2
            wp_marker.scale.x = 0.3
            wp_marker.scale.y = 0.3
            wp_marker.scale.z = 0.3
            wp_marker.color.r = 1.0
            wp_marker.color.g = 0.0
            wp_marker.color.b = 0.0
            wp_marker.color.a = 1.0
            
            marker_array.markers.append(wp_marker)
        
        self.path_marker_pub.publish(marker_array)
    
    def publish_status(self, status):
        """Publish status message"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePurePursuitFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot before shutting down
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()