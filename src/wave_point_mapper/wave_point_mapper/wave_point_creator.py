"""
Interactive tool to create and save waypoints on a ROS2 map
Click on the map in RViz to create waypoints, visualize paths, and save them
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import yaml
import math
from typing import List, Tuple
import os

class WaypointCreator(Node):
    def __init__(self):
        super().__init__('waypoint_creator')
        
        # Parameters
        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('create_mid_section_path', True)
        self.declare_parameter('path_height', 0.1)  # Height above ground for visualization
        
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoints = []
        self.current_path_segment = []
        self.paths = {}  # Dictionary to store multiple paths
        self.current_path_name = "default_path"
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        self.current_segment_pub = self.create_publisher(Path, '/current_segment', 10)
        
        # Subscribers - Listen to clicked points from RViz
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Published by RViz "2D Nav Goal" tool
            self.goal_callback,
            10
        )
        
        # Alternative: Subscribe to clicked points
        # self.clicked_point_sub = self.create_subscription(
        #     PoseStamped,
        #     '/clicked_point',  # Published by RViz "Publish Point" tool
        #     self.clicked_point_callback,
        #     10
        # )
        
        # Timer for publishing visualization
       # self.timer = self.create_timer(1.0, self.publish_visualization)
        
        self.get_logger().info('Waypoint Creator initialized!')
        self.get_logger().info('Commands:')
        self.get_logger().info('  - Use "2D Nav Goal" tool in RViz to add waypoints')
        self.get_logger().info('  - Call service to save waypoints: ros2 service call /save_waypoints')
        self.get_logger().info('  - Call service to clear waypoints: ros2 service call /clear_waypoints')
        self.get_logger().info('  - Call service to create mid-section path: ros2 service call /create_mid_path')
        
        # Services
        self.create_service_interfaces()
        
        # Load existing waypoints if file exists
        self.load_waypoints()
    
    def create_service_interfaces(self):
        """Create services for waypoint management"""
        from std_srvs.srv import Empty, Trigger
        from example_interfaces.srv import SetBool
        
        self.save_service = self.create_service(
            Empty, 
            'save_waypoints', 
            self.save_waypoints_callback
        )
        
        self.clear_service = self.create_service(
            Empty,
            'clear_waypoints',
            self.clear_waypoints_callback
        )
        
        self.create_mid_path_service = self.create_service(
            Empty,
            'create_mid_path',
            self.create_mid_section_path_callback
        )
        
        self.new_path_service = self.create_service(
            Trigger,
            'start_new_path',
            self.start_new_path_callback
        )
    
    def goal_callback(self, msg: PoseStamped):
        """Callback for when a goal is set in RViz using 2D Nav Goal tool"""
        self.add_waypoint(msg)
    
    def clicked_point_callback(self, msg: PoseStamped):
        """Callback for clicked points in RViz"""
        # Convert clicked point to pose
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = msg.pose.position
        pose.pose.orientation.w = 1.0  # Default orientation
        self.add_waypoint(pose)
    
    def add_waypoint(self, pose: PoseStamped):
        """Add a waypoint to the current path"""
        self.waypoints.append(pose)
        self.current_path_segment.append(pose)
        
        # Calculate distance from last waypoint if exists
        if len(self.waypoints) > 1:
            last_wp = self.waypoints[-2]
            distance = self.calculate_distance(
                last_wp.pose.position,
                pose.pose.position
            )
            self.get_logger().info(
                f'Added waypoint #{len(self.waypoints)} at '
                f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}), '
                f'distance from last: {distance:.2f}m'
            )
        else:
            self.get_logger().info(
                f'Added first waypoint at '
                f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
            )
        
        self.publish_visualization()
    
    def calculate_distance(self, p1: Point, p2: Point) -> float:
        """Calculate Euclidean distance between two points"""
        return math.sqrt(
            (p2.x - p1.x)**2 + 
            (p2.y - p1.y)**2
        )
    
    def create_mid_section_path_callback(self, request, response):
        """Create a path around the mid-section of the map automatically"""
        self.get_logger().info('Creating mid-section path...')
        
        # This would require the map data - for now, create a rectangular path
        # In practice, you'd analyze the map occupancy grid
        
        # Example: Create a rectangular path in the middle of a typical room
        mid_section_waypoints = [
            self.create_pose(2.0, 1.0, 0.0),      # Front right
            self.create_pose(2.0, -1.0, -1.57),   # Back right
            self.create_pose(-2.0, -1.0, 3.14),   # Back left
            self.create_pose(-2.0, 1.0, 1.57),    # Front left
            self.create_pose(2.0, 1.0, 0.0),      # Close loop
        ]
        
        self.paths['mid_section'] = mid_section_waypoints
        self.waypoints.extend(mid_section_waypoints)
        
        self.get_logger().info(f'Created mid-section path with {len(mid_section_waypoints)} waypoints')
        self.publish_visualization()
        
        return response
    
    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Helper to create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        
        return pose
    
    def publish_visualization(self):
        """Publish path and waypoint markers for visualization in RViz"""
        # Publish path
        if self.waypoints:
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = self.waypoints
            self.path_pub.publish(path)
        
        # Publish waypoint markers
        marker_array = MarkerArray()
        
        # Clear old markers
        clear_marker = Marker()
        clear_marker.header.frame_id = 'map'
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # Add waypoint markers
        for i, waypoint in enumerate(self.waypoints):
            # Sphere marker for waypoint
            marker = Marker()
            marker.header = waypoint.header
            marker.header.frame_id = 'map'
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = waypoint.pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Color code: Start=Green, End=Red, Others=Blue
            if i == 0:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif i == len(self.waypoints) - 1:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            
            marker_array.markers.append(marker)
            
            # Text marker for waypoint number
            text_marker = Marker()
            text_marker.header = waypoint.header
            text_marker.header.frame_id = 'map'
            text_marker.ns = 'waypoint_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD


            # note that this will reference equal, rather than a copy if you don't unpack.
            text_marker.pose.position.x = waypoint.pose.position.x
            text_marker.pose.position.y = waypoint.pose.position.y
            text_marker.pose.position.z = waypoint.pose.position.y
            text_marker.pose.position.z += 0.3  # Raise text above sphere
            text_marker.text = f"WP{i+1}"
            text_marker.scale.z = 0.3
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            
            marker_array.markers.append(text_marker)
        
        # Add connecting lines between waypoints
        if len(self.waypoints) > 1:
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'path_lines'
            line_marker.id = 2000
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # Line width
            line_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            
            for waypoint in self.waypoints:
                point = Point()
                point.x = waypoint.pose.position.x
                point.y = waypoint.pose.position.y
                point.z = 0.1  # Slightly above ground
                line_marker.points.append(point)
            
            marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)
    
    def save_waypoints_callback(self, request, response):
        """Save waypoints to YAML file"""
        self.save_waypoints()
        return response
    
    def save_waypoints(self):
        """Save waypoints to YAML file"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints to save!')
            return
        
        data = {
            'waypoints': [],
            'paths': {}
        }
        
        # Save individual waypoints
        for i, wp in enumerate(self.waypoints):
            waypoint_data = {
                'id': i,
                'position': {
                    'x': float(wp.pose.position.x),
                    'y': float(wp.pose.position.y),
                    'z': float(wp.pose.position.z)
                },
                'orientation': {
                    'x': float(wp.pose.orientation.x),
                    'y': float(wp.pose.orientation.y),
                    'z': float(wp.pose.orientation.z),
                    'w': float(wp.pose.orientation.w)
                }
            }
            data['waypoints'].append(waypoint_data)
        
        # Save named paths
        for path_name, path_waypoints in self.paths.items():
            data['paths'][path_name] = []
            for wp in path_waypoints:
                data['paths'][path_name].append({
                    'x': float(wp.pose.position.x),
                    'y': float(wp.pose.position.y)
                })
        
        # Write to file
        with open(self.waypoint_file, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        
        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {self.waypoint_file}')
    
    def load_waypoints(self):
        """Load waypoints from YAML file if it exists"""
        if os.path.exists(self.waypoint_file):
            try:
                with open(self.waypoint_file, 'r') as f:
                    data = yaml.safe_load(f)
                
                if data and 'waypoints' in data:
                    for wp_data in data['waypoints']:
                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.pose.position.x = wp_data['position']['x']
                        pose.pose.position.y = wp_data['position']['y']
                        pose.pose.position.z = wp_data['position']['z']
                        pose.pose.orientation.x = wp_data['orientation']['x']
                        pose.pose.orientation.y = wp_data['orientation']['y']
                        pose.pose.orientation.z = wp_data['orientation']['z']
                        pose.pose.orientation.w = wp_data['orientation']['w']
                        self.waypoints.append(pose)
                    
                    self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {self.waypoint_file}')
                    self.publish_visualization()
                    
            except Exception as e:
                self.get_logger().error(f'Failed to load waypoints: {e}')
    
    def clear_waypoints_callback(self, request, response):
        """Clear all waypoints"""
        self.waypoints = []
        self.current_path_segment = []
        self.get_logger().info('Cleared all waypoints')
        self.publish_visualization()
        return response
    
    def start_new_path_callback(self, request, response):
        """Start a new named path segment"""
        import datetime
        path_name = f"path_{datetime.datetime.now().strftime('%H%M%S')}"
        
        if self.current_path_segment:
            self.paths[self.current_path_name] = self.current_path_segment.copy()
        
        self.current_path_segment = []
        self.current_path_name = path_name
        
        response.success = True
        response.message = f"Started new path: {path_name}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCreator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down, saving waypoints...')
        node.save_waypoints()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()