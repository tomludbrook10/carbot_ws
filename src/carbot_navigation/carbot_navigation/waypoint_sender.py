
#!/usr/bin/env python3
"""
Waypoint sender for Nav2 - Reads waypoints from YAML and sends them to Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, String, Bool
from action_msgs.msg import GoalStatus
import yaml
import math
import os

class WaypointSender(Node):
    def __init__(self):
        super().__init__('navigation_waypoint_sender')
        
        # Parameters
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('use_follow_waypoints', True)  # Use FollowWaypoints vs NavigateToPose
        self.declare_parameter('auto_start', False)
        self.declare_parameter('waypoint_reached_tolerance', 0.3)  # meters
        
        # Get parameters
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.loop_waypoints = self.get_parameter('loop_waypoints').value
        self.use_follow_waypoints = self.get_parameter('use_follow_waypoints').value
        self.auto_start = self.get_parameter('auto_start').value
        self.tolerance = self.get_parameter('waypoint_reached_tolerance').value
        
        # Action clients
        if self.use_follow_waypoints:
            self._action_client = ActionClient(
                self,
                FollowWaypoints,
                'follow_waypoints'
            )
            self.get_logger().info('Using FollowWaypoints action')
        else:
            self._action_client = ActionClient(
                self,
                NavigateToPose,
                'navigate_to_pose'
            )
            self.get_logger().info('Using NavigateToPose action')
        
        # Publishers for status
        self.status_pub = self.create_publisher(String, 'navigation_status', 10)
        self.current_waypoint_pub = self.create_publisher(PoseStamped, 'current_goal_waypoint', 10)
        

        ## maybe change these to services.

        # Subscribers for control
        self.start_sub = self.create_subscription(
            Bool, 
            'start_navigation', 
            self.start_callback, 
            10
        )
        self.stop_sub = self.create_subscription(
            Bool, 
            'stop_navigation', 
            self.stop_callback, 
            10
        )
        
        # State
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.navigating = False
        self.goal_handle = None
        
        # Load waypoints
        if self.waypoint_file:
            self.load_waypoints()
        
        # Auto start if configured
        if self.auto_start and self.waypoints:
            self.start_timer = self.create_timer(5.0, self.auto_start_callback)
    
    def load_waypoints(self):
        """Load waypoints from YAML file"""
        if not os.path.exists(self.waypoint_file):
            self.get_logger().error(f'Waypoint file not found: {self.waypoint_file}')
            return
        
        try:
            with open(self.waypoint_file, 'r') as f:
                data = yaml.safe_load(f)
            
            if 'waypoints' in data:
                for wp in data['waypoints']:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = float(wp['position']['x'])
                    pose.pose.position.y = float(wp['position']['y'])
                    pose.pose.position.z = float(wp['position'].get('z', 0.0))
                    
                    # Handle orientation
                    if 'orientation' in wp:
                        pose.pose.orientation.x = float(wp['orientation'].get('x', 0.0))
                        pose.pose.orientation.y = float(wp['orientation'].get('y', 0.0))
                        pose.pose.orientation.z = float(wp['orientation'].get('z', 0.0))
                        pose.pose.orientation.w = float(wp['orientation'].get('w', 1.0))
                    else:
                        # Default orientation (facing forward)
                        pose.pose.orientation.w = 1.0
                    
                    self.waypoints.append(pose)
                
                self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
                self.publish_status(f'Loaded {len(self.waypoints)} waypoints')
            else:
                self.get_logger().error('No waypoints found in file')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
    
    def auto_start_callback(self):
        """Auto start navigation after delay"""
        self.start_timer.cancel()
        self.get_logger().info('Auto-starting navigation...')
        self.start_navigation()
    
    def start_callback(self, msg):
        """Handle start navigation command"""
        if msg.data and not self.navigating:
            self.start_navigation()
    
    def stop_callback(self, msg):
        """Handle stop navigation command"""
        if msg.data and self.navigating:
            self.stop_navigation()
    
    def start_navigation(self):
        """Start navigating through waypoints"""
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded!')
            self.publish_status('Error: No waypoints loaded')
            return
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available!')
            self.publish_status('Error: Nav2 not ready')
            return
        
        self.navigating = True
        self.current_waypoint_idx = 0
        
        if self.use_follow_waypoints:
            self.send_all_waypoints()
        else:
            self.send_next_waypoint()
    
    def send_all_waypoints(self):
        """Send all waypoints at once using FollowWaypoints"""
        self.get_logger().info('Sending all waypoints to Nav2...')
        self.publish_status('Starting waypoint following')
        
        goal_msg = FollowWaypoints.Goal()
        
        # Set timestamp for all waypoints
        for wp in self.waypoints:
            wp.header.stamp = self.get_clock().now().to_msg()
            goal_msg.poses.append(wp)
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def send_next_waypoint(self):
        """Send single waypoint using NavigateToPose"""
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.loop_waypoints:
                self.current_waypoint_idx = 0
                self.get_logger().info('Looping back to first waypoint')
            else:
                self.get_logger().info('All waypoints completed!')
                self.publish_status('Navigation complete')
                self.navigating = False
                return
        
        waypoint = self.waypoints[self.current_waypoint_idx]
        waypoint.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(
            f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}: '
            f'({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})'
        )
        
        self.publish_status(f'Going to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}')
        self.current_waypoint_pub.publish(waypoint)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        goal_msg.behavior_tree = ''  # Use default behavior tree
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.publish_status('Error: Goal rejected')
            self.navigating = False
            return
        
        self.get_logger().info('Goal accepted')
        
        # Get result
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            
            if not self.use_follow_waypoints:
                # For single waypoint navigation, move to next
                self.current_waypoint_idx += 1
                if self.navigating:  # Check if not stopped
                    self.send_next_waypoint()
            else:
                # For follow waypoints, check if looping
                if self.loop_waypoints and self.navigating:
                    self.get_logger().info('Restarting waypoint loop...')
                    self.send_all_waypoints()
                else:
                    self.publish_status('All waypoints completed')
                    self.navigating = False
                    
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation aborted!')
            self.publish_status('Navigation aborted')
            self.navigating = False
            
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation canceled')
            self.publish_status('Navigation canceled')
            self.navigating = False
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        
        if self.use_follow_waypoints:
            # FollowWaypoints feedback
            if hasattr(feedback, 'current_waypoint'):
                waypoint_idx = feedback.current_waypoint
                if waypoint_idx < len(self.waypoints):
                    self.current_waypoint_pub.publish(self.waypoints[waypoint_idx])
                    self.publish_status(f'Waypoint {waypoint_idx + 1}/{len(self.waypoints)}')
        else:
            # NavigateToPose feedback
            if hasattr(feedback, 'distance_remaining'):
                dist = feedback.distance_remaining
                self.get_logger().debug(f'Distance remaining: {dist:.2f}m')
                
                if dist < self.tolerance:
                    self.get_logger().info(f'Waypoint {self.current_waypoint_idx + 1} reached')
    
    def stop_navigation(self):
        """Stop navigation"""
        self.navigating = False
        
        if self.goal_handle is not None:
            self.get_logger().info('Canceling navigation...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        
        self.publish_status('Navigation stopped')
    
    def cancel_done_callback(self, future):
        """Handle cancel response"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().warn('Goal cancel failed')
    
    def publish_status(self, status):
        """Publish navigation status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down waypoint sender...')
        if node.navigating:
            node.stop_navigation()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()