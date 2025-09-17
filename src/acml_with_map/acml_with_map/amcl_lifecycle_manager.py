#!/usr/bin/env python3
"""
AMCL Lifecycle Manager Node for ROS2 Jazzy
Properly configures and activates AMCL with initial pose setting
FIXED: Uses async approach to avoid nested spinning issues
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import time
import sys
import math
from enum import Enum
from rclpy.executors import MultiThreadedExecutor

class LifecycleState(Enum):
    WAITING_FOR_MAP = "waiting_for_map"
    CHECKING_STATE = "checking_state"
    CONFIGURING = "configuring"
    ACTIVATING = "activating"
    SETTING_POSE = "setting_pose"
    READY = "ready"
    ERROR = "error"

class AMCLLifecycleManager(Node):
    def __init__(self):
        super().__init__('amcl_lifecycle_manager')
        
        # Parameters
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.declare_parameter('initial_cov_xx', 0.25)
        self.declare_parameter('initial_cov_yy', 0.25)
        self.declare_parameter('initial_cov_aa', 0.068)
        self.declare_parameter('wait_for_map', True)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('set_initial_pose', True)
        self.declare_parameter('amcl_namespace', '')  # Empty for no namespace
        
        # Get parameters
        self.initial_x = self.get_parameter('initial_pose_x').value
        self.initial_y = self.get_parameter('initial_pose_y').value
        self.initial_yaw = self.get_parameter('initial_pose_yaw').value
        self.initial_cov_xx = self.get_parameter('initial_cov_xx').value
        self.initial_cov_yy = self.get_parameter('initial_cov_yy').value
        self.initial_cov_aa = self.get_parameter('initial_cov_aa').value
        self.wait_for_map = self.get_parameter('wait_for_map').value
        self.auto_start = self.get_parameter('auto_start').value
        self.set_initial_pose = self.get_parameter('set_initial_pose').value
        self.amcl_namespace = self.get_parameter('amcl_namespace').value
        
        # Build service names based on namespace
        amcl_prefix = 'amcl'
        
        # Service clients for lifecycle management
        self.get_state_client = self.create_client(
            GetState, 
            f'/{amcl_prefix}/get_state'
        )
        self.change_state_client = self.create_client(
            ChangeState, 
            f'/{amcl_prefix}/change_state'
        )
        
        # Modern QoS Profile for initial pose publisher
        initial_pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            initial_pose_qos
        )
        
        # Subscribe to map to know when it's ready
        self.map_received = False
        if self.wait_for_map:
            # Modern QoS Profile for map subscription (map is typically transient local)
            map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            self.map_sub = self.create_subscription(
                OccupancyGrid,
                '/map',
                self.map_callback,
                map_qos
            )
        else:
            self.map_received = True
        
        # State tracking
        self.lifecycle_state = LifecycleState.WAITING_FOR_MAP
        self.amcl_ready = False
        self.initial_pose_sent = False
        self.pending_future = None
        self.retry_count = 0
        self.max_retries = 5
        
        # Start the configuration process
        if self.auto_start:
            self.get_logger().info('Starting AMCL lifecycle management...')
            self.get_logger().info(f'AMCL service prefix: {amcl_prefix}')
            self.timer = self.create_timer(1.0, self.manage_lifecycle_callback)
        else:
            self.get_logger().info('AMCL lifecycle manager ready. Waiting for manual start...')
    
    def map_callback(self, msg: OccupancyGrid):
        """Callback when map is received"""
        if not self.map_received:
            self.get_logger().info(f'Map received! Size: {msg.info.width}x{msg.info.height}, '
                                  f'Resolution: {msg.info.resolution}m/pixel')
            self.map_received = True
    
    def manage_lifecycle_callback(self):
        """Main lifecycle management callback - non-blocking"""
        # Handle pending service responses first
        if self.pending_future is not None:
            self.get_logger().info("fure is not null")
            if self.pending_future.done():
                self.handle_service_response()
            else:
                # Still waiting for service response
                return
        
        # State machine for lifecycle management
        if self.lifecycle_state == LifecycleState.WAITING_FOR_MAP:
            self.handle_waiting_for_map()
        elif self.lifecycle_state == LifecycleState.CHECKING_STATE:
            self.handle_checking_state()
        elif self.lifecycle_state == LifecycleState.CONFIGURING:
            # Waiting for configure response - handled above
            self.get_logger().info("Waiting for configuring respones")
        elif self.lifecycle_state == LifecycleState.ACTIVATING:
            # Waiting for activate response - handled above
            self.get_logger().info("Waiting for activating respones")
        elif self.lifecycle_state == LifecycleState.SETTING_POSE:
            self.handle_setting_pose()
        elif self.lifecycle_state == LifecycleState.READY:
            self.get_logger().info('✓ AMCL lifecycle management complete!')
            self.timer.cancel()
        elif self.lifecycle_state == LifecycleState.ERROR:
            self.get_logger().error('AMCL lifecycle management failed')
            self.timer.cancel()
    
    def handle_waiting_for_map(self):
        """Handle waiting for map state"""
        if self.wait_for_map and not self.map_received:
            self.get_logger().info('Waiting for map to be published on /map topic...')
            return
        
        self.lifecycle_state = LifecycleState.CHECKING_STATE
        self.get_logger().info('Map ready, checking AMCL state...')
    
    def handle_checking_state(self):
        """Handle checking AMCL state"""
        if not self.get_state_client.service_is_ready():
            self.get_logger().warn('AMCL get_state service not ready, waiting...')
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.get_logger().error('AMCL service not available after maximum retries')
                self.lifecycle_state = LifecycleState.ERROR
            return
        
        # Reset retry count on successful service check
        self.retry_count = 0
        
        # Make async service call
        request = GetState.Request()
        self.pending_future = self.get_state_client.call_async(request)
        self.current_operation = 'get_state'
    
    def handle_setting_pose(self):
        """Handle setting initial pose"""
        if self.set_initial_pose and not self.initial_pose_sent:
            self.set_initial_pose_estimate()
        
        self.lifecycle_state = LifecycleState.READY
        self.amcl_ready = True
    
    def handle_service_response(self):
        """Handle service response based on current operation"""
        try:
            result = self.pending_future.result()
            
            if hasattr(self, 'current_operation'):
                if self.current_operation == 'get_state':
                    self.handle_get_state_response(result)
                elif self.current_operation == 'change_state_configure':
                    self.handle_configure_response(result)
                elif self.current_operation == 'change_state_activate':
                    self.handle_activate_response(result)
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.lifecycle_state = LifecycleState.ERROR
            else:
                # Retry the current state
                pass
        finally:
            self.pending_future = None
            if hasattr(self, 'current_operation'):
                delattr(self, 'current_operation')
    
    def handle_get_state_response(self, result):
        """Handle get state service response"""
        if result is None:
            self.get_logger().error('GetState service call returned None')
            return
        
        state = result.current_state
        self.get_logger().info(f'AMCL current state: {state.label} (id: {state.id})')
        
        # Handle state transitions based on current state
        if state.id == State.PRIMARY_STATE_UNCONFIGURED:
            # Need to configure AMCL
            self.start_configure_amcl()
        
        elif state.id == State.PRIMARY_STATE_INACTIVE:
            # Need to activate AMCL
            self.start_activate_amcl()
        
        elif state.id == State.PRIMARY_STATE_ACTIVE:
            # Already active, set pose if needed
            self.lifecycle_state = LifecycleState.SETTING_POSE
        
        elif state.id == State.PRIMARY_STATE_FINALIZED:
            self.get_logger().error('AMCL is in FINALIZED state. Cannot recover.')
            self.lifecycle_state = LifecycleState.ERROR
        
        else:
            self.get_logger().warn(f'AMCL in unexpected state: {state.label} (id: {state.id})')
    
    def start_configure_amcl(self):
        """Start AMCL configuration"""
        if not self.change_state_client.service_is_ready():
            self.get_logger().warn('AMCL change_state service not ready')
            return
        
        self.get_logger().info('Configuring AMCL...')
        self.lifecycle_state = LifecycleState.CONFIGURING
        
        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        
        self.pending_future = self.change_state_client.call_async(request)
        self.current_operation = 'change_state_configure'
    
    def handle_configure_response(self, result):
        """Handle configure service response"""
        if result is None:
            self.get_logger().error('Configure service call returned None')
            return
        
        if result.success:
            self.get_logger().info('AMCL configured successfully')
            # Go back to checking state to proceed with activation
            self.lifecycle_state = LifecycleState.CHECKING_STATE
        else:
            self.get_logger().error('Failed to configure AMCL')
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.lifecycle_state = LifecycleState.ERROR
            else:
                self.lifecycle_state = LifecycleState.CHECKING_STATE
    
    def start_activate_amcl(self):
        """Start AMCL activation"""
        if not self.change_state_client.service_is_ready():
            self.get_logger().warn('AMCL change_state service not ready')
            return
        
        self.get_logger().info('Activating AMCL...')
        self.lifecycle_state = LifecycleState.ACTIVATING
        
        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        
        self.pending_future = self.change_state_client.call_async(request)
        self.current_operation = 'change_state_activate'
    
    def handle_activate_response(self, result):
        """Handle activate service response"""
        if result is None:
            self.get_logger().error('Activate service call returned None')
            return
        
        if result.success:
            self.get_logger().info('AMCL activated successfully')
            # Move to setting initial pose
            self.lifecycle_state = LifecycleState.SETTING_POSE
        else:
            self.get_logger().error('Failed to activate AMCL')
            self.retry_count += 1
            if self.retry_count > self.max_retries:
                self.lifecycle_state = LifecycleState.ERROR
            else:
                self.lifecycle_state = LifecycleState.CHECKING_STATE
    
    def set_initial_pose_estimate(self):
        """Publish initial pose estimate for AMCL"""
        # Don't send duplicate initial poses
        if self.initial_pose_sent:
            return
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose_msg.pose.pose.position.x = self.initial_x
        pose_msg.pose.pose.position.y = self.initial_y
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)
        
        # Set covariance (6x6 matrix stored as 36 element array)
        # Only x, y, and yaw have non-zero covariance
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[0] = self.initial_cov_xx  # x variance
        pose_msg.pose.covariance[7] = self.initial_cov_yy  # y variance
        pose_msg.pose.covariance[35] = self.initial_cov_aa  # yaw variance
        
        self.initial_pose_pub.publish(pose_msg)
        self.initial_pose_sent = True
        
        self.get_logger().info(
            f'Set initial pose: x={self.initial_x:.2f}, y={self.initial_y:.2f}, '
            f'yaw={self.initial_yaw:.2f} rad ({math.degrees(self.initial_yaw):.1f} deg)'
        )
    
    def cleanup_amcl(self):
        """Cleanup AMCL (deactivate and cleanup) - synchronous version for shutdown"""
        # This is only called during shutdown, so we can use blocking calls
        if not self.get_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Could not connect to AMCL for cleanup')
            return
        
        # Get current state
        request = GetState.Request()
        try:
            future = self.get_state_client.call_async(request)
            # Use a simple polling approach for shutdown
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 2.0:
                time.sleep(0.01)
            
            if future.done():
                result = future.result()
                if result and result.current_state.id == State.PRIMARY_STATE_ACTIVE:
                    # Deactivate
                    self.get_logger().info('Deactivating AMCL...')
                    deactivate_request = ChangeState.Request()
                    deactivate_request.transition = Transition()
                    deactivate_request.transition.id = Transition.TRANSITION_DEACTIVATE
                    self.change_state_client.call_async(deactivate_request)
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {str(e)}')
    
    def destroy_node(self):
        """Override destroy to ensure clean shutdown"""
        self.get_logger().info('Shutting down AMCL lifecycle manager...')
        if hasattr(self, 'timer'):
            self.timer.cancel()
        
        # Clean up pending futures
        if self.pending_future is not None and not self.pending_future.done():
            self.pending_future.cancel()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = AMCLLifecycleManager()
    
    # Create multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Received keyboard interrupt')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()