import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition, State
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import time
import sys
import math
from enum import Enum

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

        amcl_prefix = 'amcl'
        
        self.get_state_client = self.create_client(
            GetState, 
            f'/{amcl_prefix}/get_state')

        self.change_state_client = self.create_client(
            ChangeState, 
            f'/{amcl_prefix}/change_state')


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
    
        self.amcl_ready = False
        self.initial_pose_sent = False
        self.future: Future = None
        self.lifecycle_state = LifecycleState.WAITING_FOR_MAP

        self.get_logger().info("Starting AMCL lifecycle management")
        self.timer = self.create_timer(1.0, self.manage_lifecycle_callback)

    def manage_lifecycle_callback(self):

        if self.lifecycle_state == LifecycleState.WAITING_FOR_MAP:
            self.handle_waiting_for_map()
        elif self.lifecycle_state == LifecycleState.CHECKING_STATE:
            self.handle_checking_state()
        elif self.lifecycle_state == LifecycleState.CONFIGURING:
            pass
        elif self.lifecycle_state == LifecycleState.ACTIVATING:
            pass
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
        

    def map_callback(self, msg: OccupancyGrid):
        """Callback when map is received"""
        if not self.map_received:
            self.get_logger().info(f'Map received! Size: {msg.info.width}x{msg.info.height}, '
                                  f'Resolution: {msg.info.resolution}m/pixel')
            self.map_received = True



    def handle_checking_state(self):
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.get_state_client.srv_name} not available, waiting...')

        request = GetState.Request()
        self.future = self.get_state_client.call_async(request)
        self.future.add_done_callback(self.process_checking_state_response)

    def process_checking_state_response(self, future: Future):
        response = future.result()
        if response is None:
            self.get_logger().error("Get State service return NULL")
            return

        self.future: Future = None
        
        state = response.current_state
        self.get_logger().info(f'AMCL current state: {state.label} (id: {state.id})')
        
        # Handle state transitions based on current state
        if state.id == State.PRIMARY_STATE_UNCONFIGURED:
            # Need to configure AMCL
            self.start_configure_amcl()

        if state.id == State.PRIMARY_STATE_INACTIVE:
            self.start_activate_amcl()

        if state.id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info('Sucessfully activated the amcl node')
            self.lifecycle_state = LifecycleState.SETTING_POSE

    ##
    # Configuring methods. 
    ##
    def start_configure_amcl(self):
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.change_state_client.srv_name} not available, waiting...')

        self.get_logger().info('Configuring AMCL...')
        self.lifecycle_state = LifecycleState.CONFIGURING
        
        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        
        self.future = self.change_state_client.call_async(request)
        self.future.add_done_callback(self.process_configure_response)

    def process_configure_response(self, future: Future):
        response = future.result()
        if response is None:
            self.get_logger().error("Response from configuring was null")
            return

        self.future: Future = None
        self.lifecycle_state = LifecycleState.CHECKING_STATE
        self.get_logger().info('Sucessfully configured the amcl node')


    ##
    # Configuring methods. 
    ##
    def start_activate_amcl(self):
        while not self.change_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'service {self.change_state_client.srv_name} not available, waiting...')

        self.get_logger().info('Activating AMCL...')
        self.lifecycle_state = LifecycleState.ACTIVATING
        
        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        
        self.future = self.change_state_client.call_async(request)
        self.future.add_done_callback(self.process_activate_response)

    def process_activate_response(self, future: Future):
        response = future.result()
        if response is None:
            self.get_logger().error("Response from configuring was null")
            return

        self.future: Future = None
        self.lifecycle_state = LifecycleState.CHECKING_STATE

    def handle_setting_pose(self):
        """Handle setting initial pose"""
        if self.set_initial_pose and not self.initial_pose_sent:
            self.set_initial_pose_estimate()
        
        self.lifecycle_state = LifecycleState.READY
        self.amcl_ready = True

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

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received keyboard interrupt')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()