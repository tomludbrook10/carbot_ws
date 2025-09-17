import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
import time
import sys

class AMCL(Node):
    def __init__(self):
        super().__init__('amcl_test')
        
        # Service clients for lifecycle management
        self.get_state_client = self.create_client(
            GetState, '/amcl/get_state'
        )
        
        # Wait for service to be available before starting timer
        self.wait_for_service()
        
        self.timer = self.create_timer(1.0, self.get_state_callback)
        self.pending_future = None
        
    def wait_for_service(self):
        """Wait for service to become available"""
        while not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def get_state_callback(self):
        """Timer callback - handles the async service call"""
        # Check if we have a pending request
        if self.pending_future is not None:
            if self.pending_future.done():
                self.handle_service_response()
            else:
                # Still waiting for previous request
                return
        
        # Make new service request
        if self.get_state_client.service_is_ready():
            request = GetState.Request()
            self.pending_future = self.get_state_client.call_async(request)
        else:
            self.get_logger().warn('Service not ready')

    def handle_service_response(self):
        """Handle the service response"""
        try:
            result = self.pending_future.result()
            if result is not None:
                self.get_logger().info(f'Current AMCL state: {result.current_state.label} (ID: {result.current_state.id})')
            else:
                self.get_logger().error('Service call returned None')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
        finally:
            self.pending_future = None

def main(args=None):
    rclpy.init(args=args)
    node = AMCL()
    node.get_logger().info("AMCL test node started")
    
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

if __name__ == '__main__':
    main()