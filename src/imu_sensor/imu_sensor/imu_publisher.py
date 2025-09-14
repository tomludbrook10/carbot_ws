import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.time import Time
import random
from geometry_msgs.msg import Quaternion
from imu_sensor.mpu6050 import MPU
import math

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw (radians) to a normalized quaternion (w,x,y,z).
    roll: rotation about X
    pitch: rotation about Y
    yaw: rotation about Z
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    # normalize
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm == 0:
        # fallback to identity
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    w /= norm; x /= norm; y /= norm; z /= norm

    q = Quaternion()
    q.w = w
    q.x = x
    q.y = y
    q.z = z
    return q

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'demo/imu', 10)
        self.timer = self.create_timer(0.1, self.publish_imu)  # 10 Hz

        ## setting the mpu6050 api. 
        # Set up class
        gyro = 250      # 250, 500, 1000, 2000 [deg/s]
        acc = 2         # 2, 4, 7, 16 [g]
        tau = 0.98
        self.mpu = MPU(gyro, acc, tau)

        # Set up sensor and calibrate gyro with N points
        self.mpu.setUp()
        self.mpu.calibrateGyro(500)

        # Define covariance matrices based on MPU6050 specifications
        # These values are tuned for typical MPU6050 performance
        
        # Orientation covariance (roll, pitch, yaw) - complementary filter adds some noise
        # Values in rad^2, higher uncertainty for yaw due to magnetometer absence
        self.orientation_covariance = [
            0.001, 0.0,   0.0,     # roll variance and cross-correlations
            0.0,   0.001, 0.0,     # pitch variance  
            0.0,   0.0,   0.01     # yaw variance (higher due to integration drift)
        ]
        
        # Angular velocity covariance (x, y, z) in (rad/s)^2
        # MPU6050 gyro noise density: ~0.005 deg/s/√Hz
        # At 10Hz sampling: noise ≈ 0.005 * √10 ≈ 0.016 deg/s ≈ 0.0003 rad/s
        gyro_noise_variance = (0.003) ** 2
        self.angular_velocity_covariance = [
            gyro_noise_variance, 0.0, 0.0,
            0.0, gyro_noise_variance, 0.0, 
            0.0, 0.0, gyro_noise_variance
        ]
        
        # Linear acceleration covariance (x, y, z) in (m/s^2)^2  
        # MPU6050 accel noise density: ~400 μg/√Hz
        # At 10Hz: noise ≈ 400e-6 * 9.81 * √10 ≈ 0.012 m/s^2
        accel_noise_variance = (0.012) ** 2
        self.linear_acceleration_covariance = [
            accel_noise_variance, 0.0, 0.0,
            0.0, accel_noise_variance, 0.0,
            0.0, 0.0, accel_noise_variance
        ]

        self.get_logger().info('IMU setup and publishing with covariances')
        

    def publish_imu(self):
        msg = Imu()
        self.mpu.compFilter()

        current_time = self.get_clock().now()

        # Set the timestamp correctly
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'base_link'

        # Sensor data
        msg.orientation = euler_to_quaternion(math.radians(self.mpu.roll), 
                                            math.radians(self.mpu.pitch), 
                                            math.radians(self.mpu.yaw))
        msg.angular_velocity.x = self.mpu.gx
        msg.angular_velocity.y = self.mpu.gy
        msg.angular_velocity.z = self.mpu.gz
        msg.linear_acceleration.x = self.mpu.ax
        msg.linear_acceleration.y = self.mpu.ay
        msg.linear_acceleration.z = self.mpu.az
        
        # Set covariance matrices
        # ROS2 expects flattened 3x3 matrices (9 elements each)
        msg.orientation_covariance = self.orientation_covariance
        msg.angular_velocity_covariance = self.angular_velocity_covariance  
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
