/**
 * MPU6050.hpp
 * C++ implementation of MPU6050 IMU sensor with complementary filter
 * Uses JETGPIO library for I2C communication
 */

#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <jetgpio.h>
#include <cmath>
#include <iostream>
#include <chrono>

class MPU6050 {
public:
    /**
     * Constructor
     * @param gyro Gyroscope sensitivity (250, 500, 1000, 2000 deg/s)
     * @param acc Accelerometer sensitivity (2, 4, 8, 16 g)
     * @param tau Complementary filter coefficient (0-1, typically 0.98)
     */
    MPU6050(int gyro = 250, int acc = 2, float tau = 0.98);
    
    /**
     * Destructor - closes I2C connection
     */
    ~MPU6050();
    
    /**
     * Initialize the MPU6050 sensor
     * @return true if successful, false otherwise
     */
    bool setUp();
    
    /**
     * Calibrate gyroscope (sensor should be stationary)
     * @param N Number of calibration samples
     */
    void calibrateGyro(const int N = 500);
    
    /**
     * Run complementary filter to update orientation
     */
    void compFilter();
    
    /**
     * Get current roll angle in degrees
     */
    float getRoll() const { return roll; }
    
    /**
     * Get current pitch angle in degrees
     */
    float getPitch() const { return pitch; }
    
    /**
     * Get current yaw angle in degrees
     */
    float getYaw() const { return yaw; }
    
    /**
     * Get gyroscope readings in deg/s
     */
    void getGyro(float& gx_out, float& gy_out, float& gz_out) const {
        gx_out = gx; gy_out = gy; gz_out = gz;
    }
    
    /**
     * Get accelerometer readings in g
     */
    void getAccel(float& ax_out, float& ay_out, float& az_out) const {
        ax_out = ax; ay_out = ay; az_out = az;
    }

private:
    // MPU6050 register addresses
    static const int MPU6050_ADDRESS = 0x68;
    static const int PWR_MGMT_1 = 0x6B;
    static const int ACCEL_CONFIG = 0x1C;
    static const int GYRO_CONFIG = 0x1B;
    static const int GYRO_XOUT_H = 0x43;
    static const int GYRO_YOUT_H = 0x45;
    static const int GYRO_ZOUT_H = 0x47;
    static const int ACCEL_XOUT_H = 0x3B;
    static const int ACCEL_YOUT_H = 0x3D;
    static const int ACCEL_ZOUT_H = 0x3F;
    
    // I2C handle
    int i2cHandle;
    
    // Sensor configuration
    float gyroScaleFactor;
    int gyroHex;
    float accScaleFactor;
    int accHex;
    float tau;
    
    // Raw sensor data
    float gx, gy, gz;  // Gyroscope data (deg/s)
    float ax, ay, az;  // Accelerometer data (g)
    
    // Calibration offsets
    float gyroXcal, gyroYcal, gyroZcal;
    
    // Orientation angles
    float roll, pitch, yaw;
    float gyroRoll, gyroPitch, gyroYaw;
    
    // Timing for integration
    std::chrono::high_resolution_clock::time_point dtTimer;
    
    // Helper functions
    void gyroSensitivity(int sensitivity);
    void accelerometerSensitivity(int sensitivity);
    int readWord(int reg);
    void getRawData();
    void processIMUvalues();
    double getElapsedTime();
};

#endif // MPU6050_HPP