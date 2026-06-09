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
     */
    MPU6050(int gyro = 250);
    ~MPU6050();

    bool setUp();
    void calibrateGyro(const int N = 500);
    void processYaw();

    float getYaw() const { return yaw_ema; }

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
    int i2c_handle_;
    
    // Sensor configuration
    double gyro_scale_factor_;
    int gyro_hex_;
    
    // current ema yaw.
    double yaw_ema;
    double yaw;

    // ema values.
    const int N_ = 5;
    const double alpha_ = 2.0 / (N_ + 1);

    // Calibration offsets
    double gyro_z_cal_;

    // Timing for integration
    std::chrono::high_resolution_clock::time_point prev_time_;
    
    // Helper functions
    void gyroSensitivity(int sensitivity);
    int readWord(int reg);
    double getElapsedTime();
    
};

#endif // MPU6050_HPP