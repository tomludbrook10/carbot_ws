/**
 * MPU6050.cpp
 * Implementation of MPU6050 class
 */

#include "carbot_ackermann/MPU6050.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>

MPU6050::MPU6050(int gyro) 
    : i2c_handle_(-1),
      yaw_ema(0),
      gyro_z_cal_(0) {
    gyroSensitivity(gyro);
}

MPU6050::~MPU6050() {
    if (i2c_handle_ >= 0) {
        i2cClose(i2c_handle_);
        std::cout << "I2C connection closed" << std::endl;
    }
    gpioTerminate();
}

void MPU6050::gyroSensitivity(int sensitivity) {
    switch(sensitivity) {
        case 250:
            gyro_scale_factor_ = 131.0;
            gyro_hex_ = 0x00;
            break;
        case 500:
            gyro_scale_factor_ = 65.5;
            gyro_hex_ = 0x08;
            break;
        case 1000:
            gyro_scale_factor_ = 32.8;
            gyro_hex_ = 0x10;
            break;
        case 2000:
            gyro_scale_factor_ = 16.4;
            gyro_hex_ = 0x18;
            break;
        default:
            gyro_scale_factor_ = 65.5;
            gyro_hex_ = 0x08;
    }
}

bool MPU6050::setUp() {
    // Initialize GPIO library
    int init = gpioInitialise();
    if (init < 0) {
        std::cerr << "JETGPIO initialisation failed. Error code: " << init << std::endl;
        return false;
    }
    std::cout << "JETGPIO initialisation OK. Return code: " << init << std::endl;
    
    // Open I2C connection (bus 0, device 0)
    i2c_handle_ = i2cOpen(0, 0);
    if (i2c_handle_ < 0) {
        std::cerr << "Failed to open I2C port. Error code: " << i2c_handle_ << std::endl;
        return false;
    }
    std::cout << "I2C port opened. Handle: " << i2c_handle_ << std::endl;
    
    // Wake up MPU6050 (it starts in sleep mode)
    int writestat = i2cWriteByteData(i2c_handle_, MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
    if (writestat < 0) {
        std::cerr << "Failed to wake up MPU6050" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    
    // Configure gyroscope
    writestat = i2cWriteByteData(i2c_handle_, MPU6050_ADDRESS, GYRO_CONFIG, gyro_hex_);
    if (writestat < 0) {
        std::cerr << "Failed to configure gyroscope" << std::endl;
        return false;
    }

    std::cout << "MPU6050 is sleeping for 1 second" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    std::cout << "MPU6050 set up:" << std::endl;
    std::cout << "\tGyroscope: 0x" << std::hex << gyro_hex_ << std::dec 
              << " Scale: " << gyro_scale_factor_ << std::endl;
    return true;
}

int MPU6050::readWord(int reg) {
    // Read high byte
    int h = i2cReadByteData(i2c_handle_, MPU6050_ADDRESS, reg); /// return a bit mask where the bottom 8 bits are the data
    // Read low byte
    int l = i2cReadByteData(i2c_handle_, MPU6050_ADDRESS, reg + 1); // same here. 

    if (h < 0 || l < 0) {
        std::cerr << "Error reading from register 0x" << std::hex << reg << std::dec << std::endl;
        return 0;
    }
    
    // Combine into 16-bit value
    int val = (h << 8) | l; // move high byte to upper 8 bits and combine with low byte
    if (val >= 0x8000) // if 
        val = -((65535 - val) + 1);
    return val;
}

void MPU6050::calibrateGyro(const int N) {
    std::cout << "Calibrating gyroscope with " << N << " points. Do not move!" << std::endl;
    gyro_z_cal_ = 0;

    // Take N readings and accumulate
    for (int i = 0; i < N; i++) {
        auto raw_gyro_value = static_cast<double>(readWord(GYRO_ZOUT_H));
        gyro_z_cal_ += raw_gyro_value;
    }
    gyro_z_cal_ /= N;
    prev_time_ = std::chrono::high_resolution_clock::now();
}

void normalizeAngle(double &angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
}

void MPU6050::processYaw() {
    auto raw_gyro_value = static_cast<double>(readWord(GYRO_ZOUT_H));
    raw_gyro_value -= gyro_z_cal_;
    raw_gyro_value /= gyro_scale_factor_;
    raw_gyro_value *= -1; // Invert Z axis gyro, as it's mounted upside down

    // now we need turn a yaw. 
    auto dt = getElapsedTime();
    yaw += raw_gyro_value * dt;

    normalizeAngle(yaw);

    // ema filter 
    yaw_ema = alpha_ * yaw + (1 - alpha_) * yaw_ema;
}

double MPU6050::getElapsedTime() {
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - prev_time_;
    prev_time_ = current_time;
    return elapsed.count();
}