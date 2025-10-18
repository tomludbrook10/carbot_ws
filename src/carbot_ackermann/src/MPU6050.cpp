/**
 * MPU6050.cpp
 * Implementation of MPU6050 class
 */

#include "carbot_ackermann/MPU6050.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>

MPU6050::MPU6050(int gyro, int acc, float tau) 
    : i2cHandle(-1), tau(tau), gx(0), gy(0), gz(0), 
      ax(0), ay(0), az(0), gyroXcal(0), gyroYcal(0), gyroZcal(0),
      roll(0), pitch(0), yaw(0), gyroRoll(0), gyroPitch(0), gyroYaw(0) {
    
    // Set gyroscope sensitivity
    gyroSensitivity(gyro);
    
    // Set accelerometer sensitivity
    accelerometerSensitivity(acc);
}

MPU6050::~MPU6050() {
    if (i2cHandle >= 0) {
        i2cClose(i2cHandle);
        std::cout << "I2C connection closed" << std::endl;
    }
    gpioTerminate();
}

void MPU6050::gyroSensitivity(int sensitivity) {
    switch(sensitivity) {
        case 250:
            gyroScaleFactor = 131.0;
            gyroHex = 0x00;
            break;
        case 500:
            gyroScaleFactor = 65.5;
            gyroHex = 0x08;
            break;
        case 1000:
            gyroScaleFactor = 32.8;
            gyroHex = 0x10;
            break;
        case 2000:
            gyroScaleFactor = 16.4;
            gyroHex = 0x18;
            break;
        default:
            gyroScaleFactor = 65.5;
            gyroHex = 0x08;
    }
}

void MPU6050::accelerometerSensitivity(int sensitivity) {
    switch(sensitivity) {
        case 2:
            accScaleFactor = 16384.0;
            accHex = 0x00;
            break;
        case 4:
            accScaleFactor = 8192.0;
            accHex = 0x08;
            break;
        case 8:
            accScaleFactor = 4096.0;
            accHex = 0x10;
            break;
        case 16:
            accScaleFactor = 2048.0;
            accHex = 0x18;
            break;
        default:
            accScaleFactor = 8192.0;
            accHex = 0x08;
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
    i2cHandle = i2cOpen(0, 0);
    if (i2cHandle < 0) {
        std::cerr << "Failed to open I2C port. Error code: " << i2cHandle << std::endl;
        return false;
    }
    std::cout << "I2C port opened. Handle: " << i2cHandle << std::endl;
    
    // Wake up MPU6050 (it starts in sleep mode)
    int writestat = i2cWriteByteData(i2cHandle, MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
    if (writestat < 0) {
        std::cerr << "Failed to wake up MPU6050" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure accelerometer
    writestat = i2cWriteByteData(i2cHandle, MPU6050_ADDRESS, ACCEL_CONFIG, accHex);
    if (writestat < 0) {
        std::cerr << "Failed to configure accelerometer" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure gyroscope
    writestat = i2cWriteByteData(i2cHandle, MPU6050_ADDRESS, GYRO_CONFIG, gyroHex);
    if (writestat < 0) {
        std::cerr << "Failed to configure gyroscope" << std::endl;
        return false;
    }

    std::cout << "MPU6050 is sleeping for 4 seconds" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(4));
    
    std::cout << "MPU6050 set up:" << std::endl;
    std::cout << "\tAccelerometer: 0x" << std::hex << accHex << std::dec 
              << " Scale: " << accScaleFactor << std::endl;
    std::cout << "\tGyroscope: 0x" << std::hex << gyroHex << std::dec 
              << " Scale: " << gyroScaleFactor << std::endl;
    return true;
}

int MPU6050::readWord(int reg) {
    // Read high byte
    int h = i2cReadByteData(i2cHandle, MPU6050_ADDRESS, reg);
    // Read low byte
    int l = i2cReadByteData(i2cHandle, MPU6050_ADDRESS, reg + 1);
    
    if (h < 0 || l < 0) {
        std::cerr << "Error reading from register 0x" << std::hex << reg << std::dec << std::endl;
        return 0;
    }
    
    // Combine into 16-bit value
    int val = (h << 8) | l;
    if (val >= 0x8000)
        val = -((65535 - val) + 1);
    return val;
}

void MPU6050::getRawData() {
    gx = readWord(GYRO_XOUT_H);
    gy = readWord(GYRO_YOUT_H);
    gz = readWord(GYRO_ZOUT_H);
    
    ax = readWord(ACCEL_XOUT_H);
    ay = readWord(ACCEL_YOUT_H);
    az = readWord(ACCEL_ZOUT_H);
}

void MPU6050::calibrateGyro(const int N) {
    std::cout << "Calibrating gyroscope with " << N << " points. Do not move!" << std::endl;
    
    // Reset calibration values
    gyroXcal = 0;
    gyroYcal = 0;
    gyroZcal = 0;
    
    // Take N readings and accumulate
    for (int i = 0; i < N; i++) {
        getRawData();
        gyroXcal += gx;
        gyroYcal += gy;
        gyroZcal += gz;
    }
    
    // Calculate average offset
    gyroXcal /= N;
    gyroYcal /= N;
    gyroZcal /= N;
    
    std::cout << "Calibration complete:" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "\tX axis offset: " << gyroXcal << std::endl;
    std::cout << "\tY axis offset: " << gyroYcal << std::endl;
    std::cout << "\tZ axis offset: " << gyroZcal << std::endl << std::endl;
    
    // Start timer for complementary filter
    dtTimer = std::chrono::high_resolution_clock::now();
}

void MPU6050::processIMUvalues() {
    // Get raw data
    getRawData();
    
    // Subtract calibration offsets
    gx -= gyroXcal;
    gy -= gyroYcal;
    gz -= gyroZcal;
    
    // Convert to degrees per second
    gx /= gyroScaleFactor;
    gy /= gyroScaleFactor;
    gz /= gyroScaleFactor;
    
    // Convert to g force
    ax /= accScaleFactor;
    ay /= accScaleFactor;
    az /= accScaleFactor;
}

double MPU6050::getElapsedTime() {
    auto currentTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = currentTime - dtTimer;
    dtTimer = currentTime;
    return elapsed.count();
}

void MPU6050::compFilter() {
    // Process IMU values
    processIMUvalues();
    
    // Get time delta
    double dt = getElapsedTime();
    
    // Calculate angles from accelerometer
    float accPitch = atan2(ay, az) * 180.0 / M_PI;
    float accRoll = atan2(ax, az) * 180.0 / M_PI;
    
    // Gyro integration
    gyroRoll -= gy * dt;
    gyroPitch += gx * dt;
    gyroYaw += gz * dt;
    yaw = gyroYaw;
    
    // Complementary filter
    roll = tau * (roll - gy * dt) + (1.0 - tau) * accRoll;
    pitch = tau * (pitch + gx * dt) + (1.0 - tau) * accPitch;
}