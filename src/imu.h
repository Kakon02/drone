#pragma once
#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <MadgwickAHRS.h>
#include "config.h"

extern uint8_t mpuLSBData[22];                               // 14 (accel+temp+gyro) + 8 (magnetometer)
extern int16_t AccXLSB, AccYLSB, AccZLSB;                    // Accelerometer values in LSB
extern int16_t GyroPitchLSB, GyroRollLSB, GyroYawLSB;                         // Gyroscope values in LSB
extern float magX, magY, magZ;                             // Magnetometer values in LSB                  // in g
extern float AccX, AccY, AccZ;                               // Accelerometer values in g
extern float AngleRoll, AnglePitch, AngleYaw;                // Euler angles in degrees
extern float RateRoll, RatePitch, RateYaw;                   // Gyro rates in degrees per second
extern float AccXOffset, AccYOffset, AccZOffset;             // Calibration offsets for accelerometer
extern float GyroPitchOffset, GyroRollOffset, GyroYawOffset; // Calibration offsets for gyro
extern volatile bool imu_data_ready;

// Read/Write functions
void mpuWriteByte(uint8_t reg, uint8_t data);
uint8_t mpuReadByte(uint8_t reg);
void mpuBurstRead(uint8_t start_reg, uint8_t *buffer, uint8_t length);
void IRAM_ATTR imuISR(void *arg);
void setupMPU9250();
void updateIMUData();
void calibrateIMU();
void updateAttitude();
void setupMPU9250();