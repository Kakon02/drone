// config.h
#pragma once
#include <Arduino.h>
#include <math.h>

extern portMUX_TYPE spinlock;
extern hw_timer_t *timer;

constexpr uint16_t CALIBRATION_SAMPLE_CNT = 2000;
constexpr uint8_t MADGWICK_SAMPLE_FREQ = 90; // Sample frequency for Madgwick filter
constexpr uint16_t MPU_ACCEL_LSB = 4096;
constexpr uint8_t MPU_GYRO_LSB = 131;

constexpr uint8_t SCLK_PIN = 12;
constexpr uint8_t MISO_PIN = 13;
constexpr uint8_t MOSI_PIN = 11;
constexpr uint8_t MPU_CS = 10;
constexpr uint8_t MPU_INT = 4;
constexpr uint8_t AK8963_CNTL1 = 0x0A;
constexpr uint8_t AK8963_ADDRESS = 0x0C;

constexpr uint8_t GPS_UART_RX = 17;
constexpr uint8_t GPS_UART_TX = 16;

constexpr uint8_t MOTOR1_PIN =  45;
constexpr uint8_t MOTOR2_PIN =  48;
constexpr uint8_t MOTOR3_PIN =  47;
constexpr uint8_t MOTOR4_PIN =  21;

constexpr uint16_t MIN_PULSE_LENGTH = 1000; // Minimum pulse length in µs
constexpr uint16_t MAX_PULSE_LENGTH = 2000; // Maximum pulse length in µs

namespace RatePID {
  constexpr float KP_ROLL  = 0.60f;
  constexpr float KI_ROLL  = 3.50f;
  constexpr float KD_ROLL  = 0.03f;

  constexpr float KP_PITCH = KP_ROLL;
  constexpr float KI_PITCH = KI_ROLL;
  constexpr float KD_PITCH = KD_ROLL;

  constexpr float KP_YAW   = 2.00f;
  constexpr float KI_YAW   = 12.0f;
  constexpr float KD_YAW   = 0.0f;
}
