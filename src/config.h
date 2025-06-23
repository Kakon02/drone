// config.h
#pragma once
#include <Arduino.h>
#include "imu.h"
#include "nrf.h"

extern portMUX_TYPE spinlock;
extern hw_timer_t *timer;

constexpr uint16_t CALIBRATION_SAMPLE_CNT = 2000;
constexpr uint8_t MADGWICK_SAMPLE_FREQ = 90; // Sample frequency for Madgwick filter
constexpr uint16_t MPU_ACCEL_LSB = 4096;
constexpr uint8_t MPU_GYRO_LSB = 131;

constexpr uint8_t SCLK_PIN = 12;
constexpr uint8_t MISO_PIN = 13;
constexpr uint8_t MOSI_PIN = 11;
constexpr uint8_t MPU_CS = 35;
constexpr uint8_t MPU_INT = 4;
constexpr uint8_t NRF_CE = 38;
constexpr uint8_t NRF_CS = 0;
constexpr uint8_t NRF_INT = 6;
constexpr uint8_t AK8963_CNTL1 = 0x0A;
constexpr uint8_t AK8963_ADDRESS = 0x0C;

constexpr uint8_t GPS_UART_RX = 17;
constexpr uint8_t GPS_UART_TX = 16;

constexpr uint8_t MOTOR_PINS[4] = {45, 48, 47, 21}; // Motor pins for ESCs
constexpr uint8_t MOTOR_CHANNELS[4] = {0, 1, 2, 3}; // PWM channels for ESCs

constexpr uint16_t MIN_PULSE_LENGTH = 1000; // Minimum pulse length in µs
constexpr uint16_t MAX_PULSE_LENGTH = 2000; // Maximum pulse length in µs
constexpr uint16_t PWM_PERIOD_US = 20000;   // 1s / 50Hz = 20,000 microseconds = 20 milliseconds
constexpr uint8_t PWM_FREQ = 50;            // ESCs expect ~50Hz
constexpr uint8_t PWM_RESOLUTION = 13;      // 13-bit resolution: 0–8191(0~100%) (ESP32 supports maximum 16-bit resolution)

namespace RatePID
{
  constexpr float KP_ROLL = 0.60f;
  constexpr float KI_ROLL = 3.50f;
  constexpr float KD_ROLL = 0.03f;

  constexpr float KP_PITCH = KP_ROLL;
  constexpr float KI_PITCH = KI_ROLL;
  constexpr float KD_PITCH = KD_ROLL;

  constexpr float KP_YAW = 2.00f;
  constexpr float KI_YAW = 12.0f;
  constexpr float KD_YAW = 0.0f;
}

void initSharedSPI();
void boardSetup();