#include "config.h"

// Interrupt configuration for MPU9250 and nRF24L01
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;

// SPI configuration for MPU9250 and nRF24L01
const uint8_t SCLK_PIN = 12; // SCL or SCK
const uint8_t MISO_PIN = 13; // AD0
const uint8_t MOSI_PIN = 11; // SDA or SDI
const uint8_t MPU_CS = 10;   // NCS
const uint8_t MPU_INT = 4;   // INT


// AK8963 configuration
const uint8_t AK8963_CNTL1 = 0x0A;   // AK8963 control register 1
const uint8_t AK8963_ADDRESS = 0x0C; // I2C address of AK8963

uint8_t CALIBRATION_SAMPLE_CNT = 2000;
uint8_t MADGWICK_SAMPLE_FREQ = 90;
uint8_t MPU_ACCEL_LSB = 4096;
uint8_t MPU_GYRO_LSB = 131;

//GPS configuration
const uint8_t GPS_UART_RX = 17;
const uint8_t GPS_UART_TX = 16;