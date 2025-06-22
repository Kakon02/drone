#include "imu.h"

uint8_t mpuLSBData[22]; // 14 (accel+temp+gyro) + 8 (magnetometer)
int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroPitchLSB, GyroRollLSB, GyroYawLSB;

float magX, magY, magZ; // Magnetometer values
float AccX, AccY, AccZ;
float AccXOffset, AccYOffset, AccZOffset;             // Calibration offsets for accelerometer
float RatePitch, RateRoll, RateYaw;                   // Gyro rates in degrees per second, updated in main loop with madgwick filter
float GyroPitchOffset, GyroRollOffset, GyroYawOffset; // Calibration offsets for gyro
float AngleRoll, AnglePitch, AngleYaw;                // Only Value that could be used externally (pid.cpp)

volatile bool imu_data_ready = false;

Madgwick filter; // Create an instance of the Madgwick filter

// === SPI Functions ===
void mpuWriteByte(uint8_t reg, uint8_t data)
{
    digitalWrite(MPU_CS, LOW);
    SPI.transfer(reg & 0x7F); // Write
    SPI.transfer(data);
    digitalWrite(MPU_CS, HIGH);
}

uint8_t mpuReadByte(uint8_t reg)
{
    digitalWrite(MPU_CS, LOW);
    SPI.transfer(reg | 0x80); // Read
    uint8_t data = SPI.transfer(0x00);
    digitalWrite(MPU_CS, HIGH);
    return data;
}

void mpuBurstRead(uint8_t start_reg, uint8_t *buffer, uint8_t length)
{
    digitalWrite(MPU_CS, LOW);
    SPI.transfer(start_reg | 0x80);
    for (int i = 0; i < length; i++)
    {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(MPU_CS, HIGH);
}

// === ISR ===
void IRAM_ATTR imuISR(void *arg)
{
    portENTER_CRITICAL_ISR(&spinlock);
    imu_data_ready = true;
    portEXIT_CRITICAL_ISR(&spinlock);
}

void setupMPU9250()
{
    // Reset MPU and configure registers
    mpuWriteByte(0x6B, 0x00); // Wake up
    delay(100);

    // Set up I2C Master for AK8963
    mpuWriteByte(0x6A, 0x20); // Set I2C Master mode
    mpuWriteByte(0x24, 0x0D); // I2C Master clock 400kHz

    mpuWriteByte(0x28, AK8963_ADDRESS); // Save AK8963 I2C address to SLV1_ADDR register
    mpuWriteByte(0x29, AK8963_CNTL1);   // Save AK8963 CNTL1 register address to SLV1_REG register
    mpuWriteByte(0x64, 0x16);           // Set Continuouse mode 2, 16-bit resolution
    mpuWriteByte(0x2A, 0x81);           // Enable SLV1 to write to AK8963 CNTL1 register
    delay(100);

    mpuWriteByte(0x25, AK8963_ADDRESS); // Save AK8963 I2C address to SLV0_ADDR register in read mode
    mpuWriteByte(0x26, 0x02);           // Save starting register to read inside AK8963 to SLV0_REG register
    mpuWriteByte(0x27, 0x88);           // Set SLV0 to read 8 bytes from AK8963 (ST1 ~ ST2)
    delay(100);

    // Configure MPU9250 registers
    mpuWriteByte(0x6C, 0x00); // Enable gyro/accel
    mpuWriteByte(0x1A, 0x02); // DLPF config (Gyro BW 92Hz, Delay 3.9ms, Fs 1KHz => Contains data averaged over 4 samples)
    mpuWriteByte(0x1D, 0x02); // Accel DLPF config (Accel BW 99Hz, Delay 2.88ms, Fs 1KHz => Contains data averaged over 3 samples)
    mpuWriteByte(0x1B, 0x00); // Gyro ±250dps
    mpuWriteByte(0x1C, 0x00); // Accel ±2g (Default DLPF, Delay 1.88ms, )
    mpuWriteByte(0x38, 0x01); // Enable DRDY interrupt
    mpuWriteByte(0x37, 0x00); // INT pin = push-pull, active high
}

void updateIMUData()
{
    if (imu_data_ready)
    {
        portENTER_CRITICAL(&spinlock);
        imu_data_ready = false;
        portEXIT_CRITICAL(&spinlock);

        mpuBurstRead(0x3B, mpuLSBData, 22); // Read from 0x3B to 0x48

        // Accel
        AccXLSB = (mpuLSBData[0] << 8) | mpuLSBData[1];
        AccYLSB = (mpuLSBData[2] << 8) | mpuLSBData[3];
        AccZLSB = (mpuLSBData[4] << 8) | mpuLSBData[5];

        // Gyro
        GyroPitchLSB = (mpuLSBData[8] << 8) | mpuLSBData[9];
        GyroRollLSB = (mpuLSBData[10] << 8) | mpuLSBData[11];
        GyroYawLSB = (mpuLSBData[12] << 8) | mpuLSBData[13];

        // // Magnetometer (via EXT_SENS_DATA)
        // uint8_t st1 = mpuLSBData[14]; // EXT_SENS_DATA_00

        // if (st1 & 0x01)
        // {                                                  // DRDY is set
        //     magX = ((mpuLSBData[15] << 8) | mpuLSBData[16]); // HXL, HXH
        //     magY = (mpuLSBData[17] << 8) | mpuLSBData[18]; // HYL, HYH
        //     magZ = (mpuLSBData[19] << 8) | mpuLSBData[20]; // HZL, HZH
        // }

        AccX = (float)AccXLSB / MPU_ACCEL_LSB; // Convert to g
        AccY = (float)AccYLSB / MPU_ACCEL_LSB;
        AccZ = (float)AccZLSB / MPU_ACCEL_LSB;

        RatePitch = (float)GyroRollLSB / MPU_GYRO_LSB; // Convert LSB to degrees per second
        RateRoll = (float)GyroPitchLSB / MPU_GYRO_LSB;
        RateYaw = (float)GyroYawLSB / MPU_GYRO_LSB;
    }
}

void calibrateIMU()
{
    Serial.println("Starting IMU calibration in 3");
    delay(1000);
    Serial.println("2");
    delay(1000);
    Serial.println("1");
    delay(1000);

    Serial.println("Calibrating IMU...");
    Serial.println("Please keep the IMU still during calibration.");

    AccXOffset = 0.0;
    AccYOffset = 0.0;
    AccZOffset = 0.0;
    GyroPitchOffset = 0.0;
    GyroRollOffset = 0.0;
    GyroYawOffset = 0.0;

    for (int i = 0; i < CALIBRATION_SAMPLE_CNT; i++)
    {
        updateIMUData();
        AccXOffset += AccX;
        AccYOffset += AccY;
        AccZOffset += AccZ;
        GyroPitchOffset += RatePitch;
        GyroRollOffset += RateRoll;
        GyroYawOffset += RateYaw;

        delay(1); // Delay to allow for stable readings
    }
    AccXOffset /= CALIBRATION_SAMPLE_CNT;
    AccYOffset /= CALIBRATION_SAMPLE_CNT;
    AccZOffset /= CALIBRATION_SAMPLE_CNT;
    GyroPitchOffset /= CALIBRATION_SAMPLE_CNT;
    GyroRollOffset /= CALIBRATION_SAMPLE_CNT;
    GyroYawOffset /= CALIBRATION_SAMPLE_CNT;
    Serial.println("Calibration Offset Set");
}

void setupMadgwickFilter()
{
    filter.begin(MADGWICK_SAMPLE_FREQ); // Set your loop frequency (Current default is 90Hz)
}

void updateAttitude()
{
    AccX -= AccXOffset;
    AccY -= AccYOffset;
    AccZ -= AccZOffset;

    RatePitch = (RatePitch - GyroPitchOffset) * DEG_TO_RAD;
    RateRoll = (RateRoll - GyroRollOffset) * DEG_TO_RAD;
    RateYaw = (RateYaw - GyroYawOffset) * DEG_TO_RAD;
    // Call Madgwick filter update
    filter.updateIMU(RatePitch, RateRoll, RateYaw, AccX, AccY, AccZ);
    // filter.update(RatePitch, RateRoll, RateYaw,
    //               AccX, AccY, AccZ,
    //               magX, magY, magZ);
    // Removed magnetometer data for now.
    // Get filtered orientation angles
    AngleRoll = filter.getRoll();   // degrees
    AnglePitch = filter.getPitch(); // degrees
    AngleYaw = filter.getYaw();     // degrees
}
