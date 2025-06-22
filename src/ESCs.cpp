#include <Arduino.h>
#include "ESCs.h"

void displayCalibrationInstructions()
{
  Serial.println("READY - PLEASE SEND INSTRUCTIONS:");
  Serial.println("\t0 : Send min throttle");
  Serial.println("\t1 : Send max throttle");
  Serial.println("\t2 : Run test ramp from min to max");
  Serial.println("\t3 : End calibration");
}

void resetESCs()
{
  for (int i = 0; i < 4; i++)
  {
    ledcSetup(MOTOR_CHANNELS[0], PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(MOTOR_CHANNELS[1], PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(MOTOR_CHANNELS[2], PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(MOTOR_CHANNELS[3], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PINS[i], MOTOR_CHANNELS[i]);
  }
}

void sendPulse(int channelIndex, int pulse_us)
{
  pulse_us = constrain(pulse_us, 1000, 2000);
  // 5% of 8191 ≈ 410, 10% ≈ 819
  uint16_t duty = map(pulse_us, 1000, 2000, 410, 819); // uint16_t size: (0~65535)
  ledcWrite(MOTOR_CHANNELS[channelIndex], duty);
}

void sendAllMotors(int pulse_us)
{
  for (int i = 0; i < 4; i++)
  {
    sendPulse(i, pulse_us);
  }
}

void testESC()
{
  for (int pulse = MIN_PULSE_LENGTH; pulse <= MAX_PULSE_LENGTH; pulse += 5)
  {
    Serial.print("Pulse length = ");
    Serial.println(pulse);

    sendAllMotors(pulse);
    delay(200);
  }

  Serial.println("STOP");
  sendAllMotors(MIN_PULSE_LENGTH);
}

void startCalibration()
{
  bool endCalibration = false;
  displayCalibrationInstructions();

  while (!endCalibration)
  {
    if (Serial.available())
    {
      char data = Serial.read();

      switch (data)
      {
      case '0':
        Serial.println("Sending minimum throttle");
        sendAllMotors(MIN_PULSE_LENGTH);
        break;
      case '1':
        Serial.println("Sending maximum throttle");
        sendAllMotors(MAX_PULSE_LENGTH);
        break;
      case '2': // Test ESCs only after the calibration is finished
        Serial.println("Running test in 3");
        delay(1000);
        Serial.print("2 ");
        delay(1000);
        Serial.println("1...");
        delay(1000);
        testESC();
        break;
      case '3':
        Serial.println("Ending calibration");
        endCalibration = true;
        break;
      }
    }
    delay(100); // Prevents flooding the serial port
  }
}
