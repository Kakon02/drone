#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// ---------------------------------------------------------------------------
#define MIN_PULSE_LENGTH 1000  // in microseconds
#define MAX_PULSE_LENGTH 2000  // in microseconds
#define PWM_FREQ 50            // ESCs expect ~50Hz
#define PWM_RESOLUTION 13      // 13-bit resolution: 0–8191(0~100%) (ESP32 supports maximum 16-bit resolution)
#define PWM_PERIOD_US 20000    // 1s / 50Hz = 20,000 microseconds = 20 milliseconds
// ---------------------------------------------------------------------------
// ESC pins (change to your actual wiring)
const int motorPins[4] = {4, 5, 6, 7};
const int motorChannels[4] = {0, 1, 2, 3}; // 4 LEDC channels
char data;
// ---------------------------------------------------------------------------
void sendPulse(int channelIndex, int pulse_us);
void sendAllMotors(int pulse_us);
void testESC();
void displayInstructions();
// ---------------------------------------------------------------------------


void setup() {
  Serial.begin(115200);
  
  ledcSetup(motorChannels[0], PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(motorChannels[1], PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(motorChannels[2], PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(motorChannels[3], PWM_FREQ, PWM_RESOLUTION);
  
  for (int i = 0; i < 4; i++) {
  ledcAttachPin(motorPins[i], motorChannels[i]);
  sendPulse(i, MAX_PULSE_LENGTH);
}

  displayInstructions();
}

void loop() {
  if (Serial.available()) {
    data = Serial.read();

    switch (data) {
      case '0':
        Serial.println("Sending minimum throttle");
        sendAllMotors(MIN_PULSE_LENGTH);
        break;

      case '1':
        Serial.println("Sending maximum throttle");
        sendAllMotors(MAX_PULSE_LENGTH);
        break;

      case '2': //Test ESCs only after the calibration is finished
        Serial.println("Running test in 3");
        delay(1000);
        Serial.print("2 ");
        delay(1000);
        Serial.println("1...");
        delay(1000);
        testESC();
        break;
    }
  }
}

// ---------------------------------------------------------------------------
void sendPulse(int channelIndex, int pulse_us) {
  pulse_us = constrain(pulse_us, 1000, 2000);
  // 5% of 8191 ≈ 410, 10% ≈ 819
  uint16_t duty = map(pulse_us, 1000, 2000, 410, 819); //uint16_t size: (0~65535)
  ledcWrite(motorChannels[channelIndex], duty);
}

void sendAllMotors(int pulse_us) {
  for (int i = 0; i < 4; i++) {
    sendPulse(i, pulse_us);
  }
}

// ---------------------------------------------------------------------------

void testESC() {
  for (int pulse = MIN_PULSE_LENGTH; pulse <= MAX_PULSE_LENGTH; pulse += 5) {
    Serial.print("Pulse length = ");
    Serial.println(pulse);

    sendAllMotors(pulse);
    delay(200);
  }

  Serial.println("STOP");
  sendAllMotors(MIN_PULSE_LENGTH);
}

// ---------------------------------------------------------------------------

void displayInstructions() {
  Serial.println("READY - PLEASE SEND INSTRUCTIONS:");
  Serial.println("\t0 : Send min throttle");
  Serial.println("\t1 : Send max throttle");
  Serial.println("\t2 : Run test ramp from min to max");
}
