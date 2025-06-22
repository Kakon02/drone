#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include "config.h"

void resetESCs();
void sendPulse(int channelIndex, int pulse_us);
void sendAllMotors(int pulse_us);
void testESC();
void displayCalibrationInstructions();
void startCalibration();
void handleCalibrationInput(char input);