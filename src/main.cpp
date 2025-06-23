/*********************************************************************
 *  main.cpp  –  ESP32-S3 “brain” for 4-motor X-configuration quadcopter
 *               Last edit: 2025-06-23
 *********************************************************************/
#include <Arduino.h>
#include "config.h" // common pins/const and boardSetup()
#include "imu.h"
#include "nrf.h"
#include "ESCs.h"
#include "PID.h"

// ─────────── Globals ───────────
static PIDState pidRoll, pidPitch, pidYaw;
static uint32_t lastRxMicros = 0;                             // fail-safe timer
inline float mapSym(int16_t pwm, float outMin, float outMax); // helper: map radio input to set-point range
void pollRadio();                                             // helper: non-blocking read of radio packet

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("==== Quadcopter FC booting ===="));
  boardSetup();
  Serial.println(F("Board setup complete"));
  // ── Peripherals ──────────────────────────
  setupMPU9250();
  calibrateIMU();
  setupMadgwickFilter(); // 90 Hz (configurable)

  setupNRF24L01(); // loads GS addr & listens
  resetESCs();     // attach motors & zero

  Serial.println(F("Init complete waiting for throttle-low radio signal"));

  while (inputThrottle < 1020 || inputThrottle > 1050)
  {
    pollRadio(); // wait for throttle-low signal
    delay(1000);
  }

  lastRxMicros = micros();
}

// ───────────────── main loop (runs as fast as possible ≈2–3 kHz) ───────────────
void loop()
{
  static uint32_t tPrev = micros(); // tPrev: previous time in µs
  const uint32_t tNow = micros();
  const float dt = (tNow - tPrev) * 1e-6f; // seconds
  tPrev = tNow;

  // ── Sensors ──
  updateIMUData();  // pulls burst SPI when DRDY ISR set
  updateAttitude(); // Madgwick → AngleRoll/Pitch/Yaw

  // ── Radio ──
  pollRadio();

  // ── Fail-safe: if no packet in 300 ms → cut motors ─────────
  if ((uint32_t)(tNow - lastRxMicros) > 300000)
  {
    sendAllMotors(MIN_PULSE_LENGTH); // ✂ power
    return;
  }

  // ── Pilot set-points (deg & deg/s) ─────
  const float desRoll = mapSym(inputRoll, -25.0f, 25.0f); // ±25 °
  const float desPitch = mapSym(inputPitch, -25.0f, 25.0f);
  const float desYawRate = mapSym(inputYaw, -150.0f, 150.0f); // ±150 °/s
  const int baseThrottle = constrain(inputThrottle, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH - 200);
  // (throttle is 1000–2000 µs, but we reserve 200 µs for PID output)

  //    PID (outer angle loop already skipped; we go
  //    straight to rate control using Madgwick angles)
  const float errRoll = desRoll - AngleRoll;
  const float errPitch = desPitch - AnglePitch;
  const float errYaw = (desYawRate * DEG_TO_RAD) - RateYaw; // rad/s

  const float rollOut = pidCompute(errRoll,
                                   RatePID::KP_ROLL, RatePID::KI_ROLL, RatePID::KD_ROLL,
                                   dt, 500.0f, pidRoll); // ±500 µs overhead
  const float pitchOut = pidCompute(errPitch,
                                    RatePID::KP_PITCH, RatePID::KI_PITCH, RatePID::KD_PITCH,
                                    dt, 500.0f, pidPitch);
  const float yawOut = pidCompute(errYaw * RAD_TO_DEG, // keep PID gains in deg
                                  RatePID::KP_YAW, RatePID::KI_YAW, RatePID::KD_YAW,
                                  dt, 500.0f, pidYaw);

  // ── Motor mix (X-quad, front = motor0) ──────────────
  int m0 = baseThrottle + pitchOut + rollOut - yawOut;
  int m1 = baseThrottle + pitchOut - rollOut + yawOut;
  int m2 = baseThrottle - pitchOut - rollOut - yawOut;
  int m3 = baseThrottle - pitchOut + rollOut + yawOut;

  // saturate and send to ESCs
  sendPulse(0, constrain(m0, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));
  sendPulse(1, constrain(m1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));
  sendPulse(2, constrain(m2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));
  sendPulse(3, constrain(m3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH));
}

inline float mapSym(int16_t pwm, float outMin, float outMax)
{
  return (pwm - 1500) * (outMax - outMin) / 500.0f;
}

void pollRadio()
{
  if (!nrf_data_ready)
    return;

  // clear flag as quickly as possible
  portENTER_CRITICAL(&spinlock);
  nrf_data_ready = false;
  portEXIT_CRITICAL(&spinlock);

  const int len = radio.getPacketLength();
  if (radio.readData(nrf_data, len) == RADIOLIB_ERR_NONE)
  {
    unpackData(nrf_data);    // updates inputRoll/…
    lastRxMicros = micros(); // reset fail-safe timer
  }
  radio.startReceive(); // back to RX mode
}