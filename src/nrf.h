#include <Arduino.h>
#include <RadioLib.h>
#include "config.h"

extern nRF24 radio;                                             // nRF24L01 radio object
extern uint8_t nrf_data[8];                                     // Buffer for received data
extern uint16_t inputRoll, inputPitch, inputYaw, inputThrottle; // Unpacked data
extern volatile bool nrf_data_ready;

void IRAM_ATTR nrfISR(void *arg);
void setupNRF24L01();
void unpackData(const uint8_t *data);
void pack16BE(uint8_t *dst, int16_t v); // Big-Endian
int16_t unpack16BE(const uint8_t *src);
void testNRF24L01();