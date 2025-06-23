// === ISR ===
#include "nrf.h"

nRF24 radio = new Module(NRF_CE, NRF_INT, NRF_CS);
volatile bool nrf_data_ready = false;
uint8_t nrf_data[8]; // Buffer for received data
uint16_t inputRoll, inputPitch, inputYaw, inputThrottle;

inline void pack16BE(uint8_t *dst, int16_t v) // Big-Endian
{
    dst[0] = v >> 8;
    dst[1] = v & 0xFF;
}

inline int16_t unpack16BE(const uint8_t *src)
{
    return (int16_t)((src[0] << 8) | src[1]);
}

void unpackData(const uint8_t *data)
{
    inputRoll = unpack16BE(data);
    inputPitch = unpack16BE(data + 2);
    inputYaw = unpack16BE(data + 4);
    inputThrottle = unpack16BE(data + 6);
}

void IRAM_ATTR nrfISR(void *arg)
{
    portENTER_CRITICAL_ISR(&spinlock);
    nrf_data_ready = true;
    portEXIT_CRITICAL_ISR(&spinlock);
}

void setupNRF24L01()
{
    int state = radio.begin(2400, 250, -12, 5);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
        {
            delay(10);
        }
    }
    uint8_t addr[] = {0x01, 0x23, 0x45, 0x67, 0x89};
    state = radio.setReceivePipe(0, addr);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("Receive pipe set successfully!"));
    }
    else
    {
        Serial.print(F("Failed to set receive pipe, code "));
        Serial.println(state);
        while (true)
        {
            delay(10);
        }
    }
    // Set radio interrupt pin manually at config.cpp
    // Uncomment the line below if not working
    // radio.setPacketReceivedAction(nrfISR);
    Serial.print(F("[nRF24] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
        {
            delay(10);
        }
    }
}

void testNRF24L01()
{
    Serial.println(F("[nRF24] Testing ... "));
    if (nrf_data_ready)
    {
        nrf_data_ready = false;
        int numBytes = radio.getPacketLength();
        int state = radio.readData(nrf_data, numBytes);
        unpackData(nrf_data);

        if (state == RADIOLIB_ERR_NONE)
        {
            Serial.println(F("[nRF24] Received packet!"));
            Serial.print(F("[nRF24] Data:\t\t"));
            Serial.print(inputRoll);
            Serial.print(F(", "));
            Serial.print(inputPitch);
            Serial.print(F(", "));
            Serial.print(inputYaw);
            Serial.print(F(", "));
            Serial.println(inputThrottle);
        }
        else
        {
            // some other error occurred
            Serial.print(F("[nRF24] Failed, code "));
            Serial.println(state);
        }
        // put module back to listen mode
        radio.startReceive();
    }
}