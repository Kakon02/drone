#include "config.h"

// Interrupt configuration for MPU9250 and nRF24L01
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;

void initSharedSPI()
{
    SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
    pinMode(MPU_CS, OUTPUT);
    pinMode(NRF_CS, OUTPUT);
    digitalWrite(MPU_CS, HIGH);
    digitalWrite(NRF_CS, HIGH);
}

void boardSetup()
{
    initSharedSPI();
    gpio_install_isr_service(0);

    // Initialize MPU9250 interrupt
    pinMode(MPU_INT, INPUT);
    gpio_set_intr_type((gpio_num_t)MPU_INT, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)MPU_INT, imuISR, NULL);
    Serial.println(F("MPU9250 interrupt initialized"));

    // Initialize nRF24L01 interrupt
    pinMode(NRF_INT, INPUT);
    gpio_set_intr_type((gpio_num_t)NRF_INT, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)NRF_INT, nrfISR, NULL);
    Serial.println(F("nRF24L01 interrupt initialized"));
}