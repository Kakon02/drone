#include "config.h"

// Interrupt configuration for MPU9250 and nRF24L01
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t *timer = NULL;