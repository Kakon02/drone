// === ISR ===
#include "nrf.h"

volatile bool nrf_data_ready = false;

void IRAM_ATTR nrfISR(void *arg)
{
    portENTER_CRITICAL_ISR(&spinlock);
    nrf_data_ready = true;
    portEXIT_CRITICAL_ISR(&spinlock);
}