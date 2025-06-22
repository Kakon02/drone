#include <Arduino.h>
#include <NMEAGPS.h>
#include <GPSport.h> // Defines gpsPort
#include <HardwareSerial.h>
#include "esp_adc_cal.h"
#include "nrf.h"
#include "PID.h"
#include "imu.h"
#include "ESCs.h"
#include "config.h"

esp_adc_cal_characteristics_t adc_chars; // calibration data holder

NMEAGPS gps;                  // Parser
gps_fix fix;                  // Data structure
HardwareSerial GPS_Serial(1); // Use UART1
void setupAdc();              // ADC setup function
uint32_t readVoltage();

void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Configure UART1 for GPS: RX=17, TX=16 (adjust pins as needed)
  GPS_Serial.begin(115200, SERIAL_8N1, 17, 16);
  gpsPort.begin(115200); // gpsPort is a wrapper defined in GPSport.h
  setupAdc();
}

void loop()
{
  while (gps.available(gpsPort))
  {
    fix = gps.read();

    if (fix.valid.location)
    {
      Serial.print("Latitude: ");
      Serial.print(fix.latitude(), 6);
      Serial.print(" | Longitude: ");
      Serial.println(fix.longitude(), 6);
    }

    if (fix.valid.date && fix.valid.time)
    {
      Serial.print("Date: ");
      Serial.print(fix.dateTime.day);
      Serial.print("/");
      Serial.print(fix.dateTime.month);
      Serial.print("/");
      Serial.print(fix.dateTime.year);
      Serial.print(" Time: ");
      Serial.print(fix.dateTime.hours);
      Serial.print(":");
      Serial.print(fix.dateTime.minutes);
      Serial.print(":");
      Serial.println(fix.dateTime.seconds);
    }
    if (fix.valid.satellites)
    {
      Serial.print("Satellites: ");
      Serial.println(fix.satellites);
    }
  }
}

void setupAdc()
{
  const adc_unit_t unit = ADC_UNIT_1;
  const adc_atten_t atten = ADC_ATTEN_DB_11;
  const uint32_t defaultVref = 1100; // mV

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, atten); // GPIO1 = ADC1_CH0

  esp_adc_cal_value_t calType = esp_adc_cal_characterize(
      unit, atten, ADC_WIDTH_BIT_12, defaultVref, &adc_chars);

  Serial.printf("ADC calibration: %s\n",
                calType == ESP_ADC_CAL_VAL_EFUSE_VREF ? "eFuse Vref" : calType == ESP_ADC_CAL_VAL_EFUSE_TP ? "eFuse Two-Point"
                                                                                                           : "Default 1.1 V");
}

uint32_t readVoltage()
{
  uint32_t raw = adc1_get_raw(ADC1_CHANNEL_0);
  return esp_adc_cal_raw_to_voltage(raw, &adc_chars);
}