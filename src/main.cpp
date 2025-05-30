#include <Arduino.h>
#include <NMEAGPS.h>
#include <GPSport.h>  // Defines gpsPort

NMEAGPS gps;            // Parser
gps_fix fix;            // Data structure
HardwareSerial GPS_Serial(1);  // Use UART1

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Configure UART1 for GPS: RX=16, TX=17 (adjust pins as needed)
  GPS_Serial.begin(115200, SERIAL_8N1, 17, 16);
  gpsPort.begin(115200);  // gpsPort is a wrapper defined in GPSport.h
  Serial.println("NeoGPS + ESP32-S3 started");
}

void loop() {
  while (gps.available(gpsPort)) {
    fix = gps.read();

    if (fix.valid.location) {
      Serial.print("Latitude: ");
      Serial.print(fix.latitude(), 6);
      Serial.print(" | Longitude: ");
      Serial.println(fix.longitude(), 6);
    }

    if (fix.valid.date && fix.valid.time) {
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

    if (fix.valid.satellites) {
      Serial.print("Satellites: ");
      Serial.println(fix.satellites);
    }
  }
}