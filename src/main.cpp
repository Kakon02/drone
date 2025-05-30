#include <Arduino.h>
#include <NMEAGPS.h>
  // Defines gpsPort

NMEAGPS gps;            // Parser
gps_fix fix;            // Data structure
HardwareSerial GPS_Serial(1);  // Use UART1

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Configure UART1 for GPS: GPIO RX=18, TX=17 (adjust pins as needed)
  GPS_Serial.begin(115200, SERIAL_8N1, 17, 16);
}

void loop() {
  while (GPS_Serial.available()) {
    if (gps.decode(GPS_Serial.read())) {
      fix = gps.read();

      // Always print raw status
      Serial.print("Time: ");
      Serial.print(fix.dateTime.hours);
      Serial.print(":"); Serial.print(fix.dateTime.minutes);
      Serial.print(":"); Serial.println(fix.dateTime.seconds);

      Serial.print("Fix status: "); Serial.println(fix.status); // 0 = no fix

      Serial.print("Satellites: "); Serial.println(fix.satellites);

      if (fix.valid.location) {
        Serial.print("Latitude: "); Serial.println(fix.latitude(), 6);
        Serial.print("Longitude: "); Serial.println(fix.longitude(), 6);
      } else {
        Serial.println("Location not valid yet.");
      }

      Serial.println();
    }
  }
}