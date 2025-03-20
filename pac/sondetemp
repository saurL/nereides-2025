/*
  F1ATB
  GNU General Public License v3.0
*/

// Libraries
#include <OneWire.h>
#include <DallasTemperature.h>

// Capteur DS18B20
OneWire oneWire(19);
DallasTemperature ds(&oneWire);

// SETUP
void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  ds.begin();
}

// LOOP
void loop() {
  ds.requestTemperatures();
  float t = ds.getTempCByIndex(0);

  Serial.print("Température : ");
  Serial.print(t);
  Serial.println(" °C");

  delay(1000);
}
