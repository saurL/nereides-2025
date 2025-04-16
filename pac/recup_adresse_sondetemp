//adresse de notre sonde : 0x28, 0xAA, 0xA1, 0x9D, 0x0F, 0x00, 0x00, 0x50
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin où les capteurs sont connectés
#define ONE_WIRE_BUS 23

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  Serial.println("Scan des capteurs DS18B20...");

  sensors.begin();

  int numberOfDevices = sensors.getDeviceCount();
  Serial.print("Nombre de capteurs trouvés : ");
  Serial.println(numberOfDevices);

  DeviceAddress address;

  for (int i = 0; i < numberOfDevices; i++) {
    if (sensors.getAddress(address, i)) {
      Serial.print("Adresse du capteur ");
      Serial.print(i);
      Serial.print(" : ");
      printAddress(address);
    } else {
      Serial.print("Impossible de lire l'adresse du capteur à l'index ");
      Serial.println(i);
    }
  }
}

void loop() {
  // rien ici
}

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println();
}
