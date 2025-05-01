#include <Wire.h>
#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 5
#define CAN_RX 4
#define SLAVE_ADDRESS 0x08  // Adresse I2C de l'esclave

CanFrame rxFrame;
//pas oublier de mettre resistance 120 ohm pour le can de la batterie 
// Variables complètes à transmettre
struct BatteryRaw {
  float chargeVoltage = 0;
  float chargeCurrentLimit = 0;
  float dischargeCurrentLimit = 0;
  uint16_t soc = 0;
  uint16_t soh = 0;
  float moduleVoltage = 0;
  float totalCurrent = 0;
  float avgTemperature = 0;
  uint8_t prot1 = 0;
  uint8_t prot2 = 0;
  uint8_t alarm1 = 0;
  uint8_t alarm2 = 0;
  uint8_t moduleCount = 0;
  uint8_t flags = 0;
} battery;

void setup() {
  Serial.begin(115200);

  // CAN
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  if (ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }

  // I2C
  Wire.begin();  // Maître
  Wire.setClock(100000);  // 100 kHz
}

// Handlers
void handle0x355(const CanFrame& frame) {
  battery.soc = frame.data[0] | (frame.data[1] << 8);
  battery.soh = frame.data[2] | (frame.data[3] << 8);
}
void handle0x356(const CanFrame& frame) {
  battery.moduleVoltage = (int16_t)(frame.data[0] | (frame.data[1] << 8)) / 100.0;
  battery.totalCurrent = (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
  battery.avgTemperature = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
}
void handle0x351(const CanFrame& frame) {
  battery.chargeVoltage = (frame.data[0] | (frame.data[1] << 8)) / 10.0;
  battery.chargeCurrentLimit = (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
  battery.dischargeCurrentLimit = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
}
void handle0x359(const CanFrame& frame) {
  battery.prot1 = frame.data[0];
  battery.prot2 = frame.data[1];
  battery.alarm1 = frame.data[2];
  battery.alarm2 = frame.data[3];
  battery.moduleCount = frame.data[4];
}

// Envoi de la structure complète
void sendBatteryData() {
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write((uint8_t*)&battery, sizeof(battery));
  Wire.endTransmission();

  Serial.println(">> Structure complète BatteryRaw envoyée via I2C");
}

void loop() {
  if (ESP32Can.readFrame(rxFrame, 1000)) {
    switch (rxFrame.identifier) {
      case 0x355:
        handle0x355(rxFrame);
        break;
      case 0x356:
        handle0x356(rxFrame);
        break;
      case 0x351:
        handle0x351(rxFrame);
        break;
      case 0x359:
        handle0x359(rxFrame);
        break;
    }
    sendBatteryData();  // Envoie à l'esclave
  }
}
