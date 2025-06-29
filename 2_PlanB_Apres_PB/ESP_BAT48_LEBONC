//Code qui récupère en CAN les données de la batterie (500kbs + résistance à mettre) et 
// qui envoie les données en I2C à l'adresse 0x08 sur l'esp 3 CM 
#include <ESP32-TWAI-CAN.hpp>
#include <Wire.h>  // Pour I2C

#define CAN_TX 4
#define CAN_RX 5
#define I2C_SLAVE_ADDRESS 0x08  // Adresse de l'esclave I2C

CanFrame rxFrame;

struct BatteryData {
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
};

BatteryData battery;

void handle0x351(const CanFrame& frame) {
    battery.chargeVoltage = (frame.data[0] | (frame.data[1] << 8)) / 10.0;
    Serial.println(battery.chargeVoltage);
    battery.chargeCurrentLimit = (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
    battery.dischargeCurrentLimit = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
}

void handle0x355(const CanFrame& frame) {
    battery.soc = frame.data[0] | (frame.data[1] << 8);
    battery.soh = frame.data[2] | (frame.data[3] << 8);
}

void handle0x356(const CanFrame& frame) {
    battery.moduleVoltage = (int16_t)(frame.data[0] | (frame.data[1] << 8)) / 100.0;
    battery.totalCurrent = (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
    battery.avgTemperature = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
}

void handle0x359(const CanFrame& frame) {
    battery.prot1 = frame.data[0];
    battery.prot2 = frame.data[1];
    battery.alarm1 = frame.data[2];
    battery.alarm2 = frame.data[3];
    battery.moduleCount = frame.data[4];
}

void handle0x35C(const CanFrame& frame) {
    battery.flags = frame.data[0];
}

void setup() {
    Serial.begin(115200);

    // CAN
    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    if (ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }

    // I2C en maître
    Wire.begin();  // bon pin!
}

void loop() {
    if (ESP32Can.readFrame(rxFrame, 1000)) {
        if ((rxFrame.identifier >= 0x370 && rxFrame.identifier <= 0x379) ||
            rxFrame.identifier == 0x35A || 
            rxFrame.identifier == 0x35E || 
            rxFrame.identifier == 0x35F) {
            return;
        }

        // Traitement de la trame reçue
        switch (rxFrame.identifier) {
            case 0x351: handle0x351(rxFrame); break;
            case 0x355: handle0x355(rxFrame); break;
            case 0x356: handle0x356(rxFrame); break;
            case 0x359: handle0x359(rxFrame); break;
            case 0x35C: handle0x35C(rxFrame); break;
        }

        // Affichage
        Serial.println("== Données batterie mises à jour ==");
        Serial.printf("Tension charge: %.1f V, Lim. charge: %.1f A, Lim. décharge: %.1f A\n", battery.chargeVoltage, battery.chargeCurrentLimit, battery.dischargeCurrentLimit);
        Serial.printf("SOC: %d %%, SOH: %d %%\n", battery.soc, battery.soh);
        Serial.printf("Tension module: %.2f V, Courant total: %.1f A, Temp. moy.: %.1f °C\n", battery.moduleVoltage, battery.totalCurrent, battery.avgTemperature);
        Serial.printf("Prot1: 0x%02X, Prot2: 0x%02X, Alarm1: 0x%02X, Alarm2: 0x%02X, Modules: %d\n", battery.prot1, battery.prot2, battery.alarm1, battery.alarm2, battery.moduleCount);
        Serial.printf("Flags: 0x%02X\n", battery.flags);
        Serial.println("====================================");

        // Envoi I2C des données vers l'esclave
        Wire.beginTransmission(I2C_SLAVE_ADDRESS);
        Wire.write((uint8_t*)&battery, sizeof(BatteryData));
        Wire.endTransmission();
    }
}
