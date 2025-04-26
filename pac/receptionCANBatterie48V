#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 22
#define CAN_RX 21

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
void handle0x351(const CanFrame& frame);
void handle0x355(const CanFrame& frame);
void handle0x356(const CanFrame& frame);
void handle0x359(const CanFrame& frame);
void handle0x35C(const CanFrame& frame);

void setup() {
    Serial.begin(115200);
    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
    if(ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }
}
void handle0x351(const CanFrame& frame) {
    battery.chargeVoltage = (frame.data[0] | (frame.data[1] << 8)) / 10.0;
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

void loop() {
    if (ESP32Can.readFrame(rxFrame, 1000)) {
        if ((rxFrame.identifier >= 0x370 && rxFrame.identifier <= 0x379) ||
            rxFrame.identifier == 0x35A || 
            rxFrame.identifier == 0x35E || 
            rxFrame.identifier == 0x35F) {
            return;
        }

        Serial.printf("Trame reçue : ID = 0x%03X, Données : ", rxFrame.identifier);
        for (int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf("%02X ", rxFrame.data[i]);
        }
        Serial.println();

        switch (rxFrame.identifier) {
    case 0x351:
        handle0x351(rxFrame);
        break;
    case 0x355:
        handle0x355(rxFrame);
        break;
    case 0x356:
        handle0x356(rxFrame);
        break;
    case 0x359:
        handle0x359(rxFrame);
        break;
    case 0x35C:
        handle0x35C(rxFrame);
        break;
}       
        Serial.println("== Données batterie mises à jour ==");
        Serial.printf("Tension charge: %.1f V, Lim. charge: %.1f A, Lim. décharge: %.1f A\n", battery.chargeVoltage, battery.chargeCurrentLimit, battery.dischargeCurrentLimit);
        Serial.printf("SOC: %d %%, SOH: %d %%\n", battery.soc, battery.soh);
        Serial.printf("Tension module: %.2f V, Courant total: %.1f A, Temp. moy.: %.1f °C\n", battery.moduleVoltage, battery.totalCurrent, battery.avgTemperature);
        Serial.printf("Prot1: 0x%02X, Prot2: 0x%02X, Alarm1: 0x%02X, Alarm2: 0x%02X, Modules: %d\n", battery.prot1, battery.prot2, battery.alarm1, battery.alarm2, battery.moduleCount);
        Serial.printf("Flags: 0x%02X\n", battery.flags);
        Serial.println("====================================");
    }
}
