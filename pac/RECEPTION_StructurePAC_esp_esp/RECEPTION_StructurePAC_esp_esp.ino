#include <ESP32-TWAI-CAN.hpp>
#include <Arduino.h>

#define CAN_RX 4
#define CAN_TX 5

CanFrame rxFrame;

// Structure similaire à celle envoyée par l'esp de la boite PAC
struct FuelCellData {
    uint8_t systemState;
    uint8_t systemFlags;
    uint8_t h2Consumption;
    uint8_t avgTemp;

    float current;
    float voltage;
    float power;

    uint16_t errorFlags;
    uint16_t runtime;
    uint16_t energy;

    uint32_t totalRuntime;
    uint32_t totalEnergy;

    float h2Pressure;
    float auxVoltage;

    unsigned long lastUpdate;
};

FuelCellData receivedData;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    if (ESP32Can.begin()) {
        Serial.println("CAN récepteur démarré !");
    } else {
        Serial.println("Erreur CAN !");
        while (1);
    }
}

void loop() {
    // Vérifie s’il y a une trame disponible
    if (ESP32Can.readFrame(rxFrame)) {
        switch (rxFrame.identifier) {
            case 0x300:
                receivedData.systemState = rxFrame.data[0];
                receivedData.systemFlags = rxFrame.data[1];
                receivedData.h2Consumption = rxFrame.data[2];
                receivedData.avgTemp = rxFrame.data[3];
                break;

            case 0x301:
                receivedData.current = ((rxFrame.data[0] << 8) | rxFrame.data[1]) / 100.0;
                receivedData.voltage = ((rxFrame.data[2] << 8) | rxFrame.data[3]) / 100.0;
                receivedData.power = receivedData.current * receivedData.voltage;
                break;

            case 0x302:
                receivedData.h2Pressure = ((rxFrame.data[0] << 8) | rxFrame.data[1]) / 100.0;
                receivedData.auxVoltage = ((rxFrame.data[2] << 8) | rxFrame.data[3]) / 100.0;
                break;

            case 0x303:
                receivedData.runtime = (rxFrame.data[0] << 8) | rxFrame.data[1];
                receivedData.energy = (rxFrame.data[2] << 8) | rxFrame.data[3];
                receivedData.lastUpdate = millis();

                // Affichage complet à chaque réception complète
                //ne sera plus nécessaire une fois qu'on aura réussi a faire la communication UART
                Serial.println("==== Données PAC reçues ====");
                Serial.print("État système: "); Serial.println(receivedData.systemState);
                Serial.print("Flags: "); Serial.println(receivedData.systemFlags);
                Serial.print("Conso H2 (mg/s): "); Serial.println(receivedData.h2Consumption);
                Serial.print("Température moy. (°C): "); Serial.println(receivedData.avgTemp);
                Serial.print("Courant (A): "); Serial.println(receivedData.current);
                Serial.print("Tension (V): "); Serial.println(receivedData.voltage);
                Serial.print("Puissance (W): "); Serial.println(receivedData.power);
                Serial.print("Pression H2 (bar): "); Serial.println(receivedData.h2Pressure);
                Serial.print("Tension auxiliaire (V): "); Serial.println(receivedData.auxVoltage);
                Serial.print("Runtime (min): "); Serial.println(receivedData.runtime);
                Serial.print("Énergie (Wh): "); Serial.println(receivedData.energy);
                Serial.println("============================");
                break;
        }
    }
}
