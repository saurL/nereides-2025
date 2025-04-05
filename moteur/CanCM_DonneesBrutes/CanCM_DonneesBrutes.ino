#include <ESP32-TWAI-CAN.hpp>

// Default for ESP32
#define CAN_TX      5
#define CAN_RX      4

CanFrame rxFrame;

void setup() {
    // Setup serial for debugging.
    Serial.begin(115200);

    // Set pins
    ESP32Can.setPins(CAN_TX, CAN_RX);
    
    // Set queue sizes
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    // Set CAN speed to 250 Kbps (as per the controller's documentation)
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    // Initialize CAN bus
    if(ESP32Can.begin()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }
}

void loop() {
    if (ESP32Can.readFrame(rxFrame, 1000)) {
        // Afficher l'identifiant de la trame (29 bits en hexadécimal)
        Serial.printf("\nReceived frame: ID=0x%08X, DLC=%d, Data:", rxFrame.identifier, rxFrame.data_length_code);
        
        // Afficher les données brutes en hexadécimal
        for (int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf(" 0x%02X", rxFrame.data[i]);
        }
        Serial.println(); // Nouvelle ligne pour la prochaine trame
    } else {
        Serial.println("No frame received within timeout period.");
    }
}