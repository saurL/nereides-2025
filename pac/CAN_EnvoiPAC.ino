#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX  5  // Broche CAN TX (GPIO5 par défaut)
#define CAN_RX  4  // Broche CAN RX (GPIO4 par défaut)

void setup() {
    Serial.begin(115200);
    
    // Initialisation du bus CAN à 500 kbps
    ESP32Can.setPins(CAN_TX, CAN_RX);
    if (ESP32Can.begin(ESP32Can.convertSpeed(250))) {
        Serial.println("CAN initialisé !");
    } else {
        Serial.println("Échec de l'initialisation CAN !");
        while (1);  // Bloquer si le CAN échoue
    }
}

void loop() {
    // Trame pour démarrer la pile AIRCELL® (ID étendu 29 bits)
    CanFrame startFrame;
    startFrame.identifier = 0x2FC1001;  // ID de la trame
    startFrame.extd = 1;               // 1 = ID étendu (29 bits)
    startFrame.data_length_code = 3;   // 3 octets de données
    startFrame.data[0] = 0;            // Octet 0 : Pas d'arrêt d'urgence
    startFrame.data[1] = 0;            // Octet 1 : Pas d'arrêt normal
    startFrame.data[2] = 1;            // Octet 2 = 1 → Démarrage

    // Envoi de la trame
    if (ESP32Can.writeFrame(startFrame)) {
        Serial.println("Trame de démarrage envoyée !");
    } else {
        Serial.println("Échec de l'envoi !");
    }

    delay(1000);  // Envoi toutes les secondes (pour tests)
}