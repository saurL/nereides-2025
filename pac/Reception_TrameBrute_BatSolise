//Pas besoin de fil de masse 
//Pas besoin de resistance 
//Fil blanc = CAN H 
// Fil jaune = Can L 

#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 22
#define CAN_RX 21

void setup() {
  Serial.begin(115200);
  delay(1000);

  ESP32Can.setPins(CAN_TX, CAN_RX);
  if (!ESP32Can.begin(ESP32Can.convertSpeed(250))) {
    Serial.println("Erreur init CAN");
    while (1) delay(1000);
  }

  Serial.println("CAN OK 250 kbps");
}

void sendQueryFrame() {
  CanFrame query;
  query.identifier = 0x400FF80; // Trame de requête Daly
  query.extd = 1;               // Format étendu
  query.data_length_code = 8;
  for (int i = 0; i < 8; i++) query.data[i] = 0x00;

  if (ESP32Can.writeFrame(query)) {
    Serial.println("-> Trame requête 0x400FF80 envoyée");
  } else {
    Serial.println("Erreur envoi requête");
  }
}

void loop() {
  static unsigned long lastQuery = 0;
  if (millis() - lastQuery > 2000) {
    sendQueryFrame();
    lastQuery = millis();
  }

  CanFrame frame;
  if (ESP32Can.readFrame(frame, 0)) {
    Serial.printf("Reçu 0x%08X (%d): ", frame.identifier, frame.data_length_code);
    for (int i = 0; i < frame.data_length_code; i++) {
      Serial.printf("%02X ", frame.data[i]);
    }
    Serial.println();
  }

  delay(10);
}
