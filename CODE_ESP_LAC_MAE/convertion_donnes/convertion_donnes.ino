//pour recevoir les temperatures des sondes de la batterie 24v et les interpreter en CAN
const int numThermistors = 10;  //nombre de sondes
float temperature[numThermistors] = {}; //stocker les valeurs des 10 sondes dans une liste


#include <ESP32-TWAI-CAN.hpp>

// Broches du CAN transiever
#define CAN_TX 5
#define CAN_RX 4

CanFrame rxFrame;


void decodeMessageThermistances(const CanFrame &frame) { //decoder la trame et mettre a jour la liste
      // Extraction de la température en centi-degrés (int16_t signé)
      int16_t temp = ((int16_t)rxFrame.data[0] << 8) | rxFrame.data[1];
       temperature[0] = temp / 100.0; // => temperature !!!!!!  

    Serial.printf("Température reçue : %.2f °C\n", temperature[0]);
}


void setup() {
    // Setup serial for debbuging.
    Serial.begin(115200);

    // Set pins
    ESP32Can.setPins(CAN_TX, CAN_RX);

    // You can set custom size for the queues - those are default
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
    // but you can easily convert it from numerical value using .convertSpeed()
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    // You can also just use .begin()..
    if (ESP32Can.begin()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }

}

void loop() {
    // You can set custom timeout, default is 1000
    if (ESP32Can.readFrame(rxFrame, 1000)) {
        // Comment out if too many frames
        Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
        if (0x600 <= rxFrame.identifier < 0x700) { //si la trame a pour identifiant de centaine 6 (600 à 699), ça correspond aux sondes de temp de la batterie 24v
            decodeMessageThermistances(rxFrame);
        }
    }
}