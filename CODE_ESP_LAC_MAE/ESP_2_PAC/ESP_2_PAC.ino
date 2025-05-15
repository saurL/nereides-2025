//ID envoie sonde de temp : 0X304
#include <ESP32-TWAI-CAN.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
// Pins CAN + Sondes temp 
#define CAN_TX 22
#define CAN_RX 21
#define ONE_WIRE_BUS 13

// Initialiser le bus OneWire et les capteurs de température
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
CanFrame rxFrame;

//structure qui stocke données des sondes de temp
struct TemperatureData {
    float tempSensor1 = 0.0f;
    float tempSensor2 = 0.0f;
    float tempSensor3 = 0.0f;
    unsigned long lastUpdate = 0;
};

TemperatureData tempData;
// Définir les adresses des sondes de température 
DeviceAddress sensor1Address = {0x28, 0xAA, 0xA1, 0x9D, 0x0F, 0x00, 0x00, 0x50}; //courte sonde  
DeviceAddress sensor2Address = {0x28, 0x1A, 0x85, 0x46, 0xD4, 0xC9, 0x6E, 0x81}; //longue sonde avec || dessiné 
DeviceAddress sensor3Address = {0x28, 0xDA, 0xB6, 0x46, 0xD4, 0x07, 0x41, 0xDE}; //longue sonde sans ||

void setup() {
    // Setup serial pour le débogage
    Serial.begin(115200);
    while (!Serial); // Attendre que le port série soit prêt

    
    ESP32Can.setPins(CAN_TX, CAN_RX);
    
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    if(ESP32Can.begin()) {
        Serial.println("CAN bus démarré avec succès!");
    } else {
        Serial.println("Échec du démarrage du bus CAN!");
        while(1); // Bloquer si le CAN ne démarre pas
    }
    // Initialiser les capteurs de température
    sensors.begin();
}

//Envoie par CAN des données des sondes de température
void sendTemperatureData() {
    CanFrame txFrame;

    // Trame 1 : Températures des trois sondes (6 octets)
    txFrame.identifier = 0x304;
    txFrame.data_length_code = 8;  // Taille maximale de la trame CAN
    int16_t temp1 = (int16_t)(tempData.tempSensor1 * 100); 
    int16_t temp2 = (int16_t)(tempData.tempSensor2 * 100); 
    int16_t temp3 = (int16_t)(tempData.tempSensor3 * 100); 

    // Remplir les 6 premiers octets avec les températures
    txFrame.data[0] = temp1 >> 8;   // Température 1 - Octet 1
    txFrame.data[1] = temp1 & 0xFF; // Température 1 - Octet 2
    txFrame.data[2] = temp2 >> 8;   // Température 2 - Octet 3
    txFrame.data[3] = temp2 & 0xFF; // Température 2 - Octet 4
    txFrame.data[4] = temp3 >> 8;   // Température 3 - Octet 5
    txFrame.data[5] = temp3 & 0xFF; // Température 3 - Octet 6

    // Laisser les 2 derniers octets vides ou y mettre des informations supplémentaires
    txFrame.data[6] = 0;   
    txFrame.data[7] = 0;   

    if(!ESP32Can.writeFrame(txFrame)) {
        Serial.println("Erreur envoi trame 0x304");
    }
}

//Lecture des données des sondes de temp
void readTemperatureSensors() {
    // Demander la lecture des capteurs
    sensors.requestTemperaturesByAddress(sensor1Address); 
    sensors.requestTemperaturesByAddress(sensor2Address); 
    sensors.requestTemperaturesByAddress(sensor3Address); 

    // Lire les températures des sondes et les stocker dans la structure
    tempData.tempSensor1 = sensors.getTempC(sensor1Address); 
    tempData.tempSensor2 = sensors.getTempC(sensor2Address); 
    tempData.tempSensor3 = sensors.getTempC(sensor3Address); 

    // Mise à jour du timestamp
    tempData.lastUpdate = millis();
}
void printAllData() {    
    Serial.println("\n=== DONNÉES DES Sondes de temp===");
    Serial.printf("Timestamp: %lu ms\n", tempData.lastUpdate);
    Serial.println("\n=== TEMPÉRATURES ===");
    Serial.printf("Sonde 1: %.2f °C\n", tempData.tempSensor1);
    Serial.printf("Sonde 2: %.2f °C\n", tempData.tempSensor2);
    Serial.printf("Sonde 3: %.2f °C\n", tempData.tempSensor3);
    Serial.println("========================\n");
}

void loop() {
    // Lire les trames CAN avec un timeout de 1000ms
      readTemperatureSensors();
    // Envoyer les données via CAN
      sendTemperatureData();
    delay(10); // Petit délai pour éviter de surcharger le CPU
}