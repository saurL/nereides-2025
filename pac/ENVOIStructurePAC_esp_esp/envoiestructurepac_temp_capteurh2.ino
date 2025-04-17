#include <ESP32-TWAI-CAN.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
// Pins CAN (à adapter selon votre configuration)
#define CAN_TX 21
#define CAN_RX 22
#define ONE_WIRE_BUS 23

// Pins UART pour les capteurs H2
#define H2_PAC_RX 16
#define H2_PAC_TX 17
#define H2_BOTTLES_RX 18
#define H2_BOTTLES_TX 19

// Initialiser le bus OneWire et les capteurs de température
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
CanFrame rxFrame;

// Structure pour stocker toutes les données de la pile à combustible
struct FuelCellData {
    // Système
    uint8_t systemState = 0;
    String systemStateStr = "";
    uint8_t systemFlags = 0;
    uint8_t h2Consumption = 0; // mg/sec
    uint8_t avgTemp = 0;       // °C
    
    // Puissance
    float current = 0;         // A
    float voltage = 0;         // V
    float power = 0;           // W
    
    // Erreurs
    uint16_t errorFlags = 0;
    
    // Runtime
    uint16_t runtime = 0;      // minutes
    uint16_t energy = 0;       // Wh
    
    // Runtime total
    uint32_t totalRuntime = 0; // minutes
    uint32_t totalEnergy = 0;  // Wh
    
    // Pression H2
    float h2Pressure = 0;      // bar
    
    // Tension auxiliaire
    float auxVoltage = 0;      // V
    
    // Timestamp de la dernière mise à jour
    unsigned long lastUpdate = 0;
};

FuelCellData fuelCellData;

//structure qui stocke données des sondes de temp
struct TemperatureData {
    float tempSensor1 = 0.0f;
    float tempSensor2 = 0.0f;
    float tempSensor3 = 0.0f;
    unsigned long lastUpdate = 0;
};

// Structure pour stocker les données des capteurs H2
struct H2SensorData {
    uint16_t pacValue = 0;      // valeur en ppm du capteur dans le boîtier PAC
    uint16_t bottlesValue = 0;  // valeur en ppm du capteur dans le boîtier bouteilles
    bool pacSensorOk = false;   // état du capteur PAC
    bool bottlesSensorOk = false; // état du capteur bouteilles
    unsigned long lastUpdate = 0;
};

TemperatureData tempData;
H2SensorData h2Data;

// Définir les adresses des sondes de température (à remplir avec les adresses spécifiques de tes sondes)
DeviceAddress sensor1Address = {0x28, 0xAA, 0xA1, 0x9D, 0x0F, 0x00, 0x00, 0x50}; //vraie adresse de la sonde qu'on a deja 
DeviceAddress sensor2Address = {}; 
DeviceAddress sensor3Address = {}; 

// Ports série pour les capteurs H2
HardwareSerial h2PacSerial(1); // UART1 pour le capteur dans le boîtier PAC
HardwareSerial h2BottlesSerial(2); // UART2 pour le capteur dans le boîtier bouteilles

void setup() {
    // Setup serial pour le débogage
    Serial.begin(115200);
    while (!Serial); // Attendre que le port série soit prêt

    // Configuration des UART pour les capteurs H2
    h2PacSerial.begin(9600, SERIAL_8N1, H2_PAC_RX, H2_PAC_TX);
    h2BottlesSerial.begin(9600, SERIAL_8N1, H2_BOTTLES_RX, H2_BOTTLES_TX);

    // Configurer les pins CAN
    ESP32Can.setPins(CAN_TX, CAN_RX);
    
    // Taille des queues (par défaut)
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    // Vitesse du bus CAN (250 kbit/s selon documentation)
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    // Démarrer le bus CAN
    if(ESP32Can.begin()) {
        Serial.println("CAN bus démarré avec succès!");
    } else {
        Serial.println("Échec du démarrage du bus CAN!");
        while(1); // Bloquer si le CAN ne démarre pas
    }
    // Initialiser les capteurs de température
    sensors.begin();
}

void sendTemperatureData() {
    CanFrame txFrame;

    // Trame 1 : Températures des trois sondes (6 octets)
    txFrame.identifier = 0x304;
    txFrame.data_length_code = 8;  // Taille maximale de la trame CAN
    int16_t temp1 = (int16_t)(tempData.tempSensor1 * 100); // Conversion en centi-degrés
    int16_t temp2 = (int16_t)(tempData.tempSensor2 * 100); // Conversion en centi-degrés
    int16_t temp3 = (int16_t)(tempData.tempSensor3 * 100); // Conversion en centi-degrés

    // Remplir les 6 premiers octets avec les températures
    txFrame.data[0] = temp1 >> 8;   // Température 1 - Octet 1
    txFrame.data[1] = temp1 & 0xFF; // Température 1 - Octet 2
    txFrame.data[2] = temp2 >> 8;   // Température 2 - Octet 3
    txFrame.data[3] = temp2 & 0xFF; // Température 2 - Octet 4
    txFrame.data[4] = temp3 >> 8;   // Température 3 - Octet 5
    txFrame.data[5] = temp3 & 0xFF; // Température 3 - Octet 6

    // Laisser les 2 derniers octets vides ou y mettre des informations supplémentaires
    txFrame.data[6] = 0;   // Octet 7
    txFrame.data[7] = 0;   // Octet 8

    if(!ESP32Can.writeFrame(txFrame)) {
        Serial.println("Erreur envoi trame 0x304");
    }
}

// Fonction pour envoyer les données des capteurs H2 via CAN
void sendH2SensorData() {
    CanFrame txFrame;

    // Trame ID 0x035 pour les capteurs H2
    txFrame.identifier = 0x035;
    txFrame.data_length_code = 8;  // 8 octets de données
    
    // Conversion des valeurs en ppm (uint16_t)
    uint16_t pacValue = h2Data.pacValue;
    uint16_t bottlesValue = h2Data.bottlesValue;
    
    // Remplir les données dans la trame
    txFrame.data[0] = pacValue >> 8;      // H2 PAC - Octet haut
    txFrame.data[1] = pacValue & 0xFF;    // H2 PAC - Octet bas
    txFrame.data[2] = bottlesValue >> 8;  // H2 Bouteilles - Octet haut
    txFrame.data[3] = bottlesValue & 0xFF;// H2 Bouteilles - Octet bas
    
    // États des capteurs dans les bits 0 et 1 de l'octet 4
    txFrame.data[4] = (h2Data.pacSensorOk ? 0x01 : 0x00) | 
                      (h2Data.bottlesSensorOk ? 0x02 : 0x00);
    
    // Réserve pour des données futures
    txFrame.data[5] = a0;
    txFrame.data[6] = 0;
    txFrame.data[7] = 0;
    
    if(!ESP32Can.writeFrame(txFrame)) {
        Serial.println("Erreur envoi trame 0x035");
    }
}

// Fonction pour lire les données des capteurs H2 en UART
void readH2Sensors() {
    // Trame selon la documentation ZE07-H2: FF 01 78 00 00 00 A5 C2
    // FF 01: Header fixe
    // 78: Commande de lecture du capteur
    // 00 00 00: Réservé
    // A5 C2: Checksum
    byte h2Command[] = {0xFF, 0x01, 0x78, 0x00, 0x00, 0x00, 0xA5, 0xC2};
    
    // Lecture du capteur H2 PAC
    h2PacSerial.write(h2Command, sizeof(h2Command));
    delay(100); // Attente de la réponse
    
    if (h2PacSerial.available() >= 9) { // La réponse doit contenir 9 octets
        byte response[9];
        h2PacSerial.readBytes(response, 9);
        
        // Vérifier l'en-tête
        if (response[0] == 0xFF && response[1] == 0x78) {
            // Les valeurs de ppm sont dans les octets 2-3 (high byte, low byte)
            h2Data.pacValue = (response[2] << 8) | response[3];
            h2Data.pacSensorOk = true;
        } else {
            h2Data.pacSensorOk = false;
        }
    } else {
        h2Data.pacSensorOk = false;
    }
    
    // Vider le buffer
    while (h2PacSerial.available()) {
        h2PacSerial.read();
    }
    
    // Lecture du capteur H2 Bouteilles
    h2BottlesSerial.write(h2Command, sizeof(h2Command));
    delay(100); // Attente de la réponse
    
    if (h2BottlesSerial.available() >= 9) { // La réponse doit contenir 9 octets
        byte response[9];
        h2BottlesSerial.readBytes(response, 9);
        
        // Vérifier l'en-tête
        if (response[0] == 0xFF && response[1] == 0x78) {
            // Les valeurs de ppm sont dans les octets 2-3 (high byte, low byte)
            h2Data.bottlesValue = (response[2] << 8) | response[3];
            h2Data.bottlesSensorOk = true;
        } else {
            h2Data.bottlesSensorOk = false;
        }
    } else {
        h2Data.bottlesSensorOk = false;
    }
    
    // Vider le buffer
    while (h2BottlesSerial.available()) {
        h2BottlesSerial.read();
    }
    
    // Mise à jour du timestamp
    h2Data.lastUpdate = millis();
}

void readTemperatureSensors() {
    // Demander la lecture des capteurs
    sensors.requestTemperaturesByAddress(sensor1Address); // Lire la première sonde
    sensors.requestTemperaturesByAddress(sensor2Address); // Lire la deuxième sonde
    sensors.requestTemperaturesByAddress(sensor3Address); // Lire la troisième sonde

    // Lire les températures des sondes et les stocker dans la structure
    tempData.tempSensor1 = sensors.getTempC(sensor1Address); // Sonde 1
    tempData.tempSensor2 = sensors.getTempC(sensor2Address); // Sonde 2
    tempData.tempSensor3 = sensors.getTempC(sensor3Address); // Sonde 3

    // Mise à jour du timestamp
    tempData.lastUpdate = millis();
}

void processAuxVoltageFrame(CanFrame &frame) {
    if(frame.data_length_code >= 6) {
        uint16_t voltageBrute = (frame.data[4] << 8) | frame.data[5];
        fuelCellData.auxVoltage = voltageBrute * 0.0079f;
    }
}

void processH2PressureFrame(CanFrame &frame) {
    if(frame.data_length_code >= 5) {
        fuelCellData.h2Pressure = frame.data[4] * 0.01f;
    }
}

void processSystemStateFrame(CanFrame &frame) {
    if(frame.data_length_code >= 4) {
        fuelCellData.systemState = frame.data[0];
        fuelCellData.systemFlags = frame.data[1];
        fuelCellData.h2Consumption = frame.data[2];
        fuelCellData.avgTemp = frame.data[3];
        
        // Convertir l'état en chaîne de caractères
        switch(fuelCellData.systemState) {
            case 10: fuelCellData.systemStateStr = "Initialisation"; break;
            case 20: fuelCellData.systemStateStr = "Auto-check"; break;
            case 30: fuelCellData.systemStateStr = "Standby"; break;
            case 40: fuelCellData.systemStateStr = "Maintenance"; break;
            case 50: fuelCellData.systemStateStr = "Démarrage - purge"; break;
            case 51: fuelCellData.systemStateStr = "Démarrage - vérification tension"; break;
            case 52: fuelCellData.systemStateStr = "Démarrage - warm up"; break;
            case 100: fuelCellData.systemStateStr = "En opération"; break;
            case 200: fuelCellData.systemStateStr = "Arrêt - power down"; break;
            case 201: fuelCellData.systemStateStr = "Arrêt - purge"; break;
            case 202: fuelCellData.systemStateStr = "Arrêt - H2 stop"; break;
            case 203: fuelCellData.systemStateStr = "Arrêt - purge stop"; break;
            case 204: fuelCellData.systemStateStr = "Arrêt - sauvegarde eeprom"; break;
            case 250: fuelCellData.systemStateStr = "Arrêt urgent (erreur)"; break;
            default: fuelCellData.systemStateStr = "Inconnu (" + String(fuelCellData.systemState) + ")";
        }
    }
}

void processPowerDataFrame(CanFrame &frame) {
    if(frame.data_length_code >= 6) {
        // Courant pile (octets 0-1), valeur × 0.01
        uint16_t current_raw = (frame.data[0] << 8) | frame.data[1];
        fuelCellData.current = current_raw * 0.01f;

        // Tension pile (octets 4-5), valeur × 0.01
        uint16_t voltage_raw = (frame.data[4] << 8) | frame.data[5];
        fuelCellData.voltage = voltage_raw * 0.01f;

        // Calculer la puissance
        fuelCellData.power = fuelCellData.current * fuelCellData.voltage;
    }
}

void processErrorFrame(CanFrame &frame) {
    if(frame.data_length_code >= 2) {
        fuelCellData.errorFlags = (frame.data[1] << 8) | frame.data[0];
    }
}

void processRuntimeDataFrame(CanFrame &frame) {
    if(frame.data_length_code >= 4) {
        fuelCellData.runtime = (frame.data[0] << 8) | frame.data[1];
        fuelCellData.energy = (frame.data[2] << 8) | frame.data[3];
    }
}

void processTotalRuntimeDataFrame(CanFrame &frame) {
    if(frame.data_length_code >= 8) {
        fuelCellData.totalRuntime = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
        fuelCellData.totalEnergy = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
    }
}

void printAllData() {
    Serial.println("\n=== DONNEES COMPLÈTES ===");
    Serial.printf("Timestamp: %lu ms\n", fuelCellData.lastUpdate);
    
    Serial.println("\n=== SYSTÈME ===");
    Serial.printf("État: %s\n", fuelCellData.systemStateStr.c_str());
    Serial.printf("Drapeaux: 0x%02X\n", fuelCellData.systemFlags);
    Serial.printf("Consommation H2: %d mg/sec\n", fuelCellData.h2Consumption);
    Serial.printf("Température moyenne: %d°C\n", fuelCellData.avgTemp);
    
    Serial.println("\n=== PUISSANCE ===");
    Serial.printf("Courant: %.2f A\n", fuelCellData.current);
    Serial.printf("Tension: %.2f V\n", fuelCellData.voltage);
    Serial.printf("Puissance: %.2f W\n", fuelCellData.power);
    
    Serial.println("\n=== ERREURS ===");
    Serial.printf("Drapeaux d'erreur: 0x%04X\n", fuelCellData.errorFlags);
    
    Serial.println("\n=== RUNTIME ===");
    Serial.printf("Temps opération: %d minutes\n", fuelCellData.runtime);
    Serial.printf("Énergie produite: %d Wh\n", fuelCellData.energy);
    
    Serial.println("\n=== RUNTIME TOTAL ===");
    Serial.printf("Temps total: %lu minutes\n", fuelCellData.totalRuntime);
    Serial.printf("Énergie totale: %lu Wh\n", fuelCellData.totalEnergy);
    
    Serial.println("\n=== CAPTEURS ===");
    Serial.printf("Pression H2: %.2f bar\n", fuelCellData.h2Pressure);
    Serial.printf("Tension auxiliaire: %.2f V\n", fuelCellData.auxVoltage);
    
    Serial.println("\n=== DONNÉES DES Sondes de temp===");
    Serial.printf("Timestamp: %lu ms\n", tempData.lastUpdate);

    Serial.println("\n=== TEMPÉRATURES ===");
    Serial.printf("Sonde 1: %.2f °C\n", tempData.tempSensor1);
    Serial.printf("Sonde 2: %.2f °C\n", tempData.tempSensor2);
    Serial.printf("Sonde 3: %.2f °C\n", tempData.tempSensor3);
    
    Serial.println("\n=== CAPTEURS H2 ===");
    Serial.printf("Timestamp: %lu ms\n", h2Data.lastUpdate);
    Serial.printf("H2 PAC: %u ppm (Statut: %s)\n", h2Data.pacValue, h2Data.pacSensorOk ? "OK" : "ERREUR");
    Serial.printf("H2 Bouteilles: %u ppm (Statut: %s)\n", h2Data.bottlesValue, h2Data.bottlesSensorOk ? "OK" : "ERREUR");
    
    Serial.println("========================\n");
}

void loop() {
    // Lire les trames CAN avec un timeout de 1000ms
    readTemperatureSensors();
    readH2Sensors();

    // Envoyer les données via CAN
    sendTemperatureData();
    sendH2SensorData();
    
    if(ESP32Can.readFrame(rxFrame, 1000)) {
        // Mettre à jour le timestamp
        fuelCellData.lastUpdate = millis();

        // Afficher la trame brute pour débogage
        Serial.printf("Trame CAN reçue - ID: 0x%X, DLC: %d, Data: ", rxFrame.identifier, rxFrame.data_length_code);
        for(int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf("%02X ", rxFrame.data[i]);
        }
        Serial.println();

        // Traiter les différentes trames
        switch(rxFrame.identifier) {
            case 0x2FC0000: // État du système et informations générales
                processSystemStateFrame(rxFrame);
                break;
                
            case 0x2FC0001: // Courant et tension de la pile
                processPowerDataFrame(rxFrame);
                break;
                
            case 0x2FC0002: // Erreurs système
                processErrorFrame(rxFrame);
                break;
                
            case 0x2FC0003: // Temps opération et énergie produite
                processRuntimeDataFrame(rxFrame);
                break;
                
            case 0x2FC0004: // Temps opération total et énergie totale
                processTotalRuntimeDataFrame(rxFrame);
                break;
            
            case 0x091: 
                processH2PressureFrame(rxFrame);
                break; 

            case 0x090: 
                processAuxVoltageFrame(rxFrame);
                break; 

            default:
                Serial.printf("Trame non gérée - ID: 0x%X\n", rxFrame.identifier);
                break;
        }
        
        // Afficher les données après chaque mise à jour
        printAllData();
        
    } else {
        Serial.println("Timeout lecture CAN");
    }
    
    delay(10); // Petit délai pour éviter de surcharger le CPU
}
