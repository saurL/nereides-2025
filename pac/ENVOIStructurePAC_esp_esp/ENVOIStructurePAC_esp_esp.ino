#include <ESP32-TWAI-CAN.hpp>

// Pins CAN (à adapter selon votre configuration)
#define CAN_TX 5
#define CAN_RX 4

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

void setup() {
    // Setup serial pour le débogage
    Serial.begin(115200);
    while (!Serial); // Attendre que le port série soit prêt

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
    
    Serial.println("========================\n");
}

void loop() {
    // Lire les trames CAN avec un timeout de 1000ms
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
