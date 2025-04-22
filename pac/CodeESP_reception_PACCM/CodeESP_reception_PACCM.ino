#include <ESP32-TWAI-CAN.hpp>
// Pins CAN (à adapter selon votre configuration)
#define CAN_TX 21
#define CAN_RX 22

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


// Structure globale contenant toutes les données du contrôleur moteur
struct MotorControllerData {
    uint16_t speed_rpm;
    float motor_current;         // En ampères
    float battery_voltage;       // En volts
    uint16_t error_code;

    uint8_t throttle_raw;
    float throttle_voltage;
    int8_t controller_temp;      // En °C
    int8_t motor_temp;           // En °C
    const char* command;
    const char* feedback;

    bool switch_boost;
    bool switch_foot;
    bool switch_fwd;
    bool switch_bwd;
    bool switch_brake;

    bool hall_a;
    bool hall_b;
    bool hall_c;
};

// Instanciation globale
MotorControllerData controllerData;

struct TemperatureData {
    float tempSensor1 = 0.0f;
    float tempSensor2 = 0.0f;
    float tempSensor3 = 0.0f;
    unsigned long lastUpdate = 0;
};

TemperatureData tempData;

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

///////////Méthodes pour PAC 
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
/////////////////////
// méthodes controleur moteur 
void decodeMessage1(const CanFrame &frame) {
    controllerData.speed_rpm = (frame.data[1] << 8) | frame.data[0];
    controllerData.motor_current = ((frame.data[3] << 8) | frame.data[2]) / 10.0;
    controllerData.battery_voltage = ((frame.data[5] << 8) | frame.data[4]) / 10.0;
    controllerData.error_code = (frame.data[7] << 8) | frame.data[6];

    Serial.println("\n--- Message 1 (0x0CF11E05) ---");
    Serial.printf("Speed: %d RPM\n", controllerData.speed_rpm);
    Serial.printf("Motor Current: %.1f A\n", controllerData.motor_current);
    Serial.printf("Battery Voltage: %.1f V\n", controllerData.battery_voltage);
    Serial.printf("Error Code: 0x%04X\n", controllerData.error_code);

    for (int i = 0; i < 16; i++) {
        if (controllerData.error_code & (1 << i)) {
            Serial.printf("ERR%d: Active\n", i);
        }
    }
}

void decodeMessage2(const CanFrame &frame) {
    controllerData.throttle_raw = frame.data[0];
    controllerData.throttle_voltage = controllerData.throttle_raw * 5.0 / 255.0;
    controllerData.controller_temp = frame.data[1] - 40;
    controllerData.motor_temp = frame.data[2] - 30;

    uint8_t controller_status = frame.data[4];
    uint8_t switch_signals = frame.data[5];

    const char* commands[] = {"Neutral", "Forward", "Backward", "Reserved"};
    const char* feedbacks[] = {"Stationary", "Forward", "Backward", "Reserved"};

    controllerData.command = commands[controller_status & 0x03];
    controllerData.feedback = feedbacks[(controller_status >> 2) & 0x03];

    controllerData.switch_boost = (switch_signals >> 7) & 0x01;
    controllerData.switch_foot = (switch_signals >> 6) & 0x01;
    controllerData.switch_fwd = (switch_signals >> 5) & 0x01;
    controllerData.switch_bwd = (switch_signals >> 4) & 0x01;
    controllerData.switch_brake = (switch_signals >> 3) & 0x01;

    controllerData.hall_a = (switch_signals >> 0) & 0x01;
    controllerData.hall_b = (switch_signals >> 1) & 0x01;
    controllerData.hall_c = (switch_signals >> 2) & 0x01;

    Serial.println("\n--- Message 2 (0x0CF11F05) ---");
    Serial.printf("Throttle: %d (%.2f V)\n", controllerData.throttle_raw, controllerData.throttle_voltage);
    Serial.printf("Controller Temp: %d°C\n", controllerData.controller_temp);
    Serial.printf("Motor Temp: %d°C\n", controllerData.motor_temp);
    Serial.printf("Command: %s, Feedback: %s\n", controllerData.command, controllerData.feedback);
    Serial.printf("Switches: Boost=%d, Foot=%d, Fwd=%d, Bwd=%d, Brake=%d\n",
                  controllerData.switch_boost, controllerData.switch_foot,
                  controllerData.switch_fwd, controllerData.switch_bwd,
                  controllerData.switch_brake);
    Serial.printf("Hall Sensors: A=%d, B=%d, C=%d\n",
                  controllerData.hall_a, controllerData.hall_b, controllerData.hall_c);
}
//méthode pour les sondes de temp
void processTempSonde(const CanFrame &frame){
  int16_t rawTemp1 = (rxFrame.data[0] << 8) | rxFrame.data[1];
  int16_t rawTemp2 = (rxFrame.data[2] << 8) | rxFrame.data[3];
  int16_t rawTemp3 = (rxFrame.data[4] << 8) | rxFrame.data[5];

  tempData.tempSensor1 = rawTemp1 / 100.0f;
  tempData.tempSensor2 = rawTemp2 / 100.0f;
  tempData.tempSensor3 = rawTemp3 / 100.0f;
  tempData.lastUpdate = millis();
}

void loop() {
    if (ESP32Can.readFrame(rxFrame, 1000)) {
        fuelCellData.lastUpdate = millis();
        Serial.printf("\nReceived frame: 0x%08X, DLC=%d\n", rxFrame.identifier, rxFrame.data_length_code);
         Serial.printf("Trame CAN reçue - ID: 0x%X, DLC: %d, Data: ", rxFrame.identifier, rxFrame.data_length_code);
        for(int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf("%02X ", rxFrame.data[i]);
        }
        Serial.println();

        switch (rxFrame.identifier) {
            case 0x0CF11E05:    //msg 1 du cm 
                decodeMessage1(rxFrame);
                break;
            case 0x0CF11F05:    //msg 2 du cm 
                decodeMessage2(rxFrame);
                break;

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
            case 0x304 : 
                processTempSonde(rxFrame); 
                break; 
            default:
                Serial.println("Unknown frame, printing raw data:");
                for (int i = 0; i < rxFrame.data_length_code; i++) {
                    Serial.printf("Data[%d]: 0x%02X\n", i, rxFrame.data[i]);
                }
                break;
        }
    }
    else {
        Serial.println("Timeout lecture CAN");
    }
    
    delay(10);
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
    Serial.println("Donnees CM:"); 
     Serial.println("\n--- Message 1 (0x0CF11E05) ---");
    Serial.printf("Speed: %d RPM\n", controllerData.speed_rpm);
    Serial.printf("Motor Current: %.1f A\n", controllerData.motor_current);
    Serial.printf("Battery Voltage: %.1f V\n", controllerData.battery_voltage);
    Serial.printf("Error Code: 0x%04X\n", controllerData.error_code);
     Serial.println("\n--- Message 2 (0x0CF11F05) ---");
    Serial.printf("Throttle: %d (%.2f V)\n", controllerData.throttle_raw, controllerData.throttle_voltage);
    Serial.printf("Controller Temp: %d°C\n", controllerData.controller_temp);
    Serial.printf("Motor Temp: %d°C\n", controllerData.motor_temp);
    Serial.printf("Command: %s, Feedback: %s\n", controllerData.command, controllerData.feedback);
    Serial.printf("Switches: Boost=%d, Foot=%d, Fwd=%d, Bwd=%d, Brake=%d\n",
                  controllerData.switch_boost, controllerData.switch_foot,
                  controllerData.switch_fwd, controllerData.switch_bwd,
                  controllerData.switch_brake);
    Serial.printf("Hall Sensors: A=%d, B=%d, C=%d\n",
                  controllerData.hall_a, controllerData.hall_b, controllerData.hall_c);
    Serial.println("=== DONNÉES REÇUES PAR Sonde température ===");
            Serial.printf("Timestamp: %lu ms\n", tempData.lastUpdate);
            Serial.printf("Sonde 1: %.2f °C\n", tempData.tempSensor1);
            Serial.printf("Sonde 2: %.2f °C\n", tempData.tempSensor2);
            Serial.printf("Sonde 3: %.2f °C\n\n", tempData.tempSensor3);
}



