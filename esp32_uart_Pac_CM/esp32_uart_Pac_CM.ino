#include <ESP32-TWAI-CAN.hpp>
#include <ArduinoJson.h>
#include <vector>

// Pins CAN (à adapter selon votre configuration)
#define CAN_TX 21
#define CAN_RX 22

#define TX_PIN 17  // Broche TX pour UART1
#define RX_PIN 16  // Broche RX pour UART1
#define BUTTON_PIN   14  // Broche d'entrée pour le bouton qui contrôle la puissance des ventilos

const int transistorPinCapteurH2 = 26; // Pin allumage led cpateur H2
const int transistorPinTemperature = 25; // pin allumage led temperature depassee
const int transistorPinMoteur = 27; // attention à définir (moteur allume)
const int transistorPinPAC = 33 ; // attention à définir

//définition de la position initiale du bouton ventilos
int buttonState = 0;       // État actuel du bouton
bool isFanReduced = false; // Indicateur de mode de vitesse
int fanSpeed = 255;        // Vitesse initiale du ventilateur (0-255, 100%)
int lastButtonState = 0;   // Dernier état du bouton

//dénition de paramètres pour le ventilo
const int freq = 25000;    // Fréquence PWM en Hz
const int pwmChannel = 0;  // Canal PWM
const int resolution = 8;  // Résolution (8 bits = 0-255)


HardwareSerial RaspberrySerial(1);
/*  Partie du code utilisé pour la communication en UART avec le raspberry */
/*
datas = [
"pac_aux_voltage_v",
"pac_hydrogen_consumption_mgs", 
"pac_temperature_c", 
"pac_system_state",
"pac_current_a",
"pac_voltage_v",
"pac_error_flag", 
"pac_produced_energy", 
"pac_operation_time", 
"pac_total_produced_energy", 
"pac_total_operation_time",
"motor_rpm", 
"mottor_current_a", 
"battery_voltage_v",
"motor_error_code",
"motor_throttle",
"motor_controller_temp",
"motor_temp",
]
*/


// Fonction pour encoder la taille d'un JSON en VarInt
std::vector<uint8_t>  encodeVarIntLength(const std::vector<uint8_t>& json_bytes) {
    uint32_t value = json_bytes.size();
    Serial.println("on a récuperer la taille du json");
    Serial.println("taille de botre json en bytes  : " );
    Serial.println(value);
    std::vector<uint8_t> result;

    while (value >= 0x80) {
        result.push_back((value & 0x7F) | 0x80); // 7 bits + continuation
        value >>= 7;
    }
    result.push_back(value); // dernier octet (MSB = 0)
    Serial.println("Données du varint");
    for (size_t i = 0; i < result.size(); ++i) {
        Serial.println(result[i],HEX);
    }
    return result;
}

std::vector<uint8_t> stringToBytes(const String& input) {
    std::vector<uint8_t> result;
    for (size_t i = 0; i < input.length(); ++i) {
        result.push_back(static_cast<uint8_t>(input[i]));
    }
    return result;
}


void send_json(StaticJsonDocument<200>& json) {
    // Convertir en chaîne JSON
    String jsonString;
    serializeJson(json, jsonString);
    Serial.println("On vas calculr json en bytes");
    std::vector<uint8_t> json_bytes = stringToBytes(jsonString);
    // Encodage de la taille du JSON en VarInt
    std::vector<uint8_t>  varIntSize = encodeVarIntLength(json_bytes);

    // Affichage des informations
    Serial.println("JSON : " + jsonString);
    std::vector<uint8_t> message;
    message.insert(message.end(), varIntSize.begin(), varIntSize.end()); // Ajout du VarInt
    message.insert(message.end(), json_bytes.begin(), json_bytes.end()); // Ajout du JSON après le VarInt
    // Envoi du VarInt (taille du JSON)
    RaspberrySerial.write(message.data(), message.size()); // Envoi de la taille du JSON
}

void send_data(const String& data_name, float value) {
    StaticJsonDocument<200> json;
    json["data_name"] = data_name;
    json["value"] = value;

    // Envoi du JSON
    send_json(json);
}

void send_string(const String& data_name, const String& value) {
    StaticJsonDocument<200> json;
    json["data_name"] = data_name;
    json["value"] = value;

    // Envoi du JSON
    send_json(json);
}


CanFrame rxFrame;

//Structure Batterie 48V 
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

// Structure pour les capteurs d'hydrogène
struct H2SensorsData {
    uint16_t pacValue = 0;         // Valeur du capteur H2 PAC (en ppm)
    uint16_t bottlesValue = 0;     // Valeur du capteur H2 bouteilles (en ppm)
    bool pacSensorOk = false;      // État du capteur PAC
    bool bottlesSensorOk = false;  // État du capteur bouteilles
    unsigned long lastUpdate = 0;  // Timestamp de la dernière mise à jour
};

H2SensorsData h2SensorsData;


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


///////////Méthodes pour PAC 
void processAuxVoltageFrame(CanFrame &frame) {
    if(frame.data_length_code >= 6) {
        uint16_t voltageBrute = (frame.data[4] << 8) | frame.data[5];
        fuelCellData.auxVoltage = voltageBrute * 0.0079f;
    }
    send_data("pac_aux_voltage_v", fuelCellData.auxVoltage);
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
        send_data("pac_hydrogen_consumption_mgs", fuelCellData.h2Consumption);
        send_data("pac_temperature_c", fuelCellData.avgTemp);
        send_string("pac_system_state", fuelCellData.systemStateStr);

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

        send_data("pac_current_a", fuelCellData.current);
        send_data("pac_voltage_v", fuelCellData.voltage);
    }
}

void processErrorFrame(CanFrame &frame) {
    if(frame.data_length_code >= 2) {
        fuelCellData.errorFlags = (frame.data[1] << 8) | frame.data[0];
        send_data("pac_error_flag", fuelCellData.errorFlags);
    }
}

void processRuntimeDataFrame(CanFrame &frame) {
    if(frame.data_length_code >= 4) {
        fuelCellData.runtime = (frame.data[0] << 8) | frame.data[1];
        fuelCellData.energy = (frame.data[2] << 8) | frame.data[3];
        send_data("pac_produced_energy", fuelCellData.energy);
        send_data("pac_operation_time", fuelCellData.runtime);
    }
}

void processTotalRuntimeDataFrame(CanFrame &frame) {
    if(frame.data_length_code >= 8) {
        fuelCellData.totalRuntime = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
        fuelCellData.totalEnergy = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
        send_data("pac_total_produced_energy", fuelCellData.totalEnergy);
        send_data("pac_total_operation_time", fuelCellData.totalRuntime);
    }
}

/////////////////////
//Methodes Batterie 48V 

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
    send_data("motor_rpm", controllerData.speed_rpm);
    send_data("mottor_current_a", controllerData.motor_current);
    send_data("battery_voltage_v", controllerData.battery_voltage);
    send_data("motor_error_code", controllerData.error_code );
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

    send_data("motor_throttle", controllerData.throttle_raw);
    send_data("motor_controller_temp", controllerData.controller_temp);
    send_data("motor_temp", controllerData.motor_temp);

}

// Traitement de la trame 0x305 pour les capteurs H2
void processH2SensorsFrame(CanFrame &frame) {
    if(frame.data_length_code >= 8) {
        h2SensorsData.pacValue = (frame.data[0] << 8) | frame.data[1];
        h2SensorsData.bottlesValue = (frame.data[2] << 8) | frame.data[3];

        h2SensorsData.pacSensorOk = frame.data[4] & 0x01;
        h2SensorsData.bottlesSensorOk = frame.data[5] & 0x01;

        h2SensorsData.lastUpdate = millis();

        send_data("h2_pac_ppm", h2SensorsData.pacValue);
        send_data("h2_bottle_ppm", h2SensorsData.bottlesValue);
        send_string("h2_pac_status", h2SensorsData.pacSensorOk ? "OK" : "ERREUR");
        send_string("h2_bottle_status", h2SensorsData.bottlesSensorOk ? "OK" : "ERREUR");
    }
}


void setup() {
    // Setup serial pour le débogage
    Serial.begin(115200);
    while (!Serial); // Attendre que le port série soit prêt
    RaspberrySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
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

void loop() {
    if (ESP32Can.readFrame(rxFrame, 1000)) {
             if ((rxFrame.identifier >= 0x370 && rxFrame.identifier <= 0x379) ||
            rxFrame.identifier == 0x35A || 
            rxFrame.identifier == 0x35E || 
            rxFrame.identifier == 0x35F) {
            return;
        }

        fuelCellData.lastUpdate = millis();
        Serial.printf("\nReceived frame: 0x%08X, DLC=%d\n", rxFrame.identifier, rxFrame.data_length_code);
         Serial.printf("Trame CAN reçue - ID: 0x%X, DLC: %d, Data: ", rxFrame.identifier, rxFrame.data_length_code);
        for(int i = 0; i < rxFrame.data_length_code; i++) {
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
            case 0x305:
                processH2SensorsFrame(rxFrame);
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


//gestion vitesse ventilos selon position du bouton
// Lire l'état du bouton
buttonState = digitalRead(BUTTON_PIN);
// Vérifier si le bouton est pressé (transition 0 -> 1)
if (buttonState == HIGH && lastButtonState == LOW ) {
  if (isFanReduced) {
      fanSpeed = 255; // Pleine vitesse
      isFanReduced = false;
  } else {
      fanSpeed = 153; // 60 % de 255
      isFanReduced = true;
    }
  // Appliquer la nouvelle vitesse
  ledcWrite(pwmChannel, fanSpeed);
  Serial.print("Nouvelle vitesse du ventilateur : ");
  Serial.println(fanSpeed);    
  // Mettre à jour l'état précédent du bouton
  lastButtonState = buttonState;
}




//allumage led temperature
   if (controllerData.controller_temp>70) {
    // Allumer la LED (activer le transistor)
    digitalWrite(transistorPinTemperature, HIGH);
   };
   if (controllerData.motor_temp>150) {
    // Allumer la LED (activer le transistor)
    digitalWrite(transistorPinTemperature, HIGH);
   };
   if (fuelCellData.avgTemp>70) {
    // Allumer la LED (activer le transistor)
    digitalWrite(transistorPinTemperature, HIGH);
   };
   if (battery.avgTemperature>60) {
    // Allumer la LED (activer le transistor)
    digitalWrite(transistorPinTemperature, HIGH);
   };

   //allumage led H2
   if (h2SensorsData.pacValue>90) {
    // Allumer la LED (activer le transistor)
    digitalWrite(transistorPinCapteurH2, HIGH);
   };
   if (h2SensorsData.bottlesValue>90) {
    // Allumer la LED (activer le transistor)
    digitalWrite(transistorPinCapteurH2, HIGH);
   };
}