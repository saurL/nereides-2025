//esp central fusion de tout, modif 
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <ArduinoJson.h> // For JSON serialization/deserialization with Raspberry Pi
#include <vector>        // For std::vector used in UART to Raspberry communication


#define CAN_TX_PIN 22
#define CAN_RX_PIN 21
#define CAN_BITRATE 250 // 250 kbps, adjust if your CAN bus operates at a different speed

// --- CONFIGURATION UART POUR RASPBERRY ---
// Ensure these pins (GPIO17, GPIO16) are correct and not conflicting with other peripherals.
#define TX_PIN_RASPBERRY 17 // Renamed to avoid potential conflict with other TX_PIN defines
#define RX_PIN_RASPBERRY 16 // Renamed to avoid potential conflict with other RX_PIN defines
HardwareSerial RaspberrySerial(1); // Using UART1 for communication with Raspberry Pi

#define LED_PIN_1 2
#define LED_PIN_2 15


// Contrôleur moteur (identifiants existants)
#define CAN_ID_MOTOR_MSG1 0x0CF11E05
#define CAN_ID_MOTOR_MSG2 0x0CF11F05

//Sonde de température globale 
#define CAN_ID_TEMP_SENSOR 0x013

unsigned long temps;

// --- STRUCTURES DE DONNÉES ---


// Structure pour BMS 48V (CAN) - NOUVELLE STRUCTURE SÉPARÉE POUR PLUS DE CLARTÉ
// Utilisée pour les données reçues des deux BMS 48V via CAN.
struct BatteryData {
    float chargeVoltage = -100;
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
// Structure pour le contrôleur moteur
// Utilisée pour les données reçues du contrôleur moteur via CAN.
struct MotorControllerData {
    uint16_t speed_rpm = 0;
    float motor_current = 0;
    float battery_voltage = 0;
    uint16_t error_code = 0;
    uint8_t throttle_raw = 0;
    float throttle_voltage = 0;
    int8_t controller_temp = -100; // Default to a very low temperature if not received
    int8_t motor_temp = -100;      // Default to a very low temperature if not received
    const char* command = "Unknown";
    const char* feedback = "Unknown";
    bool switch_boost = false;
    bool switch_foot = false;
    bool switch_fwd = false;
    bool switch_bwd = false;
    bool switch_brake = false;
    bool hall_a = false;
    bool hall_b = false;
    bool hall_c = false;
    bool dataReceived = false; // Flag to indicate if data has been received at least once
};


          // Data for BMS 48V #2
 // Data for thermistors from BMS 48V #2
MotorControllerData controllerData;  // Data for the Motor Controller

// --- DÉCLARATION GLOBALE DE LA TRAME CAN (pour lecture) ---
CanFrame rxFrame;


// --- PROTOTYPES DE FONCTIONS ---
// Pour une meilleure organisation du code et pour éviter les erreurs de compilation (déclaration avant utilisation).
void processReceivedCanFrame(CanFrame& frame); // Traite toutes les trames CAN reçues
void displayAllData(); // Affiche l'état global du système

// Fonctions UART pour Raspberry (envoi JSON)
std::vector<uint8_t> encodeVarIntLength(const std::vector<uint8_t>& json_bytes);
void send_json(StaticJsonDocument<200>& json);
void send_data(const String& data_name, float value);
void send_error(const String& error_name, const String& value);

// Fonctions de décodage Contrôleur Moteur
void decodeMotorMessage1(const CanFrame &frame);
void decodeMotorMessage2(const CanFrame &frame);


// --- FONCTIONS UART RASPBERRY (pour envoyer les données au Raspberry Pi) ---
// Note: La fonction `encodeVarIntLength` est définie ici mais n'est pas utilisée
// directement dans `send_json` telle qu'elle est écrite, car `send_json` gère
// déjà le préfixe '{' et le reste de la chaîne JSON. La logique d'encodage de
// longueur pourrait être intégrée si un protocole de message plus strict est requis.
std::vector<uint8_t> encodeVarIntLength(const std::vector<uint8_t>& json_bytes) {
    uint32_t value = json_bytes.size();
    Serial.println("Taille du JSON en bytes: " + String(value));
    std::vector<uint8_t> result;

    while (value >= 0x80) {
        result.push_back((value & 0x7F) | 0x80);
        value >>= 7;
    }
    result.push_back(value);
    return result;
}


void send_json(StaticJsonDocument<200>& json) {
    String jsonString;
    serializeJson(json, jsonString);
    
    // Envoyer le JSON avec un délimiteur (plus de caractère nul !)
    RaspberrySerial.println(jsonString);
    
    // Petit délai pour éviter de surcharger le buffer série
    delay(2);
    
    // Debug optionnel (décommenter si besoin)
    // Serial.print("Sent to RPi: ");
    // Serial.println(jsonString);
}
// Envoie un document JSON sérialisé via UART au Raspberry Pi.
/*void send_json(StaticJsonDocument<200>& json) {
    String jsonString;
    serializeJson(json, jsonString);
    
    // Le protocole semble ajouter un octet '0' puis '{' puis le reste du JSON.
    // Cette partie est cruciale pour que le Raspberry Pi puisse parser correctement.
    std::vector<uint8_t> message;
    message.push_back(0); // Premier octet 0 (peut-être un marqueur de début de message)
    //message.push_back('{'); // Deuxième octet est '{'
    // Correction: Iterate from index 0 to send the entire JSON string including '{'.
    for (size_t i = 0; i < jsonString.length(); ++i) { 
        message.push_back(static_cast<uint8_t>(jsonString[i]));
    }
    
    RaspberrySerial.write(message.data(), message.size());
    // Serial.print("Sent to RPi: "); // Uncomment for debug prints
    // Serial.println(jsonString);   // Uncomment for debug prints
}*/

// Crée et envoie un message JSON pour une donnée float.
void send_data(const String& data_name, float value) {
    StaticJsonDocument<200> json;
    json["data_name"] = data_name;
    json["value"] = value;
    send_json(json);
}

// Crée et envoie un message JSON pour une erreur.
void send_error(const String& error_name, const String& value) {
    StaticJsonDocument<200> json;
    json["data_name"] = error_name;
    json["value"] = value;
    send_json(json);
}

struct tempGlob {
    float tempglob = 0 ; 
};
tempGlob temperatureGlobale;

// Fonction réception température globale 
void decodeTemperatureSensor(CanFrame& frame) {
    if (frame.data_length_code < 2) return;
    
    // Décoder la température (little-endian)
    int16_t tempCentidegreees = (frame.data[1] << 8) | frame.data[0];
    float temperature = tempCentidegreees / 100.0;
    temperatureGlobale.tempglob = temperature ; 
    Serial.print("==================================");
    Serial.printf("Température reçue: %.2f°C (ID: 0x%03X)\n", 
                  temperature, frame.identifier);
    
    send_data("globale_temp", temperatureGlobale.tempglob);
    Serial.print("ENVOI GLABALE TEMP OK");
}
// --- FONCTIONS DE DÉCODAGE BMS 24V (CAN) ---

void calcul_envoi_Puissance_Instant_API(){
  if (controllerData.dataReceived){
      float puissance_instant = controllerData.motor_current * controllerData.battery_voltage ; 
      send_data("API_puissance_instant", puissance_instant); 
      
  }
}




// --- FONCTIONS DE DÉCODAGE CONTRÔLEUR MOTEUR ---
void decodeMotorMessage1(const CanFrame &frame) {
    controllerData.speed_rpm = (frame.data[1] << 8) | frame.data[0];
    controllerData.motor_current = (((frame.data[3] << 8) | frame.data[2]) / 10.0) - 2000;
    controllerData.battery_voltage = ((frame.data[5] << 8) | frame.data[4]) / 10.0;
    controllerData.error_code = (frame.data[7] << 8) | frame.data[6];
    controllerData.dataReceived = true;

    send_data("motor_rpm", controllerData.speed_rpm);
    send_data("motor_current_a", controllerData.motor_current);
    send_data("motor_battery_voltage", controllerData.battery_voltage);
    send_data("motor_error_code", controllerData.error_code);
    Serial.print("ENVOIE CM OK : RPM CURRENT VOLTAGE ERROR CODE");
    Serial.printf("Motor - RPM:%d, Current:%.1fA, Voltage:%.1fV, Error:0x%04X\n",
                  controllerData.speed_rpm, controllerData.motor_current,
                  controllerData.battery_voltage, controllerData.error_code);
}

void decodeMotorMessage2(const CanFrame &frame) {
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

    send_data("motor_throttle", controllerData.throttle_raw);
    send_data("motor_controller_temp", controllerData.controller_temp);
    send_data("motor_temp", controllerData.motor_temp);
    Serial.print("ENVOIE CM OK : THROTTLE TEMP TEMP MOTOR");
    Serial.printf("Motor - Throttle:%d, CtrlTemp:%d°C, MotorTemp:%d°C, Cmd:%s\n",
                  controllerData.throttle_raw, controllerData.controller_temp,
                  controllerData.motor_temp, controllerData.command);
}
// FONCTION BATT48
//Methodes Batterie 48V 

void handle0x351(const CanFrame& frame) {
    battery.chargeVoltage = (frame.data[0] | (frame.data[1] << 8)) / 10.0;
    battery.chargeCurrentLimit = (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
    battery.dischargeCurrentLimit = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
    send_data("battery_voltage_v", battery.chargeVoltage);
}

void handle0x355(const CanFrame& frame) {
    battery.soc = frame.data[0] | (frame.data[1] << 8);
    battery.soh = frame.data[2] | (frame.data[3] << 8);
    send_data("battery_soc", battery.soc);
    send_data("battery_soh", battery.soh);
}

void handle0x356(const CanFrame& frame) {
    battery.moduleVoltage = (int16_t)(frame.data[0] | (frame.data[1] << 8)) / 100.0;
    battery.totalCurrent =  (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
    battery.avgTemperature = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
    send_data("battery_current_a", battery.totalCurrent);
    send_data("battery_temp", battery.avgTemperature);
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
// --- FONCTION PRINCIPALE DE TRAITEMENT CAN ---
void processReceivedCanFrame(CanFrame& frame) {
    // Always print the raw received frame for debugging
    Serial.printf("[CAN RX] ID:0x%08X, DLC:%d, Data: ", frame.identifier, frame.data_length_code); // Changed %03X to %08X for 32-bit IDs
    for (int i = 0; i < frame.data_length_code; i++) {
        Serial.printf("%02X ", frame.data[i]);
    }
    Serial.println();

    switch (frame.identifier) {
      

        // --- Contrôleur moteur ---
        case CAN_ID_MOTOR_MSG1:
            decodeMotorMessage1(frame);
            break;
        case CAN_ID_MOTOR_MSG2:
            decodeMotorMessage2(frame);
            break;
        case CAN_ID_TEMP_SENSOR: 
          decodeTemperatureSensor(frame); 
          break; 
        case 0x351:
        handle0x351(frame);
        break;
    case 0x355:
        handle0x355(frame);
        break;
    case 0x356:
        handle0x356(frame);
        send_data("battery_temp_c", battery.avgTemperature);
        break;
    case 0x359:
        handle0x359(frame);
        break;
    case 0x35C:
        handle0x35C(frame);
        break;      
        default:
            Serial.printf("Trame CAN avec ID non reconnu: 0x%08X\n", frame.identifier); // Changed %03X to %08X
            break;
    }
}

// --- FONCTION D'AFFICHAGE PÉRIODIQUE ---
void displayAllData() {
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║                       ÉTAT GLOBAL DU SYSTÈME             ║");
    Serial.println("╠════════════════════════════════════════════════════════╣");
    Serial.println("╠════════════════════════════════════════════════════════╣");
    // Contrôleur moteur
    Serial.println("║                       CONTRÔLEUR MOTEUR                  ║");
    if (controllerData.dataReceived) {
        Serial.printf("║ RPM:%d | I:%.1fA | V:%.1fV | Throttle:%d              ║\n",
                      controllerData.speed_rpm, controllerData.motor_current,
                      controllerData.battery_voltage, controllerData.throttle_raw);
        Serial.printf("║ TempCtrl:%d°C | TempMotor:%d°C | Cmd:%s               ║\n",
                      controllerData.controller_temp, controllerData.motor_temp,
                      controllerData.command);
        Serial.printf("║ Switches: Boost=%d, Foot=%d, Fwd=%d, Bwd=%d, Brake=%d ║\n",
                      controllerData.switch_boost, controllerData.switch_foot,
                      controllerData.switch_fwd, controllerData.switch_bwd,
                      controllerData.switch_brake);
        Serial.printf("║ Halls: A=%d, B=%d, C=%d                                ║\n",
                      controllerData.hall_a, controllerData.hall_b, controllerData.hall_c);
    } else {
        Serial.println("║ Aucune donnée contrôleur moteur reçue                   ║");
    }
        Serial.println("╠════════════════════════════════════════════════════════╣");
    Serial.println("╠════════════════════════════════════════════════════════╣");    
    Serial.println("║                       BATTERIE 48V                  ║");
    Serial.printf("║ Charge voltage : %.2f |Charge current limit : %.2f | Discharge current limit : %.2f ║ \n ", battery.chargeVoltage, battery.chargeCurrentLimit, battery.dischargeCurrentLimit);
    Serial.printf("║ SOC : %d | SOH %d                          ║ \n", battery.soc, battery.soh);
    Serial.printf("║ Module Voltage %.2f | Total Current %.2f  | Average Temperature %.2f             ║ \n", battery.moduleVoltage, battery.totalCurrent, battery.avgTemperature);
    Serial.printf("║ Prot1 %d | Prot2 %d | Alarm1 %d | Alarm 2 %d | Module count %d      ║ \n", battery.prot1, battery.prot2, battery.alarm1, battery.alarm2, battery.moduleCount);
            Serial.println("╠════════════════════════════════════════════════════════╣");
    Serial.println("╠════════════════════════════════════════════════════════╣");    
    
    Serial.println("║                       TEMP GLOBAL                  ║");
    Serial.printf("║                            %.2f                     ║ \n", temperatureGlobale.tempglob);
    Serial.println("╚════════════════════════════════════════════════════════╝\n");
}

// --- SETUP ET LOOP ---
void setup() {
    temps = 0;
    Serial.begin(115200);
    while (!Serial); // Wait for serial connection to be established

    // Initialisation UART Raspberry
    // Set up UART1 for communication with Raspberry Pi
    RaspberrySerial.begin(115200, SERIAL_8N1, RX_PIN_RASPBERRY, TX_PIN_RASPBERRY); // Use renamed pins
    if (RaspberrySerial) {
        Serial.println("UART Raspberry OK");
    } else {
        Serial.println("ERREUR: UART Raspberry NON initialisé");
    }

    // Configuration CAN
    // Set up CAN bus pins and queue sizes
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    ESP32Can.setRxQueueSize(30); // Increased Rx queue size for more robust reception
    ESP32Can.setTxQueueSize(30); // Increased Tx queue size for more robust transmission
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, LOW);
    // Set CAN bus speed. ESP32Can.convertSpeed expects the bitrate in Hz directly.
    // Corrected: pass CAN_BITRATE directly, not CAN_BITRATE / 1000.
    if (ESP32Can.begin(ESP32Can.convertSpeed(CAN_BITRATE))) {
        Serial.printf("CAN Bus initialisé avec succès (%d kbps)\n", CAN_BITRATE / 1000);
    } else {
        Serial.println("ERREUR: Échec initialisation CAN Bus");
        while (true) {
            // Halt program execution if CAN bus fails to initialize
            delay(100);
        }
    }

    
}
// Version corrigée de la fonction
void makeLedBlink() {
    static unsigned long lastTime = 0;
    static bool led1On = true;
    
    if (millis() - lastTime >= 500) {
        digitalWrite(LED_PIN_1, led1On ? HIGH : LOW);
        digitalWrite(LED_PIN_2, led1On ? LOW : HIGH);
        led1On = !led1On;
        lastTime = millis();
    }
}

void loop() {
    // Vérification des seuils pour allumer la LED
    if (controllerData.controller_temp > 63)// **Condition pour contrôleur moteur**
        { 
        makeLedBlink();                     // **Allumer la LED**
    } else {
        digitalWrite(LED_PIN_2, LOW);                      // **Éteindre la LED**
        }

     if (battery.avgTemperature> 55)
 {
  makeLedBlink();  // Allumer la LED
} else {
  digitalWrite(LED_PIN_1, LOW);  // Éteindre la LED
}
    while (ESP32Can.readFrame(rxFrame, 0)) {
      
       processReceivedCanFrame(rxFrame);
       Serial.print(temperatureGlobale.tempglob); 
    }

    // Periodic display of all collected data
    // Static variable ensures lastDisplayTime keeps its value between loop calls.
    static unsigned long lastDisplayTime = 0;
    const unsigned long displayInterval = 3000; // Display every 3 seconds

    if (millis() - lastDisplayTime >= displayInterval) {
        displayAllData();
        lastDisplayTime = millis();
    }

    // Small delay to yield to other FreeRTOS tasks and prevent busy-waiting
    delay(10);
}
