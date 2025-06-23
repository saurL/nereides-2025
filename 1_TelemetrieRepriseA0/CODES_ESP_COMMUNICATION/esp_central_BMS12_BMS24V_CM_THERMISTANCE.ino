#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <ArduinoJson.h> // For JSON serialization/deserialization with Raspberry Pi
#include <vector>        // For std::vector used in UART to Raspberry communication

// --- CONFIGURATION CAN TWAI ---
// Ensure these pins (GPIO5, GPIO4) are correct for your ESP32 board and wiring.
// Common CAN pins on ESP32 are often 4 and 5, or 21 and 22.
#define CAN_TX_PIN 5
#define CAN_RX_PIN 4
#define CAN_BITRATE 250 // 250 kbps, adjust if your CAN bus operates at a different speed

// --- CONFIGURATION UART POUR RASPBERRY ---
// Ensure these pins (GPIO17, GPIO16) are correct and not conflicting with other peripherals.
#define TX_PIN_RASPBERRY 17 // Renamed to avoid potential conflict with other TX_PIN defines
#define RX_PIN_RASPBERRY 16 // Renamed to avoid potential conflict with other RX_PIN defines
HardwareSerial RaspberrySerial(1); // Using UART1 for communication with Raspberry Pi

// --- IDENTIFIANTS CAN ---
// BMS 24V (identifiants de réponse attendus après une requête)
#define CAN_ID_BMS24V_RESP_1 0x18904001 // Total Voltage, Current, SOC
#define CAN_ID_BMS24V_RESP_2 0x18914001 // Cell Voltages (Max/Min)
#define CAN_ID_BMS24V_RESP_3 0x18924001 // Temperatures (Max/Min from BMS)
#define CAN_ID_BMS24V_RESP_4 0x18984001 // Error Status

// BMS 48V #1 (votre premier BMS 48V)
#define CAN_ID_BMS1_BASE 0x00000001 // Base ID for BMS 48V #1 data frames (0x001-0x004)

// BMS 48V #2 (votre deuxième BMS 48V)
#define CAN_ID_BMS2_BASE 0x00000005 // Base ID for BMS 48V #2 data frames (0x005-0x008)

// Thermistances 48V (min/max calculées par les BMS 48V eux-mêmes et envoyées)
// Les IDs 0x009 et 0x010 sont ceux que vous avez spécifiés pour les trames de température min/max.
#define CAN_ID_TEMP1_MINMAX 0x00000009 // Min/Max temperatures from BMS 48V #1 thermistors
#define CAN_ID_TEMP2_MINMAX 0x00000010 // Min/Max temperatures from BMS 48V #2 thermistors

// Thermistances 24V (généralement envoyées par un module thermistances 24V, ID d'exemple)
// Assurez-vous que l'ID 0x011 est bien celui que votre module thermistances 24V envoie.
#define CAN_ID_TEMP_24V_MINMAX 0x00000011 // Min/Max temperatures from 24V thermistor module

// Contrôleur moteur (identifiants existants)
#define CAN_ID_MOTOR_MSG1 0x0CF11E05
#define CAN_ID_MOTOR_MSG2 0x0CF11F05


// --- STRUCTURES DE DONNÉES ---

// Structure pour BMS 24V (CAN)
// Utilisée pour les données reçues du BMS 24V via CAN.
struct BmsData24V {
    float totalVoltage = -1;
    float gatherVoltage = -1;
    float current = -1;
    float soc = -1;
    uint16_t maxCellVoltage = 0;
    uint8_t maxCellNumber = 0;
    uint16_t minCellVoltage = 0;
    uint8_t minCellNumber = 0;
    int8_t maxTemp = -100;
    int8_t minTemp = -100;
    uint64_t errorStatus = 0;
    bool dataReceived = false; // Flag to indicate if data has been received at least once
};

// Structure pour BMS 48V (CAN) - NOUVELLE STRUCTURE SÉPARÉE POUR PLUS DE CLARTÉ
// Utilisée pour les données reçues des deux BMS 48V via CAN.
struct BmsData48V {
    float totalVoltage = -1;
    float gatherVoltage = -1;
    float current = -1;
    float soc = -1;
    uint16_t maxCellVoltage = 0;
    uint8_t maxCellNumber = 0;
    uint16_t minCellVoltage = 0;
    uint8_t minCellNumber = 0;
    int8_t maxTemp = -100;
    int8_t minTemp = -100;
    uint64_t errorStatus = 0;
    bool dataReceived = false; // Flag to indicate if data has been received at least once
};

// Structure pour les températures des thermistances (min/max agrégées)
// Utilisée pour les données min/max des thermistances 24V et 48V.
// Les valeurs sont stockées en centièmes de degré Celsius (e.g., 25.50°C -> 2550).
struct DecodedTemperatureData {
    uint16_t minTempCalculated = 0xFFFF; // Initialiser à une valeur haute pour le min
    uint16_t maxTempCalculated = 0x0000; // Initialiser à une valeur basse pour le max
    bool dataReceived = false;           // Flag to indicate if data has been received at least once
};

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


// --- INSTANCES GLOBALES DES STRUCTURES DE DONNÉES ---
BmsData24V bmsData24V;               // Data for the single 24V BMS
BmsData48V bmsData48V_1;             // Data for BMS 48V #1
BmsData48V bmsData48V_2;             // Data for BMS 48V #2
DecodedTemperatureData thermistance24V;     // Data for the 24V thermistor module
DecodedTemperatureData thermistance48V_1;   // Data for thermistors from BMS 48V #1
DecodedTemperatureData thermistance48V_2;   // Data for thermistors from BMS 48V #2
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

// Fonctions de décodage BMS 24V
void processReceivedData1890(uint8_t* data, uint32_t identifier);
void processReceivedData1891(uint8_t* data, uint32_t identifier);
void processReceivedData1892(uint8_t* data, uint32_t identifier);
void processReceivedData1898(uint8_t* data, uint32_t identifier);

// Fonctions de décodage BMS 48V (passant une référence à la bonne structure BMS)
void decodeCanFrame4801(CanFrame& frame, BmsData48V* bmsData, uint32_t canId); // Pass BmsData48V*
void decodeCanFrame4802(CanFrame& frame, BmsData48V* bmsData, uint32_t canId); // Pass BmsData48V*
void decodeCanFrame4803(CanFrame& frame, BmsData48V* bmsData, uint32_t canId); // Pass BmsData48V*
void decodeCanFrame4804(CanFrame& frame, BmsData48V* bmsData, uint32_t canId); // Pass BmsData48V*

// Fonctions de décodage Thermistances (passant une référence à la bonne structure Thermistance)
void decodeThermistance24V(CanFrame& frame); // For the single 24V thermistor module
void decodeCanFrameMinMaxTemp(CanFrame& frame, DecodedTemperatureData* tempData, uint32_t canId); // For 48V thermistors

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

// Envoie un document JSON sérialisé via UART au Raspberry Pi.
void send_json(StaticJsonDocument<200>& json) {
    String jsonString;
    serializeJson(json, jsonString);
    
    // Le protocole semble ajouter un octet '0' puis '{' puis le reste du JSON.
    // Cette partie est cruciale pour que le Raspberry Pi puisse parser correctement.
    std::vector<uint8_t> message;
    message.push_back(0); // Premier octet 0 (peut-être un marqueur de début de message)
    
    // Correction: Iterate from index 0 to send the entire JSON string including '{'.
    for (size_t i = 0; i < jsonString.length(); ++i) { 
        message.push_back(static_cast<uint8_t>(jsonString[i]));
    }
    
    RaspberrySerial.write(message.data(), message.size());
    // Serial.print("Sent to RPi: "); // Uncomment for debug prints
    // Serial.println(jsonString);   // Uncomment for debug prints
}

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


// --- FONCTIONS DE DÉCODAGE BMS 24V (CAN) ---
// Ces fonctions mettent à jour la structure bmsData24V.
void processReceivedData1890(uint8_t* data, uint32_t identifier) {
    bmsData24V.totalVoltage = ((data[0] << 8) | data[1]) * 0.1;
    bmsData24V.gatherVoltage = ((data[2] << 8) | data[3]) * 0.1;
    bmsData24V.current = (((data[4] << 8) | data[5]) - 30000) * 0.1;

    bmsData24V.current = ((((data[4] << 8) | data[5]) - 2000) * 0.1 );
    bmsData24V.soc = (data[6] == 0xFF && data[7] == 0xFF) ? -1 : ((data[6] << 8) | data[7]) * 0.1;
    bmsData24V.dataReceived = true;
    
    send_data("battery24_soc", bmsData24V.soc);
    send_data("battery24_voltage", bmsData24V.totalVoltage);
    send_data("battery24_current", bmsData24V.current);
    
    Serial.printf("BMS 24V - V:%.1fV, I:%.1fA, SOC:%.1f%%\n",
                  bmsData24V.totalVoltage, bmsData24V.current, bmsData24V.soc);
}

void processReceivedData1891(uint8_t* data, uint32_t identifier) {
    bmsData24V.maxCellVoltage = (data[0] << 8) | data[1];
    bmsData24V.maxCellNumber = data[2]; // Corrected typo: bms24V -> bmsData24V
    bmsData24V.minCellVoltage = (data[3] << 8) | data[4];
    bmsData24V.minCellNumber = data[5];
    
    send_data("battery24_maxCellVoltage", bmsData24V.maxCellVoltage);
    send_data("battery24_minCellVoltage", bmsData24V.minCellVoltage);
    
    Serial.printf("BMS 24V - CellMax:%dmV(#%d), CellMin:%dmV(#%d)\n",
                  bmsData24V.maxCellVoltage, bmsData24V.maxCellNumber,
                  bmsData24V.minCellVoltage, bmsData24V.minCellNumber);
}

void processReceivedData1892(uint8_t* data, uint32_t identifier) {
    bmsData24V.maxTemp = data[0] - 40;
    bmsData24V.minTemp = data[2] - 40;
    
    send_data("battery24_maxTemp", bmsData24V.maxTemp);
    send_data("battery24_minTemp", bmsData24V.minTemp);
    
    Serial.printf("BMS 24V - TempMax:%d°C, TempMin:%d°C\n",
                  bmsData24V.maxTemp, bmsData24V.minTemp);
}

void processReceivedData1898(uint8_t* data, uint32_t identifier) {
    bmsData24V.errorStatus = 0;
    for (int i = 0; i < 8; i++) {
        bmsData24V.errorStatus |= ((uint64_t)data[i] << (8 * i));
    }
    Serial.printf("BMS 24V - ErrorStatus:0x%llX\n", bmsData24V.errorStatus);
}

// Removed redundant readCanFrames() function, as its logic is directly in loop()


// --- FONCTIONS DE DÉCODAGE BMS 48V (CAN) ---
// Chaque fonction prend en argument la structure BMS 48V spécifique à mettre à jour.
void decodeCanFrame4801(CanFrame& frame, BmsData48V* bmsData, uint32_t canId) { // Changed BmsData24V* to BmsData48V*
    if (frame.data_length_code < 8) return;

    uint16_t totalVoltageRaw = (frame.data[0] << 8) | frame.data[1];
    uint16_t gatherVoltageRaw = (frame.data[2] << 8) | frame.data[3];
    uint16_t currentRaw = (frame.data[4] << 8) | frame.data[5];
    uint16_t socRaw = (frame.data[6] << 8) | frame.data[7];

    bmsData->totalVoltage = (float)totalVoltageRaw / 10.0;
    bmsData->gatherVoltage = (float)gatherVoltageRaw / 10.0;
    bmsData->current = ((float)currentRaw - 30000.0) / 10.0; // Invert the +30000 offset
    bmsData->soc = (socRaw == 0xFFFF) ? -1 : (float)socRaw / 10.0; // Check for invalid value
    bmsData->dataReceived = true; // Mark as received

    // Send to Raspberry Pi, differentiating between BMS 1 and BMS 2
    String bmsPrefix = (canId >= CAN_ID_BMS1_BASE && canId <= (CAN_ID_BMS1_BASE + 3)) ? "battery48_1_" : "battery48_2_";
    send_data(bmsPrefix + "soc", bmsData->soc);
    send_data(bmsPrefix + "voltage", bmsData->totalVoltage);
    send_data(bmsPrefix + "current", bmsData->current);

    Serial.printf("    Decoded 0x%03X: V_total=%.1fV, V_gather=%.1fV, I=%.1fA, SOC=%.1f%%\n",
                  canId, bmsData->totalVoltage, bmsData->gatherVoltage, bmsData->current, bmsData->soc);
}

void decodeCanFrame4802(CanFrame& frame, BmsData48V* bmsData, uint32_t canId) { // Changed BmsData24V* to BmsData48V*
    if (frame.data_length_code < 6) return; // Only 6 useful bytes

    bmsData->maxCellVoltage = (frame.data[0] << 8) | frame.data[1];
    bmsData->maxCellNumber = frame.data[2];
    bmsData->minCellVoltage = (frame.data[3] << 8) | frame.data[4];
    bmsData->minCellNumber = frame.data[5];

    String bmsPrefix = (canId >= CAN_ID_BMS1_BASE && canId <= (CAN_ID_BMS1_BASE + 3)) ? "battery48_1_" : "battery48_2_";
    send_data(bmsPrefix + "maxCellVoltage", bmsData->maxCellVoltage);
    send_data(bmsPrefix + "minCellVoltage", bmsData->minCellVoltage);

    Serial.printf("    Decoded 0x%03X: VMax=%dmV(#%d), VMin=%dmV(#%d)\n",
                  canId, bmsData->maxCellVoltage, bmsData->maxCellNumber,
                  bmsData->minCellVoltage, bmsData->minCellNumber);
}

void decodeCanFrame4803(CanFrame& frame, BmsData48V* bmsData, uint32_t canId) { // Changed BmsData24V* to BmsData48V*
    if (frame.data_length_code < 3) return; // Only 3 useful bytes (data[0], data[2])

    bmsData->maxTemp = (int8_t)(frame.data[0] - 40); // Invert the +40 offset
    bmsData->minTemp = (int8_t)(frame.data[2] - 40); // Invert the +40 offset

    String bmsPrefix = (canId >= CAN_ID_BMS1_BASE && canId <= (CAN_ID_BMS1_BASE + 3)) ? "battery48_1_" : "battery48_2_";
    send_data(bmsPrefix + "maxTemp", bmsData->maxTemp);
    send_data(bmsPrefix + "minTemp", bmsData->minTemp);

    Serial.printf("    Decoded 0x%03X: TMax=%d°C, TMin=%d°C\n",
                  canId, bmsData->maxTemp, bmsData->minTemp);
}

void decodeCanFrame4804(CanFrame& frame, BmsData48V* bmsData, uint32_t canId) { // Changed BmsData24V* to BmsData48V*
    if (frame.data_length_code < 8) return;

    bmsData->errorStatus = 0;
    for (int i = 0; i < 8; i++) {
        bmsData->errorStatus |= ((uint64_t)frame.data[i] << (8 * i)); // Reconstruct the uint64_t
    }

    String bmsPrefix = (canId >= CAN_ID_BMS1_BASE && canId <= (CAN_ID_BMS1_BASE + 3)) ? "battery48_1_" : "battery48_2_";
    // Assuming you want to send the error status as a hexadecimal string or integer.
    // Sending as float might lose precision for large uint64_t.
    // For simplicity, sending as a float for now, but consider sending as a String or splitting.
    send_data(bmsPrefix + "errorStatus", (float)bmsData->errorStatus); 


    Serial.printf("    Decoded 0x%03X: Errors=0x%llX\n", canId, bmsData->errorStatus);
}

// New decoding function for min/max temperature frames (0x009, 0x010)
// This function updates the specific DecodedTemperatureData instance passed to it.
void decodeCanFrameMinMaxTemp(CanFrame& frame, DecodedTemperatureData* tempData, uint32_t canId) {
    if (frame.data_length_code < 4) return;

    // Temperatures are encoded as uint16_t (x100), two bytes per value
    tempData->minTempCalculated = (frame.data[0] << 8) | frame.data[1];
    tempData->maxTempCalculated = (frame.data[2] << 8) | frame.data[3];
    tempData->dataReceived = true; // Mark that data has been received

    // Send to Raspberry Pi, differentiating between BMS 1 and BMS 2 thermistors
    String tempPrefix = (canId == CAN_ID_TEMP1_MINMAX) ? "thermistance48_1_" : "thermistance48_2_";
    send_data(tempPrefix + "minTemp", tempData->minTempCalculated / 100.0); // Corrected division by 100.0
    send_data(tempPrefix + "maxTemp", tempData->maxTempCalculated / 100.0); // Corrected division by 100.0

    Serial.printf("    Decoded 0x%03X: Calculated Min Temp=%.2f°C, Max Temp=%.2f°C\n",
                  canId, tempData->minTempCalculated / 100.0, tempData->maxTempCalculated / 100.0);
}


// --- FONCTIONS DE DÉCODAGE THERMISTANCES (CAN) ---
// This function decodes temperatures for the standalone 24V thermistor module (ID 0x011).
void decodeThermistance24V(CanFrame& frame) {
    if (frame.data_length_code < 4) return;

    thermistance24V.minTempCalculated = (frame.data[0] << 8) | frame.data[1];
    thermistance24V.maxTempCalculated = (frame.data[2] << 8) | frame.data[3];
    thermistance24V.dataReceived = true;

    send_data("thermistance24_minTemp", thermistance24V.minTempCalculated / 100.0);
    send_data("thermistance24_maxTemp", thermistance24V.maxTempCalculated / 100.0);

    Serial.printf("Thermistance 24V - Min:%.2f°C, Max:%.2f°C\n",
                  thermistance24V.minTempCalculated / 100.0,
                  thermistance24V.maxTempCalculated / 100.0);
}

// Removed the old decodeThermistance48V(CanFrame& frame) function
// because it was designed for a single instance and is superseded by
// decodeCanFrameMinMaxTemp which takes a specific DecodedTemperatureData*


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

    Serial.printf("Motor - Throttle:%d, CtrlTemp:%d°C, MotorTemp:%d°C, Cmd:%s\n",
                  controllerData.throttle_raw, controllerData.controller_temp,
                  controllerData.motor_temp, controllerData.command);
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
        // --- BMS 24V (réponses aux requêtes) ---
        case CAN_ID_BMS24V_RESP_1: // Using the renamed response IDs for clarity
            processReceivedData1890(frame.data, frame.identifier);
            break;
        case CAN_ID_BMS24V_RESP_2:
            processReceivedData1891(frame.data, frame.identifier);
            break;
        case CAN_ID_BMS24V_RESP_3:
            processReceivedData1892(frame.data, frame.identifier);
            break;
        case CAN_ID_BMS24V_RESP_4:
            processReceivedData1898(frame.data, frame.identifier);
            break;

        // --- BMS 48V #1 (votre premier BMS 48V) ---
        // Using CAN_ID_BMS1_BASE and its offsets
        case CAN_ID_BMS1_BASE + 0: // 0x001
            decodeCanFrame4801(frame, &bmsData48V_1, frame.identifier);
            break;
        case CAN_ID_BMS1_BASE + 1: // 0x002
            decodeCanFrame4802(frame, &bmsData48V_1, frame.identifier);
            break;
        case CAN_ID_BMS1_BASE + 2: // 0x003
            decodeCanFrame4803(frame, &bmsData48V_1, frame.identifier);
            break;
        case CAN_ID_BMS1_BASE + 3: // 0x004
            decodeCanFrame4804(frame, &bmsData48V_1, frame.identifier);
            break;

        // --- BMS 48V #2 (votre deuxième BMS 48V) ---
        // Using CAN_ID_BMS2_BASE and its offsets
        case CAN_ID_BMS2_BASE + 0: // 0x005
            decodeCanFrame4801(frame, &bmsData48V_2, frame.identifier);
            break;
        case CAN_ID_BMS2_BASE + 1: // 0x006
            decodeCanFrame4802(frame, &bmsData48V_2, frame.identifier);
            break;
        case CAN_ID_BMS2_BASE + 2: // 0x007
            decodeCanFrame4803(frame, &bmsData48V_2, frame.identifier);
            break;
        case CAN_ID_BMS2_BASE + 3: // 0x008
            decodeCanFrame4804(frame, &bmsData48V_2, frame.identifier);
            break;

        // --- Thermistances ---
        case CAN_ID_TEMP_24V_MINMAX: // Thermistors for 24V module
            decodeThermistance24V(frame);
            break;
        case CAN_ID_TEMP1_MINMAX: // Thermistors for BMS 48V #1
            decodeCanFrameMinMaxTemp(frame, &thermistance48V_1, frame.identifier);
            break;
        case CAN_ID_TEMP2_MINMAX: // Thermistors for BMS 48V #2
            decodeCanFrameMinMaxTemp(frame, &thermistance48V_2, frame.identifier);
            break;

        // --- Contrôleur moteur ---
        case CAN_ID_MOTOR_MSG1:
            decodeMotorMessage1(frame);
            break;
        case CAN_ID_MOTOR_MSG2:
            decodeMotorMessage2(frame);
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

    // BMS 24V
    Serial.println("║                           BMS 24V                        ║");
    if (bmsData24V.dataReceived) {
        Serial.printf("║ V:%.1fV | I:%.1fA | SOC:%.1f%% | T:%d°C à %d°C         ║\n",
                      bmsData24V.totalVoltage, bmsData24V.current, bmsData24V.soc,
                      bmsData24V.minTemp, bmsData24V.maxTemp);
        Serial.printf("║ Cell Max: %dmV (#%d) | Cell Min: %dmV (#%d)            ║\n",
                      bmsData24V.maxCellVoltage, bmsData24V.maxCellNumber,
                      bmsData24V.minCellVoltage, bmsData24V.minCellNumber);
        Serial.printf("║ Erreurs: 0x%llX                                        ║\n", bmsData24V.errorStatus);
    } else {
        Serial.println("║ Aucune donnée BMS 24V reçue                             ║");
    }
    Serial.println("╠════════════════════════════════════════════════════════╣");

    // BMS 48V #1
    Serial.println("║                          BMS 48V #1                      ║");
    if (bmsData48V_1.dataReceived) {
        Serial.printf("║ V:%.1fV | I:%.1fA | SOC:%.1f%% | T:%d°C à %d°C         ║\n",bmsData48V_1.totalVoltage, bmsData48V_1.current, bmsData48V_1.soc, bmsData48V_1.minTemp, bmsData48V_1.maxTemp);
        Serial.printf("║ Cell Max: %dmV (#%d) | Cell Min: %dmV (#%d)            ║\n",
                      bmsData48V_1.maxCellVoltage, bmsData48V_1.maxCellNumber,
                      bmsData48V_1.minCellVoltage, bmsData48V_1.minCellNumber);
        Serial.printf("║ Erreurs: 0x%llX                                        ║\n", bmsData48V_1.errorStatus);
        if (thermistance48V_1.dataReceived) {
            Serial.printf("║ Temp. Thermistances: %.2f°C à %.2f°C                ║\n",
                          thermistance48V_1.minTempCalculated / 100.0,
                          thermistance48V_1.maxTempCalculated / 100.0);
        } else {
            Serial.println("║ Aucune donnée thermistances 48V #1 reçue               ║");
        }
    } else {
        Serial.println("║ Aucune donnée BMS 48V #1 reçue                          ║");
    }
    Serial.println("╠════════════════════════════════════════════════════════╣");

    // BMS 48V #2
    Serial.println("║                          BMS 48V #2                      ║");
    if (bmsData48V_2.dataReceived) {
        Serial.printf("║ V:%.1fV | I:%.1fA | SOC:%.1f%% | T:%d°C à %d°C         ║\n",
                      bmsData48V_2.totalVoltage, bmsData48V_2.current, bmsData48V_2.soc,
                      bmsData48V_2.minTemp, bmsData48V_2.maxTemp);
        Serial.printf("║ Cell Max: %dmV (#%d) | Cell Min: %dmV (#%d)            ║\n",
                      bmsData48V_2.maxCellVoltage, bmsData48V_2.maxCellNumber,
                      bmsData48V_2.minCellVoltage, bmsData48V_2.minCellNumber);
        Serial.printf("║ Erreurs: 0x%llX                                        ║\n", bmsData48V_2.errorStatus);
        if (thermistance48V_2.dataReceived) {
            Serial.printf("║ Temp. Thermistances: %.2f°C à %.2f°C                ║\n",
                          thermistance48V_2.minTempCalculated / 100.0,
                          thermistance48V_2.maxTempCalculated / 100.0);
        } else {
            Serial.println("║ Aucune donnée thermistances 48V #2 reçue               ║");
        }
    } else {
        Serial.println("║ Aucune donnée BMS 48V #2 reçue                          ║");
    }
    Serial.println("╠════════════════════════════════════════════════════════╣");

    // Thermistances 24V (celles du module externe, si elles sont envoyées sur CAN 0x011)
    Serial.println("║                      THERMISTANCES 24V                  ║");
    if (thermistance24V.dataReceived) {
        Serial.printf("║ Min:%.2f°C | Max:%.2f°C                                ║\n",
                      thermistance24V.minTempCalculated / 100.0,
                      thermistance24V.maxTempCalculated / 100.0);
    } else {
        Serial.println("║ Aucune donnée thermistances 24V reçue                   ║");
    }
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

    Serial.println("╚════════════════════════════════════════════════════════╝\n");
}

// --- SETUP ET LOOP ---
void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for serial connection to be established

    // Initialisation UART Raspberry
    // Set up UART1 for communication with Raspberry Pi
    RaspberrySerial.begin(115200, SERIAL_8N1, RX_PIN_RASPBERRY, TX_PIN_RASPBERRY); // Use renamed pins
    Serial.println("UART Raspberry initialisé.");

    // Configuration CAN
    // Set up CAN bus pins and queue sizes
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    ESP32Can.setRxQueueSize(30); // Increased Rx queue size for more robust reception
    ESP32Can.setTxQueueSize(30); // Increased Tx queue size for more robust transmission
    
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

    Serial.println("=== RÉCEPTEUR CAN UNIFIÉ DÉMARRÉ ===");
    Serial.println("Équipements supportés et IDs CAN:");
    Serial.printf("- BMS 24V (IDs: 0x%08X-0x%08X)\n", CAN_ID_BMS24V_RESP_1, CAN_ID_BMS24V_RESP_4); // Using renamed IDs
    Serial.printf("- BMS 48V #1 (IDs: 0x%03X-0x%03X)\n", CAN_ID_BMS1_BASE, CAN_ID_BMS1_BASE + 3);
    Serial.printf("- BMS 48V #2 (IDs: 0x%03X-0x%03X)\n", CAN_ID_BMS2_BASE, CAN_ID_BMS2_BASE + 3);
    Serial.printf("- Thermistances 24V (ID: 0x%03X)\n", CAN_ID_TEMP_24V_MINMAX);
    Serial.printf("- Thermistances 48V #1 (ID: 0x%03X)\n", CAN_ID_TEMP1_MINMAX); // Corrected ID usage
    Serial.printf("- Thermistances 48V #2 (ID: 0x%03X)\n", CAN_ID_TEMP2_MINMAX); // Corrected ID usage
    Serial.printf("- Contrôleur moteur (IDs: 0x%08X, 0x%08X)\n", CAN_ID_MOTOR_MSG1, CAN_ID_MOTOR_MSG2);
    Serial.println("En attente de trames CAN...\n");
}

void loop() {
    // Read all available CAN frames in a non-blocking manner
    // The `0` as the second argument means no timeout, it reads immediately.
    // rxFrame is declared globally, so no need to redeclare here.
    while (ESP32Can.readFrame(rxFrame, 0)) {
        processReceivedCanFrame(rxFrame);
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
