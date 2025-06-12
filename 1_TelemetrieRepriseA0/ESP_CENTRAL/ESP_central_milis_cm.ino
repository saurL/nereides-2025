//CODE ESP CENTRAL 
//On a pb de reception des trames de la batterie 48V avec ce code 

#include <ESP32-TWAI-CAN.hpp>
#include <ArduinoJson.h>
#include <vector>

//définition pin led alerte température
#define LED_PIN 2

// Pins CAN (à adapter selon votre configuration)
#define CAN_TX 22
#define CAN_RX 21
const int numThermistors = 10;  //nombre de sondes
float temperature[numThermistors] = {}; //stocker les valeurs des 10 sondes dans une liste
#define TX_PIN 17  // Broche TX pour UART1
#define RX_PIN 16  // Broche RX pour UART1
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


/*
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
}*/



void send_json(StaticJsonDocument<200>& json) {
    // Convertir en chaîne JSON
    String jsonString;
    serializeJson(json, jsonString);
    
    // Forcer le format attendu par le code Rust
    std::vector<uint8_t> message;
    message.push_back(0); // Premier octet (peut être n'importe quoi)
    message.push_back('{'); // Deuxième octet est '{'
    
    // Ajouter le reste du JSON après le '{'
    for (size_t i = 1; i < jsonString.length(); ++i) {
        message.push_back(static_cast<uint8_t>(jsonString[i]));
    }
    
    RaspberrySerial.write(message.data(), message.size());
}

void send_data(const String& data_name, float value) {
    StaticJsonDocument<200> json;
    json["data_name"] = data_name;
    json["value"] = value;
    // Envoi du JSON
    send_json(json);
}

void send_error(const String& error_name, const String& value) {
    StaticJsonDocument<200> json;
    json["data_name"] = error_name;
    json["value"] = value;

    // Envoi du JSON
    send_json(json);
}


CanFrame rxFrame;


uint16_t temperatures24V[10]; // array regroupant les températures des thermistances de la Batterie 24V

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

//Structure batterie 24V 
struct BmsData24V {
  float totalVoltage = -1;
  float gatherVoltage = -1;
  float current = -1;
  float soc = -1;

  uint16_t maxCellVoltage = 0;
  uint8_t maxCellNumber = 0;
  uint16_t minCellVoltage = 0;
  uint8_t minCellNumber = 0;

  int8_t maxTemp = -100;// en °C
  int8_t minTemp = -100; // en °C

  uint64_t errorStatus = 0;
};

BmsData24V bmsData24V;


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

MotorControllerData controllerData;

/////////////////////
//Methodes Batterie 48V 

void handle0x351(const CanFrame& frame) {
    battery.chargeVoltage = (frame.data[0] | (frame.data[1] << 8)) / 10.0;
    battery.chargeCurrentLimit = (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
    battery.dischargeCurrentLimit = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
    Serial.println("batterie 48 charge voltage:"); Serial.println(battery.chargeVoltage);
    Serial.println("batterie 48 charge current limit:"); Serial.println(battery.chargeCurrentLimit);
    Serial.println("batterie 48 discharge current limit:"); Serial.println(battery.dischargeCurrentLimit);
}

void handle0x355(const CanFrame& frame) {
    battery.soc = frame.data[0] | (frame.data[1] << 8);
    battery.soh = frame.data[2] | (frame.data[3] << 8);
    Serial.println(battery.soh);
    send_data("battery_soc", battery.soc);
    send_data("battery_soh", battery.soh);
    Serial.println("batterie 48 SOC:"); Serial.println(battery.soc);
    Serial.println("batterie 48 SOH:"); Serial.println(battery.soh);
}

void handle0x356(const CanFrame& frame) {
    battery.moduleVoltage = (int16_t)(frame.data[0] | (frame.data[1] << 8)) / 100.0;
    battery.totalCurrent = (int16_t)(frame.data[2] | (frame.data[3] << 8)) / 10.0;
    battery.avgTemperature = (int16_t)(frame.data[4] | (frame.data[5] << 8)) / 10.0;
    send_data("battery_temp", battery.avgTemperature );
    send_data("battery_voltage_v", battery.moduleVoltage );
    send_data("battery_current_a", battery.totalCurrent  );
    Serial.println("batterie 48 module voltage:"); Serial.println(battery.moduleVoltage);
    Serial.println("batterie 48 total current:"); Serial.println(battery.totalCurrent);
    Serial.println("batterie 48 température moyenne:"); Serial.println(battery.avgTemperature);


}

void handle0x359(const CanFrame& frame) {
    battery.prot1 = frame.data[0];
    battery.prot2 = frame.data[1];
    battery.alarm1 = frame.data[2];
    battery.alarm2 = frame.data[3];
    battery.moduleCount = frame.data[4];
    Serial.println("batterie 48 protection 1:"); Serial.println(battery.prot1);
    Serial.println("batterie 48 protection 1:"); Serial.println(battery.prot2);
    Serial.println("batterie 48 alarme 1:"); Serial.println(battery.alarm1);
    Serial.println("batterie 48 alarme 2:"); Serial.println(battery.alarm2);
    Serial.println("batterie 48 Module Count:"); Serial.println(battery.moduleCount);
    
}

void handle0x35C(const CanFrame& frame) {
    battery.flags = frame.data[0];
}

//Méthodes batterie 24V : 
///////////////////////////////////
//Fonction de décodage des trames recues du BMS 24V 
//Fonction pour demander l'envoie des trames au BMS 24V 

//Fonction de décodage des trames recues du BMS 24V 
void processReceivedData1890(uint8_t* data, uint32_t identifier) {
      bmsData24V.totalVoltage = ((data[0] << 8) | data[1]) * 0.1;
      bmsData24V.gatherVoltage = ((data[2] << 8) | data[3]) * 0.1;
      bmsData24V.current = (((data[4] << 8) | data[5]) - 30000) * 0.1;
      bmsData24V.soc = (data[6] == 0xFF && data[7] == 0xFF) ? -1 : ((data[6] << 8) | data[7]) * 0.1;
      send_data("battery24_soc", bmsData24V.soc);
      send_data("battery24_voltage",bmsData24V.totalVoltage);
      send_data("battery24_current",bmsData24V.current );
      Serial.println("SOC");
      Serial.println(bmsData24V.soc);
      
}
void processReceivedData1891(uint8_t* data, uint32_t identifier) {
     Serial.println("-> processReceivedData1891");
      bmsData24V.maxCellVoltage = (data[0] << 8) | data[1];
      bmsData24V.maxCellNumber = data[2];
      bmsData24V.minCellVoltage = (data[3] << 8) | data[4];
      bmsData24V.minCellNumber = data[5];
      send_data("battery24_maxCellVoltage", bmsData24V.maxCellVoltage);
      send_data("battery24_minCellVoltage", bmsData24V.minCellVoltage);
      Serial.println("CellVolt");
      Serial.println(bmsData24V.maxCellVoltage);
      Serial.flush();
}
void processReceivedData1892(uint8_t* data, uint32_t identifier) {
      bmsData24V.maxTemp = data[0] - 40;
      bmsData24V.minTemp = data[2] - 40;
      send_data("battery24_maxTemp", bmsData24V.maxTemp );
      send_data("battery24_minTemp", bmsData24V.minTemp);
      Serial.println("max_temp");
      Serial.println(bmsData24V.maxTemp);
      Serial.println("min_temp");
      Serial.println(bmsData24V.minTemp);
}

   void processReceivedData1898(uint8_t* data, uint32_t identifier) {
      bmsData24V.errorStatus = 0;
      for (int i = 0; i < 8; i++) {
        bmsData24V.errorStatus |= ((uint64_t)data[i] << (8 * i));
      }
    
  }
/////////////////////
// méthodes controleur moteur 
void decodeMessage1(const CanFrame &frame) {
    static unsigned long lastPrint = 0;
    const unsigned long interval = 2000;

    controllerData.speed_rpm = (frame.data[1] << 8) | frame.data[0];
    controllerData.motor_current = ((frame.data[3] << 8) | frame.data[2]) / 10.0;
    controllerData.battery_voltage = ((frame.data[5] << 8) | frame.data[4]) / 10.0;
    controllerData.error_code = (frame.data[7] << 8) | frame.data[6];

    if (millis() - lastPrint >= interval) {
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
        lastPrint = millis();
    }

    send_data("motor_rpm", controllerData.speed_rpm);
    send_data("mottor_current_a", controllerData.motor_current);
    send_data("motor_error_code", controllerData.error_code);
}

void decodeMessage2(const CanFrame &frame) {
   static unsigned long lastPrint = 0;
    const unsigned long interval = 2000;

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

    if (millis() - lastPrint >= interval) {

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
        lastPrint = millis();
    }

    send_data("motor_throttle", controllerData.throttle_raw);
    send_data("motor_controller_temp", controllerData.controller_temp);
    send_data("motor_temp", controllerData.motor_temp);

}

void minmax(const uint16_t arr[], int taille, uint16_t minMax[]) {
    if (taille == 0) return; // Sécurité : tableau vide

    uint16_t minVal = arr[0];
    uint16_t maxVal = arr[0];

    for (size_t i = 1; i < taille; ++i) {
        if (arr[i] < minVal) minVal = arr[i];
        if (arr[i] > maxVal) maxVal = arr[i];
    }

    minMax[0] = minVal;
    minMax[1] = maxVal;
}


void decodeTemperature24V(const CanFrame &frame) {
    uint16_t temperature = (frame.data[0] << 8) | frame.data[1];
    uint8_t idTrame = frame.identifier - 0x600;
    temperatures24V[idTrame] = temperature;
    if (idTrame == 9) {
        uint16_t minMax[2];
        minmax(temperatures24V, 9, minMax);
        send_data("battery24_therm_max", minMax[0]);
        send_data("battery24_therm_min", minMax[1]);
    }
    Serial.printf("Thermistance numero %u, la temperature est de %u", idTrame, temperature);
}


void setup() {
    // Setup serial pour le débogage
    Serial.begin(115200);
    while (!Serial); // Attendre que le port série soit prêt
    RaspberrySerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
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

    // Vérification des seuils pour allumer la LED
    if (battery.avgTemperature > 54)           // **Condition pour batterie**
        { 
        digitalWrite(LED_PIN, HIGH);                     // **Allumer la LED**
    } else {
        digitalWrite(LED_PIN, LOW);                      // **Éteindre la LED**
    }

    
    if (controllerData.controller_temp > 63)// **Condition pour contrôleur moteur**
        { 
        digitalWrite(LED_PIN, HIGH);                     // **Allumer la LED**
    } else {
        digitalWrite(LED_PIN, LOW);                      // **Éteindre la LED**
        }

     if (bmsData24V.minTemp > 54)// **temp thermi**
        { 
        digitalWrite(LED_PIN, HIGH);                     // **Allumer la LED**
    } else {
        digitalWrite(LED_PIN, LOW);                      // **Éteindre la LED**
        }

    if (bmsData24V.maxTemp > 54)// **temp thermi**
        { 
        digitalWrite(LED_PIN, HIGH);                     // **Allumer la LED**
    } else {
        digitalWrite(LED_PIN, LOW);                      // **Éteindre la LED**
        }
       
    while (ESP32Can.readFrame(rxFrame, 0)) {  // Pas de délai d’attente
    Serial.printf("\n[CAN] Received frame: 0x%08X, DLC=%d\n", rxFrame.identifier, rxFrame.data_length_code);
        
        switch (rxFrame.identifier) {
      
      //MESSAGE CANBATTERIE 24V
    case 0x18904001:
        processReceivedData1890(rxFrame.data, rxFrame.identifier);
        break;
      case 0x18914001:
        processReceivedData1891(rxFrame.data, rxFrame.identifier);
        break;
      case 0x18924001:
        processReceivedData1892(rxFrame.data, rxFrame.identifier);
        break;
      case 0x18984001:
        processReceivedData1898(rxFrame.data, rxFrame.identifier);
        break;
          //MESSAGE CAN BATTERIE 48V 
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
    //MESSAGE CAN CM   
    case 0x0CF11E05:    //msg 1 du cm 
          decodeMessage1(rxFrame);
          break;
    case 0x0CF11F05:    //msg 2 du cm 
          decodeMessage2(rxFrame);
          break;
    default:
      Serial.println("Unknown frame, printing raw data:");
                for (int i = 0; i < rxFrame.data_length_code; i++) {
                    Serial.printf("Data[%d]: 0x%02X\n", i, rxFrame.data[i]);
                }
                break;
        }
    }
    
}
