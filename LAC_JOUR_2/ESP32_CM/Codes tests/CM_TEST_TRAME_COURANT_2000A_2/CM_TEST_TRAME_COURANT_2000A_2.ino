#include <Wire.h>
#include <ESP32-TWAI-CAN.hpp>
#define CAN_TX 5
#define CAN_RX 4
CanFrame rxFrame;

#define SLAVE_ADDRESS 0x08  // Adresse I2C de l'esclave

// Définition des pins pour les thermistances
const int thermistorPins[10] = {34, 35, 32, 33, 25, 27, 26, 14, 12, 13};
const int numThermistors = 10;
const float seriesResistor = 10000.0;           // Résistance fixe (10kΩ)
const float nominalResistance = 10000.0;        // NTC à 25°C (10kΩ)
const float nominalTemperature = 25.0;          // en °C
const float bCoefficient = 3950;                // Coefficient B
const int adcMax = 4095;                        // ADC 12 bits ESP32

// Structure pour les données des thermistances
struct ThermistorData {
  float temperatures[10];        // Températures en °C
};

ThermistorData thermistorData;

// Structure complète reçue depuis le maître
struct BatteryRaw {
  float chargeVoltage;
  float chargeCurrentLimit;
  float dischargeCurrentLimit;
  uint16_t soc;
  uint16_t soh;
  float moduleVoltage;
  float totalCurrent;
  float avgTemperature;
  uint8_t prot1;
  uint8_t prot2;
  uint8_t alarm1;
  uint8_t alarm2;
  uint8_t moduleCount;
  uint8_t flags;
};
BatteryRaw bmsData48V;

// Structure globale pour stocker les données BMS
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

// Instanciation globale
MotorControllerData controllerData;

// Identifiants CAN pour l'envoi des données des thermistances
const uint32_t THERMISTOR_DATA1_ID = 0x0CF12E05;  // Pour les températures 0, 1, 2, 3
const uint32_t THERMISTOR_DATA2_ID = 0x0CF12F05;  // Pour les températures 4,5,6,7
const uint32_t THERMISTOR_DATA3_ID = 0x0CF13005; /// Pour les températures 8,9


//Fonction qui stocke les données recues par i2C de la batterie 48V directement dans la structure 
void receiveEvent(int numBytes) {
  if (numBytes == sizeof(BatteryRaw)) {
    Wire.readBytes((char*)&bmsData48V, sizeof(BatteryRaw));
    
    // Affichage dans le moniteur série
    Serial.println("<< Données batterie reçues :");
    Serial.print("  Charge voltage: "); Serial.println(bmsData48V.chargeVoltage);
    Serial.print("  Charge current limit: "); Serial.println(bmsData48V.chargeCurrentLimit);
    Serial.print("  Discharge current limit: "); Serial.println(bmsData48V.dischargeCurrentLimit);
    Serial.print("  SOC: "); Serial.println((float)bmsData48V.soc, 1);
    Serial.print("  SOH: "); Serial.println((float)bmsData48V.soc, 1);
    Serial.print("  Module voltage: "); Serial.println(bmsData48V.moduleVoltage);
    Serial.print("  Total current: "); Serial.println(bmsData48V.totalCurrent);
    Serial.print("  Avg temperature: "); Serial.println(bmsData48V.avgTemperature);
    Serial.print("  Protection1: "); Serial.println(bmsData48V.prot1);
    Serial.print("  Protection2: "); Serial.println(bmsData48V.prot2);
    Serial.print("  Alarm1: "); Serial.println(bmsData48V.alarm1);
    Serial.print("  Alarm2: "); Serial.println(bmsData48V.alarm2);
    Serial.print("  Module count: "); Serial.println(bmsData48V.moduleCount);
    Serial.print("  Flags: "); Serial.println(bmsData48V.flags);
    Serial.println("--------------------------");
  } else {
    Serial.print("Erreur : ");
    Serial.print(numBytes);
    Serial.println(" octets reçus au lieu de ");
    Serial.println(sizeof(BatteryRaw));
  }
}

// Fonction pour reformer et envoyer les trames CAN
void reformAndSendCANFrames() {
    CanFrame frame;
    
    Serial.println("Je suis dans le boucle");
    // Trame pour 0x355
    frame.identifier = 0x355;
    frame.extd = 0;
    frame.data_length_code = 8;
    frame.data[0] = bmsData48V.soc & 0xFF;
    frame.data[1] = (bmsData48V.soc >> 8) & 0xFF;
    frame.data[2] = bmsData48V.soh & 0xFF;
    frame.data[3] = (bmsData48V.soh >> 8) & 0xFF;
    bool ok = ESP32Can.writeFrame(frame, 1000);
if (!ok) {
  Serial.print("Échec envoi trame ID 0x");
  Serial.println(frame.identifier, HEX);
}
else{
  Serial.print("Énvoyee");
}

    // Trame pour 0x356
    frame.identifier = 0x356;
    int16_t moduleVoltage = (int16_t)(bmsData48V.moduleVoltage * 100);
    int16_t totalCurrent = (int16_t)(bmsData48V.totalCurrent * 10);
    int16_t avgTemperature = (int16_t)(bmsData48V.avgTemperature * 10);
    frame.data[0] = moduleVoltage & 0xFF;
    frame.data[1] = (moduleVoltage >> 8) & 0xFF;
    frame.data[2] = totalCurrent & 0xFF;
    frame.data[3] = (totalCurrent >> 8) & 0xFF;
    frame.data[4] = avgTemperature & 0xFF;
    frame.data[5] = (avgTemperature >> 8) & 0xFF;
    ESP32Can.writeFrame(frame, 1000);

    // Trame pour 0x351
    frame.identifier = 0x351;
    int16_t chargeVoltage = (int16_t)(bmsData48V.chargeVoltage * 10);
    int16_t chargeCurrentLimit = (int16_t)(bmsData48V.chargeCurrentLimit * 10);
    int16_t dischargeCurrentLimit = (int16_t)(bmsData48V.dischargeCurrentLimit * 10);
    frame.data[0] = chargeVoltage & 0xFF;
    frame.data[1] = (chargeVoltage >> 8) & 0xFF;
    frame.data[2] = chargeCurrentLimit & 0xFF;
    frame.data[3] = (chargeCurrentLimit >> 8) & 0xFF;
    frame.data[4] = dischargeCurrentLimit & 0xFF;
    frame.data[5] = (dischargeCurrentLimit >> 8) & 0xFF;
    ESP32Can.writeFrame(frame, 1000);

    // Trame pour 0x359
    frame.identifier = 0x359;
    frame.data[0] = bmsData48V.prot1;
    frame.data[1] = bmsData48V.prot2;
    frame.data[2] = bmsData48V.alarm1;
    frame.data[3] = bmsData48V.alarm2;
    frame.data[4] = bmsData48V.moduleCount;
    ESP32Can.writeFrame(frame, 1000);
}



//Fonctions pour recevoir et stocker les données du CM dans la structure controllerData
void decodeMessage1(const CanFrame &frame) {
    controllerData.speed_rpm = (frame.data[1] << 8) | frame.data[0];
    controllerData.motor_current = ((frame.data[3] << 8) | frame.data[2]) / 10.0;
        Serial.print("TRAME RECUE MOTEUR CURRENT : ");
    Serial.print(frame.data[3], BIN);
    Serial.print(" | ");
    Serial.print(frame.data[2], BIN);
    Serial.print(" | ");
    Serial.print(controllerData.motor_current, BIN);
    Serial.print("\n");


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

//Fonction pour demander l'envoie des trames au BMS 24V 
/*
void sendBms24VRequest(uint32_t identifier) {
  CanFrame frame;
  frame.identifier = identifier;
  frame.extd = 1;
  frame.data_length_code = 8;
  memset(frame.data, 0, 8);

  if (ESP32Can.writeFrame(frame, 1000)) {
    Serial.print("Requête envoyée : ID = 0x");
    Serial.println(identifier, HEX);
  } else {
    Serial.println("Échec de l'envoi de la requête CAN");
  }
}

//Foncion pour recevoir trames de réponse du BMS 24V 
bool receiveBmsResponse(uint32_t expectedID) {
  CanFrame frame;
  if (ESP32Can.readFrame(frame, 1000)) {
    if (frame.identifier == expectedID && frame.data_length_code == 8) {
      processReceivedData(frame.data, frame.identifier);
      return true;
    }
  }
  return false;
}

//Fonction de décodage des trames recues du BMS 24V 
void processReceivedData(uint8_t* data, uint32_t identifier) {
  switch (identifier) {
    case 0x18904001:
      bmsData24V.totalVoltage = ((data[0] << 8) | data[1]) * 0.1;
      bmsData24V.gatherVoltage = ((data[2] << 8) | data[3]) * 0.1;
      bmsData24V.current = (((data[4] << 8) | data[5]) - 30000) * 0.1;
      bmsData24V.soc = (data[6] == 0xFF && data[7] == 0xFF) ? -1 : ((data[6] << 8) | data[7]) * 0.1;
      break;

    case 0x18914001:
      bmsData24V.maxCellVoltage = (data[0] << 8) | data[1];
      bmsData24V.maxCellNumber = data[2];
      bmsData24V.minCellVoltage = (data[3] << 8) | data[4];
      bmsData24V.minCellNumber = data[5];
      break;

    case 0x18924001:
      bmsData24V.maxTemp = data[0] - 40;
      bmsData24V.minTemp = data[2] - 40;
      break;

    case 0x18984001:
      bmsData24V.errorStatus = 0;
      for (int i = 0; i < 8; i++) {
        bmsData24V.errorStatus |= ((uint64_t)data[i] << (8 * i));
      }
      break;
  }
}
*/
// Fonction pour lire les valeurs des thermistances et mettre à jour la structure
void readThermistors() {
  Serial.println("\n--- Lecture des thermistances ---");
  
  for (int i = 0; i < numThermistors; i++) {
    int pin = thermistorPins[i];
    uint16_t adcValue = analogRead(pin);
    float voltage = adcValue * 3.3 / adcMax;
    
    if (voltage < 0.01) {
      // Tension trop basse, probablement un problème de câblage
      Serial.printf("Thermistance %d (pin %d): Tension trop basse (%.3f V), vérifie le câblage!\n", 
                    i, pin, voltage);
      thermistorData.temperatures[i] = -273.15;  // Valeur absurde pour indiquer une erreur
      continue;
    }
    
    // Calcul de la résistance
    float resistance = seriesResistor * (voltage / (3.3 - voltage));
    // Equation de Steinhart-Hart pour calculer la température
    float steinhart = resistance / nominalResistance;
    steinhart = log(steinhart);
    steinhart /= bCoefficient;
    steinhart += 1.0 / (nominalTemperature + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    
    thermistorData.temperatures[i] = steinhart;
  }
}
/*
// Fonction pour envoyer les données des thermistances via CAN
void sendThermistorDataCAN() {
  CanFrame frame;
  frame.extd = 1;
  frame.data_length_code = 8;
  
  // Premier message avec les températures 0-4
  frame.identifier = THERMISTOR_DATA1_ID;
  
  // Conversion des températures en entiers (multiplié par 10 pour conserver une décimale)
  int16_t temp0 = (int16_t)(thermistorData.temperatures[0] * 10);
  int16_t temp1 = (int16_t)(thermistorData.temperatures[1] * 10);
  int16_t temp2 = (int16_t)(thermistorData.temperatures[2] * 10);
  int16_t temp3 = (int16_t)(thermistorData.temperatures[3] * 10);
  
  frame.data[0] = temp0 & 0xFF;
  frame.data[1] = (temp0 >> 8) & 0xFF;
  frame.data[2] = temp1 & 0xFF;
  frame.data[3] = (temp1 >> 8) & 0xFF;
  frame.data[4] = temp2 & 0xFF;
  frame.data[5] = (temp2 >> 8) & 0xFF;
  frame.data[6] = temp3 & 0xFF;
  frame.data[7] = (temp3 >> 8) & 0xFF;
  
  if (ESP32Can.writeFrame(frame, 1000)) {
    Serial.printf("Envoi données thermistances 0-3: ID = 0x%08X\n", THERMISTOR_DATA1_ID);
  } else {
    Serial.println("Échec de l'envoi des données thermistances 0-3");
  }
  
  // Deuxième message avec les températures 4-7
  frame.identifier = THERMISTOR_DATA2_ID;
  
  int16_t temp4 = (int16_t)(thermistorData.temperatures[4] * 10);
  int16_t temp5 = (int16_t)(thermistorData.temperatures[5] * 10);
  int16_t temp6 = (int16_t)(thermistorData.temperatures[6] * 10);
  int16_t temp7 = (int16_t)(thermistorData.temperatures[7] * 10);
  
  frame.data[0] = temp4 & 0xFF;
  frame.data[1] = (temp4 >> 8) & 0xFF;
  frame.data[2] = temp5 & 0xFF;
  frame.data[3] = (temp5 >> 8) & 0xFF;
  frame.data[4] = temp6 & 0xFF;
  frame.data[5] = (temp6 >> 8) & 0xFF;
  frame.data[6] = temp7 & 0xFF;
  frame.data[7] = (temp7 >> 8) & 0xFF;
  
  if (ESP32Can.writeFrame(frame, 1000)) {
    Serial.printf("Envoi données thermistances 4-7: ID = 0x%08X\n", THERMISTOR_DATA2_ID);
  } else {
    Serial.println("Échec de l'envoi des données thermistances 4-7");
  }
  
  // Troisième message avec les températures 8-9 et min/max
  frame.identifier = THERMISTOR_DATA3_ID;
  
  int16_t temp8 = (int16_t)(thermistorData.temperatures[8] * 10);
  int16_t temp9 = (int16_t)(thermistorData.temperatures[9] * 10);
  
  frame.data[0] = temp8 & 0xFF;
  frame.data[1] = (temp8 >> 8) & 0xFF;
  frame.data[2] = temp9 & 0xFF;
  frame.data[3] = (temp9 >> 8) & 0xFF;

  
  if (ESP32Can.writeFrame(frame, 1000)) {
    Serial.printf("Envoi données thermistances 8-9 et résumé: ID = 0x%08X\n", THERMISTOR_DATA3_ID);
  } else {
    Serial.println("Échec de l'envoi des données thermistances 8-9 et résumé");
  }
}*/

void setup() {
  Serial.begin(115200);
  
  // Configuration des pins des thermistances comme entrées
  for (int i = 0; i < numThermistors; i++) {
    pinMode(thermistorPins[i], INPUT);
  }
  
  Wire.begin(SLAVE_ADDRESS);  
  Wire.onReceive(receiveEvent);
  Serial.println("Esclave I2C prêt à recevoir les données...");
   
  //Initialisation CAN 
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

  if(ESP32Can.begin()) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("fCAN bus failed!");
  }
}

void loop() {
  // Lecture des thermistances
  
  
  // Envoie des demandes de trames au BMS
  /*
  sendBms24VRequest(0x18900140);
  sendBms24VRequest(0x18910140);
  sendBms24VRequest(0x18920140);
  sendBms24VRequest(0x18980140);
  Serial.println("justeavant d envoyer");
  reformAndSendCANFrames();
  delay(50); // temps pour que les réponses CAN soient transmises
  */

  // Lecture de toutes les trames entrantes
  while (ESP32Can.readFrame(rxFrame, 10)) {
    Serial.printf("\n[CAN] Received frame: 0x%08X, DLC=%d\n", rxFrame.identifier, rxFrame.data_length_code);

    switch (rxFrame.identifier) {
      // Réponses du BMS 24V
      case 0x18904001:
      case 0x18914001:
      case 0x18924001:
      case 0x18984001:
        //processReceivedData(rxFrame.data, rxFrame.identifier);
        break;

      // Trames CM
      case 0x0CF11E05:
        decodeMessage1(rxFrame);
        break;
      case 0x0CF11F05:
        decodeMessage2(rxFrame);
        break;

      // Trames inconnues 
      default:
        Serial.println("Unknown frame, printing raw data:");
        for (int i = 0; i < rxFrame.data_length_code; i++) {
          Serial.printf("Data[%d]: 0x%02X\n", i, rxFrame.data[i]);
        }
        break;
    }
  }
  
  // Ajouter une pause pour ne pas surcharger le bus CAN
  delay(1000);
}
