#include <Wire.h>
#include <ESP32-TWAI-CAN.hpp>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define CAN_TX 4
#define CAN_RX 5
#define SLAVE_ADDRESS 0x08  // Adresse I2C de l'esclave
// Contrôleur moteur (identifiants existants)
#define CAN_ID_MOTOR_MSG1 0x0CF11E05
#define CAN_ID_MOTOR_MSG2 0x0CF11F05
//configuration sonde température globale 
#define TEMP_SENSOR_PIN 13
#define CAN_ID_TEMP_SENSOR 0x013
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature ds(&oneWire);

CanFrame rxFrame, txFrame;


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

void decodeMotorMessage1(const CanFrame &frame) {
    controllerData.speed_rpm = (frame.data[1] << 8) | frame.data[0];
    controllerData.motor_current = (((frame.data[3] << 8) | frame.data[2]) / 10.0) - 2000;
    controllerData.battery_voltage = ((frame.data[5] << 8) | frame.data[4]) / 10.0;
    controllerData.error_code = (frame.data[7] << 8) | frame.data[6];
    
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

    Serial.print("ENVOIE CM OK : THROTTLE TEMP TEMP MOTOR");
    Serial.printf("Motor - Throttle:%d, CtrlTemp:%d°C, MotorTemp:%d°C, Cmd:%s\n",
                  controllerData.throttle_raw, controllerData.controller_temp,
                  controllerData.motor_temp, controllerData.command);
}


void sendTemperatureViaCAN(float temperature) {
    // Vérifier si la température est valide (DS18B20 retourne -127 en cas d'erreur)
    if (temperature == -127.0 ) {
        Serial.println("Erreur lecture température - trame CAN non envoyée");
        return;
    }
    
    txFrame.identifier = CAN_ID_TEMP_SENSOR;
    txFrame.extd = 0;  // Trame standard (11 bits)
    txFrame.data_length_code = 4;  // 4 bytes de données
    
    // Encoder la température en centièmes de degré (16 bits)
    int16_t tempCentidegreees = (int16_t)(temperature * 100);
    
    // Remplir les données (little-endian)
    txFrame.data[0] = tempCentidegreees & 0xFF;         // LSB
    txFrame.data[1] = (tempCentidegreees >> 8) & 0xFF;  // MSB
    txFrame.data[2] = 0x00;  // Réservé
    txFrame.data[3] = 0x00;  // Réservé
    
    // Envoyer la trame CAN
    if (ESP32Can.writeFrame(txFrame)) {
        Serial.printf("Température envoyée via CAN: %.2f°C (ID: 0x%03X)\n", 
                      temperature, CAN_ID_TEMP_SENSOR);
        
        // Debug: afficher les bytes envoyés
        Serial.printf("Données CAN: %02X %02X %02X %02X\n", 
                      txFrame.data[0], txFrame.data[1], 
                      txFrame.data[2], txFrame.data[3]);
    } else {
        Serial.println("Erreur: Échec envoi trame CAN température");
    }
}

void readAndSendTemperature() {
    ds.requestTemperatures();
    float temperature = ds.getTempCByIndex(0);
    
    Serial.printf("Température lue: %.2f°C\n", temperature);
    
    // Envoyer via CAN
    sendTemperatureViaCAN(temperature);
}


// Fonction qui stocke les données reçues par I2C de la batterie 48V directement dans la structure
void receiveEvent(int numBytes) {
    if (numBytes != sizeof(BatteryRaw)) {
    Serial.printf("Erreur: %d octets attendus %d -> rejetés\n", numBytes, sizeof(BatteryRaw));
    while (Wire.available()) Wire.read();  // vider le buffer
    return;
  }

  Wire.readBytes((char*)&bmsData48V, sizeof(BatteryRaw));

    // Affichage dans le moniteur série
    Serial.println("<< Données batterie reçues :");
    Serial.print("  Charge voltage: "); Serial.println(bmsData48V.chargeVoltage);
    Serial.print("  Charge current limit: "); Serial.println(bmsData48V.chargeCurrentLimit);
    Serial.print("  Discharge current limit: "); Serial.println(bmsData48V.dischargeCurrentLimit);
    Serial.print("  SOC: "); Serial.println(bmsData48V.soc);
    Serial.print("  SOH: "); Serial.println(bmsData48V.soh);
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

    // Reformer et envoyer les trames CAN
}

// Fonction pour reformer et envoyer les trames CAN
void reformAndSendCANFrames() {
    CanFrame frame;

    // Trame pour 0x355
    frame.identifier = 0x355;
    frame.extd = 0;
    frame.data_length_code = 8;
    frame.data[0] = bmsData48V.soc & 0xFF;
    frame.data[1] = (bmsData48V.soc >> 8) & 0xFF;
    frame.data[2] = bmsData48V.soh & 0xFF;
    frame.data[3] = (bmsData48V.soh >> 8) & 0xFF;
    ESP32Can.writeFrame(frame, 1000);

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



void setup() {
  Serial.begin(115200);


  ds.begin();
  Wire.begin(SLAVE_ADDRESS);  //pin ok
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
    Serial.println("CAN bus failed!");
  }
}

void loop() {
 
    reformAndSendCANFrames();
    readAndSendTemperature();
    


  delay(1000);
}
