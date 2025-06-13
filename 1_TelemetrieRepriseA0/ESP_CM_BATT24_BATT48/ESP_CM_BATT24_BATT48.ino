//CMMM QUE JE MODIFIE AVEC LES FONCTIONS POUR BATTERIE 24V 
#include <Wire.h>
#include <ESP32-TWAI-CAN.hpp>
#define CAN_TX 4 // PAR PITIE MODIFIEZ PLUS CA A LAIDE ON A PASSE 15 MIN A CHERCHER CE QUI DECONNAIS
#define CAN_RX 5 // NE PAS LES CONFONDRES TX = 4 RX = D5 VRAIMENT
CanFrame rxFrame;

#define SLAVE_ADDRESS 0x08  // Adresse I2C de l'esclave


class Multiplexer {
    public:        
        int muxS0;
        int muxS1;
        int muxS2;
        int muxS3;
        int muxSIG;
        int numThermistors;
        Multiplexer(int S0, int S1, int S2, int S3, int SIG, int num) {

            // assignation des attributs de l'instance

            this-> muxS0 = S0;
            this-> muxS1 = S1;
            this-> muxS2 = S2;
            this-> muxS3 = S3;
            this-> muxSIG = SIG;
            this-> numThermistors = num;            

            // Configuration des broches de sélection du multiplexeur
            pinMode(this-> muxS0, OUTPUT);
            pinMode(this-> muxS1, OUTPUT);
            pinMode(this-> muxS2, OUTPUT);
            pinMode(this-> muxS3, OUTPUT);
        }

        uint16_t* getTemperatures() {
            uint16_t* temperatures = new uint16_t[this-> numThermistors];
            for (int i = 0; i < this-> numThermistors; i++) {
                uint16_t temp = this-> getTemperature(i);
                temperatures[i] = temp;
                Serial.printf("Thermistance numero %u, temperature de %u \n", i, temp);
            }
            return temperatures;
        }

    private:
        uint16_t convertAnalogToTemp(float adcValue) {
            // === Constantes pour le calcul de température ===
            const float seriesResistor = 10000.0;
            const float nominalResistance = 10000.0;
            const float nominalTemperature = 25.0;
            const float bCoefficient = 3950;
            const int adcMax = 4095;
            
            float voltage = adcValue * 3.3 / adcMax;
            
            if (voltage < 0.01) {
                Serial.println(" Tension trop basse : vérifie le câblage !");
                return (uint16_t)(0xFFFF);
            }           
            
            float resistance = seriesResistor * (voltage / (3.3 - voltage));

            float steinhart;
            steinhart = resistance / nominalResistance;
            steinhart = log(steinhart);
            steinhart /= bCoefficient;
            steinhart += 1.0 / (nominalTemperature + 273.15);
            steinhart = 1.0 / steinhart;
            steinhart -= 273.15;
            

            return (uint16_t)(steinhart * 100);
        }

        void selectMuxChannel(int channel) {
            digitalWrite(this-> muxS0, bitRead(channel, 0));
            digitalWrite(this-> muxS1, bitRead(channel, 1));
            digitalWrite(this-> muxS2, bitRead(channel, 2));
            digitalWrite(this-> muxS3, bitRead(channel, 3));
        }

        uint16_t getTemperature(int channel) {
            selectMuxChannel(channel);
            delay(5);  // Court délai pour stabiliser le signal
            int adcValue = analogRead(muxSIG);
            
            float steinhart = convertAnalogToTemp(adcValue);
            
            return steinhart;
            
        }
};
 
    Multiplexer multiplexeur1(12, 14, 27, 26, 32, 10);

// Structure complète reçue depuis le maître
struct BatteryData {
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
BatteryData bmsData48V;

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

//Fonction qui stocke les données recues par i2C de la batterie 48V directement dans la structure 
void receiveEvent(int numBytes) {
  if (numBytes == sizeof(BatteryData)) {
    Wire.readBytes((char*)&bmsData48V, sizeof(BatteryData));
    
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
    Serial.println(sizeof(BatteryData));
  }
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
    controllerData.motor_current = ((frame.data[3] << 8) | frame.data[2]) / 10.0 - 2000;
    controllerData.battery_voltage = ((frame.data[5] << 8) | frame.data[4]) / 10.0;
    controllerData.error_code = (frame.data[7] << 8) | frame.data[6];

    Serial.print("--- Message 1 (0x0CF11E05) ---");
    /*Serial.printf("Speed: %d RPM\n", controllerData.speed_rpm);
    Serial.printf("Motor Current: %.1f A\n", controllerData.motor_current);
    Serial.printf("Battery Voltage: %.1f V\n", controllerData.battery_voltage);
    Serial.printf("Error Code: 0x%04X\n", controllerData.error_code);*/

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
    
    Serial.print("--- Message 2 (0x0CF11F05) ---");/*
    Serial.printf("Throttle: %d (%.2f V)\n", controllerData.throttle_raw, controllerData.throttle_voltage);
    Serial.printf("Controller Temp: %d°C\n", controllerData.controller_temp);
    Serial.printf("Motor Temp: %d°C\n", controllerData.motor_temp);
    Serial.printf("Command: %s, Feedback: %s\n", controllerData.command, controllerData.feedback);
    Serial.printf("Switches: Boost=%d, Foot=%d, Fwd=%d, Bwd=%d, Brake=%d\n",
                  controllerData.switch_boost, controllerData.switch_foot,
                  controllerData.switch_fwd, controllerData.switch_bwd,
                  controllerData.switch_brake);
    Serial.printf("Hall Sensors: A=%d, B=%d, C=%d\n",
                  controllerData.hall_a, controllerData.hall_b, controllerData.hall_c);*/
}

//Fonction pour demander l'envoie des trames au BMS 24V 
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

//Fonction de décodage des trames recues du BMS 24V 
void processReceivedData1890(uint8_t* data, uint32_t identifier) {
      bmsData24V.totalVoltage = ((data[0] << 8) | data[1]) * 0.1;
      bmsData24V.gatherVoltage = ((data[2] << 8) | data[3]) * 0.1;
      bmsData24V.current = (((data[4] << 8) | data[5]) - 30000) * 0.1;
      bmsData24V.soc = (data[6] == 0xFF && data[7] == 0xFF) ? -1 : ((data[6] << 8) | data[7]) * 0.1;
      Serial.println("SOC");
      Serial.println(bmsData24V.soc);
      
}
void processReceivedData1891(uint8_t* data, uint32_t identifier) {
     Serial.println("-> processReceivedData1891");
      bmsData24V.maxCellVoltage = (data[0] << 8) | data[1];
      bmsData24V.maxCellNumber = data[2];
      bmsData24V.minCellVoltage = (data[3] << 8) | data[4];
      bmsData24V.minCellNumber = data[5];
      Serial.println("CellVolt");
      Serial.println(bmsData24V.maxCellVoltage);
      Serial.flush();
}
void processReceivedData1892(uint8_t* data, uint32_t identifier) {
      bmsData24V.maxTemp = data[0] - 40;
      bmsData24V.minTemp = data[2] - 40;
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

    
void setup() {
  Serial.begin(115200);
  

  
  Wire.begin(SLAVE_ADDRESS);  
  Wire.onReceive(receiveEvent);
  Serial.println("Esclave I2C prêt à recevoir les données...");
   
  //Initialisation CAN 
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(10);
  ESP32Can.setTxQueueSize(10);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

  if(ESP32Can.begin()) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }
}

unsigned long lastRequestTime = 0;

void loop() {
    uint16_t* temperatures = multiplexeur1.getTemperatures();
      for (int i = 0; i < multiplexeur1.numThermistors; i++) {
          CanFrame txFrame;
          txFrame.identifier = 0x600 + i;
          txFrame.data_length_code = 8;
  
          txFrame.data[0] = (temperatures[i] >> 8) & 0xFF ;
          txFrame.data[1] = temperatures[i]  & 0xFF;

          if (!ESP32Can.writeFrame(txFrame, 1000)) {
              Serial.printf("Erreur envoi trame 0x60%d \n", i);
          }
          else {
              Serial.printf("Reussite envoi trame 0x60%d \n", i);
          }
          delay(10);
      }
      delete[] temperatures;
    // Lecture des trames entrantes à chaque itération
    while (ESP32Can.readFrame(rxFrame, 0)) {  // Pas de délai d’attente
      Serial.printf("\n[CAN] Received frame: 0x%08X, DLC=%d\n", rxFrame.identifier, rxFrame.data_length_code);
  
      switch (rxFrame.identifier) {
        // Réponses du BMS 24V
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
  
        // Trames CM
        case 0x0CF11E05:
          decodeMessage1(rxFrame);
          break;
        case 0x0CF11F05:
          decodeMessage2(rxFrame);
          break;
  
        // Autres (tu peux activer ça pour debug)
        default:
          /*
          Serial.println("Unknown frame, printing raw data:");
          for (int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf("Data[%d]: 0x%02X\n", i, rxFrame.data[i]);
          }
          */
          break;
      }
    }
  
    // Envoie des requêtes toutes les 2 secondes
    if (millis() - lastRequestTime >= 2000) {
      Serial.println("envoiiiii");
      sendBms24VRequest(0x18900140);
      sendBms24VRequest(0x18910140);
      sendBms24VRequest(0x18920140);
      sendBms24VRequest(0x18980140);
      reformAndSendCANFrames();
      lastRequestTime = millis();
    }
    // Pas de delay(1000), laisse le CPU libre pour lire les trames
}
