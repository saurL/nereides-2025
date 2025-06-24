#include <Wire.h>
#include <ESP32-TWAI-CAN.hpp>
// === CAN Configuration ===
#define CAN_TX 4
#define CAN_RX 5
CanFrame rxFrame;
// === UART vers second ESP32 ===
#define UART_TX_PIN 22  // vers RX de l'esclave
#define UART_RX_PIN 21  // optionnel ici (non utilisé en émission)
HardwareSerial UartToSlave(1);

// === Classe Multiplexer ===
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
Multiplexer multiplexer(14,27,26,25,33,15);


// === Structure commune pour données BMS 24V ===
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
void sendBms24VRequest(uint32_t identifier) {
  CanFrame frame;
  frame.identifier = identifier;
  frame.extd = 1;
  frame.data_length_code = 8;
  memset(frame.data, 0, 8);
  ESP32Can.writeFrame(frame, 1000);
}
// === UART ENVOI VERS SLAVE ===
void sendBmsDataOverUART() {
  // Construit un JSON simple mais complet
  String json = "{";
  json += "\"Data\":\"BMS\",";
  // Tensions
  json += "\"V_total\":"  + String(bmsData24V.totalVoltage, 1)  + ",";
  json += "\"V_gather\":" + String(bmsData24V.gatherVoltage, 1) + ",";
  // Courant & SOC
  json += "\"I\":"   + String(bmsData24V.current, 1) + ",";
  json += "\"SOC\":" + String(bmsData24V.soc, 1)     + ",";
  // Cellules extrêmes
  json += "\"V_cell_max\":"   + String(bmsData24V.maxCellVoltage * 0.001, 3) + ",";  // en V
  json += "\"Cell_max_idx\":" + String(bmsData24V.maxCellNumber)            + ",";
  json += "\"V_cell_min\":"   + String(bmsData24V.minCellVoltage * 0.001, 3) + ",";
  json += "\"Cell_min_idx\":" + String(bmsData24V.minCellNumber)            + ",";
  // Températures
  json += "\"T_max\":" + String(bmsData24V.maxTemp) + ",";
  json += "\"T_min\":" + String(bmsData24V.minTemp) + ",";
  // Status d'erreur
  json += "\"Errors\":\"0x" + String((uint32_t)(bmsData24V.errorStatus >> 32), HEX)
                   + String((uint32_t)(bmsData24V.errorStatus       ), HEX) + "\"";
  json += "}\n";

  // Envoi
  UartToSlave.print(json);
  Serial.print("[UART→ESCLAVE] "); Serial.print(json);
}


void sendTemperaturesOverUART() {
    uint16_t* temperatures = multiplexer.getTemperatures();    
    String json = "{\"Data\":\"Temperatures\", \"ThermistorNumber\":" + String(multiplexer.numThermistors) + ",";
    for (int i = 0; i < multiplexer.numThermistors; i++) {
        json += String(i) + ":" + String(temperatures[i]);
        if (i != multiplexer.numThermistors - 1) json += ",";
    }
    json += "}\n";
    delete[] temperatures;    
    UartToSlave.print(json);
    Serial.print("[UART→ESCLAVE] "); Serial.print(json);
}

// === Décodage CAN ===
void processReceivedData1890(uint8_t* data, uint32_t identifier) {
  bmsData24V.totalVoltage = ((data[0] << 8) | data[1]) * 0.1;
  bmsData24V.gatherVoltage = ((data[2] << 8) | data[3]) * 0.1;
  bmsData24V.current = (((data[4] << 8) | data[5]) - 30000) * 0.1;
  bmsData24V.soc = ((data[6] << 8) | data[7]) * 0.1;
}
void processReceivedData1891(uint8_t* data, uint32_t identifier) {
  bmsData24V.maxCellVoltage = (data[0] << 8) | data[1];
  bmsData24V.maxCellNumber = data[2];
  bmsData24V.minCellVoltage = (data[3] << 8) | data[4];
  bmsData24V.minCellNumber = data[5];
}
void processReceivedData1892(uint8_t* data, uint32_t identifier) {
  bmsData24V.maxTemp = data[0] - 40;
  bmsData24V.minTemp = data[2] - 40;
}
void processReceivedData1898(uint8_t* data, uint32_t identifier) {
  bmsData24V.errorStatus = 0;
  for (int i = 0; i < 8; i++) {
    bmsData24V.errorStatus |= ((uint64_t)data[i] << (8 * i));
  }
}
void setup() {
  Serial.begin(115200);
  UartToSlave.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("ESP32 CAN + UART vers second ESP32");
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(10);
  ESP32Can.setTxQueueSize(10);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
  ESP32Can.begin();
}
unsigned long lastRequestTime = 0;
unsigned long lastUartSendTime = 0;
void loop() {
    //
    
    // Lire les trames CAN entrantes
    while (ESP32Can.readFrame(rxFrame, 0)) {
        switch (rxFrame.identifier) {
        case 0x18904001: processReceivedData1890(rxFrame.data, rxFrame.identifier); break;
        case 0x18914001: processReceivedData1891(rxFrame.data, rxFrame.identifier); break;
        case 0x18924001: processReceivedData1892(rxFrame.data, rxFrame.identifier); break;
        case 0x18984001: processReceivedData1898(rxFrame.data, rxFrame.identifier); break;
        }
    }
    // Requêtes CAN périodiques
    if (millis() - lastRequestTime >= 2000) {
        sendBms24VRequest(0x18900140);
        sendBms24VRequest(0x18910140);
        sendBms24VRequest(0x18920140);
        sendBms24VRequest(0x18980140);
        lastRequestTime = millis();
    }
    // Envoi UART périodique des données vers l'esclave
    if (millis() - lastUartSendTime >= 2000) {
        sendBmsDataOverUART();
        sendTemperaturesOverUART();
        lastUartSendTime = millis();
    }
}
