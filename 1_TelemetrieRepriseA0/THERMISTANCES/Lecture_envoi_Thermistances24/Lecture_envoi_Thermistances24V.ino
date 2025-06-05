// === Multiplexeur CD74HC4067 ===
#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX 5
#define CAN_RX 4

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


Multiplexer multiplexeur1(16, 17, 18, 19, 34, 10);

void setup() {
    Serial.begin(115200);
    delay(1000);

    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    // Initialiser le CAN
    if (ESP32Can.begin()) {
        Serial.println("CAN bus démarré avec succès!");
    } else {
        Serial.println("Échec du démarrage du bus CAN!");
        while (1);  // Bloquer si le CAN ne démarre pas
    }
}

void loop() {
    uint16_t* temperatures = multiplexeur1.getTemperatures();
    for (int i = 0; i < multiplexeur1.numThermistors; i++) {
        CanFrame txFrame;
        txFrame.identifier = 0x600 + i;
        txFrame.data_length_code = 8;

        txFrame.data[0] = (temperatures[i] >> 8) & 0xFF ;
        txFrame.data[1] = temperatures[i]  & 0xFF;

        if (!ESP32Can.writeFrame(txFrame)) {
            Serial.printf("Erreur envoi trame 0x60%d \n", i);
        }
        else {
            Serial.printf("Reussite envoi trame 0x60%d \n", i);
        }
        delay(10);
    }
    delete[] temperatures;
    //multiplexeur1.run();
    delay(1000);  // Attendre 1 seconde avant le prochain cycle
}
