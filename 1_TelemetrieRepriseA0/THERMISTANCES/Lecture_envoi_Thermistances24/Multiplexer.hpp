#ifndef MULTIPLEXER_HPP
#define MULTIPLEXER_HPP

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


#endif
