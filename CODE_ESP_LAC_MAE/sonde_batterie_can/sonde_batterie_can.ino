//initialisation pour les sondes
const int numThermistors = 10;  //nombre de sondes
const int thermistorPins[numThermistors] = { 34, 35, 32, 33, 39, 36, 25, 27, 14, 26 };


const float seriesResistor = 10000.0;     // Résistance fixe (10kΩ)
const float nominalResistance = 10000.0;  // NTC à 25°C (10kΩ)
const float nominalTemperature = 25.0;    // en °C
const float bCoefficient = 3950;          // Coefficient B
const int adcMax = 4095;

#include <ESP32-TWAI-CAN.hpp>


#define CAN_TX 5
#define CAN_RX 4



void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initialisation de l'émetteur CAN pour thermistances...");
  // Configuration CAN
  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  // Initialiser le CAN
  if (ESP32Can.begin()) {
    Serial.println("CAN bus démarré avec succès!");
  } else {
    Serial.println("Échec du démarrage du bus CAN!");
    while (1)
      ;  // Bloquer si le CAN ne démarre pas
  }
}




void loop() {
  for (int i = 0; i < numThermistors; i++) {
    int pin = thermistorPins[i];
    int adcValue = analogRead(pin);
    float voltage = adcValue * 3.3 / adcMax;

    Serial.println("------------");
    Serial.print("Thermistance sur pin GPIO ");
    Serial.println(pin);

    if (voltage < 0.01) {
      Serial.println(" Tension trop basse : vérifie le câblage !");
      continue;
    }

    float resistance = seriesResistor * (voltage / (3.3 - voltage));

    float steinhart;
    steinhart = resistance / nominalResistance;
    steinhart = log(steinhart);
    steinhart /= bCoefficient;
    steinhart += 1.0 / (nominalTemperature + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    Serial.print("ADC brut : ");
    Serial.println(adcValue);
    Serial.print("Tension : ");
    Serial.print(voltage, 3);
    Serial.println(" V");
    Serial.print("Résistance NTC : ");
    Serial.print(resistance, 1);
    Serial.println(" Ohms");
    Serial.print("Température : ");
    Serial.print(steinhart);
    Serial.println(" °C");




    //SEND en CAN!
    CanFrame txFrame;

    // Trame 1 : Températures des trois sondes (8 octets)
    txFrame.identifier = 0x600 + i;
    txFrame.data_length_code = 8;                // Taille maximale de la trame CAN
    int16_t temp1 = (int16_t)(steinhart * 100);  // Conversion en centi-degrés

    // Remplir les 2 premiers octets avec les températures
    txFrame.data[0] = temp1 >> 8;    // Température 1 - Octet 1
    txFrame.data[1] = temp1 & 0xFF;  // Température 1 - Octet 2
                                     // Laisser les 6 derniers octets vides ou y mettre des informations supplémentaires
    txFrame.data[2] = 0;             // Température 2 - Octet 3
    txFrame.data[3] = 0;             // Température 2 - Octet 4
    txFrame.data[4] = 0;             // Température 3 - Octet 5
    txFrame.data[5] = 0;             // Température 3 - Octet 6
    txFrame.data[6] = 0;             // Octet 7
    txFrame.data[7] = 0;             // Octet 8

    if (!ESP32Can.writeFrame(txFrame)) {
      Serial.print("Erreur envoi trame 0x60");
      Serial.println(i);
    }
  }

  Serial.println("======= Fin de lecture =======\n");

  delay(1000);  // Envoyer toutes les secondes
}