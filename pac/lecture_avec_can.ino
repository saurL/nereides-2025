#include <ESP32-TWAI-CAN.hpp>

// === Multiplexeur CD74HC4067 ===
const int muxS0 = 16;
const int muxS1 = 17;
const int muxS2 = 18;
const int muxS3 = 19;
const int muxSIG = 34;  // Broche analogique de l’ESP32

const int numThermistors = 10;  // Nombre de sondes utilisées (canaux 0 à 9)

// === Constantes pour le calcul de température ===
const float seriesResistor = 10000.0;
const float nominalResistance = 10000.0;
const float nominalTemperature = 25.0;
const float bCoefficient = 3950;
const int adcMax = 4095;

// === Broches CAN ===
#define CAN_TX 5
#define CAN_RX 4

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Configuration des broches de sélection du multiplexeur
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);

  Serial.println("Initialisation de l'émetteur CAN pour thermistances...");

  ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  if (ESP32Can.begin()) {
    Serial.println("CAN bus démarré avec succès!");
  } else {
    Serial.println("Échec du démarrage du bus CAN!");
    while (1);
  }
}

void loop() {
  for (int i = 0; i < numThermistors; i++) {
    selectMuxChannel(i);
    delay(5);  // Petit délai pour la stabilité du signal

    int adcValue = analogRead(muxSIG);
    float voltage = adcValue * 3.3 / adcMax;

    Serial.println("------------");
    Serial.print("Thermistance sur canal MUX ");
    Serial.println(i);

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

    // Envoi CAN
    CanFrame txFrame;
    txFrame.identifier = 0x600 + i;
    txFrame.data_length_code = 8;
    int16_t temp1 = (int16_t)(steinhart * 100);  // en centi-degrés
    txFrame.data[0] = temp1 >> 8;
    txFrame.data[1] = temp1 & 0xFF;
    for (int j = 2; j < 8; j++) txFrame.data[j] = 0;

    if (!ESP32Can.writeFrame(txFrame)) {
      Serial.print("Erreur envoi trame 0x60");
      Serial.println(i);
    }
  }

  Serial.println("======= Fin de lecture =======\n");
  delay(1000);
}

// Fonction de sélection de canal du multiplexeur
void selectMuxChannel(int channel) {
  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
}
