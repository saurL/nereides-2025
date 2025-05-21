
#include <ESP32-TWAI-CAN.hpp>


#define CAN_TX 5
#define CAN_RX 4

struct MultiPlex {
  uint8_t analogPin;
  uint8_t controlPins[4];
  uint8_t thermistorNumber;
  uint16_t temperatures[16];
};




MultiPlex multi1 = {
  33,               // analogPinacn
  {13, 12, 14, 27}, // controlPins
  16                // thermistorNumber
};

int16_t convertAnalogToTemp(uint16_t voltage) {

  const float seriesResistor = 10000.0;     // Résistance fixe (10kΩ)
  const float nominalResistance = 10000.0;  // NTC à 25°C (10kΩ)
  const float nominalTemperature = 25.0;    // en °Ca
  const float bCoefficient = 3950;          // Coefficient B
  const int adcMax = 4095;                  // ADC 12 bits ESP32

  float resistance = seriesResistor * (voltage / (3.3 - voltage));

  float steinhart;

  steinhart = resistance / nominalResistance;
  steinhart = log(steinhart);
  steinhart /= bCoefficient;
  steinhart += 1.0 / (nominalTemperature + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  return (int16_t)steinhart * 100;
}

float readThermistors(MultiPlex multi) {
  int16_t values[multi.thermistorNumber];
  int muxChannel[16][4] = {
    { 0, 0, 0, 0 },  //channel 0
    { 1, 0, 0, 0 },  //channel 1
    { 0, 1, 0, 0 },  //channel 2
    { 1, 1, 0, 0 },  //channel 3
    { 0, 0, 1, 0 },  //channel 4
    { 1, 0, 1, 0 },  //channel 5
    { 0, 1, 1, 0 },  //channel 6
    { 1, 1, 1, 0 },  //channel 7
    { 0, 0, 0, 1 },  //channel 8
    { 1, 0, 0, 1 },  //channel 9
    { 0, 1, 0, 1 },  //channel 10
    { 1, 1, 0, 1 },  //channel 11
    { 0, 0, 1, 1 },  //channel 12
    { 1, 0, 1, 1 },  //channel 13
    { 0, 1, 1, 1 },  //channel 14
    { 1, 1, 1, 1 }   //channel 15
  };
  for (int tn = 0; tn < multi.thermistorNumber; tn++) {
    for (int pin = 0; pin < 4; pin++) {
      digitalWrite(multi.controlPins[pin], muxChannel[tn][pin]);
    }
    int16_t pinTemp = convertAnalogToTemp(analogRead(multi.analogPin));
    multi.temperatures[tn] = pinTemp;
    delay(50);
  }
}



void setup() {
  Serial.begin(115200);
  
  for (int i = 0; i < 4; i++) {
    pinMode(multi1.controlPins[i], OUTPUT);
  }
  pinMode(multi1.analogPin, INPUT);

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
  readThermistors(multi1);
  for (int temp = 0; temp < multi1.thermistorNumber; temp++) {
    CanFrame txFrame;
    txFrame.identifier = 0x600 + temp;
    txFrame.data_length_code = 8;

    txFrame.data[0] = multi1.temperatures[temp] >> 8;    // Température 1 - Octet 1
    txFrame.data[1] = multi1.temperatures[temp] & 0xFF;  // Température 1 - Octet 2

    if (!ESP32Can.writeFrame(txFrame)) {
      Serial.print("Erreur envoi trame 0x60");
      Serial.println(temp);
    }
  }
  // put your main code here, to run repeatedly:

}
