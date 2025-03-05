#define MQ8_PIN 34  // GPIO utilisé pour lire la sortie analogique (A0)

void setup() {
  Serial.begin(115200);
}

void loop() {
  int sensorValue = analogRead(MQ8_PIN);  // Lire la valeur analogique
  float voltage = sensorValue * (3.3 / 4095.0);  // Conversion en tension (ESP32 en 12 bits)

  Serial.print("Valeur brute : ");
  Serial.print(sensorValue);
  Serial.print("  | Tension : ");
  Serial.print(voltage);
  Serial.println(" V");

  delay(1000);  // Pause d'1 seconde
}
