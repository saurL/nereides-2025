#include <driver/twai.h>

#define RELAY_PIN 18     // Pin du relais (électrovanne)

void setup() {
  Serial.begin(115200);

  // Configuration CAN
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial.println("TWAI (CAN) initialisé !");
  } else {
    Serial.println("Erreur d'initialisation TWAI.");
  }

  pinMode(RELAY_PIN, OUTPUT);  // Configurer la broche relais comme sortie
  digitalWrite(RELAY_PIN, LOW); // Assurer que le relais est éteint au démarrage
}

void loop() {
  twai_message_t message;
  
  // Recevoir un message CAN
  if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial.println("Message CAN reçu.");

    if (message.identifier == 0x123 && message.data_length_code == 1) {
      if (message.data[0] == 1) {
        Serial.println("Message reçu : activation relais (électrovanne ouverte).");
        digitalWrite(RELAY_PIN, HIGH);  // Ouvre l'électrovanne (active le relais)
      } else {
        Serial.println("Message reçu : désactivation relais (électrovanne fermée).");
        digitalWrite(RELAY_PIN, LOW);   // Ferme l'électrovanne (désactive le relais)
      }
    }
  }

  delay(50);  // Petite pause pour ne pas saturer la boucle
}
