#include <driver/twai.h>

#define SWITCH_PIN 16  // Interrupteur entre GND et GPIO16

void setup() {
  Serial.begin(115200);
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // important pour lecture fiable

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
      twai_start() == ESP_OK) {
    Serial.println("TWAI (CAN) initialisé !");
  } else {
    Serial.println("Erreur d'initialisation TWAI.");
  }
}

void loop() {
  static int lastState = HIGH;
  int currentState = digitalRead(SWITCH_PIN);

  if (currentState != lastState) {
    lastState = currentState;

    twai_message_t message;
    message.identifier = 0x123;
    message.data_length_code = 1;
    message.data[0] = (currentState == LOW) ? 1 : 0;  // 1 = ON, 0 = OFF
    message.rtr = 0;
    message.extd = 0;

    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
      Serial.print("Message CAN envoyé : ");
      Serial.println(message.data[0]);
    } else {
      Serial.println("Échec envoi CAN.");
    }
  }

  delay(50);
}
