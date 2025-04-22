#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <CAN.h>

CAN_device_t CAN_cfg;
const int rx_queue_size = 10;

struct PAC_FLAGS {
    uint8_t erreur_critique;
    uint8_t erreur_mineure;
    uint8_t alerte_h2;
};

struct PAC_data {
    uint16_t amperage;
    uint16_t voltage;
    uint8_t system_state;
    PAC_FLAGS flag;
};

PAC_data pac_data;

void setup() {
    Serial.begin(115200);
    //Serial.println("avant boucle");
    CAN_cfg.speed = CAN_SPEED_250KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_5;
    CAN_cfg.rx_pin_id = GPIO_NUM_4;
    CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
    Serial.println("CAN avant initialisation");
    ESP32Can.CANInit();
    Serial.println("CAN initialisé");
    //delay(10000);
}
void loop() {
    CAN_frame_t rx_frame;
   // Serial.print("dans la bcl");
    if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
        //Serial.println("dans le if");
        switch (rx_frame.MsgID) {
            Serial.println("message récupéré");
            case 0x2fc0001:
                if (rx_frame.FIR.B.DLC == 8) {
                    pac_data.amperage = (rx_frame.data.u8[0] << 8) | rx_frame.data.u8[3];
                    pac_data.voltage = (rx_frame.data.u8[4] << 8) | rx_frame.data.u8[7];
                    Serial.printf("Courant: %.2f A, Tension: %.2f V\n", pac_data.amperage * 0.01, pac_data.voltage * 0.01);
                //delay(10000);
                }

                break;

            case 0x2fc0000:
                if (rx_frame.FIR.B.DLC == 4) {
                    pac_data.system_state = rx_frame.data.u8[0];
                    pac_data.flag.erreur_critique = rx_frame.data.u8[1] & 0x01;
                    pac_data.flag.erreur_mineure = (rx_frame.data.u8[1] >> 1) & 0x01;
                    pac_data.flag.alerte_h2 = (rx_frame.data.u8[1] >> 2) & 0x01;
                    Serial.printf("État système: %d, Erreurs: Critique=%d, Mineure=%d, H2=%d\n",
                                  pac_data.system_state,
                                  pac_data.flag.erreur_critique,
                                  pac_data.flag.erreur_mineure,
                                  pac_data.flag.alerte_h2);
                  //delay(10000);
                }
                break;
        }
    }
    delay(100);  

}
