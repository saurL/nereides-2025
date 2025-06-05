#include <ESP32-TWAI-CAN.hpp>
#define CAN_TX 22
#define CAN_RX 21
CanFrame rxFrame;

void decodeTemperature(const CanFrame &frame) {
    uint16_t temperature = (frame.data[0] << 8) | frame.data[1];
    uint8_t idTrame = frame.identifier - 0x600;
    Serial.printf("Thermistance numero %u, la temperature est de %u", idTrame, temperature);
}

void setup() {
    Serial.begin(115200);
    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    if(ESP32Can.begin()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("fCAN bus failed!");
    }
}


void loop() {
    while (ESP32Can.readFrame(rxFrame, 10)) {
        Serial.printf("\n[CAN] Received frame: 0x%08X, DLC=%d\n", rxFrame.identifier, rxFrame.data_length_code);
        switch (rxFrame.identifier) {
            case 0x600:
            case 0x601:
            case 0x602:
            case 0x603:
            case 0x604:
            case 0x605:
            case 0x606:
            case 0x607:
            case 0x608:
            case 0x609:
                decodeTemperature(rxFrame);
                break;
        default:
            Serial.println("Unknown frame, printing raw data:");
            for (int i = 0; i < rxFrame.data_length_code; i++) {
            Serial.printf("Data[%d]: 0x%02X\n", i, rxFrame.data[i]);
            }
            break;
        }
    }
}
