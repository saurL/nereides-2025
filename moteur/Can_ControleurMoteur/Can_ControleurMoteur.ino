#include <ESP32-TWAI-CAN.hpp>

// Default for ESP32
#define CAN_TX      5
#define CAN_RX      4

CanFrame rxFrame;

void setup() {
    // Setup serial for debugging.
    Serial.begin(115200);

    // Set pins
    ESP32Can.setPins(CAN_TX, CAN_RX);
    
    // Set queue sizes
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    // Set CAN speed to 250 Kbps (as per the controller's documentation)
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    // Initialize CAN bus
    if(ESP32Can.begin()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }
}

void decodeMessage1(const CanFrame &frame) {
    // Decode Message 1 (0x0CF11E05): Speed, Current, Voltage, Error Codes
    uint16_t speed_rpm = (frame.data[1] << 8) | frame.data[0]; // RPM
    uint16_t motor_current = (frame.data[3] << 8) | frame.data[2]; // 0.1A/bit
    uint16_t battery_voltage = (frame.data[5] << 8) | frame.data[4]; // 0.1V/bit
    uint16_t error_code = (frame.data[7] << 8) | frame.data[6]; // Error bits

    Serial.println("\n--- Message 1 (0x0CF11E05) ---");
    Serial.printf("Speed: %d RPM\n", speed_rpm);
    Serial.printf("Motor Current: %.1f A\n", motor_current / 10.0);
    Serial.printf("Battery Voltage: %.1f V\n", battery_voltage / 10.0);
    Serial.printf("Error Code: 0x%04X\n", error_code);

    // Check individual error bits (ERR0 to ERR15)
    for (int i = 0; i < 16; i++) {
        if (error_code & (1 << i)) {
            Serial.printf("ERR%d: Active\n", i);
        }
    }
}

void decodeMessage2(const CanFrame &frame) {
    // Decode Message 2 (0x0CF11F05): Throttle, Temperatures, Status
    uint8_t throttle = frame.data[0]; // 0-255 maps to 0-5V
    int8_t controller_temp = frame.data[1] - 40; // °C (offset 40)
    int8_t motor_temp = frame.data[2] - 30; // °C (offset 30)
    uint8_t controller_status = frame.data[4]; // Status bits
    uint8_t switch_signals = frame.data[5]; // Switch states

    Serial.println("\n--- Message 2 (0x0CF11F05) ---");
    Serial.printf("Throttle: %d (%.2f V)\n", throttle, throttle * 5.0 / 255.0);
    Serial.printf("Controller Temperature: %d°C\n", controller_temp);
    Serial.printf("Motor Temperature: %d°C\n", motor_temp);

    // Decode controller status (bits 0-1: command, bits 2-3: feedback)
    const char* commands[] = {"Neutral", "Forward", "Backward", "Reserved"};
    const char* feedbacks[] = {"Stationary", "Forward", "Backward", "Reserved"};
    Serial.printf("Command: %s, Feedback: %s\n", 
                 commands[controller_status & 0x03], 
                 feedbacks[(controller_status >> 2) & 0x03]);

    // Decode switch signals (bits 0-2: Hall A/B/C, bits 3-7: switches)
    Serial.printf("Switches: Boost=%d, Foot=%d, Fwd=%d, Bwd=%d, Brake=%d\n",
                 (switch_signals >> 7) & 0x01,
                 (switch_signals >> 6) & 0x01,
                 (switch_signals >> 5) & 0x01,
                 (switch_signals >> 4) & 0x01,
                 (switch_signals >> 3) & 0x01);
    Serial.printf("Hall Sensors: A=%d, B=%d, C=%d\n", //Si on a une vitesse bizarre c'est que l'odre A B C est pas bon 
                 (switch_signals >> 0) & 0x01,
                 (switch_signals >> 1) & 0x01,
                 (switch_signals >> 2) & 0x01);
       
}

void loop() {
    if (ESP32Can.readFrame(rxFrame, 1000)) {
        Serial.printf("\nReceived frame: 0x%08X, DLC=%d\n", rxFrame.identifier, rxFrame.data_length_code);

        // Check for known controller messages
        switch (rxFrame.identifier) {
            case 0x0CF11E05: // Message 1
                decodeMessage1(rxFrame);
                break;
            case 0x0CF11F05: // Message 2
                decodeMessage2(rxFrame);
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