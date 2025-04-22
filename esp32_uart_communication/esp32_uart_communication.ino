#include <ArduinoJson.h>
#include <vector>

#define TX_PIN 17  // Broche TX pour UART1
#define RX_PIN 16  // Broche RX pour UART1
HardwareSerial RaspberrySerial(1);
// Fonction pour encoder la taille d'un JSON en VarInt
std::vector<uint8_t>  encodeVarIntLength(const std::vector<uint8_t>& json_bytes) {
    uint32_t value = json_bytes.size();
    Serial.println("on a récuperer la taille du json");
    Serial.println("taille de botre json en bytes  : " );
    Serial.println(value);
    std::vector<uint8_t> result;

    while (value >= 0x80) {
        result.push_back((value & 0x7F) | 0x80); // 7 bits + continuation
        value >>= 7;
    }
    result.push_back(value); // dernier octet (MSB = 0)
    Serial.println("Données du varint");
    for (size_t i = 0; i < result.size(); ++i) {
        Serial.println(result[i],HEX);
    }
    return result;
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

void send_json(StaticJsonDocument<200>& json) {
    // Convertir en chaîne JSON
    String jsonString;
    serializeJson(json, jsonString);

    // Encodage de la taille du JSON en VarInt
    uint32_t varIntSize = encodeVarInt(jsonString);

    // Affichage des informations
    Serial.println("JSON : " + jsonString);
    Serial.print("VarInt (taille en binaire) : ");
    Serial.println(varIntSize);

    // Envoi du VarInt (taille du JSON)
    Serial1.write(varIntSize);

    // Envoi du JSON
    Serial1.println(jsonString);
}

void loop() {
    // Création et envoi du premier JSON
    StaticJsonDocument<200> doc;
    doc["ID"] = "ID1";
    doc["value"] = 0x2FC1001;
    doc["ff"] = 0x187B1819;
    doc["gg"] = 0x1872986178;
    doc["oiu"] = 0X8971F91;
    doc["oiu"] = "gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg";
    send_json(doc);

    delay(2000);

    // Création et envoi du second JSON
    StaticJsonDocument<200> doc2;
    doc2["ID"] = "ID2";
    doc2["value"] = 2131212;
    send_json(doc2);

    delay(2000);
}
