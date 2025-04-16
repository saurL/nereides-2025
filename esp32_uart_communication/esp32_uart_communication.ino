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

std::vector<uint8_t> stringToBytes(const String& input) {
    std::vector<uint8_t> result;
    for (size_t i = 0; i < input.length(); ++i) {
        result.push_back(static_cast<uint8_t>(input[i]));
    }
    return result;
}



void send_json(StaticJsonDocument<200>& json) {
    // Convertir en chaîne JSON
    String jsonString;
    serializeJson(json, jsonString);
    Serial.println("On vas calculr json en bytes");
    std::vector<uint8_t> json_bytes = stringToBytes(jsonString);
    // Encodage de la taille du JSON en VarInt
    std::vector<uint8_t>  varIntSize = encodeVarIntLength(json_bytes);

    // Affichage des informations
    Serial.println("JSON : " + jsonString);
    std::vector<uint8_t> message;
    message.insert(message.end(), varIntSize.begin(), varIntSize.end()); // Ajout du VarInt
    message.insert(message.end(), json_bytes.begin(), json_bytes.end()); // Ajout du JSON après le VarInt
    // Envoi du VarInt (taille du JSON)
    Serial1.write(message.data(), message.size()); // Envoi de la taille du JSON
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
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

    StaticJsonDocument<200> doc33;
    doc33["ID"] = "ID3";
    doc33["value"] = 0x2FC1001;
    doc33["ff"] = 0x187B1819;
    doc33["gg"] = 0x1872986178;
    doc33["oiu"] = 0X8971F91;
    doc33["oiu"] = "gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg";
     doc33["dff"] = 0x187B1819;
    doc33["gfg"] = 0x1872986178;
    doc33["oqsdiu"] = 0X8971F91;
    doc33["oiazeu"] = "gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg";
    send_json(doc33);

    delay(2000);
    // Création et envoi du second JSON
    StaticJsonDocument<200> doc2;
    doc2["ID"] = "ID2";
    doc2["value"] = 2131212;
    send_json(doc2);

    delay(2000);
}
