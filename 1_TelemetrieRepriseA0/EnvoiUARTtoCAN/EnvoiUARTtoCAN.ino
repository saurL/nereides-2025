#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <ESP32-TWAI-CAN.hpp>

// Structure pour stocker les données BMS 24V (identique à votre code d'envoi)
struct BmsData24V {
    int can_id_bms_1890;
    int can_id_bms_1891;
    int can_id_bms_1892;
    int can_id_bms_1898;
    
    float totalVoltage = -1;
    float gatherVoltage = -1;
    float current = -1;
    float soc = -1;
    uint16_t maxCellVoltage = 0;
    uint8_t maxCellNumber = 0;
    uint16_t minCellVoltage = 0;
    uint8_t minCellNumber = 0;
    int8_t maxTemp = -100;  // en °C
    int8_t minTemp = -100;  // en °C
    uint64_t errorStatus = 0;
};

// Structure pour stocker les données de température
// Structure pour stocker les données de température
struct TemperatureData {
    uint8_t thermistorNumber = 0;
    uint16_t temperatures[16];  // Maximum 16 thermistances (ajustable)
    bool dataValid = false;
    
    // Constructeur pour initialiser le tableau
    TemperatureData() {
        for (int i = 0; i < 16; i++) {
            temperatures[i] = 0xFFFF;  // Valeur invalide par défaut
        }
    }
};

BmsData24V bmsData24V1;

BmsData24V bmsData24V2;

TemperatureData temperatureData;

#pragma region defines
// Configuration UART (correspondant à votre code d'envoi)
#define RX_PIN1 22  // Correspond à UART_RX_PIN de l'émetteur
#define TX_PIN1 21  // Correspond à UART_TX_PIN de l'émetteur
#define BAUD_RATE 9600  // Même vitesse que votre code d'envoi

#define RX_PIN2 23
#define TX_PIN2 24

// Configuration CAN
#define CAN_TX 5
#define CAN_RX 4

// IDs CAN correspondant à vos fonctions de décodage
//BMS 1 
#define CAN_ID_BMS1_1890       0x762  // 1890 en décimal = 0x762 en hex
#define CAN_ID_BMS1_1891       0x763  // 1891 en décimal = 0x763 en hex  
#define CAN_ID_BMS1_1892       0x764  // 1892 en décimal = 0x764 en hex
#define CAN_ID_BMS1_1898       0x76A  // 1898 en décimal = 0x76A en hex

//BMS 2 
#define CAN_ID_BMS2_1890       0x862  // 1890 en décimal = 0x762 en hex
#define CAN_ID_BMS2_1891       0x863  // 1891 en décimal = 0x763 en hex  
#define CAN_ID_BMS2_1892       0x864  // 1892 en décimal = 0x764 en hex
#define CAN_ID_BMS2_1898       0x86A  // 1898 en décimal = 0x76A en hex

#define CAN_ID_TEMP_DATA_1      0x200  // ID pour les données de température
#define CAN_ID_TEMP_DATA_2      0x300


#pragma endregion

HardwareSerial SerialBMS1(1); // Utilise UART1
HardwareSerial SerialBMS2(2); // Utilise UART2
String receivedData1 = "";
String receivedData2 = "";

void initCAN() {
    // Configuration du CAN avec la bibliothèque ESP32-TWAI-CAN
    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setRxQueueSize(10);
    ESP32Can.setTxQueueSize(10);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    if(ESP32Can.begin()) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }
}


void sendBmsDataOverCAN(BmsData24V& bms) {
    if (!isBmsDataValid(bms)) {
        return;  // Ne pas envoyer de données invalides
    }
    
    #pragma region frame1890
    // Message 1890: Données de base (compatible avec processReceivedData1890)
    CanFrame frame1890;
    frame1890.identifier = bms.can_id_bms_1890;
    frame1890.extd = 1;
    frame1890.data_length_code = 8;
    
    // Formatage selon processReceivedData1890:
    // totalVoltage * 10 sur 2 bytes (big endian)
    uint16_t totalVolt = (uint16_t)(bms.totalVoltage * 10);
    frame1890.data[0] = (totalVolt >> 8) & 0xFF;
    frame1890.data[1] = totalVolt & 0xFF;
    
    // gatherVoltage * 10 sur 2 bytes (big endian)
    uint16_t gatherVolt = (uint16_t)(bms.gatherVoltage * 10);
    frame1890.data[2] = (gatherVolt >> 8) & 0xFF;
    frame1890.data[3] = gatherVolt & 0xFF;
    
    // current * 10 + 30000 sur 2 bytes (big endian)
    uint16_t current = (uint16_t)((bms.current * 10) + 30000);
    frame1890.data[4] = (current >> 8) & 0xFF;
    frame1890.data[5] = current & 0xFF;
    
    // SOC * 10 sur 2 bytes (big endian), 0xFFFF si invalide
    uint16_t socValue;
    if (bms.soc < 0) {
        socValue = 0xFFFF;
    } else {
        socValue = (uint16_t)(bms.soc * 10);
    }
    frame1890.data[6] = (socValue >> 8) & 0xFF;
    frame1890.data[7] = socValue & 0xFF;
    
    if (ESP32Can.writeFrame(&frame1890) == 0) {
        Serial.println("CAN: Trame 1890 (données de base) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1890");
    }
    #pragma endregion

    #pragma region frame1891
    // Message 1891: Données cellules (compatible avec processReceivedData1891)
    CanFrame frame1891;
    frame1891.identifier = bms.can_id_bms_1891;
    frame1891.extd = 1;
    frame1891.data_length_code = 8;
    
    // maxCellVoltage sur 2 bytes (big endian)
    frame1891.data[0] = (bms.maxCellVoltage >> 8) & 0xFF;
    frame1891.data[1] = bms.maxCellVoltage & 0xFF;
    // maxCellNumber sur 1 byte
    frame1891.data[2] = bms.maxCellNumber;
    // minCellVoltage sur 2 bytes (big endian)
    frame1891.data[3] = (bms.minCellVoltage >> 8) & 0xFF;
    frame1891.data[4] = bms.minCellVoltage & 0xFF;
    // minCellNumber sur 1 byte
    frame1891.data[5] = bms.minCellNumber;
    
    // Bytes 6-7 réservés
    frame1891.data[6] = 0x00;
    frame1891.data[7] = 0x00;
    
    if (ESP32Can.writeFrame(&frame1891) == 0) {
        Serial.println("CAN: Trame 1891 (données cellules) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1891");
    }
    #pragma endregion

    #pragma region frame1892
    // Message 1892: Températures (compatible avec processReceivedData1892)
    CanFrame frame1892;
    frame1892.identifier = bms.can_id_bms_1892;
    frame1892.extd = 1;
    frame1892.data_length_code = 8;
    
    // maxTemp + 40 sur byte 0
    frame1892.data[0] = (uint8_t)(bms.maxTemp + 40);
    
    // Byte 1 réservé
    frame1892.data[1] = 0x00;
    
    // minTemp + 40 sur byte 2
    frame1892.data[2] = (uint8_t)(bms.minTemp + 40);
    
    // Bytes 3-7 réservés
    for (int i = 3; i < 8; i++) {
        frame1892.data[i] = 0x00;
    }
    
    if (ESP32Can.writeFrame(&frame1892) == 0) {
        Serial.println("CAN: Trame 1892 (températures) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1892");
    }
    #pragma endregion

    #pragma region frame1898
    // Message 1898: Erreurs (compatible avec processReceivedData1898)
    CanFrame frame1898;
    frame1898.identifier = bms.can_id_bms_1898;
    frame1898.extd = 1;
    frame1898.data_length_code = 8;
    
    // errorStatus sur 8 bytes (little endian)
    for (int i = 0; i < 8; i++) {
        frame1898.data[i] = (bms.errorStatus >> (8 * i)) & 0xFF;
    }
    
    if (ESP32Can.writeFrame(&frame1898) == 0) {
        Serial.println("CAN: Trame 1898 (erreurs) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1898");
    }
    #pragma endregion
}

void sendTemperatureDataOverCAN() {
    
    // Message température: Envoie les données de température
    
    // Nombre de thermistances
    for (int i = 0; i < temperatureData.thermistorNumber; i++) {        
        CanFrame tempFrame;
        tempFrame.identifier = 0x200 + i;
        tempFrame.extd = 1;
        tempFrame.data_length_code = 8;

        tempFrame.data[0] = (temperatureData.temperatures[i] >> 8) & 0xFF ;
        tempFrame.data[1] = temperatureData.temperatures[i]  & 0xFF;

        
        if (!ESP32Can.writeFrame(tempFrame, 1000)) {
              Serial.printf("Erreur envoi trame 0x60%d \n", i);
        }
        else {
            Serial.printf("Reussite envoi trame 0x60%d \n", i);
        }


    }
}

void readUartData(HardwareSerial& serial, String& received) {
    while (serial.available()) {
        char inChar = (char)serial.read();
        
        if (inChar == '{') {
            // Début d'un nouveau message JSON
            received = "{";
        }
        else if (inChar == '}') {
            // Fin du message JSON
            received += "}";
            break;
        }
        else if (received.length() > 0) {
            // Accumulation des caractères (ignore \n)
            if (inChar != '\n') {
                received += inChar;
            }
        }
    }
}

void processReceivedJson(String jsonString, BmsData24V& bms) {
    // Création du document JSON
    DynamicJsonDocument doc(1024);
    
    // Parse du JSON
    DeserializationError error = deserializeJson(doc, jsonString);
    
    if (error) {
        Serial.print("Erreur de parsing JSON: ");
        Serial.println(error.c_str());
        Serial.print("Données reçues: ");
        Serial.println(jsonString);
        return;
    }
    
    // Vérification du type de données
    if (!doc.containsKey("Data")) {
        Serial.println("Champ 'Data' manquant");
        return;
    }
    
    String dataType = doc["Data"].as<String>();
    
    if (dataType == "BMS") {
        parseBmsData(doc, bms);
    }
    else if (dataType == "Temperatures") {
        parseTemperatureData(doc);
    }
    else {
        Serial.println("Type de données non reconnu: " + dataType);
    }
}

void parseBmsData(DynamicJsonDocument& doc, BmsData24V& bms) {
    Serial.println("Parsing des données BMS…");

    // Tensions
    if (doc.containsKey("V_total")) {
        bms.totalVoltage = doc["V_total"].as<float>();
        Serial.print("Tension totale: "); Serial.println(bms.totalVoltage);
    }
    if (doc.containsKey("V_gather")) {
        bms.gatherVoltage = doc["V_gather"].as<float>();
        Serial.print("Tension gather: "); Serial.println(bms.gatherVoltage);
    }

    // Courant & SOC
    if (doc.containsKey("I")) {
        bms.current = doc["I"].as<float>();
        Serial.print("Courant: "); Serial.println(bms.current);
    }
    if (doc.containsKey("SOC")) {
        bms.soc = doc["SOC"].as<float>();
        Serial.print("SOC: "); Serial.println(bms.soc);
    }

    // Cellules extrêmes
    if (doc.containsKey("V_cell_max")) {
        // tu envoies cette valeur en V (3 décimales)
        bms.maxCellVoltage = uint16_t(doc["V_cell_max"].as<float>() * 1000);
        Serial.print("V_cell_max (mV): "); Serial.println(bms.maxCellVoltage);
    }
    if (doc.containsKey("Cell_max_idx")) {
        bms.maxCellNumber = doc["Cell_max_idx"].as<uint8_t>();
        Serial.print("Cell_max_idx: "); Serial.println(bms.maxCellNumber);
    }
    if (doc.containsKey("V_cell_min")) {
        bms.minCellVoltage = uint16_t(doc["V_cell_min"].as<float>() * 1000);
        Serial.print("V_cell_min (mV): "); Serial.println(bms.minCellVoltage);
    }
    if (doc.containsKey("Cell_min_idx")) {
        bms.minCellNumber = doc["Cell_min_idx"].as<uint8_t>();
        Serial.print("Cell_min_idx: "); Serial.println(bms.minCellNumber);
    }

    // Températures
    if (doc.containsKey("T_max")) {
        bms.maxTemp = doc["T_max"].as<int8_t>();
        Serial.print("T_max: "); Serial.println(bms.maxTemp);
    }
    if (doc.containsKey("T_min")) {
        bms.minTemp = doc["T_min"].as<int8_t>();
        Serial.print("T_min: "); Serial.println(bms.minTemp);
    }

    // Status d'erreur en hex string "0xAABBCCDDEEFF…"
    if (doc.containsKey("Errors")) {
        const char* errStr = doc["Errors"];
        // skip "0x" si présent, base 16
        bms.errorStatus = strtoull(errStr + (errStr[1]=='x'?2:0), NULL, 16);
        Serial.print("Errors: 0x"); Serial.println((unsigned long long) bms.errorStatus, HEX);
    }

    Serial.println("--- Données BMS mises à jour ---");
}

void parseTemperatureData(DynamicJsonDocument& doc) {
    if (doc.containsKey("ThermistorNumber")) {
        temperatureData.thermistorNumber = doc["ThermistorNumber"].as<uint8_t>();
        
        // Parse des températures individuelles
        for (int i = 0; i < temperatureData.thermistorNumber && i < 16; i++) {
            String key = String(i);
            if (doc.containsKey(key)) {
                temperatureData.temperatures[i] = doc[key].as<uint16_t>();
            }
        }
        
        Serial.println("--- Données de température mises à jour ---");
    }
}

// === FONCTIONS UTILITAIRES ===

// Fonction pour obtenir les données BMS
/*BmsData24V getBmsData() {
    return bmsData24V;
}*/

// Fonction pour obtenir les données de température
TemperatureData getTemperatureData() {
    return temperatureData;
}

// Fonction pour vérifier si les données BMS sont valides
bool isBmsDataValid(BmsData24V& bms) {
    return (bms.totalVoltage > 0 && 
            bms.soc >= 0 && 
            bms.maxTemp > -100);
}


// Fonction pour obtenir une température spécifique (en °C)
float getTemperature(uint8_t thermistorIndex) {
    if (thermistorIndex >= temperatureData.thermistorNumber || 
        !temperatureData.dataValid ||
        temperatureData.temperatures[thermistorIndex] == 0xFFFF) {
        return -999.0;  // Valeur d'erreur
    }
    
    return temperatureData.temperatures[thermistorIndex] / 100.0;
}

// Fonction pour réinitialiser toutes les données
void resetAllData(BmsData24V& bms) {
    // Reset BMS
    bms.totalVoltage = -1;
    bms.gatherVoltage = -1;
    bms.current = -1;
    bms.soc = -1;
    bms.maxCellVoltage = 0;
    bms.maxCellNumber = 0;
    bms.minCellVoltage = 0;
    bms.minCellNumber = 0;
    bms.maxTemp = -100;
    bms.minTemp = -100;
    bms.errorStatus = 0;
    
    // Reset températures
    temperatureData.thermistorNumber = 0;
    temperatureData.dataValid = false;
    for (int i = 0; i < 16; i++) {
        temperatureData.temperatures[i] = 0xFFFF;
    }
}

void setup() {
    Serial.begin(115200);
    SerialBMS1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN1, TX_PIN1);
    
    SerialBMS2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN2, TX_PIN2);
    
    Serial.println("=== ESP32 UART vers CAN ===");
    Serial.println("Réception UART et transmission CAN");
    
    // Initialisation du CAN
    initCAN();
    
    Serial.println("En attente de données UART...");

        
    bmsData24V1.can_id_bms_1890 = CAN_ID_BMS1_1890;
    bmsData24V1.can_id_bms_1891 = CAN_ID_BMS1_1891;
    bmsData24V1.can_id_bms_1892 = CAN_ID_BMS1_1892;
    bmsData24V1.can_id_bms_1898 = CAN_ID_BMS1_1898;


    bmsData24V2.can_id_bms_1890 = CAN_ID_BMS2_1890;
    bmsData24V2.can_id_bms_1891 = CAN_ID_BMS2_1891;
    bmsData24V2.can_id_bms_1892 = CAN_ID_BMS2_1892;
    bmsData24V2.can_id_bms_1898 = CAN_ID_BMS2_1898;
}

void loop() {
    // Lecture des données UART
    readUartData(SerialBMS1, receivedData1);
    readUartData(SerialBMS2, receivedData2);

    // Traitement des données complètes
    Serial.print("JSON 2 reçu: ");
    Serial.println(receivedData1);  // DEBUG: Afficher les données brutes
    
    Serial.print("JSON 1 reçu: ");
    Serial.println(receivedData2);  // DEBUG: Afficher les données brutes
    
    processReceivedJson(receivedData1, bmsData24V1);
    processReceivedJson(receivedData2, bmsData24V2);
    
    // Reset pour la prochaine réception
    receivedData1 = "";
    receivedData2 = "";

    
    // Envoi périodique sur CAN
    static unsigned long lastTransmitTime = 0;
    if (millis() - lastTransmitTime >= 1000) {  // Envoi toutes les secondes
        sendBmsDataOverCAN(bmsData24V1);
        sendBmsDataOverCAN(bmsData24V2);
        sendTemperatureDataOverCAN();
        lastTransmitTime = millis();
    }
    
    delay(10);
}

