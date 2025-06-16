#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <ESP32-TWAI-CAN.hpp>

// Structure pour stocker les données BMS 24V (identique à votre code d'envoi)
struct BmsData24V {
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
struct TemperatureData {
    uint8_t thermistorNumber;
    uint16_t temperatures[16];  // Maximum 16 thermistances (ajustable)
    
    // Constructeur pour initialiser le tableau
    TemperatureData(uint8_t Tn = 16) {
        for (int i = 0; i < 16; i++) {
            temperatures[i] = 0xFFFF;  // Valeur invalide par défaut
        }
        thermistorNumber = Tn;
    }
};

BmsData24V bmsData24V;
TemperatureData temperatureData;

// Configuration UART (correspondant à votre code d'envoi)
#define RX_PIN 22  // Correspond à UART_RX_PIN de l'émetteur
#define TX_PIN 21  // Correspond à UART_TX_PIN de l'émetteur
#define BAUD_RATE 9600  // Même vitesse que votre code d'envoi

// Configuration CAN
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// IDs CAN correspondant à vos fonctions de décodage
#define CAN_ID_BMS_1890       0x762  // 1890 en décimal = 0x762 en hex
#define CAN_ID_BMS_1891       0x763  // 1891 en décimal = 0x763 en hex  
#define CAN_ID_BMS_1892       0x764  // 1892 en décimal = 0x764 en hex
#define CAN_ID_BMS_1898       0x76A  // 1898 en décimal = 0x76A en hex
#define CAN_ID_TEMP_DATA      0x200  // ID pour les données de température

HardwareSerial SerialFromMaster(1); // Utilise UART1
String receivedData = "";

void setup() {
    Serial.begin(115200);
    SerialFromMaster.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    
    Serial.println("=== ESP32 UART vers CAN ===");
    Serial.println("Réception UART et transmission CAN");
    
    // Initialisation du CAN
    initCAN();
    
    Serial.println("En attente de données UART...");
}

void loop() {
    // Lecture des données UART
    readUartData();
    
    // Traitement des données complètes
    if (dataComplete) {
        Serial.print("JSON reçu: ");
        Serial.println(receivedData);  // DEBUG: Afficher les données brutes
        processReceivedJson(receivedData);
        
        // Reset pour la prochaine réception
        receivedData = "";
        dataComplete = false;
    }
    
    // Envoi périodique sur CAN
    static unsigned long lastTransmitTime = 0;
    if (millis() - lastTransmitTime >= 1000) {  // Envoi toutes les secondes
        sendBmsDataOverCAN();
        sendTemperatureDataOverCAN();
        lastTransmitTime = millis();
    }
    
    delay(10);
}

void initCAN() {
    // Configuration du CAN avec la bibliothèque ESP32-TWAI-CAN
    ESP32Can.CANInit();
    
    if (ESP32Can.begin(ESP32Can.convertSpeed(500))) {  // 500 kbps
        Serial.println("CAN initialisé avec succès");
        canInitialized = true;
    } else {
        Serial.println("Erreur initialisation CAN");
    }
}


void sendBmsDataOverCAN() {
    if (!isBmsDataValid()) {
        return;  // Ne pas envoyer de données invalides
    }
    
    #pragma region frame1890
    // Message 1890: Données de base (compatible avec processReceivedData1890)
    CanFrame frame1890;
    frame1890.identifier = identifier;
    frame1890.extd = 1;
    frame1890.data_length_code = 8;
    
    // Formatage selon processReceivedData1890:
    // totalVoltage * 10 sur 2 bytes (big endian)
    uint16_t totalVolt = (uint16_t)(bmsData24V.totalVoltage * 10);
    frame1890.data[0] = (totalVolt >> 8) & 0xFF;
    frame1890.data[1] = totalVolt & 0xFF;
    
    // gatherVoltage * 10 sur 2 bytes (big endian)
    uint16_t gatherVolt = (uint16_t)(bmsData24V.gatherVoltage * 10);
    frame1890.data[2] = (gatherVolt >> 8) & 0xFF;
    frame1890.data[3] = gatherVolt & 0xFF;
    
    // current * 10 + 30000 sur 2 bytes (big endian)
    uint16_t current = (uint16_t)((bmsData24V.current * 10) + 30000);
    frame1890.data[4] = (current >> 8) & 0xFF;
    frame1890.data[5] = current & 0xFF;
    
    // SOC * 10 sur 2 bytes (big endian), 0xFFFF si invalide
    uint16_t socValue;
    if (bmsData24V.soc < 0) {
        socValue = 0xFFFF;
    } else {
        socValue = (uint16_t)(bmsData24V.soc * 10);
    }
    frame1890.data[6] = (socValue >> 8) & 0xFF;
    frame1890.data[7] = socValue & 0xFF;
    
    if (ESP32Can.CANWriteFrame(&frame1890) == 0) {
        Serial.println("CAN: Trame 1890 (données de base) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1890");
    }
    #pragma endregion

    #pragma region frame1891
    // Message 1891: Données cellules (compatible avec processReceivedData1891)
    CanFrame frame1891;
    frame1891.identifier = identifier;
    frame1891.extd = 1;
    frame1891.data_length_code = 8;
    
    // maxCellVoltage sur 2 bytes (big endian)
    frame1891.data[0] = (bmsData24V.maxCellVoltage >> 8) & 0xFF;
    frame1891.data[1] = bmsData24V.maxCellVoltage & 0xFF;
    // maxCellNumber sur 1 byte
    frame1891.data[2] = bmsData24V.maxCellNumber;
    // minCellVoltage sur 2 bytes (big endian)
    frame1891.data[3] = (bmsData24V.minCellVoltage >> 8) & 0xFF;
    frame1891.data[4] = bmsData24V.minCellVoltage & 0xFF;
    // minCellNumber sur 1 byte
    frame1891.data[5] = bmsData24V.minCellNumber;
    
    // Bytes 6-7 réservés
    frame1891.data[6] = 0x00;
    frame1891.data[7] = 0x00;
    
    if (ESP32Can.CANWriteFrame(&frame1891) == 0) {
        Serial.println("CAN: Trame 1891 (données cellules) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1891");
    }
    #pragma endregion

    #pragma region frame1892
    // Message 1892: Températures (compatible avec processReceivedData1892)
    CanFrame frame1892
    frame1892.identifier = identifier;
    frame1892.extd = 1;
    frame1892.data_length_code = 8;
    
    // maxTemp + 40 sur byte 0
    frame1892.data[0] = (uint8_t)(bmsData24V.maxTemp + 40);
    
    // Byte 1 réservé
    frame1892.data[1] = 0x00;
    
    // minTemp + 40 sur byte 2
    frame1892.data[2] = (uint8_t)(bmsData24V.minTemp + 40);
    
    // Bytes 3-7 réservés
    for (int i = 3; i < 8; i++) {
        frame1892.data[i] = 0x00;
    }
    
    if (ESP32Can.CANWriteFrame(&frame1892) == 0) {
        Serial.println("CAN: Trame 1892 (températures) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1892");
    }
    #pragma endregion

    #pragma region frame1898
    // Message 1898: Erreurs (compatible avec processReceivedData1898)
    CanFrame frame1898;
    frame1898.identifier = identifier;
    frame1898.extd = 1;
    frame1898.data_length_code = 8;
    
    // errorStatus sur 8 bytes (little endian)
    for (int i = 0; i < 8; i++) {
        frame1898.data[i] = (bmsData24V.errorStatus >> (8 * i)) & 0xFF;
    }
    
    if (ESP32Can.CANWriteFrame(&frame1898) == 0) {
        Serial.println("CAN: Trame 1898 (erreurs) envoyée");
    } else {
        Serial.println("CAN: Erreur envoi trame 1898");
    }
    #pragma endregion
}

void sendTemperatureDataOverCAN() {
    if (!isTemperatureDataValid()) {
        return;  // Ne pas envoyer de données invalides
    }
    
    // Message température: Envoie les données de température
    
    // Nombre de thermistances
    for (int i = 0; i < temperatureData.thermistorNumber; i++) {        
        CanFrame tempFrame;
        tempFrame.identifier = 0x200 + i;
        tempFrame.extd = 1;
        tempFrame.data_length_code = 8;

        tempFrame.data[0] = (temperatureData.temperatures[i] >> 8) & 0xFF ;
        tempFrame.data[1] = temperatureData.temperatures[i]  & 0xFF;

        
        if !(ESP32Can.writeFrame(tempFrame, 1000)) {
              Serial.printf("Erreur envoi trame 0x60%d \n", i);
        }
        else {
            Serial.printf("Reussite envoi trame 0x60%d \n", i);
        }


    }
}

void readUartData() {
    while (SerialFromMaster.available()) {
        char inChar = (char)SerialFromMaster.read();
        
        if (inChar == '{') {
            // Début d'un nouveau message JSON
            receivedData = "{";
        }
        else if (inChar == '}') {
            // Fin du message JSON
            receivedData += "}";
            dataComplete = true;
            break;
        }
        else if (receivedData.length() > 0) {
            // Accumulation des caractères (ignore \n)
            if (inChar != '\n') {
                receivedData += inChar;
            }
        }
    }
}

void processReceivedJson(String jsonString) {
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
        parseBmsData(doc);
    }
    else if (dataType == "Temperatures") {
        parseTemperatureData(doc);
    }
    else {
        Serial.println("Type de données non reconnu: " + dataType);
    }
}

void parseBmsData(DynamicJsonDocument& doc) {
    Serial.println("Parsing des données BMS…");

    // Tensions
    if (doc.containsKey("V_total")) {
        bmsData24V.totalVoltage = doc["V_total"].as<float>();
        Serial.print("Tension totale: "); Serial.println(bmsData24V.totalVoltage);
    }
    if (doc.containsKey("V_gather")) {
        bmsData24V.gatherVoltage = doc["V_gather"].as<float>();
        Serial.print("Tension gather: "); Serial.println(bmsData24V.gatherVoltage);
    }

    // Courant & SOC
    if (doc.containsKey("I")) {
        bmsData24V.current = doc["I"].as<float>();
        Serial.print("Courant: "); Serial.println(bmsData24V.current);
    }
    if (doc.containsKey("SOC")) {
        bmsData24V.soc = doc["SOC"].as<float>();
        Serial.print("SOC: "); Serial.println(bmsData24V.soc);
    }

    // Cellules extrêmes
    if (doc.containsKey("V_cell_max")) {
        // tu envoies cette valeur en V (3 décimales)
        bmsData24V.maxCellVoltage = uint16_t(doc["V_cell_max"].as<float>() * 1000);
        Serial.print("V_cell_max (mV): "); Serial.println(bmsData24V.maxCellVoltage);
    }
    if (doc.containsKey("Cell_max_idx")) {
        bmsData24V.maxCellNumber = doc["Cell_max_idx"].as<uint8_t>();
        Serial.print("Cell_max_idx: "); Serial.println(bmsData24V.maxCellNumber);
    }
    if (doc.containsKey("V_cell_min")) {
        bmsData24V.minCellVoltage = uint16_t(doc["V_cell_min"].as<float>() * 1000);
        Serial.print("V_cell_min (mV): "); Serial.println(bmsData24V.minCellVoltage);
    }
    if (doc.containsKey("Cell_min_idx")) {
        bmsData24V.minCellNumber = doc["Cell_min_idx"].as<uint8_t>();
        Serial.print("Cell_min_idx: "); Serial.println(bmsData24V.minCellNumber);
    }

    // Températures
    if (doc.containsKey("T_max")) {
        bmsData24V.maxTemp = doc["T_max"].as<int8_t>();
        Serial.print("T_max: "); Serial.println(bmsData24V.maxTemp);
    }
    if (doc.containsKey("T_min")) {
        bmsData24V.minTemp = doc["T_min"].as<int8_t>();
        Serial.print("T_min: "); Serial.println(bmsData24V.minTemp);
    }

    // Status d'erreur en hex string "0xAABBCCDDEEFF…"
    if (doc.containsKey("Errors")) {
        const char* errStr = doc["Errors"];
        // skip "0x" si présent, base 16
        bmsData24V.errorStatus = strtoull(errStr + (errStr[1]=='x'?2:0), NULL, 16);
        Serial.print("Errors: 0x"); Serial.println((unsigned long long) bmsData24V.errorStatus, HEX);
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
        
        temperatureData.dataValid = true;
        Serial.println("--- Données de température mises à jour ---");
    }
}

// === FONCTIONS UTILITAIRES ===

// Fonction pour obtenir les données BMS
BmsData24V getBmsData() {
    return bmsData24V;
}

// Fonction pour obtenir les données de température
TemperatureData getTemperatureData() {
    return temperatureData;
}

// Fonction pour vérifier si les données BMS sont valides
bool isBmsDataValid() {
    return (bmsData24V.totalVoltage > 0 && 
            bmsData24V.soc >= 0 && 
            bmsData24V.maxTemp > -100);
}

// Fonction pour vérifier si les données de température sont valides
bool isTemperatureDataValid() {
    return temperatureData.dataValid;
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
void resetAllData() {
    // Reset BMS
    bmsData24V.totalVoltage = -1;
    bmsData24V.gatherVoltage = -1;
    bmsData24V.current = -1;
    bmsData24V.soc = -1;
    bmsData24V.maxCellVoltage = 0;
    bmsData24V.maxCellNumber = 0;
    bmsData24V.minCellVoltage = 0;
    bmsData24V.minCellNumber = 0;
    bmsData24V.maxTemp = -100;
    bmsData24V.minTemp = -100;
    bmsData24V.errorStatus = 0;
    
    // Reset températures
    temperatureData.thermistorNumber = 0;
    temperatureData.dataValid = false;
    for (int i = 0; i < 16; i++) {
        temperatureData.temperatures[i] = 0xFFFF;
    }
}
