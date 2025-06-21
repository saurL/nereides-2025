//CODE QUI RECOIT LES DONNEES DES 2 BMS EN UART ET QUI LES STOKE DANS DEX INSTANCES DE STRUCTURES ET QUI LES ENVOIE EN CAN : 

//CODE RECEPTEUR DUAL UART + ENVOI CAN - BMS 1 ET BMS 2

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
    int8_t maxTemp = -100;  // en °C (vient du JSON BMS principal, pas des thermistances individuelles)
    int8_t minTemp = -100;  // en °C (vient du JSON BMS principal, pas des thermistances individuelles)
    uint64_t errorStatus = 0;
};

// Structure pour stocker les données de température des thermistances individuelles
struct TemperatureData {
    uint8_t thermistorNumber = 0;
    uint16_t temperatures[16];  // Maximum 16 thermistances (valeur x100, ex: 25.50°C -> 2550)
    bool dataValid = false;
    
    // CORRECTION: Utiliser uint16_t au lieu de int16_t pour supporter les grandes valeurs
    uint16_t overallMinTemp = 9999; // Initialiser à une valeur haute pour le min
    uint16_t overallMaxTemp = 0;    // Initialiser à une valeur basse pour le max
    
    // Constructeur pour initialiser le tableau
    TemperatureData() {
        for (int i = 0; i < 16; i++) {
            temperatures[i] = 0xFFFF;  // Valeur invalide par défaut
        }
    }
};

// === DONNÉES POUR LES DEUX BMS ===
BmsData24V bmsData24V_1;        // Données BMS 1
BmsData24V bmsData24V_2;        // Données BMS 2
TemperatureData temperatureData_1;  // Températures BMS 1 (thermistances individuelles)
TemperatureData temperatureData_2;  // Températures BMS 2 (thermistances individuelles)

// === CONFIGURATION CAN TWAI ===
#define CAN_TX_PIN 4
#define CAN_RX_PIN 5
#define CAN_BITRATE 250000  // 250 kbps

// Instance de la bibliothèque CAN
CanFrame rxFrame, txFrame;

// Identifiants CAN pour BMS 1 (ID: 0x001-0x004) et BMS 2 (ID: 0x005-0x008)
#define CAN_ID_BMS1_BASE 0x001
#define CAN_ID_BMS2_BASE 0x005

// Identifiants CAN pour les températures MIN/MAX des thermistances individuelles
#define CAN_ID_TEMP1_MINMAX 0x009 // ID pour min/max températures BMS 1
#define CAN_ID_TEMP2_MINMAX 0x010 // ID pour min/max températures BMS 2

// === CONFIGURATION UART DUAL ===
// BMS 1 - UART1
#define RX_PIN_1 21
#define TX_PIN_1 22
#define BAUD_RATE 9600

// BMS 2 - UART2
#define RX_PIN_2 16
#define TX_PIN_2 17

HardwareSerial SerialFromBMS1(1); // UART1 pour BMS1
HardwareSerial SerialFromBMS2(2); // UART2 pour BMS2

// Variables de réception pour chaque UART
String receivedData_1 = "";
String receivedData_2 = "";
bool dataComplete_1 = false;
bool dataComplete_2 = false;

// DEBUG FLAG pour activer/désactiver les prints CAN détaillés
#define DEBUG_CAN_SEND true 

void setup() {
    Serial.begin(115200);
    
    // Initialisation des deux UART
    SerialFromBMS1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
    SerialFromBMS2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN_2, TX_PIN_2);
    
    // Initialisation du CAN TWAI
    initializeCAN();
    
    Serial.println("=== Récepteur ESP32 DUAL BMS + CAN ===");
    Serial.println("BMS 1: UART1 (pins 22/21) -> CAN ID 0x001-0x004");
    Serial.println("BMS 2: UART2 (pins 16/17) -> CAN ID 0x005-0x008");
    Serial.println("Températures Min/Max: BMS 1 -> 0x009, BMS 2 -> 0x010");
    Serial.println("CAN: TX=GPIO5, RX=GPIO4,250kbps");
    Serial.println("En attente de données...");
}

void loop() {
    // testCanTransmission(); // Désactivez cette fonction de test pour l'opération normale
    
    // Lecture des données des deux UART
    readUartData(SerialFromBMS1, receivedData_1, dataComplete_1, 1);
    readUartData(SerialFromBMS2, receivedData_2, dataComplete_2, 2);
    
    // Traitement des données BMS 1
    if (dataComplete_1) {
        if (DEBUG_CAN_SEND) { // Conditionnel pour les prints de débogage
            Serial.print("BMS 1 - JSON reçu: ");
            Serial.println(receivedData_1);
        }
        processReceivedJson(receivedData_1, 1);
        
        // Envoyer les données BMS 1 via CAN
        sendBmsDataToCAN(1); 
        // Envoyer les données de température MIN/MAX BMS 1 via CAN
        sendMinMaxTemperatureToCAN(1); // APPEL MODIFIÉ
        
        receivedData_1 = "";
        dataComplete_1 = false;
    }
    
    // Traitement des données BMS 2
    if (dataComplete_2) {
        if (DEBUG_CAN_SEND) { // Conditionnel pour les prints de débogage
            Serial.print("BMS 2 - JSON reçu: ");
            Serial.println(receivedData_2);
        }
        processReceivedJson(receivedData_2, 2);
        
        // Envoyer les données BMS 2 via CAN
        sendBmsDataToCAN(2);
        // Envoyer les données de température MIN/MAX BMS 2 via CAN
        sendMinMaxTemperatureToCAN(2); // APPEL MODIFIÉ
        
        receivedData_2 = "";
        dataComplete_2 = false;
    }
    
    // DEBUG: Afficher périodiquement les valeurs des deux structures
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 10000) {  // Toutes les 10 secondes
        displayAllBmsData();
        lastDebugTime = millis();
    }
    
    delay(10); // Petit délai pour laisser le temps aux autres tâches FreeRTOS
}

void readUartData(HardwareSerial& serial, String& receivedData, bool& dataComplete, int bmsNumber) {
    while (serial.available()) {
        char inChar = (char)serial.read();
        
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

void processReceivedJson(String jsonString, int bmsNumber) {
    // Création du document JSON
    DynamicJsonDocument doc(1024);
    
    // Parse du JSON
DeserializationError error = deserializeJson(doc, jsonString);    
    if (error) {
        Serial.print("BMS ");
        Serial.print(bmsNumber);
        Serial.print(" - Erreur de parsing JSON: ");
        Serial.println(error.c_str());
        return;
    }
    
    // Vérification du type de données
    if (!doc.containsKey("Data")) {
        Serial.print("BMS ");
        Serial.print(bmsNumber);
        Serial.println(" - Champ 'Data' manquant");
        return;
    }
    
    String dataType = doc["Data"].as<String>();
    
    if (dataType == "BMS") {
        parseBmsData(doc, bmsNumber);
    }
    else if (dataType == "Temperatures") {
        parseTemperatureData(doc, bmsNumber);
        calculateMinMaxTemperatures(bmsNumber); // NOUVEL APPEL: Calculer min/max après parsing
    }
    else {
        Serial.print("BMS ");
        Serial.print(bmsNumber);
        Serial.println(" - Type de données non reconnu: " + dataType);
    }
}

void parseBmsData(DynamicJsonDocument& doc, int bmsNumber) {
    if (DEBUG_CAN_SEND) {
        Serial.print("BMS ");
        Serial.print(bmsNumber);
        Serial.println(" - Parsing des données BMS…");
    }

    // Sélection de la structure appropriée
    BmsData24V* bmsData = (bmsNumber == 1) ? &bmsData24V_1 : &bmsData24V_2;

    // Tensions
    if (doc.containsKey("V_total")) {
        bmsData->totalVoltage = doc["V_total"].as<float>();
    }
    if (doc.containsKey("V_gather")) {
        bmsData->gatherVoltage = doc["V_gather"].as<float>();
    }

    // Courant & SOC
    if (doc.containsKey("I")) {
        bmsData->current = doc["I"].as<float>();
    }
    if (doc.containsKey("SOC")) {
        bmsData->soc = doc["SOC"].as<float>();
    }

    // Cellules extrêmes
    if (doc.containsKey("V_cell_max")) {
        bmsData->maxCellVoltage = uint16_t(doc["V_cell_max"].as<float>() * 1000);
    }
    if (doc.containsKey("Cell_max_idx")) {
        bmsData->maxCellNumber = doc["Cell_max_idx"].as<uint8_t>();
    }
    if (doc.containsKey("V_cell_min")) {
        bmsData->minCellVoltage = uint16_t(doc["V_cell_min"].as<float>() * 1000);
    }
    if (doc.containsKey("Cell_min_idx")) {
        bmsData->minCellNumber = doc["Cell_min_idx"].as<uint8_t>();
    }

    // Températures (max/min issues du JSON BMS principal, pas des thermistances individuelles)
    if (doc.containsKey("T_max")) {
        bmsData->maxTemp = doc["T_max"].as<int8_t>();
    }
    if (doc.containsKey("T_min")) {
        bmsData->minTemp = doc["T_min"].as<int8_t>();
    }

    // Status d'erreur
    if (doc.containsKey("Errors")) {
        const char* errStr = doc["Errors"];
        bmsData->errorStatus = strtoull(errStr + (errStr[1]=='x'?2:0), NULL, 16);
    }

    if (DEBUG_CAN_SEND) {
        Serial.print("BMS ");
        Serial.print(bmsNumber);
        Serial.println(" - Données BMS mises à jour");
    }
}

void parseTemperatureData(DynamicJsonDocument& doc, int bmsNumber) {
    // Sélection de la structure appropriée
    TemperatureData* tempData = (bmsNumber == 1) ? &temperatureData_1 : &temperatureData_2;
    
    if (doc.containsKey("ThermistorNumber")) {
        tempData->thermistorNumber = doc["ThermistorNumber"].as<uint8_t>();
        
        // Parse des températures individuelles
        for (int i = 0; i < tempData->thermistorNumber && i < 16; i++) {
            String key = String(i);
            if (doc.containsKey(key)) {
                // Multiplier par 100 pour stocker avec 2 décimales (ex: 25.50°C -> 2550)
                tempData->temperatures[i] = uint16_t(doc[key].as<float>() * 100); 
            }
        }
        
        tempData->dataValid = true;
        if (DEBUG_CAN_SEND) {
            Serial.print("BMS ");
            Serial.print(bmsNumber);
            Serial.println(" - Données de température mises à jour");
        }
    }
}

// NOUVELLE FONCTION: Calculer les températures min et max parmi les thermistances reçues
void calculateMinMaxTemperatures(int bmsNumber) {
    TemperatureData* tempData = (bmsNumber == 1) ? &temperatureData_1 : &temperatureData_2;
    
    if (!tempData->dataValid || tempData->thermistorNumber == 0) {
        tempData->overallMinTemp = 9999;
        tempData->overallMaxTemp = 0;
        return;
    }
    
    uint16_t currentMin, currentMax;
    bool firstValidFound = false;
    
    if (DEBUG_CAN_SEND) {
        Serial.printf("BMS %d - Calcul Min/Max sur %d thermistances:\n", bmsNumber, tempData->thermistorNumber);
    }
    
    for (int i = 0; i < tempData->thermistorNumber && i < 16; i++) {
        if (tempData->temperatures[i] != 0xFFFF) { // Vérifier si la température est valide
            uint16_t tempCenti = tempData->temperatures[i];
            
            if (DEBUG_CAN_SEND) {
                Serial.printf("  T%d: %d (%.2f°C)\n", i, tempCenti, tempCenti / 1000.0);
            }
            
            if (!firstValidFound) {
                currentMin = tempCenti;
                currentMax = tempCenti;
                firstValidFound = true;
            } else {
                if (tempCenti < currentMin) {
                    currentMin = tempCenti;
                }
                if (tempCenti > currentMax) {
                    currentMax = tempCenti;
                }
            }
        }
    }
    
    if (firstValidFound) {
        tempData->overallMinTemp = currentMin;
        tempData->overallMaxTemp = currentMax;
        
        if (DEBUG_CAN_SEND) {
            Serial.printf("BMS %d - Températures calculées: Min=%d (%.2f°C), Max=%d (%.2f°C)\n",
                         bmsNumber, tempData->overallMinTemp, tempData->overallMinTemp / 1000.0,
                         tempData->overallMaxTemp, tempData->overallMaxTemp / 1000.0);
        }
    } else {
        tempData->overallMinTemp = 9999;
        tempData->overallMaxTemp = 0;
        
        if (DEBUG_CAN_SEND) {
            Serial.printf("BMS %d - Aucune température valide trouvée\n", bmsNumber);
        }
    }
}
void displayAllBmsData() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║            ÉTAT DES DEUX BMS             ║");
    Serial.println("╠════════════════════════════════════════╣");
    
    // Affichage BMS 1
    Serial.println("║                  BMS 1                   ║");
    Serial.println("╠════════════════════════════════════════╣");
    displaySingleBmsData(bmsData24V_1, 1);
    displaySingleTemperatureData(temperatureData_1, 1); // Afficher les températures détaillées
    
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║                  BMS 2                   ║");
    Serial.println("╠════════════════════════════════════════╣");
    displaySingleBmsData(bmsData24V_2, 2);
    displaySingleTemperatureData(temperatureData_2, 2); // Afficher les températures détaillées
    
    // Calculs combinés
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║              BATTERIE TOTALE             ║");
    Serial.println("╠════════════════════════════════════════╣");
    displayCombinedData();
    
    Serial.println("╚════════════════════════════════════════╝\n");
}

void displaySingleBmsData(BmsData24V& bmsData, int bmsNumber) {
    Serial.print("║ Tension: ");
    Serial.print(bmsData.totalVoltage, 2);
    Serial.print("V | Courant: ");
    Serial.print(bmsData.current, 2);
    Serial.println("A          ║");
    
    Serial.print("║ SOC: ");
    Serial.print(bmsData.soc, 1);
    Serial.print("% | Temp: ");
    Serial.print(bmsData.minTemp);
    Serial.print("°C à ");
    Serial.print(bmsData.maxTemp);
    Serial.println("°C            ║");
    
    Serial.print("║ Cellule max: ");
    Serial.print(bmsData.maxCellVoltage);
    Serial.print("mV (#");
    Serial.print(bmsData.maxCellNumber);
    Serial.println(")       ║");
    
    Serial.print("║ Cellule min: ");
    Serial.print(bmsData.minCellVoltage);
    Serial.print("mV (#");
    Serial.print(bmsData.minCellNumber);
    Serial.println(")       ║");
}

void displaySingleTemperatureData(TemperatureData& tempData, int bmsNumber) {
    Serial.print("║ Températures individuelles BMS ");
    Serial.print(bmsNumber);
    Serial.println(" ║");
    if (tempData.dataValid && tempData.thermistorNumber > 0) {
        Serial.printf("║ Min: %.2f°C | Max: %.2f°C (calculé)       ║\n", 
                      tempData.overallMinTemp / 100.0, tempData.overallMaxTemp / 100.0);

        Serial.println("║ Détail des thermistances:             ║");
        for (int i = 0; i < tempData.thermistorNumber && i < 16; i++) {
            if (tempData.temperatures[i] != 0xFFFF) { // N'afficher que les valeurs valides
                Serial.printf("║   T%02d: %.2f°C", i + 1, tempData.temperatures[i] / 100.0);
            } else {
                Serial.printf("║   T%02d: N/A     ", i + 1);
            }
            if ((i + 1) % 2 == 0 || i == tempData.thermistorNumber - 1) { // 2 par ligne ou dernière
                Serial.println("      ║");
            } else {
                Serial.print(" |");
            }
        }
    } else {
        Serial.println("║   Aucune donnée de température individuelle.  ║");
    }
}


void displayCombinedData() {
    if (isBmsDataValid(1) && isBmsDataValid(2)) {
        float totalVoltage = bmsData24V_1.totalVoltage + bmsData24V_2.totalVoltage;
        float totalCurrent = bmsData24V_1.current + bmsData24V_2.current;
        float avgSOC = (bmsData24V_1.soc + bmsData24V_2.soc) / 2.0;
        
        Serial.print("║ Tension totale: ");
        Serial.print(totalVoltage, 2);
        Serial.println("V                  ║");
        
        Serial.print("║ Courant total: ");
        Serial.print(totalCurrent, 2);
        Serial.println("A                   ║");
        
        Serial.print("║ SOC moyen: ");
        Serial.print(avgSOC, 1);
        Serial.println("%                     ║");
    } else {
        Serial.println("║ Données incomplètes pour calcul total ║");
    }
}

// === FONCTIONS CAN TWAI ===

void initializeCAN() {
    // Initialisation du CAN avec la bibliothèque ESP32-TWAI-CAN
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
    
    if (ESP32Can.begin()) {
        Serial.println("CAN Bus initialisé avec succès (250 kbps)");
    } else {
        Serial.println("Erreur lors de l'initialisation du CAN Bus");
        return;
    }
    
    Serial.println("Driver CAN démarré avec succès");
}

void sendBmsDataToCAN(int bmsNumber) {
    BmsData24V* bmsData = (bmsNumber == 1) ? &bmsData24V_1 : &bmsData24V_2;
    uint32_t baseId = (bmsNumber == 1) ? CAN_ID_BMS1_BASE : CAN_ID_BMS2_BASE;
    
    // Vérifier que les données sont valides
    if (!isBmsDataValid(bmsNumber)) {
        if (DEBUG_CAN_SEND) {
            Serial.print("BMS ");
            Serial.print(bmsNumber);
            Serial.println(" - Données BMS invalides, pas d'envoi CAN");
        }
        return;
    }
    
    if (DEBUG_CAN_SEND) {
        Serial.print("Envoi données BMS ");
        Serial.print(bmsNumber);
        Serial.println(" via CAN...");
    }
    
    // Trame 1: Tensions et courant (ID: 0x001 ou 0x005)
    sendCanFrame4801(baseId, bmsData);
    delay(5);
    
    // Trame 2: Tensions cellules (ID: 0x002 ou 0x006)
    sendCanFrame4802(baseId + 1, bmsData);
    delay(5);
    
    // Trame 3: Températures (ID: 0x003 ou 0x007)
    // Note: Ces températures sont celles du BMS, pas les individuelles de thermistance
    sendCanFrame4803(baseId + 2, bmsData);
    delay(5);
    
    // Trame 4: Erreurs (ID: 0x004 ou 0x008)
    sendCanFrame4804(baseId + 3, bmsData);
    delay(5);
}

// NOUVELLE FONCTION MODIFIÉE: Envoi des températures MIN/MAX individuelles via CAN
void sendMinMaxTemperatureToCAN(int bmsNumber) {
    TemperatureData* tempData = (bmsNumber == 1) ? &temperatureData_1 : &temperatureData_2;
    uint32_t canId = (bmsNumber == 1) ? CAN_ID_TEMP1_MINMAX : CAN_ID_TEMP2_MINMAX;

    // Vérifier que les données de température sont valides et que min/max ont été calculés
    if (!isTemperatureDataValid(bmsNumber) || tempData->overallMinTemp == 0xFFFF || tempData->overallMaxTemp == 0x0000) {
        if (DEBUG_CAN_SEND) {
            Serial.print("BMS ");
            Serial.print(bmsNumber);
            Serial.println(" - Min/Max températures invalides, pas d'envoi CAN (ID 0x009/0x010)");
        }
        return;
    }

    txFrame.identifier = canId;
    txFrame.extd = false; // Trame standard
    txFrame.data_length_code = 4; // 2 températures (uint16_t) = 4 octets

    // Encodage de la température minimale (x100)
    txFrame.data[0] = (tempData->overallMinTemp >> 8) & 0xFF; // Octet de poids fort
    txFrame.data[1] = tempData->overallMinTemp & 0xFF;    // Octet de poids faible

    // Encodage de la température maximale (x100)
    txFrame.data[2] = (tempData->overallMaxTemp >> 8) & 0xFF; // Octet de poids fort
    txFrame.data[3] = tempData->overallMaxTemp & 0xFF;    // Octet de poids faible

    if (ESP32Can.writeFrame(txFrame)) {
        if (DEBUG_CAN_SEND) {
            Serial.printf("CAN 0x%03X envoyé: Min Temp=%.2f°C, Max Temp=%.2f°C (BMS %d)\n",
                          canId, tempData->overallMinTemp / 1000.0, tempData->overallMaxTemp / 1000.0, bmsNumber);
        }
    } else {
        Serial.printf("Erreur envoi CAN 0x%03X\n", canId);
    }
    delay(5);
}


void sendCanFrame4801(uint32_t canId, BmsData24V* bmsData) {
    // Configuration de la trame CAN
    txFrame.identifier = canId;
    txFrame.extd = false;  // Trame standard
    txFrame.data_length_code = 8;
    
    // Encodage selon le décodage de processReceivedData4801
    uint16_t totalVoltage = (uint16_t)(bmsData->totalVoltage * 10);
    uint16_t gatherVoltage = (uint16_t)(bmsData->gatherVoltage * 10);
    uint16_t current = (uint16_t)((bmsData->current * 10) + 30000);
    uint16_t soc = (bmsData->soc < 0) ? 0xFFFF : (uint16_t)(bmsData->soc * 10);
    
    txFrame.data[0] = (totalVoltage >> 8) & 0xFF;
    txFrame.data[1] = totalVoltage & 0xFF;
    txFrame.data[2] = (gatherVoltage >> 8) & 0xFF;
    txFrame.data[3] = gatherVoltage & 0xFF;
    txFrame.data[4] = (current >> 8) & 0xFF;
    txFrame.data[5] = current & 0xFF;
    txFrame.data[6] = (soc >> 8) & 0xFF;
    txFrame.data[7] = soc & 0xFF;
    
    if (ESP32Can.writeFrame(txFrame)) {
        if (DEBUG_CAN_SEND) {
            Serial.printf("CAN 0x%03X envoyé: V=%.1fV, I=%.1fA, SOC=%.1f%%\n", 
                         canId, bmsData->totalVoltage, bmsData->current, bmsData->soc);
        }
    } else {
        Serial.printf("Erreur envoi CAN 0x%03X\n", canId); // Toujours imprimer les erreurs
    }
}

void sendCanFrame4802(uint32_t canId, BmsData24V* bmsData) {
    // Configuration de la trame CAN
    txFrame.identifier = canId;
    txFrame.extd = false;  // Trame standard
    txFrame.data_length_code = 8;
    
    // Encodage selon le décodage de processReceivedData4802
    txFrame.data[0] = (bmsData->maxCellVoltage >> 8) & 0xFF;
    txFrame.data[1] = bmsData->maxCellVoltage & 0xFF;
    txFrame.data[2] = bmsData->maxCellNumber;
    txFrame.data[3] = (bmsData->minCellVoltage >> 8) & 0xFF;
    txFrame.data[4] = bmsData->minCellVoltage & 0xFF;
    txFrame.data[5] = bmsData->minCellNumber;
    txFrame.data[6] = 0x00;  // Données non utilisées
    txFrame.data[7] = 0x00;
    
    if (ESP32Can.writeFrame(txFrame)) {
        if (DEBUG_CAN_SEND) {
            Serial.printf("CAN 0x%03X envoyé: VMax=%dmV(#%d), VMin=%dmV(#%d)\n", 
                         canId, bmsData->maxCellVoltage, bmsData->maxCellNumber,
                         bmsData->minCellVoltage, bmsData->minCellNumber);
        }
    } else {
        Serial.printf("Erreur envoi CAN 0x%03X\n", canId);
    }
}

void sendCanFrame4803(uint32_t canId, BmsData24V* bmsData) {
    // Configuration de la trame CAN
    txFrame.identifier = canId;
    txFrame.extd = false;  // Trame standard
    txFrame.data_length_code = 8;
    
    // Encodage selon le décodage de processReceivedData4803
    txFrame.data[0] = (uint8_t)(bmsData->maxTemp + 40);
    txFrame.data[1] = 0x00;  // Données non utilisées
    txFrame.data[2] = (uint8_t)(bmsData->minTemp + 40);
    txFrame.data[3] = 0x00;
    txFrame.data[4] = 0x00;
    txFrame.data[5] = 0x00;
    txFrame.data[6] = 0x00;
    txFrame.data[7] = 0x00;
    
    if (ESP32Can.writeFrame(txFrame)) {
        if (DEBUG_CAN_SEND) {
            Serial.printf("CAN 0x%03X envoyé: TMax=%d°C, TMin=%d°C\n", 
                         canId, bmsData->maxTemp, bmsData->minTemp);
        }
    } else {
        Serial.printf("Erreur envoi CAN 0x%03X\n", canId);
    }
}

void sendCanFrame4804(uint32_t canId, BmsData24V* bmsData) {
    // Configuration de la trame CAN
    txFrame.identifier = canId;
    txFrame.extd = false;  // Trame standard
    txFrame.data_length_code = 8;
    
    // Encodage selon le décodage de processReceivedData4804
    for (int i = 0; i < 8; i++) {
        txFrame.data[i] = (bmsData->errorStatus >> (8 * i)) & 0xFF;
    }
    
    if (ESP32Can.writeFrame(txFrame)) {
        if (DEBUG_CAN_SEND) {
            Serial.printf("CAN 0x%03X envoyé: Erreurs=0x%llX\n", canId, bmsData->errorStatus);
        }
    } else {
        Serial.printf("Erreur envoi CAN 0x%03X\n", canId);
    }
}

// === FONCTIONS UTILITAIRES ÉTENDUES ===

// Fonctions pour obtenir les données d'un BMS spécifique
BmsData24V getBmsData(int bmsNumber) {
    return (bmsNumber == 1) ? bmsData24V_1 : bmsData24V_2;
}

TemperatureData getTemperatureData(int bmsNumber) {
    return (bmsNumber == 1) ? temperatureData_1 : temperatureData_2;
}

// Vérification validité des données BMS
bool isBmsDataValid(int bmsNumber) {
    BmsData24V* bmsData = (bmsNumber == 1) ? &bmsData24V_1 : &bmsData24V_2;
    return (bmsData->totalVoltage > 0 && 
            bmsData->soc >= 0 && 
            bmsData->maxTemp > -100); 
}

// Vérification validité des données de thermistance
bool isTemperatureDataValid(int bmsNumber) {
    TemperatureData* tempData = (bmsNumber == 1) ? &temperatureData_1 : &temperatureData_2;
    return tempData->dataValid && tempData->thermistorNumber > 0;
}

// Température spécifique d'un BMS (utilisant les 16 thermistances)
float getTemperature(int bmsNumber, uint8_t thermistorIndex) {
    TemperatureData* tempData = (bmsNumber == 1) ? &temperatureData_1 : &temperatureData_2;
    
    if (thermistorIndex >= tempData->thermistorNumber || 
        !tempData->dataValid ||
        tempData->temperatures[thermistorIndex] == 0xFFFF) { // 0xFFFF est notre marqueur d'invalidité
        return -999.0;
    }
    
    return tempData->temperatures[thermistorIndex] / 100.0; // Diviser par 100 pour retrouver la valeur réelle
}

// Calculs de batterie totale
float getTotalVoltage() {
    if (isBmsDataValid(1) && isBmsDataValid(2)) {
        return bmsData24V_1.totalVoltage + bmsData24V_2.totalVoltage;
    }
    return -1;
}

float getTotalCurrent() {
    if (isBmsDataValid(1) && isBmsDataValid(2)) {
        return bmsData24V_1.current + bmsData24V_2.current;
    }
    return -999;
}

float getAverageSOC() {
    if (isBmsDataValid(1) && isBmsDataValid(2)) {
        return (bmsData24V_1.soc + bmsData24V_2.soc) / 2.0;
    }
    return -1;
}

// Réinitialisation des données
void resetAllData() {
    resetBmsData(1);
    resetBmsData(2);
}

void resetBmsData(int bmsNumber) {
    BmsData24V* bmsData = (bmsNumber == 1) ? &bmsData24V_1 : &bmsData24V_2;
    TemperatureData* tempData = (bmsNumber == 1) ? &temperatureData_1 : &temperatureData_2;
    
    // Reset BMS
    bmsData->totalVoltage = -1;
    bmsData->gatherVoltage = -1;
    bmsData->current = -1;
    bmsData->soc = -1;
    bmsData->maxCellVoltage = 0;
    bmsData->maxCellNumber = 0;
    bmsData->minCellVoltage = 0;
    bmsData->minCellNumber = 0;
    bmsData->maxTemp = -100;
    bmsData->minTemp = -100;
    bmsData->errorStatus = 0;
    
    // Reset températures individuelles
    tempData->thermistorNumber = 0;
    tempData->dataValid = false;
    tempData->overallMinTemp = 0xFFFF;
    tempData->overallMaxTemp = 0x0000;
    for (int i = 0; i < 16; i++) {
        tempData->temperatures[i] = 0xFFFF;
    }
}

// === FONCTIONS DE TEST CAN (optionnelles) ===

void testCanTransmission() {
    Serial.println("Test transmission CAN...");
    
    // Test avec données fictives pour BMS 1 (données générales)
    BmsData24V testDataBms;
    testDataBms.totalVoltage = 25.5;
    testDataBms.gatherVoltage = 25.2;
    testDataBms.current = 12.3;
    testDataBms.soc = 85.7;
    testDataBms.maxCellVoltage = 3450;
    testDataBms.maxCellNumber = 5;
    testDataBms.minCellVoltage = 3380;
    testDataBms.minCellNumber = 2;
    testDataBms.maxTemp = 35; // Température max du BMS (non individuelle)
    testDataBms.minTemp = 28; // Température min du BMS (non individuelle)
    testDataBms.errorStatus = 0x0000000000000001;
    
    sendCanFrame4801(CAN_ID_BMS1_BASE, &testDataBms);
    delay(10);
    sendCanFrame4802(CAN_ID_BMS1_BASE + 1, &testDataBms);
    delay(10);
    sendCanFrame4803(CAN_ID_BMS1_BASE + 2, &testDataBms);
    delay(10);
    sendCanFrame4804(CAN_ID_BMS1_BASE + 3, &testDataBms);
    delay(10);

    // Test avec données fictives pour les 16 températures individuelles du BMS 1
    TemperatureData testDataTemp;
    testDataTemp.thermistorNumber = 16;
    testDataTemp.dataValid = true;
    for (int i = 0; i < 16; i++) {
        testDataTemp.temperatures[i] = (uint16_t)((20.0 + (i * 0.5)) * 100); // 20.0, 20.5, 21.0, ... 27.5
    }
    // Simuler le calcul min/max (normalement fait par calculateMinMaxTemperatures)
    testDataTemp.overallMinTemp = (uint16_t)(20.0 * 100); 
    testDataTemp.overallMaxTemp = (uint16_t)(27.5 * 100); 

    // Sauvegarde temporaire des données réelles pour ne pas les écraser
    TemperatureData temp_temperatureData_1 = temperatureData_1;
    temperatureData_1 = testDataTemp; // Utilise les données de test

    sendMinMaxTemperatureToCAN(1); // Envoie les données de température MIN/MAX de test
    delay(10);

    temperatureData_1 = temp_temperatureData_1; // Restaure les données réelles

    Serial.println("Fin du test transmission CAN.");
}