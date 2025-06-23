#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

// --- STRUCTURES DE DONNÉES POUR STOCKER LES DONNÉES REÇUES ---
// Les mêmes structures que dans ton code d'envoi pour la cohérence.
struct BmsData24V {
    float totalVoltage = -1;
    float gatherVoltage = -1;
    float current = -1;
    float soc = -1;
    uint16_t maxCellVoltage = 0;
    uint8_t maxCellNumber = 0;
    uint16_t minCellVoltage = 0;
    uint8_t minCellNumber = 0;
    int8_t maxTemp = -100;      // en °C (vient du JSON BMS principal)
    int8_t minTemp = -100;      // en °C (vient du JSON BMS principal)
    uint64_t errorStatus = 0;
};

// Nouvelle structure pour stocker les températures min/max calculées des thermistances individuelles
struct DecodedTemperatureData {
    uint16_t minTempCalculated = 0xFFFF; // Température min calculée à partir des thermistances (x100)
    uint16_t maxTempCalculated = 0x0000; // Température max calculée à partir des thermistances (x100)
    bool dataReceived = false; // Indique si cette trame a été reçue
};

// --- INSTANCES POUR STOCKER LES DONNÉES DES BMS REÇUES ---
BmsData24V receivedBmsData1;      // Données décodées pour BMS 1
BmsData24V receivedBmsData2;      // Données décodées pour BMS 2
DecodedTemperatureData receivedTempData1; // Températures individuelles décodées pour BMS 1
DecodedTemperatureData receivedTempData2; // Températures individuelles décodées pour BMS 2

// --- CONFIGURATION CAN TWAI ---
#define CAN_TX_PIN 5 // Peuvent être les mêmes que pour l'envoi si tu as un transceiveur
#define CAN_RX_PIN 4 // Peuvent être les mêmes que pour l'envoi si tu as un transceiveur
#define CAN_BITRATE 250000 // Doit correspondre au débit de l'émetteur

// Identifiants CAN pour BMS 1 (ID: 0x001-0x004) et BMS 2 (ID: 0x005-0x008)
#define CAN_ID_BMS1_BASE 0x001
#define CAN_ID_BMS2_BASE 0x005

// Identifiants CAN pour les températures MIN/MAX des thermistances individuelles
#define CAN_ID_TEMP1_MINMAX 0x009 // ID pour min/max températures BMS 1
#define CAN_ID_TEMP2_MINMAX 0x010 // ID pour min/max températures BMS 2

// --- PROTOTYPES DE FONCTIONS ---
void initializeCAN();
void readCanFrames();
void processReceivedCanFrame(CanFrame& frame);
void decodeCanFrame4801(CanFrame& frame, BmsData24V* bmsData, uint32_t canId);
void decodeCanFrame4802(CanFrame& frame, BmsData24V* bmsData, uint32_t canId);
void decodeCanFrame4803(CanFrame& frame, BmsData24V* bmsData, uint32_t canId);
void decodeCanFrame4804(CanFrame& frame, BmsData24V* bmsData, uint32_t canId);
// Nouveau prototype pour le décodage des trames de température min/max
void decodeCanFrameMinMaxTemp(CanFrame& frame, DecodedTemperatureData* tempData, uint32_t canId);

void displayAllReceivedBmsData();
// Renommée pour clarté : affiche les données générales du BMS et les températures calculées
void displayCombinedBmsData(BmsData24V& bmsData, DecodedTemperatureData& tempData, int bmsNumber);


void setup() {
    Serial.begin(115200);
    while (!Serial); // Attendre que le moniteur série soit prêt

    initializeCAN();

    Serial.println("=== Récepteur ESP32 CAN Démarré ===");
    Serial.println("CAN: TX=GPIO5, RX=GPIO4, 250kbps");
    Serial.println("En attente de trames CAN...");
}

void loop() {
    // Lire et traiter toutes les trames CAN disponibles
    readCanFrames();

    // Optionnel : Afficher les données reçues périodiquement
    static unsigned long lastDisplayTime = 0;
    if (millis() - lastDisplayTime >= 2000) { // Toutes les 2 secondes
        displayAllReceivedBmsData();
        lastDisplayTime = millis();
    }

    delay(10); // Petite pause pour ne pas saturer le CPU
}

// --- FONCTIONS CAN TWAI ---

void initializeCAN() {
    ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
    ESP32Can.setSpeed(ESP32Can.convertSpeed(CAN_BITRATE / 1000)); // Convert speed in kbps

    if (ESP32Can.begin()) {
        Serial.printf("CAN Bus initialisé avec succès (%d kbps)\n", CAN_BITRATE / 1000);
    } else {
        Serial.println("Erreur lors de l'initialisation du CAN Bus");
        // Tu pourrais ajouter un redémarrage ou une gestion d'erreur plus robuste ici
        while (true) {
            delay(100);
        }
    }
}

void readCanFrames() {
    CanFrame rxFrame;
    while (ESP32Can.readFrame(rxFrame)) {
        processReceivedCanFrame(rxFrame);
    }
}

void processReceivedCanFrame(CanFrame& frame) {
    Serial.printf("Trame reçue - ID: 0x%03X, DLC: %d, Data: ", frame.identifier, frame.data_length_code);
    for (int i = 0; i < frame.data_length_code; i++) {
        Serial.printf("%02X ", frame.data[i]);
    }
    Serial.println();

    // Déterminer de quel BMS et quelle trame il s'agit
    if (frame.identifier >= CAN_ID_BMS1_BASE && frame.identifier <= (CAN_ID_BMS1_BASE + 3)) {
        // Trame pour BMS 1 (données principales)
        switch (frame.identifier) {
            case CAN_ID_BMS1_BASE:
                decodeCanFrame4801(frame, &receivedBmsData1, frame.identifier);
                break;
            case (CAN_ID_BMS1_BASE + 1):
                decodeCanFrame4802(frame, &receivedBmsData1, frame.identifier);
                break;
            case (CAN_ID_BMS1_BASE + 2):
                decodeCanFrame4803(frame, &receivedBmsData1, frame.identifier);
                break;
            case (CAN_ID_BMS1_BASE + 3):
                decodeCanFrame4804(frame, &receivedBmsData1, frame.identifier);
                break;
        }
    } else if (frame.identifier >= CAN_ID_BMS2_BASE && frame.identifier <= (CAN_ID_BMS2_BASE + 3)) {
        // Trame pour BMS 2 (données principales)
        switch (frame.identifier) {
            case CAN_ID_BMS2_BASE:
                decodeCanFrame4801(frame, &receivedBmsData2, frame.identifier);
                break;
            case (CAN_ID_BMS2_BASE + 1):
                decodeCanFrame4802(frame, &receivedBmsData2, frame.identifier);
                break;
            case (CAN_ID_BMS2_BASE + 2):
                decodeCanFrame4803(frame, &receivedBmsData2, frame.identifier);
                break;
            case (CAN_ID_BMS2_BASE + 3):
                decodeCanFrame4804(frame, &receivedBmsData2, frame.identifier);
                break;
        }
    } else if (frame.identifier == CAN_ID_TEMP1_MINMAX) {
        // Trame pour les températures MIN/MAX des thermistances BMS 1
        decodeCanFrameMinMaxTemp(frame, &receivedTempData1, frame.identifier);
    } else if (frame.identifier == CAN_ID_TEMP2_MINMAX) {
        // Trame pour les températures MIN/MAX des thermistances BMS 2
        decodeCanFrameMinMaxTemp(frame, &receivedTempData2, frame.identifier);
    }
    else {
        Serial.printf("Trame CAN avec ID non reconnu: 0x%03X\n", frame.identifier);
    }
}

// --- FONCTIONS DE DÉCODAGE SPÉCIFIQUES AUX TRAMES ---
// Ces fonctions inversent l'encodage fait dans ton code d'envoi.

void decodeCanFrame4801(CanFrame& frame, BmsData24V* bmsData, uint32_t canId) {
    if (frame.data_length_code < 8) return;

    uint16_t totalVoltageRaw = (frame.data[0] << 8) | frame.data[1];
    uint16_t gatherVoltageRaw = (frame.data[2] << 8) | frame.data[3];
    uint16_t currentRaw = (frame.data[4] << 8) | frame.data[5];
    uint16_t socRaw = (frame.data[6] << 8) | frame.data[7];

    bmsData->totalVoltage = (float)totalVoltageRaw / 10.0;
    bmsData->gatherVoltage = (float)gatherVoltageRaw / 10.0;
    bmsData->current = ((float)currentRaw - 30000.0) / 10.0; // Inverser le décalage de +30000
    bmsData->soc = (socRaw == 0xFFFF) ? -1 : (float)socRaw / 10.0; // Vérifier la valeur invalide

    Serial.printf("    Decoded 0x%03X: V_total=%.1fV, V_gather=%.1fV, I=%.1fA, SOC=%.1f%%\n",
                  canId, bmsData->totalVoltage, bmsData->gatherVoltage, bmsData->current, bmsData->soc);
}

void decodeCanFrame4802(CanFrame& frame, BmsData24V* bmsData, uint32_t canId) {
    if (frame.data_length_code < 6) return; // Seulement 6 octets utiles

    bmsData->maxCellVoltage = (frame.data[0] << 8) | frame.data[1];
    bmsData->maxCellNumber = frame.data[2];
    bmsData->minCellVoltage = (frame.data[3] << 8) | frame.data[4];
    bmsData->minCellNumber = frame.data[5];

    Serial.printf("    Decoded 0x%03X: VMax=%dmV(#%d), VMin=%dmV(#%d)\n",
                  canId, bmsData->maxCellVoltage, bmsData->maxCellNumber,
                  bmsData->minCellVoltage, bmsData->minCellNumber);
}

void decodeCanFrame4803(CanFrame& frame, BmsData24V* bmsData, uint32_t canId) {
    if (frame.data_length_code < 3) return; // Seulement 3 octets utiles (data[0], data[2])

    bmsData->maxTemp = (int8_t)(frame.data[0] - 40); // Inverser le décalage de +40
    bmsData->minTemp = (int8_t)(frame.data[2] - 40); // Inverser le décalage de +40

    Serial.printf("    Decoded 0x%03X: TMax=%d°C, TMin=%d°C\n",
                  canId, bmsData->maxTemp, bmsData->minTemp);
}

void decodeCanFrame4804(CanFrame& frame, BmsData24V* bmsData, uint32_t canId) {
    if (frame.data_length_code < 8) return;

    bmsData->errorStatus = 0;
    for (int i = 0; i < 8; i++) {
        bmsData->errorStatus |= ((uint64_t)frame.data[i] << (8 * i)); // Reconstruire le uint64_t
    }

    Serial.printf("    Decoded 0x%03X: Errors=0x%llX\n", canId, bmsData->errorStatus);
}

// Nouvelle fonction de décodage pour les trames de température min/max (0x009, 0x010)
void decodeCanFrameMinMaxTemp(CanFrame& frame, DecodedTemperatureData* tempData, uint32_t canId) {
    if (frame.data_length_code < 4) return;

    // Les températures sont encodées en uint16_t (x100), deux octets par valeur
    tempData->minTempCalculated = (frame.data[0] << 8) | frame.data[1];
    tempData->maxTempCalculated = (frame.data[2] << 8) | frame.data[3];
    tempData->dataReceived = true; // Marquer que les données ont été reçues

    Serial.printf("    Decoded 0x%03X: Calculated Min Temp=%.2f°C, Max Temp=%.2f°C\n",
                  canId, tempData->minTempCalculated / 1000.0, tempData->maxTempCalculated / 1000.0);
}

// --- FONCTIONS D'AFFICHAGE DES DONNÉES REÇUES ---

void displayAllReceivedBmsData() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║            ÉTAT DES BMS REÇUS            ║");
    Serial.println("╠════════════════════════════════════════╣");

    // Affichage BMS 1
    Serial.println("║                  BMS 1                   ║");
    Serial.println("╠════════════════════════════════════════╣");
    displayCombinedBmsData(receivedBmsData1, receivedTempData1, 1);

    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║                  BMS 2                   ║");
    Serial.println("╠════════════════════════════════════════╣");
    displayCombinedBmsData(receivedBmsData2, receivedTempData2, 2);

    Serial.println("╚════════════════════════════════════════╝\n");
}

void displayCombinedBmsData(BmsData24V& bmsData, DecodedTemperatureData& tempData, int bmsNumber) {
    // Vérifier si les données générales du BMS ont été initialisées
    if (bmsData.totalVoltage == -1 && bmsData.current == -1 && bmsData.soc == -1) {
        Serial.println("║ Aucune donnée CAN (générale) reçue pour ce BMS. ║");
    } else {
        Serial.print("║ Tension: ");
        Serial.print(bmsData.totalVoltage, 2);
        Serial.print("V | Courant: ");
        Serial.print(bmsData.current, 2);
        Serial.println("A         ║");

        Serial.print("║ SOC: ");
        Serial.print(bmsData.soc, 1);
        Serial.print("% | Temp BMS: ");
        Serial.print(bmsData.minTemp);
        Serial.print("°C à ");
        Serial.print(bmsData.maxTemp);
        Serial.println("°C      ║");

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

    // Afficher les données de température min/max des thermistances individuelles si reçues
    if (tempData.dataReceived) {
        Serial.printf("║ Temp. Thermo: Min=%.2f°C | Max=%.2f°C  ║\n",
                      tempData.minTempCalculated / 100.0, tempData.maxTempCalculated / 100.0);
    } else {
        Serial.println("║ Pas de données temp. thermistances reçues.      ║");
    }
}
