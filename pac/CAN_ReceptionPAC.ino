#include <ESP32-TWAI-CAN.hpp>

// Pins CAN (à adapter selon votre configuration)
#define CAN_TX 5
#define CAN_RX 4

CanFrame rxFrame;

void setup() {
    // Setup serial pour le débogage
    Serial.begin(115200);
    while (!Serial); // Attendre que le port série soit prêt

    // Configurer les pins CAN
    ESP32Can.setPins(CAN_TX, CAN_RX);
    
    // Taille des queues (par défaut)
    ESP32Can.setRxQueueSize(5);
    ESP32Can.setTxQueueSize(5);

    // Vitesse du bus CAN (250 kbit/s selon documentation)
    ESP32Can.setSpeed(ESP32Can.convertSpeed(250));

    // Démarrer le bus CAN
    if(ESP32Can.begin()) {
        Serial.println("CAN bus démarré avec succès!");
    } else {
        Serial.println("Échec du démarrage du bus CAN!");
        while(1); // Bloquer si le CAN ne démarre pas
    }
}

void loop() {
    // Lire les trames CAN avec un timeout de 1000ms
    if(ESP32Can.readFrame(rxFrame, 1000)) {
        // Afficher l'ID de la trame reçue
        Serial.print("Nouvelle Trame : ");
        Serial.printf("Trame reçue - ID: 0x%03X, DLC: %d\n", rxFrame.identifier, rxFrame.data_length_code);

        // Traiter les différentes trames selon la documentation
        switch(rxFrame.identifier) {
            case 0x2FC0000: // État du système et informations générales
                processSystemStateFrame(rxFrame);
                break;
                
            case 0x2FC0001: // Courant et tension de la pile
                processPowerDataFrame(rxFrame);
                break;
                
            case 0x2FC0002: // Erreurs système
                processErrorFrame(rxFrame);
                break;
                
            case 0x2FC0003: // Temps opération et énergie produite
                processRuntimeDataFrame(rxFrame);
                break;
                
            case 0x2FC0004: // Temps opération total et énergie totale
                processTotalRuntimeDataFrame(rxFrame);
                break;
            
            case 0x091: 
                pressionInterneH2(rxFrame);
                break; 

            case 0x090: 
                tensionAuxiliaire(rxFrame);
                break; 

            default:
                // Afficher les données brutes pour les autres trames
                Serial.print("Données: ");
                for(int i = 0; i < rxFrame.data_length_code; i++) {
                    Serial.printf("%02X ", rxFrame.data[i]);
                }
                Serial.println();
        }
    }
}

void tensionAuxiliaire(CanFrame &frame){
  uint8_t voltageBrute = (frame.data[4] << 8) | frame.data[5];
  float voltage_V = voltageBrute * 0.0079f;
  Serial.println("\n=== Tension auxiliaire ===");
  Serial.printf("Tension auxiliaire: %.2f V\n", voltage_V);
}
void pressionInterneH2(CanFrame &frame){
    uint8_t pressionBrute = frame.data[4]; 
    float pressionInterneBar = (pressionBrute *0.01)  ; // dans la doc ils disent de mettre (pressionBrute *0.01)- 0.2 mais j'obtiens -0,2 
    Serial.println("\n=== Pression H2 interne ===");          //quand on a pas d'H2, donc j'ai pas mis -0,2 mais peut être a modifier 
    Serial.printf("Pression H2 interne: %.2f bar\n", pressionInterneBar );
}
void processSystemStateFrame(CanFrame &frame) {
    uint8_t systemState = frame.data[0];
    uint8_t flags = frame.data[1];
    uint8_t h2Consumption = frame.data[2];
    uint8_t avgTemp = frame.data[3];
    
    Serial.println("\n=== État du système ===");
    
    // Afficher l'état du système
    Serial.print("État: ");
    switch(systemState) {
        case 10: Serial.println("Initialisation"); break;
        case 20: Serial.println("Auto-check"); break;
        case 30: Serial.println("Standby"); break;
        case 40: Serial.println("Maintenance"); break;
        case 50: Serial.println("Démarrage - purge"); break;
        case 51: Serial.println("Démarrage - vérification tension"); break;
        case 52: Serial.println("Démarrage - warm up"); break;
        case 100: Serial.println("En opération"); break;
        case 200: Serial.println("Arrêt - power down"); break;
        case 201: Serial.println("Arrêt - purge"); break;
        case 202: Serial.println("Arrêt - H2 stop"); break;
        case 203: Serial.println("Arrêt - purge stop"); break;
        case 204: Serial.println("Arrêt - sauvegarde eeprom"); break;
        case 250: Serial.println("Arrêt urgent (erreur)"); break;
        default: Serial.printf("Inconnu (%d)\n", systemState);
    }
    
    // Afficher les drapeaux
    Serial.print("Drapeaux: ");
    if(flags & 0x01) Serial.print("[Erreur critique] ");
    if(flags & 0x02) Serial.print("[Erreur mineure] ");   // 0x02 => 2^1 => bit 1 
    if(flags & 0x04) Serial.print("[Alerte H2] ");        //0x04 => 2^2 => correspond au bit 2 
    Serial.println();
    
    // Afficher la consommation d'hydrogène et température
    Serial.printf("Consommation H2: %d mg/sec\n", h2Consumption);
    Serial.printf("Température moyenne pile: %d°C\n", avgTemp);
}

void processPowerDataFrame(CanFrame &frame) {
   /*Serial.print("Trame reçue : ");
    for (int i = 0; i < 8; i++) {
        Serial.printf("%02X ", frame.data[i]);
    }*/
    Serial.println();
    // Courant pile (octets 2-3), valeur × 0.01
    uint16_t current_raw = (frame.data[0] << 8) | frame.data[3];    // prendre les octets MSB et LSB de la doc le reste on s en fout pour cette 
    float current_A = current_raw * 0.01f;                            //partie de la doc 

    // Tension pile (octets 5-7), valeur × 0.01                       // = 0 tant qu on lui dit pas de s allumer 
    uint16_t voltage_raw = (frame.data[4] << 8) | frame.data[7];
    float voltage_V = voltage_raw * 0.01f;

    Serial.println("\n=== Données électriques ===");
    Serial.printf("Courant pile: %.2f A\n", current_A);
    Serial.printf("Tension pile: %.2f V\n", voltage_V);
    Serial.printf("Puissance: %.2f W\n", current_A * voltage_V);

}

void processErrorFrame(CanFrame &frame) {
    Serial.println("\n=== Erreurs système ===");
    
    // Octet 0
    if(frame.data[0] & 0x01) Serial.println("Erreur interne");
    if(frame.data[0] & 0x02) Serial.println("Problème tension pile");
    if(frame.data[0] & 0x04) Serial.println("Courant pile trop élevé (>70A ou >65A pendant >10s)");
    if(frame.data[0] & 0x08) Serial.println("Courant pile trop bas (<5A pendant >10s)");
    if(frame.data[0] & 0x10) Serial.println("Température pile hors limites");
    if(frame.data[0] & 0x20) Serial.println("Fuite hydrogène détectée");
    if(frame.data[0] & 0x40) Serial.println("Pression H2 hors limites");
    if(frame.data[0] & 0x80) Serial.println("Tension auxiliaire hors limites");
    
    // Octet 1
    if(frame.data[1] & 0x01) Serial.println("Erreur ventilateur (bloqué ou endommagé)");
    
    if(frame.data[0] == 0 && frame.data[1] == 0) {
        Serial.println("Aucune erreur détectée");
    }
}

void processRuntimeDataFrame(CanFrame &frame) {
    // Temps écoulé en opération (octets 0-1)
    uint16_t runtime = (frame.data[0] << 8) | frame.data[1];
    
    // Énergie produite (octets 2-3)
    uint16_t energy = (frame.data[2] << 8) | frame.data[3];
    
    Serial.println("\n=== Runtime ===");
    Serial.printf("Temps opération: %d minutes\n", runtime);
    Serial.printf("Énergie produite: %d Wh\n", energy);
}

void processTotalRuntimeDataFrame(CanFrame &frame) {
    // Temps écoulé total (octets 0-3)
    uint32_t totalRuntime = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
    
    // Énergie produite totale (octets 4-7)
    uint32_t totalEnergy = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
    
    Serial.println("\n=== Runtime total ===");
    Serial.printf("Temps opération total: %d minutes\n", totalRuntime);
    Serial.printf("Énergie produite totale: %d Wh\n", totalEnergy);
}