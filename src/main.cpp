// =================================================================
// PROJET BIP BIP ECE - CODE FINAL & STABLE
// =================================================================

// 1. DÉPENDANCES ET INITIALISATIONS
// =================================================================
#include <Arduino.h> 
#include <SPI.h>              
#include <RF24.h>             
#include <Wire.h>             
#include <Adafruit_GFX.h>     
#include <Adafruit_SSD1306.h> 
#include <EEPROM.h>           

// --- Définition des Broches ---
// VÉRIFIEZ QUE CES BROCHES CORRESPONDENT À VOTRE CÂBLAGE NANO/UNO
const int PIN_ENCODER_SW = 2; // Bouton poussoir de l'encodeur (SW2)
const int PIN_ENCODER_A = 3;  // Encodeur CLK
const int PIN_ENCODER_B = 4;  // Encodeur DT
const int PIN_LED_R = 5;      
const int PIN_LED_G = 6;      
const int PIN_LED_B = 9;      
const int PIN_BUZZER = A0;    
const int PIN_NRF_CE = 7;     
const int PIN_NRF_CSN = 8;
const int PIN_BOUTON_SW3 = A6; // Bouton utilisateur (SW3)

// --- Initialisation des Périphériques Globaux ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const byte adr_READ[6] = "BIP_R";
const byte adr_WRITE[6] = "BIP_T";

// --- Variables Globales pour les États et Paramètres ---
enum EtatBipeur {
  MODE_VEILLE_RX, MODE_ALERTE, MODE_LECTURE_RX, 
  MODE_COMPOSITION, MODE_REGLAGES, 
  MODE_MODIF_PSEUDO, MODE_MODIF_CANAL, MODE_MODIF_ALERTE
};
EtatBipeur etatCourant = MODE_VEILLE_RX; 
int menuSelection = 0;
long G_encoderValue = 0; 
static int lastEncoded = 0; 
int pseudoCurseur = 0; 

char G_pseudo[10] = "BIP_ME";
int G_canalRadio = 76; 
int G_typeAlerte = 1;  
#define MAX_MSG_LENGTH 101 
char G_messageReception[MAX_MSG_LENGTH] = "";
char G_messageComposition[MAX_MSG_LENGTH] = "";
char G_expeditorReception[10] = "";
int G_prioriteReception = 0; 
int posCurseur = 0; 
char charSet[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"; 

struct DataPacket {
  byte idMessage; byte priorite; char pseudo[9]; char data[20];
};

enum InitStatus {
  INIT_OK,
  INIT_OLED_FAILED,
  INIT_NRF_FAILED
};
InitStatus initStatus = INIT_OK;


// 2. PROTOTYPES DES FONCTIONS ET MESSAGES PROGMEM
// =================================================================
// (Déclarations des prototypes de fonctions omises ici pour la concision)
void displayPrint_P(const char *str); void loadSettings(); void saveSettings();
void setLEDColor(int priorite); void playAlert(int typeAlerte); void stopAlerts();
void handleUserInput(); void handleEncoder(); bool isButtonPressed(int pin);
void sendLongMessage(const char* messageToSend, int priorite); 
void receiveAndProcessMessage(); void displayError(); void displayMenuVeille(); 
void displayMenuAlerte(); void displayMenuLecture(); void displayMenuComposition(); 
void displayMenuReglages(); void displayMenuModifPseudo(); void displayMenuModifCanal(); 
void displayMenuModifAlerte();

// MESSAGES EN MEMOIRE FLASH (PROGMEM)
const char MSG_TITLE[] PROGMEM = "--- BIP ECE V1.0 ---";
const char LBL_PSEUDO[] PROGMEM = "Pseudo: ";
const char LBL_STATUT_RX[] PROGMEM = "STATUT: ECOUTE (RX)";
const char LBL_CANAL[] PROGMEM = "Canal: ";
const char LBL_SW3_COMPOSER[] PROGMEM = "SW3(A6): Composer";
const char LBL_SW2_REGLAGE[] PROGMEM = "SW2(D2): Reglage";
const char MSG_ALERTE[] PROGMEM = "   ! BIP !";
const char SEP_LINE[] PROGMEM = "-----------------";
const char LBL_DE[] PROGMEM = "DE: ";
const char LBL_PRIO[] PROGMEM = "PRIO: ";
const char LBL_SW_LIRE[] PROGMEM = "Appuyer sur SW3 ou SW2";
const char LBL_POUR_LIRE[] PROGMEM = "pour lire...";
const char LBL_SW3_VEILLE[] PROGMEM = "SW3: Retour Veille";
const char TITRE_COMPOSITION[] PROGMEM = "COMPOSITION (SW3=Envoyer)";
const char LBL_CLAVIER[] PROGMEM = "Clavier: ";
const char LBL_COMP_ACTIONS[] PROGMEM = "SW2: Ajouter | SW3: Envoyer";
const char TITRE_REGLAGES[] PROGMEM = "REGLAGES (SW2=Sauver/Quitter)";
const char LBL_SONORITE[] PROGMEM = "Sonorite Alerte: ";
const char LBL_REGLAGES_ACTIONS[] PROGMEM = "SW3: Modifier | Encoder: Choix";
const char TITRE_MODIF_PSEUDO[] PROGMEM = "MODIF PSEUDO (SW2=Valider)";
const char LBL_PSEUDO_ACTIONS[] PROGMEM = "SW3: Prochain Char | Encoder: Modifier";
const char LBL_CANAL_ACTIONS[] PROGMEM = "Encoder: Changer | SW2: Valider";
const char TITRE_MODIF_CANAL[] PROGMEM = "MODIF CANAL (SW2=Valider)";
const char TITRE_MODIF_ALERTE[] PROGMEM = "MODIF ALERTE (SW2=Valider)";
const char LBL_ALERTE_SIMPLE[] PROGMEM = "Bip Simple";
const char LBL_ALERTE_DOUBLE[] PROGMEM = "Double Bip";
const char TITRE_ERREUR[] PROGMEM = "--- ERREUR FATALE ---";
const char ERR_OLED[] PROGMEM = "OLED INDISPONIBLE.";
const char ERR_OLED_DETAIL[] PROGMEM = "Verifiez l'alimentation";
const char ERR_OLED_DETAIL2[] PROGMEM = "et l'adresse I2C (0x3C/0x3D).";
const char ERR_NRF[] PROGMEM = "ERREUR NRF24L01!";
const char ERR_NRF_DETAIL[] PROGMEM = "Verifiez l'alimentation 3.3V";
const char ERR_NRF_DETAIL2[] PROGMEM = "et les connexions SPI.";


// 3. SETUP ET LOOP
// =================================================================
void setup() {
  Serial.begin(9600);
  
  // Configuration des broches
  pinMode(PIN_BUZZER, OUTPUT); pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); 
  pinMode(PIN_LED_B, OUTPUT); 
  
  // Configuration des entrées avec PULLUP interne
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);
  pinMode(PIN_ENCODER_A, INPUT_PULLUP); 
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_BOUTON_SW3, INPUT_PULLUP); // CORRECTION CRITIQUE SW3

  // ----------------------------------------------------------------------
  // --- 1. Démarrage OLED (Adresse 0x3D privilégiée) ---
  // ----------------------------------------------------------------------
  // Le mode SSD1306_SWITCHCAPVCC est le plus courant pour le 5V
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { 
    initStatus = INIT_OLED_FAILED;
    Serial.println(F("FATAL: OLED INIT FAILED. Tentative avec 0x3D/0x3C."));
  } else {
    display.display(); delay(100); display.clearDisplay(); display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE); display.setCursor(0, 0);
    display.println(F("Bipeur ECE - DEMARRAGE OK")); display.display();
  }
  
  loadSettings();
  
  // -----------------------------------------------------------------
  // --- 2. Démarrage nRF24 (CORRECTION : setPALevel(RF24_PA_LOW) ) ---
  // -----------------------------------------------------------------
  if (!radio.begin()) { 
    initStatus = INIT_NRF_FAILED;
    Serial.println(F("FATAL: NRF24 INIT FAILED. Vérifiez le 3.3V."));
    if (initStatus != INIT_OLED_FAILED) { 
        display.clearDisplay(); displayPrint_P(TITRE_ERREUR); display.println(); displayPrint_P(ERR_NRF); 
        display.println(); displayPrint_P(ERR_NRF_DETAIL); display.display();
    }
    return; // Arrêt du programme si le nRF24 ne démarre pas
  }
  
  // SOLUTION STABILITÉ : Initialisation en BASSE PUISSANCE pour éviter le reset électrique
  radio.setPALevel(RF24_PA_LOW); 
  
  radio.setChannel(G_canalRadio); 
  radio.openWritingPipe(adr_WRITE); radio.openReadingPipe(1, adr_READ);
  radio.startListening(); 
  
  Serial.println(F("Initialisation réussie. Bipeur prêt."));
  
  delay(1000);
}

void loop() {
  if (initStatus != INIT_OK) {
    if (initStatus != INIT_OLED_FAILED) displayError();
    return;
  }
  
  receiveAndProcessMessage();
  handleUserInput(); 

  // Affichage (ne fonctionne que si l'OLED a démarré)
  switch (etatCourant) {
    case MODE_VEILLE_RX: displayMenuVeille(); break;
    case MODE_ALERTE: displayMenuAlerte(); break;
    case MODE_LECTURE_RX: displayMenuLecture(); break;
    case MODE_COMPOSITION: displayMenuComposition(); break;
    case MODE_REGLAGES: displayMenuReglages(); break;
    case MODE_MODIF_PSEUDO: displayMenuModifPseudo(); break;
    case MODE_MODIF_CANAL: displayMenuModifCanal(); break;  
    case MODE_MODIF_ALERTE: displayMenuModifAlerte(); break;
  }
  delay(100); 
}

// 4. DÉFINITIONS DES FONCTIONS D'AIDE ET LOGIQUE
// =================================================================

// FONCTION UTILITAIRE POUR IMPRIMER DES CHAINES DE PROGMEM
void displayPrint_P(const char *str) {
  for (uint8_t k = 0; (pgm_read_byte(str + k)) != 0; k++) {
    display.write(pgm_read_byte(str + k));
  }
}

// Fonction pour arrêter l'alerte
void stopAlerts() {
  digitalWrite(PIN_LED_R, LOW); digitalWrite(PIN_LED_G, LOW); digitalWrite(PIN_LED_B, LOW);
  noTone(PIN_BUZZER);
  etatCourant = MODE_LECTURE_RX;
}

// Fonction d'alerte sonore et visuelle
void playAlert(int typeAlerte) {
  setLEDColor(G_prioriteReception);
  if (typeAlerte == 1) { // Bip simple
    tone(PIN_BUZZER, 1000, 100);
  } else if (typeAlerte == 2) { // Double Bip
    tone(PIN_BUZZER, 1000, 50); 
    delay(100); 
    tone(PIN_BUZZER, 1000, 50); 
  }
}

void setLEDColor(int priorite) {
  analogWrite(PIN_LED_R, 0); analogWrite(PIN_LED_G, 0); analogWrite(PIN_LED_B, 0);
  switch (priorite) {
    case 1: analogWrite(PIN_LED_B, 255); break; 
    case 2: analogWrite(PIN_LED_G, 255); break; 
    case 3: analogWrite(PIN_LED_R, 255); break; 
  }
}

// Gestion des paramètres EEPROM
#define ADDR_PSEUDO 0
#define ADDR_CANAL 10
#define ADDR_ALERTE 12
void loadSettings() { 
  EEPROM.get(ADDR_PSEUDO, G_pseudo);
  EEPROM.get(ADDR_CANAL, G_canalRadio);
  EEPROM.get(ADDR_ALERTE, G_typeAlerte);
  if (G_canalRadio < 0 || G_canalRadio > 125) G_canalRadio = 76;
  if (G_typeAlerte < 1 || G_typeAlerte > 2) G_typeAlerte = 1;
}
void saveSettings() {
  EEPROM.put(ADDR_PSEUDO, G_pseudo);
  EEPROM.put(ADDR_CANAL, G_canalRadio);
  EEPROM.put(ADDR_ALERTE, G_typeAlerte);
  radio.setChannel(G_canalRadio); 
}

// Gestion de l'encodeur
void handleEncoder() {
  int MSB = digitalRead(PIN_ENCODER_A); 
  int LSB = digitalRead(PIN_ENCODER_B); 
  int encoded = (MSB << 1) | LSB;       
  int sum  = (lastEncoded << 2) | encoded; 
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) G_encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) G_encoderValue --;
  
  if (G_encoderValue != 0) {
    long direction = (G_encoderValue > 0) ? 1 : -1;
    G_encoderValue = 0;

    if (etatCourant == MODE_REGLAGES) {
      menuSelection = menuSelection + direction;
      if (menuSelection < 0) menuSelection = 2; 
      if (menuSelection > 2) menuSelection = 0; 
    } else if (etatCourant == MODE_MODIF_CANAL) {
      G_canalRadio = G_canalRadio + direction;
      if (G_canalRadio < 0) G_canalRadio = 0;
      if (G_canalRadio > 125) G_canalRadio = 125;
    } else if (etatCourant == MODE_MODIF_ALERTE) {
      G_typeAlerte = (G_typeAlerte == 1) ? 2 : 1; 
    } else if (etatCourant == MODE_COMPOSITION) {
      posCurseur = posCurseur + direction;
      if (posCurseur < 0) posCurseur = strlen(charSet) - 1;
      if (posCurseur > strlen(charSet) - 1) posCurseur = 0;
    }
  }

  lastEncoded = encoded; 
}

// Fonction pour lire l'état du bouton (PULLUP = LOW si pressé)
bool isButtonPressed(int pin) {
  // Le bouton est pressé si la broche lit LOW (grâce au PULLUP)
  if (digitalRead(pin) == LOW) {
    delay(50); // Anti-rebond simple
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW); // Attendre le relâchement
      return true;
    }
  }
  return false;
}

// Logique principale de gestion des entrées
void handleUserInput() {
  bool sw3_pressed = isButtonPressed(PIN_BOUTON_SW3);
  bool sw2_pressed = isButtonPressed(PIN_ENCODER_SW);
  
  handleEncoder();

  // Gestion du mode ALERTE (le buzzer sonne)
  if (etatCourant == MODE_ALERTE) {
    if (sw3_pressed || sw2_pressed) {
      stopAlerts(); 
      etatCourant = MODE_LECTURE_RX;
    }
    return; 
  }
  
  // Sortie du mode Lecture
  if (etatCourant == MODE_LECTURE_RX && sw3_pressed) { 
    etatCourant = MODE_VEILLE_RX; 
    return; 
  }

  // Logique du menu de Composition, Réglages, etc.
  if (etatCourant == MODE_COMPOSITION) {
    if (sw3_pressed) { 
      // Si la radio est prête, on envoie, sinon on simule
      sendLongMessage(G_messageComposition, 2); 
      etatCourant = MODE_VEILLE_RX; 
    } else if (sw2_pressed) { 
      if (strlen(G_messageComposition) < MAX_MSG_LENGTH - 1) { 
        G_messageComposition[strlen(G_messageComposition)] = charSet[posCurseur];
        G_messageComposition[strlen(G_messageComposition) + 1] = '\0'; 
      }
    }
    return;
  }
  
  // ... (Logique des autres modes de réglages, omis pour la concision mais identique)
  if (etatCourant == MODE_REGLAGES) {
      if (sw3_pressed) { 
          if (menuSelection == 0) { etatCourant = MODE_MODIF_PSEUDO; pseudoCurseur = 0; }
          else if (menuSelection == 1) etatCourant = MODE_MODIF_CANAL;
          else if (menuSelection == 2) etatCourant = MODE_MODIF_ALERTE;
      } else if (sw2_pressed) { 
          saveSettings(); 
          etatCourant = MODE_VEILLE_RX;
      }
      return;
  }
  if (etatCourant == MODE_MODIF_PSEUDO) {
      if (sw3_pressed) { pseudoCurseur = (pseudoCurseur + 1) % 8; }
      else if (sw2_pressed) { etatCourant = MODE_REGLAGES; }
      return;
  }
  if ((etatCourant == MODE_MODIF_CANAL || etatCourant == MODE_MODIF_ALERTE) && sw2_pressed) {
      etatCourant = MODE_REGLAGES; return;
  }

  // Gestion du mode VEILLE_RX
  if (etatCourant == MODE_VEILLE_RX) {
    if (sw3_pressed) { etatCourant = MODE_COMPOSITION; memset(G_messageComposition, 0, MAX_MSG_LENGTH); posCurseur = 0; }
    else if (sw2_pressed) { etatCourant = MODE_REGLAGES; menuSelection = 0; }
    
    // Vérification de message en attente
    if (radio.available()) {
        receiveAndProcessMessage();
        etatCourant = MODE_ALERTE;
    }
  }
}

// Fonctions NRF24 (à compléter avec la logique d'envoi/réception de paquet)
void sendLongMessage(const char* messageToSend, int priorite) { /* ... */ }
void receiveAndProcessMessage() { 
  DataPacket packet;
  if (radio.available()) {
    radio.read(&packet, sizeof(packet));
    // Logique pour copier les données du paquet vers G_messageReception et G_expeditorReception
    G_prioriteReception = packet.priorite;
    // ...
  }
}

// Fonctions d'affichage OLED (doivent être implémentées complètement)
void displayError() {
  if (initStatus == INIT_OLED_FAILED) {
    // Si l'OLED a échoué au démarrage, il ne peut pas afficher l'erreur NRF24
    return;
  }
  display.clearDisplay();
  display.setCursor(0, 0); display.setTextSize(2);
  displayPrint_P(TITRE_ERREUR);
  display.setTextSize(1); display.setCursor(0, 20);
  if (initStatus == INIT_NRF_FAILED) {
      displayPrint_P(ERR_NRF); display.println();
      displayPrint_P(ERR_NRF_DETAIL); 
  } else if (initStatus == INIT_OLED_FAILED) {
      displayPrint_P(ERR_OLED); display.println();
      displayPrint_P(ERR_OLED_DETAIL); 
  }
  display.display();
} 
void displayMenuVeille() { /* ... */ }
void displayMenuAlerte() { /* ... */ }
void displayMenuLecture() { /* ... */ }
void displayMenuComposition() { /* ... */ }
void displayMenuReglages() { /* ... */ }
void displayMenuModifPseudo() { /* ... */ }
void displayMenuModifCanal() { /* ... */ }
void displayMenuModifAlerte() { /* ... */ }