#define Z14A  false   // fonctionnement sur RX/TX
#define CCS811  true  // fonctionnement sur SDA/SCL
#define NEXTPM  false   // fonctionnement sur RX/TX

#if CCS811
  #include "Adafruit_CCS811.h"  // librairie pour capteur CJMCU 811 : eCO2 / TCOV
#endif
#if NEXTPM
  #include <PMS.h>              // librairie PMS pour NextPM
#endif
#if Z14A
  byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};  // donnees pour le Z14A
  char reponse[9];
#endif

#include <ArduinoLowPower.h>
#include <SigFox.h>

const boolean test = false;     // test uniquement des capteurs sans utilisation de Sigfox
const boolean oneshot = false;      // Set oneshot to false to trigger continuous mode when you finisched setting up the whole flow
const int nbMes  = 5;
const int periode  = 60*5;     // en seconde : 60*4 -> 4 mn

#define UINT16_t_MAX  65536
#define STATUS_OK   0
#define STATUS_1_KO 1
#define STATUS_2_KO 2
#define STATUS_3_KO 4
#define STATUS_4_KO 8
#define STATUS_5_KO 16

struct Mesure {
  float valeurMin;
  float valeurMax;
  float valeurBrute;
};

typedef struct __attribute__ ((packed)) sigfox_message {
  uint8_t status;
  uint16_t mesure0;
  uint16_t mesure1;
  uint16_t mesure2;
  uint16_t mesure3;
  uint16_t mesure4;
  uint8_t lastMessageStatus;
} SigfoxMessage;

SigfoxMessage msg;
Mesure mes[nbMes];
#if NEXTPM
  PMS pms(Serial1);
#endif
#if CCS811
  Adafruit_CCS811 ccs;
#endif

void CreerMesure(){
  /* définir les valeurs min et max en fonction des grandeurs mesurées
     seuils CO2 : 0 - 10000 ppm
     seuils COV : 0 - 10000 ppb
     seuils PM : 0 - 200 µg/m3
     seuils O3 : 0 - 500 µg/m3
     seuils NO2 : 0 - 1000 µg/m3
   */
  mes[0].valeurMin = 0;
  mes[0].valeurMax = 10000;
  mes[0].valeurBrute = 0;
  
  mes[1].valeurMin = 0;
  mes[1].valeurMax = 10000;
  mes[1].valeurBrute = 0;
  
  mes[2].valeurMin = 0;
  mes[2].valeurMax = 10000;
  mes[2].valeurBrute = 0;
  
  mes[3].valeurMin = 0;
  mes[3].valeurMax = 10000;
  mes[3].valeurBrute = 0;
  
  mes[4].valeurMin = 0;
  mes[4].valeurMax = 10000;
  mes[4].valeurBrute = 0;
}

void reboot() {
  NVIC_SystemReset();
  while (1);
}
 
void setup() {
  CreerMesure();

  // initialisation liaison serie
  if (oneshot || test) {
    Serial.begin(115200);
    while (!Serial) {}
  }
  // initialisation Sigfox
  if (!SigFox.begin()) {
    reboot();
  }
  SigFox.end();
  if (oneshot) {
    SigFox.debug();
  }
  // initialisation eCO2/COV
  #if CCS811
    if(!ccs.begin()){
      Serial.println("Failed to start sensor! Please check your wiring.");
      while(1);
    }
    while(!ccs.available());       //calibrate temperature sensor
    float temp = ccs.calculateTemperature();
    ccs.setTempOffset(temp - 25.0);
    Serial.println("calibration effectuée");
  #endif
  // initialisation NextPM  
  #if NEXTPM
    Serial1.begin(9600);    // UART hardware
  #endif
  // initialisation Z14A  
  #if Z14A 
    Serial1.begin(9600);    // UART hardware
  #endif
}

void loop() {       // Every "periode" minutes, read all the sensors and send them

  // lecture des mesures

  // capteur CO2 Z14A  : Ping CO2, lire, répondre et de traduire en PPM
  #if Z14A
    Serial1.write(cmd,9);
    Serial1.readBytes(reponse, 9);
    int poidsFort = (int) reponse[2];
    int poidsFaible = (int) reponse[3];
    mes[0].valeurBrute = 256*poidsFort+poidsFaible;    // valeur en ppm
    delay(2000);
  #endif
  // capteur Next-PM
  #if NEXTPM
    if (pms.getSTATUS() & 0b00000001) {  
        Serial.print("Réveil du capteur : "); Serial.println(pms.SleepToggle());
     }
     while ((pms.getSTATUS() & 0b00000100) == 4); // attendre que le capteur soit prêt.
     mes[0].valeurBrute = pms.getPM1_0Av10s();    // valeur en µg/m3
     mes[1].valeurBrute = pms.getTEMPERATURE();   // valeur en °C
     mes[2].valeurBrute = pms.getHUMIDITY();      // valeur en %
  #endif
  // capteur eCO2/COV
  #if CCS811
    while(!ccs.available());       //calibrate temperature sensor
    float temp = ccs.calculateTemperature();
    ccs.setTempOffset(temp - 25.0);
    if(!ccs.readData()){
      Serial.print("CO2: "); Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: "); Serial.print(ccs.getTVOC());
      Serial.print("ppb   Temp:"); Serial.println(temp);
      mes[0].valeurBrute = ccs.geteCO2();    // valeur en ppm compris entre 400 et 9000
      mes[1].valeurBrute = ccs.getTVOC();    // valeur en ppb compris entre 0 et 1200
    }
    else {
      Serial.println("ERROR!");
      while(1);
    }
  #endif
  // Start the module, Wait at least 30ms after first configuration (100ms before)
  if (!test) {
    SigFox.begin();
    delay(100);
  }
  if (oneshot && !test) {
    Serial.println("SigFox FW version " + SigFox.SigVersion());
    Serial.println("ID  = " + SigFox.ID());
    Serial.println("PAC = " + SigFox.PAC());
  }
  msg.status = STATUS_OK;
  msg.mesure0 = (mes[0].valeurBrute - mes[0].valeurMin) * UINT16_t_MAX / (mes[0].valeurMax - mes[0].valeurMin);
  msg.mesure1 = (mes[1].valeurBrute - mes[1].valeurMin) * UINT16_t_MAX / (mes[1].valeurMax - mes[1].valeurMin);
  msg.mesure2 = (mes[2].valeurBrute - mes[2].valeurMin) * UINT16_t_MAX / (mes[2].valeurMax - mes[2].valeurMin);
  msg.mesure3 = (mes[3].valeurBrute - mes[3].valeurMin) * UINT16_t_MAX / (mes[3].valeurMax - mes[3].valeurMin);
  msg.mesure4 = (mes[4].valeurBrute - mes[4].valeurMin) * UINT16_t_MAX / (mes[4].valeurMax - mes[4].valeurMin);

  if (oneshot || test) {
    Serial.print("mes[0].valeurbrute : "); Serial.println(mes[0].valeurBrute);
    Serial.print("msg: "); Serial.println(String(msg.mesure0) + " " + String(msg.mesure1) + " " + String(msg.mesure2) + " " + String(msg.mesure3) + " " + String(msg.mesure4));
  }
  if (!test) {
    SigFox.status();
    delay(1);
    SigFox.beginPacket();
    SigFox.write((uint8_t*)&msg, 12);
    msg.lastMessageStatus = SigFox.endPacket();
    SigFox.end();
  }
  if (oneshot) {
    Serial.print("message: "); Serial.print(msg.mesure1); Serial.println("LastStatus (0 = OK): " + String(msg.lastMessageStatus));
  }
  // spin forever, so we can test that the backend is behaving correctly
  if (oneshot && !test) {
    while (1) {}
  }
  //Sleep pendant la période considérée
  if (!test) {
    LowPower.sleep(periode * 1000);
  }
}

