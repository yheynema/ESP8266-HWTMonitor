/*  --- Entête principale -- information sur le sketche
 *   
 *  Programme: Hot Water Tank (HWT) Monitoring
 *  Date: juillet 2021
 *  Auteur: Y. Heynemand
 *  Plateforme visée: ESP8266 - ESP12-F  (4MB -> FS:1MB + OTA:~1019MB)
 *     Choisir NodeMCU 1.0 ESP-12E (4MB -> FS:2MB + OTA:~1019MB
 *  Description: Ce sketch est un moniteur de consommation électrique (courant seulement) et 
 *               de temperature (entrées/sortie d'eau et réservoir).
 *  Fonctionnalités: REST API service
 *  Notes: Ce sketche est appellé à l'aide de http et du [nom de device].local (mDNS) sans arguments
 *         le sketche retourne un fichier index.html (et le client demandera le .css et le .js).
 *         Le client aura ensuite tout ce qu'il faut pour demander le data et afficher dans le template.
 *  Utilise un tiny85 pour lire le courant, voir code: readCurrrent-sendSerial_tiny85_juin2021.ino
 *  
 *  -- Inspirations et credits: --
 *  Json parametric GET REST response with ArduinoJSON library
 *  by Mischianti Renzo <https://www.mischianti.org>
 *
 *  https://www.mischianti.org/
 *
 */

//--- Version et historique de developpement ----------------------------
#define  _VERSION "2.1.8"
/*
 * 
 * v0.1.x: version initiale de test 
 *    - REST API basé sur les instructions mischianti.org (CORS, REST, traitement, etc). Bien structuré.
 *    - formattage de données en json
 *    - lire le data du tiny85 sur le softw serial
 * v1.0.x: version qui offre le data sur web et MQTT, config des probes de temperature, fonctionnel de base
 * v1.1.x: ajout de OTA, version, config dans EEPROM, 
 * v1.2.x: recuperer le data des T sensor, envoie à emonCMS; currstatus donne etat activité et le temps depuis; refactoring: listSPIFFS, push data dans les TempC arrays;
 * v1.3.x: récupérer le data du courant (duree et timestamp)
 * v1.4.x: lire/récupérer, modifier la config, bugfix: minActDuration; bugfix: lecture de l'ordre des T sensor ms.TsensorIndex[x]; Ajout: reply msg lors d'un POST;
 *         "streamlined" listSPIFFContent, ajout dans currentStatus;
 * v1.5.x: + de robustesse dans la config setSettings; fractionnement pour envoyer data à MQTT; remodelage et simplif de loop(); ajout d'un led status (controlable), à pulsation 
 * v1.6.x: timestamp de activity est relatif pour réduire la longueur de la réponse; support de limitation du nombre d'element demandés (cas de activity et donnees de temperature)
 *         ajout du param 'last' permettant de spécifier les X derniers elements de la liste.
 *         ajout du param 'code' permettant de spécifier les aggregats Raw, 4mn, et/ou 1hr selon le code bit 0=Raw, 1=4mn, 2=1hr
 *         corrigé désignation des aggrégats 4mn->Raw, 1hr->4mn, 24hr->1hr pour mieux réfléter ce qu'elles sont.
 *         ajouté le status de connection a MQTT
 *         modifié la section de code pour emoncms
 * v1.7.x: 26 oct 2021 Yh: confirmation de résultat avec emoncms et modification de MQTT pour ajouter un ID au topic: pour supporter plusieurs HWT dans une meme localité (basé sur chipID)
 * v1.7.3: 12 janv 2022 Yh: ajout de "mqttBaseTopic" dans getCfg, ajouté "MQTTLastMsgStatus" dans currentStatus     
 * v1.7.8: 27 mai 2022 Yh: ligne 1463 - tentative de corriger bug de transfert sr le seriel entre tiny85 et ESP8266 (ce code), PAS TESTÉ!  ATTENTION: le tiny85 n'envoie pas le char de terminaison '\n'
 * 
 * v2.0.x: 28 mai 2022 Yh : modifier pour supporter Thingsboard au lieu de MQTT (simple) et eMonCMS
 * 
 * v2.1.x: 21 juillet 2023 Yh: PCB matériel, ajustement pour ESP-07 plutot qu'un ESP-12 (mauvais choix, btw)
 *         déf pour WLD (Water Leak Detector) détecteur d'eau (cas de fuite) et traitement, avis sonore, surveillance overheat
 *         déf pour LED RED+YEL et BUZZER
 *         retiré support emonCMS
 *         24 juillet 2023: bug majeur: crash lors de l'init Wifi et/ou en mode AP pour config Wifi... 
 *
 *
 *
 *
 *
 * v3.x: multi-senseurs de courant (au moins un 2e), utilisation d'un ADC externe (AD7991) si trouve le prblm du ESP-12F et le I2C
 * v4.x: ajout d'un module de commande "energy saver" avec relais et cédule, controle électronique de maintient de la T
 * v5.x: support pour plusieurs chauffe-eau, surveillance de groupe?
 * v6.x: écran OLED indiquant l'état (opt)
 * 
 * À faire (+ voir le plan des versions):
 *    ajouter element dans la reponse JSON: deviceID et locationID (configurables). DeviceID est a la base = chipID ou DevName+3dernMACByte
 *    x OTA
 *    x versionning
 *    Voir pour utiliser Preferences au lieu de EEPROM... ou sinon LittleFS  (voir RandomNerdsTut)
 *    param ICAL ajustable
 *    x save config EEPROM
 *    à tester - push dans emoncms (htttp), configurable (ip, clef, activ)
 *    à tester - push MQTT (ip, topic, activ)
 *    futur: support pour 2 current sensors - fait au niveau di tiny85 v1.1.4
 *    x cumul du temps où le courant est > (x)
 *    gérer une perte de connectivité Wifi
 *    à tester final - démélanger l'idée de stocker dans les array Average avec index... juste considérer le config lors de la présentation des données.
 *    Bug2Fix: considérer un état que "s'il est assez long". Le code ci-bas est bon pour le logging des activités mais pas lors du
 *             rapport 'since' (curstatus).
 *    À considérer: utiliser le tiny85 à 5.0v pour les mesures (meilleure sensibilité/justesse/échelle) et mettre un level shifter entre tiny85 et ESP8266
 *    
 */
//-----------------------------------------------------------------------

//--- Librairies (en ordre alpha) ---------------------------------------
#include "Arduino.h"
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <Average.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
//#include <ESP8266HTTPClient.h>  -- 24/07/2023: semble pas requis...
#include <ESP8266HTTPUpdateServer.h>
#include <EEPROM.h>
#include <FS.h>
#include <NTPClient.h>
#include <OneWire.h>
//#include <PubSubClient.h> -- inclus par TBPubSub et entrait en conflit
#include <SoftwareSerial.h>
//#include <SPIFFS.h> -- voir FS.h
#include <ThingsBoard.h>
#include <TimeLib.h>
#include <time.h>
#include <Timezone.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
//-----------------------------------------------------------------------

//--- Definitions -------------------------------------------------------
#define ONE_WIRE_BUS 4
#define TEMPERATURE_PRECISION 9
// #define LED_BUILTIN  (déjà définie)
#define LED_RED_PIN 13
#define LED_YEL_PIN 12
#define BUZZ_PIN 15
#define WLD01_PIN 5
#define WLD02_PIN 2

/*
 * logMQTTMode est un mot binaire dont les bits signifient:
 *  0 = OFF, pas de logging MQTT
 *  bit 0 = Envoi du data de la valeur du courant
 *  bit 1 = Envoi du data des 3 températures
 *  bit 2 = Envoi du data d'activité (lorsqu'elle est disponible)
 *  bit 3 = Envoi du data d'activité actuelle
 */
 #define dataI   0
 #define dataT   1
 #define dataAct 2
 #define dataCA  3
//-----------------------------------------------------------------------


//--- Declaration des objets, constantes, et variables globales----------
//#define ESP8266  -- 21/07/2023: aparamment redéfinition... 
#define webServicePort 80
#define statusLedTimerDelay 1250L

const uint16_t oneSec=1000;
const uint32_t baseMsecMinute=60*oneSec;

const IPAddress apIP(192, 168, 1, 1);
const char* apSSID = "HWTMonSetup";
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";
#define defaultDevName "HWTMonitorV2"
const char* deviceName = defaultDevName;


// -- File  system pour gérer le OTA
File fsUploadFile;
//boolean spiffSucceded=false;  //Flag to indicate SPIFFs is available
String ssidList;

ESP8266WebServer server(webServicePort);
ESP8266HTTPUpdateServer httpUpdater;


// -- Client MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// -- Client Thingsboard
//WiFiClient espClientTB;
ThingsBoardSized<200> tb(espClient);

// -- Gestion de l'heure via NTP -------------------
const int GTMOffset = 0;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ca.pool.ntp.org", GTMOffset*60*60, 60*60*1000);
// US Eastern Time Zone (New York, Detroit)
TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  // Eastern Daylight Time = UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   // Eastern Standard Time = UTC - 5 hours
Timezone usET(usEDT, usEST);

// -- Gestion des probes de temperature via DS18B20
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses - please refer to the configStruc for mapping of which is which
const byte numTempDev = 3;
byte thermalDev[numTempDev][8];


#define aggrSize 24  // on a 1 donnée aux 10 secondes
// Historique sonde temperature 0
Average<float> t0_Raw(aggrSize); //data au 10 sec pendant 4 mn = 240sec
Average<float> t0_4mn(15); // data au 4mn sur 1hr
Average<float> t0_1hr(72); // data a l'heure, sur 72h
// Historique sonde temperature 1
Average<float> t1_Raw(aggrSize); //data au 10 sec pendant 4 mn = 240sec
Average<float> t1_4mn(15); // data au 4mn sur 1hr
Average<float> t1_1hr(72); // data a l'heure, sur 72h
// Historique sonde temperature 2
Average<float> t2_Raw(aggrSize); //data au 10 sec pendant 4 mn = 240sec
Average<float> t2_4mn(15); //data au 4mn, pendant 1hr
Average<float> t2_1hr(72); //data a l'heure, sur 72h

Average<float> *AvgPtr_Raw[numTempDev] = {&t0_Raw, &t1_Raw, &t2_Raw};
Average<float> *AvgPtr_4mn[numTempDev]  = {&t0_4mn, &t1_4mn, &t2_4mn};
Average<float> *AvgPtr_1hr[numTempDev] = {&t0_1hr, &t1_1hr, &t2_1hr};

// Conserver les données de consommation
Average<uint16_t> durationData(90);
Average<uint32_t> activityData(90);

// Pour la communication avec le Tiny85 - module traitement du courantI
SoftwareSerial swser(14, 16); // RX | TX

uint32_t getTempTimer=0;
// dans config ms: const uint32_t getTempTimerDelay=5000;

const uint16_t schedROMOffset=128;  // voir detail du EEPROM mapping
const uint16_t configBaseLocation=0;


static const unsigned long STRUCT_MAGIC = 1234562014;  //3 derniers digit= _VERSION
static const byte STRUCT_VERSION = 1;

// -- Structure globale contenant l'état du système:
struct Status {

  uint8_t tsync=0;
  float currentIValue = -1;
  uint16_t countBadCurrentIStatus=0;
  bool lastCurrentIStatus = false;
  uint8_t MQTTConnected=false;
  uint8_t MQTTLastMsgSuccess=false;
  int8_t activiteCourante=0; //0=OFF, 1=ON, -1=non-initialisée
  int8_t activitePrecedente=0; //0=OFF, 1=ON, -1=non-initialisée
  uint32_t activityStartTime=0;
  float currentTempSensorValue[numTempDev]= {0,0,0};
  bool spiffSucceded=false;
  uint8_t TBConnected=false;
  uint32_t lastTBConnect=0;
  uint16_t tbConnectFailed=0;

  //Uptime data:
  uint32_t startTime=0;
  uint32_t statusLedTimer=0;

};

struct Status currentStatus;

// -- Structure globale contenant la configuration du système:
struct ConfigDataStruc {
  unsigned long magic;
  byte struct_version;
  uint16_t recordNumber;  //nombre de fois que la structure a ete ecrite dans EEPROM, compte le nb de variable modifiee

//.. placer ici les variables ..//

  uint8_t logMQTTMode; // 0=off, voir ci-haut les modes
  uint8_t emoncmsMode;
  uint8_t tbMode;
  uint32_t getTempTimerDelay;
  uint32_t minActDuration;
  uint8_t loopCount; // numb retries value for wifi connection 

  uint8_t TsensorIndex[3];

  float currentIThreshold; // valeur a partir de laquelle on compte que le chauffe-eau consomme de l'énergie. Q: que fait-on si 1 ou 2 élements?

  uint8_t emoncmsIp[4];
  int emoncmsTcp;
  char emoncmsKey[32+1];

  uint8_t tbHostIp[4];
  char tbToken[32+1];
  uint32_t tbRetryDelay;

  uint8_t mqttIp[4];
  int mqttPort;
  char mqttUser[15+1];
  char mqttPassword[31+1];

  uint8_t wifiConnectDelay;

  char devName[15 + 1];  //Max 15 char + EOS
  char ssid[31 + 1]; // WIFI ssid + null
  char password[31 + 1]; // WiFi password,  if empyt use OPEN, else use AUTO (WEP/WPA/WPA2) + null
};
ConfigDataStruc ms;
const int ConfigDataStruc_storageSize = sizeof(ConfigDataStruc);

//-----------------------------------------------------------------------


//--- Prototypes --------------------------------------------------------
void getObjData(JsonArray&, Average<uint32_t>*, uint16_t);
void getObjData(JsonArray&, Average<uint16_t>*, uint16_t);
void getObjData(JsonArray&, Average<float>*, uint16_t);
void getTempData(JsonObject&, uint8_t, uint16_t, uint8_t );
void getActivityData(JsonObject&, uint16_t, uint16_t);
void listSPIFFContent(JsonObject&);
void loadFromEEPROM(bool);
void handleFileUpload(void);
bool handleFileRead(String);
String getContentType(String);
void handleNotFound(void);
void restServerRouting(void);
void sendCrossOriginHeader(void);
void setSettings(void);
void getSettings(void);
void setCrossOrigin(void);
void saveToEEPROM(void);
boolean attemptWifiConnection(void);
void setupMode(void);
void startWebServer(void);
String makePage(String, String);
String urlDecode(String);
static tm getDateTimeByParams(long);
static String getDateTimeStringByParams(tm *, char* );
static String getEpochStringByParams(long time, char* );
uint8_t goReadCurrentSensor(void);
uint8_t pushDataToMQTT(void);
//uint8_t pushDataToEMONCMS(void);  -- N'est plus supporté
uint8_t goReadTemperatureSensors(void);
//-----------------------------------------------------------------------


//--- Routines specifiques ----------------------------------------------

/*
 * Nom: round2
 * Fonction: rounds a number to 1 decimal places,example: round(3.14159) -> 3.1
 * Argument(s) réception: double ou float
 * Argument(s) de retour: double
 * Modifie/utilise (globale):
 * Notes:  source: https://arduinojson.org/v6/how-to/configure-the-serialization-of-floats/
 * 
 */
double round2(double value) {
   return (int)(value * 10 + 0.5) / 10.0;
}

/*
 * Nom: getObjData
 * Fonction: formate un tableau de type Average<float> en objet JSon
 * Argument(s) réception: 1 objet JSon et 1 structure de données
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void getObjData(JsonArray &data, Average<float> *dataObj, uint16_t limit=0) {
  if (dataObj->getCount() > 0 ) {
   if (limit == 0) limit=dataObj->getCount();
   if (limit > dataObj->getCount()) limit=dataObj->getCount();
   uint16_t startPoint = dataObj->getCount() - limit;
   for (uint16_t i=startPoint; i<dataObj->getCount(); i++) {
    data.add(round2(dataObj->get(i)));
   }
  } else {
    data.add("empty");
  }
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void getObjData(JsonArray &data, Average<uint32_t> *dataObj, uint16_t limit=0) {
  if (dataObj->getCount() > 0 ) {
   bool mode = true;
   if (limit == 0) limit=dataObj->getCount();
   if (limit > dataObj->getCount()) limit=dataObj->getCount();
   uint16_t startPoint = dataObj->getCount() - limit;
   uint32_t prevItem=currentStatus.startTime;
   uint32_t currItem=0;
   for (uint16_t i=startPoint; i<dataObj->getCount(); i++) {
    currItem=dataObj->get(i);
    if (!mode) prevItem=0;
    data.add(currItem-prevItem);
    prevItem=currItem;
   }
  } else {
    data.add("empty");
  }
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void getObjData(JsonArray &data, Average<uint16_t> *dataObj, uint16_t limit=0) {
  if (dataObj->getCount() > 0 ) {
   if (limit == 0) limit=dataObj->getCount();
   if (limit > dataObj->getCount()) limit=dataObj->getCount();
   uint16_t startPoint = dataObj->getCount() - limit;
   for (uint16_t i=startPoint; i<dataObj->getCount(); i++) {
    data.add(dataObj->get(i));
   }
  } else {
    data.add("empty");
  }
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void getTempData(JsonObject &obj, uint8_t tempSensorIdx, uint16_t history, uint8_t codeVal) {

//Ceci doit se trouver dans l'appellant et me passer l'objet:
//    JsonObject obj = doc1.createNestedObject((String("tC"+String(tempSensorIdx))).c_str());
  JsonArray tempCValues;

  if (codeVal & 0x01) {
    tempCValues = obj.createNestedArray("Raw");
    getObjData(tempCValues, AvgPtr_Raw[tempSensorIdx], history);
  }
  if (codeVal & 0x02) {
    tempCValues = obj.createNestedArray("4mn");
    getObjData(tempCValues, AvgPtr_4mn[tempSensorIdx], history);
  }
  if (codeVal & 0x04) {
    tempCValues = obj.createNestedArray("1hr");
    getObjData(tempCValues, AvgPtr_1hr[tempSensorIdx], history);
  }
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void getActivityData(JsonObject &obj, uint16_t history) {
  obj["reftstmp"] = currentStatus.startTime;
  JsonArray tempActValues = obj.createNestedArray("endtime");
  getObjData(tempActValues, &activityData, history);
  tempActValues = obj.createNestedArray("duration");
  getObjData(tempActValues, &durationData, history);
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void listSPIFFContent(JsonObject &obj) {
  if (currentStatus.spiffSucceded) {
      uint16_t countItem = 0;
      obj["dirContent"]=String("/");
      Dir dir = SPIFFS.openDir("/");
      while (dir.next()) {
        JsonArray spiffsItem = obj.createNestedArray(String("item"+String(countItem)));
        spiffsItem.add(String(dir.fileName()));
        spiffsItem.add(String(dir.fileSize()));
        countItem++;
      }
      if (countItem==0) {
        JsonArray spiffsItem = obj.createNestedArray("item0");
        spiffsItem.add("emptyList");
        spiffsItem.add("0");
      }
  } else {
    obj["dirContent"]=F("uninitialized");
  }
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void saveToEEPROM(void) {
  // Met à jour le nombre magic et le numéro de version avant l'écriture
  ms.magic = STRUCT_MAGIC;
  ms.struct_version =  STRUCT_VERSION;
  ms.recordNumber++;
  EEPROM.put(configBaseLocation, ms);
  EEPROM.commit();
//  printFWDetails();
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void loadFromEEPROM(bool returnToDefault) {

  // Lit la mémoire EEPROM
  EEPROM.get(configBaseLocation, ms);
  
  // Détection d'une mémoire non initialisée
  byte erreur = ms.magic != STRUCT_MAGIC;

  if (erreur || returnToDefault) {

    //--- Valeurs par defaut
    
    ms.TsensorIndex[0] = 1;  //T sensor index 0
    ms.TsensorIndex[1] = 2;  //T sensor index 1
    ms.TsensorIndex[2] = 0;  //T sensor index 2

    ms.currentIThreshold = 1;
    
    ms.logMQTTMode=0;    // 0=off, bits: 0=temp msg , 1=star events. 2=stop events, 3=bucket full event, 4=remote low batt, 5=remote status, etc

    ms.tbMode=0; //Thingsboard

//    ms.minNumberofSamples = aggrSize/2; //nombre minimal d'echantillon requis pour un fonctionnement normal

    ms.wifiConnectDelay=30;

    ms.getTempTimerDelay=10000; // 1 donnee de temperature au 10 secondes
    ms.minActDuration=60; //Durée minimale d'une activité de chauffage (élimine les erreurs intempestives)

    ms.loopCount = 10; //number of retries
    ms.mqttPort = 1883;
    strcpy(ms.mqttUser, "HWTESP");
    strcpy(ms.mqttPassword,"HWTESP");
    strcpy(ms.emoncmsKey,"0f4e3efc41a19a8d003b858a80f12c14");  //initialement la clef de mon setup...

    ms.mqttIp[0] = 192;
    ms.mqttIp[1] = 168;
    ms.mqttIp[2] = 122;
    ms.mqttIp[3] = 253;

    ms.emoncmsMode = 0;
    ms.emoncmsIp[0] = 192;
    ms.emoncmsIp[1] = 168;
    ms.emoncmsIp[2] = 122;
    ms.emoncmsIp[3] = 203;
    ms.emoncmsTcp = 80;

    ms.tbMode=0; //Thingsboard
    ms.tbHostIp[0]= 192;
    ms.tbHostIp[1]= 168;
    ms.tbHostIp[2]= 122;
    ms.tbHostIp[3]= 210;
    strcpy(ms.tbToken,"uUiPWTycMqRDGy5RnU7j");
    ms.tbRetryDelay = 20000; //Delais entre 2 tentatives

    ms.wifiConnectDelay = 60; // 60 seconds
    
    strcpy(ms.devName,deviceName);
    strcpy(ms.ssid,"hwtmon");
    strcpy(ms.password,"hwtmon");
    // Sauvegarde les nouvelles données
    saveToEEPROM();
  }
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
boolean attemptWifiConnection(void) {
  int count = 0;

  uint8_t loopCount = 10;
  if (ms.loopCount>0)
    loopCount = ms.loopCount;
  uint16_t loopDelay = (((uint16_t)(ms.wifiConnectDelay)) / loopCount) * 1000;

  WiFi.hostname(ms.devName);
  WiFi.begin(ms.ssid, ms.password);
  Serial.print("Waiting for Wi-Fi connection");

  while ( count < ms.loopCount ) {
    Serial.print(".");

    if (WiFi.status() == WL_CONNECTED) {
       WiFi.hostname(ms.devName);
       Serial.println();
       Serial.print("Connected to ");
       Serial.println(WiFi.SSID());              // Tell us what network we're connected to
       Serial.print("IP address:\t");
       Serial.println(WiFi.localIP());     
// Fermer le mode WIFI_AP... est-ce que ca peut etre fait apres une connection dans le mode WIFI_STA? TBV
       WiFi.softAPdisconnect(true);
       return (true);
    }
    delay(loopDelay);  
    count++;
    digitalWrite(LED_YEL_PIN,count%2);
  }
  Serial.println("Wifi connect timed out");
  return false;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void setupMode(void) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  int n = WiFi.scanNetworks();
  delay(100);
  Serial.println("");
  for (int i = 0; i < n; ++i) {
    ssidList += "<option value=\"";
    ssidList += WiFi.SSID(i);
    ssidList += "\">";
    ssidList += WiFi.SSID(i);
    ssidList += "</option>";
  }
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  delay(100);
  WiFi.softAP(apSSID);
  delay(100);
  startWebServer();
  Serial.print("Starting Access Point at \"");
  Serial.print(apSSID);
  Serial.println("\"");
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void startWebServer(void) {
  Serial.print("Starting Web Server at ");
  Serial.println(WiFi.softAPIP());
  server.on("/settings", []() {
    String s = "<h1>Wi-Fi Settings</h1><p>Please enter your password by selecting the SSID.</p>";
    s += "<form method=\"get\" action=\"setap\"><label>SSID: </label><select name=\"ssid\">";
    s += ssidList;;
    s += "</select><br>Password: <input name=\"pass\" length=64 type=\"password\"><input type=\"submit\"></form>";
    s += "<h3>NOTE: The MAC for this device is: " + String(WiFi.macAddress()) + "</h3>";
    server.send(200, "text/html", makePage("Wi-Fi Settings", s));
  });
  server.on("/setap", []() {
    String ssid = urlDecode(server.arg("ssid"));
    Serial.print("SSID: ");
    Serial.println(ssid);
    String pass = urlDecode(server.arg("pass"));
    Serial.print("Password: ");
    Serial.println(pass);
    Serial.println("Writing SSID to EEPROM...");

    ssid.toCharArray(ms.ssid,31);
    pass.toCharArray(ms.password,31);

    saveToEEPROM();

    Serial.println("Write EEPROM done!");
    String s = "<h1>Setup complete.</h1><p>device will be connected to \"";
    s += ssid;
    s += "\" after the restart.";
    server.send(200, "text/html", makePage("Wi-Fi Settings", s));
    ESP.restart();
  });
  server.onNotFound([]() {
    String s = "<h1>AP mode</h1><p><a href=\"/settings\">Wi-Fi Settings</a></p>";
    server.send(200, "text/html", makePage("AP mode", s));
  });
  server.begin();
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
String makePage(String title, String contents) {
  String s = "<!DOCTYPE html><html><head>";
  s += "<meta name=\"viewport\" content=\"width=device-width,user-scalable=0\">";
  s += "<title>";
  s += title;
  s += "</title></head><body>";
  s += contents;
  s += "</body></html>";
  return s;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
String urlDecode(String input) {
  String s = input;
  s.replace("%20", " ");
  s.replace("+", " ");
  s.replace("%21", "!");
  s.replace("%22", "\"");
  s.replace("%23", "#");
  s.replace("%24", "$");
  s.replace("%25", "%");
  s.replace("%26", "&");
  s.replace("%27", "\'");
  s.replace("%28", "(");
  s.replace("%29", ")");
  s.replace("%30", "*");
  s.replace("%31", "+");
  s.replace("%2C", ",");
  s.replace("%2E", ".");
  s.replace("%2F", "/");
  s.replace("%2C", ",");
  s.replace("%3A", ":");
  s.replace("%3A", ";");
  s.replace("%3C", "<");
  s.replace("%3D", "=");
  s.replace("%3E", ">");
  s.replace("%3F", "?");
  s.replace("%40", "@");
  s.replace("%5B", "[");
  s.replace("%5C", "\\");
  s.replace("%5D", "]");
  s.replace("%5E", "^");
  s.replace("%5F", "-");
  s.replace("%60", "`");
  return s;
}

/*
 * Nom: 
 * Fonction: Input time in epoch format and return tm time format
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  by Renzo Mischianti <www.mischianti.org> 
 * 
 */
static tm getDateTimeByParams(long time){
    struct tm *newtime;
    const time_t tim = time;
    newtime = localtime(&tim);
    return *newtime;
}
 
/*
 * Nom: 
 * Fonction: Input tm time format and return String with format pattern
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  by Renzo Mischianti <www.mischianti.org>
 * 
 */
static String getDateTimeStringByParams(tm *newtime, char* pattern = (char *)"%d/%m/%Y %H:%M:%S"){
    char buffer[30];
    strftime(buffer, 30, pattern, newtime);
    return buffer;
}


 /*
 * Nom: 
 * Fonction: Input time in epoch format format and return String with format pattern
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  by Renzo Mischianti <www.mischianti.org> 
 * 
 */
static String getEpochStringByParams(long time, char* pattern = (char *)"%d/%m/%Y %H:%M:%S"){
//    struct tm *newtime;
    tm newtime;
    newtime = getDateTimeByParams(time);
    return getDateTimeStringByParams(&newtime, pattern);
}

 /*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void setCrossOrigin(){
    server.sendHeader(F("Access-Control-Allow-Origin"), F("*"));
    server.sendHeader(F("Access-Control-Max-Age"), F("600"));
    server.sendHeader(F("Access-Control-Allow-Methods"), F("PUT,POST,GET,OPTIONS"));
    server.sendHeader(F("Access-Control-Allow-Headers"), F("*"));
};
 
 
// Serving Hello world
/*
 * 
 void getHelloWord() {
      DynamicJsonDocument doc(512);
      doc["name"] = "Hello world";
 
      Serial.print(F("Stream..."));
      String buf;
      serializeJson(doc, buf);
      server.send(200, "application/json", buf);
      Serial.print(F("done."));
}
// Serving Hello world
*/

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void getSettings(void) {
    setCrossOrigin();
//
      // Allocate a temporary JsonDocument
      // Don't forget to change the capacity to match your requirements.
      // Use arduinojson.org/v6/assistant to compute the capacity.
    //  StaticJsonDocument<512> doc;
      // You can use DynamicJsonDocument as well
      DynamicJsonDocument doc(2048);
 
      doc["ltime"] = getEpochStringByParams(usET.toLocal(now()));
      doc["fwver"] = _VERSION;
      doc["uxtm"] = now();

      if (server.arg("getcfg") == "true") {
        doc["struct_version"] = ms.struct_version;
        doc["recordNumber"] = ms.recordNumber;
        doc["logMQTTMode"] = ms.logMQTTMode;
        doc["MQTTconnexion"] = currentStatus.MQTTConnected;
        doc["MQTTLastMsgStatus"] = currentStatus.MQTTLastMsgSuccess;
        doc["tbMode"] = ms.tbMode;
        doc["tbStatus"] = currentStatus.TBConnected;
        doc["tbFailure"] = currentStatus.tbConnectFailed;
        doc["tbConnectDly"] = ms.tbRetryDelay;
        doc["emoncmsMode"] = ms.emoncmsMode;
        doc["getTempTimerDelay"] = ms.getTempTimerDelay;
        doc["loopCount"] = ms.loopCount;
        JsonArray tsensorArr = doc.createNestedArray("TsensorIndex");
        for (uint8_t i=0; i<3 ; i++) {
          tsensorArr.add(ms.TsensorIndex[i]);
        }
        doc["currentIThreshold"] = ms.currentIThreshold;
        IPAddress emonip(ms.emoncmsIp);        
        doc["emonip"] = emonip.toString();        
        doc["emoncmsTcp"] = ms.emoncmsTcp;
        doc["emoncmsKey"] = ms.emoncmsKey;
        
        IPAddress mqttIp(ms.mqttIp);
        doc["mqttIp"] = mqttIp.toString();
        doc["mqttPort"] = ms.mqttPort;
        doc["mqttUser"] = ms.mqttUser;
        doc["mqttPassword"] = ms.mqttPassword;
        
        IPAddress tbHostIp(ms.tbHostIp);
        doc["tbHostIp"] = tbHostIp.toString();
        doc["tbToken"] = ms.tbToken;

        uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
        doc["mqttBaseTopic"] = "esphwt_"+String(num,HEX);
        doc["devName"] = ms.devName;
        doc["ssid"] = ms.ssid;
      }

      if (currentStatus.startTime>0) {
        doc["uptime"] = (now()-currentStatus.startTime);
      }

      if (server.arg("data") == "true") {
         JsonObject obj = doc.createNestedObject("temp");
         obj["cold"] = String(getTemperature(thermalDev[ms.TsensorIndex[0]]),1);
         obj["hot"] = String(getTemperature(thermalDev[ms.TsensorIndex[1]]),1);
         obj["tk"] = String(getTemperature(thermalDev[ms.TsensorIndex[2]]),1);

         doc["current"] = String(currentStatus.currentIValue,2);
      }
       
      if (server.arg("signalStrength")== "true"){
         doc["sStrengh"] = WiFi.RSSI();
      }

 
      if (server.arg("chipInfo")== "true"){
         doc["cpId"] = ESP.getChipId();  //Note Yh au 12janv2022: transcrire en HEX, ie: String(num,HEX);
         doc["flCpId"] = ESP.getFlashChipId();  //Note Yh au 12janv2022: transcrire en HEX, ie: String(num,HEX);
         doc["flCpSz"] = ESP.getFlashChipSize();  // En MB (div 1024) + "MB")
         doc["flCpRSz"] = ESP.getFlashChipRealSize();  // En MB (div 1024) + "MB")
      }
      if (server.arg("freeHeap")== "true"){
         doc["freeHeap"] = ESP.getFreeHeap();  // En MB (div 1024) + "MB")
      }
      
      if (server.arg("ipconf")== "true"){
         doc["ip"] = WiFi.localIP().toString();
         doc["gw"] = WiFi.gatewayIP().toString();
         doc["nm"] = WiFi.subnetMask().toString();
          
         uint8_t macAddr[6];
         WiFi.macAddress(macAddr);
         char macAddrC[20]={};
         sprintf(macAddrC,"%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
         doc["mac"] = macAddrC;
      }

      if (server.arg("curstatus") == "true") {
         doc["curStBdCount"] = currentStatus.countBadCurrentIStatus;
         doc["lstCurSt"] = currentStatus.lastCurrentIStatus;
         if (currentStatus.activiteCourante >=0 ) {
            doc["actcour"] = currentStatus.activiteCourante;
            doc["since"] = now()-currentStatus.activityStartTime;
         }
      }
      if (server.arg("listSPIFF") == "true") {
        JsonObject obj = doc.createNestedObject("SPIFFS");
        listSPIFFContent(obj);
   }

      if (server.arg("tC0") == "true") {
        uint16_t limit=0;
        uint8_t codeVal = 0x07 ; // equivaut à 1 & 1<<1 & 1<<2
        if (server.hasArg("last")) limit = server.arg("last").toInt();
        if (server.hasArg("code")) codeVal = server.arg("code").toInt();
        codeVal &= 0x07;
        if (codeVal > 0) {
          JsonObject obj = doc.createNestedObject("tC0");
          getTempData(obj,ms.TsensorIndex[0],limit,codeVal);
        }
      }
      if (server.arg("tC1") == "true") {
        uint16_t limit=0;
        uint8_t codeVal = 0x07 ; // equivaut à 1 & 1<<1 & 1<<2
        if (server.hasArg("last")) limit = server.arg("last").toInt();
        if (server.hasArg("code")) codeVal = server.arg("code").toInt();
        codeVal &= 0x07;
        if (codeVal > 0) {
          JsonObject obj = doc.createNestedObject("tC1");
          getTempData(obj,ms.TsensorIndex[1],limit,codeVal);
        }
      }
      if (server.arg("tC2") == "true") {
        uint16_t limit=0;
        uint8_t codeVal = 0x07 ; // equivaut à 1 & 1<<1 & 1<<2
        if (server.hasArg("last")) limit = server.arg("last").toInt();
        if (server.hasArg("code")) codeVal = server.arg("code").toInt();
        codeVal &= 0x07;
        if (codeVal > 0) {
          JsonObject obj = doc.createNestedObject("tC2");
          getTempData(obj,ms.TsensorIndex[2],limit,codeVal);
        }
      }

      if (server.arg("activity") == "true") {
        uint16_t limit=0;
        if (server.hasArg("last")) limit = server.arg("last").toInt();
        JsonObject obj = doc.createNestedObject("activity");
        getActivityData(obj,limit);
      }
       
      Serial.print(F("Stream..."));
      String buf;
      serializeJson(doc, buf);
      server.send(200, F("application/json"), buf);
      Serial.println(F("done."));
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void setSettings(void) {
    
  Serial.println(F("postConfigFile"));

  setCrossOrigin();

  String postBody = server.arg("plain");
  Serial.println(postBody);

  uint8_t needToSaveEEPROM = 0;
  DynamicJsonDocument reply(512);

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, postBody);
  if (error) {
     // if the file didn't open, print an error:
     Serial.print(F("Error parsing JSON "));
     Serial.println(error.c_str());
 
     String msg = error.c_str();
 
     server.send(400, F("text/html"), "Error parsing json body! <br>"+msg);
 
  } else {
     JsonObject postObj = doc.as<JsonObject>();

     Serial.print(F("HTTP Method: "));
     Serial.println(server.method());

     bool requiresMQTTInit = false;

     if (server.method() == HTTP_POST) {
        if (postObj.containsKey("rotate")) {
           // Récupérer le data de l'alignement des probes T:
           bool error = false;
           char devices[numTempDev][5] = {"dev0","dev1","dev2"};
           for (uint8_t i=0; i<numTempDev; i++) {
             unsigned int tempNum = postObj["rotate"][devices[i]];
             if (tempNum >= 0 && tempNum<numTempDev) { 
               ms.TsensorIndex[i] = tempNum;
             } else {
               error=true;
             }
           }

           if (!error) {
             //traiter ici le changement
             reply["rotate"]="success";
             needToSaveEEPROM++;
           } else {
              // forger un msg d'erreur avec le traitement
              reply["rotate"]="failed";
           }
         } // Cas de "rotate"
          
         if (postObj.containsKey("logMQTTMode")) {
            if (ms.logMQTTMode != postObj["logMQTTMode"]) {
               if (postObj["logMQTTMode"]>=0 && postObj["logMQTTMode"]<16) {
                  ms.logMQTTMode = postObj["logMQTTMode"];
                  reply["logMQTTMode"]="success";
                  needToSaveEEPROM++;
                } else {
                  reply["logMQTTMode"] = "invalidMode";
                }
            } else {
                reply["logMQTTMode"] = "noChg";
            }
         } // Cas de logMQTTMode;

         if (postObj.containsKey("tbMode")) {
            if (ms.tbMode != postObj["tbMode"]) {
               if (postObj["tbMode"]>=0 && postObj["tbMode"]<16) {
                  ms.tbMode = postObj["tbMode"];
                  reply["tbMode"]="success";
                  needToSaveEEPROM++;
                } else {
                  reply["tbMode"] = "invalidMode";
                }
            } else {
                reply["tbMode"] = "noChg";
            }
         } // Cas de tbMode;
        
         if (postObj.containsKey("emoncmsMode")) {
            if (ms.emoncmsMode != postObj["emoncmsMode"]) {
               ms.emoncmsMode = postObj["emoncmsMode"];
               reply["emoncmsMode"]="success";
               needToSaveEEPROM++;
            } else {
               reply["emoncmsMode"] = "noChg";
            }
         } // Cas de emoncmsMode;

         if (postObj.containsKey("emonIp")) {
            IPAddress emonIp;
            // Test si c'est une adresse IP valide
            if (emonIp.fromString(postObj["emonIp"].as<String>())) {
               bool foundOneChg = false;
               for (uint8_t i=0; i<4; i++) {
                 if (ms.emoncmsIp[i] != emonIp[i]) {
                   ms.emoncmsIp[i]= emonIp[i];
                   foundOneChg = true;
                 }
               }
               if (foundOneChg) {
                  reply["emonIp"] = "success";
                  needToSaveEEPROM++;
               } else {
                  reply["emonIp"] = "noChg";
               }
            } else {
              reply["emonIp"] = "invalidIp";
            }
         } // Cas de emonIp


         if (postObj.containsKey("tbHostIp")) {
            IPAddress tbHostIp;
            // Test si c'est une adresse IP valide
            if (tbHostIp.fromString(postObj["tbHostIp"].as<String>())) {
               bool foundOneChg = false;
               for (uint8_t i=0; i<4; i++) {
                 if (ms.tbHostIp[i] != tbHostIp[i]) {
                   ms.tbHostIp[i]= tbHostIp[i];
                   foundOneChg = true;
                 }
               }
               if (foundOneChg) {
                  reply["tbHostIp"] = "success";
                  needToSaveEEPROM++;
               } else {
                  reply["tbHostIp"] = "noChg";
               }
            } else {
              reply["tbHostIp"] = "invalidIp";
            }
         } // Cas de tbHostIp

         if (postObj.containsKey("mqttIp")) {
            IPAddress mqttIp;
            if (mqttIp.fromString(postObj["mqttIp"].as<String>())) {
               bool foundOneChg = false;
               for (uint8_t i=0; i<4; i++) {
                 if (ms.mqttIp[i] != mqttIp[i]) {
                    ms.mqttIp[i] = mqttIp[i];
                    foundOneChg = true;
                 }
               }
               if (foundOneChg) {
                  reply["mqttIp"] = "success";
                  requiresMQTTInit = true;
                  needToSaveEEPROM++;
               } else {
                  reply["mqttIp"] = "noChg";
               }
            } else {
               reply["mqttIp"] = "invalidIp";
            }
         } // Cas de mqttIp
         
         if (postObj.containsKey("mqttPort")) {
            if (postObj["mqttPort"]>1024 && postObj["mqttPort"]<65000) {
              if (ms.mqttPort != postObj["mqttPort"]) {
                 ms.mqttPort = postObj["mqttPort"];
                 requiresMQTTInit = true;
                 reply["mqttPort"]="success";
                 needToSaveEEPROM++;
              } else {
               reply["mqttPort"] = "noChg";
              }
            } else {
               reply["mqttPort"] = "invalidPort";
            }
         } // Cas de mqttPort
         
// Reste à faire:
//  char mqttUser[15+1];
//  char mqttPassword[31+1];
         if (postObj.containsKey("emoncmsTcp")) {
           if (postObj["emoncmsTcp"]>1024 && postObj["emoncmsTcp"]<65000) {
             if(ms.emoncmsTcp != postObj["emoncmsTcp"]) {
               ms.emoncmsTcp= postObj["emoncmsTcp"];
               reply["emoncmsTcp"]="success";
               needToSaveEEPROM++;
             } else {
               reply["emoncmsTcp"] = "noChg";
             }
           } else {
             reply["emoncmsTcp"] = "invalidPort";
           }
         } // Case de emoncmsTcp
         
         if (postObj.containsKey("emoncmsKey")) {
           String emoncmsKey = postObj["emoncmsKey"];
           if (emoncmsKey.length()>=16 && emoncmsKey.length()<=32) {
             // Ajouter ici un test de similarité (et si oui, code reply["emoncmsKey"]="noChg";)
             strcpy(ms.emoncmsKey, postObj["emoncmsKey"]);
             reply["emoncmsKey"]="success";
             needToSaveEEPROM++;
           } else {
             reply["emoncmsKey"]="invalidSize";
           }
         }  // Cas de emoncmsKey
         
         if (postObj.containsKey("currentIThreshold")) {
           if (postObj["currentIThreshold"]>0 && postObj["currentIThreshold"]<50) {
             if (ms.currentIThreshold != postObj["currentIThreshold"]) {
               ms.currentIThreshold=postObj["currentIThreshold"];
               reply["currentIThreshold"]="success";
               needToSaveEEPROM++;
             } else {
               reply["currentIThreshold"] = "noChg";
             }
           } else {
               reply["currentIThreshold"] = "invalidValue";
           }
         } // Cas de currentIThreshold
         
         if (postObj.containsKey("devName")) {
           if (postObj["devName"].as<String>().length()>0 && postObj["devName"].as<String>().length()<16) {
            // Ajouter ici un test de similarité (et si oui, code reply["devName"]="noChg";)
             strcpy(ms.devName, postObj["devName"]);
             reply["devName"]="success";
             needToSaveEEPROM++;
           } else {
             reply["devName"] = "invalidLength";
           }
         } // Cas de devName

         if (postObj.containsKey("tbToken")) {
           if (postObj["tbToken"].as<String>().length()>0 && postObj["tbToken"].as<String>().length()<32) {
             strcpy(ms.tbToken, postObj["tbToken"]);
             reply["tbToken"]="success";
             needToSaveEEPROM++;
           } else {
             reply["tbToken"] = "invalidLength";
           }
         } // Cas de tbToken
         
         if (postObj.containsKey("ssid")) {
           if (postObj["ssid"].as<String>().length()>0 && postObj["ssid"].as<String>().length()<32) {
            // Ajouter ici un test de similarité (et si oui, code reply["ssid"]="noChg";)
             strcpy(ms.ssid,postObj["ssid"]);
             reply["ssid"]="success";
             needToSaveEEPROM++;
           } else {
             reply["ssid"] = "invalidLength";
           }
         } // Cas de ssid (Name)
         
         if (postObj.containsKey("ssidPass")) {
           if (postObj["ssidPass"].as<String>().length()>0 && postObj["ssidPass"].as<String>().length()<32) {
            // Ajouter ici un test de similarité (et si oui, code reply["ssidPass"]="noChg";)
            strcpy(ms.password,postObj["ssidPass"]);
            reply["ssidPass"]="success";
            needToSaveEEPROM++;
           } else {
             reply["ssidPass"] = "invalidLength";
           }
         } // Cas de ssidPass
         
       } // fin de traitement method POST

       //Changement MQTT, necessite un re-init de la connexion?
       if (requiresMQTTInit)  {
          if (currentStatus.MQTTConnected) {
            //Close current connection
            if (mqttClient.connected()) mqttClient.disconnect();
          }
          if (initMQTTConnection()) {
            reply["reinitMQTT"] = "success";
            currentStatus.MQTTConnected=true;
          } else {
            reply["reinitMQTT"] = "failed";
            currentStatus.MQTTConnected=false;
          }
       } // traitement connection MQTT

       // Y a-t-il qqch à sauvegarder?
       if (needToSaveEEPROM>0) {
         saveToEEPROM();
         reply["nbElement"]=String(needToSaveEEPROM);
         String buf;
         serializeJson(reply, buf);
         server.sendHeader("Content-Length", String(buf.length()));
         server.send(201, F("application/json"),buf);
       } else {
         server.send(204, F("text/html"), F("No data found, or incorrect!"));
       }
   }  //else de erreur de deserialization
}//Fin de setSettings

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
void sendCrossOriginHeader(void){
    Serial.println(F("sendCORSHeader"));
 
    server.sendHeader(F("access-control-allow-credentials"), F("false"));
 
    setCrossOrigin();
 
    server.send(204);
}


/*
 * Nom: 
 * Fonction: Define routing
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
void restServerRouting(void) {
    server.on("/", HTTP_GET, []() {
        server.send(200, F("text/html"),
            F("Welcome to the REST Web Server"));
    });
    //server.on(F("/helloWorld"), HTTP_GET, getHelloWord);
    server.on(F("/settings"), HTTP_OPTIONS, sendCrossOriginHeader);
    server.on(F("/settings"), HTTP_GET, getSettings);
 
    server.on(F("/settings"), HTTP_POST, setSettings);
    server.on("/upload", HTTP_POST,                       // if the client posts to the upload page
    [](){ server.send(200); },                          // Send status 200 (OK) to tell the client we are ready to receive
    handleFileUpload                                    // Receive and save the file
  );
}

/*
 * Nom: 
 * Fonction: Manage not found URL
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
void handleNotFound() {
  if (!handleFileRead(server.uri())) {
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    for (uint8_t i = 0; i < server.args(); i++) {
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    server.send(404, "text/plain", message);
  }
}

/*
 * Nom: 
 * Fonction: convert the file extension to the MIME type
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

/*
 * Nom: 
 * Fonction: send the right file to the client (if it exists)
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
bool handleFileRead(String path) {
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";           // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){  // If the file exists, either as a compressed archive, or normal
     if (SPIFFS.exists(pathWithGz))                          // If there's a compressed version available
        path += ".gz";                                         // Use the compressed version
     File file = SPIFFS.open(path, "r");                    // Open the file
     size_t sent = server.streamFile(file, contentType);    // Send it to the client
     file.close();                                          // Close the file again
     Serial.println(String("\tSent file: ") + path);
     return true;
  }
  Serial.println(String("\tFile Not Found: ") + path);
  return false;                                          // If the file doesn't exist, return false
}

/*
 * Nom: 
 * Fonction: upload a new file to the SPIFFS
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
void handleFileUpload(void) {
  if (currentStatus.spiffSucceded) {
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      String filename = upload.filename;
      if(!filename.startsWith("/")) filename = "/"+filename;
      Serial.print("handleFileUpload Name: "); Serial.println(filename);
      fsUploadFile = SPIFFS.open(filename, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
      filename = String();
    } else if(upload.status == UPLOAD_FILE_WRITE){
      if(fsUploadFile)
        fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
    } else if(upload.status == UPLOAD_FILE_END){
      if(fsUploadFile) {                                    // If the file was successfully created
        fsUploadFile.close();                               // Close the file again
        Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
        server.sendHeader("Location","/index.html");      // Redirect the client to the success page
        server.send(303);
      } else {
        server.send(500, "text/plain", "500: couldn't create file");
      }
    }
  }
}

/*
 * Nom: 
 * Fonction: function to print a device (1-wire) address
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
float getTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println(F("Error: Could not read temperature data"));
  }
  return tempC;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
void MQTTcallback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print(F("Message arrived in topic: "));
  Serial.println(topic);
 
  Serial.print(F("Message:"));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println(F("-----------------------"));
 
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
bool isClean(String tmpStr) {
  bool goodValue = true;
  for (int i=0; i<tmpStr.length(); i++) {
    char c = tmpStr.charAt(i);
    if ( !((c >= '0' && c <= '9') || ( c == '.' || c == '-' || c == '\n' || c == '\r')))
      goodValue = false;
  }
  return goodValue;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 *   // Sugg d'amélio: voir si on ne doit pas attendre le CR/LF avant de récupérer le data...
 *  // ok mais readString possede déjà un timeout de 1000ms. Peut-etre utiliser readStringUntil(1000,"\n") ??
 * 
 */ 
bool getCurrentI(float& curVal) {
  swser.println('a');
  bool success = false;
  if (swser.available() > 0) {
// Yh au 27 mai 2022 - me disant que ce pourait resoudre le bug du data incomplete entre le Tiny et le ESP8266 - PAS TESTÉ
//    ATTENTION: au 27 mai 2022: le tiny85 n'envoie pas le char de terminaison '\n'
//    String tempStrI = swser.readStringUntil(1000,'\n');
// Tentative de lire jusqu'à ce qu'un '\n' soit reçu...
    String tempStrI="";
    const char charMarker = '\n';
    char inStr[12];
    int8_t ptr = -1;
    bool signOff=false;
    long timeMark = millis();
    while (swser.available()>0 && ptr<11 && !signOff) {
      char oneChar = swser.read();
      if (oneChar != charMarker)
          inStr[++ptr] = oneChar;
      else signOff=true;
      inStr[ptr+1]=0;
      delay(1); //let next char coming in (9 bits @9600b/s takes about 1ms)
    }
    tempStrI = String(inStr);

    Serial.printf("str=%s",tempStrI.c_str());
    if (tempStrI.indexOf('.') > 0 && isClean(tempStrI)) {
      float tempI = tempStrI.toFloat();
      // float tempI = swser.parseFloat();
      if (tempI >= 0.0 && tempI < 40.0) {
        success=true;
        curVal = tempI;
        Serial.printf(" val=%.2f\n",tempI);
      } else 
        Serial.println(" - valOutOfRange.");
    } else {
      Serial.println(" - junkVal.");
    }
  }
  return success;
}

/*
 * Nom: checkTBConnexion
 * Fonction: tente une connexion au serveur TB si n'est pas connecté 
 * Argument(s) réception: rien
 * Argument(s) de retour: un booléen = operation réussie ou non
 * Modifie/utilise (globale):
 * Notes: isSubcribe a été mis en commentaire, car ici n'utilise pas RPC (au 28 mai 2022)
 * 
 */
bool checkTBConnexion(void) {
  // Reconnect to ThingsBoard, if needed
  bool retCode=false;
  if (!tb.connected()) {
    if (millis() > currentStatus.lastTBConnect+ms.tbRetryDelay) {
  //    isSubcribed = false;
      //Récupération des valeurs (Preferences):
      //String tbhost = mesPreferences.getString("TBHOST","");
      //String tbtoken = mesPreferences.getString("TBTOKEN","");
      // Connect to the ThingsBoard
      Serial.print("Connecting to: ");
      IPAddress tbHostIp(ms.tbHostIp);  //Forger un objet IPAddress à partir des 4 bytes
      Serial.print(tbHostIp.toString());
      Serial.print(" with token ");
      Serial.println(ms.tbToken);
      if (tb.connect(tbHostIp.toString().c_str(), ms.tbToken)) {
        retCode=true;
        currentStatus.lastTBConnect=millis();
      } else {
        Serial.println("Failed to connect to TB");
        currentStatus.tbConnectFailed++;
        retCode=false;
      }
    }
  } else retCode=true;
  currentStatus.TBConnected=retCode;
  return retCode;
}


/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
bool initMQTTConnection(void){

  mqttClient.setServer(ms.mqttIp, ms.mqttPort);
  mqttClient.setCallback(MQTTcallback);
  
  bool giveUp=false;
  int loopCount=8;
  Serial.print(F("Connecting to MQTT "));
  while (!mqttClient.connected() && !giveUp) {
 
    if (mqttClient.connect("ESPHWTDATA")) { //, mqttUser, mqttPassword )) {
      Serial.println(F(" connected"));
    } else {
      Serial.print(".");
      loopCount--;
      if (loopCount<0) { 
        giveUp=true;
        Serial.print(F("Giving Up: "));
        Serial.println(mqttClient.state());
      } else {
        delay(1000);
      }
    }
  }
  return !giveUp;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
uint8_t goReadTemperatureSensors(void) {

  sensors.requestTemperatures();

  uint8_t tempSensorStatus=0;  //Status qui sera retourné de l'état de lecture des senseurs de T (0=notOk, 1=success), par bit 0=s0, 1=s1, etc
  
  // Pour chaque sonde: recupère la valeur.
  // Si valide, pousse dans le tableau Raw.
  // Si tableau "wrap around", le Raw est complété, fourni la moyenne dans l'aggregat 4mn
  // Idem pour aggregat 4mn vers le 1hr.
  // Si requis, on envoie à MQTT et emoncms.
  for (uint8_t i=0; i<numTempDev; i++) {
    float tempC = getTemperature(thermalDev[i]);
    if(tempC != DEVICE_DISCONNECTED_C) {
      // Si valeur ok, push dans les tableaux de T
      currentStatus.currentTempSensorValue[i]=tempC;
      bitSet(tempSensorStatus,i);
      uint32_t position = AvgPtr_Raw[i]->push(tempC);
      if (position == 0) { 
        position = AvgPtr_4mn[i]->push(AvgPtr_Raw[i]->mean());
        if (position == 0)
          AvgPtr_1hr[i]->push(AvgPtr_4mn[i]->mean());
      }
    } else {
      Serial.printf("Bad value for sensor '%d'\n",i);
    }
  } //Pour chaque T sensor  
  return tempSensorStatus;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
uint8_t goReadCurrentSensor(void) { 
  
    static float tempI;
    tempI=-1;

    // Lecture du sensor de courant via le SwSerial:
    bool currentIStatus = getCurrentI(tempI);

    // Si la lecture est bonne:
    if (currentIStatus) {
       currentStatus.currentIValue = tempI;
       //Serial.printf("I=%.2f\n",currentIValue);

       // Traiter ici le threshold et ainsi logger le événement de consommation
       // a) état de consommation: activiteCourante=ON si currentIValue > threshold sinon OFF
       // b) comparer l'état d'activité précédente à courante; si passage de ON à OFF, on loggue le timestamp (now()) et la durée (now()-startTime)
       // c) comparer l'état d'activité précédente à courante: si passage de OFF à ON, startTime=now()
       // Bug2Fix: considérer un état que "s'il est assez long". Le code ci-bas est bon pour le logging des activités mais pas lors du
       //          rapport 'since' (curstatus).
       if (currentStatus.currentIValue > ms.currentIThreshold)
         currentStatus.activiteCourante = 1;
       else 
         currentStatus.activiteCourante = 0;

       if (currentStatus.activitePrecedente==0 && currentStatus.activiteCourante==1) {
          currentStatus.activityStartTime=now();
       }
       if (currentStatus.activitePrecedente==1 && currentStatus.activiteCourante==0) {
          if (currentStatus.activityStartTime>0) {
             uint32_t duration = now() - currentStatus.activityStartTime;
             if (duration > ms.minActDuration) {
                durationData.push(duration);
                activityData.push(now());
                if (WiFi.status()== WL_CONNECTED) {
                  if (currentStatus.MQTTConnected && bitRead(ms.logMQTTMode,dataAct)) {
                    uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
                    String topic = "esphwt_"+String(num,HEX)+"/lstDuraction";
                    mqttClient.publish(topic.c_str(),(String(duration)).c_str());
                  }
                }
             } else {
               Serial.println("Event not logged (too short)");
             }
             currentStatus.activityStartTime = now();
          }
       }
       currentStatus.activitePrecedente=currentStatus.activiteCourante;
    } else 
      currentStatus.countBadCurrentIStatus++;
    currentStatus.lastCurrentIStatus = currentIStatus;

    return 1;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
uint8_t pushDataToMQTT(void) { 
  if (WiFi.status()== WL_CONNECTED) {
    if (ms.logMQTTMode && currentStatus.MQTTConnected) {
      uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
      String baseTopic = "esphwt_"+String(num,HEX);
      if (bitRead(ms.logMQTTMode,dataI)) {
        mqttClient.publish((baseTopic+"/currentI").c_str(),((String(currentStatus.currentIValue,2)).c_str())); //Topic name
      }
      if (bitRead(ms.logMQTTMode,dataCA)) {
        mqttClient.publish((baseTopic+"/activity").c_str(),((String(currentStatus.activiteCourante)).c_str())); //Topic name
      }
      if (bitRead(ms.logMQTTMode,dataT)) {
        mqttClient.publish((baseTopic+"/tempC1").c_str(),((String(currentStatus.currentTempSensorValue[ms.TsensorIndex[0]],1)).c_str())); //Topic name
        mqttClient.publish((baseTopic+"/tempC2").c_str(),((String(currentStatus.currentTempSensorValue[ms.TsensorIndex[1]],1)).c_str())); //Topic name
        mqttClient.publish((baseTopic+"/tempC3").c_str(),((String(currentStatus.currentTempSensorValue[ms.TsensorIndex[2]],1)).c_str())); //Topic name
      }
    }
  }
  return 1;
}

/*
 * Nom: pushDataToTB
 * Fonction:  envoyer 5 données de télémétrie et 5 données d'attributs vers TB
 * Argument(s) réception: 
 * Argument(s) de retour: bool indiquant le succes de l'opération d'envoie
 * Modifie/utilise (globale):  utilise les données courantes contenues dans currentStatus
 * Notes:  à analyser: retCode
 * 
 */ 
int8_t pushDataToTB(void) {
  int8_t retCode = -3;  //Code de retour indiquant un problème
  if (WiFi.status()== WL_CONNECTED) {
    retCode = -2;
    if (ms.tbMode && currentStatus.TBConnected) {
      retCode -1;
      bool success=false;
      bool attribSuccess=false;
      const int data_items = 5;
      Telemetry data[data_items] = {
        { "currentI",currentStatus.currentIValue },
        { "activity",currentStatus.activiteCourante },
        { "tempC1", currentStatus.currentTempSensorValue[ms.TsensorIndex[0]] },
        { "tempC2",currentStatus.currentTempSensorValue[ms.TsensorIndex[1]] },
        { "tempC3",currentStatus.currentTempSensorValue[ms.TsensorIndex[2]] }
      };

      if (tb.sendTelemetry(data, data_items)) success=true; else success=false;

      retCode = success;

      // Envoie des attributs
      if (success)  {
        uint32_t chipID = ESP.getChipId();
        //ESP32: String  = String(ESP.getChipModel())+"-"+String(ESP.getChipRevision());

        //Utiliser les 2 derniers bytes du MAC pour en faire le devID
        byte mac[WL_MAC_ADDR_LENGTH];
        WiFi.macAddress(mac);
        uint16_t systemID = mac[4];
        systemID = systemID << 8 + mac[5];
        String deviceID = "HWTMON_"+String(systemID,HEX);
        char macAddrC[20]={};
        sprintf(macAddrC,"%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      
        const int attribute_items = 8;
        Attribute attributes[attribute_items] = {
           { "device_type",  "sensor" },
           { "active",       true     },
           { "deviceID", deviceID.c_str() },
           { "firmware", _VERSION },
           { "chipID", String(chipID,HEX).c_str() },
           { "flashChipId", String(ESP.getFlashChipId()).c_str() },
           { "macAddr", String(macAddrC).c_str() },
           { "IPaddr", WiFi.localIP().toString().c_str() }
        };
        if (tb.sendAttributes(attributes, attribute_items)) attribSuccess=true; else attribSuccess=false;

        retCode += attribSuccess;
      }
    }
  }
  return retCode;
}

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
// uint8_t pushDataToEMONCMS(void) {
//   if (ms.emoncmsMode && WiFi.status()== WL_CONNECTED) {
//     WiFiClient localClient;
//     HTTPClient localHttp;

//     //Assemblage du serverName
//     IPAddress emoncmsSvr(ms.emoncmsIp);
//     String url = "http://"+emoncmsSvr.toString()+":"+String(ms.emoncmsTcp)+"/emoncms/input/post";
//     DynamicJsonDocument emonData(128);
//     emonData["tempC1"]=String(currentStatus.currentTempSensorValue[ms.TsensorIndex[0]],1);
//     emonData["tempC2"]=String(currentStatus.currentTempSensorValue[ms.TsensorIndex[1]],1);
//     emonData["tempC3"]=String(currentStatus.currentTempSensorValue[ms.TsensorIndex[2]],1);
//     emonData["currentI"]=String(currentStatus.currentIValue,2);
//     emonData["activity"]=String(currentStatus.activiteCourante);
//     String outbuff;
//     serializeJson(emonData,outbuff);
//     //Suggestion amélioration: ajouter un byte après hwtmon pour en faire un ID unique
//     uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
//     outbuff = "apikey="+String(ms.emoncmsKey)+"&node=hwtmon_"+String(num,HEX)+"&data="+outbuff;
//     localHttp.begin(localClient, url);
//     localHttp.addHeader("Content-Type", "application/x-www-form-urlencoded");
//     localHttp.addHeader("Authorization", "Bearer "+String(ms.emoncmsKey));  //incomplet...

//     int httpResponseCode = localHttp.POST(outbuff);
//     Serial.print("HTTP Response code: ");
//     Serial.println(httpResponseCode);

//     // Free resources after sending...
//     localHttp.end();
//     localClient.stop();
//   }
//   return 1;
// }

//-----------------------------------------------------------------------


//--- Setup et Loop -----------------------------------------------------

void setup(void) {
  pinMode(LED_RED_PIN,OUTPUT);
  digitalWrite(LED_RED_PIN,LOW);
  pinMode(LED_YEL_PIN,OUTPUT);
  digitalWrite(LED_YEL_PIN,LOW);
  pinMode(BUZZ_PIN,OUTPUT);
  digitalWrite(BUZZ_PIN,HIGH);

  pinMode(WLD01_PIN,INPUT);
  pinMode(WLD02_PIN,INPUT);
  
  Serial.begin(115200);
  while(!Serial) yield();
  Serial.println("\n");

  swser.begin(9600);

  EEPROM.begin(512);
  loadFromEEPROM(false);
  
  Serial.print("HWT Monitor - Firmw vers:");
  Serial.println(_VERSION);
  Serial.println ("Setting up Wifi");

  digitalWrite(LED_YEL_PIN,HIGH);

  digitalWrite(BUZZ_PIN,LOW);

// Attente de la connexion au réseau WiFi / Wait for connection
  if (!attemptWifiConnection()) {
    setupMode();

    //Loop until wifi gets configured and ESP reset
    while(1) {
      server.handleClient();
      if (millis() > currentStatus.statusLedTimer) {
        currentStatus.statusLedTimer = millis() + statusLedTimerDelay/2;
        digitalWrite(LED_YEL_PIN,!digitalRead(LED_YEL_PIN));
      }
    }
  }

digitalWrite(LED_YEL_PIN,HIGH);

  timeClient.begin();
  delay ( 1000 );

digitalWrite(LED_YEL_PIN,LOW);

  if (timeClient.update()){
     Serial.print ( F("Adjust local clock") );
     unsigned long epoch = timeClient.getEpochTime();
     // HERE I'M UPDATE LOCAL CLOCK
     setTime(epoch);
     currentStatus.startTime=now();
  }else{
     Serial.print ( F("NTP Update not WORK!!") );
     currentStatus.startTime=0;
  }
 
  // Activate mDNS this is used to be able to connect to the server
  // with local DNS hostmane esp8266.local
  if (MDNS.begin(ms.devName)) {
    MDNS.addService("http", "tcp", webServicePort);
    Serial.println(F("MDNS responder started"));
  }

digitalWrite(LED_YEL_PIN,HIGH);

  if (SPIFFS.begin()) { 
    currentStatus.spiffSucceded=true;
    DynamicJsonDocument spiffContent(512);
    spiffContent["currentStatus"] = "true";
    JsonObject spiff = spiffContent.createNestedObject("SPIFFS");
    listSPIFFContent(spiff);
    String buf;
    serializeJson(spiffContent,buf);
    Serial.println(F("SPIFF init succeeded:"));
    Serial.println(buf);
  } else { 
    currentStatus.spiffSucceded=false;
    Serial.println(F("Failed to initialize SPIFFS"));
  }

digitalWrite(LED_YEL_PIN,LOW);

  if (ms.logMQTTMode>0) {
    if (initMQTTConnection()) currentStatus.MQTTConnected=true;
  }

digitalWrite(LED_YEL_PIN,HIGH);

  if (ms.tbMode>0) {
    currentStatus.TBConnected = checkTBConnexion();
  }
  
digitalWrite(LED_YEL_PIN,LOW);

  // Start up the library
  sensors.begin();
  oneWire.reset_search();

  // locate devices on the bus
  Serial.print(F("Locating temp devices... "));
  Serial.print(F("Found "));
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(F(" devices."));

digitalWrite(LED_YEL_PIN,HIGH);

  //Find device 1wire addresses, print addresses, set resolution
  for (uint8_t i=0; i<numTempDev; i++) {
    if (sensors.getAddress(thermalDev[i], i)) {
      Serial.printf("Therm device %i Address: ",i);
      printAddress(thermalDev[i]);
      Serial.println();
      sensors.setResolution(thermalDev[i], TEMPERATURE_PRECISION);
    } else
      Serial.printf("Unable to find address for Device %i\n",i);
  }

  Serial.print(F("Requesting temperatures..."));
  sensors.requestTemperatures();

digitalWrite(LED_YEL_PIN,LOW);

  // Set server routing
  restServerRouting();
  
  // Set not found response
  server.onNotFound(handleNotFound);
  
  // Start server

  Serial.println("HTTP server started");

digitalWrite(LED_YEL_PIN,HIGH);

  httpUpdater.setup(&server,update_path,update_username,update_password);
  server.begin();

digitalWrite(LED_YEL_PIN,LOW);

  currentStatus.activiteCourante=-1; //0=OFF, 1=ON, -1=non-initialisée
  currentStatus.activitePrecedente=-1;
  currentStatus.activityStartTime=0;
  currentStatus.activityStartTime=now();
}

 
void loop(void) {
  server.handleClient();
  if (ms.logMQTTMode>0) {
    mqttClient.loop();
  }
  if (ms.tbMode>0) {
    if (checkTBConnexion()) {
      tb.loop();
    } else {
      currentStatus.tbConnectFailed++;
    }
  }

  if (millis() > currentStatus.statusLedTimer) {
    currentStatus.statusLedTimer = millis() + statusLedTimerDelay;
    digitalWrite(LED_YEL_PIN,!digitalRead(LED_YEL_PIN));
  }
  
  //request temp every (getTempTimerDelay) msec
  if (getTempTimer < millis()) {
    getTempTimer = millis() + ms.getTempTimerDelay;
    uint8_t tempReadStatus = goReadTemperatureSensors();

    //Signer notre passage à MQTT dans une routine de traitement:
    if (ms.logMQTTMode && currentStatus.MQTTConnected) {
       if (WiFi.status()== WL_CONNECTED) {
          uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
          String baseTopic = "esphwt_"+String(num,HEX);
          currentStatus.MQTTLastMsgSuccess = mqttClient.publish((baseTopic+"/tmstmp").c_str(),((String(now())).c_str())); //Topic name
       } else {
         currentStatus.MQTTLastMsgSuccess = false;
       }
    } else {
      currentStatus.MQTTLastMsgSuccess = false;
    }
    
    // Traitement de la lecture du courant:
    uint8_t currentReadStatus = goReadCurrentSensor();

    // Traitements après les lectures
    
    // Envoyer à MQTT le data requis:
    uint8_t resultMQTT = pushDataToMQTT();
    
   // Traiter ici l'envoi à emonCMS si ms.emoncmsMode true;
   //21/07/2023 Yh: non, plus requis: uint8_t resultEMON = pushDataToEMONCMS();

   // Traiter ici l'envoi à Thingsboard:
   int8_t resultTB = pushDataToTB();
   
  } // Toutes les (getTempTimerDelay) milisecondes
  MDNS.update();
} //Fin de loop()
//-----------------------------------------------------------------------
