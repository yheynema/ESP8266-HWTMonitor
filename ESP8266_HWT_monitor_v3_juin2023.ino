/*  --- Entête principale -- information sur le sketche
 *   
 *  Programme: Hot Water Tank (HWT) Monitoring
 *  Date: juillet 2021 - repris de juin 2022, en juillet 2023
 *  Auteur: Y. Heynemand
 *  Plateforme visée: ESP8266 - ESP12-F  (4MB -> FS:1MB + OTA:~1019MB)
 *     Choisir NodeMCU 1.0 ESP-12E (4MB -> FS:2MB + OTA:~1019MB
 *  Description: Ce sketch est un moniteur de consommation électrique (courant seulement) et 
 *               de temperature (entrées/sortie d'eau et réservoir).
 *  Fonctionnalités: REST API service
 *  Notes: Ce sketche est appellé à l'aide de http et du [nom de device].local (mDNS) sans arguments
 *         le sketche retourne un fichier index.html (et le client demandera le .css et le .js).
 *         Le client aura ensuite tout ce qu'il faut pour demander le data et afficher dans le template.
 *  Utilise un tiny85 pour lire le courant, voir code: readCurrrent-sendSerial_tiny85_juin2023.ino version 1.2.4
 *  
 *  -- Inspirations et credits: --
 *  Json parametric GET REST response with ArduinoJSON library
 *  by Mischianti Renzo <https://www.mischianti.org>
 *
 *  https://www.mischianti.org/
 *

*  Au  28 juillet 2023: est entrain de devenir LA version. Support pour 2 probes de courant
*  Au 28 juillet 2023: reste à valider/intégrer les 2 WLD, le buzzer, les DELs
*  Au 29 juillet 2023: officiellement la version du HWTMonitor, avec le PCB 1.4
*/

#define _VERSION "3.2.2"

/*
 * 
 * v0.x.x: 21 au 28 juillet 2023 - Recommencé le code puisque le précédent crashait...
 * v2.1.x: 21 juillet 2023 Yh: PCB matériel, ajustement pour ESP-07 plutot qu'un ESP-12 (mauvais choix, btw)
 *         retiré support emonCMS
 *         Simplification les libs à utiliser pour le NTP.

 * v1.0.x: juillet 2023: transformation, revue de code, modifications afin de tenter de régler le problème de crash
 *         lors des essais isolés (voir test_esp8266_wifi_juil2023.ino), j'ai pu simplifier les libs à utiliser pour le NTP.
 * avec le PCB du HWTMonitor, le 2e senseur de courant est en fonction... donc en route vers la v3.x de ce projet (ci-bas)
 *         Migration de SPIFFS à LittleFS
 * v1.1.x: 28 juillet 2023 - Ajout support MQTT, restructure du basetopic pour suppot +ieurs capteurs I
 * v1.2.x: multi-senseurs de courant à l'aide du tiny85, 2e probe de courant ... reste à calibrer
 * v2.0.x: intégration de TB: ça marche!! Il a cependant du charger la lib TBPubSubClient au lieu de la std PubSubClient
 * v3.0.x: intégration des 2 WLD; DEL rouge allume lorsque 1 ou les 2 WLD sont actifs
 * v3.1.x: plusieurs correctifs:
            - tenter de vider le buffer avant envoie d'une commande au tiny85, augmenter le delais d'attente de reponse
            - ajout à l'envoi TB du compteur d'alarme des WLD
            - ajout minuterie propre à WLD
            - ajout ms.devLoc pour la localisation du device
            - ajout ms.tbHostname et ms.mqttHostname en prévision
            - menage dans les timer (dans currentStatus) et timerDelays (dans Config)
            - ajout des nouv param de config lors de la commande getcfg
 * v3.2.x: ajout currentStatus.tempSensorDiscov pour connaitre l'état des Tsensors sur le bus
      bug: Re-vérifier le tempSensorDiscov car indique trjs "false" meme si on a les 3 températures...

à faire (x = fait):
 *         compléter le tour des param configurable via requete POST: un handler pour chaque param configurable
 *           Yh juin 2025: paramétriser via MQTT et JSON plutot que http.
 *         commande pour donner l'état, la config des DS18B20, avec addresse, dans le bon ordre
 *         ajouter protection contre un doublon d'adresse lors du discovery des ds18b20, collecter que les détectés
 *        x ajouter le devLoc à TB attributes
 *         avis sonore, surveillance overheat
 *         déf pour pour le BUZZER
 *         (ok, fini, n'est plus un prblm) Thingsboard... je soupsconne que c'est ce qui fait crasher la patente... à tester
 *
 * v4.x: ajout d'un module de commande "energy saver" avec relais et cédule, controle électronique de maintient de la T
 * v5.x: support pour plusieurs chauffe-eau, surveillance de groupe?
 * v6.x: écran OLED indiquant l'état (opt)
 * 
 * À faire (+ voir le plan des versions):
 *    faire en sorte que TB soit configuré par hostname/ip, donc un char* au lieu d'une IP dans la config
 *    que faire avec le buzzer?
 *    * WLD sur une surveillance à la seconde plutôt que 10 (configurable?) -> avoir sa propre minuterie
 *    p ajouter element dans la reponse JSON: deviceID et locationID (configurables). DeviceID est a la base = chipID ou DevName+3dernMACByte
 *    x OTA
 *    x versionning
 *    0  NoN! marche pas pour ESP8266 : voir pour utiliser Preferences au lieu de EEPROM... ou sinon LittleFS  (voir RandomNerdsTut)
 *    param ICAL ajustable
 *    x save config EEPROM
 *    x push MQTT (ip, topic, activ)
 *    x support pour 2 current sensors - fait au niveau du tiny85 v1.1.4
 *    x cumul du temps où le courant est > (x)
 *    gérer une perte de connectivité Wifi
 *    à tester final - démélanger l'idée de stocker dans les array Average avec index... juste considérer le config lors de la présentation des données.
 *    Bug2Fix: considérer un état que "s'il est assez long". Le code ci-bas est bon pour le logging des activités mais pas lors du
 *             rapport 'since' (curstatus).
 *    x utiliser le tiny85 à 5.0v pour les mesures (meilleure sensibilité/justesse/échelle) et mettre un level shifter entre tiny85 et ESP8266
 *    
 */
//-----------------------------------------------------------------------

//--- Librairies (en ordre alpha) ---------------------------------------
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <Average.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include <SoftwareSerial.h>
#include <DallasTemperature.h>
#include <TBPubSubClient.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#define USING_HTTPS false
#define ENCRYPTED false
#include <ThingsBoard.h>
#include <time.h>  //Dans le cas du ESP8266, cette lib contient tout pour le sntp

//--- Definitions -------------------------------------------------------
#define ONE_WIRE_BUS 4
#define TEMPERATURE_PRECISION 9
#define LED_RED_PIN 13
#define LED_YEL_PIN 12
#define BUZZ_PIN 15
#define WLD01_PIN 5
#define WLD02_PIN 2
#define webServicePort 80
#define swSerRxPin 14
#define swSerTxPin 16

// --- Gestion de l'heure via NTP -------------------
const int32_t gmtOffset_sec = -5*3600;
const int32_t daylightOffset_sec = 1*3600;
const char* ntpServer = "pool.ntp.org";
const int EPOCH_1_1_2019 = 1546300800; //1546300800 =  01/01/2019 @ 12:00am (UTC)

// --- WebServer et Updater -------------------------
ESP8266WebServer server(webServicePort);
ESP8266HTTPUpdateServer httpUpdater;
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";

// --- Wifi, AP -------------------------------------
const IPAddress apIP(192, 168, 1, 1);
const char* apSSID = "WEBESPSetup";
#define defaultDevName "WEBESP8266"
const char* deviceName = defaultDevName;
bool wifiSetOk = false;
String ssidList;

// --- Pour la communication avec le Tiny85 - module traitement du courantI
SoftwareSerial swser(swSerRxPin, swSerTxPin); // RX | TX

// -- Pour gérer un fichier recu avec le FS
File fsUploadFile;

// ---  Gestion des probes de temperature via DS18B20
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses - please refer to the configStruc for mapping of which is which
const uint8_t numTempDev = 3;
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
const uint8_t numProbe = 2;
Average<uint16_t> durationDataPR1(40);  //Probe 01
Average<uint32_t> activityDataPR1(40);
Average<uint16_t> durationDataPR2(40);  //Probe 02
Average<uint32_t> activityDataPR2(40);

Average<uint16_t> *durationDataProbe[numProbe] = {&durationDataPR1, &durationDataPR2};
Average<uint32_t> *activityDataProbe[numProbe] = {&activityDataPR1, &activityDataPR2};

// --- Water Leak Detectors (WLD)
const uint8_t numWLD = 2;

// --- Client MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);

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

//29/07/23: Voir si requis de déplacer ces switchs dans la config?
 #define dbgMQTTConnect false   //debugging purposes only
 #define dbgTBStatus false
 #define dbgWLDStatus false
 
// --- Client Thingsboard
constexpr uint32_t MAX_MESSAGE_SIZE = 350U;
WiFiClient espTbClient;
ThingsBoard tb(espTbClient, MAX_MESSAGE_SIZE);

// --- Configuration en mémoire ---------------------
const uint16_t schedROMOffset=128;  // voir detail du EEPROM mapping
const uint16_t configBaseLocation=0;

static const unsigned long STRUCT_MAGIC = 1234567316;  //3 derniers digit= _VERSION
static const byte STRUCT_VERSION = 1;

struct ConfigDataStruc {
  unsigned long magic;
  byte struct_version;
  uint16_t recordNumber;  //nombre de fois que la structure a ete ecrite dans EEPROM, compte le nb de variable modifiee

  uint8_t logMQTTMode; // 0=off, voir ci-haut les modes
  uint8_t tbMode;
  uint32_t getTempTimerDelay;
  uint32_t statusLedTimerDelay;
  uint32_t wldTimerDelay;

  uint32_t minActDuration;
  uint8_t loopCount; // numb retries value for wifi connection 

  bool buzzerActif; //indique si le buzzer est en service ou non; au 29/07/23: pas implémenté

  uint8_t TsensorIndex[3];

  float currentIThreshold; // valeur a partir de laquelle on compte que le chauffe-eau consomme de l'énergie. Q: que fait-on si 1 ou 2 élements?

  char tbHostname[50+1]; // FQDN du host TB  ex: mqtt.thingsboard.cloud ou tbcaltgp.claurendeau.qc.ca
  uint8_t tbHostIp[4]; //Deprecated (après v3.1)
  char tbToken[32+1];
  uint32_t tbRetryDelay;

  char mqttHostname[50+1]; //FQDN du host MQTT
  uint8_t mqttIp[4];  //Deprecated (après v3.1)
  int mqttPort;
  char mqttUser[15+1];
  char mqttPassword[31+1];

  uint8_t wifiConnectDelay;

  bool dbgReadTime;      //used for debugging
  bool dbgTempSensors;  //used for debugging
  bool dbgReadCurrent;   //used for debugging
  bool dbgTinyStatus;

  char devLoc[50+1];  // Device site location
  char basetopic[20+1];
  char devName[15 + 1];  //Max 15 char + EOS
  char ssid[31 + 1]; // WIFI ssid + null
  char password[31 + 1]; // WiFi password,  if empyt use OPEN, else use AUTO (WEP/WPA/WPA2) + null
};

ConfigDataStruc ms;
const int ConfigDataStruc_storageSize = sizeof(ConfigDataStruc);

// -- Structure globale contenant l'état du système:
struct Status {
 //Uptime data:
  uint32_t startTime=0;
  
  float currentIValue[numProbe] = {-1,-1};
  uint16_t countBadCurrentIStatus[numProbe] = {0,0};
  bool lastCurrentIStatus[numProbe] = {false,false};
  bool FSSuccess=false;

  int8_t activiteCourante[numProbe] = {0,0}; //0=OFF, 1=ON, -1=non-initialisée
  int8_t activitePrecedente[numProbe] = {0,0}; //0=OFF, 1=ON, -1=non-initialisée
  uint32_t activityStartTime[numProbe] = {0,0};

  bool WLDCurrentStatus[2] = {false,false};
  bool WLDLastStatus[2] = {false,false};
  uint16_t WLDAlarmTrigCntr[2] = {0,0};

  uint8_t MQTTConnected=false;
  uint8_t MQTTLastMsgSuccess=false;
  uint8_t TBConnected=false;
  uint32_t lastTBConnect=0;
  uint16_t tbConnectFailed=0;

  float currentTempSensorValue[numTempDev]= {0,0,0};
  bool tempSensorDiscov[numTempDev] = {false,false,false};

  //Minuteries:
  uint32_t wldTimer = 0;
  uint32_t getTempTimer = 0;
  uint32_t statusLedTimer=0;

};

struct Status currentStatus;

//--------------------------------------------------


//--- Prototypes --------------------------------------------------------
void getObjData(JsonArray&, Average<uint32_t>*, uint16_t);
void getObjData(JsonArray&, Average<uint16_t>*, uint16_t);
void getObjData(JsonArray&, Average<float>*, uint16_t);
void getTempData(JsonObject&, uint8_t, uint16_t, uint8_t );
void getActivityData(JsonObject&, uint16_t, uint16_t, uint8_t);
void listFSContent(JsonObject&);
void loadConfigData(bool);
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
time_t getNow(void);
void setupMode(void);
void startWebServer(void);
String makePage(String, String);
String urlDecode(String);
//static tm getDateTimeByParams(long);
//static String getDateTimeStringByParams(tm *, char* );
//static String getEpochStringByParams(long time, char* );
uint8_t goReadCurrentSensor(uint8_t);
//uint8_t pushDataToMQTT(void);
uint8_t goReadTemperatureSensors(void);
//-----------------------------------------------------------------------

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
void getActivityData(JsonObject &obj, uint16_t history, uint8_t probeIndx) {
  obj["reftstmp"] = currentStatus.startTime;
  obj["probeID"] = probeIndx;
  JsonArray tempActValues = obj.createNestedArray("endtime");
  getObjData(tempActValues, activityDataProbe[probeIndx], history);
  tempActValues = obj.createNestedArray("duration");
  getObjData(tempActValues, durationDataProbe[probeIndx], history);
}



/*
 * Nom: saveToEEPROM
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
 * Nom: loadConfigData
 * Fonction: charge la configuration dans la structure ms
 * Argument(s) réception: bool pour forcer une remise a l'état de base de la configuration (+/- implémentée)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale): structure ms
 */
void loadConfigData(bool returnToDefault) {

  // Lit la mémoire EEPROM
  EEPROM.get(configBaseLocation, ms);
  
  // Détection d'une mémoire non initialisée
  byte erreur = ms.magic != STRUCT_MAGIC;

  if (erreur || returnToDefault) {

   //--- Configuration par defaut
    
    ms.TsensorIndex[0] = 1;  //T sensor index 0
    ms.TsensorIndex[1] = 2;  //T sensor index 1
    ms.TsensorIndex[2] = 0;  //T sensor index 2

    ms.currentIThreshold = 1;
    
    ms.logMQTTMode=0;    // 0=off, bits: 0=temp msg , 1=star events. 2=stop events, 3=bucket full event, 4=remote low batt, 5=remote status, etc

    ms.tbMode=0; //Thingsboard

    ms.wifiConnectDelay=30;

    ms.getTempTimerDelay=10000; // 1 donnee de temperature au 10 secondes
    ms.minActDuration=60; //Durée minimale d'une activité de chauffage (élimine les erreurs intempestives)

    ms.statusLedTimerDelay=1200;
    ms.wldTimerDelay=1000;

    ms.buzzerActif = false; //Par defaut, buzzer pas en service

    ms.loopCount = 10; //number of retries
    ms.mqttPort = 1883;
    strcpy(ms.mqttUser, "HWTESP");
    strcpy(ms.mqttPassword,"HWTESP");

    strcpy(ms.basetopic,"ESPHWTMON");

    strcpy(ms.tbHostname,"tb.local");
    strcpy(ms.mqttHostname,"mqtt.local");

    ms.mqttIp[0] = 192;
    ms.mqttIp[1] = 168;
    ms.mqttIp[2] = 122;
    ms.mqttIp[3] = 253;

    ms.tbMode=0; //Thingsboard
    ms.tbHostIp[0]= 192;
    ms.tbHostIp[1]= 168;
    ms.tbHostIp[2]= 122;
    ms.tbHostIp[3]= 210;
    strcpy(ms.tbToken,"uUiPWTycMqRDGy5RnU7j");
    ms.tbRetryDelay = 20000; //Delais entre 2 tentatives

    ms.wifiConnectDelay = 20; // 60 seconds

    ms.dbgReadTime=true;
    ms.dbgTempSensors=false;
    ms.dbgReadCurrent=false;
    ms.dbgTinyStatus = false;
    
    strcpy(ms.devLoc,"here");
    strcpy(ms.devName,deviceName);
    strcpy(ms.ssid,"hwtmon");
    strcpy(ms.password,"hwtmon");

    // Sauvegarde les nouvelles données
    saveToEEPROM();
  }
}

//-----------------------------------------------

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
    
    ssid.toCharArray(ms.ssid,31);
    pass.toCharArray(ms.password,31);

    String s = "<h1>Setup complete.</h1><p>device will be connected to \"";
    s += ssid;
    s += "\" at next restart.";
    server.send(200, "text/html", makePage("Wi-Fi Settings", s));
    wifiSetOk = true;

    Serial.println("Writing SSID to EEPROM...");
    saveToEEPROM();
    delay(1000);
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
  s.replace("%20", " ");  s.replace("+", " ");
  s.replace("%21", "!");  s.replace("%22", "\"");
  s.replace("%23", "#");  s.replace("%24", "$");
  s.replace("%25", "%");  s.replace("%26", "&");
  s.replace("%27", "\'"); s.replace("%28", "(");
  s.replace("%29", ")");  s.replace("%30", "*");
  s.replace("%31", "+");  s.replace("%2C", ",");
  s.replace("%2E", ".");  s.replace("%2F", "/");
  s.replace("%2C", ",");  s.replace("%3A", ":");
  s.replace("%3A", ";");  s.replace("%3C", "<");
  s.replace("%3D", "=");  s.replace("%3E", ">");
  s.replace("%3F", "?");  s.replace("%40", "@");
  s.replace("%5B", "[");  s.replace("%5C", "\\");
  s.replace("%5D", "]");  s.replace("%5E", "^");
  s.replace("%5F", "-");  s.replace("%60", "`");
  return s;
}

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
       wifiSetOk = true;
       return (true);
    }
    delay(loopDelay);  
    count++;
    digitalWrite(LED_YEL_PIN,count%2);
  }
  Serial.println("Timed out.");
  return false;
}

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
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSSID);
  delay(100);
  
  startWebServer();
  Serial.print("Starting Access Point at \"");
  Serial.print(apSSID);
  Serial.println("\"");
}

// ------------------------------------------------------

/*
 * Nom: 
 * Fonction: 
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */
void listFSContent(JsonObject &obj) {
  if (currentStatus.FSSuccess) {
      uint16_t countItem = 0;
      obj["dirContent"]=String("/");
      Dir dir = LittleFS.openDir("/");
      while (dir.next()) {
        JsonArray fsItem = obj.createNestedArray(String("item"+String(countItem)));
        fsItem.add(String(dir.fileName()));
        fsItem.add(String(dir.fileSize()));
        countItem++;
      }
      if (countItem==0) {
        JsonArray fsItem = obj.createNestedArray("item0");
        fsItem.add("emptyList");
        fsItem.add("0");
      }
  } else {
    obj["dirContent"]="uninitialized";
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
void setCrossOrigin(){
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Max-Age", "600");
    server.sendHeader("Access-Control-Allow-Methods", "PUT,POST,GET,OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "*");
};

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
    Serial.println("sendCORSHeader");
    server.sendHeader("access-control-allow-credentials","false");
    setCrossOrigin();
    server.send(204);
}

void handleRoot() {
  server.send(200, "text/html", "Welcome to the REST Web Server");
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
  if (LittleFS.exists(pathWithGz) || LittleFS.exists(path)){  // If the file exists, either as a compressed archive, or normal
     if (LittleFS.exists(pathWithGz))                          // If there's a compressed version available
        path += ".gz";                                         // Use the compressed version
     File file = LittleFS.open(path, "r");                    // Open the file
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
 * Fonction: upload a new file to the LittleFS
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale):
 * Notes:  (spécial, source, amélioration, etc)
 * 
 */ 
void handleFileUpload(void) {
  if (currentStatus.FSSuccess) {
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START){
      String filename = upload.filename;
      if(!filename.startsWith("/")) filename = "/"+filename;
      Serial.print("handleFileUpload Name: "); Serial.println(filename);
      fsUploadFile = LittleFS.open(filename, "w");            // Open the file for writing in LittleFS (create if it doesn't exist)
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

void handleNotFound(){
  digitalWrite(BUZZ_PIN, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(BUZZ_PIN, 0);
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
void getSettings(void) {
    setCrossOrigin();
//
      // Allocate a temporary JsonDocument
      // Don't forget to change the capacity to match your requirements.
      // Use arduinojson.org/v6/assistant to compute the capacity.
    //  StaticJsonDocument<512> doc;
      // You can use DynamicJsonDocument as well
      DynamicJsonDocument doc(2048);
 
      //doc["ltime"] = getEpochStringByParams(usET.toLocal(getNow()));  -- Yh juil2023: trouver une alternative
      doc["fwver"] = _VERSION;
      doc["uxtm"] = getNow();

      //if (server.arg("getcfg") == "true") {
      if (server.hasArg("getcfg")) {
        doc["struct_version"] = ms.struct_version;
        doc["recordNumber"] = ms.recordNumber;
        doc["logMQTTMode"] = ms.logMQTTMode;
        doc["MQTTconnexion"] = currentStatus.MQTTConnected;
        doc["MQTTLastMsgStatus"] = currentStatus.MQTTLastMsgSuccess;
        doc["tbMode"] = ms.tbMode;
        doc["tbStatus"] = currentStatus.TBConnected;
        doc["tbFailure"] = currentStatus.tbConnectFailed;
        doc["tbConnectDly"] = ms.tbRetryDelay;
        doc["getTempTimerDelay"] = ms.getTempTimerDelay;
        doc["wldTimerDelay"] = ms.wldTimerDelay;
        doc["statusLedTimerDelay"] = ms.statusLedTimerDelay;
        doc["buzzerMode"] = ms.buzzerActif?"true":"false";

        doc["loopCount"] = ms.loopCount;
        JsonArray tsensorArr = doc.createNestedArray("TsensorIndex");
        for (uint8_t i=0; i<numTempDev ; i++) {
          tsensorArr.add(ms.TsensorIndex[i]);
          doc["ts"+String(i)+"Status"]=currentStatus.tempSensorDiscov[i]?"true":"false";
        }
        doc["currentIThreshold"] = ms.currentIThreshold;

        doc["mqttHost"] = ms.mqttHostname;
        IPAddress mqttIp(ms.mqttIp);
        doc["mqttIp"] = mqttIp.toString();
        doc["mqttPort"] = ms.mqttPort;
        doc["mqttUser"] = ms.mqttUser;
        doc["mqttPassword"] = ms.mqttPassword;
        
        doc["tbHost"] = ms.tbHostname;
        IPAddress tbHostIp(ms.tbHostIp);
        doc["tbHostIp"] = tbHostIp.toString();
        doc["tbToken"] = ms.tbToken;

        uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
        doc["mqttDevTopic"] = "esphwt_"+String(num,HEX);
        doc["mqttBasetopic"] = ms.basetopic;

        doc["devLoc"] = ms.devLoc;
        doc["devName"] = ms.devName;
        doc["ssid"] = ms.ssid;

        doc["dbgReadTime"]=ms.dbgReadTime?"true":"false";
        doc["dbgTempSensors"]=ms.dbgTempSensors?"true":"false";
        doc["dbgReadCurrent"]=ms.dbgReadCurrent?"true":"false";
        doc["dbgTinyStatus"]=ms.dbgTinyStatus?"true":"false";

      }

      if (currentStatus.startTime>0) {
        doc["uptime"] = (getNow()-currentStatus.startTime);
      }

      //if (server.arg("data") == "true") {
      if (server.hasArg("data")) {
         JsonObject obj = doc.createNestedObject("temp");
         obj["cold"] = String(getTemperature(thermalDev[ms.TsensorIndex[0]]),1);
         obj["hot"] = String(getTemperature(thermalDev[ms.TsensorIndex[1]]),1);
         obj["tk"] = String(getTemperature(thermalDev[ms.TsensorIndex[2]]),1);
         doc["currentPR0"] = String(currentStatus.currentIValue[0],2);
         doc["currentPR1"] = String(currentStatus.currentIValue[1],2);
         doc["WLD01"] = currentStatus.WLDCurrentStatus[0];
         doc["WLD02"] = currentStatus.WLDCurrentStatus[1];
      }
       
      //if (server.arg("signalStrength")== "true"){
      if (server.hasArg("signalStrength")){
         doc["sStrengh"] = WiFi.RSSI();
      }

      //if (server.arg("chipInfo")== "true"){
      if (server.hasArg("chipInfo")){
         doc["cpId"] = ESP.getChipId();  //Note Yh au 12janv2022: transcrire en HEX, ie: String(num,HEX);
         doc["flCpId"] = ESP.getFlashChipId();  //Note Yh au 12janv2022: transcrire en HEX, ie: String(num,HEX);
         doc["flCpSz"] = ESP.getFlashChipSize();  // En MB (div 1024) + "MB")
         doc["flCpRSz"] = ESP.getFlashChipRealSize();  // En MB (div 1024) + "MB")
      }
      //if (server.arg("freeHeap")== "true"){
      if (server.hasArg("freeHeap")){
         doc["freeHeap"] = ESP.getFreeHeap();  // En MB (div 1024) + "MB")
      }
      
      //if (server.arg("ipconf")== "true"){
      if (server.hasArg("ipconf")){
         doc["ip"] = WiFi.localIP().toString();
         doc["gw"] = WiFi.gatewayIP().toString();
         doc["nm"] = WiFi.subnetMask().toString();
          
         uint8_t macAddr[6];
         WiFi.macAddress(macAddr);
         char macAddrC[20]={};
         sprintf(macAddrC,"%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
         doc["mac"] = macAddrC;
      }

      //if (server.arg("curstatus") == "true") {
      if (server.hasArg("curstatus")) {
        char probeID[5] = "PR";
        for (uint8_t indx=0;indx<numProbe;indx++) {
          probeID[2]=indx+48;  // 0 value becomes ascii value of "0"
          probeID[3]=0;
          JsonObject obj = doc.createNestedObject(probeID);
        
          obj["curStBdCount"] = currentStatus.countBadCurrentIStatus[indx];
        
          obj["lstCurSt"] = currentStatus.lastCurrentIStatus[indx];
          if (currentStatus.activiteCourante[indx] >=0 ) {
            obj["actcour"] = currentStatus.activiteCourante[indx];
            obj["since"] = getNow()-currentStatus.activityStartTime[indx];
          }
        }//End-for
        JsonObject wld = doc.createNestedObject("WLDAlarm");
        for (uint8_t indx=0;indx<numWLD;indx++) {
          wld["WLD0"+String(indx)] = currentStatus.WLDAlarmTrigCntr[indx];
        }//End-for
      }
      //if (server.arg("listFS") == "true") {
      if (server.hasArg("listFS")) {
        JsonObject obj = doc.createNestedObject("FS");
        listFSContent(obj);
   }

      //if (server.arg("tC0") == "true") {
      if (server.hasArg("tC0")) {
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
      //if (server.arg("tC1") == "true") {
        if (server.hasArg("tC1")) {
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
      //if (server.arg("tC2") == "true") {
      if (server.hasArg("tC2")) {
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

      //if (server.arg("activity") == "true") {
      if (server.hasArg("activity")) {
        uint16_t limit=0;
        uint8_t probeIndx = 0; //Defaut probe is 0, unless specified by prbnb argument
        if (server.hasArg("last")) limit = server.arg("last").toInt();
        if (server.hasArg("prbnb")) {
          uint8_t tempVal = server.arg("prbnb").toInt();
          if (tempVal < numProbe) probeIndx = tempVal;
        }
        JsonObject obj = doc.createNestedObject("activity");
        getActivityData(obj,limit,probeIndx);
      }
       
      Serial.print("Stream...");
      String buf;
      serializeJson(doc, buf);
      server.send(200, "application/json", buf);
      Serial.println("done.");
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
    
  Serial.println("postConfigFile");

  setCrossOrigin();

  String postBody = server.arg("plain");
  Serial.println(postBody);

  uint8_t needToSaveEEPROM = 0;
  DynamicJsonDocument reply(512);

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, postBody);
  if (error) {
     // if the file didn't open, print an error:
     Serial.print("Error parsing JSON ");
     Serial.println(error.c_str());
 
     String msg = error.c_str();
 
     server.send(400, "text/html", "Error parsing json body! <br>"+msg);
 
  } else {
     JsonObject postObj = doc.as<JsonObject>();

     Serial.print("HTTP Method: ");
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
                  if (ms.logMQTTMode==0) //checking previous status. If was 0 then need to configure cnx first
                    initMQTTConnection();
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
// fontionnalité de emoncms n'est plus supportée MAIS je conserve pour inspiration du traitement...
        //  if (postObj.containsKey("emoncmsTcp")) {
        //    if (postObj["emoncmsTcp"]>1024 && postObj["emoncmsTcp"]<65000) {
        //      if(ms.emoncmsTcp != postObj["emoncmsTcp"]) {
        //        ms.emoncmsTcp= postObj["emoncmsTcp"];
        //        reply["emoncmsTcp"]="success";
        //        needToSaveEEPROM++;
        //      } else {
        //        reply["emoncmsTcp"] = "noChg";
        //      }
        //    } else {
        //      reply["emoncmsTcp"] = "invalidPort";
        //    }
        //  } // Case de emoncmsTcp
        //  if (postObj.containsKey("emoncmsKey")) {
        //    String emoncmsKey = postObj["emoncmsKey"];
        //    if (emoncmsKey.length()>=16 && emoncmsKey.length()<=32) {
        //      // Ajouter ici un test de similarité (et si oui, code reply["emoncmsKey"]="noChg";)
        //      strcpy(ms.emoncmsKey, postObj["emoncmsKey"]);
        //      reply["emoncmsKey"]="success";
        //      needToSaveEEPROM++;
        //    } else {
        //      reply["emoncmsKey"]="invalidSize";
        //    }
        //  }  // Cas de emoncmsKey

         if (postObj.containsKey("basetopic")) {
            String basetopic = postObj["basetopic"];
            if (basetopic.length()>3 && basetopic.length()<=20) {
              strcpy(ms.basetopic, postObj["basetopic"]);
              reply["basetopic"]="success";
              needToSaveEEPROM++;
            } else {
              reply["basetopic"]="invalidSize";
            }
         }  // Cas de basetopic

         if (postObj.containsKey("dbgserial")) {
           if (postObj["dbgserial"]<256) {
             uint8_t value = postObj["dbgserial"];
             Serial.print("dbgSerial:");
             Serial.println(value,HEX);
             ms.dbgReadTime=bitRead(value,0);
             ms.dbgTempSensors=bitRead(value,1);
             ms.dbgReadCurrent=bitRead(value,2);
             ms.dbgTinyStatus=bitRead(value,3);
             //reste encore 4,5,6,7
             reply["dbgserial"]="success";
             needToSaveEEPROM++;
           } else {
             reply["dbgserial"] = "dbgBadValue";
           }
         }
         
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
           if (postObj["devName"].as<String>().length()>3 && postObj["devName"].as<String>().length()<16) {
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
           if (postObj["ssid"].as<String>().length()>3 && postObj["ssid"].as<String>().length()<32) {
            // Ajouter ici un test de similarité (et si oui, code reply["ssid"]="noChg";)
             strcpy(ms.ssid,postObj["ssid"]);
             reply["ssid"]="success";
             needToSaveEEPROM++;
           } else {
             reply["ssid"] = "invalidLength";
           }
         } // Cas de ssid (Name)
         
         //Attention, il pourrait y avoir 0 length dans un passphraseSSID
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
          } else {
             reply["reinitMQTT"] = "failed";
          }
       } // traitement connection MQTT

       // Y a-t-il qqch à sauvegarder?
       if (needToSaveEEPROM>0) {
         saveToEEPROM();
         reply["nbElement"]=String(needToSaveEEPROM);
         String buf;
         serializeJson(reply, buf);
         server.sendHeader("Content-Length", String(buf.length()));
         server.send(201, "application/json",buf);
       } else {
         server.send(204, "text/html", "No data found, or incorrect!");
       }
   }  //else de erreur de deserialization
}//Fin de setSettings

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

  server.on("/", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_OPTIONS, sendCrossOriginHeader);
  server.on("/settings", HTTP_GET, getSettings);
  server.on("/settings", HTTP_POST, setSettings);
  server.on("/upload", HTTP_POST,                       // if the client posts to the upload page
    [](){ server.send(200); },                          // Send status 200 (OK) to tell the client we are ready to receive
    handleFileUpload                                    // Receive and save the file
  );
  server.on("/reset", [](){
    server.send(200, "text/plain", "ESP reset");
    delay(500);
    ESP.restart();
  });
}

/*
 * Nom: getCmdFromTiny 
 * Fonction: interroger le tiny via interface série
 * Argument(s) réception: cmd=la lettre de la commande à passer via interf sériel
 * Argument(s) de retour: un string contenant la réponse du tiny
 * Modifie/utilise (globale): (rien)
 * Notes:  amélioration: supporter la commande "I", pourvu que la cmd I soit bien fonctionnelle
 * 
 */ 
String getCmdFromTiny(char cmd) {

  const char charMarker = '\n';
  const int maxChar = 60;
  char inData[maxChar+1];  //buffer for receiv data

  //Clean data buffer... in case...
  while (swser.available()>0) {
    char oneChar = swser.read();
    delay(2); 
  }

  swser.println(cmd);
  delay(60);  //Let the Tiny react upon cmd
  int indx=0;
  bool signOff=false;

  // Tentative de lire jusqu'à ce qu'un '\n' soit reçu...
  while (swser.available()>0 && indx<maxChar && !signOff) {
      char oneChar = swser.read();
      if (oneChar != charMarker)
          inData[indx++] = oneChar;
      else signOff=true;
      delay(2); //let next char coming in (9 bits @9600b/s takes about 1ms)
  }

  inData[indx] = 0x00;

  //Clean data buffer... in case...
  while (swser.available()>0) {
    char oneChar = swser.read();
    delay(2); 
  }

  return String(inData);
}

/*
 * Nom: isClean
 * Fonction: valide si une chaine de caractere contient des caracteres autorisés
 * Argument(s) réception: la chaine à valider
 * Argument(s) de retour: bool indiquant si oui ou non la chaine est bonne
 * Modifie/utilise (globale): (rien)
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
 * Nom: getCurrent
 * Fonction: procède à la lecture d'une prode de courant avec le tiny
 * Argument(s) réception:
    curVal: une variable float dans laquelle sera retournée la valeur lue du courant
    probeIndx: numéro de la probe (0 ou 1, selon numProbe)
 * Argument(s) de retour: succès de l'opération
 * Modifie/utilise (globale): (rien)
 * Notes:  (spécial, source, amélioration, etc) 
 */ 
bool getCurrentI(float& curVal, uint8_t probeIndx) {

  bool success = false;
  char probeCmd = 'a';
  if (probeIndx == 1) probeCmd = 'b';

  String tempStrI=getCmdFromTiny(probeCmd);

  if (ms.dbgReadCurrent) Serial.printf("str=%s",tempStrI.c_str());

  if (tempStrI.length() > 3) {  
    if (tempStrI.indexOf('.') > 0 && isClean(tempStrI)) {
      float tempI = tempStrI.toFloat();
      // float tempI = swser.parseFloat();
      if (tempI >= 0.0 && tempI < 40.0) {
        success=true;
        curVal = tempI;
        if (ms.dbgReadCurrent) Serial.printf(" val=%.2f\n",tempI);
      } else 
        if (ms.dbgReadCurrent) Serial.println(" - valOutOfRange.");
    } else {
      if (ms.dbgReadCurrent) Serial.println(" - junkVal.");
    }
  } else
    if (ms.dbgReadCurrent) Serial.println(" - too short.");

  return success;
}

/*
 * Nom: getNow
 * Fonction: retourne le temps en valeur epoch (unixtime stamp); remplace now()
 * Argument(s) réception: (rien)
 * Argument(s) de retour: time_t ou int32_t
 * Modifie/utilise (globale): (rien)
 */ 
time_t getNow() {
  time_t now = 0;
  uint8_t count = 0;
  while (now < EPOCH_1_1_2019 && count<10) {
    now = time(nullptr);
    delay(500);
    count++;
  }
  return now;
}

/*
 * Nom: goReadCurrentSensor
 * Fonction: réalise la demande de lecture du courant et traite son état
 * Argument(s) réception: numéro d'index de la probe de courant
 * Argument(s) de retour: (rien de spécial)
 * Modifie/utilise (globale): etat dans la srtucture 'currentStatus'
 */ 
uint8_t goReadCurrentSensor(uint8_t probeIndx) { 
   static float tempI;
   tempI=-1;

   // Lecture du sensor de courant via le SwSerial:
   bool currentIStatus = getCurrentI(tempI,probeIndx);

   // Si la lecture est bonne:
   if (currentIStatus) {
      currentStatus.currentIValue[probeIndx] = tempI;

      // Traiter ici le threshold et ainsi logger le événement de consommation
      // a) état de consommation: activiteCourante=ON si currentIValue > threshold sinon OFF
      // b) comparer l'état d'activité précédente à courante; si passage de ON à OFF, on loggue le timestamp (now()) et la durée (now()-startTime)
      // c) comparer l'état d'activité précédente à courante: si passage de OFF à ON, startTime=now()
      // Bug2Fix: considérer un état que "s'il est assez long". Le code ci-bas est bon pour le logging des activités mais pas lors du
      //          rapport 'since' (curstatus).
      if (currentStatus.currentIValue[probeIndx] > ms.currentIThreshold)
        currentStatus.activiteCourante[probeIndx] = 1;
      else 
        currentStatus.activiteCourante[probeIndx] = 0;

      if (currentStatus.activitePrecedente[probeIndx]==0 && currentStatus.activiteCourante[probeIndx]==1) {
         currentStatus.activityStartTime[probeIndx]=getNow();
      }
      if (currentStatus.activitePrecedente[probeIndx]==1 && currentStatus.activiteCourante[probeIndx]==0) {
         if (currentStatus.activityStartTime[probeIndx]>0) {
            uint32_t duration = getNow() - currentStatus.activityStartTime[probeIndx];
            if (duration > ms.minActDuration) {
               durationDataProbe[probeIndx]->push(duration);
               activityDataProbe[probeIndx]->push(getNow());
               if (WiFi.status()== WL_CONNECTED) {
                 if (currentStatus.MQTTConnected && bitRead(ms.logMQTTMode,dataAct)) {
                   uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
                   String topic = "esphwt_"+String(num,HEX)+"/PR"+String(probeIndx)+"/lstDuraction";
                   mqttClient.publish(topic.c_str(),(String(duration)).c_str());
                 }
               }
            } else {
               Serial.println("Event not logged (too short)");
            }
            currentStatus.activityStartTime[probeIndx] = getNow();
         }
      }
      currentStatus.activitePrecedente[probeIndx]=currentStatus.activiteCourante[probeIndx];
   } else 
     currentStatus.countBadCurrentIStatus[probeIndx]++;

   currentStatus.lastCurrentIStatus[probeIndx] = currentIStatus;

   return 1;
}


/*
 * Nom: toStringAddZero
 * Fonction: padding de "0" en avant d'un chiffre, si requis
 * Argument(s) réception: valeur à analyser
 * Argument(s) de retour: chaine de caract complète
 * Modifie/utilise (globale): (rien)
 * Notes:  à considérer padding pour une longeur donnée
 * 
 */ 
String toStringAddZero(int data)
{
  String st = "";
  if (data < 10)
  {
    st = "0" + String(data);
  }
  else
  {
    st = String(data);
  }
  return st;
}

/*
 * Nom: printLocalTime
 * Fonction: affiche la valeur du temps en format humain
 * Argument(s) réception: (rien)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale): (rien)
 * Notes:  amélioration: plus de flexibilité comme retourner une string
 */ 
void printLocalTime(){

  struct tm *timeinfo;

  time_t now=0;
  time(&now);
  timeinfo = localtime(&now);

  int year = timeinfo->tm_year + 1900;
  int month = timeinfo->tm_mon + 1;
  int day = timeinfo->tm_mday;
  int hour = timeinfo->tm_hour;
  int mins = timeinfo->tm_min;
  int sec = timeinfo->tm_sec;
  int day_of_week = timeinfo->tm_wday;

  Serial.print(toStringAddZero(day) + "/" + toStringAddZero(month) + "/" + String(year));
  Serial.print(" " + toStringAddZero(hour) + ":" + toStringAddZero(mins) + ":" + toStringAddZero(sec));

}

/*
 * Nom: printAddress
 * Fonction: function to print a device (1-wire) address
 * Argument(s) réception: 1-wire device address (bytes)
 * Argument(s) de retour: (rien)
 * Modifie/utilise (globale): (rien)
 * Notes:  amélioration: retourner un string
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
 * Nom: getTemperature
 * Fonction: réalise l'opération d'obtenir la lecture de la température d'une probe donnée
 * Argument(s) réception: adresse de la probe de température
 * Argument(s) de retour: valeur de température
 * Modifie/utilise (globale): (rien)
 */ 
float getTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
  }
  return tempC;
}

/*
 * Nom: goReadTemperatureSensors
 * Fonction: commande et traite la lecture des 3 sondes de température, fait la rotation dans les tableau Avg
 * Argument(s) réception: (rien)
 * Argument(s) de retour: code de retour de l'opération (0 ou 1), sur 3 bits
 * Modifie/utilise (globale): tables currentStatus.currentTempSensorValue, Avg*
 */ 
uint8_t goReadTemperatureSensors(void) {

  sensors.requestTemperatures();

  uint8_t tempSensorStatus=0;  //Status qui sera retourné de l'état de lecture des senseurs de T (0=notOk, 1=success), par bit 0=s0, 1=s1, etc
  
  // Pour chaque sonde: recupère la valeur.
  // Si valide, pousse dans le tableau Raw.
  // Si tableau "wrap around", le Raw est complété, fourni la moyenne dans l'aggregat 4mn
  // Idem pour aggregat 4mn vers le 1hr.
  for (uint8_t i=0; i<numTempDev; i++) {
    float tempC = getTemperature(thermalDev[i]);
    if(tempC != DEVICE_DISCONNECTED_C) {
      // Si valeur ok, push dans les tableaux de T
      if (ms.dbgTempSensors) {
        Serial.printf("\tT(%d):%.2f",i,tempC);
      }
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
  if (ms.dbgTempSensors) Serial.println();

  return tempSensorStatus;
}

bool checkWLD(uint8_t probeID) {
  bool retCode = false;

  int value=0;
  if (probeID < numWLD) {
    if (probeID == 0) {
      value = !digitalRead(WLD01_PIN);  //reverse logic
    }
    if (probeID == 1) {
      value = !digitalRead(WLD02_PIN);  //reverse logic
    }
    currentStatus.WLDCurrentStatus[probeID] = value;
    if (dbgWLDStatus) Serial.printf("WLD %d status=%d\t",probeID,value);
    retCode = true;
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
void MQTTcallback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println("-----------------------");
 
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

  return true;
}

bool checkConnectMQTT(void) {
  bool retCode=false;
  int loopCount=2;  //Blocking process...
  if (WiFi.status()== WL_CONNECTED) {
    bool giveUp = false;
    if (mqttClient.connected()) retCode = true;
    else {
      if (dbgMQTTConnect) Serial.print("Connecting to MQTT ");
      do {
        if (mqttClient.connect("ESPHWTDATA")) { //, mqttUser, mqttPassword )) {
          if (dbgMQTTConnect) Serial.println(" connected");
          retCode=true;
        } else {
          if (dbgMQTTConnect) Serial.print(".");
          loopCount--;
          if (loopCount<0) { 
            giveUp = true;
            if (dbgMQTTConnect) Serial.print("Giving Up: ");
            if (dbgMQTTConnect) Serial.println(mqttClient.state());
          } else {
            delay(500);  //Blocking process...
          }
        }
      } while (!mqttClient.connected() && !giveUp);
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
uint8_t pushDataToMQTT(void) { 
  if (WiFi.status()== WL_CONNECTED) {
    if (ms.logMQTTMode && currentStatus.MQTTConnected) {
      uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
      String devID = "esphwt_"+String(num,HEX);
      for (uint8_t probeID=0; probeID<numProbe;probeID++) {
        if (bitRead(ms.logMQTTMode,dataI)) {
          mqttClient.publish((String(ms.basetopic)+"/"+devID+"/PR"+String(probeID)+"/currentI").c_str(),((String(currentStatus.currentIValue[probeID],2)).c_str())); //Topic name
        }
        if (bitRead(ms.logMQTTMode,dataCA)) {
          mqttClient.publish((String(ms.basetopic)+"/"+devID+"/PR"+String(probeID)+"/activity").c_str(),((String(currentStatus.activiteCourante[probeID])).c_str())); //Topic name
        }
      }
      if (bitRead(ms.logMQTTMode,dataT)) {
        DynamicJsonDocument doc(256);
        for (uint8_t i=0; i<numTempDev;i++) {
          doc["tempC"+String(i)]=currentStatus.currentTempSensorValue[ms.TsensorIndex[i]];
        }
        String mqttData;
        serializeJson(doc, mqttData);
        mqttClient.publish((String(ms.basetopic)+"/"+devID+"/tempProbes").c_str(),mqttData.c_str());
      }
    }
  }
  return 1;
}

bool initTBConnection(void) {
  // Reconnect to ThingsBoard, if needed
  bool retCode=false;
  if (WiFi.status() == WL_CONNECTED) {
    if (!tb.connected()) {
      if (millis() > currentStatus.lastTBConnect+ms.tbRetryDelay) {
        // Connect to the ThingsBoard
        Serial.print("TB connecting to: ");
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
  }
  currentStatus.TBConnected=retCode;
  return retCode;
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
      const int data_items = 9;
      Telemetry data[data_items] = {
        { "currentI0",currentStatus.currentIValue[0] },
        { "currentI1",currentStatus.currentIValue[1] },
        { "activity0",currentStatus.activiteCourante[0] },
        { "activity1",currentStatus.activiteCourante[1] },
        { "tempC1", currentStatus.currentTempSensorValue[ms.TsensorIndex[0]] },
        { "tempC2",currentStatus.currentTempSensorValue[ms.TsensorIndex[1]] },
        { "tempC3",currentStatus.currentTempSensorValue[ms.TsensorIndex[2]] },
        { "WLD01Alarm" , currentStatus.WLDAlarmTrigCntr[0]},
        { "WLD02Alarm" , currentStatus.WLDAlarmTrigCntr[1]}
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
      
        const int attribute_items = 9;
        Attribute attributes[attribute_items] = {
           { "device_type",  "sensor" },
           { "active",       true     },
           { "deviceID", deviceID.c_str() },
           { "devLocation", ms.devLoc },
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

//-----------------------------------------------


void setup(void){
  pinMode(LED_RED_PIN,OUTPUT);
  digitalWrite(LED_RED_PIN,LOW);
  pinMode(LED_YEL_PIN,OUTPUT);
  digitalWrite(LED_YEL_PIN,LOW);
  pinMode(BUZZ_PIN,OUTPUT);
  digitalWrite(BUZZ_PIN,LOW);

  pinMode(WLD01_PIN,INPUT);
  pinMode(WLD02_PIN,INPUT);

  Serial.begin(115200);  
  while (!Serial) {yield();}

  swser.begin(9600);

  EEPROM.begin(512);
  loadConfigData(false);
  
  Serial.println();
  Serial.print("HWT Monitor - Firmware v");
  Serial.println(_VERSION);
  Serial.println ("> Setting up Wifi");

  while (!attemptWifiConnection()) {
    setupMode();
    //Loop until wifi gets configured and ESP reset
    while(!wifiSetOk) {
      server.handleClient();
      if (millis() > currentStatus.statusLedTimer) {
        currentStatus.statusLedTimer = millis() + ms.statusLedTimerDelay/4;
        digitalWrite(LED_YEL_PIN,!digitalRead(LED_YEL_PIN));
      }
    }
    //L'idée ici est de fermer le serveur utilisé pour configurer le Wifi en prévision de le repartir à nouveau avec config différente
    server.close(); // Yh-28/07/23: non-testé
    server.stop();  // Yh-28/07/23: non-testé
  }

  Serial.println("");
  Serial.print("> WiFi connected, ");  
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("> Setting up NTP");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.print("> Time is: ");
  Serial.println(getNow());  //Attention, bloquant...
  currentStatus.startTime = getNow();

  Serial.println("> Setting webserver routing");
  restServerRouting();
  server.onNotFound(handleNotFound);

  httpUpdater.setup(&server,update_path,update_username,update_password);
  server.begin();
  Serial.println("> HTTP server started");

  // Activate mDNS this is used to be able to connect to the server
  // with local DNS hostmane esp8266.local
  Serial.println("> Setting up MDNS");
  if (MDNS.begin(ms.devName)) {
    MDNS.addService("http", "tcp", webServicePort);
    Serial.println("MDNS responder started");
  }

  if (ms.logMQTTMode>0) {
    Serial.println("> Setting up MQTT");
    if (initMQTTConnection())
      if (checkConnectMQTT()) currentStatus.MQTTConnected=true;
      else currentStatus.MQTTConnected=false;
  }

  if (ms.tbMode) {
    initTBConnection();
  }

  //Find device 1wire addresses, print addresses, set resolution
  Serial.println("> Setting up DS18B20");

//30 juillet 2023 yh: à faire: ajouter protection contre un doublon d'adresse...
// Re-vérifier le tempSensorDiscov car indique trjs "false" meme si on a les 3 températures...
  for (uint8_t i=0; i<numTempDev; i++) {
    if (sensors.getAddress(thermalDev[i], i)) {
      Serial.printf("Therm device %i Address: ",i);
      printAddress(thermalDev[i]);
      Serial.println();
      sensors.setResolution(thermalDev[i], TEMPERATURE_PRECISION);
      currentStatus.tempSensorDiscov[i]=true;
    } else
      Serial.printf("Unable to find address for Device %i\n",i);
      currentStatus.tempSensorDiscov[i]=false;
  }

  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures();

  Serial.println("> Final...");

  for (uint8_t indx=0;indx<numProbe;indx++) {
    currentStatus.activiteCourante[indx]=-1; //0=OFF, 1=ON, -1=non-initialisée
    currentStatus.activitePrecedente[indx]=-1;
    currentStatus.activityStartTime[indx]=getNow();
  }
  Serial.println("> Ready!");

}

void loop(void){
  server.handleClient();

  if (ms.logMQTTMode>0) {
      if (checkConnectMQTT()) {
        mqttClient.loop();
        currentStatus.MQTTConnected=true;
      } 
      else currentStatus.MQTTConnected=false;  
  }

  if (ms.tbMode) {
    if (initTBConnection()) {
      tb.loop();
    } else {
      currentStatus.tbConnectFailed++;
    }
  }
  
  if (millis() > currentStatus.statusLedTimer) {
    currentStatus.statusLedTimer = millis() + ms.statusLedTimerDelay/2;
    digitalWrite(LED_YEL_PIN,!digitalRead(LED_YEL_PIN));
  }//Minuterie du DEL status

  if (millis() > currentStatus.wldTimer) {
    currentStatus.wldTimer = millis() + ms.wldTimerDelay;
    // -- traitement des WLD
    for (uint8_t i=0; i<numWLD;i++) {
      bool status = checkWLD(i);
      if (status) {
        if (currentStatus.WLDCurrentStatus[i]) {
          if (currentStatus.WLDCurrentStatus[i] != currentStatus.WLDLastStatus[i]) {
            currentStatus.WLDAlarmTrigCntr[i]++;
            Serial.printf(" WLD 0%d alarm ACTIVE\n",i+1);
          }
          
        } else {
          
          if (currentStatus.WLDCurrentStatus[i] != currentStatus.WLDLastStatus[i])
            Serial.printf(" WLD 0%d back to normal\n",i);
        }
        currentStatus.WLDLastStatus[i] = currentStatus.WLDCurrentStatus[i];
      } //bad status?
    }
    if (currentStatus.WLDCurrentStatus[0] || currentStatus.WLDCurrentStatus[1])
      digitalWrite(LED_RED_PIN,HIGH);
    else
      digitalWrite(LED_RED_PIN,LOW);
  }//Minuterie du wld

  //request temp every (getTempTimerDelay) msec
  if (currentStatus.getTempTimer < millis()) {
    currentStatus.getTempTimer = millis() + ms.getTempTimerDelay;

    // Traitement de la lecture des sondes de courant:
    for (uint8_t probeIndx=0;probeIndx<numProbe;probeIndx++) {
      uint8_t currentReadStatus = goReadCurrentSensor(probeIndx);
      if (ms.dbgReadCurrent) {
        Serial.printf("\tI reading(%d): %.2f\n",probeIndx,currentStatus.currentIValue[probeIndx]);
      }
    }

    uint8_t tempReadStatus = goReadTemperatureSensors();

    if (ms.dbgReadTime) {
      Serial.print("DT: ");
      printLocalTime();
    }

    if (ms.dbgTinyStatus) {
      Serial.print("\tStatus:");
      Serial.print(getCmdFromTiny('i'));
    }

    Serial.println();

    // Traitements après les lectures

    //Signer notre passage à MQTT dans une routine de traitement:
    if (ms.logMQTTMode && currentStatus.MQTTConnected) {
       if (WiFi.status()== WL_CONNECTED) {
          uint8_t num = (uint8_t)(ESP.getChipId() & 0x000000FF);
          String devID = "esphwt_"+String(num,HEX);
          currentStatus.MQTTLastMsgSuccess = mqttClient.publish((String(ms.basetopic)+"/"+devID+"/tmstmp").c_str(),((String(getNow())).c_str())); //Topic name
          // Envoyer à MQTT le data requis:
          uint8_t resultMQTT = pushDataToMQTT();
       } else {
         currentStatus.MQTTLastMsgSuccess = false;
       }
    } else {
      currentStatus.MQTTLastMsgSuccess = false;
    }

    // Traiter ici l'envoi à Thingsboard:
    if (dbgTBStatus) Serial.print("> Sending to TB: ");
    int8_t resultTB = pushDataToTB();
    if (dbgTBStatus) Serial.println(resultTB);

  } // Toutes les (getTempTimerDelay) milisecondes  
  MDNS.update();
}


// Exemple pour POSTer (avec curl sur Windows 11):
/*
c:\Users\yheyn>curl -vvv -H "Content-Type: application/json" -d "{'dbgserial':1}" http://webesp8266.local/settings
*   Trying 192.168.121.145:80...
* Connected to webesp8266.local (192.168.121.145) port 80 (#0)
> POST /settings HTTP/1.1
> Host: webesp8266.local
> User-Agent: curl/8.0.1
> Accept: *//*
> Content-Type: application/json
> Content-Length: 15
>
< HTTP/1.1 201 Created
< Content-Type: application/json
< Access-Control-Allow-Origin: *
< Access-Control-Max-Age: 600
< Access-Control-Allow-Methods: PUT,POST,GET,OPTIONS
< Access-Control-Allow-Headers: *
< Content-Length: 39
< Content-Length: 39
< Connection: keep-alive
< Keep-Alive: timeout=2000
<
{"dbgserial":"success","nbElement":"1"}* Connection #0 to host webesp8266.local left intact

c:\Users\yheyn>
*/