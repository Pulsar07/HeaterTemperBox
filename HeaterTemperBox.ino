#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <PubSubClient.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// private settings
#include "settings.h"

const char* VERSION = "V0.12";
/*
Version History
 V0.12  29.08.2023: RS : enhanced fix for problems at dts sensor read,  minor log fixes 
 V0.11  12.02.2023: RS : added <switch un/lock> command and ignoring recurring commands, enhanced logMsges
 V0.10  22.01.2023: RS : added <switch reset> command, to overcome a pending non-VALID state
 V0.9  18.01.2023: RS : PWN value of the thyristor module is now limited to 250, at value 255 the module switched power off ;-(
 V0.8  05.01.2023: RS : supports now changes of settings while started process
 V0.7  28.07.2022: RS : more enhanced switch off/on
 V0.6  26.05.2022: RS : faster switch off temperature at cool down
 V0.5  11.04.2022: RS : new less input values only: target temperature, target temp duration, temperature gradient
                        heatup and cooldown durations are calculated based on target temperature and gradient settings
                        new/simpler power control management
 V0.4  05.02.2022: RS : added head periods
 V0.3  21.12.2021: RS : mqtt logging (log_level) added
                        heat booster command implemented, to manipulate power heat_booster = 10 is default
                        cool down mechanism to avoid cooling down to fast
                        mqtt LWT added
 V0.2  15.12.2021: RS : working beta version, with temperature gradient reduction ~10°C/h
 V0.1  06.11.2021: RS : inital version
*/

/**
 * \file HeaterTemperBox.ino
 *
 * \brief temperature controller to be used for epoxy resin temper applications
 *
 * \author Author: Rainer Stransky
 *
 * \copyright This project is released under the GNU Public License v3
 *          see https://www.gnu.org/licenses/gpl.html.
 * Contact: opensource@so-fa.de
 *
 */

/**
 * \mainpage this is a controller for a temperature regulatated heating system, especially for tempering composite material
  * \section intro Features
  * measuring of temperature using two Dallas DS18B20 temperature sensors
  ** one near temperature (near to the heating plate)
  ** one far temperature (far to the heating plate)
  * control of temperature and air flow 
  ** via a 4-Channel Relay Module  
  ** the power regulation of the heating plate is done with a thyristor module, 
      to avoid damaging the relais switching to often
  ** temperature gradient is a preset (4°C/h) but modifiable parameter to avoid temperature friction to the temper part
  ** air flow    : fan (120mm, 220V)
  ** temperature : heating plates (220V 300W-500W)
  * MQTT client for integration into smart home environment
  * OTA flashing with esp_tool
    > espota.py -d -a <password>  -i <ipaddress of ESP> -f <build bin file>
*/


#define DEF_MAX_TEMP_DIFF    9.0f
#define DEF_MAX_SENSOR_TEMP 57.0f
#define DEF_ILLEGAL_TEMP    65.0f
#define DEF_MAX_SETUP_TEMP  55.0f

const char* APPLICATION = "TemperAtureRegulator";
const char* DESCRIPTION = "regulates temperature and airflow for tempering of shell and mold parts";
const char* CMDDESCRIPTION = "switch [on,off,reset,lock,unlock], target_temp [30-60]C, heat_duration [1-95]h, temperature_gradient [1-10], log_level [1=DEBUG|2=INFO|3=WARN|4=ERROR]";
const char* BUILD_DATE = "build at: "  __DATE__ " " __TIME__;
const char* SSID = DEF_SSID;
const char* PSK = DEF_PSK;
const char* MQTT_BROKER = DEF_MQTT_BROKER;
const char* MQTT_USER = DEF_MQTT_USER;
const char* MQTT_PASSWD = DEF_MQTT_PASSWD;
const uint16_t MQTT_BROKER_PORT = DEF_MQTT_BROKER_PORT;

const char* MQTT_LWT_TOPIC = "/stat/%s/LWT";
#define BUF_SIZE 200

float ourDebugVal;
unsigned long ourSecond = -1;
boolean ourStateChanged = false;
#define MQTT_MYSELF_SIZE 28
char MQTT_MYSELF[MQTT_MYSELF_SIZE];
enum LogSeverity {
  LS_START=0,
  DEBUG,
  INFO,
  WARNING,
  ERROR,
  LS_END
};

LogSeverity ourLogSeverity=INFO;

enum InternalState {
  VALID=0,
  ERROR_TEMP_SENSOR,
  ERROR_TEMP_SENSORS_INCONSISTENT,
  ERROR_TEMP_TO_HIGH,
  INVALID_STATE
};

enum HeatControl {
  HEATER_OFF, HEATER_ON, SAFETY_SHUTDOWN
};


// some forward declarations
void setHeatDevices(HeatControl aControl,int aHeatPowerPercentage=0, InternalState aErrorValue=VALID);
void logMsg(LogSeverity aSeverity, String aMessage);
void log_printSecond();

float getTempNear();

#define HEATING_PERIOD_SIZE 10
#define NO_PERIOD_IN_USE -1
class TemperatureController {
  public:
    // int myThrottle;
    // boolean myTgetTempReached;
    // unsigned long myLastTime;
    // float myLastTemp;
    void init();
    void consumeMinute();
    void setTargetTemperature(int aTemperature);
    float getTargetTemperature();
    void setTemperatureGradient(uint8_t aGradient);
    void setTemperDuration(int aDuration);
    void setHeatupDuration(int aDuration);
    int setCooldownDuration(int aDuration);
    int getTemperTimer();
    int getHeatupTimer();
    int getCooldownTimer();
    float getCalculatedTemperature();
    uint8_t getTemperatureGradient();
    void start();
    void stop();
    void lock();
    void unlock();
    boolean isActive();
    boolean isStarted();
    boolean isLocked();
  private:
    static const uint8_t  ourBaseTemp = 20;
    uint8_t myTemperatureGradient;
    int myTemperDuration;
    int myHeatupDuration;
    int myCooldownDuration;
    int myTimer;
    float myStartTemp;
    uint8_t myTargetTemperAture;
    uint8_t myTargetTemperature;
    void calculateDurations();
    boolean myIsStarted;
    boolean myLockState;
};


void TemperatureController::init() {
  myTargetTemperAture = 0;
  myTemperDuration = 0;
  myHeatupDuration = 0;
  myCooldownDuration = 0;
  myTimer = 0;
  myTemperatureGradient = 4;
  myIsStarted = false;
  myLockState = true;
}

boolean TemperatureController::isLocked() {
  return myLockState;
}

boolean TemperatureController::isStarted() {
  return myIsStarted;
}

void TemperatureController::lock() {
  if (myLockState) {
    // do nothing, but a log
    logMsg(WARNING, String("switch lock state is already set"));
    return;
  }
  myLockState = true;
  ourStateChanged = true;
  logMsg(INFO, String("lock controller")); 
}

void TemperatureController::unlock() {
  if (!myLockState) {
    // do nothing, but a log
    logMsg(WARNING, String("switch unlock state is already set"));
    return;
  }
  myLockState = false;
  ourStateChanged = true;
  logMsg(INFO, String("unlock controller")); 
}

void TemperatureController::start() {
  if (myLockState) {
    // do nothing, but a log
    logMsg(WARNING, String("system is locked, unlock first"));
    return;
  }
  if (myIsStarted) {
    // do nothing, but a log
    logMsg(WARNING, String("switch on state is already set"));
    return;
  }
  myIsStarted = true;
  ourStateChanged = true;
  this->calculateDurations();
  logMsg(INFO, String("start temper process ")); 
}

void TemperatureController::stop() {
  if (myLockState) {
    // do nothing, but a log
    logMsg(WARNING, String("system is locked, unlock first"));
    return;
  }
  setHeatDevices(HEATER_OFF, 0);
  myIsStarted = false;
  ourStateChanged = true;
  logMsg(INFO, String("stop temper process ")); 
}


void TemperatureController::setTemperatureGradient(uint8_t aGradient) {
  if (myLockState) {
    // do nothing, but a log
    logMsg(WARNING, String("system is locked, unlock first"));
    return;
  }
  if (aGradient == myTemperatureGradient) {
    // do nothing, but a log
    logMsg(WARNING, String("temperature_gradient already set: ") + aGradient);
    return;
  }
  if (aGradient > 0 && aGradient <= 10 ) {  
     myTemperatureGradient = aGradient;
     this->calculateDurations();
     logMsg(INFO, String("set temperature gradient : ")+ aGradient); 
   } else {
     logMsg(ERROR, String("invalid temperature gradient : ")+ aGradient); 
     this->stop();
   }
}

/*
  set temper durations in minutes
*/
void TemperatureController::setTemperDuration(int aDuration) {
  if (myLockState) {
    // do nothing, but a log
    logMsg(WARNING, String("system is locked, unlock first"));
    return;
  }
  if (aDuration == myTemperDuration) {
    // do nothing, but a log
    logMsg(WARNING, String("heat_duration already set: ") + aDuration);
    return;
  }
  if (aDuration > 0 && aDuration < 96*60) {
    myTemperDuration = aDuration;
    this->calculateDurations();
  } else if (aDuration == 0) {
    myTemperDuration = 1;
    this->calculateDurations();
  } else {
    logMsg(ERROR, String("invalid temper duration: ") + aDuration);
    this->stop();
  }
}

int TemperatureController::getHeatupTimer() {
  int dura = myTimer - myTemperDuration - myCooldownDuration;
  if (dura > 0) {
    return dura;
  } else {
    return 0;
  }
}

int TemperatureController::getTemperTimer() {
  int dura = myTimer - myCooldownDuration;
  if (dura >= myTemperDuration) {
    return myTemperDuration;
  } else if (dura > 0) {
    return dura;
  } else {
    return 0;
  }
}

int TemperatureController::getCooldownTimer() {
  if (myTimer >= myCooldownDuration) {
    return myCooldownDuration;
  } else {
    return myTimer;
  }
}

void TemperatureController::setTargetTemperature(int aTemperature) {
  if (myLockState) {
    // do nothing, but a log
    logMsg(WARNING, String("system is locked, unlock first"));
    return;
  }
  if (aTemperature == myTargetTemperAture) {
    // do nothing, but a log
    logMsg(WARNING, String("target_temp already set: ") + aTemperature);
    return;
  }
  if (aTemperature >= 0 && aTemperature <= DEF_MAX_SETUP_TEMP) {
    myTargetTemperAture = aTemperature;
    this->calculateDurations();
  } else {
    logMsg(ERROR, String("invalid set temperature: ") + aTemperature);
    this->stop();
  }
}

float TemperatureController::getTargetTemperature() {
  return myTargetTemperAture;
}

void TemperatureController::calculateDurations() {
  // depending on the temper and current temperature and the gradient, 
  // the duration for heatup and cooldown is calculated
  float currentTemp = getTempNear();
  int currentTemperTimer = getTemperTimer();
  if (myTargetTemperAture > 0) { 
    myHeatupDuration = max(0.0f, 60.0f * (float) (myTargetTemperAture - currentTemp) / myTemperatureGradient); 
    myCooldownDuration = max(0.0f, 60.0f * (float) (myTargetTemperAture - ourBaseTemp) / myTemperatureGradient); 
  }

  if (myTemperDuration > 0) { 
    if (!myIsStarted ) {
      // if values are changed while a temper process is started, take 
      currentTemperTimer = myTemperDuration;
    } 
    myTimer = myHeatupDuration + currentTemperTimer + myCooldownDuration;
  }

  if (myTargetTemperAture > 0 && myTimer > 0 ) {
    myStartTemp = min(DEF_MAX_SETUP_TEMP, currentTemp);
  }
  logMsg(INFO, String("new settings S:T:D:G:") + String(myIsStarted) + ":" + String(myTargetTemperAture, 2) + ":" + String(myTemperDuration) + ":" + String(myTemperatureGradient));
}

/*
 * this function will provide the current target temperature for the actual time. In the heatup phase the temperature will be calculated depending on the current time, heatup duration and the gradient, ....
*/
float TemperatureController::getCalculatedTemperature() {
  // linear functions
  // T(t) = a * t + b
  // 
  float retVal = 0.0f;
  if (myIsStarted) {
    // heatup phase
    if (getHeatupTimer() > 0 && myTargetTemperAture > 0) {
      retVal = (float) (myTargetTemperAture-myStartTemp) * (myHeatupDuration - getHeatupTimer()) / myHeatupDuration + myStartTemp; 
    } else 
    // temper phase
    if (getTemperTimer() > 0 && myTargetTemperAture > 0) {
      retVal = myTargetTemperAture;
    } else 
    // cooldown phase
    if (getCooldownTimer() > 0 && myTargetTemperAture > 0) {
      retVal = (float) (myTargetTemperAture-ourBaseTemp) * -(myCooldownDuration - getCooldownTimer()) / myCooldownDuration + myTargetTemperAture; 
    }
  }
  return retVal;
}

uint8_t TemperatureController::getTemperatureGradient() {
  return myTemperatureGradient;
}

boolean TemperatureController::isActive() {
  if (myTargetTemperAture > 0 && myTimer > 0 && myIsStarted == true) {
    return true;
  }
  return false;
}

void TemperatureController::consumeMinute() {
  if (myTimer > 0) {
    myTimer--;
  }
}


TemperatureController ourTempCtrler;
InternalState ourInternalState = VALID;
int ourPwmHeatingPowerValue = 0;
int ourHeatPower = 0;
// int ourHeatDurationMinutes = 0;
float ourTempNear = 0.0f;
float ourTempFar = 0.0f;
String ourLogMessage = "";

 
WiFiClient ourEspClient;
PubSubClient ourMqttClient(ourEspClient);

/* Pin Layout on a Wemos D1 
Reset = | RST  :    TX | = 1
   A0 = | A0   :    RX | = 3
   16 = | D0   :    D1 | = 5
   14 = | D5   :    D2 | = 4
   12 = | D6   :    D3 | = 0
   13 = | D7   :    D4 | = 2
   15 = | D8   :     G | = GND
 3.3V = | 3V3  :    5V | = 5V in/out
*/

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2 // D4=2
#define TEMPERATURE_PRECISION 11 // 9,10,11,12 where 12 is the most precise one

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire ourOneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature ourDallasTempSensors(&ourOneWire);

// arrays to hold device addresses

struct TemperatureSensor {
  // arrays to hold device addresses
  DeviceAddress address;
  String name;
  float temperature;
  float temperature_last=-100.0f;
  uint8_t faultnum=0;
};

TemperatureSensor ourTemperatureSensorNear = { {}, "NearTempSensor", 0.0f};
TemperatureSensor ourTemperatureSensorFar = { {}, "NearTempSensor", 0.0f};

// 220V Releais on D0, D5, D6, D7

#define RELAY_1_PIN 13  // D7=13
#define RELAY_2_PIN 12  // D6=12
#define RELAY_3_PIN 14  // D5=14
#define RELAY_4_PIN 16  // D0=16
#define PWM_PIN      5  // D1=5

enum SwitchState { OFF, ON, INIT };

struct RelayChannel {
  uint8_t pin;
  String name;
  SwitchState state;
};

RelayChannel ourRelayPower = {RELAY_1_PIN, "power", INIT};
RelayChannel ourRelayFan   = {RELAY_2_PIN, "fan", INIT};
RelayChannel ourRelayPowerController = {RELAY_3_PIN, "pwm_heatcontroller", INIT};
RelayChannel ourRelayUnused = {RELAY_4_PIN, "unused", INIT};


void setup_ota() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname(String("esp8266" + WiFi.macAddress()).c_str());

  // No authentication by default
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.print("ArduinoOTA: setup ready");
  Serial.print("  IP address: ");
  Serial.println(WiFi.localIP());
}


void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.println("Networking :");
    Serial.print("  MAC address :");
    Serial.println(WiFi.macAddress());
    Serial.print("  connecting to ");
    Serial.println(SSID);
 
    WiFi.begin(SSID, PSK);
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
 
    Serial.println("");
    Serial.println("  WiFi connected");
    Serial.print("  IP address: ");
    Serial.println(WiFi.localIP());
}

void check_wifi() {
  // checking for WIFI connection
  uint8_t retry=25;
  if ((WiFi.status() != WL_CONNECTED)) {
    retry--;
    if (!retry) {
      log_printSecond();
      Serial.println("restarting due to WIFI problems");
      ESP.restart();
    }
    log_printSecond();
    Serial.println("Reconnecting to WIFI network");
    WiFi.reconnect();
    delay(500);
  }
}
 
void mqtt_reconnect() {
  while (!ourMqttClient.connected()) {
    String uid = WiFi.macAddress();
    uid.replace(":","");
    String mqtt_id = String(APPLICATION) + "_" + uid;
    log_printSecond();
    Serial.println(mqtt_id + ": MQTT reconnecting...");
    
    mqtt_subscribe(mqtt_id.c_str());
  }
}

/**
subscribe mqtt topics to receive commands and arguments
*/
void mqtt_subscribe(const char* aMqttId) {
  if (ourMqttClient.connect(aMqttId, MQTT_USER, MQTT_PASSWD, MQTT_LWT_TOPIC, 0, 0, "offline", true)) {
    char mqttTopic[BUF_SIZE];
    log_printSecond();
    Serial.println("  topic subscription:");

    // // ... and resubscribe
    snprintf (mqttTopic, BUF_SIZE, "cmnd/%s/switch", MQTT_MYSELF);
    Serial.println("  subscribing for: " +String(mqttTopic));
    ourMqttClient.subscribe(mqttTopic);
    snprintf (mqttTopic, BUF_SIZE, "cmnd/%s/target_temp", MQTT_MYSELF);
    Serial.println("  subscribing for: " +String(mqttTopic));
    ourMqttClient.subscribe(mqttTopic);
    snprintf (mqttTopic, BUF_SIZE, "cmnd/%s/heat_duration", MQTT_MYSELF);
    Serial.println("  subscribing for: " +String(mqttTopic));
    ourMqttClient.subscribe(mqttTopic);
    snprintf (mqttTopic, BUF_SIZE, "cmnd/%s/temperature_gradient", MQTT_MYSELF);
    Serial.println("  subscribing for: " +String(mqttTopic));
    ourMqttClient.subscribe(mqttTopic);
    snprintf (mqttTopic, BUF_SIZE, "cmnd/%s/log_level", MQTT_MYSELF);
    Serial.println("  subscribing for: " +String(mqttTopic));
    ourMqttClient.subscribe(mqttTopic);
  } else {
    log_printSecond();
    Serial.print("  failed, rc=");
    Serial.print(ourMqttClient.state());
    Serial.println("  retrying in 5 seconds");
    delay(5000);
  }
}

/**
mqtt_callback is called for subscribed commands in case of receiving mqtt topics
*/
void mqtt_callback(char* aTopic, byte* aPayload, unsigned int aLength) {
  // MQTT message arrived [cmnd/AlarmBell_627E5B/bell] 1
  String theTopic = String(aTopic);
  String thePayload;
  for (int i = 0; i < aLength; i++) {
    thePayload += (char)aPayload[i];
  }

  logMsg(INFO, String("mqtt command received: " + theTopic + "/" + thePayload));
  
  if (theTopic.endsWith("/switch")) {
    if (thePayload.equals("reset")) {
      ESP.restart(); //ESP.reset();
    } else if (thePayload.equals("lock")) {
      ourTempCtrler.lock();
      logMsg(INFO, String("mqtt command received lock: " + theTopic + "/" + thePayload));
    } else if (thePayload.equals("unlock")) {
      logMsg(INFO, String("mqtt command received unlock: " + theTopic + "/" + thePayload));
      ourTempCtrler.unlock();
    } else if (thePayload.equals("on")) {
      ourTempCtrler.start();
      Serial.println(String("switch on : " + theTopic));
    } else {
      ourTempCtrler.stop();
      Serial.println(String("switch off : " + theTopic));
    }
  } else
  if (theTopic.endsWith("/target_temp")) {
    int targetTemp = thePayload.toInt();
    ourTempCtrler.setTargetTemperature(targetTemp);
  } else
  if (theTopic.endsWith("/heat_duration")) {
    int duration = thePayload.toInt(); // duration is given in hours 
    duration *= 60; // duration in minutes
    ourTempCtrler.setTemperDuration(duration);
  } else
  if (theTopic.endsWith("/temperature_gradient")) {
    int gradient = thePayload.toInt(); // gradient in °C per hour
    ourTempCtrler.setTemperatureGradient(gradient);
  } else
  if (theTopic.endsWith("/log_level")) {
    int level = thePayload.toInt();
    if (level > LS_START && level < LS_END ) {
      ourLogSeverity = (LogSeverity) level;
      Serial.println(String("setting log level to : ") + ourLogSeverity);
    } else {
      Serial.println(String("invalid log level value"));
      ourLogSeverity=WARNING;
    }
  }
  // force a fast sending of current values via mqtt, to give the user a quick response
  ourStateChanged = true;
}

// setup the digital temperature sensors
void setup_dts() {

  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  ourDallasTempSensors.begin();

  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(ourDallasTempSensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (ourDallasTempSensors.isParasitePowerMode()) {
   Serial.println("ON");
  }else {
   Serial.println("OFF");
  }

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  if (!ourDallasTempSensors.getAddress(ourTemperatureSensorFar.address, 0)) Serial.println("Unable to find address for Device 0");
  if (!ourDallasTempSensors.getAddress(ourTemperatureSensorNear.address, 1)) Serial.println("Unable to find address for Device 1");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them. It might be a good idea to
  // check the CRC to make sure you didn't get garbage. The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to nearThermometer
  //if (!oneWire.search(nearThermometer)) Serial.println("Unable to find address for nearThermometer");
  // assigns the seconds address found to farThermometer
  //if (!oneWire.search(farThermometer)) Serial.println("Unable to find address for farThermometer");

  // show the addresses we found on the bus
  Serial.print("Dallas Temperature Sensor: name=");
  Serial.print(ourTemperatureSensorNear.name);
  Serial.print("index=0 address=");
  dts_printAddress(ourTemperatureSensorNear.address);
  Serial.println();

  Serial.print("Dallas Temperature Sensor: name=");
  Serial.print(ourTemperatureSensorFar.name);
  Serial.print("index=1 address=");
  dts_printAddress(ourTemperatureSensorFar.address);
  Serial.println();

  // set the resolution to 9 bit per device
  ourDallasTempSensors.setResolution(ourTemperatureSensorNear.address, TEMPERATURE_PRECISION);
  ourDallasTempSensors.setResolution(ourTemperatureSensorFar.address, TEMPERATURE_PRECISION);

  Serial.print("Device 0 Resolution: ");
  Serial.print(ourDallasTempSensors.getResolution(ourTemperatureSensorNear.address), DEC);
  Serial.println();

  Serial.print("Device 1 Resolution: ");
  Serial.print(ourDallasTempSensors.getResolution(ourTemperatureSensorFar.address), DEC);
  Serial.println();
}

// function to print a device address
void dts_printAddress(DeviceAddress aAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aAddress[i] < 16) Serial.print("0");
    Serial.print(aAddress[i], HEX);
  }
}


void setup() {
  setup_relay(); // do this immediately, to ensure switch off all relays by default
  Serial.begin(115200);
  delay(3000);  // do a long startup delay to avoid missing serial printouts in the console
  Serial.println(String(APPLICATION) + " " + String(VERSION));
  Serial.println("  " + String(DESCRIPTION));

  ourTempCtrler.init();
  setup_wifi();
  setup_mqtt();
  setup_ota();
  setup_dts();
  setup_pwm();

  Serial.println("setup successfully finished");
}

// setup of the relay related stuff
void setup_relay() {
  Serial.println("Setting relay pins to OUTPUT");
  pinMode(RELAY_1_PIN,OUTPUT);
  pinMode(RELAY_2_PIN,OUTPUT);
  pinMode(RELAY_3_PIN,OUTPUT);
  pinMode(RELAY_4_PIN,OUTPUT);

  switchRelay(&ourRelayPower, OFF);
  switchRelay(&ourRelayPowerController, OFF);
  switchRelay(&ourRelayFan, OFF);
  switchRelay(&ourRelayUnused, OFF);
}

void setup_pwm() {
  Serial.println("PWM:");
  pinMode(PWM_PIN, OUTPUT);
  // see http://www.intradocserver.wald/cgi-bin/wiki.pl?Arduino/PWM-Controller
  Serial.println("  set pwm frequence to: 10Hz");
  analogWriteFreq(60);
  analogWriteFreq(500);
  ourHeatPower = 0;
  setPWMController(ourHeatPower);
}

void setup_mqtt() {
  Serial.println("MQTT:");
  Serial.println("  Broker set to: "+ String(MQTT_BROKER) + ":" + MQTT_BROKER_PORT);
  ourMqttClient.setServer(MQTT_BROKER, MQTT_BROKER_PORT);

  const char* mac = WiFi.macAddress().c_str();
  snprintf (MQTT_MYSELF, MQTT_MYSELF_SIZE, "%s_%c%c%c%c%c%c", APPLICATION, mac[9], mac[10], mac[12], mac[13], mac[15], mac[16]);
  Serial.print("  MQTT Identification: ");
  Serial.println(MQTT_MYSELF);

  ourMqttClient.setCallback(mqtt_callback);
}

void mqtt_publishData(String aTopic, String aPayload, boolean aRetain=false) {
  char mqttTopic[BUF_SIZE];
  char mqttPayload[BUF_SIZE];
  snprintf (mqttTopic, BUF_SIZE, aTopic.c_str(), MQTT_MYSELF);
  snprintf (mqttPayload, BUF_SIZE, "%s", aPayload.c_str());
  Serial.println("  " +String(mqttTopic) + "=" + mqttPayload);
  ourMqttClient.publish(mqttTopic, mqttPayload, aRetain);
}

String getDurationString(int aDuration) {
  // aDuration is in minutes
  String duraDays = String(aDuration/(60*24));
  String duraHours = String((aDuration/60)%24);
  if (duraHours.length() == 1) {
    duraHours = "0" + duraHours;
  }
  String duraMins = String(aDuration%60);
  if (duraMins.length() == 1) {
    duraMins = "0" + duraMins;
  }
  return duraDays+":"+duraHours+":"+duraMins;
}

void mqtt_sendData() {
  static boolean isStaticMQTTInfoSent=false;
  // send data via mqtt in a cyclical manner
  // MQTT handling
  if (!ourMqttClient.connected()) {
    mqtt_reconnect();
    isStaticMQTTInfoSent=false;
  }

  log_printSecond();
  Serial.println("publish MQTT messages to: "+ String(MQTT_BROKER) + ":" + MQTT_BROKER_PORT);


  if (!isStaticMQTTInfoSent) {
    // send some static info only once with the first call
    isStaticMQTTInfoSent=true;

    if (millis() < 20000) {
      logMsg(INFO, String("startup of HeaterTemperBox ") + VERSION + " Controller successfull");
    }
    mqtt_publishData("/stat/%s/application", APPLICATION);
    mqtt_publishData("/stat/%s/description", DESCRIPTION);
    mqtt_publishData("/stat/%s/version", VERSION);
    mqtt_publishData("/stat/%s/ipaddress", WiFi.localIP().toString());
    mqtt_publishData("/stat/%s/macaddress", WiFi.macAddress());
    mqtt_publishData("/stat/%s/cmddescription", CMDDESCRIPTION);
    mqtt_publishData("/stat/%s/build", BUILD_DATE);
    mqtt_publishData(MQTT_LWT_TOPIC, "online", true);
  }


  // if there is no temper in action, send dynamic data in a 1h period
  boolean doSend=true;
  if (!ourTempCtrler.isActive() && ourTempNear < 25.0f && ourTempFar < 25.0f) {
    doSend = false;
    static unsigned long last = 0;
    if ( (millis() - last) > 60 * 60 * 1000) {
      doSend = true;
      last = millis();
    }
  }

  if (doSend) {
    // this is the volatile part of the data to be send only in case of a active temper session in progress
    //
    mqtt_publishData("/tele/%s/wifi/rssi", String(WiFi.RSSI()));
    mqtt_publishData("/tele/%s/temperature/temp_near", String(ourTempNear, 2));
    mqtt_publishData("/tele/%s/temperature/temp_far", String(ourTempFar, 2));
  }
 
  static String valuesLast = "";
  String stringValue = 
    String(ourRelayPower.state)
    + String(ourRelayFan.state)
    + String(ourRelayPowerController.state)
    + String(ourHeatPower)
    + String(ourPwmHeatingPowerValue)
    + String(ourTempCtrler.getTargetTemperature(), 2)
    + String(ourTempCtrler.getTemperatureGradient())
    + getDurationString(
         ourTempCtrler.getHeatupTimer()+ourTempCtrler.getTemperTimer()+ourTempCtrler.getCooldownTimer())
    + String(ourTempCtrler.getCalculatedTemperature())
    + String(ourTempCtrler.isStarted())
    + String(ourTempCtrler.isLocked())
    + String(ourInternalState)
    + String(ourDebugVal)
    ;
  
  if (!stringValue.equals(valuesLast)) {
    mqtt_publishData("/tele/%s/relay/" + ourRelayPower.name, String(ourRelayPower.state));
    mqtt_publishData("/tele/%s/relay/" + ourRelayFan.name, String(ourRelayFan.state));
    mqtt_publishData("/tele/%s/relay/" + ourRelayPowerController.name, String(ourRelayPowerController.state));
    mqtt_publishData("/tele/%s/heatpower", String(ourHeatPower));
    mqtt_publishData("/tele/%s/heatpwmctrl", String(ourPwmHeatingPowerValue));
    mqtt_publishData("/tele/%s/temper", String(ourTempCtrler.getTargetTemperature(), 2));
    mqtt_publishData("/tele/%s/gradient", String(ourTempCtrler.getTemperatureGradient()));
    mqtt_publishData("/tele/%s/duration_all", getDurationString(
       ourTempCtrler.getHeatupTimer()+ourTempCtrler.getTemperTimer()+ourTempCtrler.getCooldownTimer()));
    mqtt_publishData("/tele/%s/duration_temper", getDurationString(ourTempCtrler.getTemperTimer()));
    mqtt_publishData("/tele/%s/duration_heatup", getDurationString(ourTempCtrler.getHeatupTimer()));
    mqtt_publishData("/tele/%s/duration_cooldown", getDurationString(ourTempCtrler.getCooldownTimer()));
    mqtt_publishData("/tele/%s/target", String(ourTempCtrler.getCalculatedTemperature()));
    mqtt_publishData("/tele/%s/isstarted", String(ourTempCtrler.isStarted()));
    mqtt_publishData("/tele/%s/islocked", String(ourTempCtrler.isLocked()));
    mqtt_publishData("/tele/%s/internalstate", String(ourInternalState));
    mqtt_publishData("/tele/%s/debug", String(ourDebugVal, 0));
    valuesLast = stringValue;
  }
  
  ourStateChanged = false;
    
}

void setPWMController(int aPowerFactor) {
  if (aPowerFactor < 0  | aPowerFactor > 100) {
    ourPwmHeatingPowerValue = 0;
  } else {
    ourPwmHeatingPowerValue = map(aPowerFactor, 0,100,0,250);
  }
  analogWrite(PWM_PIN, ourPwmHeatingPowerValue);
}

void setState(InternalState aState, String aReason="") {
  // do not set internal state back to VALID
  if (ourInternalState != VALID) {
    return;
  }
  String msg = String(aState) + " reason=" + aReason;
  logMsg(ERROR, "setting state to: " + msg);
  ourInternalState = aState;
}

void setHeatDevices(HeatControl aControl,int aHeatPowerPercentage, InternalState aErrorValue) {
  ourHeatPower = aHeatPowerPercentage;
  if (aControl == SAFETY_SHUTDOWN) {
    setState(aErrorValue);
    // switch down the heater power controller
    setPWMController(0);
    // switch off the fan
    switchRelay(&ourRelayFan, OFF);
    // switch off the power controller
    switchRelay(&ourRelayPowerController, OFF);
    // switch off the overall power 
    switchRelay(&ourRelayPower, OFF);
  } else if (aControl == HEATER_OFF) {
    setPWMController(0);
    // switch off the power controller
    switchRelay(&ourRelayPowerController, OFF);
    const float FAN_STOP_TEMP = 30.0f;
    if (  ourTempNear < FAN_STOP_TEMP ) {
      // switch off the fan
      switchRelay(&ourRelayFan, OFF);
      // switch off 220V power 
      switchRelay(&ourRelayPower, OFF);
    } else if (  ourTempNear > (FAN_STOP_TEMP+0.5f) ) {
      // switch off 220V power 
      switchRelay(&ourRelayPower, ON);
      // switch off the fan
      switchRelay(&ourRelayFan, ON);
    }
  } else if (aControl == HEATER_ON) {
    // switch on 220V power 
    switchRelay(&ourRelayPower, ON);
    // switch on the fan
    switchRelay(&ourRelayFan, ON);
    // switch off the power controller
    switchRelay(&ourRelayPowerController, ON);
    setPWMController(aHeatPowerPercentage);
  }
}

float getTempNear() {
  return  getTemp(&ourTemperatureSensorNear);
}

float getTemp(TemperatureSensor* aSensor) {
  float newTemp = ourDallasTempSensors.getTempC(aSensor->address);
  if (aSensor->temperature_last == -100.0f) {
    // first time
    aSensor->temperature_last = newTemp;
  }

  if (newTemp == DEVICE_DISCONNECTED_C) { // = -127.0
    aSensor->faultnum++;
  } else {
    aSensor->faultnum = 0;
  }

  if (aSensor->faultnum > 10) {
    // in case of continuing  error, set system to safe state
    logMsg(ERROR, "temperature sensor " + aSensor->name + " with continuing problem: " + aSensor->faultnum);
    setState(ERROR_TEMP_SENSOR, "sensor error: not connected");
  } else
  if (aSensor->faultnum > 0) {
    // in case of temporary error, return the last value
    newTemp = aSensor->temperature_last;
    logMsg(WARNING, "temperature sensor " + aSensor->name + " with temporary problem: " + aSensor->faultnum);
  }

  if (abs(newTemp - aSensor->temperature_last) >= 3.0f) {
    // in case of inplausible value hops, set system to safe state
    logMsg(ERROR, "temperature sensor value vaults");
    setState(ERROR_TEMP_SENSOR, String(newTemp)+"/"+String(aSensor->temperature_last));
  }
  aSensor->temperature_last = newTemp;
  return newTemp;
}

void  controlTemperature() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if ( (now - last) < 1000) {
    return;
  }
  last = now;

  // do this first
  ourTempNear = getTemp(&ourTemperatureSensorNear);
  ourTempFar = getTemp(&ourTemperatureSensorFar);

  // safety instruction, do nothing if the internal state is not valid
  if (ourInternalState != VALID) {
    logMsg(ERROR, String("invalid internal state: ") + String(ourInternalState));
    setHeatDevices(SAFETY_SHUTDOWN, 0, INVALID_STATE);
    ourTempCtrler.stop();
    return;
  }
  // some more safety checks
  if (abs(ourTempNear - ourTempFar) > DEF_MAX_TEMP_DIFF) {
    // switch down the heater power controller
    setHeatDevices(SAFETY_SHUTDOWN, 0, ERROR_TEMP_SENSORS_INCONSISTENT);
    logMsg(ERROR, "inconsistent temperature sensor drift");
    ourTempCtrler.stop();
    return;
  }
  if (ourTempNear > DEF_ILLEGAL_TEMP
      || ourTempFar > DEF_ILLEGAL_TEMP ) {
    // switch down all relais
    setHeatDevices(SAFETY_SHUTDOWN, 0, ERROR_TEMP_TO_HIGH);
    ourTempCtrler.stop();
    logMsg(ERROR, "inconsistent temperature sensors");
    return;
  }
  if (ourTempNear > DEF_MAX_SENSOR_TEMP
      || ourTempFar > DEF_MAX_SENSOR_TEMP
      || ourTempCtrler.getTargetTemperature() > DEF_MAX_SETUP_TEMP
     ) {
    // switch down the heater power controller
    setHeatDevices(HEATER_OFF, 0);
    ourTempCtrler.stop();
    logMsg(ERROR, "temperature too high, set heating off");
    return;
  }

  float currentTargetTemp = ourTempCtrler.getCalculatedTemperature();
  if (currentTargetTemp == 0.0f || !ourTempCtrler.isActive()) {
    // no temperature or duration set, do nothing
    setHeatDevices(HEATER_OFF, 0);
    return;
  }
  if ( ourTempCtrler.isActive() 
           && ( 
                ourTempNear > ourTempCtrler.getCalculatedTemperature() + 2
                || ourTempFar > ourTempCtrler.getCalculatedTemperature() + 2
              )
     ) {
    // switch off the heater power controller and wait till temp gets down
    setHeatDevices(HEATER_OFF, 0);
    logMsg(WARNING, "measured temperature higher than requested");
    return;
  }
  
  static int heatPower = 30;
  static int timeLow=0;
  static int timeHigh=0;
  static int powerBalance=0;


  // if (abs(powerBalance) > 20) {
  //   heatPower = heatPower+ powerBalance/20;
  // }

  if (powerBalance > 20) {
    heatPower +=1;
  }
  if (powerBalance < -20) {
    heatPower -=1;
  }
  
  // limit heatPower to 20-100%
  heatPower = max(20, heatPower);
  heatPower = min(100, heatPower);

  int power = max(0, heatPower);
  if (ourTempNear < currentTargetTemp) {
    // temp is to Low
    powerBalance = max(-20, powerBalance);
    powerBalance = min(30, powerBalance+1);
  } else {
    // temp is to High
    powerBalance = min(20, powerBalance);
    powerBalance = max(-30, powerBalance-1);
    power = 0;
  }
  logMsg(DEBUG, String("power/powerBalance: ")+ power + "/" + powerBalance); 
  ourDebugVal = powerBalance;

  setHeatDevices(HEATER_ON, power);

}

// relay are used in a manner, that they off/open if there is no signal/voltage 
//   and the leds will light, if the relay is on/closed 
// for this behaviour we use these #defines: 
#define RELAY_CLOSED LOW
#define RELAY_OPEN HIGH
void switchRelay(RelayChannel* aPtrRelayChannel, SwitchState aState) {
  if ( aState == aPtrRelayChannel->state ) {
    // nothing to do state already set
    return;
  }
  
  aPtrRelayChannel->state = aState;
  bool digitalPinState = RELAY_OPEN;
  if (ON == aPtrRelayChannel->state) {
    digitalPinState = RELAY_CLOSED;
  } else {
    digitalPinState = RELAY_OPEN;
  }
  logMsg(DEBUG, "setting relay " +aPtrRelayChannel->name + " at pin " + aPtrRelayChannel->pin + " to state: " + aPtrRelayChannel->state );
  digitalWrite(aPtrRelayChannel->pin, digitalPinState);
}

void loop() {
  static unsigned long last = 0;

  ourMqttClient.loop();
  ArduinoOTA.handle();

  if ( (millis() - last) > 1000) {
    ourSecond++;
    last = millis();

    check_wifi();
  } else {
    return;
  }

  // everything here is done once per second
  ourDallasTempSensors.requestTemperatures();

  if ( ourSecond%60 == 0) {
    ourTempCtrler.consumeMinute();
  }

  if ( ourSecond%10 == 0 || ourStateChanged == true) {
    controlTemperature();
  }

  if ( ourSecond%10 == 0 || ourStateChanged == true) {
    mqtt_sendData();
  }
}

void logMsg(LogSeverity aSeverity, String aMessage) {
  log_printSecond();
  Serial.print(aMessage);
  Serial.println();

  if (ourMqttClient.connected()) {
    if (aSeverity >= ourLogSeverity) {
      String msg = "("+String(aSeverity)+"): "+aMessage;
      mqtt_publishData("/log/%s/message", msg);
    }
  }
}

void log_printSecond() {
  char buf[13];
  snprintf (buf, 13, "%05d: ", ourSecond);
  Serial.print(buf);
}

