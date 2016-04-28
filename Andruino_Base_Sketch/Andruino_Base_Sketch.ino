//#########################
//Andruino version rel 7.00
//A.Scavuzzo
//#########################

//7.00
//uint_16 per il sleep time remote config
//crashes when the node sensor reach 65000 connections
//new php address files (3)
//andruino address changed (from fixed to andruino.it)
//DDNS address changed (from fixed to checkip.dyndns.com)

//Datalogger
//if datalogger is enabled by App and by #define (DATA_LOGGING_DB 1), the sensors are sent to the Andruino external database




#include <SPI.h>
#include <TimerOne.h>
//#include <XBee.h>
#include <RF24.h>
#include <RF24Network.h>

#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_MAX31855.h>

#include <Ethernet.h>
//#include <EthernetV2_0.h>
//#include <WiFi.h>
//#include <YunServer.h>
//#include <YunClient.h>

//##########################
//NRF NETWORD ID
//##########################
byte nrf_pipeline_id[] = {0x2d, 0x4b, 0x60, 0x87, 0x96, 0xb4};               //you can leave them as default

#include "ANDRUINO_BASE.h"
ANDRUINO_BASE SetupIns;


//##########################
//ARDUINO USERNAME AND PASSWORD
//##########################
char ARDUINO_NAME [] = "arduino";                                             //<<<<-----change it or leave as default   //username Arduino board (you can modify it or leave as it is)
char ARDUINO_PASS [] = "andrea";                                              //<<<<-----change it or leave as default   //password Arduino board (you can modify it or leave as it is)

//##########################
//USER NETWORK CONFIGURATION
//##########################
byte mac[] = {0x00, 0x17, 0xf2, 0xd1, 0x01, 0x4A};                            //MAC ADDRESS (you can modify it or leave as it is)
byte ip[] = {10, 0, 1, 15};                                                   //IP_ADDRESS  (you can modify it), can be  {192, 168, 1, 15}
byte gateway[] = {ip[0], ip[1], 1, 1};                                               //<<<<-----change it, router MAIN IP can be {192, 168, 1, 1}
int ETHERNET_PORT = 8888;                                                     //<<<<-----change it, PORT NUMBER (you can modify it)
byte dnsAdd[] = {8, 8, 8, 8};                                                 //<<<<-----change it, DNS NUMBER (you can modify it)


//Wifi shield (if used)
char ssid[] = "TimeWiFi";                                                     //<<<<-----change it  your network SSID (name)
char pass[] = "00000000";                                                    //<<<<-----change it  your network password



void setup()
{

  SetupIns.InitSensorArray();                                                 //All the vectors and var will be initialized to avoid ecc errors


  //##############################################################################
  //USER PIN CONFIGURATION
  //
  //pin_types.setupDig(ArduinoIO[X] , Mode, State)
  //Mode=INPUT/OUTPUT/PWM  -->PIN state (DIGITAL PIN: 0=input, 1=output, 2=pwm)
  //##############################################################################

#if (DEBUG_SERIAL == 0)
  pin_types.setupDig(ArduinoIO[0], INPUT, 0);                               //D0 is used by ZigBee module or Serial.print
  pin_types.setupDig(ArduinoIO[1], INPUT, 0);                               //D1 is used by ZigBee module or Serial.print
#endif

#if THERMO_ADAFRUIT_DHT == 0
  pin_types.setupDig(ArduinoIO[2], INPUT, 0);                               //can be used by Adafruit_DHT11
#endif

  pin_types.setupPWM(ArduinoIO[3], PWM, 0);                                 //can be used by Adafruit_MAX31855


  //IMPO: PIN4 has to be high if SD card is inserted or the etheret shield is used
#if ETHERNET_SHIELD_V2 == 1
#define SDCARD_CS 4
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH);                                           //Deselect the SD card to avoid conflicts with Ethernet shield
#elif Dallas_DS18B20 == 0
  pin_types.setupDig(ArduinoIO[4], OUTPUT, 0);                            //can be used by Adafruit_MAX31855 or Dallas (comment this line if yes)
#endif

  pin_types.setupDig(ArduinoIO[5], OUTPUT, 0);                            //can be used by Adafruit_MAX31855 (comment this line if yes)
  pin_types.setupDig(ArduinoIO[6], OUTPUT, 0);
  
#if WIFI_SHIELD == 0
  pin_types.setupDig(ArduinoIO[7], OUTPUT, 0);                            //don't use it in wifi shield (handshake pin between the WiFi shield and the Arduino)
#endif

  pin_types.setupDig(ArduinoIO[8], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[9], INPUT, 0);                             //MAXPIN=9  Arduino Uno


//on Arduino UNO these are used for the ethernet shield (SPI), so they can't be used as GPIO.
//On the other boards (MEGA/YUN) they can be used
#if ARDUINO_YUN == 1 || ARDUINO_2560MEGA == 1
  pin_types.setupDig(ArduinoIO[10], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[11], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[12], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[13], OUTPUT, 0);
#endif


#if ARDUINO_DUE == 1 || ARDUINO_2560MEGA == 1
  pin_types.setupDig(ArduinoIO[14], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[15], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[16], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[17], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[18], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[19], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[20], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[21], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[22], INPUT, 0);
  pin_types.setupDig(ArduinoIO[23], INPUT, 0);
  pin_types.setupDig(ArduinoIO[24], INPUT, 0);
  pin_types.setupDig(ArduinoIO[25], INPUT, 0);
  pin_types.setupDig(ArduinoIO[26], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[27], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[28], OUTPUT, 0);
  pin_types.setupDig(ArduinoIO[29], OUTPUT, 0);
#endif

  //ANALOG PIN SETUP
  pin_types.setupAnalog(ArduinoAnalog[0]);
  pin_types.setupAnalog(ArduinoAnalog[1]);
  pin_types.setupAnalog(ArduinoAnalog[2]);
  pin_types.setupAnalog(ArduinoAnalog[3]);
  pin_types.setupAnalog(ArduinoAnalog[4]);
  pin_types.setupAnalog(ArduinoAnalog[5]);                               //MAXANA=6 Arduino Uno (configure  ANDRUINO_0DEFINES.h)

#if ARDUINO_DUE == 1
  pin_types.setupAnalog(ArduinoAnalog[6]);
  pin_types.setupAnalog(ArduinoAnalog[7]);
  pin_types.setupAnalog(ArduinoAnalog[8]);
  pin_types.setupAnalog(ArduinoAnalog[9]);
  pin_types.setupAnalog(ArduinoAnalog[10]);
  pin_types.setupAnalog(ArduinoAnalog[11]);                             //MAXANA=12 Arduino Due (configure  ANDRUINO_DFINE.h)
#endif



  SetupIns.Setup();

}


void loop() {

  SetupIns.Loop();

}











