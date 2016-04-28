////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_BASE_H
#define ANDRUINO_BASE_H

#include <TimerOne.h>

#include "Arduino.h"

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_SERVICE.h"    //service routines


#include "ANDRUINO_JSON.h"       //manages the communication with iPhone
#include "ANDRUINO_DDNS.h"       //read the external WAN address (for now, it is not used)
#include "ANDRUINO_PUSH.h"       //send push notification to iPhone
#include "ANDRUINO_LIMITS.h"
#include "ANDRUINO_TIMERS.h"
#include "ANDRUINO_XBEE.h"
#include "ANDRUINO_IRMS.h"
#include "ANDRUINO_0SENSORS.h"
#include "ANDRUINO_DATA_LOGGER.h"


#if ZIGBEE_ENABLE == 1
#include <XBee.h> 
#endif
  

//#if NRF24L_ENABLE == 1
#include "ANDRUINO_NRF.h"
//#endif


#if WIFI_SHIELD == 1
extern char ssid[]; 
extern char pass[];
#endif

extern char ARDUINO_NAME[15];
extern char ARDUINO_PASS[15];
extern float VERSION;

extern byte mac[];
extern byte ip[];
extern byte gateway[];
extern int ETHERNET_PORT;
extern byte dnsAdd[];




extern ANDRUINO_SERVICE service;
extern ANDRUINO_JSON json;
extern ANDRUINO_LIMITS limits;
extern ANDRUINO_PUSH push;
extern ANDRUINO_TIMERS timers;


//ethernet and pin instances

extern ANDRUINO_PinTypes pin_types;
extern ANDRUINO_NRF nrf;



extern ANDRUINO_PinTypes::DigitalPin ArduinoIO[MAXPIN];
extern ANDRUINO_PinTypes::AnalogPin ArduinoAnalog[MAXANA];
extern ANDRUINO_PinTypes::Variable Arduino_User_var[ARDUINO_USER_VAR_MAX];

#if NRF24L_ENABLE == 1
extern ANDRUINO_PinTypes::SystemNRF24LPinsType SystemNRF24LPins[NRF24LMaxModules];
#endif

extern byte ddns_success_cnt, internet_fail_cnt , push_success_cnt, push_fail_cnt, feedD;
extern byte seconds_counter;
extern byte minutes_counter;
extern byte hours_counter;
extern byte days_counter;
extern unsigned int network_access;
extern ANDRUINO_PinTypes pin_types;
extern boolean force_pushddns;
extern float version_sketch;
extern boolean send_push_msg;

extern int pin_push_flash;
extern char push_user[16];          //fix Oct 2014
extern char ddns_address[16];
extern unsigned int andruino_app_id;
extern unsigned int andruino_app_version;
extern boolean nrf_radio_enable;
extern boolean nrf_radio_enable_rq;
extern boolean nrf_radio_disable_rq;
extern float ADC_STEP;

extern boolean check_ddns;
extern boolean check_limits_ana;
extern boolean check_limits_xbee;
extern boolean check_limits_var;
extern boolean check_limits_dig;
extern boolean check_sensor1;
extern boolean check_sensor2;
extern boolean check_DallasTemperature;
extern boolean check_power_consumption;
extern boolean check_timers;
extern boolean send_sensor_req,send_sensor_req2;

extern boolean force_pushddns;
extern boolean send_push_msg;
extern boolean store_sensor_eeprom;

extern boolean nrf_radio_enable;
extern boolean nrf_radio_enable_rq;
extern boolean nrf_radio_disable_rq;

extern unsigned int connection_isfar;
extern unsigned int network_access;
extern boolean swap_timer;

#if ETHERNET_SHIELD == 1
#include <Ethernet.h>
extern EthernetClient _client;
#elif ETHERNET_SHIELD_V2 == 1
#include <EthernetV2_0.h>
extern EthernetClient _client;
#elif WIFI_SHIELD == 1
#include <WiFi.h>
extern WiFiClient _client;
#elif ARDUINO_YUN == 1
#include <YunServer.h>
#include <YunClient.h>
extern  YunServer server;
extern  YunClient _client;
#endif

class ANDRUINO_BASE : public ANDRUINO_0SENSORS, public ANDRUINO_SERVICE, public ANDRUINO_NRF
{
  private:

  public:
    //void Interrupt_Timer_1sec();
    void Setup();
    void Loop();

};


#endif
