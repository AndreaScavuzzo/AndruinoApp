////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_JSON_H
#define ANDRUINO_JSON_H

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_SERVICE.h"
#include "ANDRUINO_PinTypes.h"



#if ARDUINO_STDAVR == 1
#include <avr/eeprom.h>
#else
#include <DueFlashStorage.h>
extern DueFlashStorage dueFlashStorage;
#include  <avr/dtostrf.h>
#endif



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
 


#if ZIGBEE_ENABLE == 1
#include <XBee.h>              //zigbee is enabled only for Arduino Due (for ram reason)
#include "ANDRUINO_XBEE.h"
#endif


#if NRF24L_ENABLE == 1
#include "ANDRUINO_NRF.h"
#endif

#define bufferMax 128
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

extern unsigned int http_server_performance_ms;
extern unsigned int http_client_performance_ms;

class ANDRUINO_JSON
{
private:
    
 
    

    
    void JSON_Arduino_SendAll(byte mode,bool type_json);
    void JSON_FormatDigital (byte port, char *mode, byte value, boolean pulse, bool type);

    //    void JSON_FormatAna (byte port,char *mode, unsigned int value);
    //    void JSON_FormatVar (byte port,char *mode, float value);
    void JSON_FormatAnaVar (byte port, char *mode, float value, bool type);
    void JSON_PrintDataComma (char *datas, bool comma);
    void JSON_PrintDataComma (unsigned int datas, bool comma);
    void RemoteDigitalWrite(byte indexV, int duration);
    void RemoteDigitalWriteZigBee(uint8_t port, byte indexZigBeeModule);
    void RemoteDigitalWriteNRF24L(uint8_t port,uint8_t value, byte indexModule);
    short SearchRemoteNRF24LPin (byte index_node, uint8_t port) ;

    void RemoteCommandNRF24L(byte command, unsigned int nrf_module, unsigned int first, unsigned int second,unsigned int third);
    
    int FindPortNumber(int PortNumber);
    int FindAnaPortNumber(int PortNumber);
#define cmd_lenght 15
#define port_lenght 8
#define action_lenght 15
#define action2_lenght 10
#define action3_lenght 10
#define action4_lenght 8
    char cmd[cmd_lenght];
    char port[port_lenght];
    char action[action_lenght];
    char action2[action2_lenght];
    char action3[action3_lenght];
    char action4[action4_lenght];
    
    
private:
    
    bool  SearchKeyGET (char *bufferino, char *search_string, int lungh, char *out_buffer,  char *separator_char );
public:
    void ReadPushUserPin();
    
#if ARDUINO_YUN == 2
    byte WaitForRequest_and_ParseReceivedRequest(YunClient _client, String ardu_name, String ardu_pass);
#elif ETHERNET_SHIELD == 1 || ETHERNET_SHIELD_V2 == 1
    byte WaitForRequest_and_ParseReceivedRequest(EthernetClient _client, char *ardu_name, char *ardu_pass);
#elif ARDUINO_YUN == 1
    byte WaitForRequest_and_ParseReceivedRequest(YunClient _client, char *ardu_name, char *ardu_pass);
#elif WIFI_SHIELD == 1
    byte WaitForRequest_and_ParseReceivedRequest(WiFiClient _client, char *ardu_name, char *ardu_pass);
#endif
    
    void PerformRequestedCommand(bool type_json);
    
    void JSON_NRF24L(bool json_type);
    void JSON_XBEE(bool json_type);
    
    void JSON_Arduino_io(byte mode,bool type_json);    
    void JSON_Arduino_json_NRF24Lio(byte index,bool type_json);
    void JSON_Arduino_json_IO(bool type_json);
    void JSON_Arduino_json_Analog(bool type_json);
    void JSON_Arduino_json_XbeeIO(uint32_t UDID_LSB, byte index, bool type_json);
    void JSON_Arduino_json_Vars(bool type_json);
    
    void JSON_Arduino_Timers();
    void JSON_Arduino_json_System();
    void JSON_ClientFlush();
    void JSON_comma();

    
};
#endif
