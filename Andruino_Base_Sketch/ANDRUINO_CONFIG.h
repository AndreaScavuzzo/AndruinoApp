////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_CONFIG_H
#define ANDRUINO_CONFIG_H


///////////////////////////
//MAX PINS DEFINITION
//
//MAX of MAXPIN is 56 (only for 2560)
//MAX of MAXANA is 16 (only for 2560)
//MAX of ARDUINO_USER_VAR_MAX is 40: max number of user variable sent by ethernet to iPhone
///////////////////////////
#if ARDUINO_DUE == 1
#define MAXPIN 29                                                  //PIN from 0 to 28
#define MAXANA 12                                                  //ANA from 0 to 11
#define ARDUINO_USER_VAR_MAX 20                                    //variables

#elif ARDUINO_2560MEGA == 1
#define MAXPIN 50                                             //PIN from 0 to 49 
#define MAXANA 16                                             //ANA from 0 to 15
#define ARDUINO_USER_VAR_MAX 20                               //VARIABLES

#elif ARDUINO_YUN == 1                                             // ARDUINO UNO has 14 IO, 2K of RAM, 6 Analogs
#define MAXPIN 14                                              //PIN from 0 to 13
#define MAXANA 6                                              //ANA from 0 to 11
#define ARDUINO_USER_VAR_MAX 10                                //variables

#elif ARDUINO_UNO == 1                                             // ARDUINO UNO has 14 IO, 2K of RAM, 6 Analogs
#define MAXPIN 10                                              //PIN from 0 to 9
#define MAXANA 6                                               //ANA from 0 to 5
#define ARDUINO_USER_VAR_MAX 10                               //variables
#endif


//ZIGBEE module SERIES2, configured in sampling mode (read zigbee adc and digital pin, output not implemented for now)
//ZIGBEE MAX DIG/ANA
#define XBeeMaxPinModule 5                                          //max number of XBee analog/digital ports sent by ethernet to iPhone
#define XBeeMaxModules 3                                            //XBEE UNIT MAX INSTANCES (3 modules=0,1,2 unit number)


//NRF24L configuration modules (Nodes)
#define NRF24LMaxDigitalPin 6           //max number of digital ports on remote Arduino sensor with NRF24L
#define NRF24LMaxAnalogPin 5            //max number of analog ports on remote Arduino sensor with NRF24L
#define NRF24LMaxVariable 5             //5=max number of variables  on remote Arduino sensor with NRF24L (don't modify this value)
#define NRF24LMaxModules 10             //don't exceed 20 modules


#if Dallas_DS18B20 == 1
#define OneWire_ENABLE 1
#endif


//************************************************************************************************//
//DEBUG
//************************************************************************************************//

//debug with LED
#define DEBUG_LED_XBEE 0             //enable led (TXLed ArduinoIO[8]) toggle when the xbee module receive data from the sensors

//debug with SERIAL
//ENABLE SERIAL ONLY FOR DEBUG PRINTING
//WARNING, DEBUG SERIAL USE TOO RAM
#define DEBUG_SERIAL 1               //enable serial debug (uses pin0 and 1).
#define DEBUG_SERIAL_TIMER 0              //enable serial debug LEVEL 2

#define DEBUG_SERIAL_JSON 0          //enable serial debug JSON DIGITAL/ANALOG TX (uses pin0 and 1). 
#define DEBUG_SERIAL_RAM 0           //enable serial debug FREE RAM (uses pin0 and 1). 
#define DEBUG_SERIAL_ZIGBEE 0        //enable serial debug ZIGBEE (uses pin0 and 1). 
#define DEBUG_SERIAL_DDNS 0          //enable serial debug DDNS (uses pin0 and 1).
#define DEBUG_SERIAL_MAIL 0          //enable serial debug MAIL (uses pin0 and 1). 
#define DEBUG_SERIAL_IRMS 0          //enable serial debug IRMS (uses pin0 and 1). 

#define DEBUG_SERIAL_PUSH 0
#define DEBUG_SERIAL_PUSH_DEEP 0
#define DEBUG_SERIAL_LIMITS 0


#define DEBUG_SERIAL_NRF24L 0


//************************************************************************************************//
//DEBUG
//************************************************************************************************//
#if DEBUG_SERIAL_NRF24L == 1
#define IF_SERIAL_DEBUG_NRF(x) ({x;})
#else
#define IF_SERIAL_DEBUG_NRF(x)
#endif
//************************************************************************************************//
//ADC DEFINES
//************************************************************************************************//

#define ADC_MAX 1024

//XBEE
#define VDD_XBEE_ADC 1.2
#define ADC_XBEE_MAX 1024
#define ADC_XBEE_STEP VDD_XBEE_ADC/(ADC_XBEE_MAX-1)


//************************************************************************************************//
//OTHER
//************************************************************************************************//

#define PIN_PULSE_WIDTH 500                                     //IO pin pulse duration (500ms)

#if WIFI_SHIELD == 1
#define DELAY_TX_ETHERNET 0                                     //20ms for 3G transmissions
#else
#define DELAY_TX_ETHERNET 20                                    //20ms for 3G transmissions
#endif


#define TIMEOUT_RCVDATA 4000                                    //4 sec timeout for the HTTP data receive

#if ARDUINO_YUN == 1
#define TIMEOUT_RCVCMD 2000                                      //2 sec timeout for the HTTP command receive (YUN)
#else
#define TIMEOUT_RCVCMD 500                                       //0.5 sec timeout for the HTTP command receive
#endif


#define CHECK_DDNS_EVERY 10                                     //check ddns every 10 minutes
#define CHECK_LIMITS_EVERY 2                                    //every 2 seconds check the limits
#define CHECK_LIMITS_XBEE_EVERY 10                              //every 10 seconds check the limits
#define CHECK_POWER_CONSUMPTION_EVERY 4                         //every 4 seconds check the power consumption


#define SEND_SENSOR_REQ_EVERY 5
#define SEND_SENSOR_REQ2_EVERY 2

#define DDNS_READ_AFTER_INACTIVE 60                             //check ddns every 60 seconds
#define CHECK_DALLAS_TEMPERATURE_MINUTE 1                       //check DALLAS_TEMPERATURE every 1 minute



#endif

