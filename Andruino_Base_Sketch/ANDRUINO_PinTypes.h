////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_PinTypes_H
#define ANDRUINO_PinTypes_H


#include "Arduino.h"
#include "ANDRUINO_0DEFINES.h"

#define PWM 2

class ANDRUINO_PinTypes
{
  private:


  public:

    typedef struct {
      byte pin : 6;        //MAX 2^6=64 pin
      byte mode : 2;      //mode=0 --> input, mode=1 -->output, mode=3 -->pwm
      byte state : 8;     //max 256
      boolean pulse : 1;
      boolean used : 1;                                           //pin used?
      //used to understand if the limit reached is high or low
      byte limit_count;

      //counter for output timer
      unsigned int time_counter;			//not yet stored in flash (seconds, from 0 to 65535 - 0 to 18,2 hours)


      ///////////////////////////////////////
      //stored in flash and updated by App

      //used to store the alarm trigger (H or L)
      boolean  alarm;						//already stored in flash

      //16bit (used 9 bits)
      unsigned int enable_limits : 9; 							//already stored in flash
      //enable action when a  limit is reached (push or output)
      //BIT10=00 ->no limit enabled
      //BIT10=01 ->push enabled
      //BIT10=10 ->output enabled
      //BIT10=01 ->push/out enabled
      //BIT72=XXXXX ->output
      //BIT8=0/1 ->polarity limit
    } DigitalPin;



    typedef struct {
      byte pin : 6;
      float value;
      uint16_t state;
      byte limit_count;
      boolean used : 1;                                          //pin used?


      ///////////////////////////////////////
      //stored in flash and updated by App
      unsigned int  max;                                         //limits are *1024
      unsigned int  min;                                         //limits are *1024
      unsigned int  enable_limits : 9;                           //enable action when a  limit is reached (push or output)
      //BIT10=00 ->no limit enabled
      //BIT10=01 ->push enabled
      //BIT10=10 ->output enabled
      //BIT10=11 ->push/out enabled
      //BIT72=XXXXX ->output from 0 to 63
      //BIT8=0/1 ->polarity limit
    } AnalogPin;



    typedef struct {
      float value;
      byte limit_count;

      ///////////////////////////////////////
      //stored in flash and updated by App
      unsigned int  max;
      unsigned int  min;
      unsigned int  enable_limits : 9;                            //enable action when a  limit is reached (push or output)
      //BIT10=00 ->no limit enabled
      //BIT10=01 ->push enabled
      //BIT10=10 ->output enabled
      //BIT10=11 ->push/out enabled
      //BIT72=XXXXX ->output  from 0 to 63
      //BIT8=0/1 ->polarity limit

    } Variable;


    typedef struct {


      byte pin : 6;        //MAX 2^6=64 pin
      byte mode : 2;      //mode=0 --> input, mode=1 -->output, mode=3 -->pwm
      
      byte state : 8;     //max 256
      bool W;           //
      byte Wstate;        //
      
      boolean pulse : 1;
      boolean used : 1;                                           //pin used?
      //used to understand if the limit reached is high or low
      byte limit_count;

      //counter for output timer
      unsigned int time_counter;      //not yet stored in flash (seconds, from 0 to 65535 - 0 to 18,2 hours)


      ///////////////////////////////////////
      //stored in flash and updated by App

      //used to store the alarm trigger (H or L)
      boolean  alarm;           //already stored in flash

      //16bit (used 9 bits)
      unsigned int enable_limits : 9;               //already stored in flash
      //enable action when a  limit is reached (push or output)
      //BIT10=00 ->no limit enabled
      //BIT10=01 ->push enabled
      //BIT10=10 ->output enabled
      //BIT10=01 ->push/out enabled
      //BIT72=XXXXX ->output
      //BIT8=0/1 ->polarity limit
    } RemoteDigitalPin;

    //TYPES FOR XBEE (ANALOG AND DIGITAL)
    typedef struct {
      byte pin : 6;                                               //analog number port
      byte mode : 2;                                              //input(0) or output (1), or analog (2)
      float  value;                                               //analog value read by adc or pin state
      boolean used : 1;                                           //pin used?
      byte limit_count;

      ///////////////////////////////////////
      //not yet stored in flash and updated by App
      unsigned int  max;                                         //limits are *1024
      unsigned int  min;                                         //limits are *1024
      boolean enable_limits : 1;

    } XBeePinsType;


#if ZIGBEE_ENABLE == 1
    //TYPES FOR XBEE UNITS (each unit can have analog/digital pins)
    typedef struct {
      XBeePinsType Pin[XBeeMaxPinModule];
      uint32_t RemoteAddrLSB;
      uint32_t RemoteAddrMSB;
      byte samples;           //used to know how many samples has been received
    } SystemXBeePinsType;
#endif


#if NRF24L_ENABLE == 1
    //TYPES FOR NRF UNITS (each unit can have analog/digital pins)
    typedef struct {
      RemoteDigitalPin DigPin[NRF24LMaxDigitalPin];
      AnalogPin AnaPin[NRF24LMaxAnalogPin];
      Variable VarPin[NRF24LMaxVariable];
      uint8_t dig_num;                  //number of digital pin =0
      uint8_t ana_num;                  //number of analog pin =0
      uint8_t var_num;                  //number of variable pin =0

      float AdcStep;
      uint16_t Supply;
      uint16_t RNF24LAddr;
      uint16_t samples;                  //used to know how many samples has been received
      //fix 6.11
      uint16_t sleep_time_value;          //sleep seconds for the remote Arduino
      uint16_t alive_counter;            //used to know if the sensor is alive
      uint16_t alive_fail_counter;       //used to know if the sensor has been disconnected
      bool system_back_wr;               //used to send back system data
      uint16_t fw_version;               //firmware version   
      uint8_t pa_level;                  //level of PA transmitter
      uint8_t leaf;                  //level of PA transmitter =255
    } SystemNRF24LPinsType;
#endif





    void setupDig(DigitalPin& pin, boolean mode, boolean state);
    void setupPWM(DigitalPin& pin, byte mode, byte state);
    void writeDig(DigitalPin& pin, boolean state);
    void writePWM(DigitalPin& pin, byte state);
    DigitalPin& read(DigitalPin& pin);
    boolean changed(DigitalPin& pin);
    void write(DigitalPin& pin, boolean state);
    void toggle(DigitalPin& pin);
    void pulse(DigitalPin& pin, int wait);
    void flashLed(DigitalPin& pin, byte times, int wait) ;
    void setAnalog(AnalogPin& pin, boolean mode);
    void setupAnalog(AnalogPin& pin);
    AnalogPin& readAnalog(AnalogPin& pin);
    void writeAnalog(AnalogPin& pin);

};

#endif
