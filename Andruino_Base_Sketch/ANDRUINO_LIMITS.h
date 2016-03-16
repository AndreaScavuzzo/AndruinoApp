////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo Feb 2014
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_LIMITS_H
#define ANDRUINO_LIMITS_H


#include "Arduino.h"
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_JSON.h"
#include "ANDRUINO_SERVICE.h"
#include "ANDRUINO_PUSH.h"

extern ANDRUINO_PinTypes pin_types;
extern boolean check_limits_var, check_limits_xbee, check_limits_dig, check_limits_ana;
extern ANDRUINO_PinTypes::DigitalPin ArduinoIO[MAXPIN];
extern ANDRUINO_PinTypes::AnalogPin ArduinoAnalog[MAXANA];
extern ANDRUINO_PinTypes::Variable Arduino_User_var[ARDUINO_USER_VAR_MAX];
extern ANDRUINO_PUSH push;

#if ZIGBEE_ENABLE == 1
extern ANDRUINO_PinTypes::SystemXBeePinsType SystemXBeePins[XBeeMaxModules];         //max number of xbee modules
#endif


class ANDRUINO_LIMITS
{
  private:

    byte check_HiLolimits (unsigned int enable_limits, unsigned int max_lim, unsigned int min_lim, byte &limit_count, unsigned int value);
    byte check_digitalAlarm (unsigned int enable_limits, bool alarm, byte &limit_count, bool value);


  public:
    void  CheckLimitsSensors(char *push_usr, char *arduino_name);
};

#endif
