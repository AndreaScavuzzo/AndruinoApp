////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo Aug 2014
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_TIMERS_H
#define ANDRUINO_TIMERS_H


#include "Arduino.h"
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_JSON.h"
#include "ANDRUINO_SERVICE.h"
#include "ANDRUINO_PinTypes.h"
#include "ANDRUINO_PUSH.h"       //send push notification to iPhone

extern byte seconds_counter;
extern byte minutes_counter;
extern byte hours_counter;
extern byte days_counter;
extern ANDRUINO_PinTypes pin_types;
extern ANDRUINO_PinTypes::DigitalPin ArduinoIO[];
extern ANDRUINO_PUSH push;
extern unsigned int andruino_app_version;


class ANDRUINO_TIMERS
{
  private:

  public:
    void CheckTimers(char *push_usr, char *arduino_name);
};

#endif
