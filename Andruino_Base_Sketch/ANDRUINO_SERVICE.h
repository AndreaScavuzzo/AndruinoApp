////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_SERVICE_H
#define ANDRUINO_SERVICE_H


#include "Arduino.h"
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_JSON.h"
#include "ANDRUINO_PinTypes.h"
#include "ANDRUINO_XBEE.h"



extern ANDRUINO_PinTypes::DigitalPin ArduinoIO[];
extern ANDRUINO_PinTypes::AnalogPin ArduinoAnalog[];
extern ANDRUINO_PinTypes::Variable Arduino_User_var[ARDUINO_USER_VAR_MAX];


class ANDRUINO_SERVICE
{
  private:

  public:

#if ARDUINO_STDAVR == 1
    int freeRam ();
#endif
    void InitSensorArray();
    void CheckDDNS_andSendEmail();

    void RestoreDataFromFlash();
    void StoreDataToFlash();
    void printWifiStatus();
};

#endif
