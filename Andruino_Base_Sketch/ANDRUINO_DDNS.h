////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_DDNS_H
#define ANDRUINO_DDNS_H
#include "Arduino.h"
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_JSON.h"
#include "ANDRUINO_SERVICE.h"
#include "ANDRUINO_PUSH.h"       //send push notification to iPhone


#define WAIT_DDNS_RESPONSE 1000
//************************************************************************************************//
//DDNS MANAGEMENT for Arduino Uno and Due
//************************************************************************************************//
#define MAXROW 20


class ANDRUINO_DDNS
{
  private:
    bool find_ddns();
  public:

    void CheckDDNS_andSend_Notification(char *push_usr, char *arduino_name, bool force_pushddns);
};


#endif
