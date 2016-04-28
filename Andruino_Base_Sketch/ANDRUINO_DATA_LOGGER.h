////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_DATA_LOGGER_H
#define ANDRUINO_DATA_LOGGER_H


#include "Arduino.h"
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_SERVICE.h"
#include "ANDRUINO_JSON.h"


extern byte internet_fail_cnt , push_success_cnt, push_fail_cnt, feedD;
extern int pin_push_flash;
extern ANDRUINO_JSON json;

#define WAIT_PUSH_RESPONSE 500
#define WAIT_PUSH_RESPONSE_TIMEOUT 12000

class ANDRUINO_DATA_LOGGER
{
  private:
    bool GetResponse();

    bool CheckPin();

  public:
    bool SendDataLogger(char *push_usr, char *arduino_name,char *arduino_pass, byte mode);

};

#endif
