////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ANDRUINO_PUSH_H
#define ANDRUINO_PUSH_H


#include "Arduino.h"
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_SERVICE.h"
#include "ANDRUINO_JSON.h"


extern byte internet_fail_cnt , push_success_cnt, push_fail_cnt, feedD;
extern int pin_push_flash;

#define WAIT_PUSH_RESPONSE 500
#define WAIT_PUSH_RESPONSE_TIMEOUT 12000

class ANDRUINO_PUSH
{
  private:
    bool GetResponse();

    bool CheckPin();

  public:
    bool  SendPush(char *push_usr, char *arduino_name, char *type, char *mode, char *group, byte port, byte lim, float value);
    bool  SendPush2(char *push_usr, char *arduino_name, char *type, char *msg);
    bool  SendHttp(char *push_usr, char *arduino_name,char *arduino_pass, char *http_file);
    char* FinishGetStream(char *ServerResponse, byte size, char *host_name, char *find_char, unsigned int Timeout);
};

#endif
