
////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_PUSH.h"


bool  ANDRUINO_PUSH::CheckPin()

{
  if (pin_push_flash < 1000 || pin_push_flash > 9999) {       //PIN not valid
#if DEBUG_SERIAL == 1
    //        Serial.println(F("push pin empty"));
#endif
    push_fail_cnt++;                                        //increment the counter of bad push connection
    return false;
  }
  return true;
}

//SendPush(push_user, ARDUINO_NAME, "lim","ana","arduino_io", ArduinoAnalog[i].pin,OutOfLimits,ArduinoAnalog[i].value)
//SendPush(push_user, ARDUINO_NAME, "lim","dig","arduino_io", ArduinoIO[i].pin,OutOfLimits,ArduinoIO[i].state)
//SendPush(push_user, ARDUINO_NAME, "lim","var","arduino_io" ,i , OutOfLimits,Arduino_User_var[i].value)
//SendPush(push_user, ARDUINO_NAME, "pow","","",0,valore, VERSION)
bool  ANDRUINO_PUSH::SendPush(char *push_usr, char *arduino_name, char *type, char *mode, char *group, byte port, byte lim, float value)

{

  if (!CheckPin()) {  //if PIN=0 or PIN>9999 pin is wrong or  first time connection, so skip push connection
    return 0;
  }

#if DEBUG_SERIAL == 1
  Serial.println(F("push connecting.."));
#endif
  char *result = 0;

  if (_client.connect("andruino.it", 80)) {        ///andruino.it as number
    //	    if (_client.connect("andruino.it", 80)) {        ///andruino.it as number

#if DEBUG_SERIAL == 1
    Serial.println(F("connected"));
#endif

    //http://andruino.it/push3/send_push.php?user=pushuser&pin=2000
    //&type=lim (lim)
    //&mode=ana (type, ana or var or in)
    //&port=2  (port number)
    //group=arduino_io o XBee_io_0/XBee_io_1
    //&lim=1   (1 means that the high limit is reached, 2 for lower limit)
    //&value=25
    //&ardu=arduino (arduino user identify)
    //andrea_push?type=limits&mode=Ana&port=1&lim=hi&value=0.11

    _client.print(F("GET /push3/send_push?user="));
    _client.print(push_usr);
    _client.print(F("&pin="));
    _client.print(pin_push_flash);
    _client.print(F("&ardu="));
    _client.print(arduino_name);
    _client.print(F("&type="));
    _client.print(type);
    _client.print(F("&mode="));
    _client.print(mode);
    _client.print(F("&port="));
    _client.print(port);
    _client.print(F("&group="));
    _client.print(group);
    _client.print(F("&lim="));
    _client.print(lim);
    _client.print(F("&value="));
    _client.print(value, 3);			//3 decimals digits

    char ServerResponse[20];

    result = FinishGetStream(ServerResponse, sizeof(ServerResponse), "Host: andruino.it", "NO ERR", 10);
  }
#if DEBUG_SERIAL_PUSH == 1
  else
    Serial.println(F("conn. failed"));
#endif

  if (result) {
    push_success_cnt++;		//increment the counter of good push connection
    return true;
  }
  else {
    push_fail_cnt++;	   //increment the counter of bad push connection
    return false;
  }

}

bool  ANDRUINO_PUSH::SendPush2(char *push_usr, char *arduino_name, char *type, char *msg)
{
  if (!CheckPin()) {
    return 0;
  }

#if DEBUG_SERIAL_PUSH == 1
  Serial.println(F("push connecting.."));
#endif

  char *result = 0;

  if (_client.connect("andruino.it", 80)) {    //andruino.it as number
#if DEBUG_SERIAL_PUSH == 1
    Serial.println(F("connected"));
#endif
    //andrea_push?type=limits&mode=Ana&port=1&lim=hi&value=0.11
    _client.print(F("GET /push3/send_push?user="));
    _client.print(push_usr);
    _client.print(F("&pin="));
    _client.print(pin_push_flash);
    _client.print(F("&ardu="));
    _client.print(arduino_name);
    _client.print(F("&type="));
    _client.print(type);
    _client.print(F("&msg="));
    _client.print(msg);

    char ServerResponse[20];

    result = FinishGetStream(ServerResponse, sizeof(ServerResponse), "Host: andruino.it", "NO ERR", 10);
  }
#if DEBUG_SERIAL_PUSH == 1
  else
    Serial.println(F("conn. failed"));
#endif

  if (result) {
    push_success_cnt++;		//increment the counter of good push connection
    return true;
  }
  else {
    push_fail_cnt++;	   //increment the counter of bad push connection
    return false;
  }

}


//http://andruino.it/push3/launch_logger?user=pushuser&pin=0000&ardu=arduino&pass=andrea
//SendHttp(push_user, ardu_user, adru_pass, "launch_logger")
bool  ANDRUINO_PUSH::SendHttp(char *push_usr, char *arduino_name,char *arduino_pass, char *http_file)
{
  if (!CheckPin()) {
    return 0;
  }

#if DEBUG_SERIAL_PUSH == 1
  Serial.println(F("http connecting.."));
#endif

  char *result = 0;

  if (_client.connect("andruino.it", 80)) {    //andruino.it as number
#if DEBUG_SERIAL_PUSH == 1
    Serial.println(F("connected"));
#endif
    _client.print(F("GET /push3/"));
    _client.print(http_file);
    _client.print(F("?user="));
    _client.print(push_usr);
    _client.print(F("&pin="));
    _client.print(pin_push_flash);
    _client.print(F("&ardu="));
    _client.print(arduino_name);
    _client.print(F("&pass="));
    _client.print(arduino_pass);

    char ServerResponse[20];
    result = FinishGetStream(ServerResponse, sizeof(ServerResponse), "Host: andruino.it", "NO ERR", 10);
  }
#if DEBUG_SERIAL_PUSH == 1
  else
    Serial.println(F("conn.failed"));
#endif

  if (result) {
    push_success_cnt++;    //increment the counter of good push connection
    return true;
  }
  else {
    push_fail_cnt++;     //increment the counter of bad push connection
    return false;
  }

}



char* ANDRUINO_PUSH::FinishGetStream(char *buffer_rx, byte size_buffer, char *host_name, char *find_char, unsigned int Timeout) {
  bool loop_search = false;
  char *result = 0;

  _client.println(F(" HTTP/1.1"));
  _client.println(host_name);
  _client.println(F("Connection: close"));
  _client.println();



  delay(Timeout);

  unsigned long previousMillisTmp = 0;
  unsigned long currentMillisTmp = 0;
  bool tmp;

  byte index = 0;
  char c;
  previousMillisTmp = millis();

  while (!loop_search) {
    tmp = _client.available();
    currentMillisTmp = millis();
    if (currentMillisTmp - previousMillisTmp > WAIT_PUSH_RESPONSE_TIMEOUT) {
#if DEBUG_SERIAL == 1
      Serial.print(F("Timeout:")); Serial.println(host_name);
#endif
      _client.flush();
      _client.stop();
      return (result);
    }
    if (tmp) {
      c = _client.read();
      if (index < size_buffer-1) {
        buffer_rx[index] = c;
        buffer_rx[index + 1] = '\0';
      }
      index++;
#if DEBUG_SERIAL_PUSH_DEEP == 1
      Serial.print(c);
#endif

      if (c == '\n' || c == '\r') {
        if ((result = strstr((char *)buffer_rx, find_char)) && index < size_buffer-1)         // "NO ERR"
          loop_search = true;		              //then exit
        index = 0;
      }
    }
  }

#if DEBUG_SERIAL == 1
  if (loop_search) {
    Serial.print(host_name); Serial.println(F(": OK"));
  }
  else {
    Serial.print(host_name); Serial.println(F(": ERR"));
  }
#endif
  _client.flush();
  _client.stop();
  return (result);

}




