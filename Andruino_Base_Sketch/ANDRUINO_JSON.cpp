////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ANDRUINO_JSON.h"
#include "ANDRUINO_ETH_BUFFER.h"



/*
     {"arduino_io":{"INFO":["4354"],"DIGITALS":[{"port":"dig2","mode":"in","val":"0"},{"port":"dig3","mode":"pwm","val":"0"}],"ANALOGS":[{"port":"ana0","mode":"ana","val":"5.023"},{"port":"ana1","mode":"ana","val":"5.023"}]},
     "NRF24L_io_0":{"INFO":["9866","012","2744","60","182","2","1004","2","1"],"DIGITALS":[{"port":"dig3","mode":"in","val":"1"}],"ANALOGS":[{"port":"ana0","mode":"ana","val":"0.721"},{"port":"ana1","mode":"ana","val":"0.906"}]},
     "NRF24L_var_0":{"VARIABLES":[{"port":"var0","mode":"var","val":"0.000"},{"port":"var1","mode":"var","val":"47.200"}]},
     "arduino_var":{"VARIABLES":[{"port":"var0","mode":"var","val":"0.000"},{"port":"var1","mode":"var","val":"47.200"}]},
     "ardu_tim":[255,255,255,255,255,255,255,255,255,255,255,255],
     "ardu_sys":{"ardu_date":[5,15,48,11],"ardu_fd2":["pushuser","xxxx","0.0.0.0","127"],
     "ardu_fd":[4354,0,85,2,98,0,6.005,6.005]}}

 */



//return 1 if the username/password are right
//return 0 if the username/password are wrong or the connection is lost or the 500ms  of timeout occurs

bool ANDRUINO_JSON::SearchKeyGET (char *bufferino, char *search_string, int lungh,  char *out_buffer,  char *separator_char ) {

  char *key;
  char *separator_ptr;
  char *new_pointer;

  key = strstr(bufferino, search_string);

  /*    Serial.print("bufferino: ");
      Serial.println(bufferino);
      Serial.print("key: ");
      Serial.println(key);
      Serial.print("key int: ");
      Serial.println((int)key);
  */

  if (key) {
    separator_ptr  = strstr(key + 1, separator_char);

    /*        Serial.print("separator_ptr:");
            Serial.println(separator_ptr);
            Serial.print("separator_ptr int: ");
            Serial.println((int)separator_ptr);
    */

    strncat(out_buffer, key + lungh, separator_ptr - key - lungh);

    /*Serial.print("out_buffer: ");
    Serial.println(out_buffer);
    */
    return 1;
  } else
    return 0;
}


#if ETHERNET_SHIELD == 1 || ETHERNET_SHIELD_V2 == 1
byte ANDRUINO_JSON::WaitForRequest_and_ParseReceivedRequest(EthernetClient _client, char *ardu_name, char *ardu_pass)
#elif ARDUINO_YUN == 1
byte ANDRUINO_JSON::WaitForRequest_and_ParseReceivedRequest(YunClient _client, char *ardu_name, char *ardu_pass)
#elif WIFI_SHIELD == 1
byte ANDRUINO_JSON::WaitForRequest_and_ParseReceivedRequest(WiFiClient _client, char *ardu_name, char *ardu_pass)
#endif

#if ETHERNET_SHIELD == 1 || ETHERNET_SHIELD_V2 == 1  || WIFI_SHIELD == 1 || ARDUINO_YUN == 1
{
  //Received buffer contains "GET ?user &pass= &cmd= &action= HTTP/1.1".
  char* en1;
  char* en2;
  char* en3;
  char* en4;
  char* en5;
  char* en6;
  char* space2;
  char* qm;
  char* un;
  char buffer_rx[bufferMax];
  char pass[15];    // Limits length to 15 characters
  char user[15];    // Limits length to 15 characters

  int bufferSize = 0;
  unsigned long int StartTime;
  boolean ClientStatus = true;
  boolean Timeout = false;


  StartTime = millis();

  while (ClientStatus && (!Timeout)) {
    if ((millis() - StartTime) > TIMEOUT_RCVCMD) {					//exit if timeout occurs (500ms)
      Timeout = true;
#if DEBUG_SERIAL_JSON == 1
      Serial.println(F("Timeout"));
#endif
      break;
    }
    ClientStatus = _client.connected();
    if (_client.available() && ClientStatus) {
      char c = _client.read();

      if ((c == '\n' || c == ' ') && bufferSize > 30) {

        // you're starting a new line
#if DEBUG_SERIAL_JSON == 1
        Serial.print(F("size: "));
        Serial.println(bufferSize);
#endif

#if ARDUINO_YUN == 0
        _client.println(F("HTTP/1.1 200 OK"));
        _client.println(F("Content-Type: application/json"));   //Content-Type: text/plain
        _client.println();
#endif
        break;

      }
      else if (c != '\r') {
        // you've gotten a character on the current line
        if (bufferSize < bufferMax - 1) {
          buffer_rx[bufferSize] = c;                             //fill the buffer
          bufferSize++;
          buffer_rx[bufferSize] = '\0';                             //ternination
        } else {
#if DEBUG_SERIAL_JSON == 1
          Serial.println(F("overflow2"));
#endif
          return 2;
        }
      }
    }
  }


  if (ClientStatus == false || Timeout == true) {
#if DEBUG_SERIAL_JSON == 1
    Serial.println(F("Timeout or client disconnected"));
#endif
    return 3;
  }


  //http://arduinoyun.local/arduino/IO.json/user=arduinoYUN/pass=andrea/cmd=ReadAll/port=0/action=0/action2=0/action3=0/action4=77777/end
  //http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=ReadAll&port=0&action=0&action2=0&action3=0

#if DEBUG_SERIAL_JSON == 1
  Serial.println(buffer_rx);
#endif


  user[0] = 0;
  pass[0] = 0;
  cmd[0] = 0;
  port[0] = 0;
  action[0] = 0;
  action2[0] = 0;
  action3[0] = 0;
  action4[0] = 0;


  //IO.json/user=arduino/pass=andrea/cmd=ReadAll/port=0/action=0/action2=0/action3=0/action4=77777/en
  //IO.json?user=arduino&pass=andrea&cmd=ReadAll&port=0&action=0&action2=0&action3=0&action4=77777
#if ARDUINO_YUN == 1

  if (!SearchKeyGET(buffer_rx, "/user=", 6, user, "/"))  {                            //No valid stream received (\/user= not found)
    return 4;
  }
  SearchKeyGET(buffer_rx, "/pass=", 6, pass, "/");
  SearchKeyGET(buffer_rx, "/cmd=", 5, cmd, "/");
  SearchKeyGET(buffer_rx, "/port=", 6, port, "/");
  SearchKeyGET(buffer_rx, "/action=", 8, action, "/");
  SearchKeyGET(buffer_rx, "/action2=", 9, action2, "/");
  SearchKeyGET(buffer_rx, "/action3=", 9, action3, "/");
  SearchKeyGET(buffer_rx, "/action4=", 9, action4, "/");
#else
  if (!SearchKeyGET(buffer_rx, "?user=", 6, user, "&"))  {                            //No valid stream received (\/user= not found)
    return 4;
  }
  SearchKeyGET(buffer_rx, "&pass=", 6, pass, "&");
  SearchKeyGET(buffer_rx, "&cmd=", 5, cmd, "&");
  SearchKeyGET(buffer_rx, "&port=", 6, port, "&");
  SearchKeyGET(buffer_rx, "&action=", 8, action, "&");
  SearchKeyGET(buffer_rx, "&action2=", 9, action2, "&");
  SearchKeyGET(buffer_rx, "&action3=", 9, action3, "&");
  SearchKeyGET(buffer_rx, "&action4=", 9, action4, "&");
#endif


#if DEBUG_SERIAL_JSON == 1
  if (strcmp(ardu_pass, pass) == 0 && strcmp(ardu_name, user) == 0)     //Check if username and password match
    Serial.println("pass and user ok");
  else
    Serial.println("pass and or user NOT OK");
  Serial.print("user=");
  Serial.print(user);
  Serial.print(", pass=");
  Serial.print(pass);
  Serial.print(", cmd=");
  Serial.print(cmd);
  Serial.print(", port=");
  Serial.print(port);
  Serial.print(", action=");
  Serial.print(action);
  Serial.print(", action2=");
  Serial.print(action2);
  Serial.print(", action3=");
  Serial.print(action3);
  Serial.print(", action4=");
  Serial.println(action4);
#endif


  //check username and password
  if (strcmp(ardu_pass, pass) == 0 && strcmp(ardu_name, user) == 0) {    //Check if username and password match
    return 1;
  }
  else {
    return 0;
  }
}
#endif


//http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=LimAna&port=0&action=65000&action2=0&action3=0&action4=11          //send limits to ZigBee (module number 1 action4=11 )and enable them
//http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=LimAna&port=4&action=10&action2=20&action3=1&action4=0             //send limits to var (var4, low=10, high=20) and enable them action3=1
//http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=VarWrite&port=2&action=125&action2=0&action3=0&action4=0           //Variable write var[2]=125

//http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=DigWrite&port=5&action=1&action2=0&action3=0&action4=0             //Write Out Digital on PORT5(port), state H(action),  action3=0 (time), action2=0 (no zigbee)
//http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=DigWrite&port=2&action=1&action2=1&action3=2&action4=0          //Write Out ZigBee on PORT2(port), state H(action),  module2(action3),


//PerformRequestedCommand
//return 1 if the command is right
//return 0 if the command is unknown
void ANDRUINO_JSON::PerformRequestedCommand(bool json_type)
{
  unsigned int port_i;
  unsigned int action_i;
  unsigned int action2_i;
  unsigned int action3_i;
  unsigned int action4_i;          //used only in Limits and Pin

  port_i = atoi(port);
  action_i = atoi(action);
  action2_i = atoi(action2);
  action3_i = atoi(action3);
  action4_i = atoi(action4);

  //ArduinoIO
  //[self TX_Stream :c.write_cmd :DIGITAL_CH(port_i) :c.write_value(action) :@"0"(action2_i) :secondi_global(action3_i) :@"0"(action4_i) ];
  //port=2&action=1&action2=1&action3=2&action4=0
  //
  //NRF IO or XBEE
  //[self TX_Stream :c.write_cmd :DIGITAL_CH(port_i) :c.write_value(action) :@"2"(action2_i) :NRF24LModuleIndex(action3_i) :secondi_global(action4_i)];


  //port_i= port number
  //action= high or low
  //action2_i=0 --> Arduino Base=0, Xbee=1, NRF24L=2
  //action3_i=XBee/NRF module number

  if (strcmp(cmd, "DigWrite") == 0) {                              //write command is decoded
    //action2_i=0 --> Arduino IO
    if (action2_i == 0) {                                        //write on Arduino IO
      RemoteDigitalWrite(port_i, action3_i);                   //write operation is performed (port_i=PIN, action3_i=duration if not 0, action=value)
      JSON_Arduino_SendAll(2, json_type);                                 //2 --> read Arduino IO and systems (digitals)
    }
#if ZIGBEE_ENABLE == 1
    //port_i= port number
    //action= high or low
    //action2_i=1 --> Xbee IO enabled
    //action3_i=XBee module number
    else if (action2_i == 1) {
      RemoteDigitalWriteZigBee(port_i, action3_i);             //write operation is performed
      JSON_Arduino_SendAll(2, json_type);						     	              //2 --> read Arduino IO and systems (digitals
    }
#endif
#if NRF24L_ENABLE == 1
    //port_i= port number
    //action= high or low
    //action2_i=2 --> NRF24L IO enabled
    //action3_i=NRF24L module number
    else if (action2_i == 2) {
      RemoteDigitalWriteNRF24L(port_i, action_i, action3_i);             //write operation is performed
      JSON_Arduino_SendAll(2, json_type);                                //2 --> read Arduino IO and systems (digitals
    }
#endif
  }
  //port_i=portA (motor1) min6 bit
  //action_i=activate A or B or not A and B (00, 01, 10)
  //action2_i=portB (motor2) min6 bit
  //action3_i=duration
  //action4_i=0 (Arduino_io or others)
  //


#if NRF24L_ENABLE == 1
  else if (strcmp(cmd, "NRF24L") == 0) {                       //NRF24L command set

    //port_i= port number
    //action= high or low
    //action2_i=2 --> NRF24L IO enabled
    //action3_i=NRF24L module number
    RemoteCommandNRF24L(port_i, action_i, action2_i, action3_i, action4_i);
    JSON_Arduino_SendAll(0, json_type);                                //2 --> read Arduino IO and systems (digitals)

  }
#endif

  else if (strcmp(cmd, "Blnd") == 0) {                       //Blind set

    if (action4_i == 0) {

      if (action_i == 0) {                                //activate port A and disactivate port B
        pin_types.writeDig(ArduinoIO[action2_i], false);
        delay(1000);
        pin_types.writeDig(ArduinoIO[port_i], true);
        ArduinoIO[port_i].time_counter = action3_i;
      } else if (action_i == 1) {                        //activate port B and disactivate port A
        pin_types.writeDig(ArduinoIO[port_i], false);
        delay(1000);
        pin_types.writeDig(ArduinoIO[action2_i], true);
        ArduinoIO[action2_i].time_counter = action3_i;
      } else {                                           //disactivate port B and A
        pin_types.writeDig(ArduinoIO[port_i], false);
        pin_types.writeDig(ArduinoIO[action2_i], false);
      }
    }
    JSON_Arduino_SendAll(2, json_type);								//2 --> read Arduino IO and systems (digitals)

  } else if (strcmp(cmd, "PwmWrite") == 0) {                       //PWM write command is decoded
    pin_types.writePWM(ArduinoIO[port_i], action_i);
    JSON_Arduino_SendAll(2, json_type);									//2 --> read Arduino IO and systems (digitals)

  } else if (strcmp(cmd, "VarWrite") == 0) {                      //Var write command is decoded
    Arduino_User_var[port_i].value = (float)action_i / 10;      //var are trasmitted by 100 using int
    JSON_Arduino_SendAll(0, json_type);									//0 --> read all

  }
  else if (strcmp(cmd, "LimAna") == 0) {                          //download the limit for each sensor
    if (action4_i == 0) {                                       //Arduino_io Analogs is action4_i=0
      ArduinoAnalog[port_i].max = action_i;                   //action=max limit
      ArduinoAnalog[port_i].min = action2_i;                  //action2=min limit
      ArduinoAnalog[port_i].enable_limits = action3_i;        //action3 enable or disable the limit check
    }
#if ZIGBEE_ENABLE == 1
    else {                                                     //XBEE Analogs is action4_i not 0 (starts from 10)
      int index_xbee = action4_i - 10;                        //The XBEE module number is action4_i - 10
      SystemXBeePins[index_xbee].Pin[port_i].max = action_i;			    //action=max limit
      SystemXBeePins[index_xbee].Pin[port_i].min = action2_i;			//action2=min limit
      SystemXBeePins[index_xbee].Pin[port_i].enable_limits = action3_i;  //action3 enable or disable the limit check
    }
#endif
    JSON_Arduino_SendAll(0, json_type);									//read all data

  } else if (strcmp(cmd, "LimDig") == 0) {                        //download the limit for each sensor
    if (action4_i == 0) {                                       //Arduino Dig is action4_i=0
      ArduinoIO[port_i].alarm = action_i;                     //action=alarm
      ArduinoIO[port_i].enable_limits = action3_i;            //action3 enable or disable the limit check
    }
#if ZIGBEE_ENABLE == 1
    else {                                                      //XBEE Dig is action4_i not 0 (starts from 10)
      int index_xbee = action4_i - 10;                        //The XBEE module number is action4_i - 10
      SystemXBeePins[index_xbee].Pin[port_i].max = action_i;	//action=max limit
      SystemXBeePins[index_xbee].Pin[port_i].enable_limits = action3_i;  //action3 enable or disable the limit check
    }
#endif
    JSON_Arduino_SendAll(0, json_type);									//read all data

  }
  else if (strcmp(cmd, "LimVar") == 0) {                          //download the limit for each var
    Arduino_User_var[port_i].max = action_i;                    //action=max limit
    Arduino_User_var[port_i].min = action2_i;                   //action2=min limit
    Arduino_User_var[port_i].enable_limits = action3_i;         //action3 enable or disable the limit check
    JSON_Arduino_SendAll(0, json_type);									//read all data

  }


  //PORT_I= timer number

  //8 bits on action_i: out port: --> 6 bits

  //TIMER CONFIGURATION: action2 (16 bit) & action3 (16 bit)
  //action2_i
  //week: bit[6:0] S-F-T-W-T-M-S --> 7 bit (0 if is selected, 1 if is unselected, 1111111=means not used)
  //repeat: bit[7]  --> 1 bit
  //hour: bit[12:8] 0-23 --> 5 bit
  //NU: bit[13]  --> 1 bit
  //NU: bit[14]  --> 1 bit
  //NU: bit[15]  --> 1 bit

  //action3_i
  //min: bit[3:0] 5 min --> 4 bit (12)
  //duration: bit[11:4] 5 min for each hour (max 8)  --> 4+4=8  bit
  //NU: bit[15:12]  --> 4 bit

  //5 bytes for each timer

  //action2_i=0x0055 (85)
  //week: 0x55 --> 01010101 (0, SATURDAY(YES), FRIDAY, THURSDAY(YES), WEDNESDAY, TUESDAY(YES), MONDAY, SUNDAY(YES))
  //hour: 0x0 --> 0 hour
  //
  //action3_i=0x065 (101)
  //min: 0x5 --> 5 --> 5*5=25 minutes
  //duration: 0x6, 6*5=30min and 0 hours --> 0 hours and 30minutes
  //http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=Tmr&port=0&action=7&action2=85&action3=101&action4=0
  else if (strcmp(cmd, "Tmr") == 0) {                            //timers setting
    //port_i: --> timer number
    //action_i --> output port
    //action2 --> 32BITS: timer config
    uint16_t address;
    address = FLA_TIMERS_ADDRESS + (BYTES_FOR_EACH_TIMER * port_i);
    //timer number --> address index
    //port number  --> address 0
    //repeat + week --> address 1
    //hour --> address 2
    //min --> address 3
    //duration --> address 4

    /*#ifdef ARDUINO_DUE == 3
     dueFlashStorage.write(address,action_i);
     dueFlashStorage.write(address+1, (byte)((action2_i & 0xFF)));
     dueFlashStorage.write(address+2, (byte)((action2_i & 0x1F00)>>8));
     dueFlashStorage.write(address+3, (byte)((action3_i & 0xF)));
     dueFlashStorage.write(address+4, (byte)((action3_i & 0xFF0)>>4));
     dueFlashStorage.write(address+5, (byte)(action4_i & 0xFF));
     #else*/
    eeprom_write_byteNEW (address, (byte)action_i);                                               //port number (6 bit)
    eeprom_write_byteNEW ((address + 1), (byte)((action2_i & 0xFF)));                          //repeat + week bit  (8bit)
    eeprom_write_byteNEW ((address + 2), (byte)((action2_i & 0x1F00) >> 8));                   //hour bit (5 bit)
    eeprom_write_byteNEW ((address + 3), (byte)((action3_i & 0xF)));                           //min bit (4 bit)
    eeprom_write_byteNEW ((address + 4), (byte)((action3_i & 0xFF0) >> 4));                    //duration bit (8 bit)
    eeprom_write_byteNEW ((address + 5), (byte)(action4_i & 0xFF));                            //duration bit 2th byte (8 bit)
    //#endif


    JSON_Arduino_SendAll(2, json_type);                                                                          //actions --> see below JSON_Arduino_SendAll descriptions (read only systems)
  }

  //http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=RdFla&port=50&action=70&action2=0&action3=0&action4=0
  //read address from 50 to 69
  else if (strcmp(cmd, "RdFla") == 0) {                            //read flash
    //port_i: --> address
    for (unsigned int i = port_i; i < action_i; i++) {
      uint8_t tmp;

      tmp = eeprom_read_byteNEW (i);

      _client.print("A:"); _client.print(i); _client.print(",Read:"); _client.println(tmp);
    }
  }
  //http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=ErFla&port=50&action=100&action2=255&action3=0&action4=0
  else if (strcmp(cmd, "ErFla") == 0) {                            //read flash
    //port_i: --> address
    for (unsigned int i = port_i; i < action_i; i++) {
      //#ifdef ARDUINO_DUE == 1
      //            dueFlashStorage.write(i, (byte)(action2_i));
      //#else
      eeprom_write_byteNEW (i, action2_i);
      //#endif

      _client.print("A:"); _client.print(i); _client.print(",Set: "); _client.println(action_i);
    }
  }
  //http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=ReadAll&port=0&action=0&action2=0&action3=0&action4=0           //read all sensors
  else if (strcmp(cmd, "ReadAll") == 0) {                         //Read All command is decoded
    JSON_Arduino_SendAll(action3_i, json_type);						     //actions --> see below JSON_Arduino_SendAll descriptions
  }
  //http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=rst&port=0&action=0&action2=0&action3=0&action4=0           //reset Arduino
  else if (strcmp(cmd, "rst") == 0) {
    //Riavvia();
  }

  //http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=sync&port=0&action=0&action2=0&action3=0&action4=65000           //read all sensors
  else if (strcmp(cmd, "sync") == 0) {                             //sync date & reset timers if the first time
    seconds_counter = port_i;
    minutes_counter = action_i;
    hours_counter = action2_i;
    days_counter = action3_i;

    //if action4_i>0 means that a new hw board is found, so clear all the flash locations
    if (action4_i > 0) {
      eeprom_write_wordNEW ((FLA_ANDRUINO_ID_ADDRESS), action4_i);                                          //write the Andruino App ID on flash


      //erase all timers
      for (int index = 0; index < MAX_TIMERS; index++) {
        eeprom_write_byteNEW ((FLA_TIMERS_ADDRESS + BYTES_FOR_EACH_TIMER * index), 255);                    //reset all the timers
        delay(10);
      }

      //reset all the IO and all the nofification enabled
      // TO BE DONE

      //update andruino_app_id vars
      andruino_app_id = action4_i;
    }

    JSON_Arduino_SendAll(6, json_type);                                    //actions --> see below JSON_Arduino_SendAll descriptions (read only systems)
  }
  //http://192.168.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=sync&port=0&action=0&action2=0&action3=0&action4=65000           //read all sensors
  else if (strcmp(cmd, "sync2") == 0) {                             //sync date & reset timers if the first time

    eeprom_write_wordNEW ((FLA_ANDRUINO_VERSION), port_i);    //write the Andruino App version on flash
    andruino_app_version = port_i;
    JSON_Arduino_SendAll(6, json_type);
  }
  //http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=Pin&port=2000&action=0&action2=0&action3=0&action4=1               //push test and pin update
  //http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=Pin&port=2000&action=0&action2=0&action3=0&action4=0               //pin update
  else if (strcmp(cmd, "Pin") == 0) {                             //Store the USER and PIN
    //Store the USER PIN PUSH and read back
    eeprom_write_wordNEW ((FLA_PUSH_PIN_ADDRESS), port_i); // Store the PUSH PIN in flash (PIN is in port slot)
    eeprom_write_blockNEW(action, FLA_PUSH_USER_ADDRESS, strlen(action) + 1); // Store the PUSH USER in flash (USER is in action slot)
    int old_pin = pin_push_flash;
    ReadPushUserPin();                 //update pin and user



    //read Andruino ID
    andruino_app_id = eeprom_read_wordNEW ((FLA_ANDRUINO_ID_ADDRESS));	 //read the Andruino App ID on flash

    if (action4_i  == 1 || old_pin != pin_push_flash) {          //send a notification if the pin is new or as a new request
      send_push_msg = true;                                    //send a TEST push notification
      force_pushddns = true;                                   //send the WAN IP to the server to store it in the new account
    }
    JSON_Arduino_SendAll(6, json_type);                                    //actions --> see below JSON_Arduino_SendAll descriptions (read only systems)
  }
  else {
    JSON_Arduino_SendAll(0, json_type);									//read all data
  }


  ClientFlush();

}

void ANDRUINO_JSON::JSON_ClientFlush()
{
  ClientFlush();
}

/////////////////////////////////////
//read USER and PIN PUSH from flash
void ANDRUINO_JSON::ReadPushUserPin()
{
  pin_push_flash = eeprom_read_wordNEW ((FLA_PUSH_PIN_ADDRESS));
  eeprom_read_blockNEW (push_user, FLA_PUSH_USER_ADDRESS, 15);

}


void ANDRUINO_JSON::RemoteDigitalWrite(byte indexV, int duration)                                                //modify the digital pin state (output high or low, or put the buffer in input mode)
{
  // Switch port as requested
  if (strcmp(action, "1") == 0) {
    pin_types.writeDig(ArduinoIO[indexV], true);                               //configures the pin as output and put the pin high
    ArduinoIO[indexV].time_counter = duration;
  } else if  (strcmp(action, "0") == 0) {
    pin_types.writeDig(ArduinoIO[indexV], false);                              //configures the pin as output and put the pin low
  } else if  (strcmp(action, "0P") == 0 || strcmp(action, "1P") == 0) {             //a pulse request is arrived
    pin_types.pulse(ArduinoIO[indexV], PIN_PULSE_WIDTH);                                //A Pulse is given
  }
}



#if ZIGBEE_ENABLE == 1
void ANDRUINO_JSON::RemoteDigitalWriteZigBee(uint8_t port, byte indexZigBeeModule)                                                //modify the digital pin state (output high or low, or put the buffer in input mode)
{

  /*
   uint8_t OutValue0[] = {
   0x4 };                    //output, default low
   uint8_t OutValue1[] = {
   0x5 };                    //output, default high
   uint8_t AnalogVaue[] = {
   0x2 };                    //analog
   uint8_t InValue[] = {       //input pin
   0x3 };
   uint8_t ONE[] = {           //0x01
   0x1 };
   uint8_t ZERO[] = {           //0x00
   0x0 };
   */
  // Turn on I/O sampling
  uint8_t irCmd[] = {
    'I', 'R'
  };
  // Set sample rate to 65 seconds (0xffff/1000)
  uint8_t irValue[] = {
    0x03, 0xe8
  };       //1 sec of sample rate

  uint8_t OutValue[1];
  uint8_t dCmd[2] = { 'D', '2' };

  dCmd[1] = 48 + port;			//int to ascii (from D0 to D9)

  // Switch port as requested
  if (strcmp(action, "1") == 0) {
    OutValue[0] = 0x5;                  //output, high
  } else if  (strcmp(action, "0") == 0) {
    OutValue[0] = 0x4;                  //output, low
  }

  XBeeAddress64 remoteAddress = XBeeAddress64(SystemXBeePins[indexZigBeeModule].RemoteAddrMSB, SystemXBeePins[indexZigBeeModule].RemoteAddrLSB);      //seleziono lo xbee

  RemoteAtCommandRequest remoteAtRequest = RemoteAtCommandRequest(remoteAddress, dCmd, OutValue, 1);              //0x5 means outputH, 0x4 means outputL, 1 is lenght
  sendRemoteAtCommand(remoteAtRequest);

  //not needed
  //   remoteAtRequest = RemoteAtCommandRequest(remoteAddress, irCmd, irValue, sizeof(irValue));
  //   sendRemoteAtCommand(remoteAtRequest);

}
#endif







//////////////////
//Read Arduino
//0 --> read all datas
//1 --> read Arduino All IO and systems (analogs + digitals)
//2 --> read Arduino IO and systems (digitals)
//3 --> read Arduino IO and systems (analogs)
//4 --> read XBee IO and systems
//5 --> read Variables IO and systems
//
void ANDRUINO_JSON::JSON_Arduino_SendAll(byte mode, bool json_type)
{
  ClientPrint("{");            //open first bracket

  switch (mode) {

    case 1:
      JSON_Arduino_io(0, json_type);           //read All Arduino IO (digital + analog)
      ClientPrint(",");
      JSON_Arduino_json_System();          //read Arduino system vars
      break;
    case 2:
      JSON_Arduino_io(1, json_type);           //read Arduino IO (digital)
      ClientPrint(",");
      JSON_Arduino_json_System();          //read Arduino system vars
      break;
    case 3:
      JSON_Arduino_io(2, json_type);           //read Arduino IO (analog)
      ClientPrint(",");
      JSON_Arduino_json_System();          //read Arduino system vars
      break;
    case 4:
#if ZIGBEE_ENABLE == 1
      for (byte k = 0; k < XBeeMaxModules; k++) {
        if (SystemXBeePins[k].RemoteAddrLSB != 0) {
          JSON_Arduino_json_XbeeIO(SystemXBeePins[k].RemoteAddrLSB, k, json_type);          //read ZIGBee IO, full json (false)
          ClientPrint(",");
        }
      }
#endif
      JSON_Arduino_json_System();          //read Arduino system vars
      break;

    case 5:
      JSON_Arduino_json_Vars(json_type);          //read Arduino Variables
      ClientPrint(",");
      JSON_Arduino_json_System();          //read Arduino system vars
      break;

    case 6:
      JSON_Arduino_json_System();          //read Arduino system vars
      break;


    default:
      JSON_Arduino_io(0, json_type);           //read Arduino IO
      ClientPrint(",");
#if ZIGBEE_ENABLE == 1
      for (byte k = 0; k < XBeeMaxModules; k++) {
        if (SystemXBeePins[k].RemoteAddrLSB != 0) {
          JSON_Arduino_json_XbeeIO(SystemXBeePins[k].RemoteAddrLSB, k, json_type);          //read ZIGBee IO, full json (false)
          ClientPrint(",");
        }
      }
#endif
#if NRF24L_ENABLE == 1
      for (byte k = 0; k < NRF24LMaxModules; k++) {
        if (SystemNRF24LPins[k].RNF24LAddr != 0) {
          JSON_Arduino_json_NRF24Lio(k,json_type);          //read NRF24L IO, full json
          ClientPrint(",");
        }
      }
#endif
      JSON_Arduino_json_Vars(json_type);          //read Arduino Variables
      ClientPrint(",");
      JSON_Arduino_json_System();          //read Arduino system vars
      //break;
  }
  ClientPrint("}");               //close last bracket
  delay(1);


}

void ANDRUINO_JSON::JSON_comma() {
  ClientPrint(",");                     //
}


void ANDRUINO_JSON::JSON_Arduino_io(byte mode, bool json_type)
{
  ClientPrint("\"arduino_io\":{\"INFO\":[\"");                  //open JSON bracket, INFO ARDUINO

  JSON_PrintDataComma(network_access, 0);
  ClientPrint("\"],");
  switch (mode) {
    case 1:
      JSON_Arduino_json_IO(json_type);
      break;
    case 2:
      JSON_Arduino_json_Analog(json_type);
      break;
    case 4:
      JSON_Arduino_json_System();                //read Arduino system vars
      break;
    default:
      JSON_Arduino_json_IO(json_type);
      ClientPrint(",");
      JSON_Arduino_json_Analog(json_type);
  }
  ClientPrint("}");                             //close JSON bracket
}


#if NRF24L_ENABLE == 1
void ANDRUINO_JSON::JSON_NRF24L(bool json_type)
{
  bool second_data = false; 
  
  ClientPrint("\"arduino_nrf24l\":{");                  //open JSON bracket, INFO ARDUINO

  for (byte k = 0; k < NRF24LMaxModules; k++) {
    if (SystemNRF24LPins[k].RNF24LAddr != 0) {
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;  
        
      JSON_Arduino_json_NRF24Lio(k, json_type);          //read NRF24L IO, short json (true)
    }
  }

  ClientPrint("}");                             //close JSON bracket
}
#endif

#if ZIGBEE_ENABLE == 1
void ANDRUINO_JSON::JSON_XBEE(bool json_type)
{
  bool second_data = false; 
  
  ClientPrint("\"arduino_xbee\":{");                  //open JSON bracket, INFO ARDUINO

      for (byte k = 0; k < XBeeMaxModules; k++) {
        if (SystemXBeePins[k].RemoteAddrLSB != 0) {
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;  
        
        json.JSON_Arduino_json_XbeeIO(SystemXBeePins[k].RemoteAddrLSB, k, json_type);          //read ZIGBee IO, short json (true)
    }
  }

  ClientPrint("}");                                   //close JSON bracket
}
#endif



void ANDRUINO_JSON::JSON_Arduino_json_IO(bool type_json)
{
  byte ReadChannel;
  //    float analogValue;
  int sensorReading;
  boolean second_data = false;
  //Arduino IO JSON STREAM
  ClientPrint("\"DIGITALS\":[");                  //open JSON bracket
  second_data = false;
  for (int i = 0; i < MAXPIN; i++) {
    if (ArduinoIO[i].used == 1) {
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;
      //INPUT DIGITAL
      if (ArduinoIO[i].mode == 0) {                   //0=in
        ReadChannel = pin_types.read(ArduinoIO[i]).state;                       //read input/output pins
        JSON_FormatDigital(ArduinoIO[i].pin, "in", ReadChannel, ArduinoIO[i].pulse, type_json);
        //OUTPUT DIGITAL
      } else if (ArduinoIO[i].mode == 1) {            //1=out
        ReadChannel = pin_types.read(ArduinoIO[i]).state;                       //read input/output pins
        JSON_FormatDigital(ArduinoIO[i].pin, "out", ReadChannel, ArduinoIO[i].pulse, type_json);
        ArduinoIO[i].pulse = false;
        //PWM DIGITAL
      } else if (ArduinoIO[i].mode == 2) {
        ReadChannel = ArduinoIO[i].state;                       //read pwm var
        JSON_FormatDigital(ArduinoIO[i].pin, "pwm", ReadChannel, 0, type_json);
      }
    }
  }
  ClientPrint("]");                     //close JSON bracket

}

void ANDRUINO_JSON::JSON_Arduino_json_Analog(bool type_json)
{
  byte ReadChannel;
  //    float analogValue;
  int sensorReading;
  boolean second_data = false;
  ClientPrint("\"ANALOGS\":[");
  second_data = false;
  for (int i = 0; i < MAXANA; i++)
  {
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;
      //sensorReading = pin_types.readAnalog(ArduinoAnalog[i]).value;
      //analogValue = sensorReading * ADC_STEP;
      ArduinoAnalog[i].value = analogRead(ArduinoAnalog[i].pin) * ADC_STEP;
      
      JSON_FormatAnaVar(ArduinoAnalog[i].pin, "ana", ArduinoAnalog[i].value,type_json);    //full or short mode json
 
  }
  ClientPrint("]");                     //close JSON bracket
}




void ANDRUINO_JSON::JSON_Arduino_json_XbeeIO(uint32_t UDID_LSB, byte index, bool type_json)
{
//type_json=false  --> full json  
//type_json=true  --> short json  
  
  
  // Send requested information
  byte ReadChannel;
  //    float analogValue;
  float sensorReading;
  boolean second_data = false;

#if ZIGBEE_ENABLE == 1
  //XBee IO
  ClientPrint("\"XBee_io_");          //open JSON bracket
  //        ClientPrint(UDID_LSB,HEX);
  ClientPrint(index);
  ClientPrint("\":{\"INFO\":[\"");          //INFO Xbee
  ClientPrint(SystemXBeePins[index].samples);       //(samples received)
  ClientPrint("\",\"");
  ClientPrint(UDID_LSB, HEX);                       //LSB UID
  ClientPrint("\"],\"DIGITALS\":[");          //DIGITALS
  second_data = false;
  for (int i = 0; i < XBeeMaxPinModule; i++)
  {
    if (SystemXBeePins[index].Pin[i].used && SystemXBeePins[index].Pin[i].mode != 2) {        //find digital input/outputs
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;

      if (SystemXBeePins[index].Pin[i].mode == 0) {
        ReadChannel = (byte) SystemXBeePins[index].Pin[i].value;         //read  DIGITAL pins (PREVIOUSLY READ from var)
        JSON_FormatDigital(SystemXBeePins[index].Pin[i].pin, "in", ReadChannel, 0,type_json);
      } else {
        ReadChannel = (byte) SystemXBeePins[index].Pin[i].value;         //read digital outputs
        JSON_FormatDigital(SystemXBeePins[index].Pin[i].pin, "out", ReadChannel, 0,type_json);
      }
    }
  }
  ClientPrint("],\"ANALOGS\":[");
  second_data = false;
  for (int i = 0; i < XBeeMaxPinModule; i++)
  {
    if (SystemXBeePins[index].Pin[i].used && SystemXBeePins[index].Pin[i].mode == 2) {        //find analog
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;

      sensorReading = SystemXBeePins[index].Pin[i].value;     //read  ANALOG  pins (PREVIOUSLY READ from var)
      JSON_FormatAnaVar(SystemXBeePins[index].Pin[i].pin, "ana", sensorReading, type_json);
    }
  }
  ClientPrint("]}");                      //close JSON bracket
#endif
}

void ANDRUINO_JSON::JSON_Arduino_json_Vars(bool type_json)
{
  byte ReadChannel;
  float analogValue;
  int sensorReading;
  boolean second_data = false;
  //ARDUINO VARIABLES
  

  ClientPrint("\"arduino_var\":{\"VARIABLES\":[");  //open JSON bracket

  second_data = false;
  for (int i = 0; i < ARDUINO_USER_VAR_MAX; i++)
  {
    if (second_data == true) {
      ClientPrint(",");
    }
    else
      second_data = true;
      
    JSON_FormatAnaVar(i, "var", Arduino_User_var[i].value, type_json);                      //short mode json  or short mode json
 
  }

  ClientPrint("]}");                //close JSON bracket 


}


void ANDRUINO_JSON::JSON_Arduino_json_System()
{


  ////////////////////////////////
  //ARDUINO TIMERS
  boolean second_data = false;
  ClientPrint("\"ardu_tim\":[");  //open JSON bracket
  for (int i = 0; i < MAX_TIMERS; i++)
  {
    if (second_data == true) {
      ClientPrint(",");
    }
    else
      second_data = true;

    uint8_t port_number;                //255 means timer not used
    port_number = eeprom_read_byteNEW (FLA_TIMERS_ADDRESS + BYTES_FOR_EACH_TIMER * i);
    ClientPrint(port_number);
  }
  // ClientPrint("],");               //close JSON bracket

  ////////////////////////////////
  //Date and feeds
  ////////////////////////////////
  //ardu_date
  ClientPrint("],\"ardu_sys\":{\"ardu_date\":[");
  JSON_PrintDataComma(days_counter, 1);
  JSON_PrintDataComma(hours_counter, 1);
  JSON_PrintDataComma(minutes_counter, 1);
  JSON_PrintDataComma(seconds_counter, 0);
  ClientPrint("]");
  //ardu_fd2
  ClientPrint(",\"ardu_fd2\":[\"");
  ClientPrint(push_user);
  ClientPrint("\",\"");
  ClientPrint(pin_push_flash);
  ClientPrint("\",\"");
  ClientPrint(ddns_address);
  ClientPrint("\",\"");
  ClientPrint(andruino_app_id);            //new App 1.7
  ClientPrint("\"]");
  //ardu_fd
  ClientPrint(",\"ardu_fd\":[");
  JSON_PrintDataComma(network_access, 1);
  JSON_PrintDataComma(ddns_success_cnt, 1);
  JSON_PrintDataComma(internet_fail_cnt, 1);          //non usato
  JSON_PrintDataComma(push_success_cnt, 1);
  JSON_PrintDataComma(push_fail_cnt, 1);
  JSON_PrintDataComma(feedD, 1);
  //send version LIB
  ClientPrintFloat(version_sketch, 3);
  ClientPrint(",");
  ClientPrintFloat(version_sketch, 3);                      //not yet used
  ClientPrint("]}");


}


//type = 1  {["3","pwm","0"]}  port/mode/value
//type = 0  {"port":"dig3","mode":"pwm","val":"0"}
void  ANDRUINO_JSON::JSON_FormatDigital (byte port, char *mode, byte value, boolean pulse, bool type_json) {
  char buffer[50];
  
  if(!type_json) {
    sprintf(buffer, "{\"port\":\"dig%d\",\"mode\":\"%s\",\"val\":\"%d", port, mode, value); //full mode json
  }
  else { 
    sprintf(buffer, "[\"%d\",\"%s\",\"%d\",\"%d\"]", port, mode, value,pulse);                          //short mode json 
    ClientPrint(buffer);
    return;
  }
  
  
  ClientPrint(buffer);
  delay(DELAY_TX_ETHERNET);
  if (pulse == false)
    ClientPrint("\"}");
  else {
    ClientPrint("P\"}");
  }
}







//{"port":"var0","mode":"var","val":"?"}
/*void  ANDRUINO_JSON::JSON_FormatAna (byte port,char *mode, unsigned int value) {
char buffer[50];
//mode=var, ana
sprintf(buffer,"{\"port\":\"%s%d\",\"mode\":\"%s\",\"val\":\"%d\"}",mode, port,mode,value);
ClientPrint(buffer);
delay(DELAY_TX_ETHERNET);
}*/


//full mode json --> type_json=false
//short mode json --> type_json=true
void  ANDRUINO_JSON::JSON_FormatAnaVar (byte port, char *mode, float value, bool type_json) {
  char buffer[50];
  //mode=var, ana
  if(!type_json) 
    sprintf(buffer, "{\"port\":\"%s%d\",\"mode\":\"%s\",\"val\":\"", mode, port, mode);        //full mode json 
  else { 
    ClientPrintFloat(value, 3);                                                               //short mode json  
    return;
  }    
    
  ClientPrint(buffer);
  delay(DELAY_TX_ETHERNET);
  ClientPrintFloat(value, 3);      //3 digits
  ClientPrint("\"}");
}


 


void  ANDRUINO_JSON::JSON_PrintDataComma (char *datas, bool comma) {
  ClientPrint(datas);
  if (comma)
    ClientPrint(",");
  //  delay(10);
}
void  ANDRUINO_JSON::JSON_PrintDataComma (unsigned int datas, bool comma) {
  ClientPrint(datas);
  if (comma)
    ClientPrint(",");
  //  delay(10);
}


#if NRF24L_ENABLE == 1
void ANDRUINO_JSON::JSON_Arduino_json_NRF24Lio(byte index, bool type_json)
{
  // Send requested information
  byte ReadChannel;
  //    float analogValue;
  float sensorReading;
  boolean second_data = false;


  //NRF24L IO
  ClientPrint("\"NRF24L_io_");                                                //open JSON bracket
  ClientPrint(index);
  ClientPrint("\":{\"INFO\":[\"");                                            //INFO NRF
  ClientPrint(SystemNRF24LPins[index].samples);                               //(samples received)
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].RNF24LAddr, OCT);                       //UID
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].Supply, 2);                             //Supply
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].sleep_time_value);                      //sleep time remote module
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].alive_counter);                         //alive counter
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].alive_fail_counter);                    //alive fail counter
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].fw_version);                            //firmware
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].pa_level);                              //pa level of tx
  ClientPrint("\",\"");
  ClientPrint(SystemNRF24LPins[index].leaf);                                  //leaf or router
  ClientPrint("\"],\"DIGITALS\":[");                                          //DIGITALS
  second_data = false;
  for (int i = 0; i < NRF24LMaxDigitalPin; i++)
  {
    if (SystemNRF24LPins[index].DigPin[i].used && SystemNRF24LPins[index].DigPin[i].mode != 2) {        //find digital input/outputs
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;

      if (SystemNRF24LPins[index].DigPin[i].mode == 0) {
        ReadChannel = (byte) SystemNRF24LPins[index].DigPin[i].state;         //read  DIGITAL pins (PREVIOUSLY READ from var)
        JSON_FormatDigital(SystemNRF24LPins[index].DigPin[i].pin, "in", ReadChannel, 0, type_json);
      } else {
        ReadChannel = (byte) SystemNRF24LPins[index].DigPin[i].state;         //read digital outputs
        JSON_FormatDigital(SystemNRF24LPins[index].DigPin[i].pin, "out", ReadChannel, 0, type_json);
        //////// IF_SERIAL_DEBUG_NRF(printf_P(PSTR("@@@@@@@@@@@@@@@@@NRF pin sent in JSON : %d, value:%d, indexModule:%d\r\n"), SystemNRF24LPins[index].DigPin[i].pin, ReadChannel, index));
      }
    }
  }
  ClientPrint("],\"ANALOGS\":[");
  second_data = false;
  for (int i = 0; i < NRF24LMaxAnalogPin; i++)
  {
    if (SystemNRF24LPins[index].AnaPin[i].used) {        //find analog
      if (second_data == true)
        ClientPrint(",");
      else
        second_data = true;

      sensorReading = ((float)SystemNRF24LPins[index].AnaPin[i].state) * SystemNRF24LPins[index].AdcStep;   //read  ANALOG  pins (PREVIOUSLY READ from var)
      JSON_FormatAnaVar(SystemNRF24LPins[index].AnaPin[i].pin, "ana", sensorReading, type_json);
    }
  }
  ClientPrint("]},\"NRF24L_var_");                                                //open JSON bracket
  ClientPrint(index);  
  ClientPrint("\":{\"VARIABLES\":[");
  second_data = false;
  for (int i = 0; i < SystemNRF24LPins[index].var_num; i++)
  {
    if (second_data == true)
      ClientPrint(",");
    else
      second_data = true;
    JSON_FormatAnaVar(i, "var", SystemNRF24LPins[index].VarPin[i].value, type_json);
  }
  ClientPrint("]}");                      //close JSON bracket
}


void ANDRUINO_JSON::RemoteDigitalWriteNRF24L(uint8_t port, uint8_t value, byte indexModule)                                               //modify the digital pin state (output high or low, or put the buffer in input mode)
{
  bool ok;
  ANDRUINO_NRF nrf;
  //port  2,3,10
  //indexModule: 0,1,2,3

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("#####################################################################DIGITAL OUT PIN preparing data for the remote sensor\r\n")));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("port: %d, value:%d, indexModule:%d\r\n"), port, value, indexModule));

  short j = SearchRemoteNRF24LPin(indexModule, port);
  if (j < 0) {
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("NRF module not found on the list\r\n")));
    return;

  }

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Remote pin found: IndexPin:%d, state%d\r\n"), j, value));


  //If router, send immediatly the pin update
  if (SystemNRF24LPins[indexModule].leaf == 0) {
    ok = nrf.send_digital_out(SystemNRF24LPins[indexModule].RNF24LAddr, 0, SystemNRF24LPins[indexModule].DigPin[j].pin, value);
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN HAS BEEN TRASMITTED TO BASE\r\n")));
    //if ok, means that the data has bben trasmited to NODE, so update the vector and send back to APP
    if (ok) {
      SystemNRF24LPins[indexModule].DigPin[j].state = value;
    }
  }
  //LEAF NODE
  else {
    ok = nrf.send_digital_out(SystemNRF24LPins[indexModule].RNF24LAddr, 0, SystemNRF24LPins[indexModule].DigPin[j].pin, value);     //tx data immediatly
    SystemNRF24LPins[indexModule].DigPin[j].W = true;                                                                               //write request if the LEAF is sleeping
    SystemNRF24LPins[indexModule].DigPin[j].Wstate = value;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN WRITE REQUEST HAS BEEN RECORDED\r\n")));
  }




}

//search pin in the RF module
//return the index of the array
//-1 if it is not found
short ANDRUINO_JSON::SearchRemoteNRF24LPin (byte index_node, uint8_t port) {

  short j;
  for (j = 0; j < NRF24LMaxDigitalPin; j++)  {
    if (SystemNRF24LPins[index_node].DigPin[j].mode == OUTPUT &&  SystemNRF24LPins[index_node].DigPin[j].used == true  && SystemNRF24LPins[index_node].DigPin[j].pin == port)    //if it is output and is used
      return (j);
  }
  return (-1);
}


//NRF24L Command received from http
//command=0 -->  sleep time setting (for each nrf module)   //http://10.0.1.15:8888/IO.json?user=arduino&pass=andrea&cmd=NRF24L&port=0&action=0&action2=60&action3=0&action4=0
//command=1 -->  NRF radio enable or disable (NRF base)
//command=2 -->  delete all the nodes from the flash list

// RemoteCommandNRF24L(port_i, action_i, action2_i, action3_i, action4_i);
void ANDRUINO_JSON::RemoteCommandNRF24L(byte command, unsigned int nrf_module, unsigned int first, unsigned int second, unsigned int third) {

  switch (command) {
    case 0:                                                         //sleep time setting
      SystemNRF24LPins[nrf_module].sleep_time_value = first;
      SystemNRF24LPins[nrf_module].system_back_wr = true;             //write request

      break;
    case 1:             //switch OFF NRF radio system
      if (first > 0) {
        eeprom_write_byteNEW (FLA_NRF_ENABLE_ADDRESS, 0x00);      //NRF ON
        nrf_radio_enable_rq = true;
        nrf_radio_disable_rq = false;
      } else
      {
        eeprom_write_byteNEW (FLA_NRF_ENABLE_ADDRESS, 0x55);      //NRF OFF
        nrf_radio_enable_rq = false;
        nrf_radio_disable_rq = true;
      }
      break;
    case 2:                                                       //clear flash node list
      for (byte i = 0; i < NRF24LMaxModules; i++)
        eeprom_write_wordNEW ((FLA_NRF_NODE_ADDRESS + i * 2), 65535);
      break;
      //   default:
  }

}
#endif
/*
 
//arduino_io -->INFO
//              ANALOGS
//
//arduino_var-->VARIABLES
//
//ardu_sys  --> ardu_date
//              ardu_fd2
//              ardu_fd
                
//arduino_nrf24l:
//                  NRF24L_io_0 --> INFO
//                                  DIGITALS
//                                  ANALOGS
//
//                  NRF24L_var_0--> VARIABLES 
 
 
{
"arduino_io":{"INFO":["3"],"DIGITALS":[["3","pwm","0","0"],["4","out","1","0"],["5","out","0","0"],["6","out","0","0"],["7","out","0","0"],["8","out","0","0"],["9","in","0","0"],["10","out","1","0"],["11","out","0","0"],
["12","out","0","0"],["13","out","0","0"],["14","out","0","0"],["15","out","0","0"],["16","out","0","0"],["17","out","0","0"],["18","out","0","0"],["19","out","0","0"],["20","out","0","0"],["21","out","0","0"],["22","in","0","0"],["23","in","0","0"],["24","in","0","0"],["25","in","0","0"],["26","out","0","0"],["27","out","0","0"],["28","out","0","0"],
["29","out","0","0"]], "ANALOGS":[2.464,2.416,2.075,1.835,1.739,1.743,1.666,1.532,1.412,1.354,1.321,1.253,1.287,1.239,1.297,1.153]},
"arduino_var":{"VARIABLES":[52.900,23.600,3.000,3423.000,0.000,0.183,0.289,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000]},

"NRF24L_io_0":{"INFO":["55381","02","3281","88","344","0","1001","2","0"],"DIGITALS":[],"ANALOGS":[]},
"NRF24L_var_0":{"VARIABLES":[0.000,0.000,0.000,0.000,55380.000]},

"ardu_tim":[255,255,255,255,255,255,255,255,255,255,255,255],
"ardu_sys":{"ardu_date":[0,9,15,33],"ardu_fd2":["andrea.scavu","9411","255.255.255.255","21414"],"ardu_fd":[3,0,0,1,0,0,7.000,7.000]}
} 
 
 
 
 
 
 
 */
