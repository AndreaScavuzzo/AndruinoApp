
////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_DATA_LOGGER.h"




bool  ANDRUINO_DATA_LOGGER::CheckPin()

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



//push.SendDataLogger(push_user, ARDUINO_NAME, ARDUINO_PASS, 0,0);

//0: analog+vars+system
//1: digital+System
//2: NRF module
//3: ZigBee module
//4: //system
bool  ANDRUINO_DATA_LOGGER::SendDataLogger(char *push_usr, char *arduino_name, char *arduino_pass, byte mode)
{
  bool second_data;
  if (!CheckPin()) {
    return 0;
  }

#if DEBUG_SERIAL_PUSH == 1
  Serial.println(F("http POST connecting.."));
#endif

//  char *result = 0;

  if (_client.connect("andruino.it", 80)) {    //andruino.it as number
#if DEBUG_SERIAL_PUSH == 1
    Serial.println(F("connected"));
#endif
    _client.print(F("GET /push3/data_logger"));
    _client.print(F("?user="));
    _client.print(push_usr);
    _client.print(F("&pin="));
    _client.print(pin_push_flash);
    _client.print(F("&ardu="));
    _client.print(arduino_name);
    _client.print(F("&pass="));
    _client.print(arduino_pass);
    _client.print(F("&mode="));
    _client.print(mode);
    _client.print(F("&json={"));
    switch (mode) {
      
      //analog+vars+system
      case 0:
        json.JSON_Arduino_io(2,true);              //2=arduino_io:analog, true=short json packet
        json.JSON_comma(); 
        json.JSON_Arduino_json_Vars(true);    
        json.JSON_comma();  
        json.JSON_Arduino_json_System();
        break;
        
      //digital+System
      case 1:
        json.JSON_Arduino_io(1,true);              //1=arduino_io:digital, true=short json packet
      //  json.JSON_comma();     
      //  json.JSON_Arduino_json_System();
        break;
        
      //NRF module
      case 2:
#if NRF24L_ENABLE == 1
        json.JSON_NRF24L(true);
#endif      
        break;

      //ZigBee module
      case 3:
#if ZIGBEE_ENABLE == 1
        json.JSON_XBEE(true);
#endif
        break;
        
      //system
      case 4:
        json.JSON_Arduino_json_System();
        break;
      default:
        break;
    }
    json.JSON_ClientFlush();          //check the data buffer, send the remain data and clean it

    _client.println(F("} HTTP/1.1"));
    _client.println(F("Host: andruino.it"));
    _client.println(F("Connection: close\r\nContent-Type: application/json"));
    _client.println(F("User-Agent: Andruino/1.0"));
    _client.println();


    int connectLoop = 0;
    int inChar;
    while (_client.connected())
    {
      while (_client.available())
      {
        inChar = _client.read();
 //       Serial.write(inChar);
        connectLoop = 0;
      }

      delay(1);
      connectLoop++;
      if (connectLoop > 10000)
      {
        Serial.println();
        Serial.println(F("Timeout"));
        _client.stop();
      }
    }

    Serial.println();
    Serial.println(F("disconnecting."));
    _client.stop();



  }
#if DEBUG_SERIAL_PUSH == 1
  else
    Serial.println(F("conn.failed"));
#endif


/*
  if (result) {
    push_success_cnt++;    //increment the counter of good push connection
    return true;
  }
  else {
    push_fail_cnt++;     //increment the counter of bad push connection
    return false;
  }
  
  */

}






/*

{"arduino_io":{"INFO":["0"],"ANALOGS":[2.414,2.276,1.795,1.533,1.371,1.443,1.352,1.290,1.148,1.067,1.028,0.971,1.033,1.009,1.143,1.138]},"arduino_var":{"VARIABLES":[41.100,23.700,0.000,3362.000,0.000,0.563,1.247,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000]},"ardu_tim":[255,255,255,255,255,255,255,255,255,255,255,255],"ardu_sys":{"ardu_date":[0,21,52,0],"ardu_fd2":["xxxxxxxx","0000","255.255.255.255","21414"],"ardu_fd":[0,0,0,1,0,0,7.090,7.090]}}

{"arduino_io":{"INFO":["0"],"DIGITALS":[["3","pwm","0"],["4","out","1"],["5","out","0"],["6","out","0"],["7","out","0"],["8","out","0"],["9","in","0"],["10","out","1"],["11","out","0"],["12","out","0"],["13","out","0"],["14","out","0"],["15","out","0"],["16","out","0"],["17","out","0"],["18","out","0"],["19","out","0"],["20","out","0"],["21","out","0"],["22","in","0"],["23","in","0"],["24","in","0"],["25","in","0"],["26","out","0"],["27","out","0"],["28","out","0"],["29","out","0"]]}}


{"arduino_nrf24l":{"NRF24L_io_0":{"INFO":["2","02","3290","88","340","0","1001","2","0"],"DIGITALS":[["3","in","1"],["4","in","1"],["5","out","0"],["9","out","0"]],"ANALOGS":[0.344,0.553,1.051,0.720]},"NRF24L_var_0":{"VARIABLES":[0.000,0.000,0.000,0.000,1.000]},"NRF24L_io_1":{"INFO":["1","012","2816","600","2396","0","1001","2","1"],"DIGITALS":[["3","in","1"],["4","in","1"],["5","out","0"],["9","out","0"]],"ANALOGS":[0.000,0.000,0.000,0.000]},"NRF24L_var_1":{"VARIABLES":[0.000,0.000,0.000,0.000,1.000]}}}


*/



