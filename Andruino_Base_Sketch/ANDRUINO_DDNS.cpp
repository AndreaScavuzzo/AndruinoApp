////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_DDNS.h"


#if DYNAMIC_DDNS == 1
//************************************************************************************************//
//DDNS MANAGEMENT for Arduino Uno and Due
//************************************************************************************************//
/*
HTTP/1.1 200 OK
Content-Type: text/html
Server: DynDNS-CheckIP/1.0
Connection: close
Cache-Control: no-cache
Pragma: no-cache
Content-Length: 105

<html><head><title>Current IP Check</title></head><body>Current IP Address: 151.52.80.221</body></html>

*/

//return=0  --> old IP or error connection
//return=1  --> new IP
//<html><head><title>Current IP Check</title></head><body>Current IP Address: 94.37.92.137</body></html>

//http://checkip.dyndns.com/
bool ANDRUINO_DDNS::find_ddns() {


#if DEBUG_SERIAL_DDNS == 1
  Serial.println(F("checking ddns"));
#endif

  //connect to server
  if (_client.connect("216.146.43.70", 80)) {          //checkip.dyndns.com/
    // if (_client.connect("myexternalip.com", 80)) {          //checkip.dyndns.com/
#if DEBUG_SERIAL == 1
    Serial.println(F("Connected ddns"));
#endif
    //_client.print(F("GET /json"));
    _client.print(F("GET /"));

    ANDRUINO_PUSH push_i;
    char *start_address, *start_ip;
    char *end_ip;
    char buffer_rx[200];
    char read_address[16]="1.1.1.1";

    //result = push_i.FinishGetStream(buffer_rx, sizeof(buffer_rx), "Host: myexternalip.com", "X100:", 10);
    start_ip = push_i.FinishGetStream(buffer_rx, sizeof(buffer_rx), "Host: www.dyndns.com", "Address", 500);


    if (start_ip>0) {
      start_ip = (char *)start_ip + 9;
      end_ip = strstr((char *)start_ip, "<");
      
      read_address[0]=0;
      strncat(read_address, (char *)start_ip, end_ip - start_ip);    
      

      if (strcmp(read_address, ddns_address) == 0) {
#if DEBUG_SERIAL_DDNS == 1
        Serial.print(F("Same IP:")); Serial.println(read_address);
#endif
        return (0);
      } else {         //address read is different
#if DEBUG_SERIAL_DDNS == 1
        Serial.print(F("OLD IP:")); Serial.println(ddns_address);
#endif
        strcpy(ddns_address, read_address);
#if DEBUG_SERIAL_DDNS == 1
        Serial.print(F("New IP:")); Serial.println(ddns_address);
#endif
        return (1);
      }
    }
  }

  Serial.println(F("Error DDNS:"));
  internet_fail_cnt++;
  return (0);

}


/////////////////////////////////////////////////////////////////
//Check external WAN address and send it by email and/or by push
/////////////////////////////////////////////////////////////////
void ANDRUINO_DDNS::CheckDDNS_andSend_Notification(char *push_usr, char *arduino_name, bool force_pushddns) {



  //find_ddns, 1=when the address is changed (send email), 0=when the address is not changed (no email)
  //ANDRUINO_DDNS ddns_read;
  bool ddns_status =  find_ddns();  //read the external address of Arduino (update ddns_address) each minute

  if (ddns_status || force_pushddns) {           //read the external address of Arduino (update ddns_address) each minute
    force_pushddns = false;
    ANDRUINO_PUSH push;
    push.SendPush2(push_usr, arduino_name, "ip", ddns_address);            //send IP to Andruino.it
  }

}

#endif



