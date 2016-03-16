
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_JSON.h"


#define WIFI_SHIELD_MAX_PACKET 60  

char buffer_tx[WIFI_SHIELD_MAX_PACKET+2];



void ClientPrint(char* new_buffer);
void ClientPrint(unsigned int integer);
void ClientPrint(unsigned int integer, int base);
void ClientPrintFloat(float floating, int digit);


void ClientPrint(char* new_buffer) {

  int lungh_buff = strlen(buffer_tx);
  int lungh_char = strlen(new_buffer);
 // Serial.print("lungh_buff:"); Serial.print(lungh_buff); Serial.print(" lungh_char:"); Serial.print(lungh_char); Serial.print(" TOT:"); Serial.println((lungh_buff + lungh_char));
  if ((lungh_buff + lungh_char) > WIFI_SHIELD_MAX_PACKET) {
#if ARDUINO_YUN == 1
    _client.print(buffer_tx);          //tx the previous
#else
    _client.write(buffer_tx);          //tx the previous
#endif
  //  Serial.print("--->> Print buffer: "); Serial.println(buffer_tx);
    strcpy (buffer_tx, new_buffer);   //store the newone in the buffer for next tx
  //  if(lungh_buff >90)
  //    Serial.println("ERRRRRRROOOOORE");     
   
  }

  else {
    strcat (buffer_tx, new_buffer);
  }
}


void ClientPrintln(char* new_buffer) {
  ClientPrint(new_buffer);
  ClientPrint("\r\n"); 
}

void ClientPrint(unsigned int integer) {
  ClientPrint(integer,0);
}
void ClientPrintln(unsigned  int integer) {
  ClientPrint(integer);
  ClientPrint("\r\n");
}



void ClientPrint(unsigned int integer, int base) {
  char new_buffer[10];
  if(base == HEX)  {
    sprintf(new_buffer, "%X", integer);
  }
  else if (base == OCT)  {
    sprintf(new_buffer, "0%o", integer);
  }
  else {
    sprintf(new_buffer, "%u", integer);  
  }
  ClientPrint(new_buffer);
}

void ClientPrintln(unsigned  int integer, int base) {
  ClientPrint(integer, base);
  ClientPrint("\r\n");
}

void ClientPrintFloat(float floating, int digit) {
    char new_buffer[10];
    dtostrf(floating, 5, digit, new_buffer);
    ClientPrint(new_buffer);
   // Serial.print("float:"); Serial.println(new_buffer);
}
void ClientPrintFloatln(float floating, int digit) {
  ClientPrintFloat(floating, digit);
  ClientPrint("\r\n");
}


void ClientFlush() {

  int lungh_buff = strlen(buffer_tx);

  if (lungh_buff > 0) {
#if ARDUINO_YUN == 1
    _client.print(buffer_tx);          //tx the previous
#else
    _client.write(buffer_tx);          //tx the previous
#endif
    //Serial.print("--->> remaining buffer: "); Serial.println(buffer_tx);
  }
  strcpy (buffer_tx, "");
    
}

