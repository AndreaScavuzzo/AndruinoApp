////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_SERVICE.h"


#if ARDUINO_STDAVR == 1
int ANDRUINO_SERVICE::freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif


#if (DEBUG_SERIAL == 1 && WIFI_SHIELD == 1)
void ANDRUINO_SERVICE::printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.println(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.println(F("IP:"));
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.println(F("RSSI:"));
  Serial.println(rssi);
}
#endif




///////////////////
//Used to initialize all vectors to avoid ecc errors
///////////////////

void ANDRUINO_SERVICE::InitSensorArray() {



  memset(Arduino_User_var, 0, sizeof(Arduino_User_var));
  memset(ArduinoAnalog, 0, sizeof(ArduinoAnalog));
  memset(ArduinoIO, 0, sizeof(ArduinoIO));


  for (int i = 0; i < ARDUINO_USER_VAR_MAX; i++) {
    //      Arduino_User_var[i].value=0;
    //     Arduino_User_var[i].max=0;
    //     Arduino_User_var[i].min=0;
    //     Arduino_User_var[i].enable_limits=0;
    Arduino_User_var[i].limit_count = 128;  //up 128 means high limits is reached, down to 128 means low limit is reached
  }
  for (int i = 0; i < MAXANA; i++) {
    ArduinoAnalog[i].pin = i;					//used to index each pin
    /*     ArduinoAnalog[i].value=0;
         ArduinoAnalog[i].used=0;
         ArduinoAnalog[i].max=0;
         ArduinoAnalog[i].min=0;
         ArduinoAnalog[i].enable_limits=0;*/
    ArduinoAnalog[i].limit_count = 128;  //up 128 means high limits is reached, down to 128 means low limit is reached
  }
  for (int i = 0; i < MAXPIN; i++) {
    ArduinoIO[i].pin = i;					//used to index each pin
    /*  ArduinoIO[i].mode=0;
      ArduinoIO[i].state=0;
      ArduinoIO[i].pulse=0;
      ArduinoIO[i].used=0;
      ArduinoIO[i].limit_count=0;
      ArduinoIO[i].time_counter=0;		//used for timer
      ArduinoIO[i].alarm=0;
      ArduinoIO[i].enable_limits=0;       */
  }
#if ZIGBEE_ENABLE == 1
  for (byte k = 0; k < XBeeMaxModules; k++) {
    SystemXBeePins[k].RemoteAddrLSB = 0;
    SystemXBeePins[k].RemoteAddrMSB = 0;

    for (int i = 0; i < XBeeMaxPinModule; i++) {
      SystemXBeePins[k].Pin[i].pin = 0;           //pin number
      SystemXBeePins[k].Pin[i].mode = 0;          //0=input, 1=output, 2=analog
      SystemXBeePins[k].Pin[i].value = 0;         //pin state
      SystemXBeePins[k].Pin[i].used = 0;          //0=unused, 1 used

      SystemXBeePins[k].Pin[i].max = 0;
      SystemXBeePins[k].Pin[i].min = 0;
      SystemXBeePins[k].Pin[i].enable_limits = 0;
      SystemXBeePins[k].Pin[i].limit_count = 128;  //up 128 means high limits is reached, down to 128 means low limit is reached

      SystemXBeePins[k].samples = 0;              //reset samples received
    }
  }
#endif
}


void ANDRUINO_SERVICE::RestoreDataFromFlash() {
  uint16_t address;

  //read the date if the device has been power-off for a while
  uint8_t tmp;
  tmp = eeprom_read_byteNEW ((FLA_DATE_ADDRESS + 1));                              //hours
  if (tmp < 24) {
    days_counter = eeprom_read_byteNEW ((FLA_DATE_ADDRESS));						//days
    hours_counter = tmp;
    minutes_counter = eeprom_read_byteNEW ((FLA_DATE_ADDRESS + 2));				//minutes
  }

  //read Andruino ID
  andruino_app_id = eeprom_read_wordNEW ((FLA_ANDRUINO_ID_ADDRESS));

  //read Andruino App version
  andruino_app_version = eeprom_read_wordNEW ((FLA_ANDRUINO_VERSION));

  //NRF read enale
  if (eeprom_read_byteNEW (FLA_NRF_ENABLE_ADDRESS) !=0x55) {      //0x55 = OFF
    nrf_radio_enable_rq = true;
  }
  

  for (int i = 0; i < ARDUINO_USER_VAR_MAX; i++) {
    address = FLA_VAR_START_ADDRESS + MAX_BYTE_FOREACH_VAR * i;
    if (eeprom_read_byteNEW ((address)) == 0xAB) {
      //Serial.print(F("Found eeprom data, var: "));Serial.println(i);
      Arduino_User_var[i].max = eeprom_read_wordNEW ((address + 1));           //MAX 0-1
      Arduino_User_var[i].min = eeprom_read_wordNEW ((address + 3));           //MAX 2-3
      Arduino_User_var[i].enable_limits = eeprom_read_wordNEW ((address + 5)); //enable_limits 4-5
    }
  }

  for (int i = 0; i < MAXANA; i++) {
    address = FLA_ANALOG_START_ADDRESS + MAX_BYTE_FOREACH_ANA * i;
    if (eeprom_read_byteNEW ((address)) == 0xBC) {
      //Serial.print(F("Found eeprom data, analog: "));Serial.println(i);
      ArduinoAnalog[i].max = eeprom_read_wordNEW ((address + 1));            //MAX 0-1
      ArduinoAnalog[i].min = eeprom_read_wordNEW ((address + 3));           //MAX 2-3
      ArduinoAnalog[i].enable_limits = eeprom_read_wordNEW ((address + 5)); //enable_limits 4-5
    }
  }

  for (int i = 0; i < MAXPIN; i++) {
    address = FLA_DIGITAL_START_ADDRESS + MAX_BYTE_FOREACH_DIG * i;
    if (eeprom_read_byteNEW ((address)) == 0xCD) {
      ArduinoIO[i].state = eeprom_read_byteNEW ((address + 1));           	  //pin state

      //UPDATE PIN FROM FLASH
      if (ArduinoIO[i].mode == 1)														//mode1=OUT DIG
        digitalWrite(i, ArduinoIO[i].state);   										//write the DIG OUT pin from flash content
      else if (ArduinoIO[i].mode == 2)												//mode1=PWM DIG
        analogWrite(i, ArduinoIO[i].state);   										//write the pwm value from flash content


      //address_2 = alarm (bit12), polarity bit8, output bit7:2, limit type: bit1:0
      unsigned int address_2;
      address_2 = eeprom_read_wordNEW ((address + 2));   						//get the 16 bits

      if (address_2 & 0x1000)
        ArduinoIO[i].alarm = 1;												   		//extract alarm (H or L) from bit 12

      ArduinoIO[i].enable_limits = address_2 & 0x0FFF;								//extact limits infos (from bit 8 to 0)

      ArduinoIO[i].time_counter = eeprom_read_wordNEW ((address + 4));     	//counter for timers

    }
  }
}





//enable_limit has:
//limit_enable: push and/or output 00/01/10/11 (2 bits)
//port output number 6 bits (0-63)



void ANDRUINO_SERVICE::StoreDataToFlash() {
  uint16_t address;
#if DEBUG_SERIAL == 1
  Serial.println(F("StoreData"));
#endif



  //store date if the device will power-off for a while
  eeprom_write_byteNEW ((FLA_DATE_ADDRESS), days_counter);                        //days
  eeprom_write_byteNEW ((FLA_DATE_ADDRESS + 1), hours_counter);                   //hours
  eeprom_write_byteNEW ((FLA_DATE_ADDRESS + 2), minutes_counter);                 //min


  for (int i = 0; i < ARDUINO_USER_VAR_MAX; i++) {  //8 byte for each VAR
    address = FLA_VAR_START_ADDRESS + MAX_BYTE_FOREACH_VAR * i;
    eeprom_write_byteNEW ((address), 0xAB);                                    //signature (0)
    eeprom_write_wordNEW ((address + 1), Arduino_User_var[i].max);             //MAX 0-1 (1,2)
    eeprom_write_wordNEW ((address + 3), Arduino_User_var[i].min);             //MIN 2-3 (3,4)
    eeprom_write_wordNEW ((address + 5), Arduino_User_var[i].enable_limits);  //type_limits, enable_limits, port 4-5 (5,6)
  }
  for (int i = 0; i < MAXANA; i++) {              //8 byte for each ANALOG
    if (ArduinoAnalog[i].used) {
      address = FLA_ANALOG_START_ADDRESS + MAX_BYTE_FOREACH_ANA * i;
      eeprom_write_byteNEW ((address), 0xBC);                                  //signature
      eeprom_write_wordNEW ((address + 1), ArduinoAnalog[i].max);             //MAX 0-1
      eeprom_write_wordNEW ((address + 3), ArduinoAnalog[i].min);             //MAX 2-3
      eeprom_write_wordNEW ((address + 5), ArduinoAnalog[i].enable_limits);  //type_limits & enable_limits  4-5
    }

  }
  for (int i = 0; i < MAXPIN; i++) {              //6 bytes for each DIGITAL
    if (ArduinoIO[i].used) {
      address = FLA_DIGITAL_START_ADDRESS + MAX_BYTE_FOREACH_DIG * i;
      eeprom_write_byteNEW ((address), 0xCD);                                 //signature
      eeprom_write_byteNEW ((address + 1), ArduinoIO[i].state);          	 //alarm on H or L state 0


      //ENABLE LIMITS	word (16 bits)
      //BIT10=00 ->no limit enabled
      //BIT10=01 ->push enabled
      //BIT10=10 ->output enabled
      //BIT10=01 ->push/out enabled
      //BIT72=XXXXX ->output
      //BIT8=0/1 ->polarity limit
      //address_2 = alarm (bit12), polarity bit8, output bit7:2, limit type: bit1:0
      unsigned int address_2;
      address_2 = ArduinoIO[i].enable_limits & 0x0FFF;								//clear bit 12,13,14,15 (not used in enable limit)

      if (ArduinoIO[i].alarm)
        address_2 = address_2 | 0x1000;												//add the alarm bit on bit 12

      eeprom_write_wordNEW ((address + 2), address_2);        					//type_limits (7-8) & enable_limits  1-2
      eeprom_write_wordNEW ((address + 4), ArduinoIO[i].time_counter);        	//counter for timer

    }
  }

}



