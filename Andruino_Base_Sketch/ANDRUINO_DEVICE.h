////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo Jan 2015
//www.andruino.it
//
//DON'T MODIFY THIS FILE!!!!!!!!!
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ANDRUINO_DEVICE_H
#define ANDRUINO_DEVICE_H


#if defined (__ARMEL__)                     //detect Arduino 2
#define ARDUINO_DUE 1
#define ARDUINO_2560MEGA 0
#define ARDUINO_UNO 0
#define ARDUINO_YUN 0

#elif defined (ARDUINO_AVR_YUN)             //detect Arduino YUN
#define ARDUINO_YUN 1
#define ARDUINO_2560MEGA 0
#define ARDUINO_STDAVR 1
#define ARDUINO_UNO 0
#elif defined (__AVR_ATmega2560__)         //detect Arduino 2560
#define ARDUINO_YUN 0
#define ARDUINO_STDAVR 1
#define ARDUINO_2560MEGA 1
#define ARDUINO_UNO 0
#else
#define ARDUINO_DUE 0                 //detect Arduino UNO
#define ARDUINO_2560MEGA 0
#define ARDUINO_STDAVR 1
#define ARDUINO_YUN 0
#define ARDUINO_UNO 1
#endif

#if ARDUINO_YUN == 1
#define ETHERNET_SHIELD 0                //disable Ethernet shield
#define ETHERNET_SHIELD_V2 0                //disable Ethernet shield
#define WIFI_SHIELD 0                    //disable Wifi shield
#endif


//Used to manage different hardware board as AVR(Uno/Mega/YUN/etc) and DUE board
#if ARDUINO_DUE == 1
#define  eeprom_read_byteNEW(x)  dueFlashStorage.read(x)
#define  eeprom_read_wordNEW(x) (dueFlashStorage.read(x+1))<<8 | dueFlashStorage.read(x)
#define  eeprom_read_dwordNEW(x) ((uint32_t)dueFlashStorage.read(x+3))<<24 | ((uint32_t)dueFlashStorage.read(x+2))<<16 | ((uint32_t)dueFlashStorage.read(x+1))<<8 | dueFlashStorage.read(x)

#define  eeprom_write_byteNEW(x,y)  dueFlashStorage.write(x,y)
#define  eeprom_write_wordNEW(x,y)  dueFlashStorage.write(x,y & 0xFF); dueFlashStorage.write(x+1,(y & 0xFF00)>>8)
#define  eeprom_write_dwordNEW(x,y)  dueFlashStorage.write(x,y & 0xFF); dueFlashStorage.write(x+1,(y & 0xFF00)>>8); dueFlashStorage.write(x+2,(y & 0xFF0000)>>16); dueFlashStorage.write(x+3,(y & 0xFF000000)>>24)


#define  eeprom_write_blockNEW(x,y,z)  byte wdata; for(byte i=0;i<z;i++) {wdata = x[i]; dueFlashStorage.write(y+i,wdata);}
#define  eeprom_read_blockNEW(x,y,z) byte rdata; for(byte i=0;i<z;i++) {rdata=dueFlashStorage.read(y+i); if(rdata != 0 || rdata != 255) x[i]=rdata; else {x[i]=0; break;}}

#else
#define  eeprom_read_byteNEW(x) eeprom_read_byte((uint8_t*)(x))
#define  eeprom_read_wordNEW(x) eeprom_read_word((uint16_t*)(x))
#define  eeprom_read_dwordNEW(x) eeprom_read_dword((uint32_t*)(x))

#define  eeprom_write_byteNEW(x,y)  eeprom_write_byte((uint8_t *)(x),(uint8_t)y)
#define  eeprom_write_wordNEW(x,y)  eeprom_write_word((uint16_t *)(x),(uint16_t)y)
#define  eeprom_write_dwordNEW(x,y)  eeprom_write_dword((uint32_t *)(x),(uint32_t)y)

#define  eeprom_write_blockNEW(x,y,z)  eeprom_write_block((void *)x,(void *)y,z)
#define  eeprom_read_blockNEW(x,y,z)  eeprom_read_block((void *)x,(void *)y,z)

#endif





//************************************************************************************************//
//FLASH LOCATIONS ADDRESS (from 0 to 1023 bytes of EEPROM)
//************************************************************************************************//

//////////////////////////////////////////////
//POWER-ON, VERSION and DATES
#define FLA_POWER_ADDRESS 0                     //only (2 bytes)

#define FLA_DATE_ADDRESS 5              	//day, hour, min (5,6,7: 3 bytes)

#define FLA_ANDRUINO_ID_ADDRESS 8               //only (8,9: 2 bytes)

#define FLA_CODE_SIGNATURE_ADDRESS 10

#define FLA_ANDRUINO_VERSION 15                //App version  16 bit (15, 16)


#define FLA_NRF_ENABLE_ADDRESS 17 

#define FLA_FIRMWARE_ADDRESS_OK 19                //sketch version right data
#define FLA_FIRMWARE_ADDRESS 20                   //sketch version number

//////////////////////////////////////////////
//PUSH PIN AND USER
#define FLA_PUSH_PIN_ADDRESS 30              //2 bytes for pin number (30-31)
#define FLA_PUSH_USER_ADDRESS 32             //15 bytes for char (from 32-46)



//////////////////////////////////////////////
//NRF NODE ADDRESS LIST 
//from 50 to 70 (10 node, 2 bytes for each node)
#define FLA_NRF_NODE_ADDRESS_OK 49
#define FLA_NRF_NODE_ADDRESS 50



//////////////////////////////////////////////
//Store Limits enable and values in flash
//From 100 to 771 (0 to 671 bytes)
#define MAX_ABSOLUTE_VAR 40
#define MAX_ABSOLUTE_ANA 16
#define MAX_ABSOLUTE_DIG 56

#define MAX_BYTE_FOREACH_VAR 8
#define MAX_BYTE_FOREACH_ANA 8
#define MAX_BYTE_FOREACH_DIG 6

//VAR from 100 to 499 (40 var, 8 byte for each one)
#define FLA_VAR_START_ADDRESS 100
//ANALOG from 500 to 627 (16 analog, 8 byte for each one)
#define FLA_ANALOG_START_ADDRESS (FLA_VAR_START_ADDRESS + MAX_ABSOLUTE_VAR*MAX_BYTE_FOREACH_VAR)    //420
//DIGITAL from 628 to 1108 (56 digital, 4 byte for each one)
#define FLA_DIGITAL_START_ADDRESS (FLA_ANALOG_START_ADDRESS + MAX_ABSOLUTE_ANA*MAX_BYTE_FOREACH_ANA) //548
#define FLA_DIGITAL_STOP_ADDRESS ((((FLA_VAR_START_ADDRESS + MAX_ABSOLUTE_VAR*MAX_BYTE_FOREACH_VAR) + MAX_ABSOLUTE_ANA*MAX_BYTE_FOREACH_ANA) + MAX_ABSOLUTE_DIG*MAX_BYTE_FOREACH_DIG)-1) //996


//////////////////////////////////////////////
//ZIGBEE UID modules address
//from 900 to 939
#define FLA_UDID_START_ADDRESS FLA_DIGITAL_STOP_ADDRESS+10            						//4 bytes for each ZigBee module (20-21-22-23)

//////////////////////////////////////////////
//TIMERS (40 bytes, from 940 to 1023)
#define FLA_TIMERS_ADDRESS 940                  //from 50 to 99 (5 bytes for each timer, 8 timers means 40 bytes)
#define BYTES_FOR_EACH_TIMER 6         	  	//each timer uses 6 bytes

#if ARDUINO_STDAVR == 1
#define MAX_TIMERS 12             	    	//max number of timers for AVR micro - only 1K of eeprom (max is 12)
#else
#define MAX_TIMERS 32             	    	//Arduino Due has more eeprom space (emulated in flash)
#endif



#endif

