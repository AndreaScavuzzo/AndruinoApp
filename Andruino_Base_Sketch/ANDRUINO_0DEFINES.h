////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef ANDRUINO_0DEFINE_H
#define ANDRUINO_0DEFINE_H


//Ethernet interfaces
#define ETHERNET_SHIELD             1                               //Ethernet shield enabled (classic module)
#define ETHERNET_SHIELD_V2          0                               //Ethernet shield seeds studio V2 enabled (http://www.seeedstudio.com/wiki/Ethernet_Shield_V2.0 )
#define WIFI_SHIELD                 0                               //Wifi shield disabled


//RF MODULES
#define NRF24L_ENABLE               1                               //NRF MODULES ENABLE
#define ZIGBEE_ENABLE               0                               //enable the ZIGBEE module receiver (1)


//Sensors
#define Dallas_DS18B20              0                               //Enable OneWire library for DallasTemperature DS18B20
#define THERMO_ADAFRUIT_MAX31855    0                               //THERMOCOUPLE ADAFRUIT K MAX31855 (Not validated)
#define POWER_CONSUMPTION_ENABLE    1                               //enable the ADC conversion for Irms calculation on adc0/adc1  (Validated, ok)
#define THERMO_ADAFRUIT_DHT         1                               //FIRST DHT sensor
#define THERMO_ADAFRUIT_DHT2        0                               //SECOND DHT sensor


//OPTIONS TO REDUCE FLASH SPACE (but reduce also functionality)
#define CHECK_LIMITS                1
#define DYNAMIC_DDNS                1


//enable data logging on server (ENABLED BY DEFAULT)
#define DATA_LOGGING_DB             1                               //enable datalogging on server



#include "ANDRUINO_DEVICE.h"
#include "ANDRUINO_CONFIG.h"
#endif

