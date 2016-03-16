
#define VERSION_FW 1001


//1001 vs 1000
//sleep update fixed
//fixed NRF network libs 1.0 to 1.1 for sleep max time (byte to unsigned int)
//vcc read optimized
//connection counter fixed

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <TimerOne.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEBUG_SERIAL_NRF24L 0


#define INTERVAL_SENSOR   5000
#define THERMO_ADAFRUIT_DHT         0                               //FIRST DHT sensor
#define THERMO_ADAFRUIT_DHT2        0
#define Dallas_DS18B20              0                               //Enable OneWire library for DallasTemperature DS18B20



//##########################
//USER PIN CONFIGURATION
//##########################
//don't use PIN: 2,7,8,11,12,13 (0 and 1 are used for serial port, so can be used for serial sebug or as pins)

uint8_t digital_pins[] = {3, 4, 5, 9};                                      //-->>> CHANGE THE PIN TO BE USED
bool digital_pins_mode[] = {INPUT, INPUT, OUTPUT, OUTPUT};                  //-->>> CHANGE THE PIN DIRECTION (INPUT/OUTPUT)
uint8_t analog_pins[] = {0, 1, 2, 3};
const uint8_t NRF24LMaxVariable = 5;                                        //max variable are 5 (can't be >5)

//##########################
//NRF PIN CONFIGURATION
//##########################
//NRF CE and CEN PINS
#define PIN_NRF_CE 8
#define PIN_NRF_CS 7

//###############################
//USER NODE NETWORK CONFIGURATION
//###############################

// These are the Octal addresses that will be assigned - https://tmrh20.github.io/RF24Network/Tuning.html
const uint16_t node_address_array[12] = {04, 02, 05, 012, 015, 022, 025, 032, 035, 045, 0112, 0212};
typedef enum {ROUTER = 0, LEAF = 1} node_e;


// 0-1-2 (04,02,05)   = Children of Master(00)
// 3,5 (012,022) = Children of (02)
// 4,6 (015,025) = Children of (05)
// 7   (032)     = Child of (02)
// 8,9 (035,045) = Children of (05)
// 10,9 (112,212) = Children of (012)


//Use numbers 0 through to select an address from the array
#define  DEFAULT_NODE_ADDRESS  1                                      //-->>> CHANGE THE ADDRESS OF THE NODE (put FORCE_NODE_ADDRESS as true to have effect)
bool  FORCE_NODE_ADDRESS =  false;                                    //if true, the DEFAULT_NODE_ADDRESS is programmed on flash and used as node address


//FALSE (ROUTER node), TRUE (LEAF node)
node_e  NODE_CONFIG = ROUTER;                                         //-->>> CHANGE IF THE NODE IS A LEAF OR ROUTER

#define  NRF_CHANNEL_NUM 100                                          //same channel as BASE (100 is the default)

byte nrf_pipeline_id[] = {0x2d, 0x4b, 0x60, 0x87, 0x96, 0xb4};        //Netword ID (as base)


/***********************************************************************/




#include "printf.h"
#include "remote_base.h"
#include "read_sensors.h"

void setup() {

  setup_node();
  setup_sensors();

}



void loop() {

  check_rf();
  ReadSensors();
}



