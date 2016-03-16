////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_0SENSORS.h"
#include "ANDRUINO_BASE.h"


#if THERMO_ADAFRUIT_DHT == 1                              //Temperature/Humidity sensor DHT11/22 (uses ONE wire bus)
#include <DHT.h>
#define DHTPIN 2                                          // PIN 2 (you can change it), serial pin used to read the data from the sensor (one wire)
#define DHTTYPE DHT22                                     //can be DHT22  (AM2302), DHT21 (AM2301), DHT11
DHT dht(DHTPIN, DHTTYPE);

#if THERMO_ADAFRUIT_DHT2 == 1                             //second DHT sensor
#define DHTPIN2 9                                         // PIN 9 (you can change it), serial pin used to read the data from the sensor (one wire)
DHT dht2(DHTPIN2, DHTTYPE);                               //secon DHT sensor
#endif
#endif


#if THERMO_ADAFRUIT_MAX31855 == 1                       //Temperature sensor (uses SPI bus)
#include <Adafruit_MAX31855.h>
int thermoDO = 3;                                       //soft SPI Data output=PIN3 (SO)
int thermoCS = 4;                                       //soft SPI Chip select=PIN4 (CS)
int thermoCLK = 5;                                      //soft SPI Clock =PIN5 (SCK)
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);
#endif


#if OneWire_ENABLE == 1                                 //Dallas DS18B20 Temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4                                  // Data wire is plugged into pin 4 on the Arduino (you can change it)
OneWire oneWire(ONE_WIRE_BUS);                          // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
#if Dallas_DS18B20 == 1
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
#endif
#endif

void ANDRUINO_0SENSORS::SetupSensors() {

#if THERMO_ADAFRUIT_DHT == 1                              //Adafruit DHT11/22/21 setup
  dht.begin();
#if THERMO_ADAFRUIT_DHT2 == 1
  dht2.begin();                                           //for the second DHT sensor
#endif
#endif

#if Dallas_DS18B20 == 1
  sensors.begin();     
  Serial.print(F("DallasT:"));
  Serial.println(sensors.getDeviceCount(), DEC);
#endif

}

/////////////////////////////////////////////////////////////////////
//Read sensors is periodically called to refresh the sensors
/////////////////////////////////////////////////////////////////////
void ANDRUINO_0SENSORS::ReadAnalogSensors() {


#if THERMO_ADAFRUIT_MAX31855 == 1                             //THERMO_ADAFRUIT_MAX31855 read
  if (check_sensor1) {
    check_sensor1 = false;
    double c = thermocouple.readCelsius();
    if (isnan(c))
      Arduino_User_var[4].value = 199;                        //means error
    else
      Arduino_User_var[4].value = c;                         //check if the access time of this sensor is high. Eventually you can read it every minute (see below)
  }
#endif


#if THERMO_ADAFRUIT_DHT == 1                      //Adafruit DHT11/22/21 read
  float h;
  float t;
  if (check_sensor2) {                            //read temperature and humidity every 1 minute
    check_sensor2 = false;
#if DEBUG_SERIAL == 1
    Serial.println(F("DHT"));
#endif
    h = dht.readHumidity();
    t = dht.readTemperature();
    if (isnan(h))
      Arduino_User_var[0].value = 199;                     //means error
    else
      Arduino_User_var[0].value = h;                       //the Taccess is very high(500ms), so I read it avery minute
    if (isnan(t))
      Arduino_User_var[1].value = 199;                     //means error
    else
      Arduino_User_var[1].value = t;                       //the Taccess is very high(500ms), so I read it avery minute

    //second DHT sensor
#if THERMO_ADAFRUIT_DHT2 == 1
    h = dht2.readHumidity();              //for a second DHT sensor
    t = dht2.readTemperature();           //for a second DHT sensor
    if (isnan(h))
      Arduino_User_var[5].value = 199;                     //means error
    else
      Arduino_User_var[5].value = h;                       //the Taccess is very high(500ms), so I read it avery minute
    if (isnan(t))
      Arduino_User_var[6].value = 199;                     //means error
    else
      Arduino_User_var[6].value = t;                       //the Taccess is very high(500ms), so I read it avery minute
#endif
#if DEBUG_SERIAL == 1
    /*    Serial.print(F("hum:"));
     Serial.print(h);
     Serial.println(F("%"));
     Serial.print(F("temp:"));
     Serial.print(t);
     Serial.println(F("C"));*/
#endif
  }
#endif


#if Dallas_DS18B20 == 1                                   //DallasTemperature Temperature Sensors DS18B20
  if (check_DallasTemperature) {
    check_DallasTemperature = false;
#if DEBUG_SERIAL == 1
    Serial.println(F("Dallas:"));
#endif
    sensors.requestTemperatures();                               // Send the command to get temperatures
    //Here 10 sensors, you can put as you want, but you have to increase the ARDUINO_USER_VAR_MAX to allocate them

    //Each sensor has an unique ID, this has to be extracted with a sample sketch
    Arduino_User_var[7].value = sensors.getTempCByIndex(0);      //Read the sensors and fill the variables. These will be read by AndruinoApp
    /*   Arduino_User_var[8].value = sensors.getTempCByIndex(1);
     Arduino_User_var[9].value = sensors.getTempCByIndex(2);
     Arduino_User_var[10].value = sensors.getTempCByIndex(3);
     Arduino_User_var[11].value = sensors.getTempCByIndex(4);
     */
  }
#endif


#if POWER_CONSUMPTION_ENABLE == 1
  if (check_power_consumption) {
    check_power_consumption = false;
    ANDRUINO_IRMS Irms;
    if (swap_timer == 0)
      Arduino_User_var[5].value = Irms.CurrentIrms(IRMS0_ADC_CH, IRMS_CTRATIO, IRMS0_RSENSE, 500, ADC_STEP);  //read power from lights
    else
      Arduino_User_var[6].value = Irms.CurrentIrms(IRMS1_ADC_CH, IRMS_CTRATIO, IRMS1_RSENSE, 500, ADC_STEP);  //read power from plugs
    swap_timer = !swap_timer;
  }
#endif
}

