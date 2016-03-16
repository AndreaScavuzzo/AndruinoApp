

#ifndef __READ_SENSORS_H__
#define __READ_SENSORS_H__

unsigned long last_time_check;

/***********************************************************************/

#if THERMO_ADAFRUIT_DHT == 1                              //Temperature/Humidity sensor DHT11/22 (uses ONE wire bus)
#include <DHT.h>
#define DHTPIN 6                                          // PIN 2 (you can change it), serial pin used to read the data from the sensor (one wire)
#define DHTTYPE DHT22                                     //can be DHT22  (AM2302), DHT21 (AM2301), DHT11
DHT dht(DHTPIN, DHTTYPE);

#if THERMO_ADAFRUIT_DHT2 == 1                             //second DHT sensor
#define DHTPIN2 9                                         // PIN 9 (you can change it), serial pin used to read the data from the sensor (one wire)
DHT dht2(DHTPIN2, DHTTYPE);                               //secon DHT sensor
#endif
#endif

#if Dallas_DS18B20 == 1                                 //Dallas DS18B20 Temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 6                                  // Data wire is plugged into pin 4 on the Arduino (you can change it)
OneWire oneWire(ONE_WIRE_BUS);                          // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
#endif


void setup_sensors() {

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
void ReadSensors() {


  unsigned long now = millis();         
  if ( now - last_time_check < INTERVAL_SENSOR ){
    return;
  }
  last_time_check = now;
  


#if THERMO_ADAFRUIT_DHT == 1                      //Adafruit DHT11/22/21 read
  float h;
  float t;

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DHT\n\r")));
    h = dht.readHumidity();
    t = dht.readTemperature();
    if (isnan(h))
      variable_states[0] = 199;                     //means error
    else
      variable_states[0] = h;                       //the Taccess is very high(500ms), so I read it avery minute
    if (isnan(t))
      variable_states[1] = 199;                     //means error
    else
      variable_states[1] = t;                       //the Taccess is very high(500ms), so I read it avery minute

    //second DHT sensor
#if THERMO_ADAFRUIT_DHT2 == 1
    h = dht2.readHumidity();              //for a second DHT sensor
    t = dht2.readTemperature();           //for a second DHT sensor
    if (isnan(h))
      variable_states[2] = 199;                     //means error
    else
      variable_states[2] = h;                       //the Taccess is very high(500ms), so I read it avery minute
    if (isnan(t))
      variable_states[3] = 199;                     //means error
    else
      variable_states[3] = t;                       //the Taccess is very high(500ms), so I read it avery minute
#endif
#endif


#if Dallas_DS18B20 == 1                                   //DallasTemperature Temperature Sensors DS18B20

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Dallas\n\r")));
    sensors.requestTemperatures();                               // Send the command to get temperatures
    //Here 10 sensors, you can put as you want, but you have to increase the ARDUINO_USER_VAR_MAX to allocate them

    //Each sensor has an unique ID, this has to be extracted with a sample sketch
    variable_states[0] = sensors.getTempCByIndex(0);      //Read the sensors and fill the variables. These will be read by AndruinoApp
    /*   Arduino_User_var[8].value = sensors.getTempCByIndex(1);
     Arduino_User_var[9].value = sensors.getTempCByIndex(2);
     Arduino_User_var[10].value = sensors.getTempCByIndex(3);
     Arduino_User_var[11].value = sensors.getTempCByIndex(4);
     */
#endif

}

#endif // __READ_SENSORS_H__

