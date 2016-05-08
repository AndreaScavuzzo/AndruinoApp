
////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ANDRUINO_BASE.h"

//here because they are not classes
#include "ANDRUINO_VCC.h"
#include "printf.h"

extern float VERSION;


#if ETHERNET_SHIELD == 1  || ETHERNET_SHIELD_V2 == 1
EthernetServer server(ETHERNET_PORT);
#if ETHERNET_SHIELD_V2 == 1
#define W5200_CS  10
#endif
EthernetClient _client;
#endif

#if WIFI_SHIELD == 1
WiFiServer server(ETHERNET_PORT);
WiFiClient _client;
int status = WL_IDLE_STATUS;
#endif

#if ARDUINO_YUN == 1 
//#include <YunServer.h>
//#include <YunClient.h>
YunServer server;
YunClient _client;
#endif



ANDRUINO_PinTypes::DigitalPin ArduinoIO[MAXPIN];                            //DIGITAL INSTANCES (respect to the max pin in ANDRUINO_0DEFINES.h (open ANDRUINO library)
ANDRUINO_PinTypes::AnalogPin ArduinoAnalog[MAXANA];                         //ANALOG  INSTANCES (respect to the max pin in ANDRUINO_0DEFINES.h (open ANDRUINO library)
ANDRUINO_PinTypes::Variable Arduino_User_var[ARDUINO_USER_VAR_MAX];         //ARDUINO VARIABLES setup (respect to the max pin in ANDRUINO_0DEFINES.h (open ANDRUINO library)

#if ZIGBEE_ENABLE == 1
XBee xbee = XBee();
//XBEE UNIT MAX INSTANCES
ANDRUINO_PinTypes::SystemXBeePinsType SystemXBeePins[XBeeMaxModules];       //ZIGBEE  setup (respect to the max pin in ANDRUINO_0DEFINES.h (open ANDRUINO library)
#endif

#if NRF24L_ENABLE == 1
//NRF24L UNIT MAX INSTANCES
ANDRUINO_PinTypes::SystemNRF24LPinsType SystemNRF24LPins[NRF24LMaxModules];  //NRF24L  setup (respect to the max pin in ANDRUINO_0DEFINES.h (open ANDRUINO library)
#endif


ANDRUINO_JSON json;
ANDRUINO_LIMITS limits;

ANDRUINO_TIMERS timers;
ANDRUINO_PinTypes pin_types;
ANDRUINO_SERVICE service;
ANDRUINO_NRF nrf;



//************************************************************************************************//
//general variables/defines
//************************************************************************************************//
byte  seconds_counter = 0;
byte  minutes_counter = 0;
byte  hours_counter = 0;
byte  days_counter = 0;            //monday(1), tue(2), wed(3), thu(4), fri(5), sat(6), san(0)

unsigned long previousMillis = 0;
unsigned int  andruino_app_id = 0; //used to match Arduino and AndruinoApp ambient
unsigned int andruino_app_version = 0;

boolean check_ddns = true;
boolean check_limits_ana = false;
boolean check_limits_xbee = false;
boolean check_limits_var = false;
boolean check_limits_dig = false;
boolean check_sensor1 = true;
boolean check_sensor2 = true;
boolean check_DallasTemperature = true;
boolean check_power_consumption = false;
boolean check_timers = false;
boolean send_sensor_req = false;
boolean send_sensor_req2 = false;

boolean force_pushddns = false;
boolean send_push_msg = false;
boolean store_sensor_eeprom = false;

boolean nrf_radio_enable = false;
boolean nrf_radio_enable_rq = false;
boolean nrf_radio_disable_rq = false;

unsigned int connection_isfar = 0;
unsigned int network_access = 0;
boolean swap_timer = false;

char ddns_address[16] = "255.255.255.255";

unsigned long time1 = 0;
unsigned long time2 = 0;
byte ddns_success_cnt, internet_fail_cnt , push_success_cnt, push_fail_cnt, feedD;
float version_sketch = VERSION;

int pin_push_flash = 0;
char push_user [16];

bool global_set = false;

unsigned int  VCC_SUPPLY_BASE;
float ADC_STEP;

unsigned int http_server_performance_ms=0;
unsigned int http_client_performance_ms=0;


/////////////////////////////////////////////////////
//INTERRUPT ROUTINES (software interrupt done using millis()
/////////////////////////////////////////////////////
void Interrupt_Timer_1sec () {

   
    //Used to avoid to launch DDNS checks during user connections
    connection_isfar++;
    
    //  Serial.println(connection_isfar);
    
#if ARDUINO_STDAVR == 1
    Arduino_User_var[3].value = service.freeRam();                     //var[3] is used for the free memory of Arduino Uno
#endif
    
    //Calculate seconds/minutes/hours
    seconds_counter++;                                                 //increment time
    if (seconds_counter % (CHECK_LIMITS_EVERY) == 0)                   //every 20 second
        check_limits_ana = true;
    if ((seconds_counter + 5) % (CHECK_LIMITS_EVERY) == 0)            //every 20 second
        check_limits_dig = true;
    if ((seconds_counter + 10) % (CHECK_LIMITS_EVERY) == 0)            //every 20 second
        check_limits_var = true;
    if ((seconds_counter) % (CHECK_LIMITS_XBEE_EVERY) == 0)              //every 20 second
        check_limits_xbee = true;
    if ((seconds_counter) % (CHECK_POWER_CONSUMPTION_EVERY) == 0)        //every 5 second
        check_power_consumption = true;
    
    
    if (seconds_counter > 59) {
        
        seconds_counter = 0;
        minutes_counter++;
        
        if (minutes_counter % 5 == 0)                                 //every 5 minutes check the timers
            check_timers = true;
        
        if (minutes_counter % (CHECK_DDNS_EVERY) == 0)                //every 5 minutes check external address and compare with the first one
            check_ddns = true;
        
        if (minutes_counter % 3 == 0)                                 //every 3 minutes store_sensor_eeprom
            store_sensor_eeprom = true;
        
        if (minutes_counter % (SEND_SENSOR_REQ_EVERY) == 0)
            send_sensor_req = true;

        if (minutes_counter % (SEND_SENSOR_REQ2_EVERY) == 0)
            send_sensor_req2 = true;
        
        if (minutes_counter % (CHECK_DALLAS_TEMPERATURE_MINUTE) == 0) //every 2 minutes
            check_DallasTemperature = true;
        if (minutes_counter > 59) {
            minutes_counter = 0;
            hours_counter++;
            if (hours_counter > 23) {
                hours_counter = 0;
                days_counter++;
            }
        }
    }
    
    //days of the week are 0(Sun)-1(Mon)-2-3-4-5-6(sat)
    if (days_counter > 6) {
        days_counter = 0;
    }
    
    if (seconds_counter == 30)                                         //every minute read adafruit1 sensor
        check_sensor1 = true;
    if (seconds_counter == 50)                                         //every minute read adafruit2 sensor
        check_sensor2 = true;
    
    
    //////////////////////////////////////////
    //TEMPORIZED OUTPUTS OR TIMERS
    //reset the OUTPUTS after a delay
    for (int i = 0; i < MAXPIN; i++) {
        if (ArduinoIO[i].used == 1 && ArduinoIO[i].mode == 1 && ArduinoIO[i].time_counter > 0) {  //Used and INPUT (mode=0 --> input, mode=1 -->output, mode=3 -->pwm)
            ArduinoIO[i].time_counter--;
            if (ArduinoIO[i].time_counter == 0)
                pin_types.writeDig(ArduinoIO[i], false);                            //reset the OUTPUT after the delay
        }
    }
    
    
#if NRF24L_ENABLE == 1
    for (byte k = 0; k < NRF24LMaxModules; k++) {
        if (SystemNRF24LPins[k].RNF24LAddr != 0) {
            if (SystemNRF24LPins[k].alive_counter > 0)
                SystemNRF24LPins[k].alive_counter--;
            if (SystemNRF24LPins[k].alive_counter == 1)
                SystemNRF24LPins[k].alive_fail_counter++;
        }
    }
#endif


  //  Serial.print(F("seconds_counter:"));Serial.println(seconds_counter);
    
    
}


float EEPROM_readDouble(int ee)
{
   float value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       *p++ = eeprom_read_byteNEW(ee++);
   return value;
}

void EEPROM_writeDouble(int ee, float value)
{
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++)
      //pollo[i]= *p++;
      eeprom_write_byteNEW(ee+i,*p++);
}



void ANDRUINO_BASE::Setup() {

/*   float firmware_read = EEPROM_readDouble(FLA_FIRMWARE_ADDRESS);
    if(firmware_read !=VERSION) {
      //Serial.print(F("firmware was different: "));Serial.println(firmware_read,3);
      for (int i = 0; i < 1024; i++) {
        eeprom_write_byteNEW (i, 255);
      }
      EEPROM_writeDouble(FLA_FIRMWARE_ADDRESS,VERSION);
    }
*/ 

 //if the CODE signature is not 0xA3 means that the flash has to be erased
  if(eeprom_read_byteNEW (FLA_CODE_SIGNATURE_ADDRESS) != 0xA3) {
    for (int i = 0; i < 1024; i++) {
        eeprom_write_byteNEW (i, 255);
    }
    eeprom_write_byteNEW ((uint8_t *)(FLA_CODE_SIGNATURE_ADDRESS), 0xA3);
  }


  
    service.RestoreDataFromFlash();                             //restore the sensor data from flash (limits enable, values, hour, min, etc)
       
    //SERIAL SETUP
    Serial.begin(9600);
#if DEBUG_SERIAL == 1
    printf_begin();
    Serial.print(F("Start Arduino, fw:"));
    char buffer_char[10];
    dtostrf(VERSION, 5, 3, buffer_char);
    Serial.println(buffer_char);
    float vcc_arduino = ((float) readVcc()) / 1000.0;
    ADC_STEP = vcc_arduino/(ADC_MAX-1);
    Serial.print(F("Voltage: "));
    dtostrf(vcc_arduino, 5, 3, buffer_char);
    Serial.print(buffer_char);Serial.println("Volt");
#endif


  


    
  

    
    
    //ZIGBEE SETUP
#if ZIGBEE_ENABLE == 1
    xbee.setSerial(Serial);
    delay(4000);                                                  //wait 4 sec after ZIGBEE setup
#endif
    
    
    //ETHERNET SHIELD SETUP
#if ETHERNET_SHIELD == 1 || ETHERNET_SHIELD_V2 == 1
    delay(1000);
    //  Ethernet.begin(mac,ip,myDns);
    Ethernet.begin(mac, ip, dnsAdd, gateway);
    //Ethernet.begin(mac,ip);
    server.begin();
    delay(2000);
#if (DEBUG_SERIAL == 1)
    Serial.print(F("Server is:"));
    Serial.println(Ethernet.localIP());
#endif
#endif
    
    //WIFI SHIELD SETUP
#if WIFI_SHIELD == 1
    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD) {
#if (DEBUG_SERIAL == 1 || DEBUG_SERIAL_JSON == 1)
        Serial.println(F("WiFi not present"));
        // don't continue:
#endif
        while (true);
    }
    // attempt to connect to Wifi network:
    while ( status != WL_CONNECTED) {
#if DEBUG_SERIAL == 1
        Serial.print(F("SSID: "));
        Serial.println(ssid);
#endif
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);
        // wait 10 seconds for connection:
        delay(10000);
    }
    server.begin();
#if DEBUG_SERIAL == 1
    service.printWifiStatus();
#endif
#endif
    
    
    //YUN SETUP
#if ARDUINO_YUN == 1
    Bridge.begin();
    // Listen for incoming connection only from localhost
    server.listenOnLocalhost();
    server.begin();
#if (DEBUG_SERIAL == 1)
    Serial.println(F("YUN begin"));
#endif
#endif
    

    ANDRUINO_0SENSORS::SetupSensors();

        
    /////////////////////////////////////
    //read USER and PIN PUSH from flash
    json.ReadPushUserPin();
    if (pin_push_flash < 1000 || pin_push_flash > 9999) {                                         //if empty write the default value
        eeprom_write_blockNEW("pushuser", FLA_PUSH_USER_ADDRESS, strlen(push_user) + 1); // Store the PUSH USER in flash
        eeprom_write_wordNEW ((FLA_PUSH_PIN_ADDRESS), 0);                             //store PIN=0 when is not used
        json.ReadPushUserPin();
    }
#if DEBUG_SERIAL == 1
    Serial.print(F("pushUser:")); Serial.println(push_user);
    Serial.print(F("pin:")); Serial.println(pin_push_flash);
#endif
    
    
    ////////////////////////////////////////////////////
    //Power-on PUSH message
    //
    ANDRUINO_PUSH push;
    int valore = eeprom_read_wordNEW (FLA_POWER_ADDRESS);
    push.SendPush(push_user, ARDUINO_NAME, "pow", "", "", 0, valore, VERSION);    //(char *type, char *mode,byte port, byte lim,float value);
    //Use it to write message using push
    //push.SendPush2(push_user, ARDUINO_NAME, "msg", "hello this is a push");
#if ARDUINO_STDAVR == 1
#if DEBUG_SERIAL == 1
    //   Serial.print(F("Rd eeprom:"));
    //   Serial.println(valore);
#endif
    valore++;
    //program the power-on cycling
    eeprom_write_wordNEW ((FLA_POWER_ADDRESS), valore);
#endif
    
    
    
    //PUT HERE THE ZIGBEE UNIQUE ID LSB CODES
#if ZIGBEE_ENABLE == 1
    eeprom_write_dwordNEW ((FLA_UDID_START_ADDRESS), 0x40869A1C);    //index 0
    eeprom_write_dwordNEW ((FLA_UDID_START_ADDRESS + 4), 0x408C0E4E); //index 1
    eeprom_write_dwordNEW ((FLA_UDID_START_ADDRESS + 8), 0x40869A18); //index 2
    //Configure only the outputs of zigbee
    SystemXBeePins[1].Pin[1].mode = 1;                 //configure ZigBee 1 (FLA_UDID_START_ADDRESS+4) as output
    //  SystemXBeePins[1].Pin[4].mode = 1;               //configure ZigBee 1 (FLA_UDID_START_ADDRESS+4) as output
#endif
    
    
    
    //TIMER SETUP
#if ARDUINO_STDAVR == 1
    Timer1.initialize(1000000);                         //timer set for each second
    Timer1.attachInterrupt(Interrupt_Timer_1sec);       //attach the interrupt routine
#else
    //Arduino DUe
    Timer3.attachInterrupt(Interrupt_Timer_1sec);
    Timer3.start(1000000); // Calls every 1s
#endif
    
    
#if DEBUG_SERIAL == 1
    //  Serial.println(F("Setup finished"));
#endif
    
     
}


void ANDRUINO_BASE::Loop() {
  
    //examples
    
    //example of variable. You can generate variable starting from ADC values
    //Arduino_User_var[5].value = (ArduinoAnalog[0].value/0.0655)*((ArduinoAnalog[2].value-2.51)/0.0227);
    //Arduino_User_var[6].value = (ArduinoAnalog[1].value/0.0655)*((ArduinoAnalog[3].value-2.51)/0.0227);
    
    
    //HOW READ A PIN STATE  (reading directly the Arduino PIN)
    //int read_pin = pin_types.read(ArduinoIO[3]).state;            //DIGITAL
    //float adc = analogRead(ArduinoAnalog[0].pin) * ADC_STEP;      //ANALOG
    
    //HOW READ A PIN STATE (reading the global array, it is refreshed by sketch preriodically)
    //int read_state = ArduinoIO[3].state;                          //DIGITAL
    //float read_analog = ArduinoAnalog[0].value;                    //ANALOG
    
    
    
    
    
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //SENSORS READING
    ReadAnalogSensors();                                             //read various types of sensors
    
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //CHECL LIMITS AND SEND PUSH NOTIFICATIONS
#if CHECK_LIMITS == 1
    limits.CheckLimitsSensors(push_user, ARDUINO_NAME);
#endif
    
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //CHECK IS EXTERNAL MODEM ADDRESS CHANGE
#if DYNAMIC_DDNS == 1
    if (check_ddns && connection_isfar > DDNS_READ_AFTER_INACTIVE) {                    //check if the request is true and no connection are present (> 120 sec)
        check_ddns = false;
        ANDRUINO_DDNS ddns_read;
        ddns_read.CheckDDNS_andSend_Notification(push_user, ARDUINO_NAME, force_pushddns);                                                 //Check DDNS and send email or norification
    }
#endif
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //READ WIRELESS SENSORS
#if ZIGBEE_ENABLE == 1
    Call_Zigbee();                                                  //Check if ZIGBEE has received data from sensors
#endif
    
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //NETWORK CONNECTION AS SERVER
    
    
    
#if ARDUINO_YUN == 1
    _client = server.accept();                              //If Arduino YUN module has received data, let's decode them
#else
    _client = server.available();                              //If Ethernet module has received data, let's decode them
#endif
    
    if (_client) {
        connection_isfar = 0;                                        //used to avoid ddns read during sensor reading
        network_access++;

        unsigned long int http_performance_start=millis();   
        Arduino_User_var[2].value = network_access;
        byte clientStatus = json.WaitForRequest_and_ParseReceivedRequest(_client, ARDUINO_NAME, ARDUINO_PASS);   //buffer the received string in buffer[], parsing the received string, extract username, password, command, action, port
        if (clientStatus == 1) {
            json.PerformRequestedCommand(true);                          //execute the action read above (read, etc)  - false(full json)
        } else {
            _client.print(F("{\"error\":["));
            _client.print(clientStatus);
            _client.print(F("]}"));
        }
        _client.flush();
        _client.stop();
        http_server_performance_ms = (unsigned int)((unsigned long int)millis() - (unsigned long int)http_performance_start);
        //Serial.print(F("http_performance: "));Serial.print(http_performance_ms); Serial.println(F("ms"));

    }
    
    
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //PUSH TEST
    if (send_push_msg) {
        send_push_msg = false;
        //Use it to write message using push
        //  push.SendPush2(push_user, ARDUINO_NAME, "msg", "Push test");
        ANDRUINO_PUSH push;
        push.SendPush(push_user, ARDUINO_NAME, "pair", "", "", 0, 0, VERSION);    //(char *type, char *mode,byte port, byte lim,float value);
    }
    

    
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //Chech routine every minutes
    if (store_sensor_eeprom) {                //store sensor setting to eeprom
        store_sensor_eeprom = false;
        service.StoreDataToFlash();
    }
    else if (check_timers) {                  //check the timers every 5 minutes
        check_timers = false;
        timers.CheckTimers(push_user, ARDUINO_NAME);
    }
#if DATA_LOGGING_DB == 1
//    else if (send_sensor_req && pin_push_flash >0) {               //every 5 minutes send a request to server
//        send_sensor_req = false;
//        Serial.println(F("Send data to db"));
//        ANDRUINO_PUSH push;
//        push.SendHttp(push_user, ARDUINO_NAME, ARDUINO_PASS, "launch_logger");
//    }
    else if (send_sensor_req2 && pin_push_flash >0) {               //every 5 minutes send a request to server
        send_sensor_req2 = false;
        unsigned long int http_performance_start=millis();   
        ANDRUINO_DATA_LOGGER data_logger;
        
        data_logger.SendDataLogger(push_user, ARDUINO_NAME, ARDUINO_PASS, 0);    //analog + var + system
        data_logger.SendDataLogger(push_user, ARDUINO_NAME, ARDUINO_PASS, 1);    //IO + system
#if NRF24L_ENABLE == 1        
        data_logger.SendDataLogger(push_user, ARDUINO_NAME, ARDUINO_PASS, 2);    //NRF
#endif        
#if ZIGBEE_ENABLE == 1
        data_logger.SendDataLogger(push_user, ARDUINO_NAME, ARDUINO_PASS, 3);    //XBEE
#endif
        http_client_performance_ms = (unsigned int)((unsigned long int)millis() - (unsigned long int)http_performance_start);
        //Serial.print(F("http_send_performance: "));Serial.print(http_performance_ms); Serial.println(F("ms"));
    }
#endif

    
    /*
     //Example used to read one input and drive one output (bistable)
     if (pin_types.read(ArduinoIO[30]).state ==1
     || pin_types.read(ArduinoIO[31]).state ==1
     || pin_types.read(ArduinoIO[32]).state ==1
     || pin_types.read(ArduinoIO[33]).state ==1
     || pin_types.read(ArduinoIO[34]).state ==1
     || pin_types.read(ArduinoIO[35]).state ==1
     || pin_types.read(ArduinoIO[36]).state ==1
     || pin_types.read(ArduinoIO[37]).state ==1) {
     if (!global_set) {
     delay (200);                                  //software anti bump
     if (pin_types.read(ArduinoIO[30]).state ==1) {
     pin_types.toggle(ArduinoIO[22]);
     global_set = true;
     } else if (pin_types.read(ArduinoIO[31]).state ==1) {
     pin_types.toggle(ArduinoIO[23]);
     global_set = true;
     } else if (pin_types.read(ArduinoIO[32]).state ==1) {
     pin_types.toggle(ArduinoIO[24]);
     global_set = true;
     } else if (pin_types.read(ArduinoIO[33]).state ==1) {
     pin_types.toggle(ArduinoIO[25]);
     global_set = true;
     } else if (pin_types.read(ArduinoIO[34]).state ==1) {
     pin_types.toggle(ArduinoIO[26]);
     global_set = true;
     } else if (pin_types.read(ArduinoIO[35]).state ==1) {
     pin_types.toggle(ArduinoIO[27]);
     global_set = true;
     } else if (pin_types.read(ArduinoIO[36]).state ==1) {
     pin_types.toggle(ArduinoIO[28]);
     global_set = true;
     } else if (pin_types.read(ArduinoIO[37]).state ==1) {
     pin_types.toggle(ArduinoIO[29]);
     global_set = true;
     }
     }
     } else {
     global_set = false;
     }
     */


     
#if NRF24L_ENABLE == 1
    if (nrf_radio_enable_rq) {
        Serial.println(F("\n\rNRF radio is ON"));
        nrf.setup_nrf();
        nrf_radio_enable = true;
        nrf_radio_enable_rq = false;
    } else if (nrf_radio_disable_rq) {
        Serial.println(F("\n\rNRF radio is OFF"));
        powerdown_nrf();
        nrf_radio_enable = false;
        nrf_radio_disable_rq = false;
    }
    if (nrf_radio_enable)
        receive_nrf();
#endif
    
}



