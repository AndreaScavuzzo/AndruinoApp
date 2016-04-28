
////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo Feb 2014
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_LIMITS.h"

/////////////////////////////////////////////////////
//Check LIMITS and Send PUSH NOTIFICATIONS
/////////////////////////////////////////////////////
void  ANDRUINO_LIMITS::CheckLimitsSensors(char *push_usr, char *arduino_name)

{

  //check limits of Analog inputs
  //low/hi
  //THE ANALOG LIMITS ARE SENT MULTIPLIED BY 1024
  if (check_limits_ana) {
#if DEBUG_SERIAL_LIMITS ==1
    Serial.println(F("Check ANA limits"));
#endif
    check_limits_ana = false;
    for (int i = 0; i < MAXANA; i++) {
        ArduinoAnalog[i].value = analogRead(i) * ADC_STEP;    //read the Analog input
        byte OutOfLimits;
        unsigned int enable_limits = ArduinoAnalog[i].enable_limits;
        OutOfLimits = check_HiLolimits(enable_limits, ArduinoAnalog[i].max, ArduinoAnalog[i].min, ArduinoAnalog[i].limit_count, (ArduinoAnalog[i].value) * 1024);

        //ACTIVATE OUTPUT IF LIMIT IS REACHED
        if (enable_limits & 0x2) {   //if BIT1=1, output enabled
#if DEBUG_SERIAL_LIMITS ==1
          bool plarity = (enable_limits & 0x100);
          Serial.print(F("analog: ")); Serial.print(i); Serial.print(F(" enable_limits ")); Serial.print(enable_limits, BIN); Serial.print(F(", OutOfLimits")); Serial.print(OutOfLimits); Serial.print(F(", polarity")); Serial.println(plarity);
#endif

          byte out_port = (enable_limits >> 2) & 0x3F;        //bit 7:2 output enabled number
          if ((OutOfLimits == 2 && (!(enable_limits & 0x100)))  //bit 8 polarity [8]=0, low
              || (OutOfLimits == 1 && (enable_limits & 0x100)))  //bit 8 polarity [8]=1, high
            //digitalWrite(out_port, HIGH);                                   //activate output
            pin_types.writeDig(ArduinoIO[out_port], true);                    //activate output
          else if (OutOfLimits == 3)                                            //reset output
            //digitalWrite(out_port, LOW);                                    //disactivate output
            pin_types.writeDig(ArduinoIO[out_port], false);                   //disactivate output
        }


        //SENS PUSH IF LIMIT IS REACHED
        if ((OutOfLimits == 1 || OutOfLimits == 2) && (enable_limits & 0x1)) {   //if BIT0=1, push enabled
#if DEBUG_SERIAL_LIMITS ==1
          Serial.print(F("ANA LIMIT out: ")); Serial.println(OutOfLimits);
          //                    Serial.print(F("ANA MAX lim:"));Serial.println(ArduinoAnalog[i].max);
          //                    Serial.print(F("ANA value x1024:"));Serial.println((ArduinoAnalog[i].value)*1024);
          //                    Serial.print(F("ANA MIN lim:"));Serial.println(ArduinoAnalog[i].min);
#endif
          ANDRUINO_PUSH push;
          push.SendPush(push_usr, arduino_name, "lim", "ana", "arduino_io", i, OutOfLimits, ArduinoAnalog[i].value);              //SEND PUSH MAX/MIN andrea_push?type=limits&mode=Ana&port=1&lim=hi&value=0.11

        }
    }
  }
  //check limits of Digital inputs
  //low/hi
  if (check_limits_dig) {
#if DEBUG_SERIAL_LIMITS ==1
    Serial.println(F("Check DIG limits"));
#endif
    check_limits_dig = false;
    for (int i = 0; i < MAXPIN; i++) {
      if (ArduinoIO[i].used == 1 && ArduinoIO[i].mode == 0) {  //Used and INPUT (mode=0 --> input, mode=1 -->output, mode=3 -->pwm)
        pin_types.read(ArduinoIO[i]);        //read digital inputs
        byte OutOfLimits;
        unsigned int enable_limits = ArduinoIO[i].enable_limits;
        OutOfLimits = check_digitalAlarm(enable_limits, ArduinoIO[i].alarm, ArduinoIO[i].limit_count, ArduinoIO[i].state);

        //ACTIVATE OUTPUT IF LIMIT IS REACHED
        if (enable_limits & 0x2) {                                          //if BIT1=1, output enabled

#if DEBUG_SERIAL_LIMITS ==1
          bool plarity = (enable_limits & 0x100);
          Serial.print(F("dig in: ")); Serial.print(i); Serial.print(F(" enable_limits ")); Serial.print(enable_limits, BIN); Serial.print(F(", OutOfLimits")); Serial.print(OutOfLimits); Serial.print(F(", polarity")); Serial.println(plarity);
#endif

          byte out_port = (enable_limits >> 2) & 0x3F;                    //bit 7:2 output enabled number
          if (OutOfLimits == 1)
            pin_types.writeDig(ArduinoIO[out_port], true);                           //activate output
          else if (OutOfLimits == 3)
            pin_types.writeDig(ArduinoIO[out_port], false);                          //deactivate output
        }


        //SENS PUSH IF LIMIT IS REACHED
        if (OutOfLimits == 1 && (enable_limits & 0x1)) {   //if BIT0=1, push enabled
#if DEBUG_SERIAL_LIMITS ==1
          Serial.print(F("DIG LIMIT out: ")); Serial.println(OutOfLimits);
          //                    Serial.print(F("ANA MAX lim:"));Serial.println(ArduinoAnalog[i].max);
          //                    Serial.print(F("ANA value x1024:"));Serial.println((ArduinoAnalog[i].value)*1024);
          //                    Serial.print(F("ANA MIN lim:"));Serial.println(ArduinoAnalog[i].min);
#endif
          //
          ANDRUINO_PUSH push;
          push.SendPush(push_usr, arduino_name, "lim", "dig", "arduino_io", i, OutOfLimits, ArduinoIO[i].state);              //SEND PUSH MAX/MIN andrea_push?type=limits&mode=Ana&port=1&lim=hi&value=0.11
        }
      }
    }
  }

#if ZIGBEE_ENABLE == 1
  //check limits of Digital/Analog inputs of XBEE
  //low/hi
  //THE ANALOG LIMITS ARE SENT MULTIPLIED BY 1024
  if (check_limits_xbee) {
#if DEBUG_SERIAL_LIMITS ==1
    Serial.println(F("Check XBEE ANA/DIG limits"));
#endif

    check_limits_xbee = false;
    char xbee_module_name[11];
    for (int k = 0; k < XBeeMaxModules; k++) {
      sprintf(xbee_module_name, "XBee_io_%d", k);        //used to identify the XBEE module number (XBee_io_0, XBee_io_1, etc)
      for (int i = 0; i < XBeeMaxPinModule; i++) {
        if (SystemXBeePins[k].Pin[i].used == 1) {
          //DIGITAL XBEE
          unsigned int enable_limits = SystemXBeePins[k].Pin[i].enable_limits;
          if (SystemXBeePins[k].Pin[i].mode == 0) { //select digital INPUT zigbee (mode //input(0) or output (1), or analog (2))
            //                  Serial.print(F("xbee module used: ")); Serial.println(xbee_module_name);
            //                Serial.print(F("xbee IO used: ")); Serial.println(i);
            //                Serial.print(F("enable_limits: ")); Serial.println(SystemXBeePins[k].Pin[i].enable_limits);
            //                Serial.println();
            byte OutOfLimits = false;
            OutOfLimits = check_digitalAlarm(enable_limits, SystemXBeePins[k].Pin[i].max, SystemXBeePins[k].Pin[i].limit_count, SystemXBeePins[k].Pin[i].value);
            if (OutOfLimits == 1 || OutOfLimits == 2) {  //if a high or low limit is reached
#if DEBUG_SERIAL_LIMITS ==1
              Serial.print(F("XBEE DIG LIMIT out: ")); Serial.println(OutOfLimits);
#endif
              ANDRUINO_PUSH push;
              push.SendPush(push_usr, arduino_name, "lim", "dig", xbee_module_name, SystemXBeePins[k].Pin[i].pin, OutOfLimits, SystemXBeePins[k].Pin[i].value);               //SEND PUSH MAX/MIN andrea_push?type=limits&mode=Ana&port=1&lim=hi&value=0.11
            }
            //ANALOG XBEE
          } else if (SystemXBeePins[k].Pin[i].mode == 2) {  //select analog zigbee
            //                  Serial.print(F("xbee module used: ")); Serial.println(xbee_module_name);
            //                Serial.print(F("xbee IO used: ")); Serial.println(i);
            //                Serial.print(F("enable_limits: ")); Serial.println(SystemXBeePins[k].Pin[i].enable_limits);
            //                Serial.println();

            byte OutOfLimits;
            OutOfLimits = check_HiLolimits(enable_limits, SystemXBeePins[k].Pin[i].max, SystemXBeePins[k].Pin[i].min, SystemXBeePins[k].Pin[i].limit_count, (SystemXBeePins[k].Pin[i].value) * 1024);
            if ((OutOfLimits == 1 || OutOfLimits == 2) && enable_limits) {
#if DEBUG_SERIAL_LIMITS ==1
              Serial.print(F("XBEE ANA LIMIT out: ")); Serial.println(OutOfLimits);
              //                    Serial.print(F("ANA MAX lim:"));Serial.println(ArduinoAnalog[i].max);
              //                    Serial.print(F("ANA value x1024:"));Serial.println((ArduinoAnalog[i].value)*1024);
              //                    Serial.print(F("ANA MIN lim:"));Serial.println(ArduinoAnalog[i].min);
#endif
              ANDRUINO_PUSH push;
              push.SendPush(push_usr, arduino_name, "lim", "ana", xbee_module_name, SystemXBeePins[k].Pin[i].pin, OutOfLimits, SystemXBeePins[k].Pin[i].value);               //SEND PUSH MAX/MIN andrea_push?type=limits&mode=Ana&port=1&lim=hi&value=0.11
            }
          }
        }
      }
    }
  }
#endif


  //THE VARIABLE LIMITS ARE SENT MULTIPLIED BY 10
  if (check_limits_var) {
#if DEBUG_SERIAL_LIMITS ==1
    Serial.println(F("Check VAR limits"));
#endif
    check_limits_var = false;
    for (int i = 0; i < ARDUINO_USER_VAR_MAX; i++) {
      byte OutOfLimits;
      unsigned int enable_limits = Arduino_User_var[i].enable_limits;
      OutOfLimits = check_HiLolimits(enable_limits, Arduino_User_var[i].max, Arduino_User_var[i].min, Arduino_User_var[i].limit_count, (Arduino_User_var[i].value) * 10);

      //ACTIVATE OUTPUT IF LIMIT IS REACHED
      if (enable_limits & 0x2) {                                    //if BIT1=1, output enabled
        byte out_port = (enable_limits >> 2) & 0x3F;              //bit 7:2 output enabled number
        if ((OutOfLimits == 2 && (!(enable_limits & 0x100)))        //bit 8 polarity [8]=0, low
            || (OutOfLimits == 1 && (enable_limits & 0x100)))     //bit 8 polarity [8]=1, high
          pin_types.writeDig(ArduinoIO[out_port], true);                            //activate output
        else if (OutOfLimits == 3)
          pin_types.writeDig(ArduinoIO[out_port], false);                           //deactivate output
      }
      if ((OutOfLimits == 1 || OutOfLimits == 2) && (enable_limits & 0x1)) {   //if BIT0=1, push enabled
#if DEBUG_SERIAL_LIMITS ==1
        Serial.print(F("VAR LIMIT out:")); Serial.println(OutOfLimits);
        //                Serial.print(F("VAR MAX lim:"));Serial.println(Arduino_User_var[i].max);
        //                Serial.print(F("VAR value x10:"));Serial.println((Arduino_User_var[i].value)*10);
        //                Serial.print(F("VAR MIN lim:"));Serial.println(Arduino_User_var[i].min);
#endif
        ANDRUINO_PUSH push;
        push.SendPush(push_usr, arduino_name, "lim", "var", "arduino_var" , i , OutOfLimits, Arduino_User_var[i].value);              //SEND PUSH MAX/MIN andrea_push?type=limits&mode=Ana&port=1&lim=hi&value=0.11
      }


    }
  }

}

//////////////////////////////
//Check the High/Low limits
//////////////////////////////

//Return 0: no limit is reached or was before rised
//Return 1: high limit is rised
//Return 2: low limit is rised
//Return 3: limit is not more reached (reset out)

byte ANDRUINO_LIMITS::check_HiLolimits (unsigned int enable_limits, unsigned int max_lim, unsigned int min_lim, byte &limit_count, unsigned int value) {

  if (enable_limits & 0x3) {        //check if push is enabled (bit0) or out enable are enabled (bit[1])

#if DEBUG_SERIAL_LIMITS ==1
    Serial.print(F("limit count:")); Serial.println(limit_count);
#endif

    boolean sign;
    if (max_lim > min_lim)
      sign = true;
    else
      sign = false;
    ////////////////////////////////////////////////////////
    //MAX LIMIT CHECK (up 128 means high limits is reached)
    //
    if (((value > max_lim) && sign ) || ((value < max_lim) && !sign )) {
      if (limit_count < 128)  {                                        //if <128 means that the value is passing from low to high, so the output has to be reset
        limit_count = 128;
        return 3;
      }
      limit_count++;                                                  //increment the limit reach number
      if (limit_count == 255)                                         //lock the counter to 254 when reach the high limit 128 times
        limit_count = 254;

      if (limit_count == 130 || limit_count == 140)                   //first time notification or last time
        return 1;                                                   //SEND PUSH MAX (return value 1)

      ////////////////////////////////////////////////////////
      //MIN LIMIT CHECK (down to 128 means low limit is reached)
    } else if (((value < min_lim) && sign ) || ((value > min_lim) && !sign ))  {
      if (limit_count > 128)  {                                        //if >128 means that the value is passing from hi to low, so the output has to be reset
        limit_count = 128;
        return 3;
      }
      limit_count--;                                                  //increment the limit reach number
      if (limit_count == 0)                                           //lock the counter to 1 when reach the low limit 128 times
        limit_count = 1;

      if (limit_count == 126 || limit_count == 116)                   //first time notification or last time
        return 2;                                                       //SEND PUSH MIN (return  value 2)
    }
    //no limit is reached or we are exiting from a violation
    else {

      if (limit_count < 126 || limit_count > 130) {                     //means that there was an overflow before but now no (so reset the output)
        limit_count = 128;
        return 3;
      } else
        limit_count = 128;                                                  //reset the limit reached counter if no limits are reached
    }


  } else
    limit_count = 128;
  return 0;
}

//////////////////////////////
//Check the High/Low limits
//////////////////////////////

//Return 0: no limit is reached
//Return 1: limit is reached
//Return 3: limit is not more reached (reset out)

byte ANDRUINO_LIMITS::check_digitalAlarm (unsigned int enable_limits, bool alarm, byte &limit_count, bool value) {

  if (enable_limits & 0x3) {                                                    //>0 means that the limits are enabled but not reached

#if DEBUG_SERIAL_LIMITS ==1
    Serial.print(F("Dig limit count:")); Serial.println(limit_count);
#endif


    if (value == alarm) {
      limit_count++;                                                  //increment the limit reach number
      if (limit_count == 255)                                         //lock the counter to 254 when reach the high limit 128 times
        limit_count = 254;

      if (limit_count == 1 || limit_count == 10)                     //first time notification or last time
        return 1;                                                     //SEND PUSH MAX (return value 1)
    } else {

      if (limit_count > 1) {                                        //means that there was an overflow before but now no (so reset the output)
        limit_count = 0;
        return 3;
      } else
        limit_count = 0;                                          //reset the limit reached counter if no limits are reached
    }

  } else
    limit_count = 0;
  return 0;
}
