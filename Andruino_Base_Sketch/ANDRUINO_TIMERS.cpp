
////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo Aug 2014
//www.andruino.it
//Aug 2014
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_TIMERS.h"

void ANDRUINO_TIMERS::CheckTimers(char *push_usr, char *arduino_name) {


#if DEBUG_SERIAL_TIMER == 1
  Serial.println(F("Check timers"));
#endif

  for (int index = 0; index < MAX_TIMERS; index++) {
    //timer number --> address index
    //0: port number  --> address 0
    //1: repeat + week --> address 1 bit[6:0] S-F-T-W-T-M-S --> 7 bit (1 if is selected, 0 if is unselected)
    //2: hour --> address 2 4:0 (0-23) --> 5 bit
    //3: duration sec and min --> address 3  7:4 (0-11) 5 sec duration,   3:0 (0-11) 5 min --> 4 bit (0-1-2-3-4-5-6-7-8-9-10-11)(0-5-10-15-20-25-30-35-40-45-50-55)
    //4: duration --> address 4: bit[7:4] for hours (0-17 max), bit[3:0] for minutes - 5 min for each step

    uint16_t address;
    uint8_t port_number;    //255 means not used
    uint8_t week;			//S-M-T-W-T-F-S-S (0010101 means timer on S-M-W)
    uint8_t hour;
    uint8_t minutes;
    //uint8_t duration_hour;
    //uint8_t duration_min;
    uint8_t temp;
    uint16_t counter_sec;
    bool repeat;
    bool used;

    address = FLA_TIMERS_ADDRESS + BYTES_FOR_EACH_TIMER * index;
    port_number = eeprom_read_byteNEW (address);      //6 bit +2 (used or unused)

#if DEBUG_SERIAL_TIMER == 1
    Serial.print(F("Timer: ")); Serial.print(index); Serial.print(F(", port_number: ")); Serial.print(port_number); Serial.print(F(", addr: ")); Serial.println(address);
#endif


    if (port_number != 255 && port_number != 128 )	{		   	           //255 means timer not used, 128 meand that the timer is expired but associated

      week = eeprom_read_byteNEW (address + 1);         //7 bit
      //repeat = (week & 0x80)>>7;
      week = week & 0x7F;

      if (week > 0)
        repeat = true;
      else
        repeat = false;



      hour = eeprom_read_byteNEW (address + 2);      //5 bit (bit 4:0)  //5 bit used
      minutes = (eeprom_read_byteNEW (address + 3)) * 5; //4 bit for minutes mult by 5 (0-1-2-3-4-5-6-7-8-9-10-11)(0-5-10-15-20-25-30-35-40-45-50-55)

      temp = eeprom_read_byteNEW (address + 4);	//4+4 bit //8 bit used
      // duration_min = (temp & 0xF)*5;                   //minute duration, 4 bit (mult by 5) (0-1-2-3-4-5-6-7-8-9-10-11)-->(0-5-10-15-20-25-30-35-40-45-50-55)
      // duration_hour = (temp & 0xF0)>>4;		      //hours duration 4 bit




      //not more used
      // counter_sec = 60*(duration_min+duration_hour*60);

      //new App
      // if (andruino_app_version != 65535) {
      counter_sec = eeprom_read_wordNEW (address + 4);
      // }


#if DEBUG_SERIAL_TIMER == 1
      Serial.print(F("minutes: ")); Serial.print(minutes); Serial.print(F(", minutes_counter: ")); Serial.println(minutes_counter);
      Serial.print(F("hour: ")); Serial.print(hour); Serial.print(F(", hours_counter: ")); Serial.println(hours_counter);
#endif


      //check if timer is reached
      //here hour and minute are matching. Note that this is executed only one time because this routine is called each 5min, so the next time the min is not more matched
      if ((minutes_counter == minutes) && (hours_counter == hour)) {

        //days_counter 0-1-2-3-4-5-6
        //index_current_day 0000001(monday) or 0000010(tuesday) or 0000100(wednesday), etc
        uint8_t index_current_day = 1;
        index_current_day = 1 << days_counter;

#if DEBUG_SERIAL_TIMER == 1
        Serial.print(F("repeat: ")); Serial.print(repeat);
        Serial.print(F(", week: ")); Serial.print(week);
        Serial.print(F(", days_counter: ")); Serial.print(days_counter);
        Serial.print(F(", index_current_day: ")); Serial.print(index_current_day);
        Serial.print(F(", out: ")); Serial.println(port_number);
#endif


        //check if the current day is reached
        //repeat=false  --> means only one timer event
        //repeat=true  --> means that the event has to be repeat, so check the day of the week
        if (((index_current_day & week) && repeat == true) || repeat == false)	{												//means that the timer is reached
          pin_types.writeDig(ArduinoIO[port_number], true);                            //activate the output
          ArduinoIO[port_number].time_counter = counter_sec;    //program the auto switch off (time_counter is in seconds)
          //message when the timer set the outpit PIN

#if DEBUG_SERIAL_TIMER == 1
          Serial.print(F("duration in sec: ")); Serial.println(counter_sec);
#endif
          if (counter_sec < 3)  {
            delay(counter_sec * 1000);                      //pulse of ArduinoIO[port_number].time_counter
            pin_types.writeDig(ArduinoIO[port_number], false);
          }

           
          ANDRUINO_PUSH push;
          push.SendPush(push_usr, arduino_name, "tmr", "", "", 0, index, port_number);    //(char *type, char *mode,byte port, byte lim,float value);

          //4.41
          if (repeat == false)
            eeprom_write_byteNEW (address, 128);                               //128 if the timer has no repeat (timer expired)
        }

      }
    }
  }
}
