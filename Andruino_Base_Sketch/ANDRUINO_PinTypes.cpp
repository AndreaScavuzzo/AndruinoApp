////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////


//************************************************************************************************//
//PIN MANAGEMENT
//************************************************************************************************//
#include "ANDRUINO_PinTypes.h"


////////////////////////////////////////////
//DIGITALS
////////////////////////////////////////////

//DIG PIN SETUP
void ANDRUINO_PinTypes::setupDig(DigitalPin& pin, boolean mode = false, boolean state = false)
{
  pin.used = true;                   //is the pin used (0=not used, 1=used)
  pin.mode = mode;                   //used to know if the pin is: INPUT=0, OUTPUT=1, PWM=2
  pin.state = state;                 //used  to store the read pin
  pin.pulse = false;                 //used to remember if the pin has been pulsed (to transmit the feedback to Andruino)
  pinMode(pin.pin, pin.mode);         //configure the pin
  digitalWrite(pin.pin, pin.state);   //write the pin
}

//DIG PIN SETUP PWM
void ANDRUINO_PinTypes::setupPWM(DigitalPin& pin, byte mode, byte state)
{
  pin.used = true;
  pin.mode = mode;
  pin.state = state;
  pinMode( pin.pin, OUTPUT);
  analogWrite( pin.pin, pin.state);
}
//DIG PIN WRITE
void ANDRUINO_PinTypes::writeDig(DigitalPin& pin, boolean state) //the port has to be configured as output before with setupDig
{
  pin.state = state;
  digitalWrite( pin.pin, pin.state);
}
//DIG PIN WRITE PWM
void ANDRUINO_PinTypes::writePWM(DigitalPin& pin, byte state)   //the port has to be configured as output before with setupPWM
{
  pin.state = state;
  analogWrite( pin.pin, pin.state);
}
//DIG PIN READ
ANDRUINO_PinTypes::DigitalPin& ANDRUINO_PinTypes::read(DigitalPin& pin)
{
  pin.state = digitalRead( pin.pin );
  return pin;
}


//Toggle pin
void ANDRUINO_PinTypes::toggle(DigitalPin& pin) {
  writeDig(pin, !pin.state);
}

//Pulse pin
void ANDRUINO_PinTypes::pulse(DigitalPin& pin, int wait) {

  toggle(pin);
  delay(wait);
  toggle(pin);
  //#if DEBUG_SERIAL == 1
  //            Serial.print(F("Pulse on PIN: "));Serial.println(pin.pin);
  //#endif
  pin.pulse = true;
}

void ANDRUINO_PinTypes::flashLed(DigitalPin& pin, byte times, int wait) {

  for (byte i = 0; i < times; i++) {
    writeDig(pin, 1);
    delay(wait);
    writeDig(pin, 0);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}

////////////////////////////////////////////
//ANALOGS
////////////////////////////////////////////

void ANDRUINO_PinTypes::setupAnalog(AnalogPin& pin)
{
  pin.used = true;
}

/*void ANDRUINO_PinTypes::writeAnalog(AnalogPin& pin)
{
  analogWrite(pin.pin,pin.value);
}
*/



