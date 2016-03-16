

#ifndef __SLEEP_H__
#define __SLEEP_H__

#include <avr/sleep.h>
#include <avr/power.h>

// Sleep declarations
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;

void setup_watchdog(uint8_t prescalar);
void do_sleep(void);

const short sleep_cycles_per_transmission = 4;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;



void wakeUp(){
  sleep_disable();
}

// Sleep helpers

//Prescaler values
// 0=16ms, 1=32ms,2=64ms,3=125ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec

void setup_watchdog(uint8_t prescalar){

  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);                      // Clear the WD System Reset Flag
  WDTCSR = _BV(WDCE) | _BV(WDE);            // Write the WD Change enable bit to enable changing the prescaler and enable system reset
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);  // Write the prescalar bits (how long to sleep, enable the interrupt to wake the MCU
}

ISR(WDT_vect)
{
  //--sleep_cycles_remaining;
  Serial.println("WDT");
}

void do_sleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  attachInterrupt(0,wakeUp,LOW);
  WDTCSR |= _BV(WDIE);
  sleep_mode();                        // System sleeps here
                                       // The WDT_vect interrupt wakes the MCU from here
  sleep_disable();                     // System continues execution here when watchdog timed out  
  detachInterrupt(0);  
  WDTCSR &= ~_BV(WDIE);  
}

#endif // __SLEEP_H__

