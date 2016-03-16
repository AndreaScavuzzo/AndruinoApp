
#ifndef ANDRUINO_NRF_H
#define ANDRUINO_NRF_H



#include "ANDRUINO_0DEFINES.h"      //before used to know which device
#include "ANDRUINO_0DEFINES.h"    //general define used to configure the MAXPIN/MAX ANA/ETC

#include "ANDRUINO_PinTypes.h"

#if NRF24L_ENABLE == 1
#include <SPI.h>
#include <RF24Network.h>
#include <RF24.h>
extern ANDRUINO_PinTypes::SystemNRF24LPinsType SystemNRF24LPins[NRF24LMaxModules];
extern byte nrf_pipeline_id[];
#endif
#if ARDUINO_STDAVR == 1
#include <avr/eeprom.h>
#else
#include <DueFlashStorage.h>
extern DueFlashStorage dueFlashStorage;
#include  <avr/dtostrf.h>
#endif

class ANDRUINO_NRF
{
  private:


  public:
    void setup_nrf();
    void receive_nrf();
    void powerdown_nrf();
    bool send_digital_out(uint16_t tx_to_node, byte type, byte channel, byte new_state);
    bool send_digital_vector(uint16_t tx_to_node, byte index_node, byte type, byte channel, byte new_state);
};


#endif

