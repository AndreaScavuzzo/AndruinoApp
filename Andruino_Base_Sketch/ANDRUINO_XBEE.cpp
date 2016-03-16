////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////
//ZIGBEE module SERIES2, configured in sampling mode
//////////////////////////////////////////////////////

#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_XBEE.h"


#if ZIGBEE_ENABLE == 1

void Call_Zigbee()
{

  ZBRxIoSampleResponse ioSample = ZBRxIoSampleResponse();
  XBeeAddress64 test = XBeeAddress64();
  byte index_zigbee = 0;
  bool found_module = false;

  //attempt to read a packet
  xbee.readPacket();                                                            //ZIGBEE read buffer
  if (xbee.getResponse().isAvailable()) {                                       //if a response is available, enter here
    // got something
#if DEBUG_SERIAL_ZIGBEE == 1
    Serial.println(F("Read ZigBee:"));
#endif
    if (xbee.getResponse().getApiId() == ZB_IO_SAMPLE_RESPONSE) {               //parse the response

      xbee.getResponse().getZBRxIoSampleResponse(ioSample);                     //get the IoSample response
#if DEBUG_LED_XBEE == 1
      flashLed(TXLed, 10, 2);
#endif


      uint32_t LSB_UDID = ioSample.getRemoteAddress64().getLsb();
      uint32_t MSB_UDID = ioSample.getRemoteAddress64().getMsb();
      for (int i = 0; i < XBeeMaxModules; i++) {
        uint32_t  ZigBee_FLASH = eeprom_read_dwordNEW ((FLA_UDID_START_ADDRESS + (i * 4)));
        if (LSB_UDID == ZigBee_FLASH) {
#if DEBUG_SERIAL_ZIGBEE == 1
          Serial.print(F("found:"));
          Serial.print(ZigBee_FLASH, HEX);
          Serial.print(F(" - index:"));
          Serial.println(i);
#endif
          index_zigbee = i;
          found_module = true;
          break;
        }
      }
      if (!found_module)           //exit if the UDID packet is not recognized
        return;

      //store the UID address
      SystemXBeePins[index_zigbee].RemoteAddrLSB  = LSB_UDID;  //acquire the address of ZIGBEE connected
      SystemXBeePins[index_zigbee].RemoteAddrMSB  = MSB_UDID;  //acquire the address of ZIGBEE connected
      SystemXBeePins[index_zigbee].samples++;                  //increment the samples received


#if DEBUG_SERIAL_ZIGBEE == 1
      Serial.print(F("Xbee: "));
      Serial.print(index_zigbee);
      Serial.print(F(" MSB and LSB: "));
      Serial.print(SystemXBeePins[index_zigbee].RemoteAddrMSB, HEX);
      Serial.println(SystemXBeePins[index_zigbee].RemoteAddrLSB, HEX);
#endif



      if (ioSample.containsDigital() || ioSample.containsAnalog()) {                      //check if zigbee contains digitals or analogs
#if DEBUG_SERIAL_ZIGBEE == 1
        Serial.println(F("Found IO samples"));
#endif
        // scan all the XBEE PINS (both analog and digitals)
        for (int i = 0; i < XBeeMaxPinModule; i++) {

          bool digital = ioSample.isDigitalEnabled(i);                                            //check if it is a Digital
          bool analog = ioSample.isAnalogEnabled(i);                                              //check if it is an analog
          bool used_pin = digital || analog;
          SystemXBeePins[index_zigbee].Pin[i].used = used_pin;                                    //If yes put the used=1, if no used=0
          if (used_pin) {                                                                         //If digital IO is used, updates the fields
            SystemXBeePins[index_zigbee].Pin[i].pin = i;                                        //(pin number updates)
            if (analog) {
              SystemXBeePins[index_zigbee].Pin[i].value = ioSample.getAnalog(i) * ADC_XBEE_STEP; //(analog value updates)
              SystemXBeePins[index_zigbee].Pin[i].mode = 2;									//mode ANALOG
#if DEBUG_SERIAL_ZIGBEE == 1
              Serial.print(F("Analog (AI"));
              Serial.print(i, DEC);
              Serial.print(F(") is "));
              Serial.println(SystemXBeePins[index_zigbee].Pin[i].value, 3);
#endif
            }
            else {
              SystemXBeePins[index_zigbee].Pin[i].value = ioSample.isDigitalOn(i);     //(input value updates)
#if DEBUG_SERIAL_ZIGBEE == 1
              Serial.print(F("Digital (DI"));
              Serial.print(i, DEC);
              Serial.print(F(") is "));
              Serial.println(SystemXBeePins[index_zigbee].Pin[i].value, 0);
#endif
            }
          }
        }
      }


      // method for printing the entire frame data
      //for (int i = 0; i < xbee.getResponse().getFrameDataLength(); i++) {
      //  Serial.print("byte [");
      //  Serial.print(i, DEC);
      //  Serial.print("] is ");
      //  Serial.println(xbee.getResponse().getFrameData()[i], HEX);
      //}
    }
    else {
#if DEBUG_SERIAL_ZIGBEE == 1
      Serial.print(F("Expected I/O Sample, but got "));
      Serial.print(xbee.getResponse().getApiId(), HEX);
#endif
    }
  } else if (xbee.getResponse().isError()) {                                    //if here, error detected
#if DEBUG_LED_XBEE == 1
    flashLed(TXLed, 2, 100);
#endif
#if DEBUG_SERIAL_ZIGBEE == 1
    Serial.print(F("Error:"));
    Serial.println(xbee.getResponse().getErrorCode());
#endif
  }
}





boolean sendRemoteAtCommand(RemoteAtCommandRequest remoteAtRequest) {
  boolean data_sent = false;
  byte timeout = 0;

  // Create a Remote AT response object
  RemoteAtCommandResponse remoteAtResponse = RemoteAtCommandResponse();


  while (timeout < RETRY_MAX & !data_sent) {
    timeout++;
#if DEBUG_SERIAL_ZIGBEE == 1
    Serial.println();
    Serial.print(F("Send to XBee"));
    Serial.print(F(".tent: ")); Serial.println(timeout);
#endif
    // send the command
    xbee.send(remoteAtRequest);
    // wait up to 5 seconds for the status response
    if (xbee.readPacket(5000)) {
      // got a response!
#if DEBUG_SERIAL_ZIGBEE == 1
      Serial.println(F("Received response"));
#endif
      // should be an AT command response
      if (xbee.getResponse().getApiId() == REMOTE_AT_COMMAND_RESPONSE) {
        xbee.getResponse().getRemoteAtCommandResponse(remoteAtResponse);

        if (remoteAtResponse.isOk()) {
          data_sent = true;
#if DEBUG_SERIAL_ZIGBEE == 1
          Serial.print(F("Command ["));
          Serial.print(remoteAtResponse.getCommand()[0]);
          Serial.print(remoteAtResponse.getCommand()[1]);
          Serial.println(F("] was successful!"));
          if (remoteAtResponse.getValueLength() > 0) {
            Serial.print(F("Command value length is "));
            Serial.println(remoteAtResponse.getValueLength(), DEC);
            Serial.print(F("Command value: "));

            for (int i = 0; i < remoteAtResponse.getValueLength(); i++) {
              Serial.print(F("getValue: "));
              Serial.print(remoteAtResponse.getValue()[i], HEX);
              Serial.println();
            }

            Serial.println();
          }
          return 1;
        }
        else {
          Serial.println();
          Serial.print(F("Command returned error code: "));
          Serial.println(remoteAtResponse.getStatus(), HEX);
#endif
        }

      }
#if DEBUG_SERIAL_ZIGBEE == 1
      else {
        Serial.println();
        Serial.print(F("Expected Remote AT response but got "));
        Serial.print(xbee.getResponse().getApiId(), HEX);
      }
    }
    else if (xbee.getResponse().isError()) {
      Serial.println();
      Serial.print(F("Error reading packet.  Error code: "));
      Serial.println(xbee.getResponse().getErrorCode());
    }
    else {
      Serial.println();
      Serial.print(F("No response from radio"));
#endif
    }
  }

#if DEBUG_SERIAL_ZIGBEE == 1
  Serial.println(F("MAX TIMEOUT"));
#endif
  return 0;
}


#endif
