////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ANDRUINO_XBEE_H
#define ANDRUINO_XBEE_H
#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_PinTypes.h"
#include "ANDRUINO_JSON.h"

#define RETRY_MAX 2

#if ZIGBEE_ENABLE == 1
#include <XBee.h>              //zigbee is enabled only for Arduino Due (for ram reason)

extern ANDRUINO_PinTypes::SystemXBeePinsType SystemXBeePins[];
extern XBee xbee;

void Call_Zigbee();
boolean sendRemoteAtCommand(RemoteAtCommandRequest remoteAtRequest);

#endif
#endif
