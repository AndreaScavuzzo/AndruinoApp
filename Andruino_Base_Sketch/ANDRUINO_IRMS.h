////////////////////////////////////////////////////////////////////////////////////////////////////
//ANDRUINO LIBRARY
//A.Scavuzzo July 2013
//www.andruino.it
////////////////////////////////////////////////////////////////////////////////////////////////////

//************************************************************************************************//
//Function used to calculate the Irms of AC
//************************************************************************************************//

//ADCchannel=Arduino ADC channel number
//CT_ratio=trasformer ratio (ex:2000)
//Rsense=resistor value used to convert Current-voltage (CT)
//sample_number=number of ADC sample (500-4000)

//************************************************************************************************//
//POWER MEASUREMENT SETUP (data sent by ethernet port to iPhone)
//************************************************************************************************//
#ifndef ANDRUINO_IRMS_H
#define ANDRUINO_IRMS_H


#include "Arduino.h"

#define IRMS_CTRATIO 2000
#define IRMS0_ADC_CH 0        //ADC used for lights
#define IRMS1_ADC_CH 1        //ADC used for Plugs
#define IRMS0_RSENSE 384      //Rsense used for Hall sensor (lights)
#define IRMS1_RSENSE 264      //Rsense used for Hall sensor (plugs)


class ANDRUINO_IRMS
{
  private:

  public:

  float CurrentIrms (byte ADCchannel, int CT_ratio, int Rsense, int sample_number, float ADC_STEP);

};



#endif
