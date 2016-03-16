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


#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_IRMS.h"



#if POWER_CONSUMPTION_ENABLE == 1

float ANDRUINO_IRMS::CurrentIrms (byte ADCchannel, int CT_ratio, int Rsense, int sample_number, float ADC_STEP) {

  //For analog read
  double value;
  double VDoffset = 510 * ADC_STEP; //Initial value (corrected as program runs)

  //Equation of the line calibration values
  double factorA = CT_ratio / Rsense; //factorA = CT reduction factor / rsens
  //   double Ioffset = -0.08;
  //Constants set voltage waveform amplitude.
  //   double SetV = 230.0;

  //Counter
  int i = 0;
  //Used for calculating real, apparent power, Irms and Vrms.
  double sumI = 0.0;
  int sum1i = 0;
  double sumVadc = 0.0;
  double Vadc, Vsens, Imains, sqI, Irms;
  //   double apparentPower;
  bool finished = false;
  Irms = 15.0;

  while (!finished)
  {
    value = analogRead(ADCchannel);

    //Summing counter
    i++;

    //Voltage at ADC
    Vadc = value * ADC_STEP;

    //Remove voltage divider offset
    Vsens = Vadc - VDoffset;

    //Current transformer scale to find Imains
    Imains = Vsens;

    //Calculates Voltage divider offset.
    sum1i++; sumVadc = sumVadc + Vadc;
    if (sum1i >= 1000) {
      VDoffset = sumVadc / sum1i;
      sum1i = 0; sumVadc = 0.0;
    }

    //Root-mean-square method current
    //1) square current values
    sqI = Imains * Imains;
    //2) sum
    sumI = sumI + sqI;

    if (i >= sample_number)
    {
      //      i=0;
      //Calculation of the root of the mean of the current squared (rms)
      // Irms = factorA*sqrt(sumI/sample_number)+Ioffset;
      Irms = factorA * sqrt(sumI / sample_number) - 0.08;
      //Calculation of the root of the mean of the voltage squared (rms)
      //      apparentPower = Irms * SetV;
#if DEBUG_SERIAL_IRMS == 1
      Serial.print(F("Irms channel: ")); Serial.print(ADCchannel); Serial.print(F(" =")); Serial.print(Irms); Serial.println(F(" A"));
#endif
      //exit while
      return float(Irms);
    }
  }

  return float(Irms);
}
#endif
