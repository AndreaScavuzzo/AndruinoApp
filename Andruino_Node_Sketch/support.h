

#ifndef __SUPPORT_H__
#define __SUPPORT_H__


//16 bits
//system_data[0]=vcc
//system_data[1]=counter
//system_data[2]=default sleep interval
//system_data[3]=firmware version
//system_data[4]=PA level
//system_data[5]=ROUTER(0), LEAF(1)
//
uint16_t system_data[] = {1, 0, 8, 255, 255, 255};
const uint8_t num_system_data = sizeof(system_data);


const uint8_t num_digital_pins = sizeof(digital_pins);
uint8_t digital_states[num_digital_pins];


// Analog input Pins

const uint8_t num_analog_pins = sizeof(analog_pins);
uint16_t analog_states[num_analog_pins];


// Variables input
const uint8_t num_variable_pins = NRF24LMaxVariable;
float variable_states[num_variable_pins];



bool SetupDigitalPins (int i) {

  while (i--)
  {
    if (digital_pins_mode[i] == INPUT) {
      pinMode(digital_pins[i], INPUT);
      digitalWrite(digital_pins[i], HIGH);
    } else {
      pinMode(digital_pins[i], OUTPUT);
      digitalWrite(digital_pins[i], LOW);
    }
  }
}

void OutPinChange (int i, uint8_t *old_states, uint8_t *new_states) {

  while (i--)
  {
    if (digital_pins_mode[i] == OUTPUT) {
      if (old_states[i] != new_states[i]) {
        digitalWrite(digital_pins[i], new_states[i]);
       // printf("Pin OUTPUT number:%d has changed from:%d to %d\n\r", digital_pins[i],old_states[i], new_states[i] );
        old_states[i] = new_states[i];
      }
    }
  }
}

bool readDigitals (int i, uint8_t *pins, uint8_t *states) {

uint8_t temp;
bool pin_changed=false;

  while (i--)
  {
    temp = digitalRead(pins[i]);
    if(temp !=states[i]) 
      pin_changed = true;
    states[i] = temp;
  }

  return(pin_changed);
}

void readAnalogs (int i, uint8_t *pins, uint16_t *states) {

  while (i--)
  {
    states[i] = analogRead(pins[i]);
//    printf_P(PSTR("states[%d]: %d\n\r"),i,states[i]);

  }
}

void PrintVector (uint8_t *states, int i) {

  for (int k = 0; k < i; k++)
  {
    Serial.println(states[k]);
  }

}

void PrintVector (uint16_t *states, int i) {
  for (int k = 0; k < i; k++)
  {
    Serial.print(states[k]);
    if(k<i-1) {
      Serial.print(", ");
    } else
      Serial.println();
  }

}

void SetupVars (int i, float *states) {

  while (i--)
  {
    states[i] = 0;
  }

}

//copyArray(pippo, tano, 1, 8);
void copyArray (uint8_t *source, uint8_t *destination, uint8_t start_byte, uint8_t num_byte) {
  
  //(destination, source, Ndata)
  memcpy(destination, source+start_byte, num_byte );

  
/*  for (int i = 0; i < num_byte; i++) {
    destination[i] = source[i + start_byte];
  }*/
}



#endif // __VCC_H__

