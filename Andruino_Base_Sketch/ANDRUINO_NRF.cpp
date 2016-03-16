





#include "ANDRUINO_0DEFINES.h"
#include "ANDRUINO_NRF.h"


#if NRF24L_ENABLE == 1

//PIN USED FOR NRF CE and CS (you can move as you want)
#define PIN_NRF_CE 49
#define PIN_NRF_CS 48

#define NRF_CHANNEL_NUM 100

RF24 radio(PIN_NRF_CE, PIN_NRF_CS);                                   // CE & CS pins to use (Using 7,8 on Uno,Nano)
RF24Network network(radio);


const short max_active_nodes = NRF24LMaxModules;                    // Array of nodes we are aware of
//uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;

//functions
bool send_T(uint16_t to);                              // Prototypes for functions to send & handle messages
bool send_N(uint16_t to);
bool sendCommand (uint16_t to, unsigned char cmd, uint8_t *states, uint8_t num);
void handle_T(RF24NetworkHeader& header);
void handle_N(RF24NetworkHeader& header);
void handle_SYSTEM(RF24NetworkHeader& header, short index_node);
void handle_ANALOG(RF24NetworkHeader& header, short index_node, byte mode);
void handle_DIGITAL(RF24NetworkHeader& header, short index_node, byte mode);
void handle_VARIABLE(RF24NetworkHeader& header, short index_node, byte mode);
//void send_active_nodes(RF24NetworkHeader& header);
short search_node(uint16_t node);
short add_node(uint16_t node);
short search_nodeFlash(uint16_t node);
short add_nodeFlash(uint16_t node);
void PrintVector (uint16_t *states, int i);
void PrintVector (uint8_t *states, int i);
void PrintVector (float *states, int i);
void copyArray (uint8_t *source, uint8_t *destination, uint8_t start_byte, uint8_t num_byte);
short search_nodeFlash(uint16_t node);
short add_nodeFlash(uint16_t node);



//16 bits system_data
//system_data[0]=vcc
//system_data[1]=counter
//system_data[2]=default sleep interval
//system_data[3]=firmware version
//system_data[4]=PA level
//system_data[5]=router(0), leaf(1)
//
uint16_t system_data[] = {1, 0, 8, 255, 255, 255};
const uint8_t max_num_system_data = 6;



// Digital input Pins

const uint8_t max_num_digital_pins = NRF24LMaxDigitalPin;
//uint8_t digital_pins[max_num_digital_pins];                                    // {2, 3, 4, 5, 9};
//uint8_t digital_states[max_num_digital_pins];                                  // pin values
//bool digital_pins_mode[max_num_digital_pins];                                  //{INPUT, INPUT, INPUT, INPUT, OUTPUT};


// Analog input Pins
const uint8_t num_analog_pins = NRF24LMaxAnalogPin;
//uint8_t analog_pins[num_analog_pins];                         //{2, 3};
//uint16_t analog_states[num_analog_pins];


// Variables input
float variable_states[NRF24LMaxVariable];


void ANDRUINO_NRF::setup_nrf() {

  //  SPI.begin();                                           // Bring up the RF network
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  //radio.setDataRate(RF24_250KBPS);      //speed  RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  //already in begin radio.setRetries(15, 15);               //How long to wait between each retry, in multiples of 250us, max is 15. 0 means 250us, 15 means 4000us.
  
//  network.begin(/*channel*/ NRF_CHANNEL_NUM, /*node address*/ 00 );

  network.choose_pipe_address(nrf_pipeline_id);
  network.begin(/*channel*/ NRF_CHANNEL_NUM, /*node address*/ 00 );

  radio.startListening();
  IF_SERIAL_DEBUG_NRF(radio.printDetails());            // Dump the configuration of the rf unit for debugging



  //if the signature 0x73 is not present means that the flash storage is not used by NRF and so I will erase it
  if(eeprom_read_byteNEW (FLA_NRF_NODE_ADDRESS_OK) != 0x73) {
    //erase all the NRF data index
    for (int i = FLA_NRF_NODE_ADDRESS; i < (FLA_NRF_NODE_ADDRESS + NRF24LMaxModules*2); i++) {
        eeprom_write_byteNEW (i, 255);
    }
    eeprom_write_byteNEW ((uint8_t *)(FLA_NRF_NODE_ADDRESS_OK), 0x73);
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("NRF flash storage has been erased\n\r")));
  }

}

void ANDRUINO_NRF::powerdown_nrf() {
  radio.stopListening();
  radio.powerDown();
}


void ANDRUINO_NRF::receive_nrf() {


  network.update();                                      // Pump the network regularly
  while ( network.available() )  {                      // Is there anything ready for us?

    RF24NetworkHeader header;                            // If so, take a look at it
    network.peek(header);

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("---------------------------------\n\r")));
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("-->--RECEIVED--FROM--NODE: 0%o\n\r"), header.from_node));
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("---------------------------------\n\r")));


    //check if the received node is already inthe table (index_node>=0) or is not present (index_node = -1)
    short index_node =  search_nodeFlash(header.from_node);


    switch (header.type) {                             // Dispatch the message to the correct handler.
      //T and N has to be at the beginning, don't move them!
      case 'T': handle_T(header); break;
      case 'N': handle_N(header); break;
      case 'Y': handle_SYSTEM(header, index_node); break;
      case 'D': handle_DIGITAL(header, index_node, 0); break;  //DIGITAL PIN status
      case 'A': handle_ANALOG(header, index_node, 0); break;   //ANALOG PIN status

      case 'a': handle_ANALOG(header, index_node, 1); break;   //ANALOG PIN position
      case '1': handle_DIGITAL(header, index_node, 1); break;  //DIGITAL PIN mode (INPUT, OUTPUT)
      case 'd': handle_DIGITAL(header, index_node, 2); break;  //DIGITAL PIN position

      case 'V': handle_VARIABLE(header, index_node,0); break;   //VARIABLES PIN status      
      
      //     case 'R': send_active_nodes(header); break;              //send active node


      case 'S': /*This is a sleep payload, do nothing*/ break;

      default:
        IF_SERIAL_DEBUG_NRF(printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"), header.type));
        network.read(header, 0, 0);
        break;
    };
  }
}


bool sendCommand (uint16_t to, unsigned char cmd, uint8_t *states, uint8_t num) {
  uint8_t tx_buffer[50];
  radio.powerUp();
  tx_buffer[0] = cmd;
  tx_buffer[1] = num;
  copyArray(states, tx_buffer + 2, 0, num);

  delay(100);

  RF24NetworkHeader header(/*to node*/ to, /*type*/ cmd /*Time*/);

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("---------------------------------\n\r")));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("SENSOR is sending to node 0%o...\n\r"), to));
 
  bool ok = network.write(header, tx_buffer, num + 2);

  IF_SERIAL_DEBUG_NRF(PrintVector(tx_buffer, num + 2));

#if DEBUG_SERIAL_NRF24L == 1
  if (ok)
    printf("Command ok\n\r");
  else
    printf("Command failed\n\r");
  // radio.powerDown();
#endif




  return (ok);
}



/**
 * Send a 'T' message, the current time
 */
bool send_T(uint16_t to)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ 'T' /*Time*/);

  // The 'T' message that we send is just a ulong, containing the time
  unsigned long message = millis();
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("---------------------------------\n\r")));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("%lu: APP Sending %lu to 0%o...\n\r"), millis(), message, to));
  return network.write(header, &message, sizeof(unsigned long));
}




void handle_SYSTEM(RF24NetworkHeader& header, short index_node) {
  uint8_t Ndata_received;
  uint8_t command_rx[34];
  uint16_t system_data[max_num_system_data] = {1, 0, 0, 300, 0, 255};


  if (header.from_node == 00)                      //nothing if base
    return;
  if (index_node == -1)
    index_node = add_nodeFlash(header.from_node);                   //if index = -1, then add the node in the table and extract the new index

  network.read(header, command_rx, 32);
  // printf_P(PSTR("BASE has received SYSTEM INFO from 0%o\n\r"), header.from_node);


  //command_rx[1]  --> number of received data
  Ndata_received = command_rx[1];


  if ((Ndata_received / 2) < sizeof(system_data))  {  //base

    copyArray(command_rx, (uint8_t *)system_data, 2, Ndata_received);

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("SYSTEM COMMAND DECODED, node: 0%o, index: %d, data:"), header.from_node, index_node));
    IF_SERIAL_DEBUG_NRF(PrintVector(system_data, Ndata_received / 2));


    //send back SYSTEM to sensors
    if (SystemNRF24LPins[index_node].system_back_wr) {
      system_data[2] = SystemNRF24LPins[index_node].sleep_time_value;
      bool ok = sendCommand(header.from_node, 'Y', (uint8_t *)system_data, Ndata_received);
      if (ok) {
        SystemNRF24LPins[index_node].system_back_wr = false;             //no more write request
        IF_SERIAL_DEBUG_NRF(printf_P(PSTR("System written on remote sensor successfully\r\n")));
      }
    } else {

      //////////////////////
      //UPDATE: if no write request
      //REMOTE SUPPLY
      //REMOTE ADC STEP
      //REMOTE NRF ADDRESS
      //////////////////////
      SystemNRF24LPins[index_node].Supply = system_data[0];                     //int16  supply *1000
      SystemNRF24LPins[index_node].AdcStep = (((float)system_data[0]) / 1000) / 1024; //float  adc step
      SystemNRF24LPins[index_node].RNF24LAddr = header.from_node;
      SystemNRF24LPins[index_node].samples = system_data[1];
      SystemNRF24LPins[index_node].sleep_time_value = system_data[2];
      SystemNRF24LPins[index_node].fw_version = system_data[3];
      SystemNRF24LPins[index_node].pa_level = system_data[4];
      SystemNRF24LPins[index_node].leaf = system_data[5];
      SystemNRF24LPins[index_node].alive_counter = system_data[2] * 4;    //copy the sleep time * 4. This value will be decremented by one. If it will reach zero, means that the sensor doesn't respond
    }

#if DEBUG_SERIAL_NRF24L == 1
    Serial.print("Supply: "); Serial.print(SystemNRF24LPins[index_node].Supply); Serial.println("V");
    Serial.print("AdcStep: "); Serial.print(SystemNRF24LPins[index_node].AdcStep, 6); Serial.println("V");
    printf_P(PSTR("RNF24LAddr: 0%o\n\r"), SystemNRF24LPins[index_node].RNF24LAddr);
    printf_P(PSTR("samples: %d\n\r"), SystemNRF24LPins[index_node].samples);
    printf_P(PSTR("pa level: %d\n\r"), SystemNRF24LPins[index_node].pa_level);
    printf_P(PSTR("alive_counter: %d\n\r"), SystemNRF24LPins[index_node].alive_counter);
    printf_P(PSTR("leaf: %d\n\r"), SystemNRF24LPins[index_node].leaf);
    printf_P(PSTR("firmware version: %d\n\r"), SystemNRF24LPins[index_node].fw_version);
#endif

  }


}




void handle_DIGITAL(RF24NetworkHeader& header, short index_node, byte mode) {

  uint8_t command_rx[34];
  uint8_t j, Ndata_received;
  uint8_t digital_pins[max_num_digital_pins];                                    // {2, 3, 4, 5, 9};
  uint8_t digital_states[max_num_digital_pins];                                  // pin values
  bool digital_pins_mode[max_num_digital_pins];                                  //{INPUT, INPUT, INPUT, INPUT, OUTPUT};
  // The 'T' message is just a ulong, containing the time
  bool transmit_OutPin = false;

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("PRE index: %d, num_active_nodes:%d\n\r"), index_node, num_active_nodes));
  if (header.from_node == 00)                      //nothing if base
    return;
  if (index_node == -1)
    index_node = add_nodeFlash(header.from_node);                   //if index = -1, then add the node in the table and extract the new index
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("AFTER index: %d, num_active_nodes:%d\n\r"), index_node, num_active_nodes));




  network.read(header, command_rx, 32);


  //command_rx[1]  --> number of received data
  Ndata_received = command_rx[1];

  SystemNRF24LPins[index_node].dig_num = Ndata_received;

  // pin values
  if (mode == 0) {
    copyArray(command_rx, digital_states, 2, Ndata_received);

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL PIN STATE, node: 0%o, index: %d, data:"), header.from_node, index_node));
    IF_SERIAL_DEBUG_NRF(PrintVector(digital_states, Ndata_received));


    //update base vectors according to remote sensor data
    //update also output on remote sensor
    for (j = 0; j < Ndata_received; j++)  {
      //only if is OUTPUT and W=true (write request)
      if (SystemNRF24LPins[index_node].DigPin[j].W == true && SystemNRF24LPins[index_node].DigPin[j].mode == OUTPUT) {
        IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN: %d, has to be updated\r\n"), SystemNRF24LPins[index_node].DigPin[j].pin));
        digital_states[j] = SystemNRF24LPins[index_node].DigPin[j].Wstate;                                               //update the vector with the new data to be updated on sensor node
        SystemNRF24LPins[index_node].DigPin[j].state = digital_states[j];         //used to update the output immediately
        transmit_OutPin = true;
      }
      //update only input pin or the output when no write operation has been done
      else {
        SystemNRF24LPins[index_node].DigPin[j].state = digital_states[j];
      }
    }

    //transmit back OUTPUT pin values if the user has to change the values
    if (transmit_OutPin) {
      IF_SERIAL_DEBUG_NRF(printf_P(PSTR("-------------------->>>>>>> DIGITAL OUT PIN sending back to remote sensor(D)\r\n")));
      bool ok = sendCommand(header.from_node, 'D', digital_states, Ndata_received);
      //      if (ok) {
      IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN written on remote sensor successfully\r\n")));
      //clearing all the w flag because the command has been transimmted correctly
      for (j = 0; j < Ndata_received; j++)
        SystemNRF24LPins[index_node].DigPin[j].W = false;
      //      }
    }



  }

  //read the pin position for each field
  //{2, 3, 4, 5, 9};
  else if (mode == 2) {
    copyArray(command_rx, digital_pins, 2, Ndata_received);

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL PIN POSITION, node: 0%o, index: %d, data:"), header.from_node, index_node));
    IF_SERIAL_DEBUG_NRF(PrintVector(digital_pins, Ndata_received));

    for (j = 0; j < Ndata_received; j++)  {
      SystemNRF24LPins[index_node].DigPin[j].pin = digital_pins[j];
      SystemNRF24LPins[index_node].DigPin[j].used = true;
    }

  }
  //{INPUT, INPUT, INPUT, INPUT, OUTPUT};
  else if (mode == 1) {
    copyArray(command_rx, (uint8_t *)digital_pins_mode, 2, Ndata_received);

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL PIN MODE, node: 0%o, index: %d, data:"), header.from_node, index_node));
    IF_SERIAL_DEBUG_NRF(PrintVector((uint8_t *)digital_pins_mode, Ndata_received));

    for (j = 0; j < Ndata_received; j++)
      SystemNRF24LPins[index_node].DigPin[j].mode = digital_pins_mode[j];
  }

}


bool ANDRUINO_NRF::send_digital_out(uint16_t tx_to_node, byte type, byte channel, byte new_state) {

  uint8_t digital_vector[4];

  digital_vector[0] = 3;
  digital_vector[1] = type;
  digital_vector[2] = channel;
  digital_vector[3] = new_state;

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("-------------------->>>>>>> DIGITAL OUT PIN single write(Q)\r\n")));
  bool ok = sendCommand(tx_to_node, 'Q', digital_vector, sizeof(digital_vector));
  if (ok)
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN written on remote sensor successfully\r\n")));
  else
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN error\r\n")));

  return ok;
}

/*
bool ANDRUINO_NRF::send_digital_vector(uint16_t tx_to_node, byte index_node, byte type, byte channel, byte new_state) {

  uint8_t digital_states[max_num_digital_pins];
  uint8_t Ndata_received;

  Ndata_received = SystemNRF24LPins[index_node].dig_num;

  //update base vectors according to remote sensor data
  //update also output on remote sensor
  for (uint8_t j = 0; j < Ndata_received; j++)  {
    //only if is OUTPUT and W=true (write request)
    if (SystemNRF24LPins[index_node].DigPin[j].W == true && SystemNRF24LPins[index_node].DigPin[j].mode == OUTPUT) {
      IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN: %d, has to be updated\r\n"), SystemNRF24LPins[index_node].DigPin[j].pin));
      digital_states[j] = SystemNRF24LPins[index_node].DigPin[j].Wstate;                                               //update the vector with the new data to be updated on sensor node
      SystemNRF24LPins[index_node].DigPin[j].state = digital_states[j];         //used to update the output immediately
    }
  }
  //transmit back OUTPUT pin values if the user has to change the values
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("-------------------->>>>>>> DIGITAL OUT PIN sending to remote sensor\r\n")));
  bool ok = sendCommand(tx_to_node, 'D', digital_states, Ndata_received);
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("DIGITAL OUT PIN written on remote sensor successfully\r\n")));
  //clearing all the w flag because the command has been transimmted correctly
  for (uint8_t j = 0; j < Ndata_received; j++)
    SystemNRF24LPins[index_node].DigPin[j].W = false;

}
*/



void handle_ANALOG(RF24NetworkHeader& header, short index_node, byte mode) {

  uint8_t command_rx[34];
  uint8_t j, Ndata_received;
  uint8_t analog_pins[num_analog_pins];                         //{2, 3};
  uint16_t analog_states[num_analog_pins];


  if (header.from_node == 00)                      //nothing if base
    return;
  if (index_node == -1)
    index_node = add_nodeFlash(header.from_node);                   //if index = -1, then add the node in the table and extract the new index


  // The 'T' message is just a ulong, containing the time
  network.read(header, command_rx, 32);
  //  printf_P(PSTR("BASE has received ANALOG DATA from 0%o\n\r"), header.from_node);

  //command_rx[1]  --> number of received data (1 data = 8 bit)
  Ndata_received = command_rx[1];

  SystemNRF24LPins[index_node].ana_num = Ndata_received / 2;  //16 bits

  if (mode == 0) {
    copyArray(command_rx, (uint8_t *)analog_states, 2, Ndata_received);    //analog_states 16 bits

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("ANALOG VALUES, node: 0%o, index: %d, data:"), header.from_node, index_node));
    IF_SERIAL_DEBUG_NRF(PrintVector(analog_states, Ndata_received / 2));


#if DEBUG_SERIAL_NRF24L == 1
    float vcc_remote = ((float) SystemNRF24LPins[index_node].Supply) / 1000.0;
    Serial.print("Remote batt voltage: ");
    Serial.print(vcc_remote, 2);
    Serial.println("V");
#endif

    for (j = 0; j < Ndata_received / 2; j++)
      SystemNRF24LPins[index_node].AnaPin[j].state = analog_states[j];
  }
  else if (mode == 1) {
    copyArray(command_rx, analog_pins, 2, Ndata_received);    //analog_pins 8 bits

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("ANALOG PIN POSITION, node: 0%o, index: %d, data:"), header.from_node, index_node));
    IF_SERIAL_DEBUG_NRF(PrintVector(analog_pins, Ndata_received));


    SystemNRF24LPins[index_node].RNF24LAddr = header.from_node;
    for (j = 0; j < Ndata_received; j++)  {
      SystemNRF24LPins[index_node].AnaPin[j].pin = analog_pins[j];
      SystemNRF24LPins[index_node].AnaPin[j].used = true;
    }
  }
}

//One Variable is 32bit
void handle_VARIABLE(RF24NetworkHeader& header, short index_node, byte mode) {

  uint8_t command_rx[34];
  uint8_t j, Ndata_received;
  float variable_states[NRF24LMaxVariable];


  if (header.from_node == 00)                      //nothing if base
    return;
  if (index_node == -1)
    index_node = add_nodeFlash(header.from_node);                   //if index = -1, then add the node in the table and extract the new index

 // IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Payload: %d \n\r"), radio.getPayloadSize()));
  
  network.read(header, command_rx, 32);
  Ndata_received = command_rx[1];


  //IF_SERIAL_DEBUG_NRF(PrintVector(command_rx, Ndata_received+2));


  SystemNRF24LPins[index_node].var_num = Ndata_received / 4;  //32 bits
  copyArray(command_rx, (uint8_t *)variable_states, 2, Ndata_received);    //analog_states 16 bits

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("VARIABLE VALUES, node: 0%o, index: %d, data:"), header.from_node, index_node));
  IF_SERIAL_DEBUG_NRF(PrintVector(variable_states, Ndata_received / 4));

  for (j = 0; j < Ndata_received / 4; j++)
    SystemNRF24LPins[index_node].VarPin[j].value = variable_states[j];

}

/**
 * Handle a 'T' message
 * Add the node to the list of active nodes
 */
void handle_T(RF24NetworkHeader& header) {

  unsigned long message;
  network.read(header, &message, sizeof(unsigned long));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("handle_T: %lu from 0%o\n\r"), message, header.from_node));

  //Add if it is from a node (not base)
  if (header.from_node > 00 )
    add_nodeFlash(header.from_node);
}

/**
 * Handle an 'N' message, the active node list
 */
void handle_N(RF24NetworkHeader& header)
{
  static uint16_t incoming_nodes[max_active_nodes];

  network.read(header, &incoming_nodes, sizeof(incoming_nodes));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("handle_N: Received active nodes from 0%o\n\r"), header.from_node));

  int i = 0;
  while ( i < max_active_nodes && incoming_nodes[i] > 00 )
    add_nodeFlash(incoming_nodes[i++]);
}




//search the node in the flash list
//if it is not present, add it
//refresh num_active_nodes number var

short add_nodeFlash(uint16_t node) {
  short i = num_active_nodes;

  //is there the node in the list?
  while (i--)  {
    //check if it is already present
    if (eeprom_read_wordNEW ((FLA_NRF_NODE_ADDRESS + i * 2)) == node)
      break;
  }
  //the current node is not in the flash list --> store it
  if ( i == -1 && num_active_nodes < max_active_nodes ) {        // If not, add it to the table
    i = num_active_nodes;
    eeprom_write_wordNEW ((FLA_NRF_NODE_ADDRESS + i * 2), node); //write active node to flash
    num_active_nodes++;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Added 0%o to list of active nodes, index: %d, num_active_nodes:%d\n\r"), node, i, num_active_nodes));
  }
  return (i);     //return the index of the new node
}


//search the node in the whole flash list
//-1 if it is not present
//refresh num_active_nodes number var

//2 bytes for each node
short search_nodeFlash(uint16_t node) {

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Search node 0%o, num_active_nodes: %d\n\r"), node, num_active_nodes));
  short i = NRF24LMaxModules;                                    // Do we already know about this node?
  bool found_max_node = false;
  uint16_t data;
  
  while (i--)  {
    data = eeprom_read_wordNEW ((FLA_NRF_NODE_ADDRESS + i * 2));
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Module index:%d - node: 0%o\n\r"), i, data));
    //find the last element of the node list
    if (data != 65535  && found_max_node == false) {
      num_active_nodes = i + 1;
      found_max_node = true;
    }
    if (data == node) {
      IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Node already in the flash list, node 0%o, index: %d\n\r"), node, i));
      break;
    }
  }
#if DEBUG_SERIAL_NRF24L == 1
  if (found_max_node)
    printf_P(PSTR("num_active_nodes:%d\n\r"), num_active_nodes);
#endif

  if ( i == -1 && num_active_nodes < max_active_nodes ) {        // If not, add it to the table
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Node 0%o not found in the flash list\n\r"), node));
    return (i);  //not found
  }

  return (i);  //inded of node found
}



//copyArray(pippo, tano, 1, 8);
void copyArray (uint8_t *source, uint8_t *destination, uint8_t start_byte, uint8_t num_byte) {

  //(destination, source, Ndata)
  memcpy(destination, source + start_byte, num_byte );
  /*  for (int i = 0; i < num_byte; i++) {
      destination[i] = source[i + start_byte];
    }*/
}

void PrintVector (uint8_t *states, int i) {
  for (int k = 0; k < i; k++)
  {
    Serial.print(states[k]);
    if (k < i - 1) {
      Serial.print(", ");
    } else
      Serial.println();
  }
}
void PrintVector (uint16_t *states, int i) {
  for (int k = 0; k < i; k++)
  {
    Serial.print(states[k]);
    if (k < i - 1) {
      Serial.print(", ");
    } else
      Serial.println();
  }
}
void PrintVector (float *states, int i) {
  for (int k = 0; k < i; k++)
  {
    Serial.print(states[k], 2);
    if (k < i - 1) {
      Serial.print(", ");
    } else
      Serial.println();
  }
}

#endif

