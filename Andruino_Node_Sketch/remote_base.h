

#ifndef __REMOTE_BASE_H__
#define __REMOTE_BASE_H__

#include "support.h"
#include <TimerOne.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "vcc.h"

long vcc_arduino;


#define ROUTER_TX_DATA 10           //the router tx the data each 10 seconds
#define STARTUP_SLEEP_TIME 2
#define DEFAULT_SLEEP_TIME 120





RF24 radio(PIN_NRF_CE, PIN_NRF_CS);                                   // CE & CS pins to use (Using 7,8 on Uno,Nano)
RF24Network network(radio);

//address of some flash configurations
#define FLA_SLEEP_ADDRESS_ok 0
#define FLA_SLEEP_ADDRESS_data16 1

#define FLA_NODE_ADDRESS_ok 3
#define FLA_NODE_ADDRESS_add 4

#if DEBUG_SERIAL_NRF24L == 1
#define IF_SERIAL_DEBUG_NRF(x) ({x;})
#else
#define IF_SERIAL_DEBUG_NRF(x)
#endif




uint8_t node_address_index = 0;
uint16_t sleep_counter = 0;

uint16_t this_node;                                  // Our node address
uint16_t sleep_time;
bool startup_phase;
bool first_read = true;

bool analog_send_half = false;

//used to speed up the sleep when the App is changing a parameter
bool fast_sensor_scan = false;
uint16_t fast_sensor_scan_counter = 0;
#define MAX_FAST_SENSOR_SCAN 10           //


bool send_sensors_immediatly = false;

unsigned long last_time_sent;
unsigned long now;

unsigned error_transmit_total = 0;
uint8_t error_transmit = 0;
uint16_t good_transmit_total = 0;
uint8_t good_transmit = 0;

uint8_t pa_level = RF24_PA_HIGH;


const short max_active_nodes = 10;                    // Array of nodes we are aware of
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;

uint16_t timeout_base_num = 0;

uint16_t second_counter = 0;
uint8_t phase_counter = 0;




//used to send some info in certain time (not always)
bool ok_system = false;
bool ok_dig_pin_mode = false;
bool ok_dig_pin_pos = false;
bool ok_analog_pos = false;



bool send_T(uint16_t to);                              // Prototypes for functions to send & handle messages
bool send_N(uint16_t to);
bool sendCommand (uint16_t to, unsigned char cmd, uint8_t *states, uint8_t num);
void handle_T(RF24NetworkHeader& header);
void handle_N(RF24NetworkHeader& header);
void add_node(uint16_t node);
void handle_DIGITAL(RF24NetworkHeader& header);
void flash_setup();
void Interrupt_Timer_1sec ();
void handle_DIGITAL (RF24NetworkHeader& header);
void handle_DIGITAL_new (RF24NetworkHeader& header);
void handle_ANALOG (RF24NetworkHeader& header);
void handle_SYSTEM (RF24NetworkHeader& header);
void check_rf();
bool send_setup_to_base ();
bool send_data_to_base ();
void print_active_nodes ();
void go_to_sleep_mode ();


//This is for sleep mode. It is not really required, as users could just use the number 0 through 10
typedef enum {
  wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s
}
wdt_prescalar_e;



unsigned long sleepTimer = 0;                           // Used to keep track of how long the system has been awake


bool send_data_phase = false;
bool send_data_setup_phase = false;
bool go_sleep_phase = false;




void setup_node() {




  pinMode(2, INPUT);        //interrupt pin

  Serial.begin(9600);
  printf_begin();
  printf_P(PSTR("#########################################\n\r"));
  flash_setup();

  this_node = node_address_array[node_address_index];            // Which node are we?

  printf_P(PSTR("Andruino Remote sensor node:0%o\n\r"), this_node);
  //print firmware version
  printf_P(PSTR("Firmware: %d\n\r"), VERSION_FW);

  if (NODE_CONFIG == LEAF)
    printf_P(PSTR("Node configured as LEAF\n\r"));
  else
    printf_P(PSTR("Node configured as ROUTER\n\r"));


  printf_P(PSTR("#########################################\n\r\n\r"));



  SPI.begin();                                           // Bring up the RF network
  radio.begin();

  //Set Power Amplifier (PA) level to one of four levels:
  //{ RF24_PA_MIN = 0,RF24_PA_LOW=1, RF24_PA_HIGH=2, RF24_PA_MAX=3, RF24_PA_ERROR }
  radio.setPALevel(RF24_PA_HIGH);

  //radio.setDataRate(RF24_250KBPS);      //speed  RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  //radio.setRetries(15, 15);             //How long to wait between each retry, in multiples of 250us, max is 15. 0 means 250us, 15 means 4000us.

  network.choose_pipe_address(nrf_pipeline_id);
  network.begin(/*channel*/ NRF_CHANNEL_NUM, /*node address*/ this_node );


  radio.startListening();
  radio.printDetails();            // Dump the configuration of the rf unit for debugging
  radio.stopListening();

  SetupDigitalPins(num_digital_pins);
  SetupVars(num_variable_pins,  variable_states);


  /******************************** This is the configuration for sleep mode ***********************/
  network.setup_watchdog(wdt_1s);                       //The watchdog timer will wake the MCU and radio every second to send a sleep payload, then go back to sleep


  vcc_arduino = readVcc()+10;

  Timer1.initialize(1000000);                         //timer set for each second
  Timer1.attachInterrupt(Interrupt_Timer_1sec);       //attach the interrupt routine
  
}


void Interrupt_Timer_1sec () {
  bool ok, send_setup_at_startup = false, send_setup_running = false;

  second_counter++;
  phase_counter++;

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("**********phase_counter: %d "), phase_counter));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("***sleep_counter: %d "), sleep_counter));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("***second_counter: %d ************\n\r"), second_counter));


  if (sleep_counter < 15)
    startup_phase = true;
  else
    startup_phase = false;

  if ((sleep_counter % 30 == 0) && startup_phase == false)         //running
    send_setup_running = true;
  else if ((sleep_counter % 2 == 0) && startup_phase == true)     //during the startup phase
    send_setup_at_startup = true;


  /* if ((sleep_counter % 10 == 0)) {
     if (good_transmit > 10) {
       good_transmit=0;
       pa_level--;
       radio.setPALevel(pa_level);
     }
   }
  */
  if (send_sensors_immediatly) {
    send_sensors_immediatly = false;
    phase_counter = 1;
  }



  switch (phase_counter) {

    case 1:
      if (send_setup_at_startup)
        send_data_setup_phase = true;         //send sensor setup data
      else
        send_data_phase = true;               //send sensor data
      break;
    case 2:

      /*
      #if NODE_CONFIG == 1
          //decrease  PA level if PASS for 10 times (executed only each 20 seconds)
          if (good_transmit > 10 && (sleep_counter % 5 == 0)) {
            error_transmit=0;
            good_transmit=0;
            if(pa_level == RF24_PA_MIN)
              pa_level = RF24_PA_MIN;
            else
              pa_level--;

            radio.begin();
            radio.setPALevel(pa_level);
            network.begin(NRF_CHANNEL_NUM, this_node );
          }
          //increase PA level if fail
          if (error_transmit > 3) {
            good_transmit=0;
            error_transmit=0;
            if(pa_level == RF24_PA_HIGH)
              pa_level = RF24_PA_HIGH;
            else
              pa_level++;
            radio.begin();
            radio.setPALevel(pa_level);
            network.begin( NRF_CHANNEL_NUM, this_node );
          }
      #endif
      */
      break;
    case 3 :
      break;
    case 4 :
      break;
    case 5:
      if (send_setup_running)
        send_data_setup_phase = true;         //send sensor setup during running phase
      break;
    case 6 :
      break;

    default:
      if (NODE_CONFIG == LEAF) {             //LEAF NODE
        go_sleep_phase = true;
        phase_counter = 0;
        sleep_counter++;
        break;
      }
      else {                                //ROUTER NODE
        if (phase_counter == ROUTER_TX_DATA) {
          go_sleep_phase = true;
          phase_counter = 0;
          sleep_counter++;
        }
      }
      break;

  }




  bool changed = readDigitals(num_digital_pins, digital_pins, digital_states);
  if (changed && NODE_CONFIG == ROUTER)  {                                              //if the pin has changed and the node is a router
    if (first_read)
      first_read = false;                                                       //if first read, don't send use the "immediatly" phase
    else
      send_sensors_immediatly = true;
  }




}





void check_rf() {

  network.update();
  // Pump the network regularly
  /////////////////////////////////////////////
  //check if the RX has received data
  while ( network.available() )  {                      // Is there anything ready for us?

    RF24NetworkHeader header;                            // If so, take a look at it
    network.peek(header);


    switch (header.type) {                             // Dispatch the message to the correct handler.
      //T and N has to be at the beginning, don't move them!
      case 'T':
        handle_T(header);
        break;
      case 'N':
        handle_N(header);
      case 'D':
        IF_SERIAL_DEBUG_NRF(printf_P(PSTR("D-------------->>>>>>>DIGITAL WRITE RECEIVED BACK from base(D)\n\r")));
        handle_DIGITAL(header);
        break;
      case 'Q':
        IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Q-------------->>>>>>>DIGITAL WRITE RECEIVED from base(Q)\n\r")));
        handle_DIGITAL_new(header);
        break;

      case 'Y':
        IF_SERIAL_DEBUG_NRF(printf_P(PSTR("-------------->>>>>>>SYSTEM WRITE RECEIVED BACK from base\n\r")));
        handle_SYSTEM(header);                                //DIGITAL PIN status
        break;

        break;

      /************* SLEEP MODE *********/
      // Note: A 'sleep' header has been defined, and should only need to be ignored if a node is routing traffic to itself
      // The header is defined as:  RF24NetworkHeader sleepHeader(/*to node*/ 00, /*type*/ 'S' /*Sleep*/);
      case 'S': /*This is a sleep payload, do nothing*/
        break;

      default:
        IF_SERIAL_DEBUG_NRF(printf_P(PSTR("*** WARNING *** Unknown message type %c\n\r"), header.type));
        network.read(header, 0, 0);
        break;
    };

  }



  if (go_sleep_phase && NODE_CONFIG == LEAF) {                  //sleep only if the node is a LEAF
    go_sleep_phase = false;
    go_to_sleep_mode ();
    last_time_sent = 0;                                           //force send data to base after wake-up
  }

  if (send_data_setup_phase) {
    send_data_setup_phase = false;
    send_setup_to_base();
  }
  else if (send_data_phase) {
    send_data_phase = false;
    variable_states[4]++;        //increment the data_tx counter
    send_data_to_base();
  }

   

}






void go_to_sleep_mode () {



  IF_SERIAL_DEBUG_NRF(print_active_nodes());
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Going in sleep mode:"));   Serial.print(now); Serial.println(" ms"));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Transmit TOTAL error: %d\n\r"), error_transmit_total));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Transmit TOTAL good: %d\n\r"), good_transmit_total));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Transmit error tx: %d\n\r"), error_transmit));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Transmit good tx: %d\n\r"), good_transmit));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Transmit pa level: %d\n\r"), radio.getPALevel()));


  if (!fast_sensor_scan && NODE_CONFIG == LEAF)              //only for LEAF node (switch off RADIO RX during sleep)
    radio.stopListening();                           // Switch to PTX mode. Payloads will be seen as ACK payloads, and the radio will wake up

  // network.sleepNode( cycles (sec), interrupt-pin (255=no INT));
  // network.sleepNode(0,0);         // The WDT is configured in this example to sleep in cycles of 1 second. This will sleep 1 second, or until a payload is received
  // network.sleepNode(1,255);       // Sleep this node for 1 second. Do not wake up until then, even if a payload is received ( no interrupt ) Payloads will be lost.


  uint16_t sleep_time_real;

/*  if (timeout_base_num > 10) {
    if (timeout_base_num < 20)
      sleep_time_real = sleep_time * 2;
    else if (timeout_base_num > 20 && timeout_base_num < 50)
      sleep_time_real = sleep_time * 5;
    else
      sleep_time_real = sleep_time * 10;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sleep time multiplied because the base doesn't respond: %d sec\n\r"), sleep_time_real));
  }
  else {*/
    sleep_time_real = sleep_time;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sleep time: %d sec\n\r"), sleep_time_real));
//  }


  // Give the Serial print some time to finish up


  if (startup_phase || fast_sensor_scan) {

    if (fast_sensor_scan) {
      fast_sensor_scan_counter++;
      if (fast_sensor_scan_counter > MAX_FAST_SENSOR_SCAN) {
        fast_sensor_scan = false;
      }
    }

    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sleep time reduced to 2 only at start-up or when App is connected\n\r")));
    delay(100);
    network.sleepNode(STARTUP_SLEEP_TIME, 0);  // Sleep the node for 8 cycles of 1second intervals. 0=means interrupt on PIN2
  }
  else {
    delay(100);
    network.sleepNode(sleep_time_real, 0);  // Sleep the node for 8 cycles of 1second intervals. 0=means interrupt on PIN2
  }



  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Awaking from sleep mode:"));   Serial.print(millis()); Serial.println(" ms"));





}

bool send_setup_to_base () {
  unsigned long t1, t2, delta;
  t1 = millis();
  uint16_t to = 00;                                   // Who should we send to? By default, send to base
  bool ok;

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("SENDING SETUP TO BASE\n\r")));





  system_data[0] = vcc_arduino;
  system_data[2] = sleep_time;
  system_data[3] = VERSION_FW;
  system_data[4] = pa_level;
  system_data[5] = NODE_CONFIG;


#if DEBUG_SERIAL_NRF24L == 1
  float vcc = ((float) system_data[0]) / 1000.0;
  Serial.print("Batt voltage: "); Serial.print(vcc, 3); Serial.println("V");
#endif

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sending system data...\n\r")));


  ok_system = sendCommand(to, 'Y', (uint8_t *)system_data, sizeof(system_data));

  //if the base doesn't respond, stop the transmission, sleep
  if (!ok_system)
    return false;

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Now sending digitals...\n\r")));
  ok = sendCommand(to, 'D', digital_states, num_digital_pins);

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Now sending analog pin position...\n\r")));
  ok_analog_pos = sendCommand(to, 'a', analog_pins, num_analog_pins);


  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Now sending digitals pin mode...\n\r")));
  ok_dig_pin_mode = sendCommand(to, '1', (uint8_t *)digital_pins_mode, num_digital_pins);


  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Now sending digitals pin position...\n\r")));
  ok_dig_pin_pos = sendCommand(to, 'd', digital_pins, num_digital_pins);


  if (ok_system && ok_analog_pos && ok_dig_pin_mode && ok_dig_pin_pos) {                                             // Notify us of the result
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Pin setup sent ok\n\r")));
    timeout_base_num = 0;
  }
  else {
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Pin setup sent failed\n\r")));
    last_time_sent -= 100;                            // Try sending at a different time next time
    timeout_base_num++;
  }

#if DEBUG_SERIAL_NRF24L == 1
  t2 = millis();
  delta = t2 - t1;
  printf_P(PSTR("Send finished send_setup_to_base, now:")); Serial.print(t2); printf_P(PSTR(" ms"));
  printf_P(PSTR(", delta:")); Serial.print(delta); printf_P(PSTR(" ms\n\r"));
#endif

  return ok_dig_pin_pos;
}



//2.5 SEC
bool send_data_to_base () {
  unsigned long t1, t2, delta;
  t1 = millis();
  uint16_t to = 00;                                   // Who should we send to? By default, send to base
  bool ok = true;;

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("SENDING DATA TO BASE\n\r")));



  system_data[0] = vcc_arduino;
  system_data[4] = pa_level;

#if DEBUG_SERIAL_NRF24L == 1
  float vcc = ((float) system_data[0]) / 1000.0;
  Serial.print("Batt voltage: "); Serial.print(vcc, 3); Serial.println("V");
#endif


  //ok = send_T(to);
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sending system data...\n\r")));
  system_data[1]++;


  //   if(!ok_system)
  ok_system = sendCommand(to, 'Y', (uint8_t *)system_data, sizeof(system_data));

  //if the base doesn't respond, stop the transmission, sleep
  if (!ok_system)
    return false;


  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Now sending digitals...\n\r")));
  ok = sendCommand(to, 'D', digital_states, num_digital_pins);

  if (analog_send_half) {
    analog_send_half = false;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Now sending analogs:\n\r")));
    vcc_arduino = readVcc()+10;                                                    //read device vcc
    readAnalogs(num_analog_pins, analog_pins, analog_states);                   //read analog and update analog vector
    ok = sendCommand(to, 'A', (uint8_t *)analog_states, num_analog_pins * 2);
  }
  else {
    analog_send_half = true;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Now sending variables:\n\r")));
    ok = sendCommand(to, 'V', (uint8_t *)variable_states, num_variable_pins * 4);
  }

  if (ok) {                                             // Notify us of the result
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("APP Send ok\n\r")));
    timeout_base_num = 0;
  }
  else {
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("APP Send failed\n\r")));
    last_time_sent -= 100;                            // Try sending at a different time next time
    timeout_base_num++;
  }

#if DEBUG_SERIAL_NRF24L == 1
  t2 = millis();
  delta = t2 - t1;
  printf_P(PSTR("Send finished send_data_to_base, now:")); Serial.print(t2); printf_P(PSTR(" ms"));
  printf_P(PSTR(", delta:")); Serial.print(delta); printf_P(PSTR(" ms\n\r"));
#endif
  return ok;
}




bool sendCommand (uint16_t to, unsigned char cmd, uint8_t *states, uint8_t num) {
  uint8_t tx_buffer[34];
  radio.powerUp();
  tx_buffer[0] = cmd;
  tx_buffer[1] = num;
  copyArray(states, tx_buffer + 2, 0, num);
  

  //  delay(100);
  RF24NetworkHeader header(/*to node*/ to, /*type*/ cmd /*Time*/);
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("SENSOR is sending to node 0%o...\n\r"), to));
  
  
  bool ok = network.write(header, tx_buffer, num + 2);
  IF_SERIAL_DEBUG_NRF(PrintVector(tx_buffer, num + 2));

  if (ok) {
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Command ok, error count: %d\n\r"), error_transmit));
    good_transmit++;
    good_transmit_total++;
  }
  else {
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Command failed, error count: %d  --> Command failed Command failed Command failed Command failed Command failed Command failed Command failed\n\r"), error_transmit));
    error_transmit_total++;
    error_transmit++;
  }
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("---------------------------------\n\r")));
  // radio.powerDown();

  //to avoid data overlapping
  delay(200);
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

/**
 * Send an 'N' message, the active node list
 */
bool send_N(uint16_t to)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ 'N' /*Time*/);

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("---------------------------------\n\r")));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("%lu: APP Sending active nodes to 0%o...\n\r"), millis(), to));
  return network.write(header, active_nodes, sizeof(active_nodes));
}

bool send_single_cmd(uint16_t to, unsigned char cmd)
{
  RF24NetworkHeader header(/*to node*/ to, /*type*/ cmd /*Time*/);
  unsigned long message = 0;
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("---------------------------------\n\r")));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sending command %c to 0%o...\n\r"), cmd, to));
  return network.write(header, &message, sizeof(unsigned long));
}

/**
 * Handle a 'T' message
 * Add the node to the list of active nodes
 */
void handle_T(RF24NetworkHeader& header) {

  unsigned long message;                                                                      // The 'T' message is just a ulong, containing the time
  network.read(header, &message, sizeof(unsigned long));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("%lu: APP Received %lu from 0%o\n\r"), millis(), message, header.from_node));


  if ( header.from_node != this_node || header.from_node > 00 )                                // If this message is from ourselves or the base, don't bother adding it to the active nodes.
    add_node(header.from_node);
}

/**
 * Handle an 'N' message, the active node list
 */
void handle_N(RF24NetworkHeader& header)
{
  static uint16_t incoming_nodes[max_active_nodes];

  network.read(header, &incoming_nodes, sizeof(incoming_nodes));
  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("%lu: APP Received nodes from 0%o\n\r"), millis(), header.from_node));

  int i = 0;
  while ( i < max_active_nodes && incoming_nodes[i] > 00 )
    add_node(incoming_nodes[i++]);
}

/**
 * Add a particular node to the current list of active nodes
 */
void add_node(uint16_t node) {

  short i = num_active_nodes;                                    // Do we already know about this node?
  while (i--)  {
    if ( active_nodes[i] == node )
      break;
  }

  if ( i == -1 && num_active_nodes < max_active_nodes ) {        // If not, add it to the table
    active_nodes[num_active_nodes++] = node;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("%lu: APP Added 0%o to list of active nodes.\n\r"), millis(), node));
  }
}

void print_active_nodes () {

  uint16_t node;

  if (num_active_nodes > 0) {

    printf_P(PSTR("%d active nodes: "), num_active_nodes);
    for (int i = 0; i < num_active_nodes; i++) {
      node = active_nodes[i];
      printf_P(PSTR("0%o"), node);
      if (i < num_active_nodes - 1)
        Serial.print(",");
      else
        Serial.println();
    }

  }
  else {
    printf_P(PSTR("No active nodes are yet present\r\n"));
  }
}









void handle_DIGITAL(RF24NetworkHeader& header) {

  uint8_t command_rx[34];
  uint8_t Ndata_received;
  uint8_t digital_states_from_base[num_digital_pins];


  network.read(header, command_rx, num_digital_pins + 2);
  //  printf_P(PSTR("command_rx:\n\r"));
  //  PrintVector(command_rx, 32);


  //command_rx[1]  --> number of received data
  Ndata_received = command_rx[1];
  copyArray(command_rx, digital_states_from_base, 2, Ndata_received);

  // memcpy( digital_states_from_base, command_rx+2, Ndata_received );


  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("digital_states_from_base:\n\r")));
  IF_SERIAL_DEBUG_NRF(PrintVector(digital_states_from_base, Ndata_received));


  bool ok = true;
  if (ok) {
    OutPinChange (num_digital_pins, digital_states, digital_states_from_base);
  }


  //reduce the sleep time only when the App is modifying this sensor
  fast_sensor_scan = true;
  fast_sensor_scan_counter = 0;

}

void handle_DIGITAL_new (RF24NetworkHeader& header) {

  uint8_t digital_vector[6];

  //      digital_vector[2] = 3;
  //      digital_vector[3] = type;
  //      digital_vector[4] = channel;
  //      digital_vector[5] = new_state;


  network.read(header, digital_vector, sizeof(digital_vector));
  IF_SERIAL_DEBUG_NRF(PrintVector(digital_vector, sizeof(digital_vector)));

  digitalWrite(digital_vector[4], digital_vector[5]);
  printf("Pin OUTPUT number:%d has changed to:%d\n\r", digital_vector[4], digital_vector[5]);


  //reduce the sleep time only when the App is modifying this sensor
  fast_sensor_scan = true;
  fast_sensor_scan_counter = 0;
  //  send_sensors_immediatly = true;
}


//get the sleep time from system data
void handle_SYSTEM(RF24NetworkHeader& header) {

  uint8_t command_rx[34];
  uint8_t Ndata_received;
  uint16_t system_states_from_base[num_system_data];


  network.read(header, command_rx, num_system_data + 2);
  //  printf_P(PSTR("command_rx:\n\r"));
  //  PrintVector(command_rx, 32);


  //command_rx[1]  --> number of received data
  Ndata_received = command_rx[1];
  copyArray(command_rx, (uint8_t *)system_states_from_base, 2, Ndata_received);

  IF_SERIAL_DEBUG_NRF(printf_P(PSTR("system_states_from_base:\n\r")));
  IF_SERIAL_DEBUG_NRF(PrintVector(system_states_from_base, Ndata_received / 2));



  uint16_t sleep_time_tmp = system_states_from_base[2];


  if (sleep_time_tmp > 0 && sleep_time_tmp < 1800) {        //sleep >0 and <30 minutes
    system_data[2] = sleep_time_tmp;
    sleep_time = sleep_time_tmp;
    eeprom_write_byte ((uint8_t *)(FLA_SLEEP_ADDRESS_ok), 0x55);
    eeprom_write_word ((uint16_t *)(FLA_SLEEP_ADDRESS_data16), sleep_time);
  }


  //reduce the sleep time only when the App is modifying this sensor
  fast_sensor_scan = true;
  fast_sensor_scan_counter = 0;

}

void flash_setup() {



  //read sleep time from flash
  if (eeprom_read_byte((uint8_t*)(FLA_SLEEP_ADDRESS_ok)) == 0x55) {
    sleep_time = eeprom_read_word((uint16_t*)(FLA_SLEEP_ADDRESS_data16));
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sleep time is read from flash: %d sec\n\r"), sleep_time));
  } else {
    sleep_time = DEFAULT_SLEEP_TIME;
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Sleep time default: %d sec\n\r"), sleep_time));
  }


  //read sleep time from flash
  if (eeprom_read_byte((uint8_t*)(FLA_NODE_ADDRESS_ok)) == 0x55 && !FORCE_NODE_ADDRESS) {
    node_address_index = eeprom_read_byte((uint8_t*)(FLA_NODE_ADDRESS_add));
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Node address is read from flash: 0%o\n\r"), node_address_index));
  } else {
    node_address_index = DEFAULT_NODE_ADDRESS;
    eeprom_write_byte ((uint8_t *)(FLA_NODE_ADDRESS_ok), 0x55);
    eeprom_write_byte ((uint8_t *)(FLA_NODE_ADDRESS_add), node_address_index);
    IF_SERIAL_DEBUG_NRF(printf_P(PSTR("Node address default: %d sec\n\r"), node_address_index));
  }


}


#endif //

