/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

Wiring For Keystudio ATMEGA2560 R3 Development Board with Sheild v1.3:

ESP32: ATMEGA2560:
ESP32: I2C SDA -> ATMEGA2560: I2C SDA
ESP32: I2C SCL -> ATMEGA2560: I2C SCL

Wiring Satellite Count and HDOP Precision Factor Indicator:
ATMEGA2560 51 -> LEDR
ATMEGA2560 52 -> LEDG
ATMEGA2560 53 -> LEDB

Wiring Overload Indicator:
ATMEGA2560 46 -> LEDR
ATMEGA2560 47 -> LEDGW

Wiring Read/Write Interrupt indicator:
ATMEGA2560 SDA 20 -> ESP32 12
ATMEGA2560 SCL 21 -> ESP32 5

Wiring Matrix Indicators:
ATMEGA2560 48 -> 24x individually addressable WS2812B's

*/

// ------------------------------------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>
#include <FastLED.h>

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     INDICATORS

/*
Indicators:
0-19: Matrix Switch State
20: MATRIX IO ENABLED/DISABLED
21: DATA
22: OVERLOAD
23: SIGNAL
*/

// INDICATORS
#define NUM_LEDS 24
#define DATA_PIN 48
CRGB leds[NUM_LEDS];

// ------------------------------------------------------------------------------------------------------------------------------

#define ETX 0x03  // end of text character

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MATRIX DATA

bool matrix_io_enabled=false;
int gps_signal=0;
bool overload=false;
bool data_received=false;
signed int portmap_number;
bool update_portmap_bool = false;

// ------------------------------------------------------------
// Matrix switch output high/low
// ------------------------------------------------------------
int max_matrix_switch_states = 20;
bool matrix_switch_state[1][20] = {
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  }
};

// ------------------------------------------------------------
// Matrix switch output ports
// ------------------------------------------------------------
signed int matrix_port_map[1][20] = {
  {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  }
};

// ------------------------------------------------------------
// Temporary matrix switch output ports
// ------------------------------------------------------------
signed int tmp_matrix_port_map[1][20] = {
  {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  }
};

// ------------------------------------------------------------
// Matrix Indicator Colors
// ------------------------------------------------------------
int indicator_number=0;
long matrix_indicator_colors[1][20] = {
  {
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1
  }
};

long available_matrix_indicator_colors[8] = {
  CRGB::Black,
  CRGB::Red,
  CRGB::Yellow,
  CRGB::Green,
  CRGB::Blue,
  CRGB::Cyan,
  CRGB::Purple,
  CRGB::White,
};

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    TIME STRUCT

struct TimeStruct {
  double mainLoopTimeTaken; // current main loop time
  unsigned long mainLoopTimeStart; // time recorded at the start of each iteration of main loop
};
TimeStruct timeData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      I2C DATA

#define SLAVE_ADDR 9

struct I2CLinkStruct {
  char * token;
  byte OUTPUT_BUFFER[32];
  char INPUT_BUFFER[32];
  char TMP_BUFFER[32];
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     I2C EVENTS

void receiveEvent(int) {

  // ------------------------------------------------------------
  // Indicate data received
  // ------------------------------------------------------------
  data_received=true;
  
  // ------------------------------------------------------------
  // Read incoming data
  // ------------------------------------------------------------
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));

  // ------------------------------------------------------------
  // Tokenize data tag
  // ------------------------------------------------------------
  I2CLink.token = strtok(I2CLink.INPUT_BUFFER, ",");

  // ------------------------------------------------------------
  // Matrix Enabled
  // ------------------------------------------------------------
  if (strcmp(I2CLink.token, "$MENABLED")==0) {
    I2CLink.token = strtok(NULL, ",");
    if (strcmp(I2CLink.token, "0") == 0) {matrix_io_enabled=false;}
    else {matrix_io_enabled=true;}
  }

  // ------------------------------------------------------------
  // GPS Signal
  // ------------------------------------------------------------
  else if (strcmp(I2CLink.token, "$GPSSIG")==0) {
    I2CLink.token = strtok(NULL, ",");
    if      (strcmp(I2CLink.token, "0") == 0) {gps_signal=0;}
    else if (strcmp(I2CLink.token, "1") == 0) {gps_signal=1;}
    else if (strcmp(I2CLink.token, "2") == 0) {gps_signal=2;}
    else                                      {gps_signal=3;}
  }

  // ------------------------------------------------------------ 
  // Overload
  // ------------------------------------------------------------
  else if (strcmp(I2CLink.token, "$OLOAD")==0) {
    I2CLink.token = strtok(NULL, ",");
    if      (strcmp(I2CLink.token, "0") == 0) {overload=false;}
    else if (strcmp(I2CLink.token, "1") == 0) {overload=true;}
  }

  // ------------------------------------------------------------
  // Port Map
  // ------------------------------------------------------------
  else if (strcmp(I2CLink.token, "$P")==0) {
    I2CLink.token = strtok(NULL, ",");
    portmap_number = atoi(I2CLink.token);
    I2CLink.token = strtok(NULL, ",");
    matrix_port_map[0][portmap_number] = atoi(I2CLink.token);
    Serial.println("[portmap] " + String(portmap_number) + String(" [port number] ") + String(matrix_port_map[0][portmap_number]));
  }

  // ------------------------------------------------------------
  // Port States
  // ------------------------------------------------------------
  else if (strcmp(I2CLink.token, "$M")==0) {
    I2CLink.token = strtok(NULL, ",");
    portmap_number = atoi(I2CLink.token);
    I2CLink.token = strtok(NULL, ",");
    matrix_switch_state[0][portmap_number] = atoi(I2CLink.token);
    Serial.println("[portmap] " + String(portmap_number) + String(" [port state] ") + String(matrix_switch_state[0][portmap_number]));
  }

  // ------------------------------------------------------------
  // Matrix Indicators
  // ------------------------------------------------------------
  else if (strcmp(I2CLink.token, "$I")==0) {
    I2CLink.token = strtok(NULL, ",");
    indicator_number = atoi(I2CLink.token);
    I2CLink.token = strtok(NULL, ",");
    matrix_indicator_colors[0][indicator_number] = atoi(I2CLink.token);
    leds[indicator_number] = available_matrix_indicator_colors[matrix_indicator_colors[0][indicator_number]];
    Serial.println("[indicator] led index: " + String(indicator_number) + " color index: " + String(matrix_indicator_colors[0][indicator_number]));
  }
}

void requestEvent() {
  // ------------------------------------------------------------
  // Write Bytes of Chars
  // ------------------------------------------------------------
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i] = (byte)I2CLink.TMP_BUFFER[i];}
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                    PORT CONTROLLER

void satIOPortController() {
  // ------------------------------------------------------------
  // Write switch state to pins specified in port map
  // ------------------------------------------------------------
  for (int i=0; i<20; i++) {
    if (update_portmap_bool==true) {
      if (matrix_port_map[0][i] != tmp_matrix_port_map[0][i]) {

        // ------------------------------------------------------------
        // set a depricated port mapping to default values
        // ------------------------------------------------------------
        digitalWrite(matrix_port_map[0][i], LOW);
        pinMode(matrix_port_map[0][i], INPUT);

        Serial.println("[portmap] updating port: " + String(matrix_port_map[0][i]) + " -> " + String(tmp_matrix_port_map[0][i]));

        // ------------------------------------------------------------
        // setup new port
        // ------------------------------------------------------------
        matrix_port_map[0][i]=tmp_matrix_port_map[0][i];
        pinMode(matrix_port_map[0][i], OUTPUT);
      }
    }

    // ------------------------------------------------------------
    // set port high/low
    // ------------------------------------------------------------
    digitalWrite(matrix_port_map[0][i], matrix_switch_state[0][i]);

    // ------------------------------------------------------------
    // set matrix indicator
    // ------------------------------------------------------------
    if (matrix_switch_state[0][i]==1) {leds[i] = available_matrix_indicator_colors[matrix_indicator_colors[0][i]]; FastLED.show();}
    else {leds[i] = CRGB::Black; FastLED.show();}

    // ------------------------------------------------------------
    // uncomment to debug
    // ------------------------------------------------------------
    // Serial.println("[" + String(matrix_port_map[0][i]) + "] " + String(digitalRead(matrix_port_map[0][i])));
  }

  // ------------------------------------------------------------
  // Indicate matrix enabled (matrix data is being received)
  // ------------------------------------------------------------
  if (matrix_io_enabled==true) {leds[20] = CRGB::Red; FastLED.show();}
  else {leds[20] = CRGB::Black; FastLED.show();}

  // ------------------------------------------------------------
  // Indicate data received
  // ------------------------------------------------------------
  if (data_received==true) {leds[21] = CRGB::Red; FastLED.show();}
  else {leds[21] = CRGB::Black; FastLED.show();}

  // ------------------------------------------------------------
  // Indicate overload
  // ------------------------------------------------------------
  if (overload==true) {leds[22] = CRGB::Yellow; FastLED.show();}
  else {leds[22] = CRGB::Black; FastLED.show();}

  // ------------------------------------------------------------
  // Indicate GPS signal
  // ------------------------------------------------------------
  if      (gps_signal==0) {leds[23] = CRGB::Red; FastLED.show();}
  else if (gps_signal==1) {leds[23] = CRGB::Green; FastLED.show();}
  else if (gps_signal==2) {leds[23] = CRGB::Blue; FastLED.show();}
  else                    {leds[23] = CRGB::Red; FastLED.show();}
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP 

void setup() {

  // ------------------------------------------------------------
  // Serial
  // ------------------------------------------------------------
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);

  // ------------------------------------------------------------
  // Matrix indicators
  // ------------------------------------------------------------
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // ------------------------------------------------------------
  // Matrix switches
  // ------------------------------------------------------------
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }

  // ------------------------------------------------------------
  // Initialize I2C communications as Slave
  // ------------------------------------------------------------
  Wire.begin(SLAVE_ADDR);

  // ------------------------------------------------------------
  // Function to run when data requested from master
  // ------------------------------------------------------------
  Wire.onRequest(requestEvent);

  // ------------------------------------------------------------
  // Function to run when data received from master
  // ------------------------------------------------------------
  Wire.onReceive(receiveEvent);

  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {
  // ------------------------------------------------------------
  // uncomment to debug
  // ------------------------------------------------------------
  // Serial.println("---------------------------------------");
  // Serial.println("[loop] ");
  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time

  // ------------------------------------------------------------
  // reset data indicator
  // ------------------------------------------------------------
  data_received=false;
  
  // ------------------------------------------------------------
  // run port controller function
  // ------------------------------------------------------------
  satIOPortController();
  
  delay(1);

  // ------------------------------------------------------------
  // uncomment to debug
  // ------------------------------------------------------------
  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);
}
