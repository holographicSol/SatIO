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
ATMEGA2560 48 -> 20x individually addressable WS2812B's

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

// MATRIX INDICATORS
#define NUM_LEDS 20
#define DATA_PIN 48
CRGB leds[NUM_LEDS];

// SIGNAL
#define LEDSATSIGNALR 51
#define LEDSATSIGNALG 52
#define LEDSATSIGNALB 53

// OVERLOAD
#define LEDOVERLOADR 46
#define LEDOVERLOADG 47

// ------------------------------------------------------------------------------------------------------------------------------

#define ETX 0x03  // end of text character

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MATRIX DATA

signed int portmap_number;
bool update_portmap_bool = false;

// reflects matrix switch active/inactive states each loop of matrix switch function
int max_matrix_switch_states = 20;
bool matrix_switch_state[1][20] = {
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  }
};

// a placeholder for matrix switch ports
signed int matrix_port_map[1][20] = {
  {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  }
};

// a placeholder for matrix switch ports
signed int tmp_matrix_port_map[1][20] = {
  {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  }
};

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    TIME STRUCT

struct TimeStruct {
  double mainLoopTimeTaken;            // current main loop time
  unsigned long mainLoopTimeStart;     // time recorded at the start of each iteration of main loop
};
TimeStruct timeData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      I2C DATA

// Define Slave I2C Address
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

  // Serial.println("[slave] read data");
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  // Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));

  // get tag token
  I2CLink.token = strtok(I2CLink.INPUT_BUFFER, ",");

  // Port Map
  if (strcmp(I2CLink.token, "$P")==0) {
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[received] portmap: " + String(I2CLink.token));
    portmap_number = atoi(I2CLink.token);
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[received] portnumber: " + String(I2CLink.token));
    matrix_port_map[0][portmap_number] = atoi(I2CLink.token);
    Serial.println("[portmap] " + String(portmap_number) + String(" [port number] ") + String(matrix_port_map[0][portmap_number]));
  }

  // Port States
  if (strcmp(I2CLink.token, "$M")==0) {
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[received] portmap: " + String(I2CLink.token));
    portmap_number = atoi(I2CLink.token);
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[received] portnumber: " + String(I2CLink.token));
    matrix_switch_state[0][portmap_number] = atoi(I2CLink.token);
    Serial.println("[portmap] " + String(portmap_number) + String(" [port state] ") + String(matrix_switch_state[0][portmap_number]));
  }

  // Satellite Count and HDOP Precision Factor Indicator
  if (strcmp(I2CLink.token, "$GPSSIG")==0) {
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[received] Satellite Count and HDOP Precision Factor: " + String(I2CLink.token));
    if (strcmp(I2CLink.token, "0") == 0)
      // red
      {
        digitalWrite(LEDSATSIGNALG, LOW);
        digitalWrite(LEDSATSIGNALB, LOW);
        digitalWrite(LEDSATSIGNALR, LOW);
        digitalWrite(LEDSATSIGNALR, HIGH);
      }
      // green
      else if (strcmp(I2CLink.token, "1") == 0)
      {
        digitalWrite(LEDSATSIGNALR, LOW);
        digitalWrite(LEDSATSIGNALB, LOW);
        digitalWrite(LEDSATSIGNALG, LOW);   
        digitalWrite(LEDSATSIGNALG, HIGH);
      }
      // blue
      else if (strcmp(I2CLink.token, "2") == 0)
      {
        digitalWrite(LEDSATSIGNALR, LOW);
        digitalWrite(LEDSATSIGNALG, LOW);
        digitalWrite(LEDSATSIGNALB, LOW);
        digitalWrite(LEDSATSIGNALB, HIGH);
      }
  }

  // Overload Indicator
  if (strcmp(I2CLink.token, "$OLOAD")==0) {
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[received] Overload Indicator: " + String(I2CLink.token));
    if (strcmp(I2CLink.token, "0") == 0) {digitalWrite(LEDOVERLOADR, LOW); digitalWrite(LEDOVERLOADG, LOW);}
    else if (strcmp(I2CLink.token, "1") == 0) {digitalWrite(LEDOVERLOADR, HIGH); digitalWrite(LEDOVERLOADG, HIGH);}
  }
}

void requestEvent() {
  // Write Bytes of Chars
  // Serial.println("[slave] write data");
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (int i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i] = (byte)I2CLink.TMP_BUFFER[i];}
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                    PORT CONTROLLER

void satIOPortController() {
  // Serial.println("[satIOPortController] ");

  // write switch state to pins specified in port map
  for (int i=0; i<20; i++) {
    if (update_portmap_bool==true) {
      if (matrix_port_map[0][i] != tmp_matrix_port_map[0][i]) {

        // set a depricated port mapping to default values
        digitalWrite(matrix_port_map[0][i], LOW);
        pinMode(matrix_port_map[0][i], INPUT);

        Serial.println("[portmap] updating port: " + String(matrix_port_map[0][i]) + " -> " + String(tmp_matrix_port_map[0][i]));

        // setup new port
        matrix_port_map[0][i]=tmp_matrix_port_map[0][i];
        pinMode(matrix_port_map[0][i], OUTPUT);
      }
    }

    // set port high/low
    digitalWrite(matrix_port_map[0][i], matrix_switch_state[0][i]);

    // set matrix indicator
    if (matrix_switch_state[0][i]==1) {leds[i] = CRGB::Yellow; FastLED.show();}
    else {leds[i] = CRGB::Black; FastLED.show();}

    // uncomment to debug
    // Serial.println("[" + String(matrix_port_map[0][i]) + "] " + String(digitalRead(matrix_port_map[0][i])));
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP 

void setup() {

  // Serial
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);

  // Matrix indicators
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // Matrix switches
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }

  // Satellite signal indicator
  pinMode(LEDSATSIGNALR, OUTPUT);
  pinMode(LEDSATSIGNALG, OUTPUT);
  pinMode(LEDSATSIGNALB, OUTPUT);
  digitalWrite(LEDSATSIGNALR, LOW);
  digitalWrite(LEDSATSIGNALG, LOW);
  digitalWrite(LEDSATSIGNALB, LOW);

  // Overload indicator
  pinMode(LEDOVERLOADR, OUTPUT);
  pinMode(LEDOVERLOADG, OUTPUT);
  digitalWrite(LEDOVERLOADR, LOW);
  digitalWrite(LEDOVERLOADG, LOW);

  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);

  // Function to run when data received from master
  Wire.onReceive(receiveEvent);

  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {

  // Serial.println("---------------------------------------");
  
  // Serial.println("[loop] ");

  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time
  
  satIOPortController();

  delay(1);
  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);
}
