/*
Written by Benjamin Jack Cullen.

SatIOPortController - Receives messages over IIC and manipulates ports accordingly.
                      This file should be flashed to ATMEGA2560.
                      Indicator LEDs for panels/switches/etc configurable over IIC.

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

Wiring Matrix LED Indicators:
ATMEGA2560 48 -> 24x individually addressable WS2812B's

*/

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>
// #include <FastLED.h> // (uncomment to enable leds)

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     INDICATORS
// ------------------------------------------------------------------------------------------------------------------------------

/*
Indicators:
0-19: Matrix Switch State (high/low).
20:   IIC switching enabled/disabled.
21:   Data.
22:   Overload.
23:   Signal.
*/

// INDICATORS (uncomment to enable leds)
// #define NUM_LEDS 24
// #define DATA_PIN 48
// CRGB leds[NUM_LEDS];

// ------------------------------------------------------------
// Matrix Indicator Colors (uncomment to enable leds)
// ------------------------------------------------------------
// int indicator_number=0;
// long matrix_indicator_colors[1][20] = {
//   {
//     1,1,1,1,1,1,1,1,1,1,
//     1,1,1,1,1,1,1,1,1,1
//   }
// };
// long available_matrix_indicator_colors[8] = {
//   CRGB::Black,
//   CRGB::Red,
//   CRGB::Yellow,
//   CRGB::Green,
//   CRGB::Blue,
//   CRGB::Cyan,
//   CRGB::Purple,
//   CRGB::White,
// };

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MATRIX DATA
// ------------------------------------------------------------------------------------------------------------------------------

signed int portmap_number;
bool matrix_io_enabled=false;
bool update_portmap_bool = false;

// ------------------------------------------------------------
// (uncomment to enable leds)
// ------------------------------------------------------------
// int gps_signal=0;
// bool overload=false;
// bool data_received=false;

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

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    TIME STRUCT
// ------------------------------------------------------------------------------------------------------------------------------

struct TimeStruct {
  double mainLoopTimeTaken; // current main loop time
  unsigned long mainLoopTimeStart; // time recorded at the start of each iteration of main loop
};
TimeStruct timeData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA
// ------------------------------------------------------------------------------------------------------------------------------

#define SLAVE_ADDR 9

struct I2CLinkStruct {
  int i_token;
  char * token;
  byte OUTPUT_BUFFER[32];
  char INPUT_BUFFER[32];
  char TMP_BUFFER[32];
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  DATA: SENSORS
// ------------------------------------------------------------------------------------------------------------------------------

struct SensorDataStruct {
  // ----------------------------------------------------
  // Analog Sticks
  // ----------------------------------------------------
  int as_0_u=0;
  int as_0_d=0;
  int as_0_l=0;
  int as_0_r=0;
  int as_0_c=1;
  int as_1_u=0;
  int as_1_d=0;
  int as_1_l=0;
  int as_1_r=0;
  int as_1_c=1;
};
SensorDataStruct sensorData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     I2C EVENTS
// ------------------------------------------------------------------------------------------------------------------------------

bool receive_event=false;

void receiveEvent(int) {
  receive_event=true;

  // ------------------------------------------------------------
  // Indicate data received (uncomment to enable leds)
  // ------------------------------------------------------------
  // data_received=true;
  
  // ------------------------------------------------------------
  // Read incoming data
  // ------------------------------------------------------------
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  // Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));

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
  // GPS Signal (uncomment to enable leds)
  // ------------------------------------------------------------
  // else if (strcmp(I2CLink.token, "$GPSSIG")==0) {
  //   I2CLink.token = strtok(NULL, ",");
  //   if      (strcmp(I2CLink.token, "0") == 0) {gps_signal=0;}
  //   else if (strcmp(I2CLink.token, "1") == 0) {gps_signal=1;}
  //   else if (strcmp(I2CLink.token, "2") == 0) {gps_signal=2;}
  //   else                                      {gps_signal=3;}
  // }

  // ------------------------------------------------------------ 
  // Overload (uncomment to enable leds)
  // ------------------------------------------------------------
  // else if (strcmp(I2CLink.token, "$OLOAD")==0) {
  //   I2CLink.token = strtok(NULL, ",");
  //   if      (strcmp(I2CLink.token, "0") == 0) {overload=false;}
  //   else if (strcmp(I2CLink.token, "1") == 0) {overload=true;}
  // }

  // ------------------------------------------------------------
  // Port Map
  // ------------------------------------------------------------
  else if (strcmp(I2CLink.token, "$P")==0) {
    I2CLink.token = strtok(NULL, ",");
    portmap_number = atoi(I2CLink.token);
    I2CLink.token = strtok(NULL, ",");
    matrix_port_map[0][portmap_number] = atoi(I2CLink.token);
    // Serial.println("[portmap] " + String(portmap_number) + String(" [port number] ") + String(matrix_port_map[0][portmap_number]));
  }

  // ------------------------------------------------------------
  // Port States
  // ------------------------------------------------------------
  else if (strcmp(I2CLink.token, "$M")==0) {
    I2CLink.token = strtok(NULL, ",");
    portmap_number = atoi(I2CLink.token);
    I2CLink.token = strtok(NULL, ",");
    matrix_switch_state[0][portmap_number] = atoi(I2CLink.token);
    // Serial.println("[portmap] " + String(portmap_number) + String(" [port state] ") + String(matrix_switch_state[0][portmap_number]));
  }

  // ------------------------------------------------------------
  // Matrix Indicators (uncomment to enable leds)
  // ------------------------------------------------------------
  // else if (strcmp(I2CLink.token, "$I")==0) {
  //   I2CLink.token = strtok(NULL, ",");
  //   indicator_number = atoi(I2CLink.token);
  //   I2CLink.token = strtok(NULL, ",");
  //   matrix_indicator_colors[0][indicator_number] = atoi(I2CLink.token);
  //   leds[indicator_number] = available_matrix_indicator_colors[matrix_indicator_colors[0][indicator_number]];
  //   Serial.println("[indicator] led index: " + String(indicator_number) + " color index: " + String(matrix_indicator_colors[0][indicator_number]));
  // }

  else if (strcmp(I2CLink.token, "$JOY")==0) {
    I2CLink.token=strtok(NULL, ",");
    I2CLink.i_token=0;
    while (I2CLink.token != NULL) {
      // Serial.println("[token " + String(I2CLink.i_token) + "] " + String(I2CLink.token));
      if (I2CLink.i_token==0) {sensorData.as_0_u=atoi(I2CLink.token);}
      else if (I2CLink.i_token==1) {sensorData.as_0_d=atoi(I2CLink.token);}
      else if (I2CLink.i_token==2) {sensorData.as_0_l=atoi(I2CLink.token);}
      else if (I2CLink.i_token==3) {sensorData.as_0_r=atoi(I2CLink.token);}
      else if (I2CLink.i_token==4) {sensorData.as_0_c=atoi(I2CLink.token);}
      else if (I2CLink.i_token==5) {sensorData.as_1_u=atoi(I2CLink.token);}
      else if (I2CLink.i_token==6) {sensorData.as_1_d=atoi(I2CLink.token);}
      else if (I2CLink.i_token==7) {sensorData.as_1_l=atoi(I2CLink.token);}
      else if (I2CLink.i_token==8) {sensorData.as_1_r=atoi(I2CLink.token);}
      else if (I2CLink.i_token==9) {sensorData.as_1_c=atoi(I2CLink.token);}
      I2CLink.token=strtok(NULL, ",");
      I2CLink.i_token++;
    }
    // ---------------------------------------------------------------------------------------------------------------
    // Joy Sticks: Currently mapped to 1024 for sensors but can be left unmapped if outputting to MCU on the same pin.
    // ---------------------------------------------------------------------------------------------------------------
    analogWrite(A0, map(sensorData.as_0_u, 0, 50, 0, 1024));
    analogWrite(A1, map(sensorData.as_0_d, 0, 50, 0, 1024));
    analogWrite(A2, map(sensorData.as_0_l, 0, 50, 0, 1024));
    analogWrite(A3, map(sensorData.as_0_r, 0, 50, 0, 1024));
    digitalWrite(52, sensorData.as_0_c);
    analogWrite(A4, map(sensorData.as_0_u, 0, 50, 0, 1024));
    analogWrite(A5, map(sensorData.as_0_d, 0, 50, 0, 1024));
    analogWrite(A6, map(sensorData.as_0_l, 0, 50, 0, 1024));
    analogWrite(A7, map(sensorData.as_0_r, 0, 50, 0, 1024));
    digitalWrite(53, sensorData.as_1_c);
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
// ------------------------------------------------------------------------------------------------------------------

void satIOPortController() {
  // ------------------------------------------------------------
  // Write switch state to pins specified in port map
  // ------------------------------------------------------------
  for (int i=0; i<20; i++) {
    // if (update_portmap_bool==true) {
    //   if (matrix_port_map[0][i] != tmp_matrix_port_map[0][i]) {

    //     // ------------------------------------------------------------
    //     // set a depricated port mapping to default values
    //     // ------------------------------------------------------------
    //     digitalWrite(matrix_port_map[0][i], LOW);
    //     // pinMode(matrix_port_map[0][i], INPUT);

    //     // Serial.println("[portmap] updating port: " + String(matrix_port_map[0][i]) + " -> " + String(tmp_matrix_port_map[0][i]));

    //     // ------------------------------------------------------------
    //     // setup new port
    //     // ------------------------------------------------------------
    //     matrix_port_map[0][i]=tmp_matrix_port_map[0][i];
    //     // pinMode(matrix_port_map[0][i], OUTPUT);
    //   }
    // }

    // ------------------------------------------------------------
    // set port high/low
    // ------------------------------------------------------------
    digitalWrite(matrix_port_map[0][i], matrix_switch_state[0][i]);


    // ------------------------------------------------------------
    // (uncomment to enable leds)
    // ------------------------------------------------------------
    // if (matrix_switch_state[0][i]==1) {leds[i] = available_matrix_indicator_colors[matrix_indicator_colors[0][i]]; FastLED.show();}
    // else {leds[i] = CRGB::Black; FastLED.show();}


    // ------------------------------------------------------------
    // uncomment to debug
    // ------------------------------------------------------------
    // Serial.println("[" + String(matrix_port_map[0][i]) + "] " + String(digitalRead(matrix_port_map[0][i])));
  }

  // // ------------------------------------------------------------
  // // Indicate matrix enabled (matrix data is being received) (uncomment to enable leds)
  // // ------------------------------------------------------------
  // if (matrix_io_enabled==true) {leds[20] = CRGB::Red; FastLED.show();}
  // else {leds[20] = CRGB::Black; FastLED.show();}

  // // ------------------------------------------------------------
  // // Indicate data received (uncomment to enable leds)
  // // ------------------------------------------------------------
  // if (data_received==true) {leds[21] = CRGB::Red; FastLED.show();}
  // else {leds[21] = CRGB::Black; FastLED.show();}

  // // ------------------------------------------------------------
  // // Indicate overload (uncomment to enable leds)
  // // ------------------------------------------------------------
  // if (overload==true) {leds[22] = CRGB::Yellow; FastLED.show();}
  // else {leds[22] = CRGB::Black; FastLED.show();}

  // // ------------------------------------------------------------
  // // Indicate GPS signal (uncomment to enable leds)
  // // ------------------------------------------------------------
  // if      (gps_signal==0) {leds[23] = CRGB::Red; FastLED.show();}
  // else if (gps_signal==1) {leds[23] = CRGB::Green; FastLED.show();}
  // else if (gps_signal==2) {leds[23] = CRGB::Blue; FastLED.show();}
  // else                    {leds[23] = CRGB::Red; FastLED.show();}

  // ---------------------------------------------------------------------------------------------------------------
  // Joy Sticks: Currently mapped to 1024 for sensors but can be left unmapped if outputting to MCU on the same pin.
  // ---------------------------------------------------------------------------------------------------------------
  // analogWrite(A0, map(sensorData.as_0_u, 0, 50, 0, 1024));
  // analogWrite(A1, map(sensorData.as_0_d, 0, 50, 0, 1024));
  // analogWrite(A2, map(sensorData.as_0_l, 0, 50, 0, 1024));
  // analogWrite(A3, map(sensorData.as_0_r, 0, 50, 0, 1024));
  // digitalWrite(52, sensorData.as_0_c);
  // analogWrite(A4, map(sensorData.as_0_u, 0, 50, 0, 1024));
  // analogWrite(A5, map(sensorData.as_0_d, 0, 50, 0, 1024));
  // analogWrite(A6, map(sensorData.as_0_l, 0, 50, 0, 1024));
  // analogWrite(A7, map(sensorData.as_0_r, 0, 50, 0, 1024));
  // digitalWrite(53, sensorData.as_0_c);
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP
// ------------------------------------------------------------------------------------------------------------------

void setup() {

  // ------------------------------------------------------------
  // Serial
  // ------------------------------------------------------------
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);

  // ------------------------------------------------------------
  // Matrix indicators (uncomment to enable leds)
  // ------------------------------------------------------------
  // FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // ------------------------------------------------------------
  // Matrix switches
  // ------------------------------------------------------------
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }

  // ------------------------------------------------------------
  // Joy Sticks
  // ------------------------------------------------------------
  pinMode(A0, OUTPUT); // joy 0: up
  pinMode(A1, OUTPUT); // joy 0: down
  pinMode(A2, OUTPUT); // joy 0: left
  pinMode(A3, OUTPUT); // joy 0: right
  pinMode(A4, OUTPUT); // joy 1: up
  pinMode(A5, OUTPUT); // joy 1: down
  pinMode(A6, OUTPUT); // joy 1: left
  pinMode(A7, OUTPUT); // joy 1: right
  pinMode(52, OUTPUT); // joy 1: click
  pinMode(53, OUTPUT); // joy 1: click
  analogWrite(A0, 0);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
  analogWrite(A4, 0);
  analogWrite(A5, 0);
  analogWrite(A6, 0);
  analogWrite(A7, 0);
  digitalWrite(52, LOW);
  digitalWrite(53, LOW);

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
//                                                                                                          MAIN LOOP
// ------------------------------------------------------------------------------------------------------------------

void loop() {
  // ------------------------------------------------------------
  // uncomment to debug
  // ------------------------------------------------------------
  // Serial.println("---------------------------------------");
  // Serial.println("[loop] ");
  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time

  // ------------------------------------------------------------
  // reset data indicator (uncomment to enable leds)
  // ------------------------------------------------------------
  // data_received=false;
  
  // ------------------------------------------------------------
  // run port controller function
  // ------------------------------------------------------------
  if (receive_event==true) {satIOPortController(); receive_event=false;}
  
  delay(1);

  // ------------------------------------------------------------
  // uncomment to debug
  // ------------------------------------------------------------
  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);
}
