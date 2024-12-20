/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

Required wiring:

  ESP32 io27 (TXD) -> ATMEGA2560 Serial1 (RXD)

  ESP32 io22 (RXD) -> ATMEGA2560 Serial1 (TXD)

  WTGPS300P (TXD)  -> ATMEGA2560 Serial3 (RXD)

*/

#include <stdio.h>
#include <string.h>
// #include <iostream>
#include <Arduino.h>

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MATRIX DATA

signed int tmp_port;
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

#define ETX 0x03  // end of text character

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             SERIAL LINK STRUCT

struct SerialLinkStruct {
  unsigned long nbytes;
  char BUFFER[2000];            // read incoming bytes into this buffer
  char BUFFER1[2000];           // store bytes when they are different from previous read bytes
  char DATA[2000];              // buffer refined using ETX
  unsigned long T0_RXD_1 = 0;   // hard throttle current time
  unsigned long T1_RXD_1 = 0;   // hard throttle previous time
  unsigned long TT_RXD_1 = 0;   // hard throttle interval
  unsigned long T0_TXD_1 = 0;   // hard throttle current time
  unsigned long T1_TXD_1 = 0;   // hard throttle previous time
  unsigned long TT_TXD_1 = 10;  // hard throttle interval
  int i_token = 0;
  char * token;
  bool validation = false;
  char checksum[56];
  uint8_t checksum_of_buffer;
  uint8_t checksum_in_buffer;
  char gotSum[2];
  int i_XOR;
  int XOR;
  int c_XOR;
};
SerialLinkStruct SerialLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           VALIDATION: CHECKSUM

int getCheckSum(char * string) {
  /* creates a checksum for an NMEA style sentence. can be used to create checksum to append or compare */

  // uncomment to debug
  // if (SerialLink.validation == true) {Serial.println("[connected] getCheckSum: " + String(string));}
  for (SerialLink.XOR = 0, SerialLink.i_XOR = 0; SerialLink.i_XOR < strlen(string); SerialLink.i_XOR++) {
    SerialLink.c_XOR = (unsigned char)string[SerialLink.i_XOR];
    if (SerialLink.c_XOR == '*') break;
    if (SerialLink.c_XOR != '$') SerialLink.XOR ^= SerialLink.c_XOR;
  }
  // uncomment to debug
  // Serial.println("[connected] getCheckSum: " + String(SerialLink.XOR));
  return SerialLink.XOR;
}

// takes a character representing a hexadecimal digit and returns the decimal equivalent of that digit.
uint8_t h2d(char hex) {if(hex > 0x39) hex -= 7; return(hex & 0xf);}

/*
converts each digit it to its decimal equivalent, shifts first digit left by 4 bits and 'ORing' with the second digit.
The result is a single byte value representing two hexadecimal digits combined.
*/
uint8_t h2d2(char h1, char h2) {return (h2d(h1)<<4) | h2d(h2);}

bool validateChecksum(char * buffer) {
  /* validate a sentence appended with a checksum */

  // uncomment to debug
  // if (SerialLink.validation == true) {Serial.println("[connected] validateChecksum: " + String(buffer));}
  SerialLink.gotSum[2];
  SerialLink.gotSum[0] = buffer[strlen(buffer) - 3];
  SerialLink.gotSum[1] = buffer[strlen(buffer) - 2];
  SerialLink.checksum_of_buffer =  getCheckSum(buffer);
  SerialLink.checksum_in_buffer = h2d2(SerialLink.gotSum[0], SerialLink.gotSum[1]);
  if (SerialLink.checksum_of_buffer == SerialLink.checksum_in_buffer) {return true;} else {return false;}
}

void createChecksum(char * buffer) {
  SerialLink.checksum_of_buffer = getCheckSum(buffer);

  // uncomment to debug
  // Serial.print("checksum_of_buffer: "); Serial.println(checksum_of_buffer);
  // Serial.printf("Hexadecimal number is: %X", checksum_of_buffer); Serial.println();

  sprintf(SerialLink.checksum,"%X",SerialLink.checksum_of_buffer);

  // uncomment to debug
  // Serial.print("checksum: "); Serial.println(checksum); Serial.println();
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP 

void setup() {
  Serial.begin(115200);  while(!Serial);
  Serial1.begin(115200); while(!Serial1);
  Serial3.begin(115200); while(!Serial3);
  Serial1.setTimeout(10);
  Serial3.setTimeout(10);
  Serial.flush();
  Serial1.flush();
  Serial3.flush();

  // setup IO
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }
  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                 READ RXD: METHOD 0

bool readRXD1UntilETX() {
  memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
  memset(SerialLink.DATA, 0, sizeof(SerialLink.DATA));
  if (Serial1.available() > 0) {
    int rlen = Serial1.readBytes(SerialLink.BUFFER, sizeof(SerialLink.BUFFER));
    if (rlen > 80) { 
      if (rlen != 0) {
        for(int i = 0; i < rlen; i++) {
          if (SerialLink.BUFFER[i] == ETX)
            break;
          else {
            SerialLink.DATA[i] = SerialLink.BUFFER[i];
          }
        }
        return true;
      }
      else {return false;}
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                        PROCESS RXD

void readRXD1_Method0() {
  SerialLink.validation = false;
  SerialLink.T0_RXD_1 = millis();
  if (SerialLink.T0_RXD_1 >= SerialLink.T1_RXD_1+SerialLink.TT_RXD_1) {
    SerialLink.T1_RXD_1 = SerialLink.T0_RXD_1;
    if (readRXD1UntilETX() == true) {

      // Serial.println("-------------------------------------------");
      
      // store a second copy of buffer that does not contain ETX.
      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      strcpy(SerialLink.BUFFER, SerialLink.DATA);

      // uncomment to debug
      // Serial.print("[RXD]       "); Serial.println(SerialLink.DATA);
      
      // break off and compare first token
      SerialLink.token = strtok(SerialLink.DATA, ",");
      if (strcmp(SerialLink.token, "$MATRX") == 0) {

        // igonore a switch message if its the same as previous switch message
        if (!strcmp(SerialLink.BUFFER, SerialLink.BUFFER1)==0) {
          memset(SerialLink.BUFFER1, 0, sizeof(SerialLink.BUFFER1));
          strcpy(SerialLink.BUFFER1, SerialLink.BUFFER);

          // reset values
          update_portmap_bool=false;
          SerialLink.validation = false;
          SerialLink.i_token = 0;
          SerialLink.token = strtok(NULL, ",");
          while(SerialLink.token != NULL) {

            // uncomment to debug
            // Serial.print("[" + String(matrix_port_map[0][SerialLink.i_token]) + "] [RXD TOKEN] "); Serial.println(SerialLink.token);

            // check eack token for portmap
            if (SerialLink.i_token<20) { 
              for (int i=0; i<20; i++) {
                tmp_matrix_port_map[0][i] = atoi(SerialLink.token);
                if (atoi(SerialLink.token) != matrix_port_map[0][i]) {update_portmap_bool=true;}
                // uncomment to debug
                // Serial.println(String(i) + " [portmap] " + String(matrix_port_map[0][i]));
                
                SerialLink.i_token++;
                SerialLink.token = strtok(NULL, ",");
              }
            }

            // check eack token for exactly 1 or 0 for matrix switch state
            if ((SerialLink.i_token>=20) && (SerialLink.i_token<40)) { 
              for (int i=0; i<20; i++) {
                if      (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][i] = 0;}
                else if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][i] = 1;}
                SerialLink.i_token++;
                SerialLink.token = strtok(NULL, ",");
              }
            }

            // handle expected checksum
            if (SerialLink.i_token == 40)  {
              SerialLink.validation = validateChecksum(SerialLink.BUFFER);
              // try to get another read. this may be written differently later but currently this is the primary objective.
              // if (SerialLink.validation==false) {readRXD1_Method0();}
              break;
            }
            // iterate counters and snap off used token
            SerialLink.i_token++;
            SerialLink.token = strtok(NULL, ",");
          }
        }
        // else {Serial.println("[skipping]   " + String(SerialLink.BUFFER));}
      }
      // else {readRXD1_Method0();}
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                    PORT CONTROLLER

/* please ensure checksum was validated properly before actually implementing this final function in the feild */

void satIOPortController() {

  // Serial.println("[processing] " + String(SerialLink.BUFFER));

  // make ports high or low according to validated data
  for (int i=0; i<20; i++) {

    // handle current port configuration
    if (update_portmap_bool==true) {
      if (matrix_port_map[0][i] != tmp_matrix_port_map[0][i]) {
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

    // uncomment to debug
    // Serial.println("[" + String(matrix_port_map[0][i]) + "] " + String(digitalRead(matrix_port_map[0][i])));
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  READ GPS DATA

void readGPS() {
  if (Serial3.available() > 0) {
    // loop to write gps serial to SatIO serial about 5 times because we are looking for GNGGA, GNRMC, GPATT and ignoring DESBI.
    for (int i=0; i <5; i++) {
      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      // After migrating to CYD, lets see if the Serial1 set pins will be be stable.
      SerialLink.nbytes = (Serial3.readBytesUntil('\n', SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));
      // Serial.print(SerialLink.nbytes); Serial.print(" "); Serial.println(SerialLink.BUFFER); // debug
      if (!strncmp(SerialLink.BUFFER, "$DESBI", 6) == 0) {
        Serial1.write(SerialLink.BUFFER);
        Serial1.write(ETX);
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {
  readGPS();
  readRXD1_Method0();
  if (SerialLink.validation == true) {satIOPortController();}
}
