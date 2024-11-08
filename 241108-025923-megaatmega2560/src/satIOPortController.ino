/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

ATMEGA2560 RX1 (19) -> Serial1
ATMEGA2560 TX1 (18) -> Serial1

*/

#include <stdio.h>
#include <string.h>
// #include <iostream>
#include <Arduino.h>

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MATRIX DATA

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
    23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
    33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
  }
};

// ------------------------------------------------------------------------------------------------------------------------------

#define ETX 0x03  // end of text character

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             SERIAL LINK STRUCT

struct SerialLinkStruct {
  unsigned long nbytes;
  char BUFFER[1024];            // read incoming bytes into this buffer
  char DATA[1024];              // buffer refined using ETX
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
};
SerialLinkStruct SerialLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           VALIDATION: CHECKSUM

int getCheckSum(char * string) {
  /* creates a checksum for an NMEA style sentence. can be used to create checksum to append or compare */
  if (SerialLink.validation == true) {Serial.println("[connected] getCheckSum: " + String(string));}
  int i;
  int XOR;
  int c;
  for (XOR = 0, i = 0; i < strlen(string); i++) {
    c = (unsigned char)string[i];
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  Serial.println("[connected] getCheckSum: " + String(XOR));
  return XOR;
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
  if (SerialLink.validation == true) {Serial.println("[connected] validateChecksum: " + String(buffer));}
  char gotSum[2];
  gotSum[0] = buffer[strlen(buffer) - 3];
  gotSum[1] = buffer[strlen(buffer) - 2];
  uint8_t checksum_of_buffer =  getCheckSum(buffer);
  uint8_t checksum_in_buffer = h2d2(gotSum[0], gotSum[1]);
  if (checksum_of_buffer == checksum_in_buffer) {return true;} else {return false;}
}

void createChecksum(char * buffer) {
  uint8_t checksum_of_buffer = getCheckSum(buffer);
  // Serial.print("checksum_of_buffer: ");
  // Serial.println(checksum_of_buffer);
  // Serial.printf("Hexadecimal number is: %X", checksum_of_buffer); 
  // Serial.println();
  sprintf(SerialLink.checksum,"%X",checksum_of_buffer);
  // Serial.print("checksum: ");
  // Serial.println(checksum);
  // Serial.println();
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP 

void setup(void) {
  Serial.begin(115200); while(!Serial);
  Serial1.begin(115200); while(!Serial1);
  Serial1.setTimeout(10);
  Serial.flush();
  Serial1.flush();

  // setup IO
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }
  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                 READ RXD: METHOD 0

bool readRXD1_Method00() {
  if (Serial1.available() > 0) {
    memset(SerialLink.BUFFER, 0, 1024);
    memset(SerialLink.DATA, 0, 1024);
    int rlen = Serial1.readBytes(SerialLink.BUFFER, 1024);
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
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                        PROCESS RXD

void readRXD1_Method0() {
  SerialLink.T0_RXD_1 = millis();
  if (SerialLink.T0_RXD_1 >= SerialLink.T1_RXD_1+SerialLink.TT_RXD_1) {
    SerialLink.T1_RXD_1 = SerialLink.T0_RXD_1;
    if (readRXD1_Method00() == true) {
      Serial.println("-------------------------------------------");

      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      strcpy(SerialLink.BUFFER, SerialLink.DATA);

      // uncomment to debug
      Serial.print("[RXD]       "); Serial.println(SerialLink.DATA);
      
      // tag specific processing (like nmea sentences, if we know the tag then we should know what elements are where)
      SerialLink.token = strtok(SerialLink.DATA, ",");
      if (strcmp(SerialLink.token, "$MATRX") == 0) {

        // initiate counter; compare expected element to actual RXD TOKEN; count negative comparison; for 1 million iterations
        SerialLink.validation = false;
        SerialLink.i_token = 0;
        SerialLink.token = strtok(NULL, ",");
        while(SerialLink.token != NULL) {

          // uncomment to debug
          // Serial.print("[" + String(matrix_port_map[0][SerialLink.i_token]) + "] [RXD TOKEN] "); Serial.println(SerialLink.token);

          // check eack token for exactly 1 or 0
          if (SerialLink.i_token == 0)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][0] = 0;}}
          if (SerialLink.i_token == 0)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][0] = 1;}}
          if (SerialLink.i_token == 1)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][1] = 0;}}
          if (SerialLink.i_token == 1)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][1] = 1;}}
          if (SerialLink.i_token == 2)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][2] = 0;}}
          if (SerialLink.i_token == 2)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][2] = 1;}}
          if (SerialLink.i_token == 3)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][3] = 0;}}
          if (SerialLink.i_token == 3)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][3] = 1;}}
          if (SerialLink.i_token == 4)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][4] = 0;}}
          if (SerialLink.i_token == 4)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][4] = 1;}}
          if (SerialLink.i_token == 5)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][5] = 0;}}
          if (SerialLink.i_token == 5)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][5] = 1;}}
          if (SerialLink.i_token == 6)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][6] = 0;}}
          if (SerialLink.i_token == 6)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][6] = 1;}}
          if (SerialLink.i_token == 7)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][7] = 0;}}
          if (SerialLink.i_token == 7)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][7] = 1;}}
          if (SerialLink.i_token == 8)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][8] = 0;}}
          if (SerialLink.i_token == 8)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][8] = 1;}}
          if (SerialLink.i_token == 9)   {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][9] = 0;}}
          if (SerialLink.i_token == 9)   {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][9] = 1;}}
          if (SerialLink.i_token == 10)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][10] = 0;}}
          if (SerialLink.i_token == 10)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][10] = 1;}}
          if (SerialLink.i_token == 11)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][11] = 0;}}
          if (SerialLink.i_token == 11)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][11] = 1;}}
          if (SerialLink.i_token == 12)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][12] = 0;}}
          if (SerialLink.i_token == 12)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][12] = 1;}}
          if (SerialLink.i_token == 13)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][13] = 0;}}
          if (SerialLink.i_token == 13)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][13] = 1;}}
          if (SerialLink.i_token == 14)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][14] = 0;}}
          if (SerialLink.i_token == 14)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][14] = 1;}}
          if (SerialLink.i_token == 15)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][15] = 0;}}
          if (SerialLink.i_token == 15)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][15] = 1;}}
          if (SerialLink.i_token == 16)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][16] = 0;}}
          if (SerialLink.i_token == 16)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][16] = 1;}}
          if (SerialLink.i_token == 17)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][17] = 0;}}
          if (SerialLink.i_token == 17)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][17] = 1;}}
          if (SerialLink.i_token == 18)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][18] = 0;}}
          if (SerialLink.i_token == 18)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][18] = 1;}}
          if (SerialLink.i_token == 19)  {if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][19] = 0;}}
          if (SerialLink.i_token == 19)  {if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][19] = 1;}}

          // handle expected checksum
          if (SerialLink.i_token == 20)  {
            SerialLink.validation = validateChecksum(SerialLink.BUFFER);
            break;
          }
          
          // iterate counters and snap off used token
          SerialLink.i_token++;
          SerialLink.token = strtok(NULL, ",");
        }
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                    PORT CONTROLLER

/* please ensure checksum was validated properly before actually implementing this final function in the feild */

void satIOPortController() {

  for (int i=0; i<20; i++) {
    digitalWrite(matrix_port_map[0][i], matrix_switch_state[0][i]);

    // uncomment to debug
    // Serial.println("[" + String(matrix_port_map[0][i]) + "] " + String(digitalRead(matrix_port_map[0][i])));
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {
    readRXD1_Method0();
    if (SerialLink.validation == true) {satIOPortController();}
}
