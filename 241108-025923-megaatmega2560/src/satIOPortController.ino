/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

Required wiring:

ESP32 io27 (TXD) -> ATMEGA2560 Serial1 (RXD)

*/

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <limits.h>
#include <TinyGPSPlus.h>
#include <stdlib.h>
// #include <conio.h>

#define MAX_BUFF 1000

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
  signed int i_nbytes;
  long i_sync;
  char char_i_sync[56];
  bool syn = false;
  bool data = false;
  char BUFFER[MAX_BUFF];            // read incoming bytes into this buffer
  char TMP[MAX_BUFF];               // buffer refined using ETX
  signed int nbytes;
  unsigned long T0_RXD_1 = 0;   // hard throttle current time
  unsigned long T1_RXD_1 = 0;   // hard throttle previous time
  unsigned long TT_RXD_1 = 0;   // hard throttle interval
  unsigned long T0_TXD_1 = 0;   // hard throttle current time
  unsigned long T1_TXD_1 = 0;   // hard throttle previous time
  unsigned long TT_TXD_1 = 10;  // hard throttle interval
  unsigned long TOKEN_i;
  int i_token = 0;
  char * token;
  bool validation = false;
  char checksum[56];
  uint8_t checksum_of_buffer;
  uint8_t checksum_in_buffer;
  char gotSum[4];
  unsigned int i_XOR;
  signed int XOR;
  int c_XOR;
};
SerialLinkStruct SerialLink;

struct TimeStruct {
  double seconds;               // seconds accumulated since startup
  double mainLoopTimeTaken;     // current main loop time
  unsigned long mainLoopTimeStart;     // time recorded at the start of each iteration of main loop
  unsigned long mainLoopTimeTakenMax;  // current record of longest main loop time
  unsigned long mainLoopTimeTakenMin;  // current record of shortest main loop time
  unsigned long t0;                    // micros time 0
  unsigned long t1;                    // micros time 1
  int i_accumukate_time_taken;
  int accumukate_time_taken;
  long main_seconds_0;
};
TimeStruct timeData;

bool gngga_bool = false;
bool gnrmc_bool = false;
bool gpatt_bool = false;
char gngga_sentence[MAX_BUFF];
char gnrmc_sentence[MAX_BUFF];
char gpatt_sentence[MAX_BUFF];
bool gngga_valid_checksum = false;
bool gnrmc_valid_checksum = false;
bool gpatt_valid_checksum = false;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           VALIDATION: CHECKSUM

int getCheckSum(char * string) {
  /* creates a checksum for an NMEA style sentence. can be used to create checksum to append or compare */

  // uncomment to debug
  // if (SerialLink.validation == true) {Serial.println("[connected] getCheckSum: " + String(string));}
  for (SerialLink.XOR = 0, SerialLink.i_XOR = 0; SerialLink.i_XOR < strlen(string); SerialLink.i_XOR++) {
    SerialLink.c_XOR = (unsigned char)string[SerialLink.i_XOR];
    // Serial.println(SerialLink.c_XOR);
    // Serial.println(string[SerialLink.i_XOR]);
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
  // Serial.print("[gotSum[0] "); Serial.println(SerialLink.gotSum[0]);
  // Serial.print("[gotSum[1] "); Serial.println(SerialLink.gotSum[1]);
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
  Serial2.begin(115200); while(!Serial2);
  Serial1.setTimeout(100);
  Serial2.setTimeout(100);
  Serial1.flush();
  Serial2.flush();

  // setup IO
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }
  Serial.println("starting...");
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

void processMatrixData() {

  // reset values
  update_portmap_bool=false;
  SerialLink.validation = false;
  SerialLink.i_token = 0;
  SerialLink.token = strtok(SerialLink.BUFFER, ",");
  while(SerialLink.token != NULL) {

    // uncomment to debug
    Serial.print("[" + String(matrix_port_map[0][SerialLink.i_token]) + "] [RXD TOKEN] "); Serial.println(SerialLink.token);

    // check eack token for portmap
    if (SerialLink.i_token<20) { 
      for (int i=0; i<20; i++) {
        tmp_matrix_port_map[0][i] = atoi(SerialLink.token);
        if (atoi(SerialLink.token) != matrix_port_map[0][i]) {update_portmap_bool=true;}
        // uncomment to debug
        Serial.println(String(i) + " [portmap] " + String(matrix_port_map[0][i]));
        
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
      SerialLink.validation = validateChecksum(SerialLink.TMP);
      // try to get another read. this may be written differently later but currently this is the primary objective.
      // Serial.print("[matrix validation] "); Serial.println(SerialLink.validation);
      break;
    }
    // iterate counters and snap off used token
    SerialLink.i_token++;
    SerialLink.token = strtok(NULL, ",");
  }
}

// READ RXD1 --------------------------------------------------------------------------------------------------------
void readRXD1() {
  if (Serial1.available() > 0) {
    memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
    SerialLink.nbytes = (Serial1.readBytesUntil(ETX, SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));
    if (SerialLink.nbytes > 1) {
      
      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      strcpy(SerialLink.TMP, SerialLink.BUFFER);
      Serial.print("[RXD] "); Serial.println(SerialLink.BUFFER);
      SerialLink.TOKEN_i = 0;

      SerialLink.token = strtok(SerialLink.TMP, ",");
      if (strcmp(SerialLink.token, "$MATRX") == 0) {processMatrixData();}
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {

  // Serial.println("---------------------------------------");
  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time

  // readRXD1();

  // make high/low
  // satIOPortController();

  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);

}
