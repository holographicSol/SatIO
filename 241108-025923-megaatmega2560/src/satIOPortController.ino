/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

Required wiring:

  ESP32 io27 (TXD) -> ATMEGA2560 Serial1 (RXD)

  ESP32 io22 (RXD) -> ATMEGA2560 Serial1 (TXD)

  WTGPS300P (TXD)  -> ATMEGA2560 Serial2 (RXD)

*/

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <limits.h>
#include <TinyGPSPlus.h>
#include <stdlib.h>
// #include <conio.h>

TinyGPSPlus gps;

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
  char BUFFER[1500];            // read incoming bytes into this buffer
  char TMP[1500];               // buffer refined using ETX
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
char gngga_sentence[1200];
char gnrmc_sentence[1200];
char gpatt_sentence[1200];
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

bool nmeaChecksumCompare(const uint8_t packet[], const uint8_t packetEndIndex, const uint8_t receivedCSbyte1, const uint8_t receivedCSbyte2)
    {
      uint8_t calcChecksum = 0;

      for(uint8_t i=1; i<packetEndIndex-4; ++i) // packetEndIndex is the "size" of the packet minus 1. Loop from 1 to packetEndIndex-4 because the checksum is calculated between $ and *
      {
        calcChecksum = calcChecksum^packet[i];
      }

      uint8_t nibble1 = (calcChecksum&0xF0) >> 4; //"Extracts" the first four bits and shifts them 4 bits to the right. Bitwise AND followed by a bitshift
      uint8_t nibble2 = calcChecksum&0x0F; 

      uint8_t translatedByte1 = (nibble1<=0x9) ? (nibble1+'0') : (nibble1-10+'A'); //Converting the number "nibble1" into the ASCII representation of that number
      uint8_t translatedByte2 = (nibble2<=0x9) ? (nibble2+'0') : (nibble2-10+'A'); //Converting the number "nibble2" into the ASCII representation of that number

      if(translatedByte1==receivedCSbyte1 && translatedByte2==receivedCSbyte2) //Check if the checksum calculated from the packet payload matches the checksum in the packet
      { 
        return true; 
      } 
      else
      { 
        return false; 
      }
    }

bool validateChecksum(char * buffer) {
  /* validate a sentence appended with a checksum */

  // uncomment to debug
  // Serial.println("[validateChecksum] " + String(buffer));

  memset(SerialLink.gotSum, 0, sizeof(SerialLink.gotSum));
  
  SerialLink.gotSum[0] = buffer[strlen(buffer) - 3];
  SerialLink.gotSum[1] = buffer[strlen(buffer) - 2];

  // Serial.print("[checksum_in_buffer] "); Serial.println(SerialLink.gotSum);

  uint8_t calcChecksum = 0;

  for(uint8_t i=1; i<strlen(buffer)-4; ++i) // packetEndIndex is the "size" of the packet minus 1. Loop from 1 to packetEndIndex-4 because the checksum is calculated between $ and *
  {
    // Serial.println(calcChecksum);
    calcChecksum = calcChecksum^(uint8_t)buffer[i];
  }

  uint8_t nibble1 = (calcChecksum&0xF0) >> 4; //"Extracts" the first four bits and shifts them 4 bits to the right. Bitwise AND followed by a bitshift
  uint8_t nibble2 = calcChecksum&0x0F;

  // Serial.println("[nibble1] " + String(nibble1));
  // Serial.println("[nibble2] " + String(nibble2));

  uint8_t translatedByte1 = (nibble1<=0x9) ? (nibble1+'0') : (nibble1-10+'A'); //Converting the number "nibble1" into the ASCII representation of that number
  uint8_t translatedByte2 = (nibble2<=0x9) ? (nibble2+'0') : (nibble2-10+'A'); //Converting the number "nibble2" into the ASCII representation of that number

  // Serial.println("[translatedByte1] " + String(translatedByte1));
  // Serial.println("[translatedByte2] " + String(translatedByte2));

  if(translatedByte1==SerialLink.gotSum[0] && translatedByte2==SerialLink.gotSum[1]) //Check if the checksum calculated from the packet payload matches the checksum in the packet
  {
    // Serial.println("[validateChecksum] true");
    return true; 
  } 
  else
  { 
    // Serial.println("[validateChecksum] false");
    return false; 
  }

  // SerialLink.checksum_of_buffer =  getCheckSum(buffer);
  // Serial.print("[checksum_of_buffer] "); Serial.println(SerialLink.checksum_of_buffer);
  // sprintf(SerialLink.checksum,"%X",SerialLink.checksum_of_buffer);
  // Serial.print("[checksum_of_buffer converted] "); Serial.println(SerialLink.checksum);
  // SerialLink.checksum_in_buffer = h2d2(SerialLink.gotSum[0], SerialLink.gotSum[1]);
  // Serial.print("[checksum_in_buffer] "); Serial.println(SerialLink.checksum_in_buffer);

  // if (strcmp(SerialLink.gotSum, SerialLink.checksum)==0) {return true;}
  // return false;
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

// RXD1 THROTTLE CHECKS ---------------------------------------------------------------------------------------------
bool RXD1ThrottleChecks() {
  SerialLink.T0_RXD_1 = millis();
  if (SerialLink.T0_RXD_1 >= SerialLink.T1_RXD_1+SerialLink.TT_RXD_1) {
    SerialLink.T1_RXD_1 = SerialLink.T0_RXD_1;
    return true;
  }
}

// TXD1 THROTTLE CHECKS ---------------------------------------------------------------------------------------------
bool TXD1ThrottleChecks() {
  SerialLink.T0_TXD_1 = millis();
  if (SerialLink.T0_TXD_1 >= SerialLink.T1_TXD_1+SerialLink.TT_TXD_1) {
    SerialLink.T1_TXD_1 = SerialLink.T0_TXD_1;
    return true;
  }
}

// REMOVE ETX -------------------------------------------------------------------------------------------------------
void removeNL() {
  memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
  SerialLink.nbytes = sizeof(SerialLink.BUFFER);
  if (SerialLink.nbytes != 0) {
    for(SerialLink.i_nbytes = 0; SerialLink.i_nbytes < SerialLink.nbytes; SerialLink.i_nbytes++) {
      if (SerialLink.BUFFER[SerialLink.i_nbytes] == '\n')
        break;
      else {SerialLink.TMP[SerialLink.i_nbytes] = SerialLink.BUFFER[SerialLink.i_nbytes];}
    }
  }
  memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
  strcpy(SerialLink.BUFFER, SerialLink.TMP);
}

// REMOVE ETX -------------------------------------------------------------------------------------------------------
void removeETX() {
  memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
  SerialLink.nbytes = sizeof(SerialLink.BUFFER);
  if (SerialLink.nbytes != 0) {
    for(SerialLink.i_nbytes = 0; SerialLink.i_nbytes < SerialLink.nbytes; SerialLink.i_nbytes++) {
      if (SerialLink.BUFFER[SerialLink.i_nbytes] == ETX)
        break;
      else {SerialLink.TMP[SerialLink.i_nbytes] = SerialLink.BUFFER[SerialLink.i_nbytes];}
    }
  }
  memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
  strcpy(SerialLink.BUFFER, SerialLink.TMP);
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

// $GNGGA,081016.80,0.0,N,0.0,W,2,34,0.5,6.000*70
// $GNGGA,081017.60,0.0,N,0.0,W,2,33,0.5,6.000*74

// $GNRMC,081016.80,A,0.0,N,0.0,W,0.01,128.4941124,0,W,A*38
// $GNRMC,081017.60,A,0.0,N,0.0,W,0.01,128.491124,0,W,A*3B  // note received is not ddmmyy like it should be: 128.491124

void readGPS() {
  gngga_bool = false;
  gnrmc_bool = false;
  gpatt_bool = false;
  memset(gngga_sentence, 0, sizeof(gngga_sentence));
  memset(gnrmc_sentence, 0, sizeof(gnrmc_sentence));
  memset(gpatt_sentence, 0, sizeof(gpatt_sentence));

  if (Serial2.available() > 0) {

    while(1) {
      
      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      SerialLink.nbytes = (Serial2.readBytesUntil('\r\n', SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));

      if (SerialLink.nbytes>0) {

        // Serial.print("[RXD] " + String(SerialLink.BUFFER)); // debug

        if (gngga_bool==true && gnrmc_bool==true && gpatt_bool==true) {break;}

        if (strncmp(SerialLink.BUFFER, "$GPATT", 6) == 0) {
          strcpy(gpatt_sentence, SerialLink.BUFFER);
          gpatt_bool = true;
        }

        if (strncmp(SerialLink.BUFFER, "$GNGGA", 6) == 0) {
          strcpy(gngga_sentence, SerialLink.BUFFER);
          gngga_bool = true;
        }

        if (strncmp(SerialLink.BUFFER, "$GNRMC", 6) == 0) {
          strcpy(gnrmc_sentence, SerialLink.BUFFER);
          gnrmc_bool = true; 
        }
      }
    }
    // Serial.println("[CHECKING] ");
    // gngga_valid_checksum = validateChecksum(gngga_sentence);
    // gnrmc_valid_checksum = validateChecksum(gnrmc_sentence);
    // gpatt_valid_checksum = validateChecksum(gpatt_sentence);

    // if (gngga_valid_checksum==true && gnrmc_valid_checksum==true && gpatt_valid_checksum==true) {
    // Serial.println("[WRITING] ");
    Serial1.write(gngga_sentence);
    Serial1.write(ETX);
    Serial1.write(gnrmc_sentence);
    Serial1.write(ETX);
    Serial1.write(gpatt_sentence);
    Serial1.write(ETX);
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
      
      removeETX();
      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      strcpy(SerialLink.TMP, SerialLink.BUFFER);
      // Serial.print("[RXD] "); Serial.println(SerialLink.BUFFER);
      SerialLink.TOKEN_i = 0;

      SerialLink.token = strtok(SerialLink.TMP, ",");
      if (strcmp(SerialLink.token, "$MATRX") == 0) {SerialLink.syn = true;}
      // if (strcmp(SerialLink.token, "$MATRX") == 0) {SerialLink.syn = true; processMatrixData();}
      else if (strcmp(SerialLink.token, "$SYN") == 0) {SerialLink.syn = true;}
    }
  }
}

// WRITE TXD1 -------------------------------------------------------------------------------------------------------
void writeTXD1() {
  if (TXD1ThrottleChecks() == true) {
    if (Serial1.availableForWrite()) {
      // Serial.print("[TXD] "); Serial.println(SerialLink.BUFFER);
      Serial1.write(SerialLink.BUFFER);
      Serial1.write(ETX);
    }
  }
}

// SEND SYN ---------------------------------------------------------------------------------------------------------
void sendSyn() {
  SerialLink.i_sync++;
  if (SerialLink.i_sync>LONG_MAX) {SerialLink.i_sync=0;}
  itoa(SerialLink.i_sync, SerialLink.char_i_sync, 10);
  memset(SerialLink.BUFFER, 0, 1024);
  strcat(SerialLink.BUFFER, "$SYN,");
  strcat(SerialLink.BUFFER, SerialLink.char_i_sync);
  writeTXD1();
}

// RECEIVE SYN ------------------------------------------------------------------------------------------------------
void receiveSyn() {
  // while (1) {readRXD1(); Serial.println("[syn] waiting"); if (SerialLink.syn == true) {SerialLink.syn = false; break;}}
  while (1) {readRXD1(); if (SerialLink.syn == true) {SerialLink.syn = false; break;}}
}

// SEND RECEIVE SYN -------------------------------------------------------------------------------------------------
void synCom() {
  // Serial.println("-------------------------------------------");
  sendSyn();
  receiveSyn();
}

// RECEIVE DATA -----------------------------------------------------------------------------------------------------
void receiveData() {
  // while (1) {readRXD1(); Serial.println("[data] waiting"); if (SerialLink.syn == true) {SerialLink.syn = false; break;}}
  while (1) {readRXD1(); if (SerialLink.syn == true) {SerialLink.syn = false; break;}}
}

// SEND DATA --------------------------------------------------------------------------------------------------------
void sendData(char * data) {
  memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER)); strcat(SerialLink.BUFFER, data); writeTXD1();
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {

  // Serial.println("---------------------------------------");
  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time

  // sendSyn();

  // sync received
  synCom(); readGPS();

  // make high/low
  // satIOPortController();

  // delay(1000);

  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);

}
