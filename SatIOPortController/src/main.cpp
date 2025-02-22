/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

ESP32 io25 (TXD) -> ATMEGA2560 Serial1 (RXD)
ESP32 null (RXD) -> ATMEGA2560 Serial1 (TXD)

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

*/

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <limits.h>
#include <stdlib.h>
#include <TimeLib.h>
#include <CD74HC4067.h>

#define LEDSATSIGNALR 51
#define LEDSATSIGNALG 52
#define LEDSATSIGNALB 53

#define LEDOVERLOADR 46
#define LEDOVERLOADG 47

#define MAX_BUFF 256

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
  char BUFFER[MAX_BUFF];        // read incoming bytes into this buffer
  char TMP[MAX_BUFF];           // buffer refined using ETX
  signed int nbytes;
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
  double mainLoopTimeTaken;            // current main loop time
  unsigned long mainLoopTimeStart;     // time recorded at the start of each iteration of main loop
};
TimeStruct timeData;

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
  // Serial.print("checksum_of_buffer: "); Serial.println(SerialLink.checksum_of_buffer);
  // Serial.printf("Hexadecimal number is: %X", SerialLink.checksum_of_buffer); Serial.println();

  sprintf(SerialLink.checksum,"%X",SerialLink.checksum_of_buffer);

  // uncomment to debug
  // Serial.print("checksum: "); Serial.println(SerialLink.checksum); Serial.println();
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP 

void ISR_ATMEGA_0() {
  // ready for i2C communication between atmega2560 and esp32
}

void setup() {
  // serial

  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);
  Serial1.setTimeout(50); // ensure this is set before begin()
  Serial1.begin(115200); while(!Serial1);
  Serial1.flush();

  // matrix switches
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }

  // satellite signal indicator
  pinMode(LEDSATSIGNALR, OUTPUT);
  pinMode(LEDSATSIGNALG, OUTPUT);
  pinMode(LEDSATSIGNALB, OUTPUT);
  digitalWrite(LEDSATSIGNALR, LOW);
  digitalWrite(LEDSATSIGNALG, LOW);
  digitalWrite(LEDSATSIGNALB, LOW);

  // overload indicator
  pinMode(LEDOVERLOADR, OUTPUT);
  pinMode(LEDOVERLOADG, OUTPUT);
  digitalWrite(LEDOVERLOADR, LOW);
  digitalWrite(LEDOVERLOADG, LOW);

  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                    PORT CONTROLLER

/* please ensure checksum was validated properly before actually implementing this final function in the feild */

void satIOPortController() {
  Serial.println("[satIOPortController] ");
  // Serial.println("[processing] " + String(SerialLink.BUFFER));

  // write switch state to pins in port map!
  for (int i=0; i<20; i++) {
    if (update_portmap_bool==true) {
      if (matrix_port_map[0][i] != tmp_matrix_port_map[0][i]) {
        digitalWrite(matrix_port_map[0][i], LOW);
        pinMode(matrix_port_map[0][i], INPUT);

        // Serial.println("[portmap] updating port: " + String(matrix_port_map[0][i]) + " -> " + String(tmp_matrix_port_map[0][i]));

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

int i_pmd = 0;
void processMatrixData() {
  Serial.println("[processMatrixData] ");

  // reset values
  SerialLink.i_token = 0;
  
  // checksum
  SerialLink.validation = false;
  SerialLink.validation = validateChecksum(SerialLink.BUFFER);
  if (SerialLink.validation==true) {
    
    // snap off the tag token
    SerialLink.token = strtok(NULL, ",");
    while(SerialLink.token != NULL) {
      
      // uncomment to debug
      Serial.print("[" + String(matrix_port_map[0][SerialLink.i_token]) + "] [RXD TOKEN] "); Serial.println(SerialLink.token);
      
      // port map
      if (SerialLink.i_token<20) {
        update_portmap_bool=false;
        for (i_pmd=0; i_pmd<20; i_pmd++) {
          tmp_matrix_port_map[0][i_pmd] = atoi(SerialLink.token);
          if (atoi(SerialLink.token) != matrix_port_map[0][i_pmd]) {update_portmap_bool=true;}

          // uncomment to debug
          Serial.println("[switch: " + String(i_pmd) + "] [port: " + String(matrix_port_map[0][i_pmd]) + "] [state: " + String(digitalRead(tmp_matrix_port_map[0][i_pmd])) + "]");
          
          SerialLink.i_token++;
          SerialLink.token = strtok(NULL, ",");
        }
      }

      // matrix switch states
      if ((SerialLink.i_token>=20) && (SerialLink.i_token<40)) { 
        for (i_pmd=0; i_pmd<20; i_pmd++) {
          if      (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][i_pmd] = 0;}
          else if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][i_pmd] = 1;}
          SerialLink.i_token++;
          SerialLink.token = strtok(NULL, ",");
        }
      }

      // signal indicator
      if (SerialLink.i_token==40) {
        if (strcmp(SerialLink.token, "0") == 0)
        // red
        {
          digitalWrite(LEDSATSIGNALG, LOW);
          digitalWrite(LEDSATSIGNALB, LOW);
          digitalWrite(LEDSATSIGNALR, LOW);
          digitalWrite(LEDSATSIGNALR, HIGH);
          // delay(3000);
        }
        // green
        else if (strcmp(SerialLink.token, "1") == 0)
        {
          digitalWrite(LEDSATSIGNALR, LOW);
          digitalWrite(LEDSATSIGNALB, LOW);
          digitalWrite(LEDSATSIGNALG, LOW);   
          digitalWrite(LEDSATSIGNALG, HIGH);
          // delay(3000);
        }
        // blue
        else if (strcmp(SerialLink.token, "2") == 0)
        {
          digitalWrite(LEDSATSIGNALR, LOW);
          digitalWrite(LEDSATSIGNALG, LOW);
          digitalWrite(LEDSATSIGNALB, LOW);
          digitalWrite(LEDSATSIGNALB, HIGH);
        }
      }

      // overload indicator
      if (SerialLink.i_token==41) {
        if (strcmp(SerialLink.token, "0") == 0) {digitalWrite(LEDOVERLOADR, LOW); digitalWrite(LEDOVERLOADG, LOW);}
        else if (strcmp(SerialLink.token, "1") == 1) {digitalWrite(LEDOVERLOADR, HIGH); digitalWrite(LEDOVERLOADG, HIGH);}
      }

      // iterate counters and snap off used token
      SerialLink.i_token++;
      SerialLink.token = strtok(NULL, ",");
    }
  }
}

// READ RXD1 --------------------------------------------------------------------------------------------------------
int i_rx = 0;
void readRXD1() {
  Serial.println("[readRXD1] ");
  for (i_rx=0; i_rx<10; i_rx++) {
    if (Serial1.available()) {

      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      SerialLink.nbytes = (Serial1.readBytesUntil(ETX, SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));

      if (SerialLink.nbytes > 1) {

        // store a copy that will not be tokenized (remains intact)
        memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
        strcpy(SerialLink.TMP, SerialLink.BUFFER);

        // Serial.print("[RXD] "); Serial.println(SerialLink.BUFFER);

        if (strncmp(SerialLink.BUFFER, "$MATRIX", 7) == 0) {
          // Note: if adding/removing tokens from the sending side then change the value below compared to nbytes
          if (SerialLink.nbytes == 116) {
            SerialLink.i_token = 0;
            // get tag token
            SerialLink.token = strtok(SerialLink.TMP, ",");
            // parse matrix sentence
            Serial.print("[RXD] "); Serial.println(SerialLink.BUFFER);
            processMatrixData();
            break;
          }
        }
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        PADDING

char pad_digits_new[56];            // a placeholder for digits preappended with zero's.
char pad_current_digits[56];        // a placeholder for digits to be preappended with zero's.

String pad3DigitsZero(int digits) {
  /* preappends char 0 to pad string of digits evenly */
  memset(pad_digits_new, 0, sizeof(pad_digits_new));
  memset(pad_current_digits, 0, sizeof(pad_current_digits));
  if ((digits < 1000) && (digits > 100)) {strcat(pad_digits_new, "0");}
  else if ((digits < 100) && (digits > 10)) {strcat(pad_digits_new, "00");}
  else if (digits < 10) {strcat(pad_digits_new, "000");}
  itoa(digits, pad_current_digits, 10);
  strcat(pad_digits_new, pad_current_digits);
  return pad_digits_new;
}

String padDigitZero(int digits) {
  /* preappends char 0 to pad string of digits evenly */
  memset(pad_digits_new, 0, sizeof(pad_digits_new));
  memset(pad_current_digits, 0, sizeof(pad_current_digits));
  if (digits < 10) {strcat(pad_digits_new, "0");}
  itoa(digits, pad_current_digits, 10);
  strcat(pad_digits_new, pad_current_digits);
  return pad_digits_new;
}

int i_tx = 0;
void writeTXD1Data0() {
  Serial.println("[writeTXD1Data0] ");
  for (i_tx=0; i_tx<20; i_tx++) {
    if (Serial1.availableForWrite()) {
      // data
      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      strcpy(SerialLink.BUFFER, "$D0,");
      // append checksum
      createChecksum(SerialLink.BUFFER);
      strcat(SerialLink.BUFFER, "*");
      strcat(SerialLink.BUFFER, SerialLink.checksum);
      strcat(SerialLink.BUFFER, "\n");
      Serial1.write(SerialLink.BUFFER);
      Serial1.write(ETX);
      // Serial.println("[TXD] " + String(SeSrialLink.BUFFER)); // debug (at a perfromance cost)
      break;
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {

  // Serial.println("---------------------------------------");
  
  Serial.println("[loop] ");

  timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time
  
  readRXD1();
  if (SerialLink.validation==true) {satIOPortController();}

  timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);

  delay(10);
}