/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

Wiring CD74HC4067 16-Channel Analog Digital Multiplexer:
CD74HC4067 S0          -> ATMEGA2560 8 
CD74HC4067 S1          -> ATMEGA2560 9
CD74HC4067 S2          -> ATMEGA2560 10
CD74HC4067 S3          -> ATMEGA2560 11
ESP32 io22 (RXD)       -> CD74HC4067 SIG (output selected channel to ESP32)
ATMEGA2560 Serial1 RXD -> CD74HC4067 C0
WTGPS300               -> CD74HC4067 C1 

Wiring TCA9548A i2C Multiplexer:
TCA9548A: SDA, SCL -> ATMEGA2560: SDA 20, SCL 21
TCA9548A: SDA0, SCL1 -> DS3231 Precision RTC: D (Data), C (Clock)

Wiring Satellite Count and HDOP Precision Factor Indicator:
ATMEGA2560 5 -> LEDR
ATMEGA2560 6 -> LEDG
ATMEGA2560 7 -> LEDB

*/

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <limits.h>
#include <stdlib.h>
#include <RTClib.h>
#include <SPI.h>
#include <Wire.h>  
#include <TimeLib.h>
#include <CD74HC4067.h>

#define LEDSATSIGNALR 5
#define LEDSATSIGNALG 6
#define LEDSATSIGNALB 7

int MUX0_CHANNEL = 0;
int MUX1_CHANNEL = 0;

#define MAX_BUFF 1000

RTC_DS3231 rtc;
DateTime dt_now;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MATRIX DATA

// last satellite time placeholders
int rcv_year        = 2000;
int rcv_month       = 01;
int rcv_day         = 01;
int rcv_hour        = 00;
int rcv_minute      = 00;
int rcv_second      = 00;
int rcv_millisecond = 00;
char rcv_dt_0[56];
char rcv_dt_1[56];
char tmp_dt[56];



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
  char BUFFER[MAX_BUFF];        // read incoming bytes into this buffer
  char TMP[MAX_BUFF];           // buffer refined using ETX
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
  double seconds;                      // seconds accumulated since startup
  double mainLoopTimeTaken;            // current main loop time
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

int muxChannel[16][4]={
    {0,0,0,0}, //channel 0 port controller
    {1,0,0,0}, //channel 1 GPS
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

//Mux control pins
int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;
int controlPin[] = {s0, s1, s2, s3};

#define TCAADDR   0x70

void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  digitalWrite(controlPin[0], muxChannel[0][0]); // default channel 0 (esp32 listens to port controller)
  digitalWrite(controlPin[1], muxChannel[0][1]); // default channel 0 (esp32 listens to port controller)
  digitalWrite(controlPin[2], muxChannel[0][2]); // default channel 0 (esp32 listens to port controller)
  digitalWrite(controlPin[3], muxChannel[0][3]); // default channel 0 (esp32 listens to port controller)

  Serial.begin(115200);  while(!Serial);
  Serial1.begin(115200); while(!Serial1);
  Serial2.begin(115200); while(!Serial2);
  Serial1.setTimeout(100);
  Serial2.setTimeout(100);
  Serial1.flush();
  Serial2.flush();

  // setp TCA9548A
  Wire.begin();
  // setup RTC
  rtc.begin();   //initializes the I2C to the RTC  

  MUX0_CHANNEL = 0;
  for(int i = 0; i < 4; i++){
    digitalWrite(controlPin[i], muxChannel[MUX0_CHANNEL][i]);
  }
  MUX1_CHANNEL = 0;
  tcaselect(MUX1_CHANNEL); // zero by default

  // setup IO
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }

  pinMode(LEDSATSIGNALR, OUTPUT);
  pinMode(LEDSATSIGNALG, OUTPUT);
  pinMode(LEDSATSIGNALB, OUTPUT);
  digitalWrite(LEDSATSIGNALR, LOW);
  digitalWrite(LEDSATSIGNALG, LOW);
  digitalWrite(LEDSATSIGNALB, LOW);

  Serial.println("starting...");
}

void SerialDisplayRTCDateTime() {
  // test dt
  DateTime dt_now = rtc.now();
  // display dt
  Serial.println("[rtc] " + String(dt_now.hour()) + ":" + String(dt_now.minute()) + ":" + String(dt_now.second()) + " " + String(dt_now.day()) + "/" + String(dt_now.month()) + "/" + String(dt_now.year()));
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                    PORT CONTROLLER

/* please ensure checksum was validated properly before actually implementing this final function in the feild */

void satIOPortController() {

  Serial.println("[satIOPortController] ");

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

  SerialLink.validation = validateChecksum(SerialLink.BUFFER);
  
  if (SerialLink.validation==true) {

    Serial.println("[processMatrixData] ");

    // clear temporary datetime char array ready to reconstruct and compare to previous datetime char array
    memset(rcv_dt_0, 0, sizeof(rcv_dt_0));

    // snap off the first token and then keep breaking off a token in loop
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
          Serial.println("[switch: " + String(i) + "] [port: " + String(matrix_port_map[0][i]) + "] [state: " + String(digitalRead(tmp_matrix_port_map[0][i])) + "]");
          
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

      // reconstruct temporary datetime char array with token each iteration
      strcat(rcv_dt_0, SerialLink.token);
      if (SerialLink.i_token==40) {rcv_year = atoi(SerialLink.token);}
      if (SerialLink.i_token==41) {rcv_month = atoi(SerialLink.token);}
      if (SerialLink.i_token==42) {rcv_day = atoi(SerialLink.token);}
      if (SerialLink.i_token==43) {rcv_hour = atoi(SerialLink.token);}
      if (SerialLink.i_token==44) {rcv_minute = atoi(SerialLink.token);}
      if (SerialLink.i_token==45) {rcv_second = atoi(SerialLink.token);}

      // satellite count > 0 indicator
      if (SerialLink.i_token==46) {
        Serial.println("[satcount] " + String(SerialLink.token));
        if (atoi(SerialLink.token)==0) {
          digitalWrite(LEDSATSIGNALR, HIGH);
          digitalWrite(LEDSATSIGNALG, LOW);
          digitalWrite(LEDSATSIGNALB, LOW);
        }
        else if (atoi(SerialLink.token)==1) {
          digitalWrite(LEDSATSIGNALR, LOW);
          digitalWrite(LEDSATSIGNALG, HIGH);
          digitalWrite(LEDSATSIGNALB, LOW);
        }
        else if (atoi(SerialLink.token)==2) {
          digitalWrite(LEDSATSIGNALR, LOW);
          digitalWrite(LEDSATSIGNALG, LOW);
          digitalWrite(LEDSATSIGNALB, HIGH);
        }
      }

      // iterate counters and snap off used token
      SerialLink.i_token++;
      SerialLink.token = strtok(NULL, ",");
    }
  }
}

// READ RXD1 --------------------------------------------------------------------------------------------------------
void readRXD1() {

  // rcv_matrix_tag = false;
  if (Serial1.available() > 0) {

    // Serial.println("[readRXD1] ");

    memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
    SerialLink.nbytes = (Serial1.readBytesUntil(ETX, SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));
    if (SerialLink.nbytes > 1) {
      
      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      strcpy(SerialLink.TMP, SerialLink.BUFFER);
      // Serial.print("[RXD] "); Serial.println(SerialLink.BUFFER);
      SerialLink.TOKEN_i = 0;

      // get tag token
      SerialLink.token = strtok(SerialLink.TMP, ",");

      // parse incoming analogu multiplexer instructions sentence
      if (strcmp(SerialLink.token, "$MUX") == 0) {
        SerialLink.validation = validateChecksum(SerialLink.BUFFER);
        if (SerialLink.validation==true) {

          // instruct analogu multiplexer
          SerialLink.token = strtok(NULL, ",");
          MUX0_CHANNEL = atoi(SerialLink.token);
          for(int i = 0; i < 4; i++){
            digitalWrite(controlPin[i], muxChannel[MUX0_CHANNEL][i]);
          }

          // instruct i2C multiplexer
          SerialLink.token = strtok(NULL, ",");
          MUX1_CHANNEL = atoi(SerialLink.token);
          tcaselect(MUX1_CHANNEL);
        }
      }
    }

    // parse matrix sentence
    if (strcmp(SerialLink.token, "$MATRIX") == 0) {
      processMatrixData();
    }
  }
}

void writeTXD1() {
  if (Serial1.availableForWrite() > 0) {

    // Serial.println("[writeTXD1] ");

    dt_now = rtc.now();

    memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));

    strcpy(SerialLink.BUFFER, "$RTC,");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(dt_now.year(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(dt_now.month(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(dt_now.day(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(dt_now.hour(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(dt_now.minute(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(dt_now.second(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    // append checksum
    createChecksum(SerialLink.BUFFER);
    strcat(SerialLink.BUFFER, "*");
    strcat(SerialLink.BUFFER, SerialLink.checksum);
    strcat(SerialLink.BUFFER, "\n");

    // Serial.println(SerialLink.BUFFER);

    Serial1.write(SerialLink.BUFFER);
    Serial1.write(ETX);

  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {

  // Serial.println("---------------------------------------");
  // SerialDisplayRTCDateTime();
  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time

  // read other serial
  // readRXD2();

  // Serial.println("[loop] ");

  
  // read matrix data
  readRXD1();
  // if (MUX0_CHANNEL==1) {readRXD1();}
  
  if (MUX0_CHANNEL==0) {writeTXD1();}
  // writeTXD1();

  // run portcontroller
  // if (SerialLink.validation==true) {satIOPortController();}

  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);

  // delay(1000);

}
