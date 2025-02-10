/*

Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIOPortController - Receives messages from SatIO over serial and manipulates IO accordingly.
                      This file should be flashed to ATMEGA2560.

Required wiring for port controller to receive instructions from ESP32:
ESP32 io27 (TXD) -> ATMEGA2560 Serial1 (RXD)

Wiring for portcontroller to sync RTC on latest satellite downlink:
ATMEGA2560: SDA 20, SCL 21 -> DS3231 Precision RTC: D (Data), C (Clock)

Other wiring for 1x button and 1x LED can be ignored for now.

*/

#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include <limits.h>
#include <TinyGPSPlus.h>
#include <stdlib.h>
#include <RTClib.h>
// #include <conio.h>
#include <SPI.h>
#include <Wire.h>  
#include <TimeLib.h>
#include <CD74HC4067.h>

int MUX0_CHANNEL = 0;

#define MAX_BUFF 1000

RTC_DS3231 rtc;

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


volatile bool portcontroller_enabled = false;
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
    // control pin 0
    {0,0,0,0}, //channel 0 port controller
    {1,0,0,0}, //channel 1 GPS
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    // control pin 1
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    // control pin 2
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    // control pin 4
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
// s0 s1 s2 s3
//  controlPin{8, 9, 10, 11);  // create a new CD74HC4067 object with its four control pins
int controlPin[] = {s0, s1, s2, s3};

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

  // setup IO
  for (int i=0; i<20; i++) {
    pinMode(matrix_port_map[0][i], OUTPUT);
    digitalWrite(matrix_port_map[0][i], LOW);
  }

  // indicator led
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // button
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), ISR_Toggle_PortController, FALLING);

  Wire.begin();  //sets up the I2C  
  rtc.begin();   //initializes the I2C to the RTC  
  // SerialDisplayRTCDateTime();
    
  //  Set the RTC Time to 5:10:30 Nov 3 2020  
  // rtc.adjust(DateTime(2020,11,3,5,10,30));  
  //Set Arduino Time Library different than RTC time 9:27:05 so see how sync works  
  // setTime(9, 27, 05, 4, 07, 2015);  

  Serial.println("starting...");


}

void SerialDisplayRTCDateTime() {
  // test dt
  DateTime now = rtc.now();
  // display dt
  Serial.println("[rtc] " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + " " + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year()));
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

  SerialLink.validation = validateChecksum(SerialLink.BUFFER);
  
  if (SerialLink.validation==true) {

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

      // compare and set RTC accordingly
      if (strlen(rcv_dt_0)==14) {
        if (!strcmp(rcv_dt_0, rcv_dt_1)) {
          Serial.println("[adjusting RTC]");
          memset(rcv_dt_1, 0, sizeof(rcv_dt_1));
          strcpy(rcv_dt_1, rcv_dt_0);
          // adjust RTC according to last datetime received from satellites
          rtc.adjust(DateTime(rcv_year,rcv_month,rcv_day,rcv_hour,rcv_minute,rcv_second));
        }
      }

      // iterate counters and snap off used token
      SerialLink.i_token++;
      SerialLink.token = strtok(NULL, ",");
    }
  }
  else {
  }
}

// READ RXD1 --------------------------------------------------------------------------------------------------------
void readRXD1() {

  // rcv_matrix_tag = false;
  if (Serial1.available() > 0) {
    // while(1) {
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
        if (strcmp(SerialLink.token, "$MUX0") == 0) {
          SerialLink.validation = validateChecksum(SerialLink.BUFFER);
          if (SerialLink.validation==true) {
            SerialLink.token = strtok(NULL, ",");
            MUX0_CHANNEL = atoi(SerialLink.token);
            // Serial.println("[MUX TOKEN] " + String(MUX0_CHANNEL));

            for(int i = 0; i < 4; i++){
              // Serial.println("[MUX CHANNEL BYTE] " + String(muxChannel[MUX0_CHANNEL][i]));
              digitalWrite(controlPin[i], muxChannel[MUX0_CHANNEL][i]);
            }
          }
          // break;
        }
      }

      // parse matrix sentence
      if (strcmp(SerialLink.token, "$MATRIX") == 0) {
        processMatrixData();
        // break;
      // }
    }
  }
}

void writeTXD1() {
  if (Serial1.availableForWrite() > 0) {

    DateTime now = rtc.now();

    memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));

    strcpy(SerialLink.BUFFER, "$RTC,");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(now.year(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(now.month(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(now.day(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(now.hour(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(now.minute(), tmp_dt, 10);
    strcat(SerialLink.BUFFER, tmp_dt);
    strcat(SerialLink.BUFFER, ",");

    memset(tmp_dt, 0, sizeof(tmp_dt));
    itoa(now.second(), tmp_dt, 10);
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

void readRXD2() {
  if (Serial2.available() > 0) {
    memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
    SerialLink.nbytes = (Serial2.readBytesUntil(ETX, SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));
    if (SerialLink.nbytes > 1) {
      Serial1.write(SerialLink.BUFFER);
    }
  }
}


// todo: debounce
void ISR_Toggle_PortController() {
  Serial.println("[ISR_Toggle_PortController] connected" + String());
  // False
  // 1: disables portcontroller instructions.
  // 2: turns all satIO pins low.
  // True
  // 1: enables portcontroller instructions.
  if      (portcontroller_enabled == false) {portcontroller_enabled=true;}
  else if (portcontroller_enabled == true)  {portcontroller_enabled=false;}
  Serial.println("[portcontroller_enabled] " + String(portcontroller_enabled));
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                         MAIN LOOP

void loop() {

  // Serial.println("---------------------------------------");
  // SerialDisplayRTCDateTime();
  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time

  // read other serial
  // readRXD2();

  // Serial.println("[SANITY] ");

  // read matrix data
  readRXD1();

  writeTXD1();

  // debug
  // Serial.println("[switch] " + String(analogRead(2)));
  // Serial.println("[portcontroller_enabled] " + String(portcontroller_enabled));

  // port controller diabled
  portcontroller_enabled = true; // debug
  if (portcontroller_enabled == false) {
    for (int i=0; i<20; i++) {
      pinMode(matrix_port_map[0][i], OUTPUT);
      digitalWrite(matrix_port_map[0][i], LOW);
    }
    // indicate portcontroller diabled
    // digitalWrite(13, LOW);
    digitalWrite(12, LOW);
  }

  // port controller enabled
  else if (portcontroller_enabled == true)  {

    // indicate portcontroller enabled
    digitalWrite(12, HIGH);
    // digitalWrite(13, HIGH);

    // run portcontroller
    if (SerialLink.validation==true) {satIOPortController();}
  }

  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);

  // delay(1000);

}
