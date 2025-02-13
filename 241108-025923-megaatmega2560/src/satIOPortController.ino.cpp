# 1 "C:\\Users\\Benjamin\\AppData\\Local\\Temp\\tmpis_37cc0"
#include <Arduino.h>
# 1 "C:/Users/Benjamin/Documents/PlatformIO/Projects/241108-025923-megaatmega2560/src/satIOPortController.ino"
# 45 "C:/Users/Benjamin/Documents/PlatformIO/Projects/241108-025923-megaatmega2560/src/satIOPortController.ino"
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

#define MAX_BUFF 2048

RTC_DS3231 rtc;
DateTime dt_now;

#include <DHT.h>
#define DHTPIN 2



#define DHTTYPE DHT11


DHT dht(DHTPIN, DHTTYPE);

float dht11_h_0;
float dht11_c_0;
float dht11_f_0;
float dht11_hif_0;
float dht11_hic_0;

#define PHOTORESISTOR_0 A0
int photoresistor_0;

#define TRACKING_0 A1
int tracking_0;





int rcv_year = 2000;
int rcv_month = 01;
int rcv_day = 01;
int rcv_hour = 00;
int rcv_minute = 00;
int rcv_second = 00;
int rcv_millisecond = 00;
char rcv_dt_0[56];
char rcv_dt_1[56];
char tmp_dt[56];



signed int tmp_port;
bool update_portmap_bool = false;


int max_matrix_switch_states = 20;
bool matrix_switch_state[1][20] = {
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  }
};


signed int matrix_port_map[1][20] = {
  {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  }
};


signed int tmp_matrix_port_map[1][20] = {
  {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  }
};



#define ETX 0x03




struct SerialLinkStruct {
  signed int i_nbytes;
  long i_sync;
  char char_i_sync[56];
  bool syn = false;
  bool data = false;
  char BUFFER[MAX_BUFF];
  char TMP[MAX_BUFF];
  signed int nbytes;
  unsigned long T0_RXD_1 = 0;
  unsigned long T1_RXD_1 = 0;
  unsigned long TT_RXD_1 = 0;
  unsigned long T0_TXD_1 = 0;
  unsigned long T1_TXD_1 = 0;
  unsigned long TT_TXD_1 = 10;
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
  double seconds;
  double mainLoopTimeTaken;
  unsigned long mainLoopTimeStart;
  unsigned long mainLoopTimeTakenMax;
  unsigned long mainLoopTimeTakenMin;
  unsigned long t0;
  unsigned long t1;
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
int getCheckSum(char * string);
uint8_t h2d(char hex);
uint8_t h2d2(char h1, char h2);
bool validateChecksum(char * buffer);
void createChecksum(char * buffer);
void tcaselect(uint8_t channel);
void setup();
void SerialDisplayRTCDateTime();
void satIOPortController();
void processMatrixData();
void readRXD1();
String pad3DigitsZero(int digits);
String padDigitZero(int digits);
void writeTXD1Data();
void loop();
#line 198 "C:/Users/Benjamin/Documents/PlatformIO/Projects/241108-025923-megaatmega2560/src/satIOPortController.ino"
int getCheckSum(char * string) {




  for (SerialLink.XOR = 0, SerialLink.i_XOR = 0; SerialLink.i_XOR < strlen(string); SerialLink.i_XOR++) {
    SerialLink.c_XOR = (unsigned char)string[SerialLink.i_XOR];


    if (SerialLink.c_XOR == '*') break;
    if (SerialLink.c_XOR != '$') SerialLink.XOR ^= SerialLink.c_XOR;
  }


  return SerialLink.XOR;
}


uint8_t h2d(char hex) {if(hex > 0x39) hex -= 7; return(hex & 0xf);}





uint8_t h2d2(char h1, char h2) {return (h2d(h1)<<4) | h2d(h2);}

bool validateChecksum(char * buffer) {




  SerialLink.gotSum[2];
  SerialLink.gotSum[0] = buffer[strlen(buffer) - 3];
  SerialLink.gotSum[1] = buffer[strlen(buffer) - 2];


  SerialLink.checksum_of_buffer = getCheckSum(buffer);
  SerialLink.checksum_in_buffer = h2d2(SerialLink.gotSum[0], SerialLink.gotSum[1]);
  if (SerialLink.checksum_of_buffer == SerialLink.checksum_in_buffer) {return true;} else {return false;}
}

void createChecksum(char * buffer) {
  SerialLink.checksum_of_buffer = getCheckSum(buffer);





  sprintf(SerialLink.checksum,"%X",SerialLink.checksum_of_buffer);



}




int muxChannel[16][4]={
    {0,0,0,0},
    {1,0,0,0},
    {0,1,0,0},
    {1,1,0,0},
    {0,0,1,0},
    {1,0,1,0},
    {0,1,1,0},
    {1,1,1,0},
    {0,0,0,1},
    {1,0,0,1},
    {0,1,0,1},
    {1,1,0,1},
    {0,0,1,1},
    {1,0,1,1},
    {0,1,1,1},
    {1,1,1,1}
  };


int s0 = 8;
int s1 = 9;
int s2 = 10;
int s3 = 11;
int controlPin[] = {s0, s1, s2, s3};

#define TCAADDR 0x70

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
  digitalWrite(controlPin[0], muxChannel[0][0]);
  digitalWrite(controlPin[1], muxChannel[0][1]);
  digitalWrite(controlPin[2], muxChannel[0][2]);
  digitalWrite(controlPin[3], muxChannel[0][3]);

  pinMode(PHOTORESISTOR_0, INPUT);
  pinMode(TRACKING_0, INPUT);

  Serial.begin(115200); while(!Serial);
  Serial1.begin(115200); while(!Serial1);
  Serial2.begin(115200); while(!Serial2);
  Serial1.setTimeout(100);
  Serial2.setTimeout(100);
  Serial1.flush();
  Serial2.flush();


  Wire.begin();

  rtc.begin();

  MUX0_CHANNEL = 0;
  for(int i = 0; i < 4; i++){
    digitalWrite(controlPin[i], muxChannel[MUX0_CHANNEL][i]);
  }
  MUX1_CHANNEL = 0;
  tcaselect(MUX1_CHANNEL);

  dht.begin();


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

  Serial.println(F("------------------------------------"));

  Serial.println("starting...");
}

void SerialDisplayRTCDateTime() {

  DateTime dt_now = rtc.now();

  Serial.println("[rtc] " + String(dt_now.hour()) + ":" + String(dt_now.minute()) + ":" + String(dt_now.second()) + " " + String(dt_now.day()) + "/" + String(dt_now.month()) + "/" + String(dt_now.year()));
}






void satIOPortController() {






  for (int i=0; i<20; i++) {


    if (update_portmap_bool==true) {
      if (matrix_port_map[0][i] != tmp_matrix_port_map[0][i]) {
        digitalWrite(matrix_port_map[0][i], LOW);
        pinMode(matrix_port_map[0][i], INPUT);




        matrix_port_map[0][i]=tmp_matrix_port_map[0][i];
        pinMode(matrix_port_map[0][i], OUTPUT);
      }
    }


    digitalWrite(matrix_port_map[0][i], matrix_switch_state[0][i]);



  }
}

void processMatrixData() {


  update_portmap_bool=false;
  SerialLink.validation = false;
  SerialLink.i_token = 0;

  SerialLink.validation = validateChecksum(SerialLink.BUFFER);

  if (SerialLink.validation==true) {




    memset(rcv_dt_0, 0, sizeof(rcv_dt_0));


    SerialLink.token = strtok(NULL, ",");
    while(SerialLink.token != NULL) {





      if (SerialLink.i_token<20) {
        for (int i=0; i<20; i++) {
          tmp_matrix_port_map[0][i] = atoi(SerialLink.token);
          if (atoi(SerialLink.token) != matrix_port_map[0][i]) {update_portmap_bool=true;}




          SerialLink.i_token++;
          SerialLink.token = strtok(NULL, ",");
        }
      }


      if ((SerialLink.i_token>=20) && (SerialLink.i_token<40)) {
        for (int i=0; i<20; i++) {
          if (strcmp(SerialLink.token, "0") == 0) { matrix_switch_state[0][i] = 0;}
          else if (strcmp(SerialLink.token, "1") == 0) { matrix_switch_state[0][i] = 1;}
          SerialLink.i_token++;
          SerialLink.token = strtok(NULL, ",");
        }
      }


      strcat(rcv_dt_0, SerialLink.token);
      if (SerialLink.i_token==40) {rcv_year = atoi(SerialLink.token);}
      if (SerialLink.i_token==41) {rcv_month = atoi(SerialLink.token);}
      if (SerialLink.i_token==42) {rcv_day = atoi(SerialLink.token);}
      if (SerialLink.i_token==43) {rcv_hour = atoi(SerialLink.token);}
      if (SerialLink.i_token==44) {rcv_minute = atoi(SerialLink.token);}
      if (SerialLink.i_token==45) {rcv_second = atoi(SerialLink.token);}


      if (SerialLink.i_token==46) {

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


      SerialLink.i_token++;
      SerialLink.token = strtok(NULL, ",");
    }
  }
}


void readRXD1() {

  for (int i=0; i<8; i++) {

    if (Serial1.available() > 0) {



      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      SerialLink.nbytes = (Serial1.readBytesUntil(ETX, SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));
      if (SerialLink.nbytes > 1) {

        memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
        strcpy(SerialLink.TMP, SerialLink.BUFFER);

        SerialLink.TOKEN_i = 0;


        SerialLink.token = strtok(SerialLink.TMP, ",");


        if (strcmp(SerialLink.token, "$MUX") == 0) {
          SerialLink.validation = validateChecksum(SerialLink.BUFFER);
          if (SerialLink.validation==true) {


            SerialLink.token = strtok(NULL, ",");
            MUX0_CHANNEL = atoi(SerialLink.token);

            for(int i = 0; i < 4; i++){
              digitalWrite(controlPin[i], muxChannel[MUX0_CHANNEL][i]);
            }


            SerialLink.token = strtok(NULL, ",");
            MUX1_CHANNEL = atoi(SerialLink.token);
            tcaselect(MUX1_CHANNEL);



          }
        }
      }


      if (strcmp(SerialLink.token, "$MATRIX") == 0) {
        processMatrixData();
      }
    }
    break;
  }
}




char pad_digits_new[56];
char pad_current_digits[56];

String pad3DigitsZero(int digits) {

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

  memset(pad_digits_new, 0, sizeof(pad_digits_new));
  memset(pad_current_digits, 0, sizeof(pad_current_digits));
  if (digits < 10) {strcat(pad_digits_new, "0");}
  itoa(digits, pad_current_digits, 10);
  strcat(pad_digits_new, pad_current_digits);

  return pad_digits_new;
}

void writeTXD1Data() {

  for (int i=0; i<8; i++) {

    if (Serial1.availableForWrite() > 0) {



      memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
      strcpy(SerialLink.BUFFER, "$D0,");


      dht11_h_0 = dht.readHumidity();
      dht11_c_0 = dht.readTemperature();
      dht11_f_0 = dht.readTemperature(true);
      if (isnan(dht11_h_0) || isnan(dht11_c_0) || isnan(dht11_f_0)) {
        Serial.println(F("Failed to read from DHT sensor!"));
      }
      dht11_hif_0 = dht.computeHeatIndex(dht11_f_0, dht11_h_0);
      dht11_hic_0 = dht.computeHeatIndex(dht11_c_0, dht11_h_0, false);

      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      dtostrf(dht11_h_0, 2, 2, SerialLink.TMP);


      strcat(SerialLink.BUFFER, SerialLink.TMP);
      strcat(SerialLink.BUFFER, ",");

      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      dtostrf(dht11_c_0, 2, 2, SerialLink.TMP);


      strcat(SerialLink.BUFFER, SerialLink.TMP);
      strcat(SerialLink.BUFFER, ",");

      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      dtostrf(dht11_f_0, 2, 2, SerialLink.TMP);


      strcat(SerialLink.BUFFER, SerialLink.TMP);
      strcat(SerialLink.BUFFER, ",");

      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      dtostrf(dht11_hif_0, 2, 2, SerialLink.TMP);


      strcat(SerialLink.BUFFER, SerialLink.TMP);
      strcat(SerialLink.BUFFER, ",");

      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      dtostrf(dht11_hic_0, 2, 2, SerialLink.TMP);


      strcat(SerialLink.BUFFER, SerialLink.TMP);
      strcat(SerialLink.BUFFER, ",");


      photoresistor_0 = analogRead(PHOTORESISTOR_0);

      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      itoa(photoresistor_0, SerialLink.TMP, 10);

      strcat(SerialLink.BUFFER, SerialLink.TMP);
      strcat(SerialLink.BUFFER, ",");


      tracking_0 = analogRead(TRACKING_0);

      memset(SerialLink.TMP, 0, sizeof(SerialLink.TMP));
      itoa(tracking_0, SerialLink.TMP, 10);

      strcat(SerialLink.BUFFER, SerialLink.TMP);
      strcat(SerialLink.BUFFER, ",");


      dt_now = rtc.now();
      strcat(SerialLink.BUFFER, padDigitZero(dt_now.year()).c_str());
      strcat(SerialLink.BUFFER, ",");
      strcat(SerialLink.BUFFER, padDigitZero(dt_now.month()).c_str());
      strcat(SerialLink.BUFFER, ",");
      strcat(SerialLink.BUFFER, padDigitZero(dt_now.day()).c_str());
      strcat(SerialLink.BUFFER, ",");
      strcat(SerialLink.BUFFER, padDigitZero(dt_now.hour()).c_str());
      strcat(SerialLink.BUFFER, ",");
      strcat(SerialLink.BUFFER, padDigitZero(dt_now.minute()).c_str());
      strcat(SerialLink.BUFFER, ",");
      strcat(SerialLink.BUFFER, padDigitZero(dt_now.second()).c_str());
      strcat(SerialLink.BUFFER, ",");


      createChecksum(SerialLink.BUFFER);
      strcat(SerialLink.BUFFER, "*");
      strcat(SerialLink.BUFFER, SerialLink.checksum);
      strcat(SerialLink.BUFFER, "\n");

      Serial.println("[TXD] " + String(SerialLink.BUFFER));

      Serial1.write(SerialLink.BUFFER);
      Serial1.write(ETX);

      break;
    }
  }
}




void loop() {
# 678 "C:/Users/Benjamin/Documents/PlatformIO/Projects/241108-025923-megaatmega2560/src/satIOPortController.ino"
  readRXD1();


  if (MUX0_CHANNEL==0) {writeTXD1Data();}


  if (SerialLink.validation==true) {satIOPortController();}




  delay(1);
}