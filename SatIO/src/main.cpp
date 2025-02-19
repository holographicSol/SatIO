

/*

                                        SatIO - Written by Benjamin Jack Cullen.

                                                   "The GPS MAster"

                              A general purpose programmable satellite and inertial switch.

                                      SatIO is the system, a matrix is the program.

            Receives and Processes Transmissions from Satellites and makes the data available for calculations.

            Possible combinations example: 

  10=digit characters   15=lenght of double   3=doubles per function   10=functions per switch   20=switches  190=available functions
                                                (((10^15 * 3) * 10) * 20) ^ 190

          Currently there are over 200 different checks that can be performed using just several small primitive functions and
  currently each matrix activation/deactivaion can occur based on up to 10 different checks per 20 switches resulting true or false per switch. 

                                    
                                  Wiring For ESP32 keystudio dev module (with keystudio sheild V1.3)

                                        Serial Communication Between ESP32 & ATMEGA2560:
                                        ESP32 io25 (TXD) -> ATMEGA2560 Serial1 (RXD)
                                        ESP32 io26 (RXD) -> ATMEGA2560 Serial1 (TXD)

                                        Serial Communication Between ESP32 & WTGPS300P:
                                        ESP32 io27 (RXD) -> WTGPS300P (TXD) (5v)

                                        ESP32 i2C Multiplexing:
                                        ESP32 i2C          -> TCA9548A i2c Multiplexer: SDA, SCL (3.3v)
                                        TCA9548A SDA0 SCL0 -> RTC DS3231 SDA, SCL (5v)

                                        ESP32 Analog+Digital Multiplexing:
                                        ESP32 io4  -> CD74HC4067 Analog/Digital Multiplexer: SIG
                                        ESP32 io32 -> CD74HC4067 Analog/Digital Multiplexer: S0
                                        ESP32 io33 -> CD74HC4067 Analog/Digital Multiplexer: S1
                                        ESP32 io12 -> CD74HC4067 Analog/Digital Multiplexer: S2
                                        ESP32 io13 -> CD74HC4067 Analog/Digital Multiplexer: S3
                                        CD74HC4067 C0 -> Photo Resistor SIG
                                        CD74HC4067 C1 -> DHT11 SIG

                                        ESP32 SDCARD:
                                        ESP32 io5  -> HW-125 Micro SD Card Module CS (SS)
                                        ESP32 io23 -> HW-125 Micro SD Card Module DI (MOSI)
                                        ESP32 io19 -> HW-125 Micro SD Card Module DO (MISO)
                                        ESP32 io18 -> HW-125 Micro SD Card Module SCK (SCLK)

                                                          SENTENCE $SATIO
                                                                                  
                          START Tag                Last Sat Time                    Converted Longitude        
                            |                   |               |                   |               |                  
                          $SATIO,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                                |               |               |                 |                              
                                  DatetimeStamp                  Converted Latitude                                 


          Ultimately this system is being built as a unit to turn on/off IO/GPIO/relays and or send messages to other controllers,
                    where potentially anything can be plugged in such as simple modules or pre-programmed MCU's, 
              making a foundation for other creative projects that may make use of such satellite and or inertial data.
                  The idea is that each matrix is a compound of logic (limited by memory), and the logic itself
              is programmable before and after flashing. Allowing for a reusable and general purpose system for any future
                                              projects requiring use of GPS data. 
                                          Robots and flying machines and automation.

                                          The IO is virtualized as well as physical! 

              Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
                  of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)

                      Use case from a syncronized clock to riding the INS (roll, pitch, yaw) on a fine line to within a certain degree of
                      expected drift, if GPS data is stale or unavailable.

                        Note: This project is now temporarily headless while architecture & performance are the focus.
  */

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      LIBRARIES

#include <Arduino.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <iostream>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
// #include <SoftwareSerial.h>
#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>          // https://github.com/PaulStoffregen/Time
#include <Timezone.h>         // https://github.com/JChristensen/Timezone
// #include <TFT_eSPI.h>         // https://github.com/Bodmer/TFT_eSPI
// #include <XPT2046_Bitbang.h>  // https://github.com/ddxfish/XPT2046_Bitbang_Arduino_Library/
#include <SiderealPlanets.h>  // https://github.com/DavidArmstrong/SiderealPlanets
#include <SiderealObjects.h>  // https://github.com/DavidArmstrong/SiderealObjects
// #include <JPEGDecoder.h>
#include "esp_pm.h"
#include "esp_attr.h"
#include <DHT.h>
#include <CD74HC4067.h>

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                            RTC
RTC_DS3231 rtc;
DateTime dt_now;
DateTime rcv_dt_0;
DateTime rcv_dt_1;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SENSORS
#define DHTPIN 4 // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.
// Uncomment whatever type you're using!
#define DHTTYPE DHT11 // DHT 11
// #define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21 // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);
#define PHOTORESISTOR_0 4

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                           PINS
// #define INTERRUPT_ATMEGA_0 12 // pin to instruct atmega2560 to go into read mode
// #define INTERRUPT_ATMEGA_1 5  // pin to instruct atmega2560 to go into write mode
// const signed int portcontroller_mode[1][2] {{INTERRUPT_ATMEGA_0, INTERRUPT_ATMEGA_1}} ;
const int8_t ctsPin = -1;  // remap hardware serial TXD
const int8_t rtsPin = -1;  // remap hardware serial RXD
const byte txd_to_atmega = 25;  // 
const byte rxd_from_gps = 26;   // 

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          TASKS

TaskHandle_t TSTask;    // touchscreen task
TaskHandle_t UpdateDisplayTask;  // time task
TaskHandle_t Task0;  // time task
TaskHandle_t Task1;  // time task

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SIDEREAL PLANETS

SiderealPlanets myAstro;    // for calculating azimuth and altitude
SiderealObjects myAstroObj; // for getting right ascension and declination of objects from star table

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                         SDCARD

SPIClass sdspi = SPIClass(VSPI);

// Uncomment and set up if you want to use custom pins for the SPI communication
// #define REASSIGN_PINS
// int sck = 5;
// int miso = 12;
// int mosi = 13;
// int cs = 23;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                            ETX

#define ETX 0x03  // end of text character useful for parsing serial data

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: SYSTEM

struct systemStruct {
  bool overload = false;
  bool satio_enabled = true;           // enables/disables data extrapulation from existing GPS data (coordinate degrees, etc)
  bool gngga_enabled = true;           // enables/disables parsing of serial GPS data
  bool gnrmc_enabled = true;           // enables/disables parsing of serial GPS data
  bool gpatt_enabled = true;           // enables/disables parsing of serial GPS data
  bool matrix_enabled = false;         // enables/disables matrix switch
  bool run_on_startup = false;         // enables/disable matrix switch on startup as specified by system configuration file
  bool output_satio_enabled = false;   // enables/disables output SatIO sentence over serial
  bool output_gngga_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_gnrmc_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_gpatt_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_matrix_enabled = false;  // enables/disables output matrix switch active/inactive states sentence over serial
  bool port_controller_enabled = true; // may be false by default but is default true for now.

  bool sidereal_track_sun = true;       // enables/disables celestial body tracking
  bool sidereal_track_moon = true;      // enables/disables celestial body tracking
  bool sidereal_track_mercury = true;  // enables/disables celestial body tracking
  bool sidereal_track_venus = true;    // enables/disables celestial body tracking
  bool sidereal_track_mars = true;     // enables/disables celestial body tracking
  bool sidereal_track_jupiter = true;  // enables/disables celestial body tracking
  bool sidereal_track_saturn = true;   // enables/disables celestial body tracking
  bool sidereal_track_uranus = true;   // enables/disables celestial body tracking
  bool sidereal_track_neptune = true;  // enables/disables celestial body tracking
  
  bool display_auto_dim = true;               // enables/disables screen brightness to be automatically reduced
  int           display_auto_dim_p0 = 5000;   // time after last interaction screen brightness should be reduced
  unsigned long display_auto_dim_t0;          // value set and used by the system)
  unsigned long display_auto_dim_t1;          // value set and used by the system
  bool          display_dim_bool = false;     // value set and used by the system
  bool display_auto_off = false;              // enables/disables screen backlight turning off automatically
  int           display_auto_off_p0 = 10000;  // time after last interaction creen backlight should be turned off
  unsigned long display_auto_off_t0;          // value set and used by the system
  unsigned long display_auto_off_t1;          // value set and used by the system
  bool          display_off_bool = false;     // value set and used by the system
  uint32_t display_brightness = 255;          // value of current screen brightness (0-255)
  uint32_t display_autodim_brightness = 50;   // value of automatically reduced screen brightness (0-255)
  int index_display_autodim_times = 1;        // index of currently used time 
  int max_display_autodim_times = 6;          // max available times 
  int display_autodim_times[6][56] = {3000, 5000, 10000, 15000, 30000, 60000};  // available times
  int index_display_autooff_times = 2;                                          // index of currently used time 
  int max_display_autooff_times = 6;                                            // max available times 
  int display_autooff_times[6][56] = {3000, 5000, 10000, 15000, 30000, 60000};  // available times

  char translate_enable_bool[2][56] = {"DISABLED", "ENABLED"}; // bool used as index selects bool translation
  char translate_plus_minus[2][56]  = {"+", "-"};              // bool used as index selects bool translation
};
systemStruct systemData;


signed int yaw_min = 0;     signed int yaw_max = 360;
signed int pitch_min = -90; signed int pitch_max = 90;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: DEBUG

struct sysDebugStruct {
  bool gngga_sentence = false;    // enables/disables itemized sentence value output after processing
  bool gnrmc_sentence = false;    // enables/disables itemized sentence value output after processing
  bool gpatt_sentence = false;    // enables/disables itemized sentence value output after processing
  bool serial_0_sentence = true;  // enables/disables itemized command values output after processing
  bool validation = false;        // enables/disables data validation such as checksum, length and type checking
  bool verbose_file = true;       // provide more information about files being loaded/saved/etc.
};
sysDebugStruct sysDebugData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     DATA: MENU

struct menuStruct {
  char input[100];                 // char array input by user
  signed int numpad_key = -1;      // allows numpad to differentiate between values
  int page = 0;                    // currently displayed page
  int backpage = 0;                // page we would like to return to after current page
  int previous_page = 0;           // a page different from backpage, as should be distintly previous page
  int matrix_select = 0;           // index matrix switch
  int matrix_function_select = 0;  // index available functions for matrix switch
  int function_index = 0;          // index function for matrix switch
  int matrix_filenames_index = 0;  // index available matrix files
};
menuStruct menuData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 DATA: SERIAL 0

struct Serial0Struct {
  unsigned long nbytes;                // number of bytes read by serial
  unsigned long iter_token;            // count token iterations
  char BUFFER[2000];                   // serial buffer
  char * token = strtok(BUFFER, ",");  // token pointer 
  char data_0[56];                     // value placeholder
  char data_1[56];                     // value placeholder
  char data_2[56];                     // value placeholder
  char data_3[56];                     // value placeholder
  char data_4[56];                     // value placeholder
  char data_5[56];                     // value placeholder
  char data_6[56];                     // value placeholder
};
Serial0Struct serial0Data;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 DATA: SERIAL 1

struct Serial1Struct {
  unsigned long nbytes;                // number of bytes read by serial
  unsigned long iter_token;            // count token iterations
  char BUFFER[2000];                   // serial buffer
  char * token = strtok(BUFFER, ",");  // token pointer 
  int collected = 0;                   // counts how many unique sentences have been collected.
  bool gngga_bool = false;             // has sentence been collected
  bool gnrmc_bool = false;             // has sentence been collected
  bool gpatt_bool = false;             // has sentence been collected
  bool rtc_bool = false;
};
Serial1Struct serial1Data;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             SERIAL LINK STRUCT

struct SerialLinkStruct {
  int i_nbytes;
  long i_sync;
  char char_i_sync[56];
  bool syn = false;
  bool data = false;
  char BUFFER[2000];           // read incoming bytes into this buffer
  char BUFFER1[2000];               // buffer refined using ETX
  // char TMP[2000];               // buffer refined using ETX
  unsigned long nbytes;
  unsigned long TOKEN_i;
  // unsigned long T0_RXD_1 = 0;   // hard throttle current time
  // unsigned long T1_RXD_1 = 0;   // hard throttle previous time
  // unsigned long TT_RXD_1 = 0;   // hard throttle interval
  // unsigned long T0_TXD_1 = 0;   // hard throttle current time
  // unsigned long T1_TXD_1 = 0;   // hard throttle previous time
  // unsigned long TT_TXD_1 = 10;  // hard throttle interval
  int i_token = 0;
  char * token;
  bool validation = false;
  char checksum[56];
  uint8_t checksum_of_buffer;
  uint8_t checksum_in_buffer;
  char gotSum[4];
  int i_XOR;
  int XOR;
  int c_XOR;
};
SerialLinkStruct SerialLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: SDCARD

struct SDCardStruct {
  bool is_writing = false;
  bool is_reading = false;
  bool sdcard_mount_bool = false;
  bool sdcard_attached_bool = false;
  uint8_t card_type = CARD_NONE;
  char sdcard_types[1][4][56] = {
    "NONE",
    "MMC",
    "SD",
    "SDHC"
  };
  uint64_t card_size = 0;
  int max_matrix_filenames = 20;                               // max matrix file names available 
  char matrix_filenames[20][56] = {  
    "", "", "", "", "",
    "", "", "", "", "",
    "", "", "", "", "",
    "", "", "", "", "",
    };                                                         // matrix filenames created, stored and found by system
  char sysconf[56] = "/SYSTEM/SYSTEM.CONFIG";                  // filepath
  char default_matrix_filepath[56] = "/MATRIX/MATRIX_0.SAVE";  // filepath
  char matrix_filepath[56] = "";                               // current matrix filepath
  char tempmatrixfilepath[56];                                 // used for laoding filepaths
  char system_dirs[2][56] = {"/MATRIX", "/SYSTEM"};            // root dirs
  unsigned long nbytes;                                        // number of bytes read
  unsigned long iter_token;                                    // count token iterations
  char BUFFER[2048];                                           // buffer
  String SBUFFER;                                              // String buffer
  char * token = strtok(BUFFER, ",");                          // token pointer 
  char data_0[56];                                             // value placeholder
  char data_1[56];                                             // value placeholder
  char data_2[56];                                             // value placeholder
  char data_3[56];                                             // value placeholder
  char data_4[56];                                             // value placeholder
  char data_5[56];                                             // value placeholder
  char data_6[56];                                             // value placeholder
  char data_7[56];                                             // value placeholder
  char file_data[256];                                         // buffer
  char delim[2] = ",";                                         // delimiter char
  char tmp[256];                                               // buffer
  char tag_0[56] = "r";                                        // file line tag
  char tag_1[56] = "e";                                        // file line tag
  File current_file;                                           // file currently handled
  int initialization_interval = 5000;
  int sdcard_check_interval = 5000;
  long last_initialization_time = 0;
  long last_sdcard_check_time = 0;
  bool initialization_flag = false;
};
SDCardStruct sdcardData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     DATA: TIME

struct TimeStruct {
  double seconds;                      // seconds accumulated since startup
  unsigned long mainLoopTimeTaken;     // current main loop time
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


volatile int interrupt_second_counter;  //for counting interrupt
hw_timer_t * second_timer = NULL;      //H/W timer defining (Pointer to the Structure)
portMUX_TYPE second_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void isr_second_timer() {      //Defining Inerrupt function with for faster access
  portENTER_CRITICAL_ISR(&second_timer_mux);
  interrupt_second_counter++;
  timeData.seconds++;
  portEXIT_CRITICAL_ISR(&second_timer_mux);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               DATA: VALIDATION

struct validationStruct {
  int  valid_i = 0;           // validation counter
  bool valid_b = true;        // validation bool
  char *find_char;            // validation pointer
  int  index;                 // a placeholder for char index
  bool bool_data_0 = false;   // load matrix values validation bool
  bool bool_data_1 = false;   // load matrix values validation bool
};
validationStruct validData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 ANALOGUE WRITE

// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 4095 from 2 ^ 12 - 1
  uint32_t duty = (4095 / valueMax) * min(value, valueMax);
  // write duty to LEDC
  ledcWrite(channel, duty);
}

String SerialDisplayRTCDateTime() {
  return String(String(rtc.now().hour()) + ":" + String(rtc.now().minute()) + ":" + String(rtc.now().second()) +
  " " + String(rtc.now().day()) + "." + String(rtc.now().month()) + "." + String(rtc.now().year()));
}

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
  // Serial.println("[validateChecksum]");
  // Serial.println("[validateChecksum] " + String(buffer));

  memset(SerialLink.gotSum, 0, sizeof(SerialLink.gotSum));
  
  SerialLink.gotSum[0] = buffer[strlen(buffer) - 3];
  SerialLink.gotSum[1] = buffer[strlen(buffer) - 2];

  // Serial.print("[checksum_in_buffer] "); Serial.println(SerialLink.gotSum);

  SerialLink.checksum_of_buffer =  getCheckSum(buffer);
  // Serial.print("[checksum_of_buffer] "); Serial.println(SerialLink.checksum_of_buffer);
  // sprintf(SerialLink.checksum,"%X",SerialLink.checksum_of_buffer);
  // Serial.print("[checksum_of_buffer converted] "); Serial.println(SerialLink.checksum);
  SerialLink.checksum_in_buffer = h2d2(SerialLink.gotSum[0], SerialLink.gotSum[1]);
  // Serial.print("[checksum_in_buffer] "); Serial.println(SerialLink.checksum_in_buffer);

  if (SerialLink.checksum_in_buffer == SerialLink.checksum_of_buffer) {return true;}
  return false;
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

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               VALIDATION: DATA

/*
checks can be tuned and ellaborated upon individually.
each sentence has a checksum that is for checking if the payload is more or less intact, while in contrast checks below are for
sanitizing each element of a sentence. thorough testing is required to ensure no false negatives are encountered but its worth
the extra work, rather than assuming all elements will be what we expect every time.
*/


bool count_digits(char * data, int expected) {
  if (sysDebugData.validation == true) {Serial.println("[connected] count_digits: " + String(data));}
  validData.valid_i = 0;
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 1) {validData.valid_i++;}}
  if (validData.valid_i == expected) {return true;} else {return false;}
}

bool count_alpha(char * data, int expected) {
  if (sysDebugData.validation == true) {Serial.println("[connected] count_alpha: " + String(data));}
  validData.valid_i = 0;
  for (int i = 0; i < strlen(data); i++) {if (isalpha(data[i]) == 1) {validData.valid_i++;}}
  if (validData.valid_i == expected) {return true;} else {return false;}
}

bool is_all_digits(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] is_all_digits: " + String(data));}
  validData.valid_b = true;
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {validData.valid_b = false;}}
  return validData.valid_b;
}

bool is_all_digits_plus_char(char * data, char find_char) {
  /* designed to check all chars are digits except one period and is more general purpose than just accepting a period */
  if (sysDebugData.validation == true) {Serial.println("[connected] is_all_digits_plus_char: " + String(data));}
  validData.valid_b = true;
  validData.find_char = strchr(data, find_char);
  validData.index = (int)(validData.find_char - data);
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {if (i != validData.index) {validData.valid_b = false;}}}
  return validData.valid_b;
}

bool is_positive_negative_num(char * data) {

  /*
  designed to check all chars are digits except one period and the signed bit. allows positive/negative floats,
  doubles and ints.
  allows one period anywhere.
  allows one minus (-) sign at index zero.
  */

  if (sysDebugData.validation == true) {Serial.println("[connected] is_positive_negative_num: " + String(data));}
  validData.valid_b = true;
  validData.find_char = strchr(data, '.');
  validData.index = (int)(validData.find_char - data);
  for (int i = 0; i < strlen(data); i++) {
    if (isdigit(data[i]) == 0) {if (i != validData.index) {if ((data[i] != '-') && (i > 0)) {validData.valid_b = false;}}}}
  return validData.valid_b;
}

bool is_all_alpha(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] is_all_alpha: " + String(data));}
  validData.valid_b = true;
  for (int i = 0; i < strlen(data); i++) {if (isalpha(data[i]) == 0) {validData.valid_b = false;}}
  return validData.valid_b;
}

bool val_utc_time(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_utc_time: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 9) {
    if (data[6] == '.') {
      if (count_digits(data, 8) == true) {
        if ((atoi(data) >= 0.0) && (atoi(data) <= 235959.99)) {check_pass = true;}
      }
    }
  }
  return check_pass;
}

bool val_utc_date(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_utc_date: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 6) {
    if (is_all_digits(data) == true) {
      if ((atoi(data) >= 0.0) && (atoi(data) <= 999999)) {check_pass = true;}
    }
  }
  return check_pass;
}

bool val_latitude(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_latitude: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 13) {
    if (data[4] == '.') {
      if (count_digits(data, 12) == true) {
        if (is_positive_negative_num(data) == true) {
          check_pass = true;
        }
      }
    }
  }
  return check_pass;
}

bool val_longitude(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_longitude: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 14) {
    if (data[5] == '.') {
      if (count_digits(data, 13) == true) {
        if (is_positive_negative_num(data) == true) {
          check_pass = true;
        }
      }
    }
  }
  return check_pass;
}

bool val_latitude_H(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_latitude_H: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "N") == 0) || (strcmp(data, "S") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_longitude_H(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_longitude_H: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "E") == 0) || (strcmp(data, "W") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_positioning_status_gngga(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_positioning_status_gngga: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if (is_all_digits(data) == true) {
      if ((atoi(data) >= 0) && (atoi(data) <= 6)) {
        check_pass = true;
      }
    }
  }
  return check_pass;
}

bool val_satellite_count(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_satellite_count: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) >= 0){
      check_pass = true;
      }
  }
  return check_pass;
}

bool val_hdop_precision_factor(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_hdop_precision_factor: " + String(data));}
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if (atoi(data) >= 0){
      check_pass = true;
  }
  }
  return check_pass;
}

bool val_altitude(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_altitude: " + String(data));}
  // account for decimal point
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
      check_pass = true;
  }
  return check_pass;
}

bool val_altitude_units(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_altitude_units: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if (strcmp(data, "M") == 0) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_geoidal(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_geoidal: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_geoidal_units(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_geoidal_units: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if (strcmp(data, "M") == 0) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_differential_delay(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_differential_delay: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_basestation_id(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_basestation_id: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (strlen(data) == 4) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_positioning_status_gnrmc(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_positioning_status_gnrmc: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "A") == 0) || (strcmp(data, "V") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_ground_speed(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ground_speed: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_ground_heading(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ground_heading: " + String(data));}
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 360)) {
      check_pass = true;
    }
  }
  return check_pass;
}

// todo
bool val_installation_angle(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_installation_angle: " + String(data));}
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if (atoi(data) >= 0) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_installation_angle_direction(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_installation_angle_direction: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "E") == 0) || (strcmp(data, "W") == 0) || (strcmp(data, "M") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_mode_indication(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_mode_indication: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "A") == 0) || (strcmp(data, "D") == 0) || (strcmp(data, "E") == 0) || (strcmp(data, "N") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_pitch_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_pitch_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_roll_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_roll_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_yaw_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_yaw_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_angle_channle_p_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_angle_channle_p_gpatt: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "p") == 0) {check_pass = true;}
  return check_pass;
}

bool val_angle_channle_r_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_angle_channle_r_gpatt: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "r") == 0) {check_pass = true;}
  return check_pass;
}

bool val_angle_channle_y_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_angle_channle_y_gpatt: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "y") == 0) {check_pass = true;}
  return check_pass;
}

bool val_version_channel_s_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_version_channel_s_gpatt: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "S") == 0) {check_pass = true;}
  return check_pass;
}

bool val_software_version_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_software_version_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) == 20230219) {check_pass = true;}
  }
  return check_pass;
}

bool val_product_id_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_product_id_gpatt: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "003E009") == 0) {check_pass = true;}
  return check_pass;
}

bool val_id_channel_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_id_channel_gpatt: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "ID") == 0) {check_pass = true;}
  return check_pass;
}

bool val_ins_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ins_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_channel_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ins_channel_gpatt: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "INS") == 0) {check_pass = true;}
  return check_pass;
}

bool val_hardware_version_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_hardware_version_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (strcmp(data, "3335") == 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_run_state_flag_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_run_state_flag_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((strcmp(data, "01") == 0) || (strcmp(data, "02") == 0) || (strcmp(data, "03") == 0)) {check_pass = true;}
  }
  return check_pass;
}

// todo
bool val_mis_angle_num_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_mis_angle_num_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_static_flag_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_static_flag_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

// todo
bool val_user_code_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_user_code_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_gst_data_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_gst_data_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_line_flag_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_line_flag_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_mis_att_flag_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_mis_att_flag_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_imu_kind_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_imu_kind_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_car_kind_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_car_kind_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 1) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_mileage_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_mileage_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_run_inetial_flag_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_run_inetial_flag_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_speed_enable_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_speed_enable_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_speed_num_gpatt(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_speed_num_gpatt: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_speed_status(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_speed_status: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  }
  return check_pass;
}

bool val_accelleration_delimiter(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_accelleration_delimiter: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "A") == 0) {check_pass = true;}
  return check_pass;
}

bool val_axis_accelleration(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_axis_accelleration: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_angular_velocity_delimiter(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_angular_velocity_delimiter: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "G") == 0) {check_pass = true;}
  return check_pass;
}

bool val_gyro_angular_velocity(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_gyro_angular_velocity: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_status_delimiter(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_status_delimiter: " + String(data));}
  bool check_pass = false;
  if (strcmp(data, "S") == 0) {check_pass = true;}
  return check_pass;
}

bool val_ubi_state_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_state_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_state_kind_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_state_kind_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_code_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_code_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_gset_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_gset_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_sset_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_sset_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ang_dget_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ang_dget_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_run_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ins_run_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_fix_kind_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_fix_kind_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_fiobject_roll_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_fiobject_roll_flag: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_fix_pitch_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_fix_pitch_flag: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_ubi_on_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_on_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_kind_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_kind_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_a_set(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_a_set: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_b_set(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_b_set: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  }
  return check_pass;
}

bool val_acc_X_data(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_acc_X_data: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_acc_Y_data(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_acc_Y_data: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_gyro_Z_data(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_gyro_Z_data: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_pitch_angle(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_pitch_angle: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_roll_angle(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_roll_angle: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_yaw_angle(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_yaw_angle: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_car_speed(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_car_speed: " + String(data));}
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 100)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ins_flag: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_num(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_num: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_ubi_valid(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_ubi_valid: " + String(data));}
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_coll_T_data(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_coll_T_data: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_coll_T_heading(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_coll_T_heading: " + String(data));}
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_custom_flag(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_custom_flag: " + String(data));}
  bool check_pass = false;
  if (strlen(data) >= 1) {check_pass = true;}
  return check_pass;
}

bool val_checksum(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_checksum: " + String(data));}
  bool check_pass = false;
  if (strlen(data) == 3) {check_pass = true;}
  return check_pass;
}

bool val_scalable(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] val_scalable: " + String(data));}
  bool check_pass = false;
  if (strlen(data) >= 1) {check_pass = true;}
  return check_pass;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: MATRIX

struct MatrixStruct {

  int max_matrices = 20;          // number of matrix switches 
  int max_matrix_functions = 10;  // number of functions available to a matrix switch

  int matrix_enabled_i = 0;       // count how many matrx switches are enabled
  int matrix_disabled_i = 0;      // count how many matrx switches are disabled
  int matrix_active_i = 0;        // count how many matrx switches are active
  int matrix_inactive_i = 0;      // count how many matrx switches are inactive

  char temp[2000];                     // a general place to store temporary chars relative to MatrixStruct
  char matrix_sentence[2000];  // an NMEA inspired sentence reflecting matrix switch states  
  char checksum_str[56];               // placeholder for char checksum relative to MatrixStruct
  char checksum[56];                      // placeholder for int checksum relative to MatrixStruct

  // reflects matrix switch active/inactive states each loop of matrix switch function
  bool matrix_switch_state[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };

  // reflects matrix switch enabled/disabled
  int matrix_switch_enabled[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };

  // a placeholder for timings when timer functions are selected for a matrix switch
  unsigned long matrix_timers[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };

  // a placeholder for matrix switch ports (default no port)
  signed int matrix_port_map[1][20] = {
    {
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    }
  };

  // a placeholder for matrix switch ports (default ATMEGA2560 digital)
  // signed int matrix_port_map[1][20] = {
  //   {
  //     23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
  //     33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
  //   }
  // };

  // a matrix max_matrices by max_matrix_functions storing function names for each matrix switch (default $NONE)
  char matrix_function[20][10][100] = {
    {"$SecondsTimer", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 1
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 2
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 3
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 4
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 5
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 6
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 7
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 8
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 9
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 10
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 11
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 12
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 13
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 14
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 15
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 16
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 17
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 18
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 19
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 20
     },
    };

  /*
  a matrix max_matrices by max_matrix_functions storing function values for each matrix switch
          0     1     2     
          X     Y     Z    
  {  {   0.0,  0.0,  0.0   } }
  */
  double matrix_function_xyz[20][10][3] = {
    {
      {2.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 1
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 2
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 3
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 4
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 5
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 6
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 7
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 8
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 9
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 10
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 11
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 12
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 13
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 14
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 15
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 16
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 17
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 18
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 19
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
    {
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 20
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}
    },
  };

  // number of available function names that can be used to program a matrix switch
  int max_matrix_function_names = 226;
  // number of available function names that can be used to program a matrix switch (keep strlen() <=23)
  char matrix_function_names[226][25] = 
  {
    "$NONE",
    "$ENABLED",
    "$OVERLOAD_TRUE",
    "$OVERLOAD_FALSE",
    "$SWITCHLINKTRUE",
    "$SWITCHLINKFALSE",
    "SecondsTimer",
    "RTCTimeOver",
    "RTCTimeUnder",
    "RTCTimeEqual",
    "RTCTimeRange",
    "DaySunday",
    "DayMonday",
    "DayTuesday",
    "DayWednesday",
    "DayThursday",
    "DayFriday",
    "DaySaturday",
    "DateDayX",
    "DateMonthX",
    "DateYearX",
    "DegreesLatGNGGAOver",
    "DegreesLonGNGGAOver",
    "DegreesLatGNGGAUnder",
    "DegreesLonGNGGAUnder",
    "DegreesLatGNGGAEqual",
    "DegreesLonGNGGAEqual",
    "DegreesLonGNGGARange",
    "DegreesLatGNGGARange",
    "DegreesGNGGARanges",
    "DegreesLatGNRMCOver",
    "DegreesLonGNRMCOver",
    "DegreesLatGNRMCUnder",
    "DegreesLonGNRMCUnder",
    "DegreesLatGNRMCEqual",
    "DegreesLonGNRMCEqual",
    "DegreesLatGNRMCRange",
    "DegreesLonGNRMCRange",
    "DegreesGNRMCRanges",
    "UTCTimeGNGGAOver",
    "UTCTimeGNGGAUnder",
    "UTCTimeGNGGAEqual",
    "UTCTimeGNGGARange",
    "LatGNGGAOver",
    "LonGNGGAOver",
    "LatGNGGAUnder",
    "LonGNGGAUnder",
    "LatGNGGAEqual",
    "LonGNGGAEqual",
    "LatGNGGARange",
    "LonGNGGARange",
    "PositioningStatusGNGGA",
    "SatelliteCountOver",
    "SatelliteCountUnder",
    "SatelliteCountEqual",
    "SatelliteCountRange",
    "HemisphereGNGGANorth",
    "HemisphereGNGGASouth",
    "HemisphereGNGGAEast",
    "HemisphereGNGGAWest",
    "GPSPrecisionOver",
    "GPSPrecisionUnder",
    "GPSPrecisionEqual",
    "GPSPrecisionRange",
    "AltitudeGNGGAOver",
    "AltitudeGNGGAUnder",
    "AltitudeGNGGAEqual",
    "AltitudeGNGGARange",
    "UTCTimeGNRMCOver",
    "UTCTimeGNRMCUnder",
    "UTCTimeGNRMCEqual",
    "UTCTimeGNRMCRange",
    "PositioningStatusGNRMCA",
    "PositioningStatusGNRMCV",
    "ModeGNRMCA",
    "ModeGNRMCD",
    "ModeGNRMCE",
    "ModeGNRMCN",
    "LatGNRMCOver",
    "LonGNRMCOver",
    "LatGNRMCUnder",
    "LonGNRMCUnder",
    "LonGNRMCEqual",
    "LatGNRMCEqual",
    "LatGNRMCRange",
    "LonGNRMCRange",
    "HemisphereGNRMCNorth",
    "HemisphereGNRMCSouth",
    "HemisphereGNRMCEast",
    "HemisphereGNRMCWest",
    "GroundSpeedGNRMCOver",
    "GroundSpeedGNRMCUnder",
    "GroundSpeedGNRMCEqual",
    "GroundSpeedGNRMCRange",
    "HeadingGNRMCOver",
    "HeadingGNRMCUnder",
    "HeadingGNRMCEqual",
    "HeadingGNRMCRange",
    "UTCDateGNRMCOver",
    "UTCDateGNRMCUnder",
    "UTCDateGNRMCEqual",
    "UTCDateGNRMCRange",
    "LineFlagGPATTEqual",
    "StaticFlagGPATTEqual",
    "RunStateFlagGPATTEqual",
    "INSGPATTEqual",
    "SpeedNumGPATTOver",
    "SpeedNumGPATTUnder",
    "SpeedNumGPATTEqual",
    "SpeedNumGPATTRange",
    "MileageGPATTOver",
    "MileageGPATTUnder",
    "MileageGPATTEqual",
    "MileageGPATTRange",
    "GSTDataGPATTOver",
    "GSTDataGPATTUnder",
    "GSTDataGPATTEqual",
    "GSTDataGPATTRange",
    "YawGPATTOver",
    "YawGPATTUnder",
    "YawGPATTEqual",
    "YawGPATTRange",
    "RollGPATTOver",
    "RollGPATTUnder",
    "RollGPATTEqual",
    "RollGPATTRange",
    "PitchGPATTOver",
    "PitchGPATTUnder",
    "PitchGPATTEqual",
    "PitchGPATTRange",
    "GNGGAValidChecksum",
    "GNRMCValidChecksum",
    "GPATTValidChecksum",
    "GNGGAInvalidChecksum",
    "GNRMCInvalidChecksum",
    "GPATTInvalidChecksum",
    "GNGGAValidCheckData",
    "GNRMCValidCheckData",
    "GPATTValidCheckData",
    "GNGGAInvalidCheckData",
    "GNRMCInvalidCheckData",
    "GPATTInvalidCheckData",
    "SunAzimuthRange",
    "SunAltitudeRange",
    "DayTime",
    "NightTime",
    "Sunrise",
    "Sunset",
    "MoonAzimuthRange",
    "MoonAltitudeRange",
    "MoonUp",
    "MoonDown",
    "Moonrise",
    "Moonset",
    "MoonPhase",
    "MercuryAzimuthRange",
    "MercuryAltitudeRange",
    "MercuryUp",
    "MercuryDown",
    "MercuryRise",
    "MercurySet",
    "VenusAzimuthRange",
    "VenusAltitudeRange",
    "VenusUp",
    "VenusDown",
    "VenusRise",
    "VenusSet",
    "MarsAzimuthRange",
    "MarsAltitudeRange",
    "MarsUp",
    "MarsDown",
    "MarsRise",
    "MarsSet",
    "JupiterAzimuthRange",
    "JupiterAltitudeRange",
    "JupiterUp",
    "JupiterDown",
    "JupiterRise",
    "JupiterSet",
    "SaturnAzimuthRange",
    "SaturnAltitudeRange",
    "SaturnUp",
    "SaturnDown",
    "SaturnRise",
    "SaturnSet",
    "UranusAzimuthRange",
    "UranusAltitudeRange",
    "UranusUp",
    "UranusDown",
    "UranusRise",
    "UranusSet",
    "NeptuneAzimuthRange",
    "NeptuneAltitudeRange",
    "NeptuneUp",
    "NeptuneDown",
    "NeptuneRise",
    "NeptuneSet",
    "DHT11_0_H_Under",
    "DHT11_0_H_Over",
    "DHT11_0_H_Equal",
    "DHT11_0_H_Range",
    "DHT11_0_C_Under",
    "DHT11_0_C_Over",
    "DHT11_0_C_Equal",
    "DHT11_0_C_Range",
    "DHT11_0_F_Under",
    "DHT11_0_F_Over",
    "DHT11_0_F_Equal",
    "DHT11_0_F_Range",
    "DHT11_0_HIC_Under",
    "DHT11_0_HIC_Over",
    "DHT11_0_HIC_Equal",
    "DHT11_0_HIC_Range",
    "DHT11_0_HIF_Under",
    "DHT11_0_HIF_Over",
    "DHT11_0_HIF_Equal",
    "DHT11_0_HIF_Range",
    "PhotoResistor_0_Under",
    "PhotoResistor_0_Over",
    "PhotoResistor_0_Equal",
    "PhotoResistor_0_Range",
    "Tracking_0_Under",
    "Tracking_0_Over",
    "Tracking_0_Equal",
    "Tracking_0_Range"
  };

  /* false if first or all functions $NONE. true if preceeding functions are populated. */
  char default_matrix_function[25]         = "$NONE";

  char default_enable_matrix_function[25]  = "$ENABLED";  // always true.

  char OVERLOAD_TRUE[25] = "OVERLOAD_TRUE";
  char OVERLOAD_FALSE[25] = "OVERLOAD_FALSE";

  /* link matrix switch to another matrix switch (standard). specify x (matrix switch number 0-19) in matrix. */
  char SwitchLinkTrue[25]                 = "$SWITCHLINKTRUE";

  /* link matrix switch to another matrix switch (inverted). specify x (matrix switch number 0-19) in matrix. */
  char SwitchLinkFalse[25]                = "$SWITCHLINKFALSE";
  
  char SecondsTimer[25] = "SecondsTimer";  // specify x (seconds) in matrix.

  char RTCTimeOver[25]            = "RTCTimeOver";             // specify x (ddmmyyhhmmss.ms) in matrix.
  char RTCTimeUnder[25]      = "RTCTimeUnder";       // specify x (ddmmyyhhmmss.ms) in matrix.
  char RTCTimeEqual[25]      = "RTCTimeEqual";       // specify x (ddmmyyhhmmss.ms) in matrix.
  char RTCTimeRange[25]      = "RTCTimeRange";       // specify x (ddmmyyhhmmss.ms) y (ddmmyyhhmmss.ms in matrix.

  char DaySunday[25]    = "DaySunday";     // true for day. takes not further arguments.
  char DayMonday[25]    = "DayMonday";     // true for day. takes not further arguments.
  char DayTuesday[25]   = "DayTuesday";    // true for day. takes not further arguments.
  char DayWednesday[25] = "DayWednesday";  // true for day. takes not further arguments.
  char DayThursday[25]  = "DayThursday";   // true for day. takes not further arguments.
  char DayFriday[25]    = "DayFriday";     // true for day. takes not further arguments. 
  char DaySaturday[25]  = "DaySaturday";   // true for day. takes not further arguments.

  char DateDayX[25]     = "DateDayX";      // specify x in matrix. example: 1 for 1st of the month
  char DateMonthX[25]   = "DateMonthX";    // specify x in matrix. example: 1 for 1st month of the year
  char DateYearX[25]    = "DateYearX";     // specify x in matrix. example: 2030 for year 2030.

  // do local time

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   SATIO DATA

  char DegreesLatGNGGAOver[25]   = "DegreesLatGNGGAOver";   // specify x (degrees lat) in matrix.
  char DegreesLatGNGGAUnder[25]  = "DegreesLatGNGGAUnder";  // specify x (degrees lat) in matrix.
  char DegreesLatGNGGAEqual[25]  = "DegreesLatGNGGAEqual";  // specify x (degrees lat) in matrix.
  char DegreesLonGNGGAOver[25]   = "DegreesLonGNGGAOver";   // specify x (degrees lon) in matrix.
  char DegreesLonGNGGAUnder[25]  = "DegreesLonGNGGAUnder";  // specify x (degrees lon) in matrix.
  char DegreesLonGNGGAEqual[25]  = "DegreesLonGNGGAEqual";  // specify x (degrees lon) in matrix.
  char DegreesLatGNGGARange[25]  = "DegreesLatGNGGARange";  // specify x (degrees lat) z (meters range) in matrix.
  char DegreesLonGNGGARange[25]  = "DegreesLonGNGGARange";  // specify x (degrees lon) z (meters range) in matrix.
  char DegreesGNGGARanges[25]    = "DegreesGNGGARanges";    // specify x (degrees lat) y (degrees lon) z (meters range) in matrix.

  char DegreesLatGNRMCOver[25]   = "DegreesLatGNRMCOver";   // specify x (degrees lat) in matrix.
  char DegreesLatGNRMCUnder[25]  = "DegreesLatGNRMCUnder";  // specify x (degrees lat) in matrix.
  char DegreesLatGNRMCEqual[25]  = "DegreesLatGNRMCEqual";  // specify x (degrees lat) in matrix.
  char DegreesLonGNRMCOver[25]   = "DegreesLonGNRMCOver";   // specify x (degrees lon) in matrix.
  char DegreesLonGNRMCUnder[25]  = "DegreesLonGNRMCUnder";  // specify x (degrees lon) in matrix.
  char DegreesLonGNRMCEqual[25]  = "DegreesLonGNRMCEqual";  // specify x (degrees lon) in matrix.
  char DegreesLatGNRMCRange[25]  = "DegreesLatGNRMCRange";  // specify x (degrees lat) z (meters range) in matrix.
  char DegreesLonGNRMCRange[25]  = "DegreesLonGNRMCRange";  // specify x (degrees lon) z (meters range) in matrix.
  char DegreesGNRMCRanges[25]    = "DegreesGNRMCRanges";    // specify x (degrees lat) y (degrees lon) z (meters range) in matrix.

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNGGA DATA

  char UTCTimeGNGGAOver[25]       = "UTCTimeGNGGAOver";        // specify x (ddmmyyhhmmss.ms) in matrix.
  char UTCTimeGNGGAUnder[25]      = "UTCTimeGNGGAUnder";       // specify x (ddmmyyhhmmss.ms) in matrix.
  char UTCTimeGNGGAEqual[25]      = "UTCTimeGNGGAEqual";       // specify x (ddmmyyhhmmss.ms) in matrix.
  char UTCTimeGNGGARange[25]      = "UTCTimeGNGGARange";       // specify x (ddmmyyhhmmss.ms) y (ddmmyyhhmmss.ms in matrix.

  char LatGNGGAOver[25]           = "LatGNGGAOver";            // specify x (absolute lat) in matrix.
  char LonGNGGAOver[25]           = "LonGNGGAOver";            // specify x (absolute lon) in matrix.
  char LatGNGGAUnder[25]          = "LatGNGGAUnder";           // specify x (absolute lat) in matrix.
  char LonGNGGAUnder[25]          = "LonGNGGAUnder";           // specify x (absolute lon) in matrix.
  char LatGNGGAEqual[25]          = "LatGNGGAEqual";           // specify x (absolute lat) in matrix.
  char LonGNGGAEqual[25]          = "LonGNGGAEqual";           // specify x (absolute lon) in matrix.
  char LatGNGGARange[25]          = "LatGNGGARange";           // specify x (absolute lat) z (meters range) in matrix.
  char LonGNGGARange[25]          = "LonGNGGARange";           // specify x (absolute lon) z (meters range) in matrix.
  
  /*
  specify x in matrix.
  0 : invalid solution; 1 : Single point positioning solution; 2 : Pseudorange difference; 6: Pure inertial navigation solution
  */
  char PositioningStatusGNGGA[25] = "PositioningStatusGNGGA";
  
  char SatelliteCountOver[25]     = "SatelliteCountOver";      // specify x (satellite number 0+) in matrix.
  char SatelliteCountUnder[25]    = "SatelliteCountUnder";     // specify x (satellite number 0+) in matrix.
  char SatelliteCountEqual[25]    = "SatelliteCountEqual";     // specify x (satellite number 0+) in matrix.
  char SatelliteCountRange[25]    = "SatelliteCountRange";     // specify x (satellite number 0+) in matrix.

  char HemisphereGNGGANorth[25]   = "HemisphereGNGGANorth";    // takes no further arguments.
  char HemisphereGNGGAEast[25]    = "HemisphereGNGGAEast";     // takes no further arguments.
  char HemisphereGNGGASouth[25]   = "HemisphereGNGGASouth";    // takes no further arguments.
  char HemisphereGNGGAWest[25]    = "HemisphereGNGGAWest";     // takes no further arguments.

  char GPSPrecisionOver[25]       = "GPSPrecisionOver";        // specify x (meters) in matrix.
  char GPSPrecisionUnder[25]      = "GPSPrecisionUnder";       // specify x (meters) in matrix.
  char GPSPrecisionEqual[25]      = "GPSPrecisionEqual";       // specify x (meters) in matrix.
  char GPSPrecisionRange[25]      = "GPSPrecisionRange";       // specify x (meters) y (meters) in matrix.

  char AltitudeGNGGAOver[25]      = "AltitudeGNGGAOver";       // specify x (meters) in matrix.
  char AltitudeGNGGAUnder[25]     = "AltitudeGNGGAUnder";      // specify x (meters) in matrix.
  char AltitudeGNGGAEqual[25]     = "AltitudeGNGGAEqual";      // specify x (meters) in matrix.
  char AltitudeGNGGARange[25]     = "AltitudeGNGGARange";      // specify x (meters) y (meters) in matrix.

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNRMC DATA

  char UTCTimeGNRMCOver[25]         = "UTCTimeGNRMCOver";         // specify x (hhmmss.ms) in matrix.
  char UTCTimeGNRMCUnder[25]        = "UTCTimeGNRMCUnder";        // specify x (hhmmss.ms) in matrix.
  char UTCTimeGNRMCEqual[25]        = "UTCTimeGNRMCEqual";        // specify x (hhmmss.ms) in matrix.
  char UTCTimeGNRMCRange[25]        = "UTCTimeGNRMCRange";        // specify x (hhmmss.ms) y (ddmmyyhhmmss.ms in matrix.
  
  char PositioningStatusGNRMCA[25]  = "PositioningStatusGNRMCA";  // A = valid positioning. takes no further arguments.
  char PositioningStatusGNRMCV[25]  = "PositioningStatusGNRMCV";  // V = invalid positioning. takes no further arguments.
  
  char ModeGNRMCA[25]               = "ModeGNRMCA";               // A = autonomous positioning. takes no further arguments.
  char ModeGNRMCD[25]               = "ModeGNRMCD";               // D = differential. takes no further arguments.
  char ModeGNRMCE[25]               = "ModeGNRMCE";               // E = estimation. takes no further arguments.
  char ModeGNRMCN[25]               = "ModeGNRMCN";               // N = invalid data. takes no further arguments.
  
  char LatGNRMCOver[25]             = "LatGNRMCOver";             // specify x (absolute lat) in matrix.
  char LonGNRMCOver[25]             = "LonGNRMCOver";             // specify x (absolute lon) in matrix.
  char LatGNRMCUnder[25]            = "LatGNRMCUnder";            // specify x (absolute lat) in matrix.
  char LonGNRMCUnder[25]            = "LonGNRMCUnder";            // specify x (absolute lon) in matrix.
  char LatGNRMCEqual[25]            = "LatGNRMCEqual";            // specify x (absolute lat) in matrix.
  char LonGNRMCEqual[25]            = "LonGNRMCEqual";            // specify x (absolute lon) in matrix.
  char LatGNRMCRange[25]            = "LatGNRMCRange";            // specify x (absolute lat) z (meters range) in matrix.
  char LonGNRMCRange[25]            = "LonGNRMCRange";            // specify x (absolute lon) z (meters range) in matrix.

  char HemisphereGNRMCNorth[25]     = "HemisphereGNRMCNorth";     // takes no further arguments.
  char HemisphereGNRMCEast[25]      = "HemisphereGNRMCEast";      // takes no further arguments.
  char HemisphereGNRMCSouth[25]     = "HemisphereGNRMCSouth";     // takes no further arguments.
  char HemisphereGNRMCWest[25]      = "HemisphereGNRMCWest";      // takes no further arguments.

  char GroundSpeedGNRMCOver[25]     = "GroundSpeedGNRMCOver";     // specify x (kilometers/h) in matrix.
  char GroundSpeedGNRMCUnder[25]    = "GroundSpeedGNRMCUnder";    // specify x (kilometers/h) in matrix.
  char GroundSpeedGNRMCEqual[25]    = "GroundSpeedGNRMCEqual";    // specify x (kilometers/h) in matrix.
  char GroundSpeedGNRMCRange[25]    = "GroundSpeedGNRMCRange";    // specify x (kilometers/h) y (kilometers/h) in matrix.

  char HeadingGNRMCOver[25]         = "HeadingGNRMCOver";         // specify x (degrees: 0-360) in matrix.
  char HeadingGNRMCUnder[25]        = "HeadingGNRMCUnder";        // specify x (degrees: 0-360) in matrix.
  char HeadingGNRMCEqual[25]        = "HeadingGNRMCEqual";        // specify x (degrees: 0-360) in matrix.
  char HeadingGNRMCRange[25]        = "HeadingGNRMCRange";        // specify x (degrees: 0-360) y (degrees: 0-360) in matrix.

  char UTCDateGNRMCOver[25]         = "UTCDateGNRMCOver";         // specify x (ddmmyy) in matrix.
  char UTCDateGNRMCUnder[25]        = "UTCDateGNRMCUnder";        // specify x (ddmmyy) in matrix.
  char UTCDateGNRMCEqual[25]        = "UTCDateGNRMCEqual";        // specify x (ddmmyy) in matrix.
  char UTCDateGNRMCRange[25]        = "UTCDateGNRMCRange";        // specify x (ddmmyy) y (ddmmyy) in matrix.

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GPATT DATA

  /* specify x (0-1) in matrix. 1 : straight driving, 0: turning driving */
  char LineFlagGPATTEqual[25]     = "LineFlagGPATTEqual";

  /* specify x (0-1) in matrix. 1 : static, 0 : dynamic */
  char StaticFlagGPATTEqual[25]   = "StaticFlagGPATTEqual";

  /*
  // specify x (flag) in matrix. 0: Prepare for initialization.
                                 1: INS converged. Inertial navigation can be started. flag: 01/02
                                 2: Initial convergence of Inertial Navigation. Inertial navigation can be started. flag: 03/04
                                 3: Inertial Navigation is converging. Inertial navigation can be started. flag: 03/04
                                 4. Inertial Navigation convergence complete. Inertial navigation can be started. flag: 03/04
  */
  char RunStateFlagGPATTEqual[25] = "RunStateFlagGPATTEqual";

  char INSGPATTEqual[25]          = "INSGPATTEqual";        // specify x (0-1) in matrix. 1 : On, 0 : Off

  /* specify x (0-99) in matrix. add one each time, return to zero after reaching 99 */
  char SpeedNumGPATTOver[25]      = "SpeedNumGPATTOver";

  /* specify x (0-99) in matrix. add one each time, return to zero after reaching 99 */
  char SpeedNumGPATTUnder[25]     = "SpeedNumGPATTUnder[";

  /* specify x (0-99) in matrix. add one each time, return to zero after reaching 99 */
  char SpeedNumGPATTEqual[25]     = "SpeedNumGPATTEqual";

  /* specify x (0-99) y (0-99) in matrix. add one each time, return to zero after reaching 99 */
  char SpeedNumGPATTRange[25]     = "SpeedNumGPATTRange";

  char MileageGPATTOver[25]       = "MileageGPATTOver";     // specify x (mileage) in matrix.
  char MileageGPATTUnder[25]      = "MileageGPATTUnder[";   // specify x (mileage) in matrix.
  char MileageGPATTEqual[25]      = "MileageGPATTEqual";    // specify x (mileage) in matrix.
  char MileageGPATTRange[25]      = "MileageGPATTRange";    // specify x (mileage) y (mileage) in matrix.

  char GSTDataGPATTOver[25]       = "GSTDataGPATTOver";     // specify x (GST data) in matrix.
  char GSTDataGPATTUnder[25]      = "GSTDataGPATTUnder[";   // specify x (GST data) in matrix.
  char GSTDataGPATTEqual[25]      = "GSTDataGPATTEqual";    // specify x (GST data) in matrix.
  char GSTDataGPATTRange[25]      = "GSTDataGPATTRange";    // specify x (GST data) y (GST data) in matrix.

  char YawGPATTOver[25]           = "YawGPATTOver";         // specify x (yaw -90 -> 90)) in matrix.
  char YawGPATTUnder[25]          = "YawGPATTUnder[";       // specify x (yaw -90 -> 90) in matrix.
  char YawGPATTEqual[25]          = "YawGPATTEqual";        // specify x (yaw -90 -> 90) in matrix.
  char YawGPATTRange[25]          = "YawGPATTRange";        // specify x (yaw -90 -> 90) y (yaw 0-180) in matrix.

  char RollGPATTOver[25]          = "RollGPATTOver";        // specify x (roll -90 -> 90) in matrix.
  char RollGPATTUnder[25]         = "RollGPATTUnder[";      // specify x (roll -90 -> 90) in matrix.
  char RollGPATTEqual[25]         = "RollGPATTEqual";       // specify x (roll -90 -> 90) in matrix.
  char RollGPATTRange[25]         = "RollGPATTRange";       // specify x (roll -90 -> 90) y (roll -90 -> 90) in matrix.

  char PitchGPATTOver[25]         = "PitchGPATTOver";       // specify x (pitch -90 -> 90) in matrix.
  char PitchGPATTUnder[25]        = "PitchGPATTUnder[";     // specify x (pitch -90 -> 90) in matrix.
  char PitchGPATTEqual[25]        = "PitchGPATTEqual";      // specify x (pitch -90 -> 90) in matrix.
  char PitchGPATTRange[25]        = "PitchGPATTRange";      // specify x (pitch -90 -> 90) y (pitch -90 -> 90) in matrix.

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                             SIDEREAL PLANETS

  char SunAzimuthRange[25]  = "SunAzimuthRange";           // specify x (0-360) y (0-360) in matrix.
  char SunAltitudeRange[25] = "SunAltitudeRange";          // specify x (0-90) y (0-90) in matrix.
  char DayTime[25]   = "DayTime";                          // takes no further arguments.
  char NightTime[25] = "NightTime";                        // takes no further arguments.
  char Sunrise[25]   = "Sunrise";                          // takes no further arguments.
  char Sunset[25]    = "Sunset";                           // takes no further arguments.

  char MoonAzimuthRange[25]  = "MoonAzimuthRange";         // specify x (0-360) y (0-360) in matrix.
  char MoonAltitudeRange[25] = "MoonAltitudeRange";        // specify x (0-90) y (0-90) in matrix.
  char MoonUp[25]    = "MoonUp";                           // takes no further arguments.
  char MoonDown[25]  = "MoonDown";                         // takes no further arguments.
  char Moonrise[25]  = "Moonrise";                         // takes no further arguments.
  char Moonset[25]   = "Moonset";                          // takes no further arguments.
  char MoonPhase[25] = "MoonPhase";                        // takes no further arguments.

  char MercuryAzimuthRange[25]  = "MercuryAzimuthRange";   // specify x (0-360) y (0-360) in matrix.
  char MercuryAltitudeRange[25] = "MercuryAltitudeRange";  // specify x (0-90) y (0-90) in matrix.
  char MercuryUp[25]    = "MercuryUp";                     // takes no further arguments.
  char MercuryDown[25]  = "MercuryDown";                   // takes no further arguments.
  char MercuryRise[25]  = "MercuryRise";                   // takes no further arguments.
  char MercurySet[25]   = "MercurySet";                    // takes no further arguments.

  char VenusAzimuthRange[25]  = "VenusAzimuthRange";       // specify x (0-360) y (0-360) in matrix.
  char VenusAltitudeRange[25] = "VenusAltitudeRange";      // specify x (0-90) y (0-90) in matrix.
  char VenusUp[25]    = "VenusUp";                         // takes no further arguments.
  char VenusDown[25]  = "VenusDown";                       // takes no further arguments.
  char VenusRise[25]  = "VenusRise";                       // takes no further arguments.
  char VenusSet[25]   = "VenusSet";                        // takes no further arguments.

  char MarsAzimuthRange[25]  = "MarsAzimuthRange";         // specify x (0-360) y (0-360) in matrix.
  char MarsAltitudeRange[25] = "MarsAltitudeRange";        // specify x (0-90) y (0-90) in matrix.
  char MarsUp[25]    = "MarsUp";                           // takes no further arguments.
  char MarsDown[25]  = "MarsDown";                         // takes no further arguments.
  char MarsRise[25]  = "MarsRise";                         // takes no further arguments.       
  char MarsSet[25]   = "MarsSet";                          // takes no further arguments.

  char JupiterAzimuthRange[25]  = "JupiterAzimuthRange";   // specify x (0-360) y (0-360) in matrix.
  char JupiterAltitudeRange[25] = "JupiterAltitudeRange";  // specify x (0-90) y (0-90) in matrix.
  char JupiterUp[25]    = "JupiterUp";                     // takes no further arguments.
  char JupiterDown[25]  = "JupiterDown";                   // takes no further arguments.
  char JupiterRise[25]  = "JupiterRise";                   // takes no further arguments.
  char JupiterSet[25]   = "JupiterSet";                    // takes no further arguments.

  char SaturnAzimuthRange[25]  = "SaturnAzimuthRange";     // specify x (0-360) y (0-360) in matrix.
  char SaturnAltitudeRange[25] = "SaturnAltitudeRange";    // specify x (0-90) y (0-90) in matrix.
  char SaturnUp[25]    = "SaturnUp";                       // takes no further arguments.
  char SaturnDown[25]  = "SaturnDown";                     // takes no further arguments.
  char SaturnRise[25]  = "SaturnRise";                     // takes no further arguments.
  char SaturnSet[25]   = "SaturnSet";                      // takes no further arguments.

  char UranusAzimuthRange[25]  = "UranusAzimuthRange";     // specify x (0-360) y (0-360) in matrix.
  char UranusAltitudeRange[25] = "UranusAltitudeRange";    // specify x (0-90) y (0-90) in matrix.
  char UranusUp[25]    = "UranusUp";                       // takes no further arguments.
  char UranusDown[25]  = "UranusDown";                     // takes no further arguments.
  char UranusRise[25]  = "UranusRise";                     // takes no further arguments.
  char UranusSet[25]   = "UranusSet";                      // takes no further arguments.

  char NeptuneAzimuthRange[25]  = "NeptuneAzimuthRange";   // specify x (0-360) y (0-360) in matrix.
  char NeptuneAltitudeRange[25] = "NeptuneAltitudeRange";  // specify x (0-90) y (0-90) in matrix.
  char NeptuneUp[25]    = "NeptuneUp";                     // takes no further arguments.
  char NeptuneDown[25]  = "NeptuneDown";                   // takes no further arguments.
  char NeptuneRise[25]  = "NeptuneRise";                   // takes no further arguments.
  char NeptuneSet[25]   = "NeptuneSet";                    // takes no further arguments.

  char DHT11_0_H_Under[25] = "DHT11_0_H_Under";
  char DHT11_0_H_Over[25] = "DHT11_0_H_Over";
  char DHT11_0_H_Equal[25] = "DHT11_0_H_Equal";
  char DHT11_0_H_Range[25] = "DHT11_0_H_Range";

  char DHT11_0_C_Under[25] = "DHT11_0_C_Under";
  char DHT11_0_C_Over[25] = "DHT11_0_C_Over";
  char DHT11_0_C_Equal[25] = "DHT11_0_C_Equal";
  char DHT11_0_C_Range[25] = "DHT11_0_C_Range";

  char DHT11_0_F_Under[25] = "DHT11_0_F_Under";
  char DHT11_0_F_Over[25] = "DHT11_0_F_Over";
  char DHT11_0_F_Equal[25] = "DHT11_0_F_Equal";
  char DHT11_0_F_Range[25] = "DHT11_0_F_Range";

  char DHT11_0_HIC_Under[25] = "DHT11_0_HIC_Under";
  char DHT11_0_HIC_Over[25] = "DHT11_0_HIC_Over";
  char DHT11_0_HIC_Equal[25] = "DHT11_0_HIC_Equal";
  char DHT11_0_HIC_Range[25] = "DHT11_0_HIC_Range";

  char DHT11_0_HIF_Under[25] = "DHT11_0_HIF_Under";
  char DHT11_0_HIF_Over[25] = "DHT11_0_HIF_Over";
  char DHT11_0_HIF_Equal[25] = "DHT11_0_HIF_Equal";
  char DHT11_0_HIF_Range[25] = "DHT11_0_HIF_Range";

  
  char PhotoResistor_0_Under[25] = "PhotoResistor_0_Under";
  char PhotoResistor_0_Over[25] = "PhotoResistor_0_Over";
  char PhotoResistor_0_Equal[25] = "PhotoResistor_0_Equal";
  char PhotoResistor_0_Range[25] = "PhotoResistor_0_Range";

  char Tracking_0_Under[25] = "Tracking_0_Under";
  char Tracking_0_Over[25] = "Tracking_0_Over";
  char Tracking_0_Equal[25] = "Tracking_0_Equal";
  char Tracking_0_Range[25] = "Tracking_0_Range";
  
  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                VALIDITY DATA

  char GNGGAValidChecksum[56] = "GNGGAValidChecksum";        // returns true or false. takes no further arguments.
  char GNRMCValidChecksum[56] = "GNRMCValidChecksum";        // returns true or false. takes no further arguments.
  char GPATTValidChecksum[56] = "GPATTValidChecksum";        // returns true or false. takes no further arguments.
  char GNGGAInvalidChecksum[56] = "GNGGAInvalidChecksum";    // returns true or false. takes no further arguments.
  char GNRMCInvalidChecksum[56] = "GNRMCInvalidChecksum";    // returns true or false. takes no further arguments.
  char GPATTInvalidChecksum[56] = "GPATTInvalidChecksum";    // returns true or false. takes no further arguments.

  char GNGGAValidCheckData[56] = "GNGGAValidCheckData";      // returns true or false. takes no further arguments.
  char GNRMCValidCheckData[56] = "GNRMCValidCheckData";      // returns true or false. takes no further arguments.
  char GPATTValidCheckData[56] = "GPATTValidCheckData";      // returns true or false. takes no further arguments.
  char GNGGAInvalidCheckData[56] = "GNGGAInvalidCheckData";  // returns true or false. takes no further arguments.
  char GNRMCInvalidCheckData[56] = "GNRMCInvalidCheckData";  // returns true or false. takes no further arguments.
  char GPATTInvalidCheckData[56] = "GPATTInvalidCheckData";  // returns true or false. takes no further arguments.
};
MatrixStruct matrixData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: GNGGA

struct GNGGAStruct {
  char sentence[2000];
  char tag[56];                                                                                                            // <0> Log header
  char utc_time[56];                    unsigned long bad_utc_time_i;              bool bad_utc_time = true;               // <1> UTC time, the format is hhmmss.sss
  char latitude[56];                    unsigned long bad_latitude_i;              bool bad_latitude = true;               // <2> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];         unsigned long bad_latitude_hemisphere_i;   bool bad_latitude_hemisphere = true;    // <3> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                   unsigned long bad_longitude_i;             bool bad_longitude = true;              // <4> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];        unsigned long bad_longitude_hemisphere_i;  bool bad_longitude_hemisphere = true;   // <5> Longitude hemisphere, E or W (east longitude or west longitude)
  char solution_status[56];             unsigned long bad_solution_status_i;       bool bad_solution_status = true;            // <6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2: pseudorange difference, 6: pure INS */
  char satellite_count_gngga[56] = "0"; unsigned long bad_satellite_count_gngga_i; bool bad_satellite_count_gngga = true;  // <7> Number of satellites used
  char hdop_precision_factor[56];       unsigned long bad_hdop_precision_factor_i; bool bad_hdop_precision_factor = true;  // <8> HDOP level precision factor
  char altitude[56];                    unsigned long bad_altitude_i;              bool bad_altitude = true;               // <9> Altitude
  char altitude_units[56];              unsigned long bad_altitude_units_i;        bool bad_altitude_units = true;         // <10> 
  char geoidal[56];                     unsigned long bad_geoidal_i;               bool bad_geoidal = true;                // <11> The height of the earth ellipsoid relative to the geoid 
  char geoidal_units[56];               unsigned long bad_geoidal_units_i;         bool bad_geoidal_units = true;          // <12> 
  char differential_delay[56];          unsigned long bad_differential_delay_i;    bool bad_differential_delay = true;     // <13>
  char id[56];                          unsigned long bad_id_i;                    bool bad_id = true;                     // <14> base station ID
  char check_sum[56];                   unsigned long bad_check_sum_i;             bool bad_check_sum = true;              // <15> XOR check value of all bytes starting from $ to *
  int check_data = 0;                   unsigned long bad_checksum_validity;       bool valid_checksum = false;            // Checksum validity bool, counters and a counter for how many elements passed further testing (gngga check_data should result in 16)
  char temporary_data[56];
  char temporary_data_1[56];
};
GNGGAStruct gnggaData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          GNGGA

void GNGGA() {
  gnggaData.check_data = 0;
  memset(gnggaData.tag, 0, 56);
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(gnggaData.sentence, ",");
  while( serial1Data.token != NULL ) {
    if     (serial1Data.iter_token == 0)                                                                {strcpy(gnggaData.tag, "GNGGA");                                                                             gnggaData.check_data++;}
    else if (serial1Data.iter_token ==1)  {if (val_utc_time(serial1Data.token) == true)                 {memset(gnggaData.utc_time, 0, 56);              strcpy(gnggaData.utc_time, serial1Data.token);              gnggaData.check_data++; gnggaData.bad_utc_time = false;}              else {gnggaData.bad_utc_time_i++;              gnggaData.bad_utc_time = true;}}
    else if (serial1Data.iter_token ==2)  {if (val_latitude(serial1Data.token) == true)                 {memset(gnggaData.latitude, 0, 56);              strcpy(gnggaData.latitude, serial1Data.token);              gnggaData.check_data++; gnggaData.bad_latitude = false;}              else {gnggaData.bad_latitude_i++;              gnggaData.bad_latitude = true;}}
    else if (serial1Data.iter_token ==3)  {if (val_latitude_H(serial1Data.token) == true)               {memset(gnggaData.latitude_hemisphere, 0, 56);   strcpy(gnggaData.latitude_hemisphere, serial1Data.token);   gnggaData.check_data++; gnggaData.bad_latitude_hemisphere = false;}   else {gnggaData.bad_latitude_hemisphere_i++;   gnggaData.bad_latitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==4)  {if (val_longitude(serial1Data.token) == true)                {memset(gnggaData.longitude, 0, 56);             strcpy(gnggaData.longitude, serial1Data.token);             gnggaData.check_data++; gnggaData.bad_longitude = false;}             else {gnggaData.bad_longitude_i++;             gnggaData.bad_longitude = true;}}
    else if (serial1Data.iter_token ==5)  {if (val_longitude_H(serial1Data.token) == true)              {memset(gnggaData.longitude_hemisphere, 0, 56);  strcpy(gnggaData.longitude_hemisphere, serial1Data.token);  gnggaData.check_data++; gnggaData.bad_longitude_hemisphere = false;}  else {gnggaData.bad_longitude_hemisphere_i++;  gnggaData.bad_longitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==6)  {if (val_positioning_status_gngga(serial1Data.token) == true) {memset(gnggaData.solution_status, 0, 56);       strcpy(gnggaData.solution_status, serial1Data.token);       gnggaData.check_data++; gnggaData.bad_solution_status = false;}       else {gnggaData.bad_solution_status_i++;       gnggaData.bad_solution_status = true;}}
    else if (serial1Data.iter_token ==7)  {if (val_satellite_count(serial1Data.token) == true)          {memset(gnggaData.satellite_count_gngga, 0, 56); strcpy(gnggaData.satellite_count_gngga, serial1Data.token); gnggaData.check_data++; gnggaData.bad_satellite_count_gngga = false;} else {gnggaData.bad_satellite_count_gngga_i++; gnggaData.bad_satellite_count_gngga = true;}}
    else if (serial1Data.iter_token ==8)  {if (val_hdop_precision_factor(serial1Data.token) == true)    {memset(gnggaData.hdop_precision_factor, 0, 56); strcpy(gnggaData.hdop_precision_factor, serial1Data.token); gnggaData.check_data++; gnggaData.bad_hdop_precision_factor = false;} else {gnggaData.bad_hdop_precision_factor_i++; gnggaData.bad_hdop_precision_factor = true;}}
    else if (serial1Data.iter_token ==9)  {if (val_altitude(serial1Data.token) == true)                 {memset(gnggaData.altitude, 0, 56);              strcpy(gnggaData.altitude, serial1Data.token);              gnggaData.check_data++; gnggaData.bad_altitude = false;}              else {gnggaData.bad_altitude_i++;              gnggaData.bad_altitude = true;}}
    else if (serial1Data.iter_token ==10) {if (val_altitude_units(serial1Data.token) == true)           {memset(gnggaData.altitude_units, 0, 56);        strcpy(gnggaData.altitude_units, serial1Data.token);        gnggaData.check_data++; gnggaData.bad_altitude_units = false;}        else {gnggaData.bad_altitude_units_i++;        gnggaData.bad_altitude_units = true;}}
    else if (serial1Data.iter_token ==11) {if (val_geoidal(serial1Data.token) == true)                  {memset(gnggaData.geoidal, 0, 56);               strcpy(gnggaData.geoidal, serial1Data.token);               gnggaData.check_data++; gnggaData.bad_geoidal = false;}               else {gnggaData.bad_geoidal_i++;               gnggaData.bad_geoidal = true;}}
    else if (serial1Data.iter_token ==12) {if (val_geoidal_units(serial1Data.token) == true)            {memset(gnggaData.geoidal_units, 0, 56);         strcpy(gnggaData.geoidal_units, serial1Data.token);         gnggaData.check_data++; gnggaData.bad_geoidal_units = false;}         else {gnggaData.bad_geoidal_units_i++;         gnggaData.bad_geoidal_units = true;}}
    else if (serial1Data.iter_token ==13) {if (val_differential_delay(serial1Data.token) == true)       {memset(gnggaData.differential_delay, 0, 56);    strcpy(gnggaData.differential_delay, serial1Data.token);    gnggaData.check_data++; gnggaData.bad_differential_delay = false;}    else {gnggaData.bad_differential_delay_i++;    gnggaData.bad_differential_delay = true;}}
    // else if (serial1Data.iter_token ==14) {
    //   memset(gnggaData.temporary_data, 0, 56);
    //   strcpy(gnggaData.temporary_data, strtok(serial1Data.token, "*"));
    //   if (val_basestation_id(gnggaData.temporary_data) == true) {memset(gnggaData.id, 0, 56); strcpy(gnggaData.id, gnggaData.temporary_data); gnggaData.check_data++; gnggaData.bad_id = false;} else {gnggaData.bad_id_i++; gnggaData.bad_id = true;}
    //   serial1Data.token = strtok(NULL, "*");
    //   memset(gnggaData.temporary_data_1, 0, 56);
    //   strcpy(gnggaData.temporary_data_1, strtok(serial1Data.token, "*"));
    //   if (val_checksum(gnggaData.temporary_data_1) == true) {memset(gnggaData.check_sum, 0, 56); strcpy(gnggaData.check_sum, gnggaData.temporary_data_1); gnggaData.check_data++; gnggaData.bad_check_sum = false;} else {gnggaData.bad_check_sum_i++; gnggaData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
  }
  if (sysDebugData.gngga_sentence == true) {
    Serial.println("[gnggaData.tag] "                     + String(gnggaData.tag));
    Serial.println("[gnggaData.utc_time] "                + String(gnggaData.utc_time));
    Serial.println("[gnggaData.latitude] "                + String(gnggaData.latitude));
    Serial.println("[gnggaData.latitude_hemisphere] "     + String(gnggaData.latitude_hemisphere));
    Serial.println("[gnggaData.longitude] "               + String(gnggaData.longitude));
    Serial.println("[gnggaData.longitude_hemisphere] "    + String(gnggaData.longitude_hemisphere));
    Serial.println("[gnggaData.solution_status] "      + String(gnggaData.solution_status));
    Serial.println("[gnggaData.satellite_count_gngga] "   + String(gnggaData.satellite_count_gngga));
    Serial.println("[gnggaData.hdop_precision_factor] "   + String(gnggaData.hdop_precision_factor));
    Serial.println("[gnggaData.altitude] "                + String(gnggaData.altitude));
    Serial.println("[gnggaData.altitude_units] "          + String(gnggaData.altitude_units));
    Serial.println("[gnggaData.geoidal] "                 + String(gnggaData.geoidal));
    Serial.println("[gnggaData.geoidal_units] "           + String(gnggaData.geoidal_units));
    Serial.println("[gnggaData.differential_delay] "      + String(gnggaData.differential_delay));
    Serial.println("[gnggaData.id] "                      + String(gnggaData.id));
    Serial.println("[gnggaData.check_sum] "               + String(gnggaData.check_sum));
    Serial.println("[gnggaData.check_data] "              + String(gnggaData.check_data));
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: GNRMC

struct GNRMCStruct {
  char sentence[2000];
  char tag[56];                                                                                                                             // <0> Log header
  char utc_time[56];                       unsigned long bad_utc_time_i;                     bool bad_utc_time = true;                      // <1> UTC time, the format is hhmmss.sss
  char positioning_status[56];             unsigned long bad_positioning_status_i;           bool bad_positioning_status = true;            // <2> Positioning status, A=effective positioning, V=invalid positioning
  char latitude[56];                       unsigned long bad_latitude_i;                     bool bad_latitude = true;                      // <3> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];            unsigned long bad_latitude_hemisphere_i;          bool bad_latitude_hemisphere = true;           // <4> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                      unsigned long bad_longitude_i;                    bool bad_longitude = true;                     // <5> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];           unsigned long bad_longitude_hemisphere_i;         bool bad_longitude_hemisphere = true;          // <6> Longitude hemisphere, E or W (east longitude or west longitude)
  char ground_speed[56];                   unsigned long bad_ground_speed_i;                 bool bad_ground_speed = true;                  // <7> Ground speed
  char ground_heading[56];                 unsigned long bad_ground_heading_i;               bool bad_ground_heading = true;                // <8> Ground heading (take true north as the reference datum)
  char utc_date[56];                       unsigned long bad_utc_date_i;                     bool bad_utc_date = true;                      // <9> UTC date, the format is ddmmyy (day, month, year)
  char installation_angle[56];             unsigned long bad_installation_angle_i;           bool bad_installation_angle = true;            // <10> Magnetic declination (000.0~180.0 degrees)
  char installation_angle_direction[56];   unsigned long bad_installation_angle_direction_i; bool bad_installation_angle_direction = true;  // <11> Magnetic declination direction, E (east) or W (west)
  char mode_indication[56];                unsigned long bad_mode_indication_i;              bool bad_mode_indication = true;               // <12> Mode indication (A=autonomous positioning, D=differential E=estimation, N=invalid data) */
  char check_sum[56];                      unsigned long bad_check_sum_i;                    bool bad_check_sum = true;                     // <13> XOR check value of all bytes starting from $ to *
  int check_data = 0;                      unsigned long bad_checksum_validity;              bool valid_checksum = false;                   // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 14)
  char temporary_data[56];
  char temporary_data_1[56];
};
GNRMCStruct gnrmcData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          GNRMC

void GNRMC() {
  gnrmcData.check_data = 0;
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(gnrmcData.sentence, ",");
  while( serial1Data.token != NULL ) {
    if      (serial1Data.iter_token == 0)                                                                   {strcpy(gnrmcData.tag, "GNRMC");                                                                                           gnrmcData.check_data++;}
    else if (serial1Data.iter_token ==1)  {if (val_utc_time(serial1Data.token) == true)                     {memset(gnrmcData.utc_time, 0, 56);                     strcpy(gnrmcData.utc_time, serial1Data.token);                     gnrmcData.check_data++; gnrmcData.bad_utc_time = false;}                     else {gnrmcData.bad_utc_time_i++;                     gnrmcData.bad_utc_time = true;}}
    else if (serial1Data.iter_token ==2)  {if (val_positioning_status_gnrmc(serial1Data.token) == true)     {memset(gnrmcData.positioning_status, 0, 56);           strcpy(gnrmcData.positioning_status, serial1Data.token);           gnrmcData.check_data++; gnrmcData.bad_positioning_status = false;}           else {gnrmcData.bad_positioning_status_i++;           gnrmcData.bad_positioning_status = true;}}
    else if (serial1Data.iter_token ==3)  {if (val_latitude(serial1Data.token) == true)                     {memset(gnrmcData.latitude, 0, 56);                     strcpy(gnrmcData.latitude, serial1Data.token);                     gnrmcData.check_data++; gnrmcData.bad_latitude = false;}                     else {gnrmcData.bad_latitude_i++;                     gnrmcData.bad_latitude = true;}}
    else if (serial1Data.iter_token ==4)  {if (val_latitude_H(serial1Data.token) == true)                   {memset(gnrmcData.latitude_hemisphere, 0, 56);          strcpy(gnrmcData.latitude_hemisphere, serial1Data.token);          gnrmcData.check_data++; gnrmcData.bad_latitude_hemisphere = false;}          else {gnrmcData.bad_latitude_hemisphere_i++;          gnrmcData.bad_latitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==5)  {if (val_longitude(serial1Data.token) == true)                    {memset(gnrmcData.longitude, 0, 56);                    strcpy(gnrmcData.longitude, serial1Data.token);                    gnrmcData.check_data++; gnrmcData.bad_longitude = false;}                    else {gnrmcData.bad_longitude_i++;                    gnrmcData.bad_longitude = true;}}
    else if (serial1Data.iter_token ==6)  {if (val_longitude_H(serial1Data.token) == true)                  {memset(gnrmcData.longitude_hemisphere, 0, 56);         strcpy(gnrmcData.longitude_hemisphere, serial1Data.token);         gnrmcData.check_data++; gnrmcData.bad_longitude_hemisphere = false;}         else {gnrmcData.bad_longitude_hemisphere_i++;         gnrmcData.bad_longitude_hemisphere = true;}}
    else if (serial1Data.iter_token ==7)  {if (val_ground_speed(serial1Data.token) == true)                 {memset(gnrmcData.ground_speed, 0, 56);                 strcpy(gnrmcData.ground_speed, serial1Data.token);                 gnrmcData.check_data++; gnrmcData.bad_ground_speed = false;}                 else {gnrmcData.bad_ground_speed_i++;                 gnrmcData.bad_ground_speed = true;}}
    else if (serial1Data.iter_token ==8)  {if (val_ground_heading(serial1Data.token) == true)               {memset(gnrmcData.ground_heading, 0, 56);               strcpy(gnrmcData.ground_heading, serial1Data.token);               gnrmcData.check_data++; gnrmcData.bad_ground_heading = false;}               else {gnrmcData.bad_ground_heading_i++;               gnrmcData.bad_ground_heading = true;}}
    else if (serial1Data.iter_token ==9)  {if (val_utc_date(serial1Data.token) == true)                     {memset(gnrmcData.utc_date, 0, 56);                     strcpy(gnrmcData.utc_date, serial1Data.token);                     gnrmcData.check_data++; gnrmcData.bad_utc_date = false;}                     else {gnrmcData.bad_utc_date_i++;                     gnrmcData.bad_utc_date = true;}}
    else if (serial1Data.iter_token ==10) {if (val_installation_angle(serial1Data.token) == true)           {memset(gnrmcData.installation_angle, 0, 56);           strcpy(gnrmcData.installation_angle, serial1Data.token);           gnrmcData.check_data++; gnrmcData.bad_installation_angle = false;}           else {gnrmcData.bad_installation_angle_i++;           gnrmcData.bad_installation_angle = true;}}
    else if (serial1Data.iter_token ==11) {if (val_installation_angle_direction(serial1Data.token) == true) {memset(gnrmcData.installation_angle_direction, 0, 56); strcpy(gnrmcData.installation_angle_direction, serial1Data.token); gnrmcData.check_data++; gnrmcData.bad_installation_angle_direction = false;} else {gnrmcData.bad_installation_angle_direction_i++; gnrmcData.bad_installation_angle_direction = true;}}
    // else if (serial1Data.iter_token ==12) {
    //   memset(gnggaData.temporary_data, 0, 56);
    //   strcpy(gnrmcData.temporary_data, strtok(serial1Data.token, "*"));
    //   if (val_mode_indication(gnrmcData.temporary_data) == true) {memset(gnrmcData.mode_indication, 0, 56); strcpy(gnrmcData.mode_indication, gnrmcData.temporary_data); gnrmcData.check_data++; gnrmcData.bad_mode_indication = false;} else {gnrmcData.bad_mode_indication_i++; gnrmcData.bad_mode_indication = true;}
    //   serial1Data.token = strtok(NULL, "*");
    //   memset(gnggaData.temporary_data_1, 0, 56);
    //   strcpy(gnrmcData.temporary_data_1, strtok(serial1Data.token, "*"));
    //   if (val_checksum(gnrmcData.temporary_data_1) == true) {memset(gnrmcData.check_sum, 0, 56); strcpy(gnrmcData.check_sum, gnrmcData.temporary_data_1); gnrmcData.check_data++; gnrmcData.bad_check_sum = false;} else {gnrmcData.bad_check_sum_i++; gnrmcData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
  }
  if (sysDebugData.gnrmc_sentence == true) {
    Serial.println("[gnrmcData.tag] "                          + String(gnrmcData.tag));
    Serial.println("[gnrmcData.utc_time] "                     + String(gnrmcData.utc_time));
    Serial.println("[gnrmcData.positioning_status] "           + String(gnrmcData.positioning_status));
    Serial.println("[gnrmcData.latitude] "                     + String(gnrmcData.latitude));
    Serial.println("[gnrmcData.latitude_hemisphere] "          + String(gnrmcData.latitude_hemisphere));
    Serial.println("[gnrmcData.longitude] "                    + String(gnrmcData.longitude));
    Serial.println("[gnrmcData.longitude_hemisphere] "         + String(gnrmcData.longitude_hemisphere));
    Serial.println("[gnrmcData.ground_speed] "                 + String(gnrmcData.ground_speed));
    Serial.println("[gnrmcData.ground_heading] "               + String(gnrmcData.ground_heading));
    Serial.println("[gnrmcData.utc_date] "                     + String(gnrmcData.utc_date));
    Serial.println("[gnrmcData.installation_angle] "           + String(gnrmcData.installation_angle));
    Serial.println("[gnrmcData.installation_angle_direction] " + String(gnrmcData.installation_angle_direction));
    Serial.println("[gnrmcData.mode_indication] "              + String(gnrmcData.mode_indication));
    Serial.println("[gnrmcData.check_sum] "                    + String(gnrmcData.check_sum));
    Serial.println("[gnrmcData.check_data] "                   + String(gnrmcData.check_data));
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: GPATT

struct GPATTStruct {
  char sentence[2000];
  char tag[56];                                                                                       // <0> Log header
  char pitch[56];            unsigned long bad_pitch_i;            bool bad_pitch = true;             // <1> pitch angle
  char angle_channel_0[56];  unsigned long bad_angle_channel_0_i;  bool bad_angle_channel_0 = true;   // <2> P
  char roll[56];             unsigned long bad_roll_i;             bool bad_roll = true;              // <3> Roll angle
  char angle_channel_1[56];  unsigned long bad_angle_channel_1_i;  bool bad_angle_channel_1 = true;   // <4> R
  char yaw[56];              unsigned long bad_yaw_i;              bool bad_yaw = true;               // <5> Yaw angle
  char angle_channel_2[56];  unsigned long bad_angle_channel_2_i;  bool bad_angle_channel_2 = true;   // <6> Y
  char software_version[56]; unsigned long bad_software_version_i; bool bad_software_version = true;  // <7> software verion
  char version_channel[56];  unsigned long bad_version_channel_i;  bool bad_version_channel = true;   // <8> S
  char product_id[56];       unsigned long bad_product_id_i;       bool bad_product_id = true;        // <9> Product ID: 96 bit unique ID
  char id_channel[56];       unsigned long bad_id_channel_i;       bool bad_id_channel = true;        // <10> ID 
  char ins[56];              unsigned long bad_ins_i;              bool bad_ins = true;               // <11> INS Default open inertial navigation system
  char ins_channel[56];      unsigned long bad_ins_channel_i;      bool bad_ins_channel = true;       // <12> whether inertial navigation open
  char hardware_version[56]; unsigned long bad_hardware_version_i; bool bad_hardware_version = true;  // <13> Named after master chip
  char run_state_flag[56];   unsigned long bad_run_state_flag_i;   bool bad_run_state_flag = true;    // <14> Algorithm status flag: 1->3
  char mis_angle_num[56];    unsigned long bad_mis_angle_num_i;    bool bad_mis_angle_num = true;     // <15> number of Installation
  char custom_logo_0[56];    unsigned long bad_custom_logo_0_i;    bool bad_custom_logo_0 = true;     // <16>
  char custom_logo_1[56];    unsigned long bad_custom_logo_1_i;    bool bad_custom_logo_1 = true;     // <17>
  char custom_logo_2[56];    unsigned long bad_custom_logo_2_i;    bool bad_custom_logo_2 = true;     // <18>
  char static_flag[56];      unsigned long bad_static_flag_i;      bool bad_static_flag = true;       // <19> 1:Static 0：dynamic
  char user_code[56];        unsigned long bad_user_code_i;        bool bad_user_code = true;         // <20> 1：Normal user X：Customuser
  char gst_data[56];         unsigned long bad_gst_data_i;         bool bad_gst_data = true;          // <21> User satellite accuracy
  char line_flag[56];        unsigned long bad_line_flag_i;        bool bad_line_flag = true;         // <22> 1：straight driving，0：curve driving
  char custom_logo_3[56];    unsigned long bad_custom_logo_3_i;    bool bad_custom_logo_3 = true;     // <23>
  char mis_att_flag[56];     unsigned long bad_mis_att_flag_i;     bool bad_mis_att_flag = true;      // <24> 
  char imu_kind[56];         unsigned long bad_imu_kind_i;         bool bad_imu_kind = true;          // <25> Sensor Type: 0->BIms055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW 4->ICM-40607; 5->ICM-40608 6->ICM-42670; 7->LSM6DSR
  char ubi_car_kind[56];     unsigned long bad_ubi_car_kind_i;     bool bad_ubi_car_kind = true;      // <26> 1: small car, 2: big car
  char mileage[56];          unsigned long bad_mileage_i;          bool bad_mileage = true;           // <27> kilometers: max 9999 kilometers
  char custom_logo_4[56];    unsigned long bad_custom_logo_4_i;    bool bad_custom_logo_4 = true;     // <28>
  char custom_logo_5[56];    unsigned long bad_custom_logo_5_i;    bool bad_custom_logo_5 = true;     // <29>
  char run_inetial_flag[56]; unsigned long bad_run_inetial_flag_i; bool bad_run_inetial_flag = true;  // <30> 1->4
  char custom_logo_6[56];    unsigned long bad_custom_logo_6_i;    bool bad_custom_logo_6 = true;     // <31>
  char custom_logo_7[56];    unsigned long bad_custom_logo_7_i;    bool bad_custom_logo_7 = true;     // <32>
  char custom_logo_8[56];    unsigned long bad_custom_logo_8_i;    bool bad_custom_logo_8 = true;     // <33>
  char custom_logo_9[56];    unsigned long bad_custom_logo_9_i;    bool bad_custom_logo_9 = true;     // <34>
  char speed_enable[56];     unsigned long bad_speed_enable_i;     bool bad_speed_enable = true;      // <35> 
  char custom_logo_10[56];   unsigned long bad_custom_logo_10_i;   bool bad_custom_logo_10 = true;    // <36>
  char custom_logo_11[56];   unsigned long bad_custom_logo_11_i;   bool bad_custom_logo_11 = true;    // <37>
  char speed_num[56];        unsigned long bad_speed_num_i;        bool bad_speed_num = true;         // <38> 1：fixed setting，0：Self adaptive installation
  char scalable[56];         unsigned long bad_scalable_i;         bool bad_scalable = true;          // <39> 
  char check_sum[56];        unsigned long bad_check_sum_i;        bool bad_check_sum = true;         // <40> XOR check value of all bytes starting from $ to *
  int check_data = 0;        unsigned long bad_checksum_validity;  bool valid_checksum = false;       // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 41)
  char temporary_data[56];
  char temporary_data_1[56];
};
GPATTStruct gpattData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          GPATT

void GPATT() {
  gpattData.check_data = 0;
  serial1Data.iter_token = 0;
  serial1Data.token = strtok(gpattData.sentence, ",");
  while( serial1Data.token != NULL ) { 
    if      (serial1Data.iter_token == 0)                                                              {strcpy(gpattData.tag, "GPATT");                                                                   gpattData.check_data++;}
    else if (serial1Data.iter_token == 1) {if (val_pitch_gpatt(serial1Data.token) == true)             {memset(gpattData.pitch, 0, 56); strcpy(gpattData.pitch, serial1Data.token);                       gpattData.check_data++; gpattData.bad_pitch = false;}            else {gpattData.bad_pitch_i++;            gpattData.bad_pitch = true;}}
    else if (serial1Data.iter_token == 2) {if (val_angle_channle_p_gpatt(serial1Data.token) == true)   {memset(gpattData.angle_channel_0, 0, 56); strcpy(gpattData.angle_channel_0, serial1Data.token);   gpattData.check_data++; gpattData.bad_angle_channel_0 = false;}  else {gpattData.bad_angle_channel_0_i++;  gpattData.bad_angle_channel_0 = true;}}
    else if (serial1Data.iter_token == 3) {if (val_roll_gpatt(serial1Data.token) == true)              {memset(gpattData.roll, 0, 56); strcpy(gpattData.roll, serial1Data.token);                         gpattData.check_data++; gpattData.bad_roll = false;}             else {gpattData.bad_roll_i++;             gpattData.bad_roll = true;}}
    else if (serial1Data.iter_token == 4) {if (val_angle_channle_r_gpatt(serial1Data.token) == true)   {memset(gpattData.angle_channel_1, 0, 56); strcpy(gpattData.angle_channel_1, serial1Data.token);   gpattData.check_data++; gpattData.bad_angle_channel_1 = false;}  else {gpattData.bad_angle_channel_1_i++;  gpattData.bad_angle_channel_1 = true;}}
    else if (serial1Data.iter_token == 5) {if (val_yaw_gpatt(serial1Data.token) == true)               {memset(gpattData.yaw, 0, 56); strcpy(gpattData.yaw, serial1Data.token);                           gpattData.check_data++; gpattData.bad_yaw = false;}              else {gpattData.bad_yaw_i++;              gpattData.bad_yaw = true;}}
    else if (serial1Data.iter_token == 6) {if (val_angle_channle_y_gpatt(serial1Data.token) == true)   {memset(gpattData.angle_channel_2, 0, 56); strcpy(gpattData.angle_channel_2, serial1Data.token);   gpattData.check_data++; gpattData.bad_angle_channel_2 = false;}  else {gpattData.bad_angle_channel_2_i++;  gpattData.bad_angle_channel_2 = true;}}
    else if (serial1Data.iter_token == 7) {if (val_software_version_gpatt(serial1Data.token) == true)  {memset(gpattData.software_version, 0, 56); strcpy(gpattData.software_version, serial1Data.token); gpattData.check_data++; gpattData.bad_software_version = false;} else {gpattData.bad_software_version_i++; gpattData.bad_software_version = true;}}
    else if (serial1Data.iter_token == 8) {if (val_version_channel_s_gpatt(serial1Data.token) == true) {memset(gpattData.version_channel, 0, 56); strcpy(gpattData.version_channel, serial1Data.token);   gpattData.check_data++; gpattData.bad_version_channel = false;}  else {gpattData.bad_version_channel_i++;  gpattData.bad_version_channel = true;}}
    else if (serial1Data.iter_token == 9) {if (val_product_id_gpatt(serial1Data.token) == true)        {memset(gpattData.product_id, 0, 56); strcpy(gpattData.product_id, serial1Data.token);             gpattData.check_data++; gpattData.bad_product_id = false;}       else {gpattData.bad_product_id_i++;       gpattData.bad_product_id = true;}}
    else if (serial1Data.iter_token == 10) {if (val_id_channel_gpatt(serial1Data.token) == true)       {memset(gpattData.id_channel, 0, 56); strcpy(gpattData.id_channel, serial1Data.token);             gpattData.check_data++; gpattData.bad_id_channel = false;}       else {gpattData.bad_id_channel_i++;       gpattData.bad_id_channel = true;}}
    else if (serial1Data.iter_token == 11) {if (val_ins_gpatt(serial1Data.token) == true)              {memset(gpattData.ins, 0, 56); strcpy(gpattData.ins, serial1Data.token);                           gpattData.check_data++; gpattData.bad_ins = false;}              else {gpattData.bad_ins_i++;              gpattData.bad_ins = true;}}
    else if (serial1Data.iter_token == 12) {if (val_ins_channel_gpatt(serial1Data.token) == true)      {memset(gpattData.ins_channel, 0, 56); strcpy(gpattData.ins_channel, serial1Data.token);           gpattData.check_data++; gpattData.bad_ins_channel = false;}      else {gpattData.bad_ins_channel_i++;      gpattData.bad_ins_channel = true;}}
    else if (serial1Data.iter_token == 13) {if (val_hardware_version_gpatt(serial1Data.token) == true) {memset(gpattData.hardware_version, 0, 56); strcpy(gpattData.hardware_version, serial1Data.token); gpattData.check_data++; gpattData.bad_hardware_version = false;} else {gpattData.bad_hardware_version_i++; gpattData.bad_hardware_version = true;}}
    else if (serial1Data.iter_token == 14) {if (val_run_state_flag_gpatt(serial1Data.token) == true)   {memset(gpattData.run_state_flag, 0, 56); strcpy(gpattData.run_state_flag, serial1Data.token);     gpattData.check_data++; gpattData.bad_run_state_flag = false;}   else {gpattData.bad_run_state_flag_i++;   gpattData.bad_run_state_flag = true;}}
    else if (serial1Data.iter_token == 15) {if (val_mis_angle_num_gpatt(serial1Data.token) == true)    {memset(gpattData.mis_angle_num, 0, 56); strcpy(gpattData.mis_angle_num, serial1Data.token);       gpattData.check_data++; gpattData.bad_mis_angle_num = false;}    else {gpattData.bad_mis_angle_num_i++;    gpattData.bad_mis_angle_num = true;}}
    else if (serial1Data.iter_token == 16) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_0, 0, 56); strcpy(gpattData.custom_logo_0, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_0 = false;}    else {gpattData.bad_custom_logo_0_i++;    gpattData.bad_custom_logo_0 = true;}}
    else if (serial1Data.iter_token == 17) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_1, 0, 56); strcpy(gpattData.custom_logo_1, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_1 = false;}    else {gpattData.bad_custom_logo_1_i++;    gpattData.bad_custom_logo_1 = true;}}
    else if (serial1Data.iter_token == 18) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_2, 0, 56); strcpy(gpattData.custom_logo_2, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_2 = false;}    else {gpattData.bad_custom_logo_2_i++;    gpattData.bad_custom_logo_2 = true;}}
    else if (serial1Data.iter_token == 19) {if (val_static_flag_gpatt(serial1Data.token) == true)      {memset(gpattData.static_flag, 0, 56); strcpy(gpattData.static_flag, serial1Data.token);           gpattData.check_data++; gpattData.bad_static_flag = false;}      else {gpattData.bad_static_flag_i++;      gpattData.bad_static_flag = true;}}
    else if (serial1Data.iter_token == 20) {if (val_user_code_gpatt(serial1Data.token) == true)        {memset(gpattData.user_code, 0, 56); strcpy(gpattData.user_code, serial1Data.token);               gpattData.check_data++; gpattData.bad_user_code = false;}        else {gpattData.bad_user_code_i++;        gpattData.bad_user_code = true;}}
    else if (serial1Data.iter_token == 21) {if (val_gst_data_gpatt(serial1Data.token) == true)         {memset(gpattData.gst_data, 0, 56); strcpy(gpattData.gst_data, serial1Data.token);                 gpattData.check_data++; gpattData.bad_gst_data = false;}         else {gpattData.bad_gst_data_i++;         gpattData.bad_gst_data = true;}}
    else if (serial1Data.iter_token == 22) {if (val_line_flag_gpatt(serial1Data.token) == true)        {memset(gpattData.line_flag, 0, 56); strcpy(gpattData.line_flag, serial1Data.token);               gpattData.check_data++; gpattData.bad_line_flag = false;}        else {gpattData.bad_line_flag_i++;        gpattData.bad_line_flag = true;}}
    else if (serial1Data.iter_token == 23) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_3, 0, 56); strcpy(gpattData.custom_logo_3, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_3 = false;}    else {gpattData.bad_custom_logo_3_i++;    gpattData.bad_custom_logo_3 = true;}}
    else if (serial1Data.iter_token == 24) {if (val_mis_att_flag_gpatt(serial1Data.token) == true)     {memset(gpattData.mis_att_flag, 0, 56); strcpy(gpattData.mis_att_flag, serial1Data.token);         gpattData.check_data++; gpattData.bad_mis_att_flag = false;}     else {gpattData.bad_mis_att_flag_i++;     gpattData.bad_mis_att_flag = true;}}
    else if (serial1Data.iter_token == 25) {if (val_imu_kind_gpatt(serial1Data.token) == true)         {memset(gpattData.imu_kind, 0, 56); strcpy(gpattData.imu_kind, serial1Data.token);                 gpattData.check_data++; gpattData.bad_imu_kind = false;}         else {gpattData.bad_imu_kind_i++;         gpattData.bad_imu_kind = true;}}
    else if (serial1Data.iter_token == 26) {if (val_ubi_car_kind_gpatt(serial1Data.token) == true)     {memset(gpattData.ubi_car_kind, 0, 56); strcpy(gpattData.ubi_car_kind, serial1Data.token);         gpattData.check_data++; gpattData.bad_ubi_car_kind = false;}     else {gpattData.bad_ubi_car_kind_i++;     gpattData.bad_ubi_car_kind = true;}}
    else if (serial1Data.iter_token == 27) {if (val_mileage_gpatt(serial1Data.token) == true)          {memset(gpattData.mileage, 0, 56); strcpy(gpattData.mileage, serial1Data.token);                   gpattData.check_data++; gpattData.bad_mileage = false;}          else {gpattData.bad_mileage_i++;          gpattData.bad_mileage = true;}}
    else if (serial1Data.iter_token == 28) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_4, 0, 56); strcpy(gpattData.custom_logo_4, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_4 = false;}    else {gpattData.bad_custom_logo_4_i++;    gpattData.bad_custom_logo_4 = true;}}
    else if (serial1Data.iter_token == 29) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_5, 0, 56); strcpy(gpattData.custom_logo_5, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_5 = false;}    else {gpattData.bad_custom_logo_5_i++;    gpattData.bad_custom_logo_5 = true;}}
    else if (serial1Data.iter_token == 30) {if (val_run_inetial_flag_gpatt(serial1Data.token) == true) {memset(gpattData.run_inetial_flag, 0, 56); strcpy(gpattData.run_inetial_flag, serial1Data.token); gpattData.check_data++; gpattData.bad_run_inetial_flag = false;} else {gpattData.bad_run_inetial_flag_i++; gpattData.bad_run_inetial_flag = true;}}
    else if (serial1Data.iter_token == 31) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_6, 0, 56); strcpy(gpattData.custom_logo_6, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_6 = false;}    else {gpattData.bad_custom_logo_6_i++;    gpattData.bad_custom_logo_6 = true;}}
    else if (serial1Data.iter_token == 32) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_7, 0, 56); strcpy(gpattData.custom_logo_7, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_7 = false;}    else {gpattData.bad_custom_logo_7_i++;    gpattData.bad_custom_logo_7 = true;}}
    else if (serial1Data.iter_token == 33) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_8, 0, 56); strcpy(gpattData.custom_logo_8, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_8 = false;}    else {gpattData.bad_custom_logo_8_i++;    gpattData.bad_custom_logo_8 = true;}}
    else if (serial1Data.iter_token == 34) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_9, 0, 56); strcpy(gpattData.custom_logo_9, serial1Data.token);       gpattData.check_data++; gpattData.bad_custom_logo_9 = false;}    else {gpattData.bad_custom_logo_9_i++;    gpattData.bad_custom_logo_9 = true;}}
    else if (serial1Data.iter_token == 35) {if (val_speed_enable_gpatt(serial1Data.token) == true)     {memset(gpattData.speed_enable, 0, 56); strcpy(gpattData.speed_enable, serial1Data.token);         gpattData.check_data++; gpattData.bad_speed_enable = false;}     else {gpattData.bad_speed_enable_i++;     gpattData.bad_speed_enable = true;}}
    else if (serial1Data.iter_token == 36) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_10, 0, 56); strcpy(gpattData.custom_logo_10, serial1Data.token);     gpattData.check_data++; gpattData.bad_custom_logo_10 = false;}   else {gpattData.bad_custom_logo_10_i++;   gpattData.bad_custom_logo_10 = true;}}
    else if (serial1Data.iter_token == 37) {if (val_custom_flag(serial1Data.token) == true)            {memset(gpattData.custom_logo_11, 0, 56); strcpy(gpattData.custom_logo_11, serial1Data.token);     gpattData.check_data++; gpattData.bad_custom_logo_11 = false;}   else {gpattData.bad_custom_logo_11_i++;   gpattData.bad_custom_logo_11 = true;}}
    else if (serial1Data.iter_token == 38) {if (val_speed_num_gpatt(serial1Data.token) == true)        {memset(gpattData.speed_num, 0, 56); strcpy(gpattData.speed_num, serial1Data.token);               gpattData.check_data++; gpattData.bad_speed_num = false;}        else {gpattData.bad_speed_num_i++;        gpattData.bad_speed_num = true;}}
    // else if (serial1Data.iter_token == 39) {
    //   memset(gnggaData.temporary_data, 0, 56);
    //   strcpy(gpattData.temporary_data, strtok(serial1Data.token, "*"));
    //   if (val_scalable(gpattData.temporary_data) == true) {memset(gpattData.scalable, 0, 56); strcpy(gpattData.scalable, gpattData.temporary_data); gpattData.check_data++; gpattData.bad_scalable = false;} else {gpattData.bad_scalable_i++; gpattData.bad_scalable = true;}
    //   serial1Data.token = strtok(NULL, "*");
    //   memset(gnggaData.temporary_data_1, 0, 56);
    //   strcpy(gpattData.temporary_data_1, strtok(serial1Data.token, "*"));
    //   if (val_checksum(gpattData.temporary_data_1) == true) {memset(gpattData.check_sum, 0, 56); strcpy(gpattData.check_sum, gpattData.temporary_data_1); gpattData.check_data++; gpattData.bad_check_sum = false;} else {gpattData.bad_check_sum_i++; gpattData.bad_check_sum = true;}}
    serial1Data.token = strtok(NULL, ",");
    serial1Data.iter_token++;
  }
  if (sysDebugData.gpatt_sentence == true) {
    Serial.println("[gpattData.tag] "              + String(gpattData.tag));
    Serial.println("[gpattData.pitch] "            + String(gpattData.pitch));
    Serial.println("[gpattData.angle_channel_0] "  + String(gpattData.angle_channel_0));
    Serial.println("[gpattData.roll] "             + String(gpattData.roll));
    Serial.println("[gpattData.angle_channel_1] "  + String(gpattData.angle_channel_1));
    Serial.println("[gpattData.yaw] "              + String(gpattData.yaw)); 
    Serial.println("[gpattData.angle_channel_2] "  + String(gpattData.angle_channel_2));
    Serial.println("[gpattData.software_version] " + String(gpattData.software_version));
    Serial.println("[gpattData.version_channel] "  + String(gpattData.version_channel));
    Serial.println("[gpattData.product_id] "       + String(gpattData.product_id));
    Serial.println("[gpattData.id_channel] "       + String(gpattData.id_channel));
    Serial.println("[gpattData.ins] "              + String(gpattData.ins));
    Serial.println("[gpattData.ins_channel] "      + String(gpattData.ins_channel));
    Serial.println("[gpattData.hardware_version] " + String(gpattData.hardware_version));
    Serial.println("[gpattData.run_state_flag] "   + String(gpattData.run_state_flag));
    Serial.println("[gpattData.mis_angle_num] "    + String(gpattData.mis_angle_num));
    Serial.println("[gpattData.custom_logo_0] "    + String(gpattData.custom_logo_0));
    Serial.println("[gpattData.custom_logo_1] "    + String(gpattData.custom_logo_1));
    Serial.println("[gpattData.custom_logo_2] "    + String(gpattData.custom_logo_2));
    Serial.println("[gpattData.static_flag] "      + String(gpattData.static_flag));
    Serial.println("[gpattData.user_code] "        + String(gpattData.user_code));
    Serial.println("[gpattData.gst_data] "         + String(gpattData.gst_data));
    Serial.println("[gpattData.line_flag] "        + String(gpattData.line_flag));
    Serial.println("[gpattData.custom_logo_3] "    + String(gpattData.custom_logo_3));
    Serial.println("[gpattData.imu_kind] "         + String(gpattData.imu_kind));
    Serial.println("[gpattData.ubi_car_kind] "     + String(gpattData.ubi_car_kind));
    Serial.println("[gpattData.mileage] "          + String(gpattData.mileage));
    Serial.println("[gpattData.custom_logo_4] "    + String(gpattData.custom_logo_4));
    Serial.println("[gpattData.custom_logo_5] "    + String(gpattData.custom_logo_5));
    Serial.println("[gpattData.run_inetial_flag] " + String(gpattData.run_inetial_flag));
    Serial.println("[gpattData.custom_logo_6] "    + String(gpattData.custom_logo_6));
    Serial.println("[gpattData.custom_logo_7] "    + String(gpattData.custom_logo_7));
    Serial.println("[gpattData.custom_logo_8] "    + String(gpattData.custom_logo_8));
    Serial.println("[gpattData.custom_logo_9] "    + String(gpattData.custom_logo_9));
    Serial.println("[gpattData.speed_enable] "     + String(gpattData.speed_enable));
    Serial.println("[gpattData.custom_logo_10] "   + String(gpattData.custom_logo_10));
    Serial.println("[gpattData.custom_logo_11] "   + String(gpattData.custom_logo_11));
    Serial.println("[gpattData.speed_num] "        + String(gpattData.speed_num));
    Serial.println("[gpattData.scalable] "         + String(gpattData.scalable));
    Serial.println("[gpattData.check_sum] "        + String(gpattData.check_sum));
    Serial.println("[gpattData.check_data] "        + String(gpattData.check_data));
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: SATIO

struct SatDatatruct {
  char checksum_str[56];                                            // checksum string
  int checksum_i;                                                   // checksum int
  char satio_sentence[2000];                                        // buffer
  char sat_time_stamp_string[56];                                   // datetime timestamp from satellite
  char satDataTag[10]                 = "$SATIO";                   // satio sentence tag
  char last_sat_time_stamp_str[56]    = "0000000000000000";         // record last time satellites were seen yyyymmddhhmmssmm
  bool convert_coordinates            = true;                       // enables/disables coordinate conversion to degrees
  char coordinate_conversion_mode[10] = "GNGGA";                    // sentence coordinates degrees created from
  double latitude_meter               = 0.0000100;                  // one meter (tune)
  double longitude_meter              = 0.0000100;                  // one meter (tune)
  double latitude_mile                = latitude_meter  * 1609.34;  // one mile
  double longitude_mile               = longitude_meter * 1609.34;  // one mile
  double abs_latitude_gngga_0         = 0.0;                        // absolute latitude from $ sentence
  double abs_longitude_gngga_0        = 0.0;                        // absolute longditude from $ sentence
  double abs_latitude_gnrmc_0         = 0.0;                        // absolute latitude from $ sentence
  double abs_longitude_gnrmc_0        = 0.0;                        // absolute longditude from $ sentence
  double temp_latitude_gngga;                                       // degrees converted from absolute
  double temp_longitude_gngga;                                      // degrees converted from absolute
  double temp_latitude_gnrmc;                                       // degrees converted from absolute
  double temp_longitude_gnrmc;                                      // degrees converted from absolute
  double location_latitude_gngga;                                   // degrees converted from absolute
  double location_longitude_gngga;                                  // degrees converted from absolute
  double location_latitude_gnrmc;                                   // degrees converted from absolute
  double location_longitude_gnrmc;                                  // degrees converted from absolute
  char location_latitude_gngga_str[56];                             // degrees converted from absolute
  char location_longitude_gngga_str[56];                            // degrees converted from absolute
  char location_latitude_gnrmc_str[56];                             // degrees converted from absolute
  char location_longitude_gnrmc_str[56];                            // degrees converted from absolute
  double minutesLat;                                                // used for converting absolute latitude and longitude
  double minutesLong;                                               // used for converting absolute latitude and longitude
  double degreesLat;                                                // used for converting absolute latitude and longitude
  double degreesLong;                                               // used for converting absolute latitude and longitude
  double secondsLat;                                                // used for converting absolute latitude and longitude
  double secondsLong;                                               // used for converting absolute latitude and longitude
  double millisecondsLat;                                           // used for converting absolute latitude and longitude
  double millisecondsLong;                                          // used for converting absolute latitude and longitude

  signed int utc_offset = 0;          // can be used to offset UTC (+/-), to account for daylight saving and or timezones.
  bool utc_offset_flag = 0;           // 0: add hours to time; 1: deduct hours from time

  char pad_digits_new[56];            // a placeholder for digits preappended with zero's.
  char pad_current_digits[56];        // a placeholder for digits to be preappended with zero's.

  /* TEMPORARY TIME VALUES */
  signed int tmp_year_int;                   // temp current year
  signed int tmp_month_int;                  // temp current month
  signed int tmp_day_int;                    // temp current day
  signed int tmp_hour_int;                   // temp current hour
  signed int tmp_minute_int;                 // temp current minute
  signed int tmp_second_int;                 // temp current second
  signed int tmp_millisecond_int;            // temp current millisecond
  char tmp_year[56];                  // temp current year
  char tmp_month[56];                 // temp current month
  char tmp_day[56];                   // temp current day
  char tmp_hour[56];                  // temp current hour
  char tmp_minute[56];                // temp current minute
  char tmp_second[56];                // temp current second
  char tmp_millisecond[56];           // temp current millisecond

  /* TIME VALUES FOR RTC AND OTHER USE */
  signed int lt_year_int = 0;                    // last year satellite count > zero
  signed int lt_month_int = 0;                   // last month satellite count > zero
  signed int lt_day_int = 0;                     // last day satellite count > zero
  signed int lt_hour_int = 0;                    // last hour satellite count > zero
  signed int lt_minute_int = 0;                  // last minute satellite count > zero
  signed int lt_second_int = 0;                  // last second satellite count > zero
  signed int lt_millisecond_int = 0;             // last millisecond satellite count > zero

  // long current_unixtime;
};
SatDatatruct satData;

struct SensorDataStruct {
  float dht11_h_0 = 0.0;
  float dht11_c_0 = 0.0;
  float dht11_f_0 = 0.0;
  float dht11_hif_0 = 0.0;
  float dht11_hic_0 = 0.0;
  bool dht11_0_display_hic = true;
  int photoresistor_0 = 0;
  int tracking_0 = 0;
};
SensorDataStruct sensorData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                         CONVERT COORDINTE DATA
void calculateLocation(){

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                  GNGGA COORDINATE CONVERSION

  /*
  Convert GNGGA latitude & longitude strings to decimal degrees and format into hours, minutes, seconds, milliseconds.
  */
  if (String(satData.coordinate_conversion_mode) == "GNGGA") {

    // Extract absolute latitude value from GNGGA data as decimal degrees.
    satData.abs_latitude_gngga_0 = atof(String(gnggaData.latitude).c_str());
    // Store absolute latitude in temporary variable for further processing.
    satData.temp_latitude_gngga = satData.abs_latitude_gngga_0;
    // Separate the integer degrees value from the fractional part.
    satData.degreesLat = trunc(satData.temp_latitude_gngga / 100);
    // Calculate minutes and seconds values based on remaining fractional part.
    satData.minutesLat = satData.temp_latitude_gngga - (satData.degreesLat * 100);
    // Convert excess fractional part to seconds.
    satData.secondsLat = (satData.minutesLat - trunc(satData.minutesLat)) * 60;
    // Convert excess seconds to milliseconds.
    satData.millisecondsLat = (satData.secondsLat - trunc(satData.secondsLat)) * 1000;
    // Round off minutes and seconds values to nearest integer.
    satData.minutesLat = trunc(satData.minutesLat);
    satData.secondsLat = trunc(satData.secondsLat);
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    satData.location_latitude_gngga =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnggaData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gngga = 0 - satData.location_latitude_gngga;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.location_latitude_gngga);
    // Convert latitude value to a human-readable string representation.
    sprintf(satData.location_latitude_gngga_str, "%f", satData.location_latitude_gngga);

    // Extract absolute longitude value from GNGGA data as decimal degrees.
    satData.abs_longitude_gngga_0 = atof(String(gnggaData.longitude).c_str());
    // Store absolute latitude in temporary variable for further processing.
    satData.temp_longitude_gngga = satData.abs_longitude_gngga_0;
    // Separate the integer degrees value from the fractional part.
    satData.degreesLong = trunc(satData.temp_longitude_gngga / 100);
    // Calculate minutes and seconds values based on remaining fractional part.
    satData.minutesLong = satData.temp_longitude_gngga - (satData.degreesLong * 100);
    // Convert excess fractional part to seconds.
    satData.secondsLong = (satData.minutesLong - trunc(satData.minutesLong)) * 60;
    // Convert excess seconds to milliseconds.
    satData.millisecondsLong = (satData.secondsLong - trunc(satData.secondsLong)) * 1000;
    // Round off minutes and seconds values to nearest integer.
    satData.minutesLong = trunc(satData.minutesLong);
    satData.secondsLong = trunc(satData.secondsLong);
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    satData.location_longitude_gngga =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnggaData.longitude_hemisphere, "W") == 0) {
      satData.location_longitude_gngga = 0 - satData.location_longitude_gngga;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.location_longitude_gngga);
    // Convert latitude value to a human-readable string representation.
    sprintf(satData.location_longitude_gngga_str, "%f", satData.location_longitude_gngga);
  }

  // ------------------------------------------------------------------------------------------------------------------------
  //                                                                                              GNRMC COORDINATE CONVERSION

  /*
  Convert GNRMC latitude & longitude strings to decimal degrees and format into hours, minutes, seconds, milliseconds.
  */
  else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
    // Extract absolute latitude value from GNGGA data as decimal degrees.
    satData.abs_latitude_gnrmc_0 = atof(String(gnrmcData.latitude).c_str());
    // Store absolute latitude in temporary variable for further processing.
    satData.temp_latitude_gnrmc = satData.abs_latitude_gnrmc_0;
    // Separate the integer degrees value from the fractional part.
    satData.degreesLat = trunc(satData.temp_latitude_gnrmc / 100);
    // Calculate minutes and seconds values based on remaining fractional part.
    satData.minutesLat = satData.temp_latitude_gnrmc - (satData.degreesLat * 100);
    // Convert excess fractional part to seconds.
    satData.secondsLat = (satData.minutesLat - (satData.minutesLat)) * 60;
    // Convert excess seconds to milliseconds.
    satData.millisecondsLat = (satData.secondsLat - trunc(satData.secondsLat)) * 1000;
    // Round off minutes and seconds values to nearest integer.
    satData.minutesLat = trunc(satData.minutesLat);
    satData.secondsLat = trunc(satData.secondsLat);
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    satData.location_latitude_gnrmc =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnrmcData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gnrmc = 0 - satData.location_latitude_gnrmc;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.location_latitude_gnrmc);
    // Convert latitude value to a human-readable string representation.
    sprintf(satData.location_latitude_gnrmc_str, "%f", satData.location_latitude_gnrmc);

    // Extract absolute latitude value from GNGGA data as decimal degrees.
    satData.abs_longitude_gnrmc_0 = atof(String(gnrmcData.longitude).c_str());
    // Store absolute latitude in temporary variable for further processing.
    satData.temp_longitude_gnrmc = satData.abs_longitude_gnrmc_0;
    // Separate the integer degrees value from the fractional part.
    satData.degreesLong = trunc(satData.temp_longitude_gnrmc / 100);
    // Calculate minutes and seconds values based on remaining fractional part.
    satData.minutesLong = satData.temp_longitude_gnrmc - (satData.degreesLong * 100);
    // Convert excess fractional part to seconds.
    satData.secondsLong = (satData.minutesLong - trunc(satData.minutesLong)) * 60;
    // Convert excess seconds to milliseconds.
    satData.millisecondsLong = (satData.secondsLong - trunc(satData.secondsLong)) * 1000;
    // Round off minutes and seconds values to nearest integer.
    satData.minutesLong = trunc(satData.minutesLong);
    satData.secondsLong = trunc(satData.secondsLong);
    // Combine degrees, minutes, seconds, and milliseconds into a single decimal latitude value.
    satData.location_longitude_gnrmc =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnrmcData.longitude_hemisphere, "W") == 0) {
      satData.location_longitude_gnrmc = 0 - satData.location_longitude_gnrmc;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.location_longitude_gnrmc);
    // Convert latitude value to a human-readable string representation.
    sprintf(satData.location_longitude_gnrmc_str, "%f", satData.location_longitude_gnrmc);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 SATIO SENTENCE

String padDigitsZero(int digits) {
  /* preappends char 0 to pad string of digits evenly */
  memset(satData.pad_digits_new, 0, sizeof(satData.pad_digits_new));
  memset(satData.pad_current_digits, 0, sizeof(satData.pad_current_digits));
  if(digits < 10) {strcat(satData.pad_digits_new, "0");}
  itoa(digits, satData.pad_current_digits, 10);
  strcat(satData.pad_digits_new, satData.pad_current_digits);
  return satData.pad_digits_new;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                        SET LAST SATELLITE TIME

void syncRTCOnDownlink() {
  rtc.adjust(DateTime(satData.lt_year_int, satData.lt_month_int, satData.lt_day_int, satData.lt_hour_int, satData.lt_minute_int, satData.lt_second_int));
  Serial.println("[synchronized] " + SerialDisplayRTCDateTime());
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           CONVERT UTC TO LOCAL

char hours_minutes_seconds[6];
char tmp_hours_minutes_seconds[4];
int hoursMinutesSecondsToInt(int hours, int minutes, int seconds) {
  itoa(hours, tmp_hours_minutes_seconds, 10);
  hours_minutes_seconds[0] = tmp_hours_minutes_seconds[0];
  hours_minutes_seconds[1] = tmp_hours_minutes_seconds[1];
  itoa(minutes, tmp_hours_minutes_seconds, 10);
  hours_minutes_seconds[2] = tmp_hours_minutes_seconds[0];
  hours_minutes_seconds[3] = tmp_hours_minutes_seconds[1];
  itoa(seconds, tmp_hours_minutes_seconds, 10);
  hours_minutes_seconds[4] = tmp_hours_minutes_seconds[0];
  hours_minutes_seconds[5] = tmp_hours_minutes_seconds[1];
  return atoi(hours_minutes_seconds);
}
int hoursMinutesToInt(int hours, int minutes) {
  itoa(hours, tmp_hours_minutes_seconds, 10);
  hours_minutes_seconds[0] = tmp_hours_minutes_seconds[0];
  hours_minutes_seconds[1] = tmp_hours_minutes_seconds[1];
  itoa(minutes, tmp_hours_minutes_seconds, 10);
  hours_minutes_seconds[2] = tmp_hours_minutes_seconds[0];
  hours_minutes_seconds[3] = tmp_hours_minutes_seconds[1];
  return atoi(hours_minutes_seconds);
}

// temporary char time values so that we do not disturb the primary values while converting.
char temp_sat_time_stamp_string[128];
bool first_gps_pass = true;

bool isTwoDiff(int a, int b) {
  return abs(a - b) <= 2;
}
bool isOneDiff(int a, int b) {
  return abs(a - b) <= 1;
}

void convertUTCToLocal() {

  // // live data from satellites
  // Serial.println("---------------------"); // debug
  // Serial.print("[utc_time] "); Serial.println(gnrmcData.utc_time); // debug
  // Serial.print("[utc_date] "); Serial.println(gnrmcData.utc_date); // debug

  /*                                     TEMPORARY TIME                                        */
  /* make temporary values that will not disturb final values untiil values whole and complete */

  // temp store date
  satData.tmp_day[0] = gnrmcData.utc_date[0];
  satData.tmp_day[1] = gnrmcData.utc_date[1];
  // Serial.print("[tmp_day] "); Serial.println(satData.tmp_day);
  satData.tmp_month[0] = gnrmcData.utc_date[2];
  satData.tmp_month[1] = gnrmcData.utc_date[3];
  // Serial.print("[tmp_month] "); Serial.println(satData.tmp_month);
  satData.tmp_year[0] = gnrmcData.utc_date[4];
  satData.tmp_year[1] = gnrmcData.utc_date[5];
  // Serial.print("[tmp_year] "); Serial.println(satData.tmp_year);

  // temp store time
  satData.tmp_hour[0] = gnrmcData.utc_time[0];
  satData.tmp_hour[1] = gnrmcData.utc_time[1];
  // Serial.print("[tmp_hour] "); Serial.println(satData.tmp_hour);
  satData.tmp_minute[0] = gnrmcData.utc_time[2];
  satData.tmp_minute[1] = gnrmcData.utc_time[3];
  // Serial.print("[tmp_minute] "); Serial.println(satData.tmp_minute);
  satData.tmp_second[0] = gnrmcData.utc_time[4];
  satData.tmp_second[1] = gnrmcData.utc_time[5];
  // Serial.print("[tmp_second] "); Serial.println(satData.tmp_second);
  satData.tmp_millisecond[0] = gnrmcData.utc_time[7];
  satData.tmp_millisecond[1] = gnrmcData.utc_time[8];
  // Serial.print("[tmp_second] "); Serial.println(satData.tmp_second);

  // temporary int time values so that we do not disturb the primary values while converting.
  satData.tmp_day_int = atoi(satData.tmp_day);
  satData.tmp_month_int = atoi(satData.tmp_month);
  satData.tmp_year_int = atoi(satData.tmp_year);
  satData.tmp_hour_int = atoi(satData.tmp_hour);
  satData.tmp_minute_int = atoi(satData.tmp_minute);
  satData.tmp_second_int = atoi(satData.tmp_second);
  satData.tmp_millisecond_int = atoi(satData.tmp_millisecond);

  // uncomment to debug before conversion
  // Serial.print("[temp datetime]           ");
  // Serial.print(satData.tmp_hour_int);
  // Serial.print(":"); Serial.print(satData.tmp_minute_int);
  // Serial.print(":"); Serial.print(satData.tmp_second_int);
  // Serial.print(" "); Serial.print(satData.tmp_day_int);
  // Serial.print("."); Serial.print(satData.tmp_month_int);
  // Serial.print("."); Serial.print(satData.tmp_year_int);
  // Serial.print(" (abbreviated year: "); Serial.print(satData.tmp_year_int); Serial.println(")");

  // set time using time elements with 2 digit year
  setTime(
    satData.tmp_hour_int,
    satData.tmp_minute_int,
    satData.tmp_second_int,
    satData.tmp_day_int,
    satData.tmp_month_int,
    satData.tmp_year_int);

  // set elements as time return functions
  tmElements_t tm_return = {second(), minute(), hour(), weekday(), day(), month(), year()};

  // return time
  time_t tmp_makeTime = makeTime(tm_return);
  // Serial.print("[tmp_makeTime]            "); Serial.println(tmp_makeTime);

  // adjust tmp_makeTime back/forward according to UTC offset
  if      (satData.utc_offset_flag==0) {adjustTime(satData.utc_offset*SECS_PER_HOUR);}
  else                                 {adjustTime(-satData.utc_offset*SECS_PER_HOUR);}

  // uncomment to debug before conversion
  // Serial.print("[temp datetime +- offset] ");
  // Serial.print(hour());
  // Serial.print(":"); Serial.print(minute());
  // Serial.print(":"); Serial.print(second());
  // Serial.print(" "); Serial.print(day());
  // Serial.print("."); Serial.print(month());
  // Serial.print("."); Serial.println(year());

  /*                        RTC TIME                        */
  /* store current local time on RTC if we have a downlink  */
  if (atoi(gnggaData.satellite_count_gngga) > 3) {

    // update last possible downlink time
    satData.lt_year_int = year();
    satData.lt_month_int = month();
    satData.lt_day_int = day();
    satData.lt_hour_int = hour();
    satData.lt_minute_int = minute();
    satData.lt_second_int = second();
    satData.lt_millisecond_int = satData.tmp_millisecond_int;

    /*
    adjust rtc while we appear to have a downlink and allow for a certain amount of drift (hardware/software depending)
    to avoid setting the RTC too often so that we should have a stable, steady and predictable RTC time where
    otherwise time from the RTC if set too often would be as useful (and equal to) any time calculated live from the
    downlink which would defeat the point of having an RTC and depending on certain conditions may not be suitable at all
    for steady timings.
    to do: when synchronizing RTC, only synchronize RTC within a 10th of a second of live GPS data. example gnrmc_utc=01.03.00=sync, gnrmc_utc=01.03.10=dont sync.
    */
    if ((first_gps_pass==true) ) {
      if (satData.tmp_millisecond_int==00) {first_gps_pass=false; syncRTCOnDownlink();} // maybe synchronize on first pass of this function (like on startup for example)
    }
    else {
      // sync every minute according to downlinked time. 
      if (satData.lt_second_int == 0) {if (satData.tmp_millisecond_int==00) {syncRTCOnDownlink();}}
    }
  }

  // Serial.println("[rtc time] " + SerialDisplayRTCDateTime()); // debug

  /*    now we can do things with time (using rtc time)     */

}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           BUILD SATIO SENTENCE
void buildSatIOSentence() {

  /* create a comma delimited sentence of new data, to print over serial that can be parsed by other systems */

  // start building satio sentence
  memset(satData.satio_sentence, 0, sizeof(satData.satio_sentence));
  strcat(satData.satio_sentence, satData.satDataTag);
  strcat(satData.satio_sentence, ",");

    // make unix second time
    // Serial.println("[unix time]               " + String(rtc.now().unixtime()));
    // Serial.println("-------------");

  // create timestamp for satio sentence 
  strcat(satData.satio_sentence, rtc.now().timestamp().c_str());
  strcat(satData.satio_sentence, ",");

  // append to satio sentence (pending new method)
  // strcat(satData.satio_sentence, satData.last_sat_time_stamp_str);
  // strcat(satData.satio_sentence, ",");

  // coordinate conversion mode
  if (satData.convert_coordinates == true) {
    if (String(satData.coordinate_conversion_mode) == "GNGGA") {
      // append to satio sentence
      strcat(satData.satio_sentence, satData.location_latitude_gngga_str);
      strcat(satData.satio_sentence, ",");
      strcat(satData.satio_sentence, satData.location_longitude_gngga_str);
      strcat(satData.satio_sentence, ",");
    }
    else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
      // append to satio sentence
      strcat(satData.satio_sentence, satData.location_latitude_gnrmc_str);
      strcat(satData.satio_sentence, ",");
      strcat(satData.satio_sentence, satData.location_longitude_gnrmc_str);
      strcat(satData.satio_sentence, ",");
    }
  }
  else {strcat(satData.satio_sentence, "0.0,0.0,");}

  // append checksum
  createChecksum(satData.satio_sentence);
  strcat(satData.satio_sentence, "*");
  strcat(satData.satio_sentence, SerialLink.checksum);
  strcat(satData.satio_sentence, "\n");
  if (systemData.output_satio_enabled == true) {Serial.println(satData.satio_sentence);}

  // Serial.println(satData.satio_sentence);  // debug
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                          SDCARD: PRINT FILE CONTENTS TO SERIAL

bool sdcard_read_to_serial(fs::FS &fs, char * file) {

  /* prints the contents of a file to serial  */

  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file);
  if (sdcardData.current_file) {
    while (sdcardData.current_file.available()) {Serial.write(sdcardData.current_file.read());}
    sdcardData.current_file.close(); return true;
  }
  else {sdcardData.current_file.close(); return false;}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                              SDCARD: SAVE SYSTEM CONFIGURATION

void sdcard_save_system_configuration(fs::FS &fs, char * file, int return_page) {

  /* saves tagged, system configuration data to file */

  sdcardData.is_writing = true;

  if (sysDebugData.verbose_file==true) {
  Serial.println("[sdcard] attempting to save file: " + String(file));
  }
  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file, FILE_WRITE);
  if (sdcardData.current_file) {

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "MATRIX_FILEPATH,");
    if (!sdcardData.matrix_filepath) {strcat(sdcardData.file_data, sdcardData.default_matrix_filepath);}
    else {strcat(sdcardData.file_data, sdcardData.matrix_filepath);}
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "AUTO_RESUME,");
    itoa(systemData.run_on_startup, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "MATRIX_ENABLED,");
    itoa(systemData.matrix_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "SATIO_ENABLED,");
    itoa(systemData.satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "GNGGA_ENABLED,");
    itoa(systemData.gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "GNRMC_ENABLED,");
    itoa(systemData.gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "GPATT_ENABLED,");
    itoa(systemData.gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_BRIGHTNESS,");
    itoa(systemData.display_brightness, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_AUTO_DIM,");
    itoa(systemData.display_auto_dim, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_AUTO_DIM_BRIGHTNESS,");
    itoa(systemData.display_autodim_brightness, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_AUTO_DIM_TIMEOUT,");
    itoa(systemData.display_auto_dim_p0, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_AUTO_OFF,");
    itoa(systemData.display_auto_off, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_AUTO_OFF_TIMEOUT,");
    itoa(systemData.display_auto_off_p0, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_SATIO_SENTENCE,");
    itoa(systemData.output_satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_GNGGA_SENTENCE,");
    itoa(systemData.output_gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_GNRMC_SENTENCE,");
    itoa(systemData.output_gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_GPATT_SENTENCE,");
    itoa(systemData.output_gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "UTC_OFFSET,");
    itoa(satData.utc_offset, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "UTC_OFFSET_FLAG,");
    itoa(satData.utc_offset_flag, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_SUN,");
    itoa(systemData.sidereal_track_sun, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_MOON,");
    itoa(systemData.sidereal_track_moon, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_MERCURY,");
    itoa(systemData.sidereal_track_mercury, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_VENUS,");
    itoa(systemData.sidereal_track_venus, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_MARS,");
    itoa(systemData.sidereal_track_mars, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");
    
    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_JUPITER,");
    itoa(systemData.sidereal_track_jupiter, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_SATURN,");
    itoa(systemData.sidereal_track_saturn, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_URANUS,");
    itoa(systemData.sidereal_track_uranus, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "TRACK_NEPTUNE,");
    itoa(systemData.sidereal_track_neptune, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");    

    sdcardData.current_file.close();
    Serial.println("[sdcard] saved file successfully: " + String(file));
    sdcardData.is_writing = false;
  }
  else {sdcardData.current_file.close(); Serial.println("[sdcard] failed to save file: " + String(file));}
  sdcardData.is_writing = false;
}

void PrintFileToken() {if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [reading] " +  String(sdcardData.token));}}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                              SDCARD: LOAD SYSTEM CONFIGURATION 

bool sdcard_load_system_configuration(fs::FS &fs, char * file, int return_page) {

  sdcardData.is_reading = true;

  Serial.println("[sdcard] attempting to load file: " + String(file));
  // open file to read
  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file); 
  if (sdcardData.current_file) {
    while (sdcardData.current_file.available()) {
      // read line
      sdcardData.SBUFFER = "";
      memset(sdcardData.BUFFER, 0, 2048);
      sdcardData.SBUFFER = sdcardData.current_file.readStringUntil('\n');
      sdcardData.SBUFFER.toCharArray(sdcardData.BUFFER, sdcardData.SBUFFER.length()+1);
      if (sysDebugData.verbose_file==true) {
      Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));
      }
      // check matrix filepath
      if (strncmp(sdcardData.BUFFER, "MATRIX_FILEPATH", 15) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        PrintFileToken();
        memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
        strcpy(sdcardData.matrix_filepath, sdcardData.token);
      }
      // check auto resume
      if (strncmp(sdcardData.BUFFER, "AUTO_RESUME", 11) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          if (atoi(sdcardData.token) == 0) {systemData.run_on_startup = false;} else {systemData.run_on_startup = true;}
        }
      }
      // continue to enable/disable only if auto resume is true
      if (systemData.run_on_startup == true) {
        if (strncmp(sdcardData.BUFFER, "MATRIX_ENABLED", strlen("MATRIX_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.matrix_enabled = false;} else {systemData.matrix_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "SATIO_ENABLED", strlen("SATIO_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.satio_enabled = false;} else {systemData.satio_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "GNGGA_ENABLED", strlen("GNGGA_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.gngga_enabled = false;} else {systemData.gngga_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "GNRMC_ENABLED", strlen("GNRMC_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.gnrmc_enabled = false;} else {systemData.gnrmc_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "GPATT_ENABLED", strlen("GPATT_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.gpatt_enabled = false;} else {systemData.gpatt_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_BRIGHTNESS", strlen("DISPLAY_BRIGHTNESS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            systemData.display_brightness = atoi(sdcardData.token);
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_DIM_BRIGHTNESS", strlen("DISPLAY_AUTO_DIM_BRIGHTNESS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            systemData.display_autodim_brightness = atoi(sdcardData.token);
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_DIM_TIMEOUT", strlen("DISPLAY_AUTO_DIM_TIMEOUT")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            systemData.display_auto_dim_p0 = atoi(sdcardData.token);
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_DIM", strlen("DISPLAY_AUTO_DIM")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.display_auto_dim = false;} else {systemData.display_auto_dim = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_OFF_TIMEOUT", strlen("DISPLAY_AUTO_OFF_TIMEOUT")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            systemData.display_auto_off_p0 = atoi(sdcardData.token);
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_OFF", strlen("DISPLAY_AUTO_OFF")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.display_auto_off = false;} else {systemData.display_auto_off = true;}
            PrintFileToken();
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_SATIO_SENTENCE", strlen("OUTPUT_SATIO_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_satio_enabled = false;} else {systemData.output_satio_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GNGGA_SENTENCE", strlen("OUTPUT_GNGGA_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_gngga_enabled = false;} else {systemData.output_gngga_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GNRMC_SENTENCE", strlen("OUTPUT_GNRMC_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_gnrmc_enabled = false;} else {systemData.output_gnrmc_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GPATT_SENTENCE", strlen("OUTPUT_GPATT_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_gpatt_enabled = false;} else {systemData.output_gpatt_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "UTC_OFFSET,", strlen("UTC_OFFSET,")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            satData.utc_offset = atoi(sdcardData.token);
          }
        }
        else if (strncmp(sdcardData.BUFFER, "UTC_OFFSET_FLAG", strlen("UTC_OFFSET_FLAG")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {satData.utc_offset_flag = false;} else {satData.utc_offset_flag = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_SUN", strlen("TRACK_SUN")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_sun = false;} else {systemData.sidereal_track_sun = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_MOON", strlen("TRACK_MOON")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_moon = false;} else {systemData.sidereal_track_moon = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_MERCURY", strlen("TRACK_MERCURY")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_mercury = false;} else {systemData.sidereal_track_mercury = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_VENUS", strlen("TRACK_VENUS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_venus = false;} else {systemData.sidereal_track_venus = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_MARS", strlen("TRACK_MARS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_mars = false;} else {systemData.sidereal_track_mars = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_JUPITER", strlen("TRACK_JUPITER")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_jupiter = false;} else {systemData.sidereal_track_jupiter = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_SATURN", strlen("TRACK_SATURN")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_saturn = false;} else {systemData.sidereal_track_saturn = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_URANUS", strlen("TRACK_URANUS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_uranus = false;} else {systemData.sidereal_track_uranus = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "TRACK_NEPTUNE", strlen("TRACK_NEPTUNE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_neptune = false;} else {systemData.sidereal_track_neptune = true;}
          }
        }
      }
    }
    sdcardData.current_file.close();
    Serial.println("[sdcard] loaded file successfully: " + String(file));
    sdcardData.is_reading = false;
    return true;
  }
  else {sdcardData.current_file.close(); Serial.println("[sdcard] failed to load file: " + String(file));
  sdcardData.is_reading = false;
  return false;}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                         SDCARD: MAKE DIRECTORY

/* creates a single directory */

void sdcard_mkdir(fs::FS &fs, char * dir){
  if (!fs.exists(dir)) {
    Serial.println("[sdcard] attempting to create directory: " + String(dir));
    if (!fs.mkdir(dir)) {Serial.println("[sdcard] failed to create directory: " + String(dir));}
    else {Serial.println("[sdcard] found directory: " + String(dir));}}
  else {Serial.println("[sdcard] directory already exists: " + String(dir));}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                       SDCARD: MAKE DIRECTORIES

/* creates root directories required by the system to work properly */

void sdcard_mkdirs() {for (int i = 0; i < 2; i++) {sdcard_mkdir(SD, sdcardData.system_dirs[i]);}}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                      SDCARD: PUT ALL MATRIX FILENAMES IN ARRAY

/* discovers and compiles an array of matrix filenames */

void sdcard_list_matrix_files(fs::FS &fs, char * dir, char * name, char * ext) {
  char tempname[56];
  char temppath[56];
  char temp_i[4];
  for (int i = 0; i < sdcardData.max_matrix_filenames; i++) {memset(sdcardData.matrix_filenames[i], 0, 56);}
  for (int i = 0; i < sdcardData.max_matrix_filenames; i++) {
    memset(temppath, 0, 56);
    strcpy(temppath, dir);
    strcat(temppath, name);
    strcat(temppath, "_");
    itoa(i, temp_i, 10);
    strcat(temppath, temp_i);
    strcat(temppath, ext);
    memset(tempname, 0, 56);
    strcat(tempname, name);
    strcat(tempname, "_");
    strcat(tempname, temp_i);
    strcat(tempname, ext);
    // Serial.println("[sdcard] calculating: " + String(temppath)); // debug
    if (fs.exists(temppath)) {
      Serial.println("[sdcard] calculated filename found: " + String(temppath));
      memset(sdcardData.matrix_filenames[i], 0, 56); strcpy(sdcardData.matrix_filenames[i], temppath);
      Serial.println("[matrix_filenames] " + String(sdcardData.matrix_filenames[i]));
      }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    ZERO MATRIX

/* writes $NONE to every matrix function name for every matrix switch and writes 0 to every matrix function xyz values */

void zero_matrix() {
  Serial.println("[matrix] setting all matrix values to zero.");
  // iterate over each matrix matrix
  for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
    matrixData.matrix_switch_enabled[0][Mi] = 0;
    for (int Fi = 0; Fi < matrixData.max_matrix_functions; Fi++) {
      memset(matrixData.matrix_function[Mi][Fi], 0, 56);
      strcpy(matrixData.matrix_function[Mi][Fi], "$NONE");
      matrixData.matrix_function_xyz[Mi][Fi][0] = 0.0;
      matrixData.matrix_function_xyz[Mi][Fi][1] = 0.0;
      matrixData.matrix_function_xyz[Mi][Fi][2] = 0.0;
      matrixData.matrix_port_map[0][Mi] = -1;
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                            SDCARD: LOAD MATRIX 

/* loads tagged, comma delimited data from a matrix file */

bool sdcard_load_matrix(fs::FS &fs, char * file) {

  sdcardData.is_reading = true;
  
  Serial.println("[sdcard] attempting to load file: " + String(file));
  // open file to read
  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file); 
  if (sdcardData.current_file) {
    while (sdcardData.current_file.available()) {
      // read line
      sdcardData.SBUFFER = "";
      memset(sdcardData.BUFFER, 0, 2048);
      sdcardData.SBUFFER = sdcardData.current_file.readStringUntil('\n');
      sdcardData.SBUFFER.toCharArray(sdcardData.BUFFER, sdcardData.SBUFFER.length()+1);
      if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));}
      // tag: r
      if (strncmp(sdcardData.BUFFER, "r", 1) == 0) {
        // ensure cleared
        memset(sdcardData.data_0, 0, 56); memset(sdcardData.data_1, 0, 56); memset(sdcardData.data_2, 0, 56);
        memset(sdcardData.data_3, 0, 56); memset(sdcardData.data_4, 0, 56); memset(sdcardData.data_5, 0, 56);
        memset(sdcardData.data_6, 0, 56); memset(sdcardData.data_7, 0, 56);
        validData.bool_data_0 = false;
        validData.bool_data_1 = false;
        // split line on delimiter
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        // matrix index
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_0, sdcardData.token);
        if (is_all_digits(sdcardData.data_0) == true) {validData.bool_data_0 = true;
          if (sysDebugData.verbose_file==true) {Serial.println("[Mi] [PASS] " +String(sdcardData.data_0));}
        }
        else {if (sysDebugData.verbose_file==true) {Serial.println("[Mi] [INVALID] " +String(sdcardData.data_0));}}
        // matrix function index
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_1, sdcardData.token);
        if (is_all_digits(sdcardData.data_1) == true) {validData.bool_data_1 = true;
          if (sysDebugData.verbose_file==true) {Serial.println("[Fi] [PASS] " +String(sdcardData.data_1));}
        }
        else {if (sysDebugData.verbose_file==true) {Serial.println("[Fi] [INVALID] " +String(sdcardData.data_1));}}
        // continue if we have valid index numbers
        if ((validData.bool_data_0 == true) && (validData.bool_data_1 == true)) {
          // matrix function name
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_2, sdcardData.token);
          memset(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], 0, 56);
          strcpy(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], sdcardData.data_2);
          if (sysDebugData.verbose_file==true) {Serial.println("[Fn] [MATRIX] " +String(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]));}
          // matrix function data: x
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_3, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_3) == true) {
            matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0] = atol(sdcardData.data_3);
            if (sysDebugData.verbose_file==true) {Serial.println("[X]  [MATRIX] " +String(matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0]));}
          }
          else {if (sysDebugData.verbose_file==true) {Serial.println("[X] [INVALID] " + String(sdcardData.data_3));}}
          // matrix function data: y
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_4, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_4) == true) {
            matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1] = atol(sdcardData.data_4);
            if (sysDebugData.verbose_file==true) {Serial.println("[Y]  [MATRIX] " +String(matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1]));}
          }
          else {if (sysDebugData.verbose_file==true) {Serial.println("[Y] [INVALID] " + String(sdcardData.data_4));}}
          // matrix function data: z
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_5, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_5) == true) {
            matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2] = atol(sdcardData.data_5);
            if (sysDebugData.verbose_file==true) {Serial.println("[Z]  [MATRIX] " +String(matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2]));}
          }
          else {if (sysDebugData.verbose_file==true) {Serial.println("[Z] [INVALID] " + String(sdcardData.data_5));}}
        }
      }
      // tag: e
      else if (strncmp(sdcardData.BUFFER, "e", 1) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        sdcardData.token = strtok(NULL, ",");
        sdcardData.token = strtok(NULL, ",");
        // enabled/disabled
        strcpy(sdcardData.data_6, sdcardData.token);
        if (is_all_digits(sdcardData.data_6) == true) {
          matrixData.matrix_switch_enabled[0][atoi(sdcardData.data_0)] = atoi(sdcardData.data_6);
          if (sysDebugData.verbose_file==true) {Serial.println("[E]  [MATRIX] " +String(matrixData.matrix_switch_enabled[0][atoi(sdcardData.data_0)]));}
          }
        else {if (sysDebugData.verbose_file==true) {Serial.println("[E]  [INVALID] " +String(sdcardData.data_6));}}
        // port
        sdcardData.token = strtok(NULL, ",");
        // check
        if (is_all_digits_plus_char(sdcardData.data_7, '-') == true) {
          strcpy(sdcardData.data_7, sdcardData.token);
          matrixData.matrix_port_map[0][atoi(sdcardData.data_0)] = atoi(sdcardData.data_7);
          if (sysDebugData.verbose_file==true) {Serial.println("[E]  [MATRIX] " +String(matrixData.matrix_port_map[0][atoi(sdcardData.data_0)]));}
          }
        else {if (sysDebugData.verbose_file==true) {Serial.println("[E]  [INVALID] " +String(sdcardData.data_7));}}
      }
    }
    // update current matrix filepath
    strcpy(sdcardData.tempmatrixfilepath, file);
    memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
    strcpy(sdcardData.matrix_filepath, sdcardData.tempmatrixfilepath);
    Serial.println("[sdcard] loaded file successfully:   " + String(file));
    Serial.println("[sdcard] sdcardData.matrix_filepath: " + String(sdcardData.matrix_filepath));
    sdcardData.current_file.close();
    sdcardData.is_reading = false;
    return true;
  }
  // update matrix filepath (clear)
  else {
    sdcardData.current_file.close();
    Serial.println("[sdcard] failed to load file: " + String(file));
    memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
    sdcardData.is_reading = false;
    return false;
    }
  sdcardData.is_reading = false;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                            SDCARD: SAVE MATRIX

/* saves tagged, comma delimited data to a matrix file */

bool sdcard_save_matrix(fs::FS &fs, char * file) {

  sdcardData.is_writing = true;

  Serial.println("[sdcard] attempting to save file: " + String(file));
  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file, FILE_WRITE);
  if (sdcardData.current_file) {
    for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
      for (int Fi = 0; Fi < matrixData.max_matrix_functions; Fi++) {
        memset(sdcardData.file_data, 0 , 256);
        // tag: matrix (r)
        strcat(sdcardData.file_data, sdcardData.tag_0); strcat(sdcardData.file_data, sdcardData.delim);
        // matrix index
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%d", Mi);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // matrix function index
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%d", Fi);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function name
        strcat(sdcardData.file_data, matrixData.matrix_function[Mi][Fi]); strcat(sdcardData.file_data, sdcardData.delim);
        // function value x
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", matrixData.matrix_function_xyz[Mi][Fi][0]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value y
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", matrixData.matrix_function_xyz[Mi][Fi][1]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value z
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", matrixData.matrix_function_xyz[Mi][Fi][2]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // write line
        if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
        sdcardData.current_file.println(sdcardData.file_data);
      }
      memset(sdcardData.file_data, 0 , 256);
      // tag: enable (e)
      strcat(sdcardData.file_data, sdcardData.tag_1); strcat(sdcardData.file_data, sdcardData.delim);
      // matrix index
      memset(sdcardData.tmp, 0 , 256);
      sprintf(sdcardData.tmp, "%d", Mi);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // matrix enabled 0/1
      memset(sdcardData.tmp, 0 , 256);
      itoa(matrixData.matrix_switch_enabled[0][Mi], sdcardData.tmp, 10);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // matrix switch port
      memset(sdcardData.tmp, 0 , 256);
      itoa(matrixData.matrix_port_map[0][Mi], sdcardData.tmp, 10);
      if (sysDebugData.verbose_file==true) {Serial.println("[check] " + String(matrixData.matrix_port_map[0][Mi]));}
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // write line
      if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
      sdcardData.current_file.println("");
      sdcardData.current_file.println(sdcardData.file_data);
      sdcardData.current_file.println("");
    }
    sdcardData.current_file.close();
    Serial.println("[sdcard] saved file successfully: " + String(file));
    strcpy(sdcardData.matrix_filepath, file);
    sdcardData.is_writing = false;
    return true;
  }
  else {sdcardData.current_file.close(); Serial.println("[sdcard] failed to save file: " + String(file));
  sdcardData.is_writing = false;
  return false;}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                     SDCARD: DELETE MATRIX FILE

void sdcard_delete_matrix(fs::FS &fs, char * file) {
  sdcardData.is_writing = true;
  // at least for now, do not allow deletion of MATRIX_0.SAVE.
  if (fs.exists(file)) {
    Serial.println("[sdcard] attempting to delete file: " + String(file));
    // try remove
    fs.remove(file);
    if (!fs.exists(file)) {
      Serial.println("[sdcard] successfully deleted file: " + String(file));
      Serial.println("attempting to remove filename from filenames.");
      // recreate matrix filenames
      sdcard_list_matrix_files(SD, "/MATRIX/", "MATRIX", ".SAVE");
      // zero the matrix
      zero_matrix();
      // delete matrix filepath.
      memset(sdcardData.matrix_filepath, 0, 56);
    }
    else {Serial.println("[sdcard] failed to deleted file: " + String(file));}
  }
  else {Serial.println("[sdcard] file does not exist: " + String(file));}
  sdcardData.is_writing = false;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                              MATRIX: SET ENTRY

/*
updates a matrix entry over serial.
example test command: $MATRIX_SET_ENTRY,0,0,SatelliteCountOver,1,0,0
*/

void matrix_object_set_entry() {
  Serial.println("[matrix_object_set_entry] connected");
  memset(serial0Data.data_0, 0, 56);
  memset(serial0Data.data_1, 0, 56);
  memset(serial0Data.data_2, 0, 56);
  memset(serial0Data.data_3, 0, 56);
  memset(serial0Data.data_4, 0, 56);
  memset(serial0Data.data_5, 0, 56);
  serial0Data.iter_token = 0;
  serial0Data.token = strtok(serial0Data.BUFFER, ",");
  while( serial0Data.token != NULL ) {
    if      (serial0Data.iter_token == 0) {}
    else if (serial0Data.iter_token == 1) {strcpy(serial0Data.data_0, serial0Data.token);} // rn
    else if (serial0Data.iter_token == 2) {strcpy(serial0Data.data_1, serial0Data.token);} // fn
    else if (serial0Data.iter_token == 3) {strcpy(serial0Data.data_2, serial0Data.token);} // function
    else if (serial0Data.iter_token == 4) {strcpy(serial0Data.data_3, serial0Data.token);} // x
    else if (serial0Data.iter_token == 5) {strcpy(serial0Data.data_4, serial0Data.token);} // y
    else if (serial0Data.iter_token == 6) {strcpy(serial0Data.data_5, serial0Data.token);} // z
    serial0Data.token = strtok(NULL, ",");
    serial0Data.iter_token++;
  }
  if (sysDebugData.serial_0_sentence == true) {
    Serial.println("[serial0Data.data_0] "         + String(serial0Data.data_0));
    Serial.println("[serial0Data.data_1] "         + String(serial0Data.data_1));
    Serial.println("[serial0Data.data_2] "         + String(serial0Data.data_2));
    Serial.println("[serial0Data.data_3] "         + String(serial0Data.data_3));
    Serial.println("[serial0Data.data_4] "         + String(serial0Data.data_4));
    Serial.println("[serial0Data.data_5] "         + String(serial0Data.data_5));
  }
  char *ptr;
  // set function
  strcpy(matrixData.matrix_function[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)], serial0Data.data_2);
  // set function value x
  matrixData.matrix_function_xyz[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]=strtod(serial0Data.data_3, &ptr);
  // set function value x
  matrixData.matrix_function_xyz[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]=strtod(serial0Data.data_4, &ptr);
  // set function value z
  matrixData.matrix_function_xyz[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]=strtod(serial0Data.data_5, &ptr);
  Serial.println("[Mi] " +String(serial0Data.data_0));
  Serial.println("[Fi] " +String(serial0Data.data_1));
  Serial.println("[Fn] " +String(matrixData.matrix_function[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)]));
  Serial.println("[X] " +String(matrixData.matrix_function_xyz[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]));
  Serial.println("[Y] " +String(matrixData.matrix_function_xyz[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]));
  Serial.println("[Z] " +String(matrixData.matrix_function_xyz[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]));
}


// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   MATRIX: ENABLE/DISABLE ENTRY

/*
updates a matrix entry over serial.
*/

void matriobject_set_enabled(bool b) {
  Serial.println("[matriobject_set_enabled] connected");
  memset(serial0Data.data_0, 0, 56);
  serial0Data.iter_token = 0;
  serial0Data.token = strtok(serial0Data.BUFFER, ",");
  while( serial0Data.token != NULL ) {
    if      (serial0Data.iter_token == 0) {}
    else if (serial0Data.iter_token == 1) {strcpy(serial0Data.data_0, serial0Data.token);} // 0/1
    serial0Data.token = strtok(NULL, ",");
    serial0Data.iter_token++;
  }
  if (sysDebugData.serial_0_sentence == true) {
    Serial.println("[serial0Data.data_0] "         + String(serial0Data.data_0));
  }
  matrixData.matrix_switch_enabled[0][atoi(serial0Data.data_0)] = b; // set enable/disable
  Serial.println(
    "[R] " +
    String(serial0Data.data_0) +
    " [E] " +
    String(matrixData.matrix_switch_enabled[0][atoi(serial0Data.data_0)]));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                            MATRIX: DISABLE ALL

/*
disable all matrix entries. does not turn matrices off. this allows for overriding a matrix switch without deactivating.
automatically deactivating a matrix when a matrix is made disabled should be carefully added as a feature and is differnt from
automatically/manually enabling/disabling. 
this is explicitly disable matrix switch from automatically activating/deactivating.
*/
void matrix_disable_all() {for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {matrixData.matrix_switch_enabled[0][Mi]=0;}}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             MATRIX: ENABLE ALL


/*
enable all matrix entries. does not directly turn matrix switches on, instead enables matrix switch to automatically
activate/deactivate.
*/
void matrix_enable_all() {for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {matrixData.matrix_switch_enabled[0][Mi]=1;}}


// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                         MATRIX: ALL MATRIX OFF

// turn all matrix switches off. recommended to first disable matrix switches from being automatically activated/deactivated.
void matrix_deactivate_all() {for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {matrixData.matrix_switch_state[0][Mi]=0;}}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                          MATRIX: ALL MATRIX ON

// turn all matrix switches on. recommended to first disable matrix switches from being automatically activated/deactivated.
void matrix_activate_all() {for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {matrixData.matrix_switch_state[0][Mi]=1;}}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                     SATIO: CONVERT COORDINATES

// enable/disable coordinate conversion. performance/efficiency as required.
void satio_convert_coordinates_on()  {satData.convert_coordinates = true;}
void satio_convert_coordinates_off() {satData.convert_coordinates = false;}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   MATRIX FUNCTIONS: PRIMITIVES

/*
matrix switch requires all checks to return true for a matrix to be active, therefore checks can be inverted as required, to
return true when otherwise a check would return false, which allows more flexibility.
*/

// calculate if n0 in (+- range/2) of n1
bool in_range_check_true(double n0, double n1, double r) {
  // Serial.println(
  //   "in_range_check_true: (n0 " +
  //   String(n0) +
  //   " >= n1 (" +
  //   String(n1) +
  //   " - r/2 " +
  //   String(r/2) +
  //   ")) && (n0 " +
  //   String(n0) +
  //   " <= n1 (" +
  //   String(n1) +
  //   " + r/2 " +
  //   String(r/2) +
  //   "))");
  if ((n0  >=  n1 - r/2) && (n0  <= n1 + r/2)) {return true;}
  else {return false;}
}

// calculate if n0 in (+- range/2) of n1
bool in_range_check_false(double n0, double n1, double r) {
  // Serial.println(
  //   "in_range_check_false: (n0 " +
  //   String(n0) +
  //   " >= n1 (" +
  //   String(n1) +
  //   " - r/2 " +
  //   String(r/2) +
  //   ")) && (n0 " +
  //   String(n0) +
  //   " <= n1 (" +
  //   String(n1) +
  //   " + r/2 " +
  //   String(r/2) +
  //   "))");
  if ((n0  >=  n1 - r/2) && (n0  <= n1 + r/2)) {return false;}
  else {return true;}
}

bool in_ranges_check_true(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r) == true) {
    if (in_range_check_true(y0, y1, r) == true) {return true;} else return false;}
  else {return false;}
}

bool in_ranges_check_false(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r) == true) {
    if (in_range_check_true(y0, y1, r) == true) {return false;} else return true;}
  else {return true;}
}

bool check_over_true(double n0, double n1) {
  Serial.println("check_over_true: n0 " + String(n0) + " > n1 " + String(n1));
  if (n0 > n1) {return true;}
  else {return false;}
}

bool check_over_false(double n0, double n1) {
  Serial.println("check_over_false: n0 " + String(n0) + " > n1 " + String(n1));
  if (n0 > n1) {return false;}
  else {return true;}
}

bool check_under_true(double n0, double n1) {
  // Serial.println("check_under_true: n0 " + String(n0) + " < n1 " + String(n1));
  if (n0 < n1) {return true;}
  else {return false;}
}

bool check_under_false(double n0, double n1) {
  // Serial.println("check_under_false: n0 " + String(n0) + " < n1 " + String(n1));
  if (n0 < n1) {return false;}
  else {return true;}
}

bool check_equal_true(double n0, double n1) {
  // Serial.println("check_equal_true: n0 " + String(n0) + " == n1 " + String(n1));
  if (n0 == n1) {return true;}
  else {return false;}
}

bool check_equal_false(double n0, double n1) {
  // Serial.println("check_equal_false: n0 " + String(n0) + " == n1 " + String(n1));
  if (n0 != n1) {return true;}
  else {return false;}
}

bool check_ge_and_le_true(double n0, double n1, double n2) {
  // Serial.println(
  //   "check_ge_and_le_true: n0 " +
  //   String(n0) +
  //   " >= n1 " +
  //   String(n1) +
  //   " && n0 " +
  //   String(n0) +
  //   " <= " +
  //   String(n2));
  if ((n0 >= n1) && (n0 <= n2)) {return true;}
  else {return false;}
}

bool check_ge_and_le_false(double n0, double n1, double n2) {
  // Serial.println(
  //   "check_ge_and_le_false: n0 " +
  //   String(n0) +
  //   " >= n1 " +
  //   String(n1) +
  //   " && n0 " +
  //   String(n0) +
  //   " <= " +
  //   String(n2));
  if ((n0 >= n1) && (n0 <= n2)) {return false;}
  else {return true;}
}

bool check_strncmp_true(char * c0, char * c1, int n) {
  // Serial.println("check_strncmp_true: c0 " + String(c0) + " == c1 " + String(c1) + " (n=" + String(n) + ")");
  if (strncmp(c0, c1, n) == 0) {return true;}
  else {return false;}
}

bool check_strncmp_false(char * c0, char * c1, int n) {
  // Serial.println("check_strncmp_false: c0 " + String(c0) + " == c1 " + String(c1) + " (n=" + String(n) + ")");
  if (strncmp(c0, c1, n) == 0) {return false;}
  else {return true;}
}

bool check_bool_true(bool _bool) {
  // Serial.println("check_bool_true: " + String(_bool));
  if (_bool == true) {return true;} else {return false;}
}

bool check_bool_false(bool _bool) {
  // Serial.println("check_bool_false: " + String(_bool));
  if (_bool == false) {return true;} else {return false;}
}

bool SecondsTimer(double n0, double n1, int Mi) {

  /*
  seconds accumulated by an isr alarm. this does not use satellite data. 
  
  x (n0): time interval
  y (n1): on time period
  */

  // turn on or remain off
  if (matrixData.matrix_switch_state[0][Mi] == 0) {
    if ((timeData.seconds - matrixData.matrix_timers[0][Mi]) < n0) {return false;}
    if ((timeData.seconds - matrixData.matrix_timers[0][Mi]) > n0) {matrixData.matrix_timers[0][Mi] = timeData.seconds; return true;}
    else {false;}
  }

  // turn off or remain on
  else if (matrixData.matrix_switch_state[0][Mi] == 1) {
    if      ((timeData.seconds - matrixData.matrix_timers[0][Mi]) < n1) {return true;}
    /*
    timer style: stacked time: y on time period is stacked on top of x time interval.
                 (1) total off time is x (x time interval effectively becomes an off time period).
                 (2) total on time is y.
                 (3) total on off time is x+y.
                 (4) considerations: harder to predict because on and off times will creep.
    */
    // else if ((timeData.seconds - matrixData.matrix_timers[0][Mi]) > n1) {matrixData.matrix_timers[0][Mi] = timeData.seconds; return false;}

    /*
    timer style: integrated time: y on time occurrs for a period within x time interval.
                 (1) total off time is x - y (time interval minus on time period).
                 (2) total on time is y.
                 (3) total on off time is x.
                 (4) considerations: take care no to overlap x and y to prevent always returning true or false.
                 
    */
    else if ((timeData.seconds - matrixData.matrix_timers[0][Mi]) > n1) {matrixData.matrix_timers[0][Mi] = timeData.seconds-n1; return false;}
    else {true;}
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                     MATRIX FUNCTIONS: ADVANCED

/*
Astronomy: Ra:  Right Ascension (ranges from 0 to 24 hours)
           Dec: Declination     (ranges from -90 to 90 degrees)
           Az:  Azimuth         (ranges from 0 to 360 degrees)
           Alt: Altitude        (ranges from -90 to 90 degrees)
           R:   Rise            (time)
           S:   Set             (time)
           P:   Phase
*/

struct SiderealPlantetsStruct {
  double sun_ra;
  double sun_dec;
  double sun_az;
  double sun_alt;
  double sun_r;
  double sun_s;
  double moon_ra;
  double moon_dec;
  double moon_az;
  double moon_alt;
  double moon_r;
  double moon_s;
  double moon_p;
  double mercury_ra;
  double mercury_dec;
  double mercury_az;
  double mercury_alt;
  double mercury_r;
  double mercury_s;
  double mercury_helio_ecliptic_lat;
  double mercury_helio_ecliptic_long;
  double mercury_radius_vector;
  double mercury_distance;
  double mercury_ecliptic_lat;
  double mercury_ecliptic_long;
  double venus_ra;
  double venus_dec;
  double venus_az;
  double venus_alt;
  double venus_r;
  double venus_s;
  double venus_helio_ecliptic_lat;
  double venus_helio_ecliptic_long;
  double venus_radius_vector;
  double venus_distance;
  double venus_ecliptic_lat;
  double venus_ecliptic_long;
  double mars_ra;
  double mars_dec;
  double mars_az;
  double mars_alt;
  double mars_r;
  double mars_s;
  double mars_helio_ecliptic_lat;
  double mars_helio_ecliptic_long;
  double mars_radius_vector;
  double mars_distance;
  double mars_ecliptic_lat;
  double mars_ecliptic_long;
  double jupiter_ra;
  double jupiter_dec;
  double jupiter_az;
  double jupiter_alt;
  double jupiter_r;
  double jupiter_s;
  double jupiter_helio_ecliptic_lat;
  double jupiter_helio_ecliptic_long;
  double jupiter_radius_vector;
  double jupiter_distance;
  double jupiter_ecliptic_lat;
  double jupiter_ecliptic_long;
  double saturn_ra;
  double saturn_dec;
  double saturn_az;
  double saturn_alt;
  double saturn_r;
  double saturn_s;
  double saturn_helio_ecliptic_lat;
  double saturn_helio_ecliptic_long;
  double saturn_radius_vector;
  double saturn_distance;
  double saturn_ecliptic_lat;
  double saturn_ecliptic_long;
  double uranus_ra;
  double uranus_dec;
  double uranus_az;
  double uranus_alt;
  double uranus_r;
  double uranus_s;
  double uranus_helio_ecliptic_lat;
  double uranus_helio_ecliptic_long;
  double uranus_radius_vector;
  double uranus_distance;
  double uranus_ecliptic_lat;
  double uranus_ecliptic_long;
  double neptune_ra;
  double neptune_dec;
  double neptune_az;
  double neptune_alt;
  double neptune_r;
  double neptune_s;
  double neptune_helio_ecliptic_lat;
  double neptune_helio_ecliptic_long;
  double neptune_radius_vector;
  double neptune_distance;
  double neptune_ecliptic_lat;
  double neptune_ecliptic_long;
};
SiderealPlantetsStruct siderealPlanetData;

struct SiderealObjectStruct {
  char object_name[56];
  char object_table_name[56];
  int  object_number;
  int  object_table_i;
  double object_ra;
  double object_dec;
  double object_az;
  double object_alt;
  double object_mag;
  double object_r;
  double object_s;
  // double objects_data[609][7];
  char object_table[7][20] =
  {
    "Star Table",          // 0
    "NGC Table",           // 1
    "IC Table",            // 2
    "Other Objects Table", // 3
    "Messier Table",       // 4
    "Caldwell Table",      // 5
    "Herschel 400 Table",  // 6
  };
};
SiderealObjectStruct siderealObjectData;

void trackObject(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second,
                 int object_table_i, int object_i) {
  myAstro.setLatLong(latitude, longitude);
  // myAstro.setTimeZone(tz);
  myAstro.rejectDST();
  myAstro.setGMTdate(year, month, day);
  myAstro.setLocalTime(hour, minute, second);
  myAstro.setGMTtime(hour, minute, second);
  if (object_table_i == 0) {myAstroObj.selectStarTable(object_i);}
  if (object_table_i == 1) {myAstroObj.selectNGCTable(object_i);}
  if (object_table_i == 2) {myAstroObj.selectICTable(object_i);}
  if (object_table_i == 3) {myAstroObj.selectMessierTable(object_i);}
  if (object_table_i == 4) {myAstroObj.selectCaldwellTable(object_i);}
  if (object_table_i == 5) {myAstroObj.selectHershel400Table(object_i);}
  if (object_table_i == 6) {myAstroObj.selectOtherObjectsTable(object_i);}
  myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
  myAstro.doRAdec2AltAz();
  siderealObjectData.object_az = myAstro.getAzimuth();
  siderealObjectData.object_alt = myAstro.getAltitude();
  siderealObjectData.object_r = myAstro.getRiseTime();
  siderealObjectData.object_s = myAstro.getSetTime();
}

void IdentifyObject(double object_ra, double object_dec) {
  myAstroObj.setRAdec(object_ra, object_dec);
  myAstro.doRAdec2AltAz();
  siderealObjectData.object_mag = myAstroObj.getStarMagnitude();
  myAstroObj.identifyObject();
  switch(myAstroObj.getIdentifiedObjectTable()) {
  case(1):
  siderealObjectData.object_table_i = 0; break;
	case(2):
  siderealObjectData.object_table_i = 1; break;
	case(3):
  siderealObjectData.object_table_i = 2;  break;
	case(7):
  siderealObjectData.object_table_i = 3;  break;
  }
  if (myAstroObj.getIdentifiedObjectTable() == 1) {
    // set table name
    memset(siderealObjectData.object_table_name, 0, 56);
    strcpy(siderealObjectData.object_table_name, siderealObjectData.object_table[siderealObjectData.object_table_i]);
    // set object id name
    memset(siderealObjectData.object_name, 0, 56);
    strcpy(siderealObjectData.object_name, myAstroObj.printStarName(myAstroObj.getIdentifiedObjectNumber()));
    }
  if (myAstroObj.getAltIdentifiedObjectTable()) {
    switch(myAstroObj.getAltIdentifiedObjectTable()) {
	  casematrix_indi_h:
    siderealObjectData.object_table_i = 4;  break;
	  case(5):
    siderealObjectData.object_table_i = 5;  break;
	  case(6):
    siderealObjectData.object_table_i = 6;  break;
    }
  // set table name
  memset(siderealObjectData.object_table_name, 0, 56);
  strcpy(siderealObjectData.object_table_name, siderealObjectData.object_table[siderealObjectData.object_table_i]);
  // set object id number
  siderealObjectData.object_number = myAstroObj.getAltIdentifiedObjectNumber();
  }
}


void trackSun() {
  myAstro.doSun();
  siderealPlanetData.sun_ra  = myAstro.getRAdec();
  siderealPlanetData.sun_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.sun_az  = myAstro.getAzimuth();
  siderealPlanetData.sun_alt = myAstro.getAltitude();
  myAstro.doSunRiseSetTimes();
  siderealPlanetData.sun_r  = myAstro.getSunriseTime();
  siderealPlanetData.sun_s  = myAstro.getSunsetTime();
}

void trackMoon() {
  myAstro.doSun();
  siderealPlanetData.moon_ra  = myAstro.getRAdec();
  siderealPlanetData.moon_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.moon_az  = myAstro.getAzimuth();
  siderealPlanetData.moon_alt = myAstro.getAltitude();
  myAstro.doMoonRiseSetTimes();
  siderealPlanetData.moon_r  = myAstro.getMoonriseTime();
  siderealPlanetData.moon_s  = myAstro.getMoonsetTime();
  siderealPlanetData.moon_p  = myAstro.getMoonPhase();
}

void trackMercury() {
  myAstro.doMercury();
  siderealPlanetData.mercury_ra  = myAstro.getRAdec();
  siderealPlanetData.mercury_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.mercury_az  = myAstro.getAzimuth();
  siderealPlanetData.mercury_alt = myAstro.getAltitude();
  siderealPlanetData.mercury_helio_ecliptic_lat = myAstro.getHelioLat();
  siderealPlanetData.mercury_helio_ecliptic_long = myAstro.getHelioLong();
  siderealPlanetData.mercury_radius_vector = myAstro.getRadiusVec();
  siderealPlanetData.mercury_distance = myAstro.getDistance();
  siderealPlanetData.mercury_ecliptic_lat = myAstro.getEclipticLatitude();
  siderealPlanetData.mercury_ecliptic_long = myAstro.getEclipticLongitude();
  myAstro.doXRiseSetTimes();
  siderealPlanetData.mercury_r = myAstro.getRiseTime();
  siderealPlanetData.mercury_s = myAstro.getSetTime();
}

void trackVenus() {
  myAstro.doVenus();
  siderealPlanetData.venus_ra  = myAstro.getRAdec();
  siderealPlanetData.venus_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.venus_az  = myAstro.getAzimuth();
  siderealPlanetData.venus_alt = myAstro.getAltitude();
  siderealPlanetData.venus_helio_ecliptic_lat = myAstro.getHelioLat();
  siderealPlanetData.venus_helio_ecliptic_long = myAstro.getHelioLong();
  siderealPlanetData.venus_radius_vector = myAstro.getRadiusVec();
  siderealPlanetData.venus_distance = myAstro.getDistance();
  siderealPlanetData.venus_ecliptic_lat = myAstro.getEclipticLatitude();
  siderealPlanetData.venus_ecliptic_long = myAstro.getEclipticLongitude();
  myAstro.doXRiseSetTimes();
  siderealPlanetData.venus_r = myAstro.getRiseTime();
  siderealPlanetData.venus_s = myAstro.getSetTime();
}

void trackMars() {
  myAstro.doMars();
  siderealPlanetData.mars_ra  = myAstro.getRAdec();
  siderealPlanetData.mars_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.mars_az  = myAstro.getAzimuth();
  siderealPlanetData.mars_alt = myAstro.getAltitude();
  siderealPlanetData.mars_helio_ecliptic_lat = myAstro.getHelioLat();
  siderealPlanetData.mars_helio_ecliptic_long = myAstro.getHelioLong();
  siderealPlanetData.mars_radius_vector = myAstro.getRadiusVec();
  siderealPlanetData.mars_distance = myAstro.getDistance();
  siderealPlanetData.mars_ecliptic_lat = myAstro.getEclipticLatitude();
  siderealPlanetData.mars_ecliptic_long = myAstro.getEclipticLongitude();
  myAstro.doXRiseSetTimes();
  siderealPlanetData.mars_r = myAstro.getRiseTime();
  siderealPlanetData.mars_s = myAstro.getSetTime();
}

void trackJupiter() {
  myAstro.doJupiter();
  siderealPlanetData.jupiter_ra  = myAstro.getRAdec();
  siderealPlanetData.jupiter_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.jupiter_az  = myAstro.getAzimuth();
  siderealPlanetData.jupiter_alt = myAstro.getAltitude();
  siderealPlanetData.jupiter_helio_ecliptic_lat = myAstro.getHelioLat();
  siderealPlanetData.jupiter_helio_ecliptic_long = myAstro.getHelioLong();
  siderealPlanetData.jupiter_radius_vector = myAstro.getRadiusVec();
  siderealPlanetData.jupiter_distance = myAstro.getDistance();
  siderealPlanetData.jupiter_ecliptic_lat = myAstro.getEclipticLatitude();
  siderealPlanetData.jupiter_ecliptic_long = myAstro.getEclipticLongitude();
  myAstro.doXRiseSetTimes();
  siderealPlanetData.jupiter_r = myAstro.getRiseTime();
  siderealPlanetData.jupiter_s = myAstro.getSetTime();
}

void trackSaturn() {
  myAstro.doSaturn();
  siderealPlanetData.saturn_ra  = myAstro.getRAdec();
  siderealPlanetData.saturn_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.saturn_az  = myAstro.getAzimuth();
  siderealPlanetData.saturn_alt = myAstro.getAltitude();
  siderealPlanetData.saturn_helio_ecliptic_lat = myAstro.getHelioLat();
  siderealPlanetData.saturn_helio_ecliptic_long = myAstro.getHelioLong();
  siderealPlanetData.saturn_radius_vector = myAstro.getRadiusVec();
  siderealPlanetData.saturn_distance = myAstro.getDistance();
  siderealPlanetData.saturn_ecliptic_lat = myAstro.getEclipticLatitude();
  siderealPlanetData.saturn_ecliptic_long = myAstro.getEclipticLongitude();
  myAstro.doXRiseSetTimes();
  siderealPlanetData.saturn_r = myAstro.getRiseTime();
  siderealPlanetData.saturn_s = myAstro.getSetTime();
}

void trackUranus() {
  myAstro.doUranus();
  siderealPlanetData.uranus_ra  = myAstro.getRAdec();
  siderealPlanetData.uranus_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.uranus_az  = myAstro.getAzimuth();
  siderealPlanetData.uranus_alt = myAstro.getAltitude();
  siderealPlanetData.uranus_helio_ecliptic_lat = myAstro.getHelioLat();
  siderealPlanetData.uranus_helio_ecliptic_long = myAstro.getHelioLong();
  siderealPlanetData.uranus_radius_vector = myAstro.getRadiusVec();
  siderealPlanetData.uranus_distance = myAstro.getDistance();
  siderealPlanetData.uranus_ecliptic_lat = myAstro.getEclipticLatitude();
  siderealPlanetData.uranus_ecliptic_long = myAstro.getEclipticLongitude();
  myAstro.doXRiseSetTimes();
  siderealPlanetData.uranus_r = myAstro.getRiseTime();
  siderealPlanetData.uranus_s = myAstro.getSetTime();
}

void trackNeptune() {
  myAstro.doNeptune();
  siderealPlanetData.neptune_ra  = myAstro.getRAdec();
  siderealPlanetData.neptune_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.neptune_az  = myAstro.getAzimuth();
  siderealPlanetData.neptune_alt = myAstro.getAltitude();
  siderealPlanetData.neptune_helio_ecliptic_lat = myAstro.getHelioLat();
  siderealPlanetData.neptune_helio_ecliptic_long = myAstro.getHelioLong();
  siderealPlanetData.neptune_radius_vector = myAstro.getRadiusVec();
  siderealPlanetData.neptune_distance = myAstro.getDistance();
  siderealPlanetData.neptune_ecliptic_lat = myAstro.getEclipticLatitude();
  siderealPlanetData.neptune_ecliptic_long = myAstro.getEclipticLongitude();
  myAstro.doXRiseSetTimes();
  siderealPlanetData.neptune_r = myAstro.getRiseTime();
  siderealPlanetData.neptune_s = myAstro.getSetTime();
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   TASK: PLANETARY CALCULATIONS

void trackPlanets() {
  if (systemData.sidereal_track_sun == true) {trackSun();}
  if (systemData.sidereal_track_moon == true) {trackMoon();}
  if (systemData.sidereal_track_mercury == true) {trackMercury();}
  if (systemData.sidereal_track_venus == true) {trackVenus();}
  if (systemData.sidereal_track_mars == true) {trackMars();}
  if (systemData.sidereal_track_jupiter == true) {trackJupiter();}
  if (systemData.sidereal_track_saturn == true) {trackSaturn();}
  if (systemData.sidereal_track_uranus == true) {trackUranus();}
  if (systemData.sidereal_track_neptune == true) {trackNeptune();}
}

void setTrackPlanets() {
  myAstro.setLatLong(satData.location_latitude_gngga, satData.location_longitude_gngga);
  myAstro.rejectDST();
  myAstro.setGMTdate(rtc.now().year(), rtc.now().month(), rtc.now().day());
  myAstro.setLocalTime(rtc.now().hour(), rtc.now().minute(), rtc.now().second());
  myAstro.setGMTtime(rtc.now().hour(), rtc.now().minute(), rtc.now().second());
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 MATRIX: SWITCH

void matrixSwitch() {

  /*
  compound condition checks, each resulting in zero/one at the final_bool.
  */

  // iterate through matrices
  for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
    // Serial.println("[Mi] " + String(Mi) + " [E] " + String(matrixData.matrix_switch_enabled[0][Mi]));
    if (matrixData.matrix_switch_enabled[0][Mi] == 1) {

      /*
      temporary switch must be zero each time
      */
      bool tmp_matrix[matrixData.max_matrix_functions] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      int count_none_function = 0;

      // iterate over each function name in the current matrix
      for (int Fi = 0; Fi < matrixData.max_matrix_functions; Fi++) {

        // uncomment to debug
        // Serial.println("[Mi] " + String(Mi));
        // Serial.println("[Fi] " + String(Fi));
        // Serial.println("[matrixData.matrix_function[Mi][Fi]] " + String(matrixData.matrix_function[Mi][Fi]));

        /*
        perfromance and logic prefers adding functions from position zero else if position zero $NONE then break.
        */
        if ((strcmp(matrixData.matrix_function[Mi][Fi], matrixData.default_matrix_function) == 0) && (Fi == 0)) {break;}

        /*
        put true in temporary matrix for functions after position zero that are set to $NONE. allows for 1-10 functions to be set.
        */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.default_matrix_function) == 0) {
          tmp_matrix[Fi] = 1; count_none_function++;}

        /*
        put true in temporary matrix if switch is $ENABLED (different from enabling disabling) regardless of data. if used,
        function name $ENABLED will always return true.
        */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.default_enable_matrix_function) == 0) {tmp_matrix[Fi] = 1;}

        /* a special pair of switches to combine with logic that requires timing be below any specified overload max */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.OVERLOAD_TRUE) == 0) {
          tmp_matrix[Fi] = check_bool_true(systemData.overload);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.OVERLOAD_FALSE) == 0) {
          tmp_matrix[Fi] = check_bool_false(systemData.overload);}

        /*
         Special Switch Link Function: Mirrors/inverts switch X state (on/off) for switch using SwitchLink function. benefits:
         gain 9+ (over original 10) functions on a switch, simple inverted logic, logic expansion, etc. 
        */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SwitchLinkTrue) == 0) {
          tmp_matrix[Fi] = check_equal_true(matrixData.matrix_switch_state[0][(int)matrixData.matrix_function_xyz[Mi][Fi][0]], 1);}
          
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SwitchLinkFalse) == 0) {
          tmp_matrix[Fi] = check_equal_false(matrixData.matrix_switch_state[0][(int)matrixData.matrix_function_xyz[Mi][Fi][0]], 1);}

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                              TIME DATA

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SecondsTimer) == 0) {
          tmp_matrix[Fi] = SecondsTimer(matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1], Mi);
          }
        

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RTCTimeOver) == 0) {
          tmp_matrix[Fi] = check_over_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RTCTimeUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
  
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RTCTimeEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
  
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RTCTimeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DaySunday) == 0) {
          if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Sunday")==0) {tmp_matrix[Fi] = 1;}}

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DayMonday) == 0) {
          if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Monday")==0) {tmp_matrix[Fi] = 1;}}

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DayTuesday) == 0) {
          if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Tuesday")==0) {tmp_matrix[Fi] = 1;}}
          
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DayWednesday) == 0) {
          if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Wednesday")==0) {tmp_matrix[Fi] = 1;}}

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DayThursday) == 0) {
          if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Thursday")==0) {tmp_matrix[Fi] = 1;}}

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DayFriday) == 0) {
          if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Friday")==0) {tmp_matrix[Fi] = 1;}}

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DaySaturday) == 0) {
          if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Saturday")==0) {tmp_matrix[Fi] = 1;}}

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DateDayX) == 0) {
          tmp_matrix[Fi] = check_equal_true(rtc.now().day(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DateMonthX) == 0) {
          tmp_matrix[Fi] = check_equal_true(rtc.now().month(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DateYearX) == 0) {
          tmp_matrix[Fi] = check_equal_true(rtc.now().year(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  SATIO

        // GNGGA (requires satData.coordinate_conversion_mode gngga)

        // over
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNGGAOver) == 0) {
          tmp_matrix[Fi] = check_over_true(satData.location_latitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNGGAOver) == 0) {
          tmp_matrix[Fi] = check_over_true(satData.location_longitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // under
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNGGAUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(satData.location_longitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNGGAUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(satData.location_latitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // equal
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNGGAEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNGGAEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // range
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNGGARange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNGGARange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0], matrixData.matrix_function_xyz[Mi][Fi][2]);
          
          }
        // ranges
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesGNGGARanges) == 0) {
          tmp_matrix[Fi] = in_ranges_check_true(satData.location_latitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          satData.location_longitude_gngga,
          matrixData.matrix_function_xyz[Mi][Fi][1],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        
        // GNRMC (requires satData.coordinate_conversion_mode gnrmc)

        // over
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(satData.location_latitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(satData.location_longitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // under
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(satData.location_latitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(satData.location_longitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // equal
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // range
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLatGNRMCRange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesLonGNRMCRange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        // ranges
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DegreesGNRMCRanges) == 0) {
          tmp_matrix[Fi] = in_ranges_check_true(satData.location_latitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          satData.location_longitude_gnrmc,
          matrixData.matrix_function_xyz[Mi][Fi][1],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  GNGGA

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNGGAOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnggaData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNGGAUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnggaData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNGGAEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnggaData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNGGARange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNGGAOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnggaData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNGGAUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnggaData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNGGAEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnggaData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNGGARange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNGGAOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnggaData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNGGAUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnggaData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNGGAEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnggaData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNGGARange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PositioningStatusGNGGA) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnggaData.solution_status),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SatelliteCountOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnggaData.satellite_count_gngga),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SatelliteCountUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnggaData.satellite_count_gngga),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SatelliteCountEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnggaData.satellite_count_gngga),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SatelliteCountRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.satellite_count_gngga),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNGGANorth) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "N", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNGGAEast) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "E", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNGGASouth) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "S", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNGGAWest) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "W", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPSPrecisionOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnggaData.hdop_precision_factor),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPSPrecisionUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnggaData.hdop_precision_factor),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPSPrecisionEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnggaData.hdop_precision_factor),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPSPrecisionRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.hdop_precision_factor),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.AltitudeGNGGAOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnggaData.altitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.AltitudeGNGGAUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnggaData.altitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.AltitudeGNGGAEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnggaData.altitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.AltitudeGNGGARange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.altitude),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  GNRMC

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCTimeGNRMCRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_time),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnrmcData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnrmcData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LatGNRMCRange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.latitude),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnrmcData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnrmcData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LonGNRMCRange) == 0) {
          tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.longitude),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNRMCNorth) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "N", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNRMCEast) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "E", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNRMCSouth) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "S", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HemisphereGNRMCWest) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "W", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GroundSpeedGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_speed),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GroundSpeedGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_speed),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GroundSpeedGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_speed),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GroundSpeedGNRMCRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_speed),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HeadingGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_heading),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HeadingGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_heading),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HeadingGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_heading),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.HeadingGNRMCRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_heading),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCDateGNRMCOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_date),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCDateGNRMCUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_date),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCDateGNRMCEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_date),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UTCDateGNRMCRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_date),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PositioningStatusGNRMCA) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "A", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi],matrixData.PositioningStatusGNRMCV) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "V", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.ModeGNRMCA) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "A", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.ModeGNRMCD) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "D", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.ModeGNRMCE) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "E", 1);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.ModeGNRMCN) == 0) {
          tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "N", 1);
          }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  GPATT

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PitchGPATTOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gpattData.pitch),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PitchGPATTUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gpattData.pitch),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PitchGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.pitch),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PitchGPATTRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.pitch),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RollGPATTOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gpattData.roll),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RollGPATTUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gpattData.roll),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RollGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.roll),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RollGPATTRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.roll),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.YawGPATTOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gpattData.yaw),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.YawGPATTUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gpattData.yaw),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.YawGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.yaw),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.YawGPATTRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.yaw),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GSTDataGPATTOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gpattData.gst_data),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GSTDataGPATTUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gpattData.gst_data),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GSTDataGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.gst_data),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GSTDataGPATTRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.gst_data),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MileageGPATTOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gpattData.mileage),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MileageGPATTUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gpattData.mileage),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MileageGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.mileage),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MileageGPATTRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.mileage),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SpeedNumGPATTOver) == 0) {
          tmp_matrix[Fi] = check_over_true(atol(gpattData.speed_num),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SpeedNumGPATTUnder) == 0) {
          tmp_matrix[Fi] = check_under_true(atol(gpattData.speed_num),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }  

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SpeedNumGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.speed_num),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SpeedNumGPATTRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.speed_num),
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.LineFlagGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.line_flag),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.INSGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.ins),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.RunStateFlagGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.run_state_flag),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.StaticFlagGPATTEqual) == 0) {
          tmp_matrix[Fi] = check_equal_true(atol(gpattData.static_flag),
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                     SIDEREAL TIME: SUN

        // sun azimuth:
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SunAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        // sun altitude:
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SunAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        // daytime: current time in range of sunrise and sunset
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DayTime) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.sun_r, siderealPlanetData.sun_s);
          }

        // nighttime: current time not in range of sunrise and sunset
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.NightTime) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.sun_r,
          siderealPlanetData.sun_s);
          }

        // sunrise time less than current time: true after sunrise until midnight
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Sunrise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.sun_r, hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        // sunset time less than current time: true after sunset until midnight                                                                  
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Sunset) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.sun_s, hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                                 SIDEREAL TIME: MOON

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MoonAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MoonAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Moonrise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.moon_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Moonset) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.moon_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MoonUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.moon_r,
          siderealPlanetData.moon_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MoonDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.moon_r,
          siderealPlanetData.moon_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MoonPhase) == 0) {
          tmp_matrix[Fi] = check_equal_true(siderealPlanetData.moon_p,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                              SIDEREAL TIME: MERCURY

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MercuryAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MercuryAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MercuryRise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.mercury_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MercurySet) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.mercury_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MercuryUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.mercury_r,
          siderealPlanetData.mercury_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MercuryDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.mercury_r,
          siderealPlanetData.mercury_s);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                                SIDEREAL TIME: VENUS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.VenusAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.VenusAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.VenusRise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.venus_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.VenusSet) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.venus_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.VenusUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.venus_r, 
          siderealPlanetData.venus_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.VenusDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.venus_r,
          siderealPlanetData.venus_s);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                                 SIDEREAL TIME: MARS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MarsAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MarsAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MarsRise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.mars_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MarsSet) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.mars_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MarsUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.mars_r,
          siderealPlanetData.mars_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.MarsDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.mars_r,
          siderealPlanetData.mars_s);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                              SIDEREAL TIME: JUPITER

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.JupiterAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.JupiterAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.JupiterRise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.jupiter_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.JupiterSet) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.jupiter_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.JupiterUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.jupiter_r,
          siderealPlanetData.jupiter_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.JupiterDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.jupiter_r,
          siderealPlanetData.jupiter_s);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                               SIDEREAL TIME: SATURN

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SaturnAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SaturnAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SaturnRise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.saturn_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SaturnSet) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.saturn_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SaturnUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.saturn_r,
          siderealPlanetData.saturn_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.SaturnDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.saturn_r,
          siderealPlanetData.saturn_s);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                               SIDEREAL TIME: URANUS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UranusAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UranusAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UranusRise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.uranus_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UranusSet) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.uranus_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UranusUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.uranus_r,
          siderealPlanetData.uranus_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.UranusDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.uranus_r,
          siderealPlanetData.uranus_s);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                              SIDEREAL TIME: NEPTUNE

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.NeptuneAzimuthRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_az,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.NeptuneAltitudeRange) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_alt,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][2]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.NeptuneRise) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.neptune_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.NeptuneSet) == 0) {
          tmp_matrix[Fi] = check_under_true(siderealPlanetData.neptune_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.NeptuneUp) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.neptune_r,
          siderealPlanetData.neptune_s);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.NeptuneDown) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
          siderealPlanetData.neptune_r,
          siderealPlanetData.neptune_s);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                       DHT11_0 HUMIDITY
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_H_Over) == 0) {
          tmp_matrix[Fi] = check_over_true(sensorData.dht11_h_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_H_Under) == 0) {
          tmp_matrix[Fi] = check_under_true(sensorData.dht11_h_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_H_Equal) == 0) {
          tmp_matrix[Fi] = check_equal_true(sensorData.dht11_h_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_H_Range) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_h_0,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                        DHT11_0 CELSIUS
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_C_Over) == 0) {
          tmp_matrix[Fi] = check_over_true(sensorData.dht11_c_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_C_Under) == 0) {
          tmp_matrix[Fi] = check_under_true(sensorData.dht11_c_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_C_Equal) == 0) {
          tmp_matrix[Fi] = check_equal_true(sensorData.dht11_c_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_C_Range) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_c_0,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                     DHT11_0 FAHRENHEIT
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_F_Over) == 0) {
          tmp_matrix[Fi] = check_over_true(sensorData.dht11_f_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_F_Under) == 0) {
          tmp_matrix[Fi] = check_under_true(sensorData.dht11_f_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_F_Equal) == 0) {
          tmp_matrix[Fi] = check_equal_true(sensorData.dht11_f_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_F_Range) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_f_0,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                             DHT11_0 HEAT INDEX CELSIUS
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIC_Over) == 0) {
          tmp_matrix[Fi] = check_over_true(sensorData.dht11_hic_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIC_Under) == 0) {
          tmp_matrix[Fi] = check_under_true(sensorData.dht11_hic_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIC_Equal) == 0) {
          tmp_matrix[Fi] = check_equal_true(sensorData.dht11_hic_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIC_Range) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_hic_0,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                          DHT11_0 HEAT INDEX FAHRENHEIT
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIF_Over) == 0) {
          tmp_matrix[Fi] = check_over_true(sensorData.dht11_hif_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIF_Under) == 0) {
          tmp_matrix[Fi] = check_under_true(sensorData.dht11_hif_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIF_Equal) == 0) {
          tmp_matrix[Fi] = check_equal_true(sensorData.dht11_hif_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.DHT11_0_HIF_Range) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_hif_0,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                        PHOTO RESISTORS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PhotoResistor_0_Under) == 0) {
          tmp_matrix[Fi] = check_under_true(sensorData.photoresistor_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PhotoResistor_0_Over) == 0) {
          tmp_matrix[Fi] = check_over_true(sensorData.photoresistor_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PhotoResistor_0_Equal) == 0) {
          tmp_matrix[Fi] = check_equal_true(sensorData.photoresistor_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.PhotoResistor_0_Range) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(sensorData.photoresistor_0,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               TRACKING

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Tracking_0_Under) == 0) {
          tmp_matrix[Fi] = check_under_true(sensorData.tracking_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Tracking_0_Over) == 0) {
          tmp_matrix[Fi] = check_over_true(sensorData.tracking_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Tracking_0_Equal) == 0) {
          tmp_matrix[Fi] = check_equal_true(sensorData.tracking_0,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.Tracking_0_Range) == 0) {
          tmp_matrix[Fi] = check_ge_and_le_true(sensorData.tracking_0,
          matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1]);
          }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               VALIDITY

        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNGGAValidChecksum) == 0) {
          tmp_matrix[Fi] = check_bool_true(gnggaData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNGGAInvalidChecksum) == 0) {
          tmp_matrix[Fi] = check_bool_false(gnggaData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNRMCValidChecksum) == 0) {
          tmp_matrix[Fi] = check_bool_true(gnrmcData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNRMCInvalidChecksum) == 0) {
          tmp_matrix[Fi] = check_bool_false(gnrmcData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPATTValidChecksum) == 0) {
          tmp_matrix[Fi] = check_bool_true(gpattData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPATTInvalidChecksum) == 0) {
          tmp_matrix[Fi] = check_bool_false(gpattData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNGGAValidCheckData) == 0) {
          tmp_matrix[Fi] = check_equal_true(gnggaData.check_data, 16);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNGGAInvalidCheckData) == 0) {
          tmp_matrix[Fi] = check_equal_false(gnggaData.check_data, 16);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNRMCValidCheckData) == 0) {
          tmp_matrix[Fi] = check_equal_true(gnrmcData.check_data, 14);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GNRMCInvalidCheckData) == 0) {
          tmp_matrix[Fi] = check_equal_false(gnrmcData.check_data, 14);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPATTValidCheckData) == 0) {
          tmp_matrix[Fi] = check_equal_true(gpattData.check_data, 41);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], matrixData.GPATTInvalidCheckData) == 0) {
          tmp_matrix[Fi] = check_equal_false(gpattData.check_data, 41);}
      }

      // ----------------------------------------------------------------------------------------------------------------------
      //                                                                                                           FINAL SWITCH
      
      /*
      safety layer: disengage if all entries are $NONE.
      this is a second layer on top of initial check for $NONE set at position zero, function 0.
      */
      if (count_none_function <= matrixData.max_matrix_functions-1) {

        /*
        it all comes down to this, the final switch.
        default final bool default is true: if a single false is found then final bool should be set to false and remain false.
        prevent/allow the matrix switch to activate.
        */
        bool final_bool = true;
        
        // debug (same as line below but with output)
        // for (int FC = 0; FC < matrixData.max_matrix_functions-1; FC++) {
        //   Serial.println("[tmp_matrix[FC]] " + String(tmp_matrix[FC])); if (tmp_matrix[FC] == 0) {final_bool = false;}}

        for (int FC = 0; FC < matrixData.max_matrix_functions-1; FC++) {if (tmp_matrix[FC] == 0) {final_bool = false; break;}}

        /*
        WARNING: why do you think you can trust the data you are receiving?
                 once you plug something into this, the 'satellites' are in control unless you have a way to override.

                 critical systems: arduino is neither medical nor military grade.
        */

        // debug (same as line below but with output)
        // if (final_bool == false) {Serial.println("[matrix " + String(Mi) + "] inactive"); matrixData.matrix_switch_state[0][Mi] = 0;}
        // else if (final_bool == true) {Serial.println("[matrix " + String(Mi) + "] active"); matrixData.matrix_switch_state[0][Mi] = 1;}

        /* a short call to the port controller each iteration may be made here */

        if (final_bool == false) {matrixData.matrix_switch_state[0][Mi] = 0;}
        else if (final_bool == true) {matrixData.matrix_switch_state[0][Mi] = 1;}
      }
      else {Serial.println("[matrix " + String(Mi) + "] WARNING: Matrix checks are enabled for an non configured matrix!");}
    }
    // handle Mi's that are disbaled.
    else {matrixData.matrix_switch_state[0][Mi] = 0;}

    // reset matrix switch state sentence.
    memset(matrixData.matrix_sentence, 0, sizeof(matrixData.matrix_sentence));
    strcpy(matrixData.matrix_sentence, "$MATRIX,");

    // append port mapping data
    for (int i=0; i < matrixData.max_matrices; i++) {
      itoa(matrixData.matrix_port_map[0][i], matrixData.temp, 10);
      strcat(matrixData.matrix_sentence, matrixData.temp);
      strcat(matrixData.matrix_sentence, ",");
      }
    
    // append matrix switch state data
    for (int i=0; i < matrixData.max_matrices; i++) {
      if      (matrixData.matrix_switch_state[0][i] == 0) {strcat(matrixData.matrix_sentence, "0,");}
      else if (matrixData.matrix_switch_state[0][i] == 1) {strcat(matrixData.matrix_sentence, "1,");}
    }

    // Satellite Count and HDOP Precision Factor Indicator
    if (atoi(gnggaData.satellite_count_gngga)==0) {strcat(matrixData.matrix_sentence, "0,");}
    else if ((atoi(gnggaData.satellite_count_gngga)>0) && (atof(gnggaData.hdop_precision_factor)>1.0)) {strcat(matrixData.matrix_sentence, "1,");}
    else if ((atoi(gnggaData.satellite_count_gngga)>0) && (atof(gnggaData.hdop_precision_factor)<=1.0)) {strcat(matrixData.matrix_sentence, "2,");}

    // Overload Indicator
    if (systemData.overload==false) {strcat(matrixData.matrix_sentence, "0,");}
    else {strcat(matrixData.matrix_sentence, "1,");}

    // append checksum
    createChecksum(matrixData.matrix_sentence);
    strcat(matrixData.matrix_sentence, "*");
    strcat(matrixData.matrix_sentence, SerialLink.checksum);
    strcat(matrixData.matrix_sentence, "\n");

    // serial output: switch states.
    if (systemData.output_matrix_enabled == true) {
      Serial.println(matrixData.matrix_sentence);
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     READ RXD 0

void readSerialCommands() {
    
  if (Serial.available() > 0) {
    
    memset(serial0Data.BUFFER, 0, 2000);
    serial0Data.nbytes = (Serial.readBytesUntil('\n', serial0Data.BUFFER, sizeof(serial0Data.BUFFER)));
    // Serial.println(serial0Data.nbytes); // debug

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                                          MATRIX: SET ENTRY

    if (strncmp(serial0Data.BUFFER, "$MATRIX_SET_ENTRY", 17) == 0) {
      matrix_object_set_entry();
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                               MATRIX: ENABLE/DISABLE ENTRY

    else if (strncmp(serial0Data.BUFFER, "$MATRIX_ENABLE_ENTRY", 19) == 0) {
      matriobject_set_enabled(true);
    }

    else if (strncmp(serial0Data.BUFFER, "$MATRIX_DISABLE_ENTRY", 21) == 0) {
      matriobject_set_enabled(false);
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                                        MATRIX: DISABLE ALL

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_DISABLE_ALL") == 0) {
      matrix_disable_all();
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                                         MATRIX: ENABLE ALL

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_ENABLE_ALL") == 0) {
      matrix_enable_all();
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                                 MATRIX: TURN ALL matrix ON

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_matrix_ALL_ON") == 0) {
      matrix_activate_all();
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                                MATRIX: TURN ALL matrix OFF

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_matrix_ALL_OFF") == 0) {
      matrix_deactivate_all();
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                           SDCARD: SERIAL PRINT MATRIX FILE

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_READ_MATRIX") == 0) {
      sdcard_read_to_serial(SD, sdcardData.matrix_filepath);
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                              SDCARD: SAVE MATRIX TO SDCARD

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_SAVE_MATRIX") == 0) {
      sdcard_save_matrix(SD, sdcardData.matrix_filepath);
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                            SDCARD: LOAD MATRIX FROM SDCARD

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_LOAD_MATRIX") == 0) {
      sdcard_load_matrix(SD, sdcardData.matrix_filepath);
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                                 SATIO: CONVERT COORDINATES

    else if (strcmp(serial0Data.BUFFER, "$SATIO_CONVERT_COORDINATES_ON") == 0) {
      satio_convert_coordinates_on();
    }
    else if (strcmp(serial0Data.BUFFER, "$SATIO_CONVERT_COORDINATES_OFF") == 0) {
      satio_convert_coordinates_off();
    }

    // --------------------------------------------------------------------------------------------------------------------------
    //                                                                                                            UNKNOWN COMMAND

    else {
      Serial.println("[unknown] " + String(serial0Data.BUFFER));
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 STATS COUNTERS

void CountMatrixEnabled(){
  matrixData.matrix_enabled_i = 0;
  matrixData.matrix_disabled_i = 0;
  for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
    if (matrixData.matrix_switch_enabled[0][Mi] == 1) {matrixData.matrix_enabled_i++;} else {matrixData.matrix_disabled_i++;}}
}

void CountMatrixActive(){
  matrixData.matrix_active_i = 0;
  matrixData.matrix_inactive_i = 0;
  for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
    if (matrixData.matrix_switch_state[0][Mi] == 1) {matrixData.matrix_active_i++;} else {matrixData.matrix_inactive_i++;}}
}

void MatrixStatsCounter() {
  CountMatrixEnabled();
  CountMatrixActive();
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                PORT CONTROLLER

void SatIOPortController(char * data) {

  for (int i=0; i<10; i++) {

    if (Serial1.availableForWrite()) {

      /* uncomment to see what will be sent to the port controller */
      // Serial.print("[TXD] "); Serial.println(matrixData.matrix_sentence);

        /* write matrix switch states to the port controller */
        Serial1.write(data);
        Serial1.write(ETX);
        break;
    }
  }
}

void SatIOPortControllerAnalogMux(const char * mux0_channel, const char * mux1_channel) {
  if (Serial1.availableForWrite()) {

    // create sentence tag and set channel token(s)
    memset(SerialLink.BUFFER1, 0, sizeof(SerialLink.BUFFER1));
    strcpy(SerialLink.BUFFER1, "$MUX,");
    strcat(SerialLink.BUFFER1, mux0_channel);
    strcat(SerialLink.BUFFER1, ",");
    strcat(SerialLink.BUFFER1, mux1_channel);
    strcat(SerialLink.BUFFER1, ",");
    

    // append checksum
    createChecksum(SerialLink.BUFFER1);
    strcat(SerialLink.BUFFER1, "*");
    strcat(SerialLink.BUFFER1, SerialLink.checksum);
    strcat(SerialLink.BUFFER1, "\n");

    /* uncomment to see what will be sent to the port controller */
    // Serial.print("[TXD] "); Serial.println(SerialLink.BUFFER1);

    /* write matrix switch states to the port controller */
    Serial1.write(SerialLink.BUFFER1);
    Serial1.write(ETX);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                        SDCARD: FULL INITIALIZE

void setupSDCard() {
  /*
  initializes sdcard, attempts to load saved system configuration file and saved matrix file. creates new directory tree, system file
  and matrix file if not exists.
  */

  // check pin reassignment
  #ifdef REASSIGN_PINS
  SPI.begin(sck, miso, mosi, cs);
  if (!SD.begin(cs)) {
#else
  if (!SD.begin()) {
#endif
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  // basic checks
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  // create/load system files
  if (!(cardType == CARD_NONE)) {
    Serial.println("[sdcard] initialized");
    sdcard_mkdirs();
    // load system configuration file
    if (!sdcard_load_system_configuration(SD, sdcardData.sysconf, 0)) {sdcard_save_system_configuration(SD, sdcardData.sysconf, 0);}
    // load matrix file specified by configuration file
    if (!sdcard_load_matrix(SD, sdcardData.matrix_filepath)) {
      Serial.println("[sdcard] specified matrix file not found!");
      // create default matrix file
      if (strcmp(sdcardData.matrix_filepath, sdcardData.default_matrix_filepath)==0) {
        Serial.println("[sdcard] default matrix file not found!");
        if (!sdcard_save_matrix(SD, sdcardData.matrix_filepath)) {Serial.println("[sdcard] failed to write default marix file.");}
        else if (!sdcard_load_matrix(SD, sdcardData.default_matrix_filepath)) {Serial.println("[sdcard] failed to load matrix file");}
      }
    }
  }
  else {Serial.println("[sdcard] failed to initialize.");}
  }

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                       SDCARD: LIGHT INITIALIZE

void setupSDCardLight() {
  /*
  attempt to initialize sdcard.
  */
  Serial.println("[sdcard] initializing");
  if (SD.begin(SS, sdspi, 80000000)) {
    // sdcard_mkdirs();
    Serial.println("[sdcard] initialized");
  }
  else {Serial.println("[sdcard] failed to initialize.");
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SDCARD: CHECK

void sdcardCheck() {

  /* basic sdcard initialization. todo: make some bool of initialization results */

  // if (satData.current_unixtime > sdcardData.last_sdcard_check_time+sdcardData.sdcard_check_interval) {
  //   sdcardData.last_sdcard_check_time = satData.current_unixtime;
  //   Serial.println("[checking] sdcard");
    
    // note that information will be displayed if sdcard not present.
    if (SD.exists("/")==true) {
      // sdcardData.card_type = SD.cardType();
      // sdcardData.card_size = SD.cardSize() / (1024 * 1024);
      
      // uncomment to debug
      // Serial.print("[sdcard] card type: " + String(sdcardData.sdcard_types[0][sdcardData.card_type]));
      // Serial.printf("SD Card Size: %lluMB\n", sdcardData.card_size);
    }
    else {
      Serial.println("[sdcard] resetting values");
      // sdcardData.card_type=CARD_NONE; sdcardData.card_size=0;
      SD.end();
      setupSDCardLight();
    }
  // }
  // else {Serial.println("[pending] sdcard check");}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  READ GPS DATA

void check_gngga() {
  // Serial.println("[check_gngga]");
  if (systemData.gngga_enabled == true){
    if (systemData.output_gngga_enabled==true) {Serial.println(gnggaData.sentence);}
    gnggaData.valid_checksum = validateChecksum(gnggaData.sentence);
    // output.println("[gnggaData.sentence] " + String(gnggaData.sentence));
    // output.println("[gnggaData.valid_checksum] " + String(gnggaData.valid_checksum));
    if (gnggaData.valid_checksum == true) {GNGGA();}
    else {gnggaData.bad_checksum_validity++;}
    // GNGGA();
  }
}

void check_gnrmc() {
  // Serial.println("[check_gnrmc]");
  if (systemData.gnrmc_enabled == true) {
    if (systemData.output_gnrmc_enabled == true) {Serial.println(gnrmcData.sentence);}
    gnrmcData.valid_checksum = validateChecksum(gnrmcData.sentence);
    // output.println("[gnrmcData.sentence] " + String(gnrmcData.sentence));
    // output.println("[gnrmcData.valid_checksum] " + String(gnrmcData.valid_checksum));
    if (gnrmcData.valid_checksum == true) {GNRMC();}
    else {gnrmcData.bad_checksum_validity++;}
    // GNRMC();
  }
}

void check_gpatt() {
  // Serial.println("[check_gpatt]");
  if (systemData.gpatt_enabled == true) {
    if (systemData.output_gpatt_enabled == true) {Serial.println(gpattData.sentence);}
    gpattData.valid_checksum = validateChecksum(gpattData.sentence);
    // output.println("[gpattData.sentence] " + String(gpattData.sentence));
    // output.println("[gpattData.valid_checksum] " + String(gpattData.valid_checksum));
    if (gpattData.valid_checksum == true) {GPATT();}
    else {gpattData.bad_checksum_validity++;}
    // GPATT();
  }
}

bool isGPSEnabled() {
  if ((systemData.gngga_enabled==true) || (systemData.gnrmc_enabled==true) || (systemData.gpatt_enabled==true)) {return true;}
  return false;
}


int gps_read_t;
bool gps_done = false;
int gps_done_t = millis();
int i_gps = 0;
bool cs = false;

void gpstest() {
  if (Serial2.available()) {

    gps_read_t = millis();
    memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));

    SerialLink.nbytes = Serial2.readBytesUntil('\n', SerialLink.BUFFER, sizeof(SerialLink.BUFFER));
    Serial.println("[readGPS RXD] [t=" + String(millis()-gps_read_t) + "] [b=" + String(SerialLink.nbytes) + "] " + String(SerialLink.BUFFER)); // debug

    cs = false;
    cs = validateChecksum(SerialLink.BUFFER);
    Serial.println("[check] " + String(cs));

  }
}

void readGPS(void * pvParameters) {
  // Serial.println("[readGPS] ");

  while (1) {

    // gpstest();

    if (gps_done==false) {

      gps_done_t = millis();
      serial1Data.gngga_bool = false;
      serial1Data.gnrmc_bool = false;
      serial1Data.gpatt_bool = false;
      memset(gnggaData.sentence, 0, sizeof(gnggaData.sentence));
      memset(gnrmcData.sentence, 0, sizeof(gnrmcData.sentence));
      memset(gpattData.sentence, 0, sizeof(gpattData.sentence));

      /* 
      this setup should read every sentence (gngga, desbi, gpatt, gnrmc) coming from the WTGPS300 once every 100ms.
       */

      for (i_gps = 0; i_gps < 10; i_gps++) {
        if (Serial2.available()) {

          if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}

          // gps_read_t = millis();

          memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
          
          SerialLink.nbytes = Serial2.readBytesUntil('\r\n', SerialLink.BUFFER, sizeof(SerialLink.BUFFER));

          // Serial.println("[readGPS RXD] [t=" + String(millis()-gps_read_t) + "] [b=" + String(SerialLink.nbytes) + "] " + String(SerialLink.BUFFER)); // debug

          if (SerialLink.nbytes>50) {

            if (serial1Data.gngga_bool==false) {
              if (strncmp(SerialLink.BUFFER, "$GNGGA", 6) == 0) {
                if (systemData.gngga_enabled == true){
                  // Serial.println("[readGPS RXD] [t=" + String(millis()-gps_read_t) + "] [b=" + String(SerialLink.nbytes) + "] " + String(SerialLink.BUFFER)); // debug
                  strcpy(gnggaData.sentence, SerialLink.BUFFER);
                  // Serial.println("[serial1Data.gngga_bool] " + String(serial1Data.gngga_bool));
                  // Serial.println("[serial1Data.gnrmc_bool] " + String(serial1Data.gnrmc_bool));
                  // Serial.println("[serial1Data.gpatt_bool] " + String(serial1Data.gpatt_bool));
                  serial1Data.gngga_bool = true;
                  if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
                }
              }
            }

            if (serial1Data.gnrmc_bool==false) {
              if (strncmp(SerialLink.BUFFER, "$GNRMC", 6) == 0) {
                if (systemData.gnrmc_enabled == true){
                  // Serial.println("[readGPS RXD] [t=" + String(millis()-gps_read_t) + "] [b=" + String(SerialLink.nbytes) + "] " + String(SerialLink.BUFFER)); // debug
                  strcpy(gnrmcData.sentence, SerialLink.BUFFER);
                  // Serial.println("[serial1Data.gngga_bool] " + String(serial1Data.gngga_bool));
                  // Serial.println("[serial1Data.gnrmc_bool] " + String(serial1Data.gnrmc_bool));
                  // Serial.println("[serial1Data.gpatt_bool] " + String(serial1Data.gpatt_bool));
                  serial1Data.gnrmc_bool = true;
                  if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
                }
              }
            }

            if (serial1Data.gpatt_bool==false) {
              if (strncmp(SerialLink.BUFFER, "$GPATT", 6) == 0) {
                if (systemData.gpatt_enabled == true){
                  // Serial.println("[readGPS RXD] [t=" + String(millis()-gps_read_t) + "] [b=" + String(SerialLink.nbytes) + "] " + String(SerialLink.BUFFER)); // debug
                  strcpy(gpattData.sentence, SerialLink.BUFFER);
                  // Serial.println("[serial1Data.gngga_bool] " + String(serial1Data.gngga_bool));
                  // Serial.println("[serial1Data.gnrmc_bool] " + String(serial1Data.gnrmc_bool));
                  // Serial.println("[serial1Data.gpatt_bool] " + String(serial1Data.gpatt_bool));
                  serial1Data.gpatt_bool = true;
                  if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
                }
              }
            }
          }
        }
      }
    
      if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {

        if (systemData.gngga_enabled == true){
          if (systemData.output_gngga_enabled==true) {Serial.println(gnggaData.sentence);}
          gnggaData.valid_checksum = validateChecksum(gnggaData.sentence);
          // Serial.println("[gnggaData.valid_checksum] " + String(gnggaData.valid_checksum));
          if (gnggaData.valid_checksum == true) {GNGGA();}
          else {gnggaData.bad_checksum_validity++;}
        }
        
        if (systemData.gnrmc_enabled == true) {
          if (systemData.output_gnrmc_enabled == true) {Serial.println(gnrmcData.sentence);}
          gnrmcData.valid_checksum = validateChecksum(gnrmcData.sentence);
          // Serial.println("[gnrmcData.valid_checksum] " + String(gnrmcData.valid_checksum));
          if (gnrmcData.valid_checksum == true) {GNRMC();}
          else {gnrmcData.bad_checksum_validity++;}
        }

        if (systemData.gpatt_enabled == true) {
          if (systemData.output_gpatt_enabled == true) {Serial.println(gpattData.sentence);}
          gpattData.valid_checksum = validateChecksum(gpattData.sentence);
          // Serial.println("[gpattData.valid_checksum] " + String(gpattData.valid_checksum));
          if (gpattData.valid_checksum == true) {GPATT();}
          else {gpattData.bad_checksum_validity++;}
        }

        if ((gnggaData.valid_checksum=true) && (gnrmcData.valid_checksum=true) && (gpattData.valid_checksum=true)) {
          // Serial.println("[gps_done_t] " + String(millis()-gps_done_t)); // debug
          gps_done_t = millis();
          gps_done=true;
        }
      }
    }
    delay(1);
  }
}

int tpc;

// void readPortController(void * pvParameters) {
void  readPortController() {
  // Serial.println("[readPortController] ");
  // while(1) {
    for (int i = 0; i < 10; i++) {
        if (Serial1.available()) {
        tpc = millis();
        memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
        SerialLink.nbytes = Serial1.readBytesUntil(ETX, SerialLink.BUFFER, 150);
        // Serial.println("[readPC RXD 0] [t=" + String(millis()-tpc) + "] [b=" + String(SerialLink.nbytes) + "] " + String(SerialLink.BUFFER)); // debug
        if (SerialLink.nbytes>10) {
          if (strncmp(SerialLink.BUFFER, "$D0", 3) == 0) {
            // Serial.println("[readPC RXD 1] " + String(SerialLink.BUFFER)); // debug
            if (validateChecksum(SerialLink.BUFFER)==true) {
              // Serial.println("[validated] " + String(SerialLink.BUFFER)); // debug
              SerialLink.TOKEN_i = 0;
              SerialLink.token = strtok(SerialLink.BUFFER, ",");
              while (SerialLink.token != NULL) {
                if (SerialLink.TOKEN_i==1)  {
                }
                SerialLink.token = strtok(NULL, ",");
                SerialLink.TOKEN_i++;
              }
              break;
            }
          }
        }
      }
    // }
    // delay(1);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   MULTIPLEXERS

/* i2c multiplexer */

#define TCAADDR 0x70 // i2c address of TCAADDR i2c multiplexer 

void setMultiplexChannel_TCAADDR(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);  // change channel of i2c multiplexer
  Wire.endTransmission();
}

/* analog/digital multiplexer */

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

int s0 = 32; // control pin
int s1 = 33; // control pin
int s2 = 12; // control pin
int s3 = 13; // control pin
int sig = 4; // signal pin
int controlPin[] = {s0, s1, s2, s3};

void setMultiplexChannel_CD74HC4067(int ch0) {
  for(int i = 0; i < 4; i++){
    digitalWrite(controlPin[i], muxChannel[ch0][i]); // change channel of analog/digital multiplexer
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      SENSORS

bool sensors_done = false;

void getSensorData(void * pvParameters) {

  while (1) {

    // Serial.println("[getSensorData] ");

    if (sensors_done==false) {

      // photo resistor
      setMultiplexChannel_CD74HC4067(0);
      delay(10);
      sensorData.photoresistor_0 = analogRead(PHOTORESISTOR_0);
      // Serial.println("[photoresistor_0] " + String(sensorData.photoresistor_0));

      // dht11
      setMultiplexChannel_CD74HC4067(1);
      sensorData.dht11_h_0 = dht.readHumidity();
      sensorData.dht11_c_0 = dht.readTemperature();     // celsius default
      sensorData.dht11_f_0 = dht.readTemperature(true); // fahreheit = true
      if (isnan(sensorData.dht11_h_0) || isnan(sensorData.dht11_c_0) || isnan(sensorData.dht11_f_0)) {
        Serial.println(F("Failed to read from DHT sensor!"));
      }
      sensorData.dht11_hif_0 = dht.computeHeatIndex(sensorData.dht11_f_0, sensorData.dht11_h_0);        // fahreheit default
      sensorData.dht11_hic_0 = dht.computeHeatIndex(sensorData.dht11_c_0, sensorData.dht11_h_0, false); // fahreheit = false
      // Serial.println("[dht11_hic_0] " + String(sensorData.dht11_hic_0));

      sensors_done=true;
    }
    delay(1);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          SETUP

void setup() {
  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                SETUP: SERIAL

  Serial.println("[setup] serial");

  Serial.setRxBufferSize(2000); // ensure this is set before begin()
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200); while(!Serial);

  // ESP32 can map hardware serial to alternative pins.
  Serial1.setPins(26, 25, ctsPin, rtsPin); // serial to port controller module . ensure this is set before begin()
  Serial1.setRxBufferSize(2000); // ensure this is set before begin()
  Serial1.setTimeout(50); // ensure this is set before begin()
  Serial1.begin(115200);

  Serial2.setPins(27, 14, ctsPin, rtsPin); // serial to gps module. ensure this is set before begin()
  Serial2.setRxBufferSize(2000); // ensure this is set before begin()
  Serial2.setTimeout(50); // ensure this is set before begin()
  Serial2.begin(115200);

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                  SETUP: PINS

  Serial.println("[setup] pins");

  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 
  pinMode(sig, INPUT); 
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                       SETUP: PORT CONTROLLER
  // setPortControllerReadMode(0); // put port controller in read mode

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                          SETUP: SECOND TIMER

  Serial.println("[setup] second timer");

  // to do: sync second timer with rtc
  second_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(second_timer, &isr_second_timer, true);
  timerAlarmWrite(second_timer, 1000000, true);
  timerAlarmEnable(second_timer);

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                  SETUP: WIRE 

  
  setMultiplexChannel_CD74HC4067(1);
  
  Serial.println("[setup] wire");
  Wire.begin();  // sets up the I2C  

  Serial.println("[setup] selecting i2C channel: 0");
  setMultiplexChannel_TCAADDR(0);  // set i2c multiplexer channel

  Serial.println("[setup] rtc");
  rtc.begin();   // initializes the I2C device

  dht.begin();

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                           SETUP: SYSTEM INFO

  delay(1000);
  Serial.println("[xPortGetCoreID] " + String(xPortGetCoreID()));
  Serial.println("[ESP_PM_CPU_FREQ_MAX] " + String(ESP_PM_CPU_FREQ_MAX));
  Serial.println("[ESP_PM_APB_FREQ_MAX] " + String(ESP_PM_APB_FREQ_MAX));
  Serial.println("[ESP_PM_NO_LIGHT_SLEEP] " + String(ESP_PM_NO_LIGHT_SLEEP));
  Serial.println("[CONFIG_ESPTOOLPY_FLASHFREQ] " + String(CONFIG_ESPTOOLPY_FLASHFREQ));
  Serial.println("[CONFIG_ESPTOOLPY_FLASHMODE] " + String(CONFIG_ESPTOOLPY_FLASHMODE));
  // Serial.println("[CONFIG_COMPILER_OPTIMIZATION] " + String(CONFIG_COMPILER_OPTIMIZATION));
  Serial.println("[CONFIG_ESP32_REV_MIN] " + String(CONFIG_ESP32_REV_MIN));
  Serial.println("[CONFIG_LOG_DEFAULT_LEVEL] " + String(CONFIG_LOG_DEFAULT_LEVEL));
  Serial.println("[CONFIG_BOOTLOADER_LOG_LEVEL] " + String(CONFIG_BOOTLOADER_LOG_LEVEL));
  Serial.println("[CONFIG_ESP_CONSOLE_UART_BAUDRATE] " + String(CONFIG_ESP_CONSOLE_UART_BAUDRATE));
  // Serial.println("[ CONFIG_LOG_DYNAMIC_LEVEL_CONTROL] " + String(CONFIG_LOG_DYNAMIC_LEVEL_CONTROL));
  // Serial.println("[  CONFIG_LOG_TAG_LEVEL_IMPL] " + String( CONFIG_LOG_TAG_LEVEL_IMPL));
  Serial.println("[CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL] " + String(CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL));
  // IRAM https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html#iram
  Serial.println("[getCpuFrequencyMhz] " + String(getCpuFrequencyMhz()));
  Serial.println("[APB_CLK_FREQ] " + String(getApbFrequency()));

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                SETUP: SDCARD

  // setupSDCard();

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                            SETUP: CORE TASKS

  // Create touchscreen task to increase performance (core 0 also found to be best for this task)
  xTaskCreatePinnedToCore(
      readGPS, /* Function to implement the task */
      "Task0", /* Name of the task */
      10000,   /* Stack size in words */
      NULL,    /* Task input parameter */
      2,       /* Priority of the task */
      &Task0,  /* Task handle. */
      0);      /* Core where the task should run */
    
  // Create touchscreen task to increase performance (core 0 also found to be best for this task)
  xTaskCreatePinnedToCore(
    getSensorData, /* Function to implement the task */
    "Task1",       /* Name of the task */
    10000,         /* Stack size in words */
    NULL,          /* Task input parameter */
    2,             /* Priority of the task */
    &Task1,        /* Task handle. */
    0);            /* Core where the task should run */

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                      SETUP: SIDEREAL PLANETS

  myAstro.begin();
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MAIN LOOP

int t0 = millis();
long i_loops_between_gps_reads = 0;
int t_gps_all = millis();
bool track_planets_period = false;
void loop() {

  Serial.println("----------------------------------------");
  // Serial.println("[loop] ");

  timeData.mainLoopTimeStart = millis();
  i_loops_between_gps_reads++;

  // uncomment to override default values
  systemData.matrix_enabled = true;
  systemData.run_on_startup = true;

  // ---------------------------------------------------------------------
  //                                                                   GPS

  /* ensure safe execution each loop. final matrix values for port controller
  must not be altered while instructing port controller. this must be true
  while also instructing port controller every loop and while not blocking
  the loop so that we can utilize the port controller for other instructions
  and do other things if needed until gps data is ready. the wtgps300 outputs
  each sentence (gngga, gpatt, gnrmc, desbi) 10 times a second, every 100
  milliseconds. */
  if (gps_done==true) {
    // Serial.println("[gps_done_t]          " + String(millis()-gps_done_t));
    // Serial.println("[loops between gps]   " + String(i_loops_between_gps_reads));
    i_loops_between_gps_reads = 0;

    // t0 = millis();
    convertUTCToLocal();
    // Serial.println("[convertUTCToLocal]   " + String(millis()-t0));

    // t0 = millis();
    calculateLocation();
    // Serial.println("[calculateLocation]   " + String(millis()-t0));

    /* uncomment to limit planet tracking to once per second. */
    // if (track_planets_period == true) {
      track_planets_period = false;
      // t0 = millis();
      setTrackPlanets();
      // Serial.println("[setTrackPlanets]     " + String(millis()-t0));

      // t0 = millis();
      trackPlanets();
      // Serial.println("[trackPlanets]        " + String(millis()-t0));
    // }

    // t0 = millis();
    if (systemData.satio_enabled == true) {buildSatIOSentence();}
    // Serial.println("[buildSatIOSentence]  " + String(millis()-t0));
    
    // t0 = millis();
    if (sensors_done==true) {
      if (systemData.matrix_enabled == true) {matrixSwitch();}
    }
    // Serial.println("[matrixSwitch]        " + String(millis()-t0));

    MatrixStatsCounter();

    // instruct port controller: matrix
    // setPortControllerReadMode(0);
    // t0 = millis();
    SatIOPortController(matrixData.matrix_sentence);
    // Serial.println("[writePortController] " + String(millis()-t0));
    gps_done = false;
    sensors_done=false;
  }

  // ---------------------------------------------------------------------


  // instruct port controller the rest of the time: (data other than matrix)
  // setPortControllerReadMode(0);
  // t0 = millis();
  // SatIOPortController(otherdata);
  // Serial.println("[writePortController] " + String(millis()-t0));

  // ---------------------------------------------------------------------

  if (interrupt_second_counter > 0) {
    portENTER_CRITICAL(&second_timer_mux);
    interrupt_second_counter--;
    track_planets_period = true;
    portEXIT_CRITICAL(&second_timer_mux);
  }

  // delay(1000); // debug test overload: increase loop time
  timeData.mainLoopTimeTaken = (millis() - timeData.mainLoopTimeStart);
  if (timeData.mainLoopTimeTaken>=500) {systemData.overload=true;}
  else {systemData.overload=false;}
  // if (timeData.mainLoopTimeTaken > timeData.mainLoopTimeTakenMax) {timeData.mainLoopTimeTakenMax = timeData.mainLoopTimeTaken;}
  // if (timeData.mainLoopTimeTaken < timeData.mainLoopTimeTakenMin) {timeData.mainLoopTimeTakenMin = timeData.mainLoopTimeTaken;}

  // some data while running headless
  Serial.println("[UTC_Datetime]          " + String(gnrmcData.utc_time) + " " + String(String(gnrmcData.utc_date))); // (at this point stale)
  Serial.println("[RTC Datetime]          " + SerialDisplayRTCDateTime()); // fresh from RTC
  Serial.println("[Satellite Count]       " + String(gnggaData.satellite_count_gngga));
  Serial.println("[HDOP Precision Factor] " + String(gnggaData.hdop_precision_factor));
  Serial.println("[photoresistor_0]       " + String(sensorData.photoresistor_0));
  Serial.println("[dht11_hic_0]           " + String(sensorData.dht11_hic_0));
  Serial.println("[Looptime]              " + String(timeData.mainLoopTimeTaken));
  // Serial.println("[Looptime Max] " + String(timeData.mainLoopTimeTakenMax));
  // Serial.println("[Looptime Min] " + String(timeData.mainLoopTimeTakenMin));

  // delay(500);
  
  // ---------------------------------------------------------------------
}
