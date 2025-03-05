

/*

                                        SatIO - Written by Benjamin Jack Cullen.

                                                   "The GPS Master"

                          A general purpose programmable satellite, sensor and inertial platform.

                                     SatIO is the system, a matrix is the program.

            Design: Break out all the things and build I2C peripherals as required to orbit the ESP32/Central-MCU.

                                    
                                    Wiring For Keystudio ESP32 PLUS Development Board

                                          ESP32: 1st ATMEGA2560 with sheild as Port Controller custom peripheral (for large creative potential out):
                                          ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                          ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                          ESP32: 2nd ATMEGA2560 with sheild as Control Panel custom peripheral (for large creative potential in):
                                          ESP32: io25    -> ATMEGA2560: io22
                                          ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                          ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                          ESP32: WTGPS300P (5v) (for getting a downlink):
                                          ESP32: io27 RXD -> WTGPS300P: TXD
                                          ESP32: null TXD -> WTGPS300P: RXD

                                          ESP32 i2C: i2C Multiplexing (3.3v) (for peripherals):
                                          ESP32: i2C          -> TCA9548A: SDA, SCL
                                          TCA9548A: SDA0 SCL0 -> DS3231: SDA, SCL (5v)

                                          ESP32: Analog/Digital Multiplexing (3.3v) (for peripherals):
                                          ESP32: io4  -> CD74HC4067: SIG
                                          ESP32: io32 -> CD74HC4067: S0
                                          ESP32: io33 -> CD74HC4067: S1
                                          ESP32: io16 -> CD74HC4067: S2
                                          ESP32: io17 -> CD74HC4067: S3
                                          CD74HC4067 C0 -> Photo Resistor: SIG
                                          CD74HC4067 C1 -> DHT11: SIG

                                          ESP32 VSPI: SDCARD (5v) (for matrix and system data):
                                          ESP32: io5  -> HW-125: CS (SS)
                                          ESP32: io23 -> HW-125: DI (MOSI)
                                          ESP32: io19 -> HW-125: DO (MISO)
                                          ESP32: io18 -> HW-125: SCK (SCLK)

                                          ESP32 HSPI: SSD1351 OLED (5v) (for interfacing):
                                          ESP32: io14 -> SSD1351: SCL/SCLK
                                          ESP32: io12 -> SSD1351: MISO/DC
                                          ESP32: io13 -> SSD1351: SDA
                                          ESP32: io26 -> SSD1351: CS

                                          Upcoming: MCP23017 I2C Expansion board (for more custom peripherals)


                                                  SENTENCE $SATIO
                                                                          
                  START Tag                Last Sat Time                    Converted Longitude        
                    |                   |               |                   |               |                  
                  $SATIO,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                        |               |               |                 |                              
                          DatetimeStamp                  Converted Latitude                                 

      Use case: From a clock syncronized with satellites to riding the INS (roll, pitch, yaw) on a fine line to within a certain degree of
                                    expected drift, if GPS data is stale or unavailable.
                        Robots, flying machines and automation, or for use with local LLM's like ollama, anything.

      Bare bones architecture: SatIO is an extended development platform built on and around ESP32, allowing for many different kinds of projects
      using SatIO as a standalone system and or integrating SatIO into other systems as a 'part'.
                                              Extended I2C
                                              Extended Analogue/Digital.
                                              Supports Extended VSPI and HSPI.
                                              Extended IO (using an ATMEGA2560).
      
      Flexibility: The system is designed to be highly flexible, so that input/output/calculations of all kinds can be turned on/off for different use cases,
      including simply returning calculated results from programmable matrix as zero's and one's over the serial for another system to read. Serial
      output is modular so that depending on the use case, transmission over serial can be more efficient and specific, this expands the creative
      potential of using satio, like letting an LLM on another system know what satio knows for one example.

      Port Controller: ESP32 receives sensory data, calculates according to programmable matrix, then instructs the port controller to turn pins high/low
      according to results from the calculations. The pins could be switching led's, motors or microconrtollers for some examples.

      UI: The matrix has been programmable via the UI however the UI has only just been reinstated, after focusing on performance and architecture. The
      feature of programming the matrix through switches and UI will be reimplemented. Until then, the matrix can be hardcoded for testing purposes.
      Focus is payed to emphasis to importance, consistancy and clarity, nothing more, this keeps things simple and purely practical. The UI is not
      polished yet and is still under construction. Only emphasis to importance, consistancy and clarity will be polished, that is all there intends to be.

            Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
                of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)

        ToDo: Latitude and longitude terrain elevation dictionary.

        ToDo: AI peripheral that performs object detection and returns object name.

        ToDo: Create a custom SatIO PCB with all headers broken out and extended, so the system as a 'motherboard' can be mounted and peripherals
        including indicators and such can be configurable/customizably placed.

        ToDo: Add zero-in functionality to matrix functions. find that signature, find that uap. allows for finding things only the sensors can see.

        Todo: wire up the existing functionality through to the interface level.

        ToDo: esp32 has a NIC, host an RSS feed that can be enabled/disabled.

        ToDo: override: setup for special input controls that can override variable output pins on the port controller (satio drives you/you drive satio).
              (redirects joysticks/trigger input to output, variably). requires a special menu page where any given analogue input controls can be calibrated
              and mapped to with certain thresholds for stabalizing input as required.
        
        ToDo: output via portcontroller is currently high/low, allowing for turning devices on/off and or for devices to interpret a high low. provide
              output configuration allowing for more variable analogue signals to be output.
        
        ToDo: allow for user defined parsing of gps data: allowing for different gps modules to be used even if they have a different data structure and or data content.
  
        ToDo: Black capped (occasionally alien green) grey cherry mx switches for the control panel (80 cN operating force takes more pressure than other switches).

        Ethics. Do we really need to be 'turning our lights on at nautical twighlight' automatically, using data from advanced weapons systems (gps data)? This
        project may soon be discontinued and disasembled.

        */

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      LIBRARIES

#include "soc/rtc_wdt.h"
#include <Arduino.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <iostream>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>          // https://github.com/PaulStoffregen/Time
#include <Timezone.h>         // https://github.com/JChristensen/Timezone
#include <SiderealPlanets.h>  // https://github.com/DavidArmstrong/SiderealPlanets
#include <SiderealObjects.h>  // https://github.com/DavidArmstrong/SiderealObjects
#include "esp_pm.h"
#include "esp_attr.h"
#include <DHT.h>
#include <CD74HC4067.h>

#include "lcdgfx.h"
#include "lcdgfx_gui.h"

void beginSDCARD();
void endSDCARD();
void beginSSD1351();
void endSSD1351();
void sdcardCheck();
void UpdateUI();

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                           PINS

const int8_t ctsPin = -1;  // remap hardware serial TXD
const int8_t rtsPin = -1;  // remap hardware serial RXD
const byte txd_to_atmega = 25; // 
const byte rxd_from_gps = 26;  //

#define ISR_I2C_PERIPHERAL_PIN 25 // allows the Control Panel to interrupt us

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   MULTIPLEXERS

/* i2c multiplexer */

#define TCA9548AADDR 0x70 // i2c address of TCA9548A i2c multiplexer 

void setMultiplexChannel_TCA9548A(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548AADDR);
  Wire.write(1 << channel); // change channel of i2c multiplexer
  Wire.endTransmission();
}

/* analog/digital multiplexer */

int CD74HC4067_Mux_Channel[16][4]={
  {0,0,0,0}, //channel 0 
  {1,0,0,0}, //channel 1 
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

const int CD74HC4067_S0 = 32; // control pin
const int CD74HC4067_S1 = 33; // control pin
const int CD74HC4067_S2 = 16; // control pin
const int CD74HC4067_S3 = 17; // control pin
const int CD74HC4067_SIG = 4; // signal pin
const int CD74HC4067_ControlPin[] = {CD74HC4067_S0, CD74HC4067_S1, CD74HC4067_S2, CD74HC4067_S3};

void setMultiplexChannel_CD74HC4067(int channel) {
  for(int i = 0; i < 4; i++){
    digitalWrite(CD74HC4067_ControlPin[i], CD74HC4067_Mux_Channel[channel][i]); // change channel of analog/digital multiplexer
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                            SPI

void beginSPIDevice(int SCLK, int MISO, int MOSI, int SS) {
  /*
  ESP32 default VSPI pins: SCLK=18, MISO=19, MOSI=23, SS=26
  ESP32 default HSPI pins: SCLK=14, MISO=12, MOSI=13, SS=15
  Devices sharing a bus require seperate CS/SS pin and may require seperate MISO pin.
  Note that this is a preliminary begin to be called before a 'library specific begin' like SD.begin() for example when stacking
  multiple SPI devices on the same SPI bus.
  */
  SPI.begin(SCLK, MISO, MOSI, SS); // set pins
  digitalWrite(SS, LOW); // set control pin low to begin transmission
}

void endSPIDevice(int SS) {
  SPI.end();
  digitalWrite(SS, HIGH); // set control pin high to end transmission
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        DISPLAY

// SSD1351 HSPI pins on esp32 with custom CS
int SSD1351_SCLK = 14; // (SCL)
int SSD1351_MISO = 12; // (DC)
int SSD1351_MOSI = 13; // (SDA)
int SSD1351_CS   = 26; // (CS)

// The parameters are  RST pin, BUS number, CS pin, DC pin, FREQ (0 means default), CLK pin, MOSI pin
DisplaySSD1351_128x128x16_SPI display( (int8_t)-1, {  (int8_t)-1,  (int8_t)SSD1351_CS,  (int8_t)SSD1351_MISO,  (int8_t)0,  (int8_t)-1,  (int8_t)-1  });
NanoCanvas<6,8,1> canvas6x8;
NanoCanvas<8,8,1> canvas8x8; 
NanoCanvas<19,8,1> canvas19x8;
NanoCanvas<33,24,1> canvas33x24;
NanoCanvas<120,8,1> canvas120x8;
NanoCanvas<120,24,1> canvas120x24;
NanoCanvas<120,120,1> canvas120x120;
NanoPoint sprite;
NanoEngine16<DisplaySSD1351_128x128x16_SPI> engine( display );

bool update_ui = true;
bool ui_cleared = false;
int menu_page = 0;

const char *menuHomeItems[1] =
{
  "SETUP",
};
LcdGfxMenu menuHome( menuHomeItems, 1, {{2, 2}, {47, 25}} );


const char *menuMainItems[7] =
{
    "   MATRIX        ", // allows matrix configuration
    "   FILE          ", // load/save/delete system and matrix configurations
    "   GPS           ", // enable/disable parsing of sentences from the gps module
    "   SERIAL        ", // enable/disable output of various comma delimited sentences
    "   SYSTEM        ",
    "   UNIVERSE      ", // enable/disable solar tracking, planet tracking and or other celestial calculations
    "   DISPLAY       ",
};
LcdGfxMenu menuMain( menuMainItems, 7, {{3, 34}, {124, 124}} );

const char *menuMatrixSwitchSelectItems[20] =
{
    "M0 ",
    "M1 ",
    "M2 ",
    "M3 ",
    "M4 ",
    "M5 ",
    "M6 ",
    "M7 ",
    "M8 ",
    "M9 ",
    "M10",
    "M11",
    "M12",
    "M13",
    "M14",
    "M15",
    "M16",
    "M17",
    "M18",
    "M19",
}; // 33px x 23px
LcdGfxMenu menuMatrixSwitchSelect( menuMatrixSwitchSelectItems, 20, {{2, 2}, {35, 25}} );


const char *menuMatrixFunctionSelectItems[10] =
{
    "F0 ",
    "F1 ",
    "F2 ",
    "F3 ",
    "F4 ",
    "F5 ",
    "F6 ",
    "F7 ",
    "F8 ",
    "F9 ",
}; // 33px x 23px
LcdGfxMenu menuMatrixFunctionSelect( menuMatrixFunctionSelectItems, 10, {{92, 2}, {125, 25}} );


const char *menuMatrixConfigureFunctionItems[4] =
{
    "SELECT FUNCTION ",
    "ENTER VALUE X   ",
    "ENTER VALUE Y   ",
    "ENTER VALUE Z   ",
};
LcdGfxMenu menuMatrixConfigureFunction( menuMatrixConfigureFunctionItems, 4, {{3, 76}, {124, 124}} );

const char *menuFileItems[6] =
{
    "NEW MATRIX        ",
    "SAVE MATRIX       ",
    "LOAD MATRIX       ",
    "DELETE MATRIX     ",
    "SAVE SYSTEM CONFIG",
    "RESTORE DEFAULTS  ",
};
LcdGfxMenu menuFile( menuFileItems, 6, {{3, 34}, {124, 124}} );

const char *menuMatrixFilepathItems[20];
LcdGfxMenu menuMatrixFilepath( menuMatrixFilepathItems, 20, {{3, 34}, {124, 124}} );

const char *menuGPSItems[5];
LcdGfxMenu menuGPS( menuGPSItems, 5, {{3, 34}, {124, 124}} );

const char *menuSerialItems[5];
LcdGfxMenu menuSerial( menuSerialItems, 5, {{3, 34}, {124, 124}} );

const char *menuUniverseItems[7];
LcdGfxMenu menuUniverse( menuUniverseItems, 7, {{3, 34}, {124, 124}} );

const char *menuDisplayItems[3];
LcdGfxMenu menuDisplay( menuDisplayItems, 3, {{3, 34}, {124, 124}} );

const char *menuSystemItems[3];
LcdGfxMenu menuSystem( menuSystemItems, 1, {{3, 34}, {124, 124}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SENSORS

/*
Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14. Pin 15 can work but DHT must be disconnected during program upload.
Uncomment whatever type you're using!
*/
#define DHTTYPE DHT11 // DHT11
// #define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321
// #define DHTTYPE DHT21 // DHT 21 (AM2301)
DHT dht(CD74HC4067_SIG, DHTTYPE); // plug DHT11 into CD74HC406 analog/digital multiplexer signal pin

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          TASKS

/* ESP32 has 2 cores. initiate task handles */
TaskHandle_t Task0;
TaskHandle_t Task1;
TaskHandle_t Task2;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                            RTC

RTC_DS3231 rtc;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SIDEREAL PLANETS

SiderealPlanets myAstro;    // for calculating azimuth and altitude
SiderealObjects myAstroObj; // for getting right ascension and declination of objects from star table

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                         SDCARD

// SSD1351 VSPI pins on esp32
int SD_SCLK = 18;  // default esp32 VSPI
int SD_MISO = 19;  // default esp32 VSPI
int SD_MOSI = 23;  // default esp32 VSPI
int SD_CS   = 5;   // default esp32 VSPI
SPIClass sdspi = SPIClass(VSPI);
#if !defined(CONFIG_IDF_TARGET_ESP32)
#define VSPI FSPI
#endif

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SPI SWITCHING

void beginSDCARD() {
  beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  sdcardCheck();
 }

 void endSDCARD() {
  SD.end();
  endSPIDevice(SD_CS);
 }

 void beginSSD1351() {
  beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
  display.begin();
 }

 void endSSD1351() {
  display.end();
  endSPIDevice(SSD1351_CS);
 }

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          CHARS

#define ETX 0x03  // end of text character useful for parsing serial data

char A_char[2] = "A";
char D_char[2] = "D";
char N_char[2] = "N";
char E_char[2] = "E";
char S_char[2] = "S";
char V_char[2] = "V";
char W_char[2] = "W";

char digit_0[2] = "0";
char digit_1[2] = "1";
char digit_2[2] = "2";
char digit_3[2] = "3";
char digit_4[2] = "4";
char digit_5[2] = "5";
char digit_6[2] = "6";
char digit_7[2] = "7";
char digit_8[2] = "8";
char digit_9[2] = "9";
char hyphen_char[2] = "-";
char period_char[2] = ".";

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: SYSTEM

struct systemStruct {
  bool overload = false;               // false providing main loop time under specified amount of time. useful if we need to know data is accurate to within overload threshhold time.
  bool matrix_run_on_startup = false;         // enables/disable matrix switch on startup as specified by system configuration file

  bool satio_enabled = true;           // enables/disables data extrapulation from existing GPS data (coordinate degrees, etc)
  bool gngga_enabled = true;           // enables/disables parsing of serial GPS data
  bool gnrmc_enabled = true;           // enables/disables parsing of serial GPS data
  bool gpatt_enabled = true;           // enables/disables parsing of serial GPS data
  bool matrix_enabled = false;         // enables/disables matrix switch

  bool output_satio_enabled = false;   // enables/disables output SatIO sentence over serial
  bool output_gngga_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_gnrmc_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_gpatt_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_matrix_enabled = false;  // enables/disables output matrix switch active/inactive states sentence over serial
  bool port_controller_enabled = true; // may be false by default but is default true for now.

  bool sidereal_track_sun = true;      // enables/disables celestial body tracking
  bool sidereal_track_moon = true;     // enables/disables celestial body tracking
  bool sidereal_track_mercury = true;  // enables/disables celestial body tracking
  bool sidereal_track_venus = true;    // enables/disables celestial body tracking
  bool sidereal_track_mars = true;     // enables/disables celestial body tracking
  bool sidereal_track_jupiter = true;  // enables/disables celestial body tracking
  bool sidereal_track_saturn = true;   // enables/disables celestial body tracking
  bool sidereal_track_uranus = true;   // enables/disables celestial body tracking
  bool sidereal_track_neptune = true;  // enables/disables celestial body tracking

  // oled protection
  bool display_auto_off = true; // recommended
  int index_display_autooff_times = 5; // index of currently used time 
  int max_display_autooff_times = 6; // max available times
  int display_autooff_times[6] = {3, 5, 10, 15, 30, 60}; // available times
  char char_display_autooff_times[6][56] = {
    "AUTO OFF TIME 3",
    "AUTO OFF TIME 5",
    "AUTO OFF TIME 10",
    "AUTO OFF TIME 15",
    "AUTO OFF TIME 30",
    "AUTO OFF TIME 60",
  };
  int display_timeout = display_autooff_times[index_display_autooff_times];

  // personalization: color
  int index_display_color = 6;
  int max_color_index = 7;
  int display_color[7] = {
    RGB_COLOR16(255,0,0), // red
    RGB_COLOR16(255,255,0), // yellow
    RGB_COLOR16(0,255,0), // green
    RGB_COLOR16(0,0,255), // blue
    RGB_COLOR16(0,255,255), // light blue
    RGB_COLOR16(255,0,255), // purple
    RGB_COLOR16(255,255,255), // white
  };
  char char_display_color[7][56] = {
    "COLOR RED",
    "COLOR YELLOW",
    "COLOR GREEN",
    "COLOR BLUE",
    "COLOR LIGHT BLUE",
    "COLOR PURPLE",
    "COLOR WHITE",
  };
  int color_border = display_color[index_display_color];
  int color_content = display_color[index_display_color];

  // conversion maps
  char translate_enable_bool[2][10] = {"DISABLED", "ENABLED"}; // bool used as index selects bool translation
  char translate_plus_minus[2][2]  = {"+", "-"}; // bool used as index selects bool translation

  char tmp0[56];
  char tmp1[56];
};
systemStruct systemData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: DEBUG

struct sysDebugStruct {
  bool gngga_sentence = false;   // enables/disables itemized sentence value output after processing
  bool gnrmc_sentence = false;   // enables/disables itemized sentence value output after processing
  bool gpatt_sentence = false;   // enables/disables itemized sentence value output after processing
  bool serial_0_sentence = true; // enables/disables itemized command values output after processing
  
  bool validation = false;  // enables/disables data validation such as checksum, length and type checking
  bool verbose_file = true; // provide more information about files being loaded/saved/etc.
};
sysDebugStruct sysDebugData;

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
};
Serial1Struct serial1Data;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             SERIAL LINK STRUCT

struct SerialLinkStruct {
  char BUFFER[2000];
  char BUFFER1[2000];
  unsigned long nbytes;
  unsigned long TOKEN_i;
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
  int max_matrix_filenames = 20;                               // max matrix file names available 
  char matrix_filenames[20][56] = {  
    "", "", "", "", "",
    "", "", "", "", "",
    "", "", "", "", "",
    "", "", "", "", "",
    };                                                         // matrix filenames created, stored and found by system
  char sysconf[56] = "/SYSTEM/SYSTEM.CONFIG";                  // filepath
  char default_matrix_filepath[56] = "/MATRIX/M_0.SAVE";  // filepath
  char matrix_filepath[56] = "";                               // current matrix filepath
  char tempmatrixfilepath[56];                                 // used for laoding filepaths
  char system_dirs[2][56] = {"/MATRIX/", "/SYSTEM/"};            // root dirs
  char save_ext[56] = ".SAVE";
  char matrix_fname[10] = "M";
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
  char data_8[56];                                             // value placeholder
  char file_data[1024];                                        // buffer
  char delim[56] = ",";                                         // delimiter char
  char tmp[56];                                                 // buffer
  char tag_0[56] = "r";                                         // file line tag
  char tag_1[56] = "e";                                         // file line tag
  File current_file;                                           // file currently handled
  char newfilename[56];
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
sanitizing each element of a sentence. thorough testing is required to ensure no false negatives/positives.
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

  char temp[256];                 // a general place to store temporary chars relative to MatrixStruct
  char matrix_sentence[256];      // an NMEA inspired sentence reflecting matrix switch states  

  // reflects matrix switch active/inactive states each loop of matrix switch function
  bool matrix_switch_state[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };

  // reflects matrix switch active/inactive states each loop of matrix switch function
  bool tmp_matrix_switch_state[1][20] = {
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

  // reflects matrix switch output mode: 0=high/low
  int matrix_switch_output_mode[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };

  // reflects matrix switch inverted logic bool
  bool matrix_switch_inverted_logic[20][10] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 1
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 2
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 3
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 4
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 5
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 6
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 7
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 8
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 9
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 11
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 12
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 13
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 14
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 15
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 16
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 17
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 18
    },
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 19
    },
  };

  // a placeholder for timings when timer functions are selected for a matrix switch (currently intended as one timer per switch so be careful)
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

  // a placeholder for matrix switch ports (default no port)
  signed int tmp_matrix_port_map[1][20] = {
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

  // a matrix max_matrices by max_matrix_functions storing function names for each matrix switch (default None)
  char matrix_function[20][10][25] = {
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 1
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 2
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 3
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 4
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 5
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 6
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 7
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 8
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 9
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 10
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 11
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 12
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 13
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 14
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 15
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 16
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 17
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 18
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 19
     },
    {"None", "None", "None", "None", "None", "None", "None", "None", "None", "None", // 20
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
      {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, // 1
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

  /* function names for function name matrix */

  // number of available function names that can be used to program a matrix switch
  int max_matrix_function_names = 213;
  // number of available function names that can be used to program a matrix switch (keep strlen() <=23)
  char matrix_function_names[213][25] = 
  {
    "None",
    "Enabled",
    "Overload",
    "SwitchLink",
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
    "DegLatGNGGAOver",
    "DegLatGNGGAUnder",
    "DegLatGNGGAEqual",
    "DegLatGNGGARange",
    "DegLonGNGGAOver",
    "DegLonGNGGAUnder",
    "DegLonGNGGAEqual",
    "DegLonGNGGARange",
    "DegGNGGARanges",
    "DegLatGNRMCOver",
    "DegLatGNRMCUnder",
    "DegLatGNRMCEqual",
    "DegLatGNRMCRange",
    "DegLonGNRMCOver",
    "DegLonGNRMCUnder",
    "DegLonGNRMCEqual",
    "DegLonGNRMCRange",
    "DegGNRMCRanges",
    "UTCTimeGNGGAOver",
    "UTCTimeGNGGAUnder",
    "UTCTimeGNGGAEqual",
    "UTCTimeGNGGARange",
    "LatGNGGAOver",
    "LatGNGGAUnder",
    "LatGNGGAEqual",
    "LatGNGGARange",
    "LonGNGGAOver",
    "LonGNGGAUnder",
    "LonGNGGAEqual",
    "LonGNGGARange",
    "PosStatusGNGGA",
    "SatCountOver",
    "SatCountUnder",
    "SatCountEqual",
    "SatCountRange",
    "HemiGNGGANorth",
    "HemiGNGGASouth",
    "HemiGNGGAEast",
    "HemiGNGGAWest",
    "GPSPrecisionOver",
    "GPSPrecisionUnder",
    "GPSPrecisionEqual",
    "GPSPrecisionRange",
    "AltGNGGAOver",
    "AltGNGGAUnder",
    "AltGNGGAEqual",
    "AltGNGGARange",
    "UTCTimeGNRMCOver",
    "UTCTimeGNRMCUnder",
    "UTCTimeGNRMCEqual",
    "UTCTimeGNRMCRange",
    "PosStatusGNRMCA",
    "PosStatusGNRMCV",
    "ModeGNRMCA",
    "ModeGNRMCD",
    "ModeGNRMCE",
    "ModeGNRMCN",
    "LatGNRMCOver",
    "LatGNRMCUnder",
    "LatGNRMCEqual",
    "LatGNRMCRange",
    "LonGNRMCOver",
    "LonGNRMCUnder",
    "LonGNRMCEqual",
    "LonGNRMCRange",
    "HemiGNRMCNorth",
    "HemiGNRMCSouth",
    "HemiGNRMCEast",
    "HemiGNRMCWest",
    "GSpeedGNRMCOver",
    "GSpeedGNRMCUnder",
    "GSpeedGNRMCEqual",
    "GSpeedGNRMCRange",
    "HeadingGNRMCOver",
    "HeadingGNRMCUnder",
    "HeadingGNRMCEqual",
    "HeadingGNRMCRange",
    "UTCDateGNRMCOver",
    "UTCDateGNRMCUnder",
    "UTCDateGNRMCEqual",
    "UTCDateGNRMCRange",
    "LineFlagGPATTEqual",
    "StaticFlagGPATTEQ",
    "RStateFlagGPATTEQ",
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
    "GNGGAValidCS",
    "GNRMCValidCS",
    "GPATTValidCS",
    "GNGGAValidCD",
    "GNRMCValidCD",
    "GPATTValidCD",
    "SunAzRange",
    "SunAltRange",
    "DayTime",
    "NightTime",
    "Sunrise",
    "Sunset",
    "MoonAzRange",
    "MoonAltRange",
    "MoonUp",
    "MoonDown",
    "Moonrise",
    "Moonset",
    "MoonPhase",
    "MercuryAzRange",
    "MercuryAltRange",
    "MercuryUp",
    "MercuryDown",
    "MercuryRise",
    "MercurySet",
    "VenusAzRange",
    "VenusAltRange",
    "VenusUp",
    "VenusDown",
    "VenusRise",
    "VenusSet",
    "MarsAzRange",
    "MarsAltRange",
    "MarsUp",
    "MarsDown",
    "MarsRise",
    "MarsSet",
    "JupiterAzRange",
    "JupiterAltRange",
    "JupiterUp",
    "JupiterDown",
    "JupiterRise",
    "JupiterSet",
    "SaturnAzRange",
    "SaturnAltRange",
    "SaturnUp",
    "SaturnDown",
    "SaturnRise",
    "SaturnSet",
    "UranusAzRange",
    "UranusAltRange",
    "UranusUp",
    "UranusDown",
    "UranusRise",
    "UranusSet",
    "NeptuneAzRange",
    "NeptuneAltRange",
    "NeptuneUp",
    "NeptuneDown",
    "NeptuneRise",
    "NeptuneSet",
    "DHT11H0Under",
    "DHT11H0Over",
    "DHT11H0Equal",
    "DHT11H0Range",
    "DHT11C0Under",
    "DHT11C0Over",
    "DHT11C0Equal",
    "DHT11C0Range",
    "DHT11F0Under",
    "DHT11F0Over",
    "DHT11F0Equal",
    "DHT11F0Range",
    "DHT11HIC0Under",
    "DHT11HIC0Over",
    "DHT11HIC0Equal",
    "DHT11HIC0Range",
    "DHT11HIF0Under",
    "DHT11HIF0Over",
    "DHT11HIF0Equal",
    "DHT11HIF0Range",
    "PhotoRes0Under",
    "PhotoRes0Over",
    "PhotoRes0Equal",
    "PhotoRes0Range",
  };
};
MatrixStruct matrixData;

// note that we could work out of this item list entirely to be more efficient but then our function name items would have a
// display driver dependency so for now we have two instances and with the menu items depending on our actual item list.
const char *menuMatrixSetFunctionNameItems[213] =
{
  matrixData.matrix_function_names[0],
  matrixData.matrix_function_names[1],
  matrixData.matrix_function_names[2],
  matrixData.matrix_function_names[3],
  matrixData.matrix_function_names[4],
  matrixData.matrix_function_names[5],
  matrixData.matrix_function_names[6],
  matrixData.matrix_function_names[7],
  matrixData.matrix_function_names[8],
  matrixData.matrix_function_names[9],
  matrixData.matrix_function_names[10],
  matrixData.matrix_function_names[11],
  matrixData.matrix_function_names[12],
  matrixData.matrix_function_names[13],
  matrixData.matrix_function_names[14],
  matrixData.matrix_function_names[15],
  matrixData.matrix_function_names[16],
  matrixData.matrix_function_names[17],
  matrixData.matrix_function_names[18],
  matrixData.matrix_function_names[19],
  matrixData.matrix_function_names[20],
  matrixData.matrix_function_names[21],
  matrixData.matrix_function_names[22],
  matrixData.matrix_function_names[23],
  matrixData.matrix_function_names[24],
  matrixData.matrix_function_names[25],
  matrixData.matrix_function_names[26],
  matrixData.matrix_function_names[27],
  matrixData.matrix_function_names[28],
  matrixData.matrix_function_names[29],
  matrixData.matrix_function_names[30],
  matrixData.matrix_function_names[31],
  matrixData.matrix_function_names[32],
  matrixData.matrix_function_names[33],
  matrixData.matrix_function_names[34],
  matrixData.matrix_function_names[35],
  matrixData.matrix_function_names[36],
  matrixData.matrix_function_names[37],
  matrixData.matrix_function_names[38],
  matrixData.matrix_function_names[39],
  matrixData.matrix_function_names[40],
  matrixData.matrix_function_names[41],
  matrixData.matrix_function_names[42],
  matrixData.matrix_function_names[43],
  matrixData.matrix_function_names[44],
  matrixData.matrix_function_names[45],
  matrixData.matrix_function_names[46],
  matrixData.matrix_function_names[47],
  matrixData.matrix_function_names[48],
  matrixData.matrix_function_names[49],
  matrixData.matrix_function_names[50],
  matrixData.matrix_function_names[51],
  matrixData.matrix_function_names[52],
  matrixData.matrix_function_names[53],
  matrixData.matrix_function_names[54],
  matrixData.matrix_function_names[55],
  matrixData.matrix_function_names[56],
  matrixData.matrix_function_names[57],
  matrixData.matrix_function_names[58],
  matrixData.matrix_function_names[59],
  matrixData.matrix_function_names[60],
  matrixData.matrix_function_names[61],
  matrixData.matrix_function_names[62],
  matrixData.matrix_function_names[63],
  matrixData.matrix_function_names[64],
  matrixData.matrix_function_names[65],
  matrixData.matrix_function_names[66],
  matrixData.matrix_function_names[67],
  matrixData.matrix_function_names[68],
  matrixData.matrix_function_names[69],
  matrixData.matrix_function_names[70],
  matrixData.matrix_function_names[71],
  matrixData.matrix_function_names[72],
  matrixData.matrix_function_names[73],
  matrixData.matrix_function_names[74],
  matrixData.matrix_function_names[75],
  matrixData.matrix_function_names[76],
  matrixData.matrix_function_names[77],
  matrixData.matrix_function_names[78],
  matrixData.matrix_function_names[79],
  matrixData.matrix_function_names[80],
  matrixData.matrix_function_names[81],
  matrixData.matrix_function_names[82],
  matrixData.matrix_function_names[83],
  matrixData.matrix_function_names[84],
  matrixData.matrix_function_names[85],
  matrixData.matrix_function_names[86],
  matrixData.matrix_function_names[87],
  matrixData.matrix_function_names[88],
  matrixData.matrix_function_names[89],
  matrixData.matrix_function_names[90],
  matrixData.matrix_function_names[91],
  matrixData.matrix_function_names[92],
  matrixData.matrix_function_names[93],
  matrixData.matrix_function_names[94],
  matrixData.matrix_function_names[95],
  matrixData.matrix_function_names[96],
  matrixData.matrix_function_names[97],
  matrixData.matrix_function_names[98],
  matrixData.matrix_function_names[99],
  matrixData.matrix_function_names[100],
  matrixData.matrix_function_names[101],
  matrixData.matrix_function_names[102],
  matrixData.matrix_function_names[103],
  matrixData.matrix_function_names[104],
  matrixData.matrix_function_names[105],
  matrixData.matrix_function_names[106],
  matrixData.matrix_function_names[107],
  matrixData.matrix_function_names[108],
  matrixData.matrix_function_names[109],
  matrixData.matrix_function_names[110],
  matrixData.matrix_function_names[111],
  matrixData.matrix_function_names[112],
  matrixData.matrix_function_names[113],
  matrixData.matrix_function_names[114],
  matrixData.matrix_function_names[115],
  matrixData.matrix_function_names[116],
  matrixData.matrix_function_names[117],
  matrixData.matrix_function_names[118],
  matrixData.matrix_function_names[119],
  matrixData.matrix_function_names[120],
  matrixData.matrix_function_names[121],
  matrixData.matrix_function_names[122],
  matrixData.matrix_function_names[123],
  matrixData.matrix_function_names[124],
  matrixData.matrix_function_names[125],
  matrixData.matrix_function_names[126],
  matrixData.matrix_function_names[127],
  matrixData.matrix_function_names[128],
  matrixData.matrix_function_names[129],
  matrixData.matrix_function_names[130],
  matrixData.matrix_function_names[131],
  matrixData.matrix_function_names[132],
  matrixData.matrix_function_names[133],
  matrixData.matrix_function_names[134],
  matrixData.matrix_function_names[135],
  matrixData.matrix_function_names[136],
  matrixData.matrix_function_names[137],
  matrixData.matrix_function_names[138],
  matrixData.matrix_function_names[139],
  matrixData.matrix_function_names[140],
  matrixData.matrix_function_names[141],
  matrixData.matrix_function_names[142],
  matrixData.matrix_function_names[143],
  matrixData.matrix_function_names[144],
  matrixData.matrix_function_names[145],
  matrixData.matrix_function_names[146],
  matrixData.matrix_function_names[147],
  matrixData.matrix_function_names[148],
  matrixData.matrix_function_names[149],
  matrixData.matrix_function_names[150],
  matrixData.matrix_function_names[151],
  matrixData.matrix_function_names[152],
  matrixData.matrix_function_names[153],
  matrixData.matrix_function_names[154],
  matrixData.matrix_function_names[155],
  matrixData.matrix_function_names[156],
  matrixData.matrix_function_names[157],
  matrixData.matrix_function_names[158],
  matrixData.matrix_function_names[159],
  matrixData.matrix_function_names[160],
  matrixData.matrix_function_names[161],
  matrixData.matrix_function_names[162],
  matrixData.matrix_function_names[163],
  matrixData.matrix_function_names[164],
  matrixData.matrix_function_names[165],
  matrixData.matrix_function_names[166],
  matrixData.matrix_function_names[167],
  matrixData.matrix_function_names[168],
  matrixData.matrix_function_names[169],
  matrixData.matrix_function_names[170],
  matrixData.matrix_function_names[171],
  matrixData.matrix_function_names[172],
  matrixData.matrix_function_names[173],
  matrixData.matrix_function_names[174],
  matrixData.matrix_function_names[175],
  matrixData.matrix_function_names[176],
  matrixData.matrix_function_names[177],
  matrixData.matrix_function_names[178],
  matrixData.matrix_function_names[179],
  matrixData.matrix_function_names[180],
  matrixData.matrix_function_names[181],
  matrixData.matrix_function_names[182],
  matrixData.matrix_function_names[183],
  matrixData.matrix_function_names[184],
  matrixData.matrix_function_names[185],
  matrixData.matrix_function_names[186],
  matrixData.matrix_function_names[187],
  matrixData.matrix_function_names[188],
  matrixData.matrix_function_names[189],
  matrixData.matrix_function_names[190],
  matrixData.matrix_function_names[191],
  matrixData.matrix_function_names[192],
  matrixData.matrix_function_names[193],
  matrixData.matrix_function_names[194],
  matrixData.matrix_function_names[195],
  matrixData.matrix_function_names[196],
  matrixData.matrix_function_names[197],
  matrixData.matrix_function_names[198],
  matrixData.matrix_function_names[199],
  matrixData.matrix_function_names[200],
  matrixData.matrix_function_names[201],
  matrixData.matrix_function_names[202],
  matrixData.matrix_function_names[203],
  matrixData.matrix_function_names[204],
  matrixData.matrix_function_names[205],
  matrixData.matrix_function_names[206],
  matrixData.matrix_function_names[207],
  matrixData.matrix_function_names[208],
  matrixData.matrix_function_names[209],
  matrixData.matrix_function_names[210],
  matrixData.matrix_function_names[211],
  matrixData.matrix_function_names[212],
};
LcdGfxMenu menuMatrixSetFunctionName( menuMatrixSetFunctionNameItems, 213, {{3, 46}, {124, 124}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: GNGGA

struct GNGGAStruct {
  char sentence[200];
  char tag[56];                                                                                                            // <0> Log header
  char utc_time[56];                     unsigned long bad_utc_time_i;              bool bad_utc_time = true;              // <1> UTC time, the format is hhmmss.sss
  char latitude[56];                    unsigned long bad_latitude_i;              bool bad_latitude = true;              // <2> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];          unsigned long bad_latitude_hemisphere_i;   bool bad_latitude_hemisphere = true;   // <3> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                   unsigned long bad_longitude_i;             bool bad_longitude = true;             // <4> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];         unsigned long bad_longitude_hemisphere_i;  bool bad_longitude_hemisphere = true;  // <5> Longitude hemisphere, E or W (east longitude or west longitude)
  char solution_status[56];              unsigned long bad_solution_status_i;       bool bad_solution_status = true;       // <6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2: pseudorange difference, 6: pure INS */
  char satellite_count_gngga[56] = "0"; unsigned long bad_satellite_count_gngga_i; bool bad_satellite_count_gngga = true; // <7> Number of satellites used
  char hdop_precision_factor[56];       unsigned long bad_hdop_precision_factor_i; bool bad_hdop_precision_factor = true; // <8> HDOP level precision factor
  char altitude[56];                    unsigned long bad_altitude_i;              bool bad_altitude = true;              // <9> Altitude
  char altitude_units[56];               unsigned long bad_altitude_units_i;        bool bad_altitude_units = true;        // <10> 
  char geoidal[56];                     unsigned long bad_geoidal_i;               bool bad_geoidal = true;               // <11> The height of the earth ellipsoid relative to the geoid 
  char geoidal_units[56];                unsigned long bad_geoidal_units_i;         bool bad_geoidal_units = true;         // <12> 
  char differential_delay[56];          unsigned long bad_differential_delay_i;    bool bad_differential_delay = true;    // <13>
  char id[56];                          unsigned long bad_id_i;                    bool bad_id = true;                    // <14> base station ID
  char check_sum[56];                    unsigned long bad_check_sum_i;             bool bad_check_sum = true;             // <15> XOR check value of all bytes starting from $ to *
  int check_data = 0;                   unsigned long bad_checksum_validity;       bool valid_checksum = false;           // Checksum validity bool, counters and a counter for how many elements passed further testing (gngga check_data should result in 16)
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
  char sentence[200];
  char tag[56];                                                                                                                          // <0> Log header
  char utc_time[56];                     unsigned long bad_utc_time_i;                     bool bad_utc_time = true;                     // <1> UTC time, the format is hhmmss.sss
  char positioning_status[56];           unsigned long bad_positioning_status_i;           bool bad_positioning_status = true;           // <2> Positioning status, A=effective positioning, V=invalid positioning
  char latitude[56];                    unsigned long bad_latitude_i;                     bool bad_latitude = true;                     // <3> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];          unsigned long bad_latitude_hemisphere_i;          bool bad_latitude_hemisphere = true;          // <4> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                   unsigned long bad_longitude_i;                    bool bad_longitude = true;                    // <5> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];         unsigned long bad_longitude_hemisphere_i;         bool bad_longitude_hemisphere = true;         // <6> Longitude hemisphere, E or W (east longitude or west longitude)
  char ground_speed[56];                unsigned long bad_ground_speed_i;                 bool bad_ground_speed = true;                 // <7> Ground speed
  char ground_heading[56];              unsigned long bad_ground_heading_i;               bool bad_ground_heading = true;               // <8> Ground heading (take true north as the reference datum)
  char utc_date[56];                     unsigned long bad_utc_date_i;                     bool bad_utc_date = true;                     // <9> UTC date, the format is ddmmyy (day, month, year)
  char installation_angle[56];           unsigned long bad_installation_angle_i;           bool bad_installation_angle = true;           // <10> Magnetic declination (000.0~180.0 degrees)
  char installation_angle_direction[56]; unsigned long bad_installation_angle_direction_i; bool bad_installation_angle_direction = true; // <11> Magnetic declination direction, E (east) or W (west)
  char mode_indication[56];              unsigned long bad_mode_indication_i;              bool bad_mode_indication = true;              // <12> Mode indication (A=autonomous positioning, D=differential E=estimation, N=invalid data) */
  char check_sum[56];                    unsigned long bad_check_sum_i;                    bool bad_check_sum = true;                    // <13> XOR check value of all bytes starting from $ to *
  int check_data = 0;                   unsigned long bad_checksum_validity;              bool valid_checksum = false;                  // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 14)
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
  char sentence[200];
  char tag[56];                                                                                       // <0> Log header
  char pitch[56];             unsigned long bad_pitch_i;            bool bad_pitch = true;            // <1> pitch angle
  char angle_channel_0[56];   unsigned long bad_angle_channel_0_i;  bool bad_angle_channel_0 = true;  // <2> P
  char roll[56];              unsigned long bad_roll_i;             bool bad_roll = true;             // <3> Roll angle
  char angle_channel_1[56];   unsigned long bad_angle_channel_1_i;  bool bad_angle_channel_1 = true;  // <4> R
  char yaw[56];               unsigned long bad_yaw_i;              bool bad_yaw = true;              // <5> Yaw angle
  char angle_channel_2[56];   unsigned long bad_angle_channel_2_i;  bool bad_angle_channel_2 = true;  // <6> Y
  char software_version[56]; unsigned long bad_software_version_i; bool bad_software_version = true; // <7> software verion
  char version_channel[56];   unsigned long bad_version_channel_i;  bool bad_version_channel = true;  // <8> S
  char product_id[56];       unsigned long bad_product_id_i;       bool bad_product_id = true;       // <9> Product ID: 96 bit unique ID
  char id_channel[56];       unsigned long bad_id_channel_i;       bool bad_id_channel = true;       // <10> ID 
  char ins[56];               unsigned long bad_ins_i;              bool bad_ins = true;              // <11> INS Default open inertial navigation system
  char ins_channel[56];       unsigned long bad_ins_channel_i;      bool bad_ins_channel = true;      // <12> whether inertial navigation open
  char hardware_version[56]; unsigned long bad_hardware_version_i; bool bad_hardware_version = true; // <13> Named after master chip
  char run_state_flag[56];    unsigned long bad_run_state_flag_i;   bool bad_run_state_flag = true;   // <14> Algorithm status flag: 1->3
  char mis_angle_num[56];    unsigned long bad_mis_angle_num_i;    bool bad_mis_angle_num = true;    // <15> number of Installation
  char custom_logo_0[56];    unsigned long bad_custom_logo_0_i;    bool bad_custom_logo_0 = true;    // <16>
  char custom_logo_1[56];    unsigned long bad_custom_logo_1_i;    bool bad_custom_logo_1 = true;    // <17>
  char custom_logo_2[56];    unsigned long bad_custom_logo_2_i;    bool bad_custom_logo_2 = true;    // <18>
  char static_flag[56];       unsigned long bad_static_flag_i;      bool bad_static_flag = true;      // <19> 1:Static 0：dynamic
  char user_code[56];         unsigned long bad_user_code_i;        bool bad_user_code = true;        // <20> 1：Normal user X：Customuser
  char gst_data[56];          unsigned long bad_gst_data_i;         bool bad_gst_data = true;         // <21> User satellite accuracy
  char line_flag[56];         unsigned long bad_line_flag_i;        bool bad_line_flag = true;        // <22> 1：straight driving，0：curve driving
  char custom_logo_3[56];    unsigned long bad_custom_logo_3_i;    bool bad_custom_logo_3 = true;    // <23>
  char mis_att_flag[56];      unsigned long bad_mis_att_flag_i;     bool bad_mis_att_flag = true;     // <24> 
  char imu_kind[56];          unsigned long bad_imu_kind_i;         bool bad_imu_kind = true;         // <25> Sensor Type: 0->BIms055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW 4->ICM-40607; 5->ICM-40608 6->ICM-42670; 7->LSM6DSR
  char ubi_car_kind[56];      unsigned long bad_ubi_car_kind_i;     bool bad_ubi_car_kind = true;     // <26> 1: small car, 2: big car
  char mileage[56];          unsigned long bad_mileage_i;          bool bad_mileage = true;          // <27> kilometers: max 9999 kilometers
  char custom_logo_4[56];    unsigned long bad_custom_logo_4_i;    bool bad_custom_logo_4 = true;    // <28>
  char custom_logo_5[56];    unsigned long bad_custom_logo_5_i;    bool bad_custom_logo_5 = true;    // <29>
  char run_inetial_flag[56];  unsigned long bad_run_inetial_flag_i; bool bad_run_inetial_flag = true; // <30> 1->4
  char custom_logo_6[56];    unsigned long bad_custom_logo_6_i;    bool bad_custom_logo_6 = true;    // <31>
  char custom_logo_7[56];    unsigned long bad_custom_logo_7_i;    bool bad_custom_logo_7 = true;    // <32>
  char custom_logo_8[56];    unsigned long bad_custom_logo_8_i;    bool bad_custom_logo_8 = true;    // <33>
  char custom_logo_9[56];    unsigned long bad_custom_logo_9_i;    bool bad_custom_logo_9 = true;    // <34>
  char speed_enable[56];      unsigned long bad_speed_enable_i;     bool bad_speed_enable = true;     // <35> 
  char custom_logo_10[56];   unsigned long bad_custom_logo_10_i;   bool bad_custom_logo_10 = true;   // <36>
  char custom_logo_11[56];   unsigned long bad_custom_logo_11_i;   bool bad_custom_logo_11 = true;   // <37>
  char speed_num[56];         unsigned long bad_speed_num_i;        bool bad_speed_num = true;        // <38> 1：fixed setting，0：Self adaptive installation
  char scalable[56];         unsigned long bad_scalable_i;         bool bad_scalable = true;         // <39> 
  char check_sum[56];         unsigned long bad_check_sum_i;        bool bad_check_sum = true;        // <40> XOR check value of all bytes starting from $ to *
  int check_data = 0;        unsigned long bad_checksum_validity;  bool valid_checksum = false;      // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 41)
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
  int checksum_i;                                                  // checksum int
  char satio_sentence[200];                                        // buffer
  char satDataTag[56]                  = "$SATIO";                  // satio sentence tag
  char last_sat_time_stamp_str[56]    = "0000000000000000";        // record last time satellites were seen yyyymmddhhmmssmm
  bool convert_coordinates            = true;                      // enables/disables coordinate conversion to degrees
  char coordinate_conversion_mode[56]  = "GNGGA";                   // sentence coordinates degrees created from
  double latitude_meter               = 0.0000100;                 // one meter (tune)
  double longitude_meter              = 0.0000100;                 // one meter (tune)
  double latitude_mile                = latitude_meter  * 1609.34; // one mile
  double longitude_mile               = longitude_meter * 1609.34; // one mile
  double abs_latitude_gngga_0         = 0.0;                       // absolute latitude from $ sentence
  double abs_longitude_gngga_0        = 0.0;                       // absolute longditude from $ sentence
  double abs_latitude_gnrmc_0         = 0.0;                       // absolute latitude from $ sentence
  double abs_longitude_gnrmc_0        = 0.0;                       // absolute longditude from $ sentence
  double temp_latitude_gngga;                                      // degrees converted from absolute
  double temp_longitude_gngga;                                     // degrees converted from absolute
  double temp_latitude_gnrmc;                                      // degrees converted from absolute
  double temp_longitude_gnrmc;                                     // degrees converted from absolute
  double location_latitude_gngga;                                  // degrees converted from absolute
  double location_longitude_gngga;                                 // degrees converted from absolute
  double location_latitude_gnrmc;                                  // degrees converted from absolute
  double location_longitude_gnrmc;                                 // degrees converted from absolute
  char location_latitude_gngga_str[56];                            // degrees converted from absolute
  char location_longitude_gngga_str[56];                           // degrees converted from absolute
  char location_latitude_gnrmc_str[56];                            // degrees converted from absolute
  char location_longitude_gnrmc_str[56];                           // degrees converted from absolute
  double minutesLat;                                               // used for converting absolute latitude and longitude
  double minutesLong;                                              // used for converting absolute latitude and longitude
  double degreesLat;                                               // used for converting absolute latitude and longitude
  double degreesLong;                                              // used for converting absolute latitude and longitude
  double secondsLat;                                               // used for converting absolute latitude and longitude
  double secondsLong;                                              // used for converting absolute latitude and longitude
  double millisecondsLat;                                          // used for converting absolute latitude and longitude
  double millisecondsLong;                                         // used for converting absolute latitude and longitude

  signed int utc_offset = 0; // can be used to offset UTC (+/-), to account for daylight saving and or timezones.
  bool utc_offset_flag = 0;  // 0: add hours to time; 1: deduct hours from time

  char pad_digits_new[56]; // a placeholder for digits preappended with zero's.
  char pad_current_digits[56]; // a placeholder for digits to be preappended with zero's.

  /* TEMPORARY TIME VALUES */
  signed int tmp_year_int;        // temp current year
  signed int tmp_month_int;       // temp current month
  signed int tmp_day_int;         // temp current day
  signed int tmp_hour_int;        // temp current hour
  signed int tmp_minute_int;      // temp current minute
  signed int tmp_second_int;      // temp current second
  signed int tmp_millisecond_int; // temp current millisecond
  char tmp_year[56];               // temp current year
  char tmp_month[56];              // temp current month
  char tmp_day[56];                // temp current day
  char tmp_hour[56];               // temp current hour
  char tmp_minute[56];             // temp current minute
  char tmp_second[56];             // temp current second
  char tmp_millisecond[56];        // temp current millisecond

  /* TIME VALUES FOR RTC AND OTHER USE */
  signed int lt_year_int = 0;        // last year satellite count > zero
  signed int lt_month_int = 0;       // last month satellite count > zero
  signed int lt_day_int = 0;         // last day satellite count > zero
  signed int lt_hour_int = 0;        // last hour satellite count > zero
  signed int lt_minute_int = 0;      // last minute satellite count > zero
  signed int lt_second_int = 0;      // last second satellite count > zero
  signed int lt_millisecond_int = 0; // last millisecond satellite count > zero

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
//                                                                                                                       RTC TIME

String formatRTCTime() {
  return String(String(padDigitsZero( rtc.now().hour())) + ":" + String(padDigitsZero(rtc.now().minute())) + ":" + String(padDigitsZero(rtc.now().second())) +
  " " + String(padDigitsZero(rtc.now().day())) + "." + String(padDigitsZero(rtc.now().month())) + "." + String(padDigitsZero(rtc.now().year())));
}

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
//                                                                                                        SET LAST SATELLITE TIME

void syncRTCOnDownlink() {
  rtc.adjust(DateTime(satData.lt_year_int, satData.lt_month_int, satData.lt_day_int, satData.lt_hour_int, satData.lt_minute_int, satData.lt_second_int));
  Serial.println("[synchronized] " + formatRTCTime());
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
  tmElements_t tm_return = {(uint8_t)second(), (uint8_t)minute(), (uint8_t)hour(), (uint8_t)weekday(), (uint8_t)day(), (uint8_t)month(), (uint8_t)year()};

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

  // Serial.println("[rtc time] " + formatRTCTime()); // debug

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

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "MATRIX_FILEPATH,");
    if (!sdcardData.matrix_filepath) {strcat(sdcardData.file_data, sdcardData.default_matrix_filepath);}
    else {strcat(sdcardData.file_data, sdcardData.matrix_filepath);}
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "AUTO_RESUME,");
    itoa(systemData.matrix_run_on_startup, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "DISPLAY_AUTO_OFF,");
    itoa(systemData.display_auto_off, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_AUTO_OFF,");
    itoa(systemData.index_display_autooff_times, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_COLOR,");
    itoa(systemData.index_display_color, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "MATRIX_ENABLED,");
    itoa(systemData.matrix_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "SATIO_ENABLED,");
    itoa(systemData.satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "GNGGA_ENABLED,");
    itoa(systemData.gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "GNRMC_ENABLED,");
    itoa(systemData.gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "GPATT_ENABLED,");
    itoa(systemData.gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_SATIO_SENTENCE,");
    itoa(systemData.output_satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_GNGGA_SENTENCE,");
    itoa(systemData.output_gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_GNRMC_SENTENCE,");
    itoa(systemData.output_gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_GPATT_SENTENCE,");
    itoa(systemData.output_gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "UTC_OFFSET,");
    itoa(satData.utc_offset, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "UTC_OFFSET_FLAG,");
    itoa(satData.utc_offset_flag, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_SUN,");
    itoa(systemData.sidereal_track_sun, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_MOON,");
    itoa(systemData.sidereal_track_moon, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_MERCURY,");
    itoa(systemData.sidereal_track_mercury, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_VENUS,");
    itoa(systemData.sidereal_track_venus, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_MARS,");
    itoa(systemData.sidereal_track_mars, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");
    
    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_JUPITER,");
    itoa(systemData.sidereal_track_jupiter, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_SATURN,");
    itoa(systemData.sidereal_track_saturn, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_URANUS,");
    itoa(systemData.sidereal_track_uranus, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
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
      memset(sdcardData.BUFFER, 0, sizeof(sdcardData.BUFFER));
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
          if (atoi(sdcardData.token) == 0) {systemData.matrix_run_on_startup = false;} else {systemData.matrix_run_on_startup = true;}
        }
      }

      // display auto off
      if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_OFF", strlen("DISPLAY_AUTO_OFF")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          if (atoi(sdcardData.token) == 0) {systemData.display_auto_off = false;} else {systemData.display_auto_off = true;}
        }
      }

      // display auto off time index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_AUTO_OFF", strlen("INDEX_DISPLAY_AUTO_OFF")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_autooff_times = atoi(sdcardData.token);
          systemData.display_timeout = systemData.display_autooff_times[systemData.index_display_autooff_times];
        }
      }

      // display color index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_COLOR", strlen("INDEX_DISPLAY_COLOR")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_color = atoi(sdcardData.token);
          systemData.color_border = systemData.display_color[systemData.index_display_color];
          systemData.color_content = systemData.display_color[systemData.index_display_color];
        }
      }

      // continue to enable/disable only if auto resume is true
      if (systemData.matrix_run_on_startup == true) {
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
    else {strcpy(sdcardData.matrix_filenames[i], "EMPTY");}
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    ZERO MATRIX

/* writes None to every matrix function name for every matrix switch and writes 0 to every matrix function xyz values */

void zero_matrix() {
  Serial.println("[matrix] setting all matrix values to zero.");
  // iterate over each matrix matrix
  for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
    matrixData.matrix_switch_enabled[0][Mi] = 0;
    for (int Fi = 0; Fi < matrixData.max_matrix_functions; Fi++) {
      memset(matrixData.matrix_function[Mi][Fi], 0, 56);
      strcpy(matrixData.matrix_function[Mi][Fi], "None");
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
      memset(sdcardData.BUFFER, 0, sizeof(sdcardData.BUFFER));
      sdcardData.SBUFFER = sdcardData.current_file.readStringUntil('\n');
      sdcardData.SBUFFER.toCharArray(sdcardData.BUFFER, sdcardData.SBUFFER.length()+1);
      if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));}
      // tag: r
      if (strncmp(sdcardData.BUFFER, sdcardData.tag_0, 1) == 0) {
        // ensure cleared
        memset(sdcardData.data_0, 0, sizeof(sdcardData.data_0)); memset(sdcardData.data_1, 0, sizeof(sdcardData.data_1)); memset(sdcardData.data_2, 0, sizeof(sdcardData.data_2));
        memset(sdcardData.data_3, 0, sizeof(sdcardData.data_3)); memset(sdcardData.data_4, 0, sizeof(sdcardData.data_4)); memset(sdcardData.data_5, 0, sizeof(sdcardData.data_5));
        memset(sdcardData.data_6, 0, sizeof(sdcardData.data_6)); memset(sdcardData.data_7, 0, sizeof(sdcardData.data_7)); memset(sdcardData.data_8, 0, sizeof(sdcardData.data_8));
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
          memset(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], 0, sizeof(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]));
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
          // matrix function data: inverted logic
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_8, sdcardData.token);
          if (is_all_digits(sdcardData.data_8) == true) {matrixData.matrix_switch_inverted_logic[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]=atoi(sdcardData.data_8);}
        }
      }
      // tag: e
      else if (strncmp(sdcardData.BUFFER, sdcardData.tag_1, 1) == 0) {
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
        memset(sdcardData.file_data, 0 , sizeof(sdcardData.file_data));
        // tag: matrix (r)
        strcat(sdcardData.file_data, sdcardData.tag_0); strcat(sdcardData.file_data, sdcardData.delim);
        // matrix index
        memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
        sprintf(sdcardData.tmp, "%d", Mi);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // matrix function index
        memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
        sprintf(sdcardData.tmp, "%d", Fi);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function name
        strcat(sdcardData.file_data, matrixData.matrix_function[Mi][Fi]); strcat(sdcardData.file_data, sdcardData.delim);
        // function value x
        memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
        sprintf(sdcardData.tmp, "%f", matrixData.matrix_function_xyz[Mi][Fi][0]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value y
        memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
        sprintf(sdcardData.tmp, "%f", matrixData.matrix_function_xyz[Mi][Fi][1]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value z
        memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
        sprintf(sdcardData.tmp, "%f", matrixData.matrix_function_xyz[Mi][Fi][2]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);

        // // inverted function logic
        memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
        itoa(matrixData.matrix_switch_inverted_logic[Mi][Fi], sdcardData.tmp, 10);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        
        // // write line
        if (sysDebugData.verbose_file==true) {Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));}
        sdcardData.current_file.println(sdcardData.file_data);
      }
      memset(sdcardData.file_data, 0 , sizeof(sdcardData.file_data));
      // tag: enable (e)
      strcat(sdcardData.file_data, sdcardData.tag_1); strcat(sdcardData.file_data, sdcardData.delim);
      // matrix index
      memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
      sprintf(sdcardData.tmp, "%d", Mi);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);

      // matrix enabled 0/1
      memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
      itoa(matrixData.matrix_switch_enabled[0][Mi], sdcardData.tmp, 10);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);

      // matrix switch port
      memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
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
  // at least for now, do not allow deletion of M_0.SAVE.
  if (fs.exists(file)) {
    Serial.println("[sdcard] attempting to delete file: " + String(file));
    // try remove
    fs.remove(file);
    if (!fs.exists(file)) {
      Serial.println("[sdcard] successfully deleted file: " + String(file));
      Serial.println("attempting to remove filename from filenames.");
      // recreate matrix filenames
      sdcard_list_matrix_files(SD, sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
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
  
  x (n0): total time to be divided between on and off time
  y (n1): total time on within x total time
  example: X=2, Y=1 = on 1 second, off one second, total time 2 seconds.

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
  return false;
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
        perfromance and logic prefers adding functions from position zero else if position zero None then break.
        */
        if ((strcmp(matrixData.matrix_function[Mi][Fi], "None") == 0) && (Fi == 0)) {break;}

        /*
        put true in temporary matrix for functions after position zero that are set to None. allows for 1-10 functions to be set.
        */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "None") == 0) {
          tmp_matrix[Fi] = 1; count_none_function++;}

        /*
        put true in temporary matrix if switch is Enabled (different from enabling disabling) regardless of data. if used,
        function name Enabled will always return true.
        */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Enabled") == 0) {tmp_matrix[Fi] = 1;}

        /* a special pair of switches to combine with logic that requires timing be below any specified overload max */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Overload") == 0) {
          tmp_matrix[Fi] = check_bool_true(systemData.overload);}

        /*
         Special Switch Link Function: Mirrors/inverts switch X state (on/off) for switch using SwitchLink function. benefits:
         gain 9+ (over original 10) functions on a switch, simple inverted logic, logic expansion, etc. 
        */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SwitchLink") == 0) {
          tmp_matrix[Fi] = check_equal_true(matrixData.matrix_switch_state[0][(int)matrixData.matrix_function_xyz[Mi][Fi][0]], 1);}

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                              TIME DATA

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SecondsTimer") == 0) {
          tmp_matrix[Fi] = SecondsTimer(matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1], Mi);
          }
        
        // next up: matrix_switch_inverted_logic. utilize inverted theoretical primitives for all checks so we can choose to return true when true or to return true when false
        
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RTCTimeOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RTCTimeUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }
  
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RTCTimeEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }
  
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RTCTimeRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DaySunday") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Sunday")==0) {tmp_matrix[Fi] = 1;}
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            if (!strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Sunday")==0) {tmp_matrix[Fi] = 1;}
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DayMonday") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Monday")==0) {tmp_matrix[Fi] = 1;}
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            if (!strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Monday")==0) {tmp_matrix[Fi] = 1;}
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DayTuesday") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Tuesday")==0) {tmp_matrix[Fi] = 1;}
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            if (!strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Tuesday")==0) {tmp_matrix[Fi] = 1;}
          }
        }
          
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DayWednesday") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Wednesday")==0) {tmp_matrix[Fi] = 1;}
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            if (!strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Wednesday")==0) {tmp_matrix[Fi] = 1;}
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DayThursday") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Thursday")==0) {tmp_matrix[Fi] = 1;}
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            if (!strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Thursday")==0) {tmp_matrix[Fi] = 1;}
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DayFriday") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Friday")==0) {tmp_matrix[Fi] = 1;}
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            if (!strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Friday")==0) {tmp_matrix[Fi] = 1;}
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DaySaturday") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            if (strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Saturday")==0) {tmp_matrix[Fi] = 1;}
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            if (!strcmp(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()).c_str(), "Saturday")==0) {tmp_matrix[Fi] = 1;}
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DateDayX") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(rtc.now().day(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(rtc.now().day(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DateMonthX") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(rtc.now().month(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(rtc.now().month(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DateYearX") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(rtc.now().year(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(rtc.now().year(), (int)matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  SATIO

        // GNGGA (requires satData.coordinate_conversion_mode gngga)

        // over
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNGGAOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_true(satData.location_latitude_gngga,
              matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNGGAOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // under
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNGGAUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(satData.location_longitude_gngga,
              matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNGGAUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // equal
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNGGAEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNGGAEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // range
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNGGARange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNGGARange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0], matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0], matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        // ranges
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegGNGGARanges") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_ranges_check_true(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][1],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_ranges_check_false(satData.location_latitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            satData.location_longitude_gngga,
            matrixData.matrix_function_xyz[Mi][Fi][1],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }
        
        // GNRMC (requires satData.coordinate_conversion_mode gnrmc)

        // over
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // under
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // equal
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // range
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        // ranges
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegGNRMCRanges") == 0) {\
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_ranges_check_true(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][1],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_ranges_check_false(satData.location_latitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            satData.location_longitude_gnrmc,
            matrixData.matrix_function_xyz[Mi][Fi][1],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  GNGGA

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNGGAOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNGGAUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNGGAEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNGGARange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(atol(gnggaData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNGGAOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNGGAUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNGGAEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNGGARange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(atol(gnggaData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNGGAOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNGGAUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNGGAEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNGGARange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnggaData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PosStatusGNGGA") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnggaData.solution_status),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnggaData.solution_status),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SatCountOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SatCountUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SatCountEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SatCountRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnggaData.satellite_count_gngga),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNGGANorth") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, N_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnggaData.latitude_hemisphere, N_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNGGAEast") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, E_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnggaData.longitude_hemisphere, E_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNGGASouth") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, S_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnggaData.latitude_hemisphere, S_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNGGAWest") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, W_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnggaData.longitude_hemisphere, W_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GPSPrecisionOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GPSPrecisionUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GPSPrecisionEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GPSPrecisionRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnggaData.hdop_precision_factor),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "AltGNGGAOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "AltGNGGAUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "AltGNGGAEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "AltGNGGARange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnggaData.altitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  GNRMC

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCTimeGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnrmcData.utc_time),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LatGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(atol(gnrmcData.latitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }
          
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LonGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(atol(gnrmcData.longitude),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNRMCNorth") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, N_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.latitude_hemisphere, N_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNRMCEast") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, E_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.longitude_hemisphere, E_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNRMCSouth") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, S_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.latitude_hemisphere, S_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HemiGNRMCWest") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, W_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.longitude_hemisphere, W_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSpeedGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSpeedGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSpeedGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSpeedGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnrmcData.ground_speed),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HeadingGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0]); 
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0]); 
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HeadingGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HeadingGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "HeadingGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnrmcData.ground_heading),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCDateGNRMCOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCDateGNRMCUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCDateGNRMCEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UTCDateGNRMCRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gnrmcData.utc_date),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PosStatusGNRMCA") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, A_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.positioning_status, A_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PosStatusGNRMCV") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, V_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.positioning_status, V_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "ModeGNRMCA") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, A_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.mode_indication, A_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "ModeGNRMCD") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, D_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.mode_indication, D_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "ModeGNRMCE") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, E_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.mode_indication, E_char, 1);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "ModeGNRMCN") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, N_char, 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_strncmp_false(gnrmcData.mode_indication, N_char, 1);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  GPATT

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PitchGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PitchGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PitchGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PitchGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.pitch),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RollGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RollGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RollGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RollGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.roll),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "YawGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "YawGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "YawGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "YawGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.yaw),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSTDataGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSTDataGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSTDataGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GSTDataGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.gst_data),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MileageGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MileageGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MileageGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MileageGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.mileage),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SpeedNumGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SpeedNumGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SpeedNumGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SpeedNumGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.speed_num),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LineFlagGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "INSGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.ins),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.ins),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RStateFlagGPATTEQ") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "StaticFlagGPATTEQ") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                     SIDEREAL TIME: SUN

        // sun azimuth:
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SunAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.sun_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        // sun altitude:
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SunAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.sun_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        // daytime: current time in range of sunrise and sunset
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DayTime") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.sun_r, siderealPlanetData.sun_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.sun_r, siderealPlanetData.sun_s);
          }
        }

        // nighttime: current time not in range of sunrise and sunset
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NightTime") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.sun_r,
            siderealPlanetData.sun_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.sun_r,
            siderealPlanetData.sun_s);
          }
        }

        // sunrise time less than current time: true after sunrise until midnight
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sunrise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.sun_r, hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.sun_r, hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        // sunset time less than current time: true after sunset until midnight                                                                  
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sunset") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.sun_s, hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.sun_s, hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                                 SIDEREAL TIME: MOON

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.moon_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.moon_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Moonrise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.moon_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.moon_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Moonset") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.moon_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.moon_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.moon_r,
            siderealPlanetData.moon_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.moon_r,
            siderealPlanetData.moon_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.moon_r,
            siderealPlanetData.moon_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.moon_r,
            siderealPlanetData.moon_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonPhase") == 0) {
          tmp_matrix[Fi] = check_equal_true(siderealPlanetData.moon_p,
          matrixData.matrix_function_xyz[Mi][Fi][0]);
          }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                              SIDEREAL TIME: MERCURY

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercuryAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mercury_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercuryAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mercury_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercuryRise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.mercury_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.mercury_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercurySet") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.mercury_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.mercury_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercuryUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mercury_r,
            siderealPlanetData.mercury_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mercury_r,
            siderealPlanetData.mercury_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercuryDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mercury_r,
            siderealPlanetData.mercury_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mercury_r,
            siderealPlanetData.mercury_s);
          }
        }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                                SIDEREAL TIME: VENUS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.venus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.venus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusRise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.venus_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.venus_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusSet") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.venus_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.venus_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.venus_r, 
            siderealPlanetData.venus_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.venus_r, 
            siderealPlanetData.venus_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.venus_r,
            siderealPlanetData.venus_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.venus_r,
            siderealPlanetData.venus_s);
          }
        }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                                 SIDEREAL TIME: MARS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mars_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsAltRange") == 0) {\
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mars_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsRise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.mars_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.mars_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsSet") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.mars_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.mars_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mars_r,
            siderealPlanetData.mars_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mars_r,
            siderealPlanetData.mars_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mars_r,
            siderealPlanetData.mars_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.mars_r,
            siderealPlanetData.mars_s);
          }
        }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                              SIDEREAL TIME: JUPITER

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.jupiter_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.jupiter_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterRise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.jupiter_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.jupiter_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterSet") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.jupiter_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.jupiter_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.jupiter_r,
            siderealPlanetData.jupiter_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.jupiter_r,
            siderealPlanetData.jupiter_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.jupiter_r,
            siderealPlanetData.jupiter_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.jupiter_r,
            siderealPlanetData.jupiter_s);
          }
        }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                               SIDEREAL TIME: SATURN

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.saturn_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.saturn_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnRise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.saturn_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.saturn_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnSet") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.saturn_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.saturn_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.saturn_r,
            siderealPlanetData.saturn_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.saturn_r,
            siderealPlanetData.saturn_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.saturn_r,
            siderealPlanetData.saturn_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.saturn_r,
            siderealPlanetData.saturn_s);
          }
        }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                               SIDEREAL TIME: URANUS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.uranus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.uranus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusRise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.uranus_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.uranus_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusSet") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.uranus_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.uranus_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.uranus_r,
            siderealPlanetData.uranus_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.uranus_r,
            siderealPlanetData.uranus_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.uranus_r,
            siderealPlanetData.uranus_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.uranus_r,
            siderealPlanetData.uranus_s);
          }
        }

        // // -------------------------------------------------------------------------------------------------------------------
        // //                                                                                              SIDEREAL TIME: NEPTUNE

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.neptune_az,
              matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.neptune_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneRise") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.neptune_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.neptune_r,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneSet") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(siderealPlanetData.neptune_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(siderealPlanetData.neptune_s,
            hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()));
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneUp") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.neptune_r,
            siderealPlanetData.neptune_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.neptune_r,
            siderealPlanetData.neptune_s);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneDown") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.neptune_r,
            siderealPlanetData.neptune_s);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
            siderealPlanetData.neptune_r,
            siderealPlanetData.neptune_s);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                       DHT11_0 HUMIDITY
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11H0Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11H0Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11H0Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11H0Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.dht11_h_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                        DHT11_0 CELSIUS
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11C0Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11C0Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11C0Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11C0Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.dht11_c_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                     DHT11_0 FAHRENHEIT
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11F0Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11F0Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11F0Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11F0Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.dht11_f_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                             DHT11_0 HEAT INDEX CELSIUS
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIC0Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIC0Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIC0Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIC0Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.dht11_hic_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                          DHT11_0 HEAT INDEX FAHRENHEIT
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIF0Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIF0Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIF0Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DHT11HIF0Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.dht11_hif_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                        PHOTO RESISTORS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PhotoRes0Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PhotoRes0Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PhotoRes0Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "PhotoRes0Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.photoresistor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }
        
        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               VALIDITY

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GNGGAValidCS") == 0) {
          tmp_matrix[Fi] = check_bool_true(gnggaData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GNRMCValidCS") == 0) {
          tmp_matrix[Fi] = check_bool_true(gnrmcData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GPATTValidCS") == 0) {
          tmp_matrix[Fi] = check_bool_true(gpattData.valid_checksum);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GNGGAValidCD") == 0) {
          tmp_matrix[Fi] = check_equal_true(gnggaData.check_data, 16);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GNRMCValidCD") == 0) {
          tmp_matrix[Fi] = check_equal_true(gnrmcData.check_data, 14);}
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "GPATTValidCD") == 0) {
          tmp_matrix[Fi] = check_equal_true(gpattData.check_data, 41);}
      }

      // ----------------------------------------------------------------------------------------------------------------------
      //                                                                                                           FINAL SWITCH
      
      /*
      safety layer: disengage if all entries are None.
      this is a second layer on top of initial check for None set at position zero, function 0.
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
//                                                                                                        MATRIX SWITCH FUNCTIONS

void setAllMatrixSwitchesEnabledFalse() {
  for (int i=0; i<matrixData.max_matrices; i++) {matrixData.matrix_switch_enabled[0][i]=false;}
}

void setAllMatrixSwitchesStateFalse() {
  for (int i=0; i<matrixData.max_matrices; i++) {matrixData.matrix_switch_state[0][i]=false;}
}

void setAllMatrixSwitchesEnabledTrue() {
  for (int i=0; i<matrixData.max_matrices; i++) {matrixData.matrix_switch_enabled[0][i]=true;}
}

void setAllMatrixSwitchesStateTrue() {
  for (int i=0; i<matrixData.max_matrices; i++) {matrixData.matrix_switch_state[0][i]=true;}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     INPUT DATA

bool make_i2c_request = false;
float unixtime_control_panel_request;
int previous_menu_page;
char input_data[128];
char tmp_input_data[128];
char allow_input_data = false;
signed int enter_digits_key = -1;
int menu_column_selection=0;
int previous_menu_column_selection;

void inputChar(char * data) {

  // allow signing as first char
  if ((strcmp(data, "-")==0) && (strlen(input_data)==0)) {if (allow_input_data==true) {strcat(input_data, data);}}

  else {

    // port
    if (enter_digits_key==1) {
      if (allow_input_data==true) {
        // create temporary data to concat and test
        memset(tmp_input_data, 0, sizeof(tmp_input_data));
        strcpy(tmp_input_data, input_data);
        strcat(tmp_input_data, data);
        // test range
        if ((atoi(tmp_input_data) <= 99) && (atoi(tmp_input_data) >= -1)) {
          memset(input_data, 0, sizeof(input_data));
          strcpy(input_data, tmp_input_data);
        }
      }
    }

    // <= long
    else if ((enter_digits_key==2) || (enter_digits_key==3) || (enter_digits_key==4)) {
      if (allow_input_data==true) {
        // create temporary data to concat and test
        memset(tmp_input_data, 0, sizeof(tmp_input_data));
        strcpy(tmp_input_data, input_data);
        strcat(tmp_input_data, data);
        // test range
        if ((atoi(tmp_input_data) <= 179769313486232) && (atoi(tmp_input_data) >= -179769313486232)) {
          memset(input_data, 0, sizeof(input_data));
          strcpy(input_data, tmp_input_data);
        }
      }
    }

  }
}

// ------------------------------------------------
//                                          UI DATA

char TMP_UI_DATA_0[56];
char TMP_UI_DATA_1[56];

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        MENU UP

void menuUp() {
  if (menu_page==0) {menuHome.up();}
  else if (menu_page==1) {menuMain.up();}
  else if (menu_page==2) {}
  else if (menu_page==3) {
    if (menu_column_selection==0) {menuMatrixSwitchSelect.up();}
    if (menu_column_selection==1) {}
    if (menu_column_selection==2) {}
    if (menu_column_selection==4) {menuMatrixFunctionSelect.up();}
  }
  else if (menu_page==4) {}
  else if (menu_page==5) {menuMatrixConfigureFunction.up();}
  else if (menu_page==6) {menuMatrixSetFunctionName.up();}
  else if (menu_page==20) {menuFile.up();}
  else if (menu_page==21) {menuMatrixFilepath.up();}
  else if (menu_page==22) {menuMatrixFilepath.up();}
  else if (menu_page==23) {menuMatrixFilepath.up();}
  else if (menu_page==50) {menuGPS.up();}
  else if (menu_page==60) {menuSerial.up();}
  else if (menu_page==70) {menuUniverse.up();}
  else if (menu_page==80) {menuDisplay.up();}
  else if (menu_page==90) {menuSystem.up();}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MENU DOWN

void menuDown() {
  if (menu_page==0) {menuHome.down();}
  else if (menu_page==1) {menuMain.down();}
  else if (menu_page==2) {}
  else if (menu_page==3) {
    if (menu_column_selection==0) {menuMatrixSwitchSelect.down();}
    if (menu_column_selection==1) {}
    if (menu_column_selection==2) {}
    if (menu_column_selection==4) {menuMatrixFunctionSelect.down();}
  }
  else if (menu_page==4) {}
  else if (menu_page==5) {menuMatrixConfigureFunction.down();}
  else if (menu_page==6) {menuMatrixSetFunctionName.down();}
  else if (menu_page==20) {menuFile.down();}
  else if (menu_page==21) {menuMatrixFilepath.down();}
  else if (menu_page==22) {menuMatrixFilepath.down();}
  else if (menu_page==23) {menuMatrixFilepath.down();}
  else if (menu_page==50) {menuGPS.down();}
  else if (menu_page==60) {menuSerial.down();}
  else if (menu_page==70) {menuUniverse.down();}
  else if (menu_page==80) {menuDisplay.down();}
  else if (menu_page==90) {menuSystem.down();}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     MENU RIGHT

void menuRight() {
  if (menu_page==0) {}
  else if (menu_page==1) {}
  else if (menu_page==2) {}
  else if (menu_page==3) {menu_column_selection++; if (menu_column_selection>4) {menu_column_selection=0;}}
  else if (menu_page==4) {}
  else if (menu_page==5) {}
  else if (menu_page==6) {}
  Serial.println("[menu_column_selection] " + String(menu_column_selection));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MENU LEFT

void menuLeft() {
  if (menu_page==0) {}
  else if (menu_page==1) {}
  else if (menu_page==2) {}
  else if (menu_page==3) {menu_column_selection--; if (menu_column_selection<0) {menu_column_selection=4;}}
  else if (menu_page==4) {}
  else if (menu_page==5) {}
  else if (menu_page==6) {}
  Serial.println("[menu_column_selection] " + String(menu_column_selection));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     MENU ENTER

void menuEnter() {

  // home page
  if (menu_page==0) {

    // go to main menu
    if (menuHome.selection()==0) {menu_page=1;}
  }

  // main menu
  else if (menu_page==1) {

    // go to matrix menu
    if (menuMain.selection()==0) {
      menu_page=3;
    }

    // go to file menu
    if (menuMain.selection()==1) {
      menu_page=20;
    }

    // go to gps menu
    if (menuMain.selection()==2) {
      menu_page=50;
    }

    // go to serial menu
    if (menuMain.selection()==3) {
      menu_page=60;
    }

    // go to system menu
    if (menuMain.selection()==4) {
      menu_page=90;
    }

    // go to universe menu
    if (menuMain.selection()==5) {
      menu_page=70;
    }

    // go to display menu
    if (menuMain.selection()==6) {
      menu_page=80;
    }

  }

  // matrix switch configuration
  else if (menu_page==3) {

    // go to set port page
    if (menu_column_selection==1) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 1;
      menu_page=4;
    }

    else if (menu_column_selection==2) {
      // enable matrix switch: also turns switch off
      matrixData.matrix_switch_enabled[0][menuMatrixSwitchSelect.selection()]^=true;
      matrixData.matrix_switch_state[0][menuMatrixSwitchSelect.selection()]^=true;
    }

    else if (menu_column_selection==3) {
      matrixData.matrix_switch_inverted_logic[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]^=true;
    }

    else if (menu_column_selection==4) {
      // go to function name selection
      menu_page=5;
    }
  }

  // set digits
  else if (menu_page==4) {
    allow_input_data=false;
    if (enter_digits_key==1) {matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]=atoi(input_data); menu_page=3;}
    else if (enter_digits_key==2) {matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]=atoi(input_data); menu_page=5;}
    else if (enter_digits_key==3) {matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]=atoi(input_data); menu_page=5;}
    else if (enter_digits_key==4) {matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]=atoi(input_data); menu_page=5;}
    enter_digits_key = -1;
  }

  // matrix switch select function name, x, y, or z
  else if (menu_page==5) {
    if (menuMatrixConfigureFunction.selection()==0) {menu_page=6;}

    // go to set function value x page
    if (menuMatrixConfigureFunction.selection()==1) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 2;
      menu_page=4;
    }
    // go to set function value y page
    else if (menuMatrixConfigureFunction.selection()==2) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 3;
      menu_page=4;
    }
    // go to set function value z page
    else if (menuMatrixConfigureFunction.selection()==3) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 4;
      menu_page=4;
    }
  }

  // matrix switch set function name
  else if (menu_page==6) {
    memset(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()], 0, sizeof(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]));
    strcpy(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()], matrixData.matrix_function_names[menuMatrixSetFunctionName.selection()]);
    menu_page=5;
  }

  // file menu
  else if (menu_page==20) {

    // new matrix
    if (menuFile.selection()==0) {
      // disable and turn off all matrix switches
      setAllMatrixSwitchesEnabledFalse();
      setAllMatrixSwitchesStateFalse();
      // zero the matrix and clear current matrix file path
      zero_matrix();
      memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
    }

    // goto save matrix page
    else if (menuFile.selection()==1) {

      // create list of matrix filespaths and go to save page
      endSSD1351();
      beginSDCARD();
      sdcard_list_matrix_files(SD, sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
      endSDCARD();
      beginSSD1351();
      menu_page=21;
    }

    // goto load matrix page
    else if (menuFile.selection()==2) {

      // create list of matrix filespaths and go to load page
      endSSD1351();
      beginSDCARD();
      sdcard_list_matrix_files(SD, sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
      endSDCARD();
      beginSSD1351();
      menu_page=22;
    }

    // goto delete matrix page
    else if (menuFile.selection()==3) {

      // create list of matrix filespaths and go to delete page
      endSSD1351();
      beginSDCARD();
      sdcard_list_matrix_files(SD, sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
      endSDCARD();
      beginSSD1351();
      menu_page=23;
    }

    // save system settings
    else if (menuFile.selection()==4) {
      menu_page=33;
      UpdateUI();
      endSSD1351();
      beginSDCARD();
      sdcard_save_system_configuration(SD, sdcardData.sysconf, 0);
      endSDCARD();
      beginSSD1351();
      delay(2000);
      menu_page=20;
    }

    // restore default system settings
    else if (menuFile.selection()==5) {
      menu_page=34;
      UpdateUI();
      endSSD1351();
      beginSDCARD();
      // restore defaults and save
      endSDCARD();
      beginSSD1351();
      delay(2000);
      menu_page=20;
    }
  }

  // save matrix menu
  else if (menu_page==21) {
    // generate filename according to selection index
    memset(sdcardData.newfilename, 0, sizeof(sdcardData.newfilename));
    strcpy(sdcardData.newfilename, "/MATRIX/M_");
    memset(sdcardData.tmp, 0, sizeof(sdcardData.tmp));
    itoa(menuMatrixFilepath.selection(), sdcardData.tmp, 10);
    strcat(sdcardData.newfilename, sdcardData.tmp);
    strcat(sdcardData.newfilename, ".SAVE");
    Serial.println("[saving] " + String(sdcardData.newfilename));
    // set notification page
    menu_page=30;
    UpdateUI();
    // switch spi devices
    endSSD1351();
    beginSDCARD();
    sdcard_save_matrix(SD, sdcardData.newfilename);
    // switch spi devices
    endSDCARD();
    beginSSD1351();
    // delay(2000);
    menu_page=20;
  }

  // load matrix menu
  else if (menu_page==22) {
    // handle empty slots
    if (!strcmp(sdcardData.matrix_filenames[menuMatrixFilepath.selection()], "EMPTY")==0) {
      // generate filename according to selection index
      memset(sdcardData.newfilename, 0, sizeof(sdcardData.newfilename));
      strcpy(sdcardData.newfilename, "/MATRIX/M_");
      memset(sdcardData.tmp, 0, sizeof(sdcardData.tmp));
      itoa(menuMatrixFilepath.selection(), sdcardData.tmp, 10);
      strcat(sdcardData.newfilename, sdcardData.tmp);
      strcat(sdcardData.newfilename, ".SAVE");
      Serial.println("[loading] " + String(sdcardData.newfilename));
      // set notification page
      menu_page=31;
      UpdateUI();
      // switch spi devices
      endSSD1351();
      beginSDCARD();
      sdcard_load_matrix(SD, sdcardData.newfilename);
      // switch spi devices
      endSDCARD();
      beginSSD1351();
    }
    else {Serial.println("[loading] aborting! cannot load empty slot.");}
    // delay(2000);
    menu_page=20;
  }

  // delete matrix menu
  else if (menu_page==23) {
    // handle empty slots
    if (!strcmp(sdcardData.matrix_filenames[menuMatrixFilepath.selection()], "EMPTY")==0) {
      // generate filename according to selection index
      memset(sdcardData.newfilename, 0, sizeof(sdcardData.newfilename));
      strcpy(sdcardData.newfilename, "/MATRIX/M_");
      memset(sdcardData.tmp, 0, sizeof(sdcardData.tmp));
      itoa(menuMatrixFilepath.selection(), sdcardData.tmp, 10);
      strcat(sdcardData.newfilename, sdcardData.tmp);
      strcat(sdcardData.newfilename, ".SAVE");
      Serial.println("[deleting] " + String(sdcardData.newfilename));
      // set notification page
      menu_page=32;
      UpdateUI();
      // switch spi devices
      endSSD1351();
      beginSDCARD();
      sdcard_delete_matrix(SD, sdcardData.newfilename);
      // switch spi devices
      endSDCARD();
      beginSSD1351();
    }
    else {Serial.println("[deleting] aborting! cannot delete empty slot.");}
    // delay(2000);
    menu_page=20;
  }

  // gps page
  else if (menu_page==50) {
    if (menuGPS.selection()==0) {systemData.satio_enabled^=true;}
    if (menuGPS.selection()==1) {systemData.gngga_enabled^=true;}
    if (menuGPS.selection()==2) {systemData.gnrmc_enabled^=true;}
    if (menuGPS.selection()==3) {systemData.gpatt_enabled^=true;}
    if (menuGPS.selection()==4) {
      if (strcmp(satData.coordinate_conversion_mode, "GNGGA")==0) {
        memset(satData.coordinate_conversion_mode, 0, sizeof(satData.coordinate_conversion_mode));
        strcpy(satData.coordinate_conversion_mode, "GNRMC");
      }
      else if (strcmp(satData.coordinate_conversion_mode, "GNRMC")==0) {
        memset(satData.coordinate_conversion_mode, 0, sizeof(satData.coordinate_conversion_mode));
        strcpy(satData.coordinate_conversion_mode, "GNGGA");
      }
    }
  }

  // serial page
  else if (menu_page==60) {
    if (menuSerial.selection()==0) {systemData.output_satio_enabled^=true;}
    if (menuSerial.selection()==1) {systemData.output_gngga_enabled^=true;}
    if (menuSerial.selection()==2) {systemData.output_gnrmc_enabled^=true;}
    if (menuSerial.selection()==3) {systemData.output_gpatt_enabled^=true;}
    if (menuSerial.selection()==4) {systemData.output_matrix_enabled^=true;}
  }

  // universe page
  else if (menu_page==70) {
    if (menuUniverse.selection()==0) {systemData.sidereal_track_sun^=true;}
    if (menuUniverse.selection()==1) {systemData.sidereal_track_mercury^=true;}
    if (menuUniverse.selection()==2) {systemData.sidereal_track_venus^=true;}
    if (menuUniverse.selection()==3) {systemData.sidereal_track_mars^=true;}
    if (menuUniverse.selection()==4) {systemData.sidereal_track_jupiter^=true;}
    if (menuUniverse.selection()==5) {systemData.sidereal_track_saturn^=true;}
    if (menuUniverse.selection()==6) {systemData.sidereal_track_uranus^=true;}
    if (menuUniverse.selection()==7) {systemData.sidereal_track_neptune^=true;}
  }

  // dispaly page
  else if (menu_page==80) {

    // display auto off
    if (menuDisplay.selection()==0)  {systemData.display_auto_off^=true;}
    
    // iter display auto off timing
    if (menuDisplay.selection()==1)  {
      systemData.index_display_autooff_times++;
      if (systemData.index_display_autooff_times>systemData.max_display_autooff_times-1) {systemData.index_display_autooff_times=0;}
      systemData.display_timeout = systemData.display_autooff_times[systemData.index_display_autooff_times];
    }

    // iter display color
    if (menuDisplay.selection()==2) {systemData.index_display_color++;
      if (systemData.index_display_color>systemData.max_color_index-1) {systemData.index_display_color=0;}
      systemData.color_content=systemData.display_color[systemData.index_display_color];
      systemData.color_border=systemData.display_color[systemData.index_display_color];
    }
  }

  // system page
  else if (menu_page==90) {

    // startup run matrix
    if (menuSystem.selection()==0) {systemData.matrix_run_on_startup^=true;}
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                Crude Function Name to Associated Value Mapping 

String getRelatedY(char * data) {
  if (strcmp("DegGNGGARanges", data)==0) {return String(satData.location_longitude_gngga, 10);}
  if (strcmp("DegLatGNRMCRange", data)==0) {return String(satData.location_longitude_gnrmc, 10);}
  return String("");
}

String getRelatedZ(char * data) {
  return String("");
}

String getRelatedX(char * data) {
  // if (strcmp("None", data)==0) {return String();}
  // if (strcmp("Enabled", data)==0) {return String();}
  if (strcmp("Overload", data)==0) {return String(systemData.overload);}
  // if (strcmp("SwitchLink", data)==0) {return String();}
  // if (strcmp("SecondsTimer", data)==0) {return String();}

  // potentially redirect calls like these to existing values so that the values are already set before here: pros=calculate once, cons=stale data
  if (strcmp("RTCTimeOver", data)==0) {return String(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()));}
  if (strcmp("RTCTimeUnder", data)==0) {return String(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()));}
  if (strcmp("RTCTimeEqual", data)==0) {return String(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()));}
  if (strcmp("RTCTimeRange", data)==0) {return String(hoursMinutesSecondsToInt(rtc.now().hour(), rtc.now().minute(), rtc.now().second()));}
  if (strcmp("DaySunday", data)==0) {return String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()));}
  if (strcmp("DayMonday", data)==0) {return String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()));}
  if (strcmp("DayTuesday", data)==0) {return String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()));}
  if (strcmp("DayWednesday", data)==0) {return String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()));}
  if (strcmp("DayThursday", data)==0) {return String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()));}
  if (strcmp("DayFriday", data)==0) {return String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()));}
  if (strcmp("DaySaturday", data)==0) {return String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(),rtc.now().day()));}
  if (strcmp("DateDayX", data)==0) {return String(rtc.now().day());}
  if (strcmp("DateMonthX", data)==0) {return String(rtc.now().month());}
  if (strcmp("DateYearX", data)==0) {return String(rtc.now().year());}

  if (strcmp("DegLatGNGGAOver", data)==0) {return String(satData.location_latitude_gngga, 10);}
  if (strcmp("DegLatGNGGAUnder", data)==0) {return String(satData.location_latitude_gngga, 10);}
  if (strcmp("DegLatGNGGAEqual", data)==0) {return String(satData.location_latitude_gngga, 10);}
  if (strcmp("DegLatGNGGARange", data)==0) {return String(satData.location_latitude_gngga, 10);}
  if (strcmp("DegLonGNGGAOver", data)==0) {return String(satData.location_longitude_gngga, 10);}
  if (strcmp("DegLonGNGGAUnder", data)==0) {return String(satData.location_longitude_gngga, 10);}
  if (strcmp("DegLonGNGGAEqual", data)==0) {return String(satData.location_longitude_gngga, 10);}
  if (strcmp("DegLonGNGGARange", data)==0) {return String(satData.location_longitude_gngga, 10);}
  if (strcmp("DegGNGGARanges", data)==0) {return String(satData.location_latitude_gngga, 10);}
  if (strcmp("DegLatGNRMCOver", data)==0) {return String(satData.location_latitude_gnrmc, 10);}
  if (strcmp("DegLatGNRMCUnder", data)==0) {return String(satData.location_latitude_gnrmc, 10);}
  if (strcmp("DegLatGNRMCEqual", data)==0) {return String(satData.location_latitude_gnrmc, 10);}
  if (strcmp("DegLatGNRMCRange", data)==0) {return String(satData.location_latitude_gnrmc, 10);}
  if (strcmp("DegLonGNRMCOver", data)==0) {return String(satData.location_longitude_gnrmc, 10);}
  if (strcmp("DegLonGNRMCUnder", data)==0) {return String(satData.location_longitude_gnrmc, 10);}
  if (strcmp("DegLonGNRMCEqual", data)==0) {return String(satData.location_longitude_gnrmc, 10);}
  if (strcmp("DegLonGNRMCRange", data)==0) {return String(satData.location_longitude_gnrmc, 10);}
  if (strcmp("DegGNRMCRanges", data)==0) {return String(satData.location_latitude_gnrmc, 10);}
  if (strcmp("UTCTimeGNGGAOver", data)==0) {return String(gnggaData.utc_time);}
  if (strcmp("UTCTimeGNGGAUnder", data)==0) {return String(gnggaData.utc_time);}
  if (strcmp("UTCTimeGNGGAEqual", data)==0) {return String(gnggaData.utc_time);}
  if (strcmp("UTCTimeGNGGARange", data)==0) {return String(gnggaData.utc_time);}
  if (strcmp("LatGNGGAOver", data)==0) {return String(gnggaData.latitude, 10);}
  if (strcmp("LatGNGGAUnder", data)==0) {return String(gnggaData.latitude, 10);}
  if (strcmp("LatGNGGAEqual", data)==0) {return String(gnggaData.latitude, 10);}
  if (strcmp("LatGNGGARange", data)==0) {return String(gnggaData.latitude, 10);}
  if (strcmp("LonGNGGAOver", data)==0) {return String(gnggaData.longitude, 10);}
  if (strcmp("LonGNGGAUnder", data)==0) {return String(gnggaData.longitude, 10);}
  if (strcmp("LonGNGGAEqual", data)==0) {return String(gnggaData.longitude, 10);}
  if (strcmp("LonGNGGARange", data)==0) {return String(gnggaData.longitude, 10);}
  if (strcmp("PosStatusGNGGA", data)==0) {return String(gnggaData.solution_status);}
  if (strcmp("SatCountOver", data)==0) {return String(gnggaData.satellite_count_gngga);}
  if (strcmp("SatCountUnder", data)==0) {return String(gnggaData.satellite_count_gngga);}
  if (strcmp("SatCountEqual", data)==0) {return String(gnggaData.satellite_count_gngga);}
  if (strcmp("SatCountRange", data)==0) {return String(gnggaData.satellite_count_gngga);}
  if (strcmp("HemiGNGGANorth", data)==0) {return String(gnggaData.latitude_hemisphere);}
  if (strcmp("HemiGNGGASouth", data)==0) {return String(gnggaData.latitude_hemisphere);}
  if (strcmp("HemiGNGGAEast", data)==0) {return String(gnggaData.latitude_hemisphere);}
  if (strcmp("HemiGNGGAWest", data)==0) {return String(gnggaData.latitude_hemisphere);}
  if (strcmp("GPSPrecisionOver", data)==0) {return String(gnggaData.hdop_precision_factor, 2);}
  if (strcmp("GPSPrecisionUnder", data)==0) {return String(gnggaData.hdop_precision_factor, 2);}
  if (strcmp("GPSPrecisionEqual", data)==0) {return String(gnggaData.hdop_precision_factor, 2);}
  if (strcmp("GPSPrecisionRange", data)==0) {return String(gnggaData.hdop_precision_factor, 2);}
  if (strcmp("AltGNGGAOver", data)==0) {return String(gnggaData.altitude, 2);}
  if (strcmp("AltGNGGAUnder", data)==0) {return String(gnggaData.altitude, 2);}
  if (strcmp("AltGNGGAEqual", data)==0) {return String(gnggaData.altitude, 2);}
  if (strcmp("AltGNGGARange", data)==0) {return String(gnggaData.altitude, 2);}
  if (strcmp("UTCTimeGNRMCOver", data)==0) {return String(gnrmcData.utc_time);}
  if (strcmp("UTCTimeGNRMCUnder", data)==0) {return String(gnrmcData.utc_time);}
  if (strcmp("UTCTimeGNRMCEqual", data)==0) {return String(gnrmcData.utc_time);}
  if (strcmp("UTCTimeGNRMCRange", data)==0) {return String(gnrmcData.utc_time);}
  if (strcmp("PosStatusGNRMCA", data)==0) {return String(gnrmcData.positioning_status);}
  if (strcmp("PosStatusGNRMCV", data)==0) {return String(gnrmcData.positioning_status);}
  if (strcmp("ModeGNRMCA", data)==0) {return String(gnrmcData.mode_indication);}
  if (strcmp("ModeGNRMCD", data)==0) {return String(gnrmcData.mode_indication);}
  if (strcmp("ModeGNRMCE", data)==0) {return String(gnrmcData.mode_indication);}
  if (strcmp("ModeGNRMCN", data)==0) {return String(gnrmcData.mode_indication);}
  if (strcmp("LatGNRMCOver", data)==0) {return String(gnrmcData.latitude, 10);}
  if (strcmp("LatGNRMCUnder", data)==0) {return String(gnrmcData.latitude, 10);}
  if (strcmp("LatGNRMCEqual", data)==0) {return String(gnrmcData.latitude, 10);}
  if (strcmp("LatGNRMCRange", data)==0) {return String(gnrmcData.latitude, 10);}
  if (strcmp("LonGNRMCOver", data)==0) {return String(gnrmcData.longitude, 10);}
  if (strcmp("LonGNRMCUnder", data)==0) {return String(gnrmcData.longitude, 10);}
  if (strcmp("LonGNRMCEqual", data)==0) {return String(gnrmcData.longitude, 10);}
  if (strcmp("LonGNRMCRange", data)==0) {return String(gnrmcData.longitude, 10);}
  if (strcmp("HemiGNRMCNorth", data)==0) {return String(gnrmcData.latitude_hemisphere);}
  if (strcmp("HemiGNRMCSouth", data)==0) {return String(gnrmcData.latitude_hemisphere);}
  if (strcmp("HemiGNRMCEast", data)==0) {return String(gnrmcData.latitude_hemisphere);}
  if (strcmp("HemiGNRMCWest", data)==0) {return String(gnrmcData.latitude_hemisphere);}
  if (strcmp("GSpeedGNRMCOver", data)==0) {return String(gnrmcData.ground_speed, 2);}
  if (strcmp("GSpeedGNRMCUnder", data)==0) {return String(gnrmcData.ground_speed, 2);}
  if (strcmp("GSpeedGNRMCEqual", data)==0) {return String(gnrmcData.ground_speed, 2);}
  if (strcmp("GSpeedGNRMCRange", data)==0) {return String(gnrmcData.ground_speed, 2);}
  if (strcmp("HeadingGNRMCOver", data)==0) {return String(gnrmcData.ground_heading, 2);}
  if (strcmp("HeadingGNRMCUnder", data)==0) {return String(gnrmcData.ground_heading, 2);}
  if (strcmp("HeadingGNRMCEqual", data)==0) {return String(gnrmcData.ground_heading, 2);}
  if (strcmp("HeadingGNRMCRange", data)==0) {return String(gnrmcData.ground_heading, 2);}
  if (strcmp("UTCDateGNRMCOver", data)==0) {return String(gnrmcData.utc_date);}
  if (strcmp("UTCDateGNRMCUnder", data)==0) {return String(gnrmcData.utc_date);}
  if (strcmp("UTCDateGNRMCEqual", data)==0) {return String(gnrmcData.utc_date);}
  if (strcmp("UTCDateGNRMCRange", data)==0) {return String(gnrmcData.utc_date);}
  if (strcmp("LineFlagGPATTEqual", data)==0) {return String(gpattData.line_flag);}
  if (strcmp("StaticFlagGPATTEQ", data)==0) {return String(gpattData.static_flag);}
  if (strcmp("RStateFlagGPATTEQ", data)==0) {return String(gpattData.run_state_flag);}
  if (strcmp("INSGPATTEqual", data)==0) {return String(gpattData.ins);}
  if (strcmp("SpeedNumGPATTOver", data)==0) {return String(gpattData.speed_num);}
  if (strcmp("SpeedNumGPATTUnder", data)==0) {return String(gpattData.speed_num);}
  if (strcmp("SpeedNumGPATTEqual", data)==0) {return String(gpattData.speed_num);}
  if (strcmp("SpeedNumGPATTRange", data)==0) {return String(gpattData.speed_num);}
  if (strcmp("MileageGPATTOver", data)==0) {return String(gpattData.mileage, 2);}
  if (strcmp("MileageGPATTUnder", data)==0) {return String(gpattData.mileage, 2);}
  if (strcmp("MileageGPATTEqual", data)==0) {return String(gpattData.mileage, 2);}
  if (strcmp("MileageGPATTRange", data)==0) {return String(gpattData.mileage, 2);}
  if (strcmp("GSTDataGPATTOver", data)==0) {return String(gpattData.gst_data);}
  if (strcmp("GSTDataGPATTUnder", data)==0) {return String(gpattData.gst_data);}
  if (strcmp("GSTDataGPATTEqual", data)==0) {return String(gpattData.gst_data);}
  if (strcmp("GSTDataGPATTRange", data)==0) {return String(gpattData.gst_data);}
  if (strcmp("YawGPATTOver", data)==0) {return String(gpattData.yaw, 2);}
  if (strcmp("YawGPATTUnder", data)==0) {return String(gpattData.yaw, 2);}
  if (strcmp("YawGPATTEqual", data)==0) {return String(gpattData.yaw, 2);}
  if (strcmp("YawGPATTRange", data)==0) {return String(gpattData.yaw, 2);}
  if (strcmp("RollGPATTOver", data)==0) {return String(gpattData.roll, 2);}
  if (strcmp("RollGPATTUnder", data)==0) {return String(gpattData.roll, 2);}
  if (strcmp("RollGPATTEqual", data)==0) {return String(gpattData.roll, 2);}
  if (strcmp("RollGPATTRange", data)==0) {return String(gpattData.roll, 2);}
  if (strcmp("PitchGPATTOver", data)==0) {return String(gpattData.pitch, 2);}
  if (strcmp("PitchGPATTUnder", data)==0) {return String(gpattData.pitch, 2);}
  if (strcmp("PitchGPATTEqual", data)==0) {return String(gpattData.pitch, 2);}
  if (strcmp("PitchGPATTRange", data)==0) {return String(gpattData.pitch, 2);}
  if (strcmp("GNGGAValidCS", data)==0) {return String(gnggaData.valid_checksum);}
  if (strcmp("GNRMCValidCS", data)==0) {return String(gnrmcData.valid_checksum);}
  if (strcmp("GPATTValidCS", data)==0) {return String(gpattData.valid_checksum);}
  if (strcmp("GNGGAValidCD", data)==0) {return String(gnggaData.check_data);}
  if (strcmp("GNRMCValidCD", data)==0) {return String(gnrmcData.check_data);}
  if (strcmp("GPATTValidCD", data)==0) {return String(gpattData.check_data);}
  if (strcmp("SunAzRange", data)==0) {return String(siderealPlanetData.sun_az);}
  if (strcmp("SunAltRange", data)==0) {return String(siderealPlanetData.sun_alt);}

  // potentially redirect calls like these to existing values so that the values are already set before here  
  if (strcmp("DayTime", data)==0) {return String(check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
    siderealPlanetData.sun_r, siderealPlanetData.sun_s));}
  if (strcmp("NightTime", data)==0) {return String(check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()),
    siderealPlanetData.sun_r, siderealPlanetData.sun_s));}

  if (strcmp("Sunrise", data)==0) {return String(siderealPlanetData.sun_r);}
  if (strcmp("Sunset", data)==0) {return String(siderealPlanetData.sun_s);}
  if (strcmp("MoonAzRange", data)==0) {return String(siderealPlanetData.moon_az);}
  if (strcmp("MoonAltRange", data)==0) {return String(siderealPlanetData.moon_alt);}
  // if (strcmp("MoonUp", data)==0) {return String();}
  // if (strcmp("MoonDown", data)==0) {return String();}
  if (strcmp("Moonrise", data)==0) {return String(siderealPlanetData.moon_r);}
  if (strcmp("Moonset", data)==0) {return String(siderealPlanetData.moon_s);}
  if (strcmp("MoonPhase", data)==0) {return String(siderealPlanetData.moon_p);}
  if (strcmp("MercuryAzRange", data)==0) {return String(siderealPlanetData.mercury_az);}
  if (strcmp("MercuryAltRange", data)==0) {return String(siderealPlanetData.mercury_alt);}
  // if (strcmp("MercuryUp", data)==0) {return String();}
  // if (strcmp("MercuryDown", data)==0) {return String();}
  if (strcmp("MercuryRise", data)==0) {return String(siderealPlanetData.mercury_r);}
  if (strcmp("MercurySet", data)==0) {return String(siderealPlanetData.mercury_s);}
  if (strcmp("VenusAzRange", data)==0) {return String(siderealPlanetData.venus_az);}
  if (strcmp("VenusAltRange", data)==0) {return String(siderealPlanetData.venus_alt);}
  // if (strcmp("VenusUp", data)==0) {return String();}
  // if (strcmp("VenusDown", data)==0) {return String();}
  if (strcmp("VenusRise", data)==0) {return String(siderealPlanetData.venus_r);}
  if (strcmp("VenusSet", data)==0) {return String(siderealPlanetData.venus_s);}
  if (strcmp("MarsAzRange", data)==0) {return String(siderealPlanetData.mars_az);}
  if (strcmp("MarsAltRange", data)==0) {return String(siderealPlanetData.mars_alt);}
  // if (strcmp("MarsUp", data)==0) {return String();}
  // if (strcmp("MarsDown", data)==0) {return String();}
  if (strcmp("MarsRise", data)==0) {return String(siderealPlanetData.mars_r);}
  if (strcmp("MarsSet", data)==0) {return String(siderealPlanetData.mars_s);}
  if (strcmp("JupiterAzRange", data)==0) {return String(siderealPlanetData.jupiter_az);}
  if (strcmp("JupiterAltRange", data)==0) {return String(siderealPlanetData.jupiter_alt);}
  // if (strcmp("JupiterUp", data)==0) {return String();}
  // if (strcmp("JupiterDown", data)==0) {return String();}
  if (strcmp("JupiterRise", data)==0) {return String(siderealPlanetData.jupiter_r);}
  if (strcmp("JupiterSet", data)==0) {return String(siderealPlanetData.jupiter_s);}
  if (strcmp("SaturnAzRange", data)==0) {return String(siderealPlanetData.saturn_az);}
  if (strcmp("SaturnAltRange", data)==0) {return String(siderealPlanetData.saturn_alt);}
  // if (strcmp("SaturnUp", data)==0) {return String();}
  // if (strcmp("SaturnDown", data)==0) {return String();}
  if (strcmp("SaturnRise", data)==0) {return String(siderealPlanetData.saturn_r);}
  if (strcmp("SaturnSet", data)==0) {return String(siderealPlanetData.saturn_s);}
  if (strcmp("UranusAzRange", data)==0) {return String(siderealPlanetData.uranus_az);}
  if (strcmp("UranusAltRange", data)==0) {return String(siderealPlanetData.uranus_alt);}
  // if (strcmp("UranusUp", data)==0) {return String();}
  // if (strcmp("UranusDown", data)==0) {return String();}
  if (strcmp("UranusRise", data)==0) {return String(siderealPlanetData.uranus_r);}
  if (strcmp("UranusSet", data)==0) {return String(siderealPlanetData.uranus_s);}
  if (strcmp("NeptuneAzRange", data)==0) {return String(siderealPlanetData.neptune_az);}
  if (strcmp("NeptuneAltRange", data)==0) {return String(siderealPlanetData.neptune_alt);}
  // if (strcmp("NeptuneUp", data)==0) {return String();}
  // if (strcmp("NeptuneDown", data)==0) {return String();}
  if (strcmp("NeptuneRise", data)==0) {return String(siderealPlanetData.neptune_r);}
  if (strcmp("NeptuneSet", data)==0) {return String(siderealPlanetData.neptune_s);}
  if (strcmp("DHT11H0Under", data)==0) {return String(sensorData.dht11_h_0);}
  if (strcmp("DHT11H0Over", data)==0) {return String(sensorData.dht11_h_0);}
  if (strcmp("DHT11H0Equal", data)==0) {return String(sensorData.dht11_h_0);}
  if (strcmp("DHT11H0Range", data)==0) {return String(sensorData.dht11_h_0);}
  if (strcmp("DHT11C0Under", data)==0) {return String(sensorData.dht11_c_0);}
  if (strcmp("DHT11C0Over", data)==0) {return String(sensorData.dht11_c_0);}
  if (strcmp("DHT11C0Equal", data)==0) {return String(sensorData.dht11_c_0);}
  if (strcmp("DHT11C0Range", data)==0) {return String(sensorData.dht11_c_0);}
  if (strcmp("DHT11F0Under", data)==0) {return String(sensorData.dht11_f_0);}
  if (strcmp("DHT11F0Over", data)==0) {return String(sensorData.dht11_f_0);}
  if (strcmp("DHT11F0Equal", data)==0) {return String(sensorData.dht11_f_0);}
  if (strcmp("DHT11F0Range", data)==0) {return String(sensorData.dht11_f_0);}
  if (strcmp("DHT11HIC0Under", data)==0) {return String(sensorData.dht11_hic_0);}
  if (strcmp("DHT11HIC0Over", data)==0) {return String(sensorData.dht11_hic_0);}
  if (strcmp("DHT11HIC0Equal", data)==0) {return String(sensorData.dht11_hic_0);}
  if (strcmp("DHT11HIC0Range", data)==0) {return String(sensorData.dht11_hic_0);}
  if (strcmp("DHT11HIF0Under", data)==0) {return String(sensorData.dht11_hif_0);}
  if (strcmp("DHT11HIF0Over", data)==0) {return String(sensorData.dht11_hif_0);}
  if (strcmp("DHT11HIF0Equal", data)==0) {return String(sensorData.dht11_hif_0);}
  if (strcmp("DHT11HIF0Range", data)==0) {return String(sensorData.dht11_hif_0);}
  if (strcmp("PhotoRes0Under", data)==0) {return String(sensorData.photoresistor_0);}
  if (strcmp("PhotoRes0Over", data)==0) {return String(sensorData.photoresistor_0);}
  if (strcmp("PhotoRes0Equal", data)==0) {return String(sensorData.photoresistor_0);}
  if (strcmp("PhotoRes0Range", data)==0) {return String(sensorData.photoresistor_0);}
  return String("");
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                           UI

/*

IMPORTANT: beware of image retention and other damage that can be caused to OLED displays.

*/

// ------------------------------------------------
//                                        UI BORDER

void drawMainBorder() {
  display.setColor(systemData.color_border);
  display.drawRect(1, 1, 126, 126);
}

// ------------------------------------------------
//                                    UI BORDER RED

void drawMainBorderRed() {
  display.setColor(RGB_COLOR16(255,0,0));
  display.drawRect(1, 1, 126, 126);
}

void drawMainBorderGreen() {
  display.setColor(RGB_COLOR16(0,255,0));
  display.drawRect(1, 1, 126, 126);
}

void setMenuMatrixFilePathItems() {
    // set menu items
    menuMatrixFilepathItems[0] = sdcardData.matrix_filenames[0];
    menuMatrixFilepathItems[1] = sdcardData.matrix_filenames[1];
    menuMatrixFilepathItems[2] = sdcardData.matrix_filenames[2];
    menuMatrixFilepathItems[3] = sdcardData.matrix_filenames[3];
    menuMatrixFilepathItems[4] = sdcardData.matrix_filenames[4];
    menuMatrixFilepathItems[5] = sdcardData.matrix_filenames[5];
    menuMatrixFilepathItems[6] = sdcardData.matrix_filenames[6];
    menuMatrixFilepathItems[7] = sdcardData.matrix_filenames[7];
    menuMatrixFilepathItems[8] = sdcardData.matrix_filenames[8];
    menuMatrixFilepathItems[9] = sdcardData.matrix_filenames[9];
    menuMatrixFilepathItems[10] = sdcardData.matrix_filenames[10];
    menuMatrixFilepathItems[11] = sdcardData.matrix_filenames[11];
    menuMatrixFilepathItems[12] = sdcardData.matrix_filenames[12];
    menuMatrixFilepathItems[13] = sdcardData.matrix_filenames[13];
    menuMatrixFilepathItems[14] = sdcardData.matrix_filenames[14];
    menuMatrixFilepathItems[15] = sdcardData.matrix_filenames[15];
    menuMatrixFilepathItems[16] = sdcardData.matrix_filenames[16];
    menuMatrixFilepathItems[17] = sdcardData.matrix_filenames[17];
    menuMatrixFilepathItems[18] = sdcardData.matrix_filenames[18];
    menuMatrixFilepathItems[19] = sdcardData.matrix_filenames[19];
}

// ------------------------------------------------
//                                               UI

void UpdateUI() {

  // ------------------------------------------------
  //                                  OLED PROTECTION

  // oled protection: enable/disable ui updates
  if (systemData.display_auto_off==true) {
    if (rtc.now().unixtime() >= unixtime_control_panel_request+systemData.display_timeout) {update_ui=false;}
    else {update_ui=true;}
  }
  else {update_ui=true;}

  // ------------------------------------------------
  //                                DEVELOPER OPTIONS

  // update_ui = true; // uncomment to debug. warning: do not leave enabled or risk damaging your oled display. if this line is enabled then you are the screensaver.
  // menu_page=3; // uncomment to debug

  // ------------------------------------------------
  //                                  UPDATE UI PAGES

  if (update_ui==true) {
    ui_cleared = false;

    // Serial.println("[oled protection] allowing ui update");
    Serial.println("[menu page] " + String(menu_page));

    // ------------------------------------------------
    //                                        HOME PAGE

    if (menu_page==0) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      drawMainBorder();
      menuHome.show( display );
      canvas120x8.clear();
      canvas120x8.printFixed(2, 1, String(formatRTCTime()).c_str(), STYLE_BOLD );
      display.drawCanvas(3, 40, canvas120x8);
    }

    // ------------------------------------------------
    //                                    SETTINGS PAGE

    else if (menu_page==1) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("SETTINGS")/2)*6), 1, "SETTINGS", STYLE_BOLD );
      display.drawCanvas(4, 6, canvas120x8);

      // seperator
      display.drawHLine(2, 20, 126);

      menuMain.show( display );
    }

    // ------------------------------------------------
    //                         MATRIX SWITCH LOGIC PAGE

    else if (menu_page==3) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}

      drawMainBorder();

      // seperator (slightly lower than other header seperators to allow height space for combination bar)
      display.drawHLine(2, 26, 126);

      // function name
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
      display.drawCanvas(6, 33, canvas120x8);
      
      // function x
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "X ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 43, canvas120x8);

      // function y
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Y ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 53, canvas120x8);

      // function z
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Z ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 63, canvas120x8);

      // seperator
      display.drawHLine(2, 80, 126);

      // real time header
      // canvas120x8.clear();
      // canvas120x8.printFixed((120/2)-((strlen("REAL-TIME")/2)*6), 1, "REAL-TIME", STYLE_BOLD );
      // display.drawCanvas(2, 84, canvas120x8);

      // real x: display each functions associated value in 'real time' at the switch logic level (this level)
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "X ");
      strcat(TMP_UI_DATA_0, getRelatedX(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 90, canvas120x8);

      // real y: display each functions associated value in 'real time' at the switch logic level (this level)
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Y ");
      strcat(TMP_UI_DATA_0, getRelatedY(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 100, canvas120x8);

      // real z: display each functions associated value in 'real time' at the switch logic level (this level)
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Z ");
      strcat(TMP_UI_DATA_0, getRelatedZ(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 110, canvas120x8);

      // show the menu
      display.setColor(systemData.color_content);

      // clear any previously highlighted menus (the canvas needs to be slighly larger in dimensions to wipe all the menu away)
      if (previous_menu_column_selection!=menu_column_selection) {
        // menu
        canvas120x24.clear();
        display.drawCanvas(5, 2, canvas120x24);
        // set
        previous_menu_column_selection=menu_column_selection;
      }

      // highlight matrix switch select menu
      if (menu_column_selection == 0) {menuMatrixSwitchSelect.show( display );}
      else {
        // draw currently selected menu item when menu not highlighted
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "");
        strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelectItems[menuMatrixSwitchSelect.selection()]).c_str());
        canvas19x8.clear();
        canvas19x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(10, 9, canvas19x8);
      }

      // highlight matrix switch port select
      if (menu_column_selection == 1) {
        // port number
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "P");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
        canvas19x8.clear();
        // state on/off indicator (according to matrix_switch_state not a pin read)
        if (matrixData.matrix_switch_state[0][menuMatrixSwitchSelect.selection()]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        else {display.setColor(RGB_COLOR16(255,0,0));}
        canvas19x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_NORMAL);
        display.drawCanvas(39, 10, canvas19x8);
        display.setColor(systemData.color_content);
        display.drawRect(35, 6, 62, 21);
      }
      else {
        // draw currently selected menu item when menu not highlighted
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "P");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
        canvas19x8.clear();
        // state on/off indicator (according to matrix_switch_state not a pin read)
        if (matrixData.matrix_switch_state[0][menuMatrixSwitchSelect.selection()]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        else {display.setColor(RGB_COLOR16(255,0,0));}
        canvas19x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(39, 10, canvas19x8);
        display.setColor(systemData.color_content);
      }

      // highlight matrix switch enable/disable
      if (menu_column_selection == 2) {
        canvas8x8.clear();
        if (matrixData.matrix_switch_enabled[0][menuMatrixSwitchSelect.selection()]==true) {
          display.setColor(RGB_COLOR16(0,0,255)); // emphasis
          canvas8x8.printFixed(1, 1, "E", STYLE_NORMAL ); // enabled
        }
        else if (matrixData.matrix_switch_enabled[0][menuMatrixSwitchSelect.selection()]==false) {
          display.setColor(RGB_COLOR16(255,0,0)); // emphasis
          canvas8x8.printFixed(1, 1, "D", STYLE_NORMAL ); // disabled
        }
        display.drawCanvas(68, 10, canvas8x8);
        display.setColor(systemData.color_content);
        display.drawRect(66, 6, 79, 21);
      }
      else {
        // draw currently selected menu item when menu not highlighted
        canvas8x8.clear();
        if (matrixData.matrix_switch_enabled[0][menuMatrixSwitchSelect.selection()]==true) {
          display.setColor(RGB_COLOR16(0,0,255)); // emphasis
          canvas8x8.printFixed(1, 1, "E", STYLE_BOLD ); // enabled
        }
        else if (matrixData.matrix_switch_enabled[0][menuMatrixSwitchSelect.selection()]==false) {
          display.setColor(RGB_COLOR16(255,0,0)); // emphasis
          canvas8x8.printFixed(1, 1, "D", STYLE_BOLD ); // disabled
        }
        display.drawCanvas(68, 10, canvas8x8);
        display.setColor(systemData.color_content);
      }

      // highlight matrix switch inverted logic
      if (menu_column_selection == 3) {
        canvas8x8.clear();
        display.setColor(systemData.color_content);
        if (matrixData.matrix_switch_inverted_logic[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]==true) {
          display.setColor(RGB_COLOR16(255,255,0)); // emphasis
          canvas8x8.printFixed(1, 1, "I", STYLE_NORMAL ); // inverted function logic (not switch logic, this is per function on a switch) 
        }
        else if (matrixData.matrix_switch_inverted_logic[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]==false) {
          canvas8x8.printFixed(1, 1, "S", STYLE_NORMAL ); // standard function logic (not switch logic, this is per function on a switch) 
        }
        display.drawCanvas(84, 10, canvas8x8);
        display.setColor(systemData.color_content);
        display.drawRect(83, 6, 93, 21);
      }
      else {
        // draw currently selected menu item when menu not highlighted
        canvas8x8.clear();
        display.setColor(systemData.color_content);
        if (matrixData.matrix_switch_inverted_logic[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]==true) {
          display.setColor(RGB_COLOR16(255,255,0)); // emphasis
          canvas8x8.printFixed(1, 1, "I", STYLE_BOLD ); // inverted function logic (not switch logic, this is per function on a switch) 
        }
        else if (matrixData.matrix_switch_inverted_logic[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]==false) {
          canvas8x8.printFixed(1, 1, "S", STYLE_BOLD ); // standard function logic (not switch logic, this is per function on a switch) 
        }
        display.drawCanvas(83, 10, canvas8x8);
        display.setColor(systemData.color_content);
      }

      // highlight matrix switch function select menu
      if (menu_column_selection == 4) {menuMatrixFunctionSelect.show( display );}
      else {
        // draw currently selected menu item when menu not highlighted
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "");
        strcat(TMP_UI_DATA_0, String(menuMatrixFunctionSelectItems[menuMatrixFunctionSelect.selection()]).c_str());
        canvas19x8.clear();
        canvas19x8.printFixed(5, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(91+4, 9, canvas19x8);
        // display.drawRect(91+4, 6, 91+30, 21); // uncomment to border
      }
      
    }

    // ------------------------------------------------
    //                                ENTER DIGITS PAGE

    // enter digits page
    else if (menu_page==4) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}

      drawMainBorderRed();

      canvas120x8.clear();
      if (enter_digits_key==1) {canvas120x8.printFixed((120/2)-((strlen("ENTER PORT NUMBER")/2)*6), 1, "ENTER PORT NUMBER", STYLE_BOLD );}
      else if (enter_digits_key==2) {canvas120x8.printFixed((120/2)-((strlen("ENTER VALUE X")/2)*6), 1, "ENTER VALUE X", STYLE_BOLD );}
      else if (enter_digits_key==3) {canvas120x8.printFixed((120/2)-((strlen("ENTER VALUE Y")/2)*6), 1, "ENTER VALUE Y", STYLE_BOLD );}
      else if (enter_digits_key==4) {canvas120x8.printFixed((120/2)-((strlen("ENTER VALUE Z")/2)*6), 1, "ENTER VALUE Z", STYLE_BOLD );}
      display.drawCanvas(2, 6, canvas120x8);

      // seperator
      display.drawHLine(2, 20, 126);

      if (enter_digits_key==1) {

        // matrix switch number 
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "M");
        strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelect.selection()).c_str());
        strcat(TMP_UI_DATA_0, " / P");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(3, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, 26, canvas120x8);
      }

      else if ((enter_digits_key==2) || (enter_digits_key==3) || (enter_digits_key==4)) {

        // matrix switch number 
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "M");
        strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelect.selection()).c_str());
        strcat(TMP_UI_DATA_0, " / F");
        strcat(TMP_UI_DATA_0, String(menuMatrixFunctionSelect.selection()).c_str());
        strcat(TMP_UI_DATA_0, " / P");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(3, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, 26, canvas120x8);

        // function name
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
        display.drawCanvas(6, 36, canvas120x8);
        
        // function x
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "X ");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(6, 46, canvas120x8);

        // function y
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Y ");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(6, 56, canvas120x8);

        // function z
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Z ");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(6, 66, canvas120x8);

        // real x: display each functions associated value in 'real time' at the switch logic level (this level)
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "X ");
        strcat(TMP_UI_DATA_0, getRelatedX(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(6, 76, canvas120x8);

        // real y: display each functions associated value in 'real time' at the switch logic level (this level)
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Y ");
        strcat(TMP_UI_DATA_0, getRelatedY(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(6, 86, canvas120x8);

        // real z: display each functions associated value in 'real time' at the switch logic level (this level)
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Z ");
        strcat(TMP_UI_DATA_0, getRelatedZ(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(6, 96, canvas120x8);
      }

      // seperator
      display.drawHLine(2, 108, 126);

      canvas120x8.clear();
      // canvas120x8.printFixed(3, 1, String(input_data).c_str(), STYLE_BOLD );
      canvas120x8.printFixed((120/2)-((strlen(String(input_data).c_str())/2)*6), 1, String(input_data).c_str(), STYLE_BOLD );
      display.drawCanvas(2, 112, canvas120x8);
    }

    // ------------------------------------------------
    //                     SELECT FUNCTION OPTIONS PAGE

    // select function name, x, y, or z
    else if (menu_page==5) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}

      canvas120x8.setFixedFont(ssd1306xled_font6x8);

      drawMainBorder();

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("SETUP SWITCH LOGIC")/2)*6), 1, "SETUP SWITCH LOGIC", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // seperator
      display.drawHLine(2, 20, 126);

      // matrix switch number 
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "M");
      strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelect.selection()).c_str());
      strcat(TMP_UI_DATA_0, " / F");
      strcat(TMP_UI_DATA_0, String(menuMatrixFunctionSelect.selection()).c_str());
      strcat(TMP_UI_DATA_0, " / P");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(3, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 26, canvas120x8);

      // function name
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
      display.drawCanvas(6, 36, canvas120x8);
      
      // function x
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "X ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 46, canvas120x8);

      // function y
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Y ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 56, canvas120x8);

      // function z
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Z ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(6, 66, canvas120x8);

      // show the menu
      menuMatrixConfigureFunction.show( display );
    }

    // ------------------------------------------------
    //                        SELECT FUNCTION NAME PAGE

    // matrix switch set function name
    else if (menu_page==6) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("SELECT FUNCTION")/2)*6), 1, "SELECT FUNCTION", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // seperator
      display.drawHLine(2, 20, 126);

      // matrix switch number 
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "M");
      strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelect.selection()).c_str());
      strcat(TMP_UI_DATA_0, " / F");
      strcat(TMP_UI_DATA_0, String(menuMatrixFunctionSelect.selection()).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(3, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 26, canvas120x8);

      // function name
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
      display.drawCanvas(6, 36, canvas120x8);

      menuMatrixSetFunctionName.show( display );
    }

    // ------------------------------------------------
    //                                        FILE MENU

    else if (menu_page==20) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("FILE")/2)*6), 1, "FILE", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      menuFile.show( display );
    }

    // ------------------------------------------------
    //                                 SAVE MATRIX MENU

    // save matrix
    else if (menu_page==21) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("SAVE")/2)*6), 1, "SAVE", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // set items each iteration so that if changed anywhere will be reflected in ui
      setMenuMatrixFilePathItems();

      // show menu
      menuMatrixFilepath.show( display );
      
    }

    // ------------------------------------------------
    //                                 LOAD MATRIX MENU

    // load matrix
    else if (menu_page==22) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("LOAD")/2)*6), 1, "LOAD", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // set items each iteration so that if changed anywhere will be reflected in ui
      setMenuMatrixFilePathItems();

      // show menu
      menuMatrixFilepath.show( display );
    }

    // ------------------------------------------------
    //                               DELETE MATRIX MENU

    // delete matrix
    else if (menu_page==23) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("DELETE")/2)*6), 1, "DELETE", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // set items each iteration so that if changed anywhere will be reflected in ui
      setMenuMatrixFilePathItems();

      // show menu
      menuMatrixFilepath.show( display );
    }

    // ------------------------------------------------
    //                            SAVE MATRIX INDICATOR

    // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
    else if (menu_page==30) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(RGB_COLOR16(0,255,0));
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("SAVING")/2)*6), (display.height()/2)-16, "SAVING", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }

    // ------------------------------------------------
    //                            LOAD MATRIX INDICATOR

    // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
    else if (menu_page==31) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(RGB_COLOR16(0,255,0));
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("LOADING")/2)*6), (display.height()/2)-16, "LOADING", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }

    // ------------------------------------------------
    //                          DELETE MATRIX INDICATOR

    // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
    if (menu_page==32) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(RGB_COLOR16(0,255,0));
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("DELETING")/2)*6), (display.height()/2)-16, "DELETING", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }

    // ------------------------------------------------
    //                   SAVING SYSTEM CONFIG INDICATOR

    // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
    if (menu_page==33) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(RGB_COLOR16(0,255,0));
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("SAVING")/2)*6), (display.height()/2)-16, "SAVING", STYLE_BOLD );
      canvas120x120.printFixed((120/2)-((strlen("SYSTEM CONFIG")/2)*6), (display.height()/2), "SYSTEM CONFIG", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }

    // ------------------------------------------------
    //        RESTORING DEFAULT SYSTEM CONFIG INDICATOR

    // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
    if (menu_page==34) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(RGB_COLOR16(0,255,0));
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("RESTORING")/2)*6), (display.height()/2)-16, "RESTORING", STYLE_BOLD );
      canvas120x120.printFixed((120/2)-((strlen("SYSTEM CONFIG")/2)*6), (display.height()/2), "SYSTEM CONFIG", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }

    // restore default system settings

    // ------------------------------------------------
    //                                         GPS MENU

    else if (menu_page==50) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("GPS")/2)*6), 1, "GPS", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // set items each iteration so that if changed anywhere will be reflected in ui

      if (systemData.satio_enabled==true) {menuGPSItems[0]="SATIO ENABLED";}
      else {menuGPSItems[0]="SATIO DISABLED";}

      if (systemData.gngga_enabled==true) {menuGPSItems[1]="GNGGA ENABLED";}
      else {menuGPSItems[1]="GNGGA DISABLED";}

      if (systemData.gnrmc_enabled==true) {menuGPSItems[2]="GNRMC ENABLED";}
      else {menuGPSItems[2]="GNRMC DISABLED";}

      if (systemData.gpatt_enabled==true) {menuGPSItems[3]="GPATT ENABLED";}
      else {menuGPSItems[3]="GPATT DISABLED";}

      if (strcmp(satData.coordinate_conversion_mode, "GNGGA")==0) {menuGPSItems[4]="COORD CONVERT GNGGA";}
      if (strcmp(satData.coordinate_conversion_mode, "GNRMC")==0) {menuGPSItems[4]="COORD CONVERT GNRMC";}

      // show menu
      menuGPS.show( display );
    }

    // ------------------------------------------------
    //                                      SERIAL MENU

    /* output data to be parsed by other systems or to be read by humans */

    else if (menu_page==60) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("SERIAL")/2)*6), 1, "SERIAL", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // set items each iteration so that if changed anywhere will be reflected in ui

      if (systemData.output_satio_enabled==true) {menuSerialItems[0]="SATIO ENABLED";}
      else {menuSerialItems[0]="SATIO DISABLED";}

      if (systemData.output_gngga_enabled==true) {menuSerialItems[1]="GNGGA ENABLED";}
      else {menuSerialItems[1]="GNGGA DISABLED";}

      if (systemData.output_gnrmc_enabled==true) {menuSerialItems[2]="GNRMC ENABLED";}
      else {menuSerialItems[2]="GNRMC DISABLED";}

      if (systemData.output_gpatt_enabled==true) {menuSerialItems[3]="GPATT ENABLED";}
      else {menuSerialItems[3]="GPATT DISABLED";}

      if (systemData.output_matrix_enabled==true) {menuSerialItems[4]="MATRIX ENABLED";}
      else {menuSerialItems[4]="MATRIX DISABLED";}

      // show menu
      menuSerial.show( display );
    }

    // ------------------------------------------------
    //                SOLAR AND PLANETARY TRACKING MENU

    /* currently solar system tracking */

    else if (menu_page==70) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("UNIVERSE")/2)*6), 1, "UNIVERSE", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // set items each iteration so that if changed anywhere will be reflected in ui

      if (systemData.sidereal_track_sun==true) {menuUniverseItems[0]="SUN ENABLED";} 
      else {menuUniverseItems[0]="SUN DISABLED";}

      if (systemData.sidereal_track_mercury==true) {menuUniverseItems[1]="MERCURY ENABLED";}
      else {menuUniverseItems[1]="MERCURY DISABLED";}

      if (systemData.sidereal_track_venus==true) {menuUniverseItems[2]="VENUS ENABLED";}
      else {menuUniverseItems[2]="VENUS DISABLED";}

      if (systemData.sidereal_track_mars==true) {menuUniverseItems[3]="MARS ENABLED";}
      else {menuUniverseItems[3]="MARS DISABLED";}

      if (systemData.sidereal_track_jupiter==true) {menuUniverseItems[4]="JUPITER ENABLED";}
      else {menuUniverseItems[4]="JUPITER DISABLED";}

      if (systemData.sidereal_track_saturn==true) {menuUniverseItems[5]="SATURN ENABLED";}
      else {menuUniverseItems[5]="SATURN DISABLED";}

      if (systemData.sidereal_track_uranus==true) {menuUniverseItems[6]="URANUS ENABLED";}
      else {menuUniverseItems[6]="URANUS DISABLED";}

      if (systemData.sidereal_track_neptune==true) {menuUniverseItems[7]="NEPTUNE ENABLED";}
      else {menuUniverseItems[7]="NEPTUNE DISABLED";}

      // show menu
      menuUniverse.show( display );
    }

    // ------------------------------------------------
    //                                     DISPLAY MENU

    else if (menu_page==80) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("DISPLAY")/2)*6), 1, "DISPLAY", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // auto off
      if (systemData.display_auto_off==true) {menuDisplayItems[0]="AUTO OFF ENABLED";}
      else {menuDisplayItems[0]="AUTO OFF DISABLED";}

      // auto off time
      menuDisplayItems[1] = systemData.char_display_autooff_times[systemData.index_display_autooff_times];

      // color
      menuDisplayItems[2] = systemData.char_display_color[systemData.index_display_color];

      // show menu
      menuDisplay.show( display );
    }

    // ------------------------------------------------
    //                                      SYSTEM MENU

    else if (menu_page==90) {
      if (menu_page != previous_menu_page) {previous_menu_page=menu_page; display.clear();}
      display.setColor(systemData.color_content);

      drawMainBorder();

      // seperator
      display.drawHLine(2, 20, 126);

      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen("SYSTEM")/2)*6), 1, "SYSTEM", STYLE_BOLD );
      display.drawCanvas(3, 6, canvas120x8);

      // run matrix on startup
      if (systemData.matrix_run_on_startup==true) {menuSystemItems[0]="AUTO MATRIX ON";}
      else {menuSystemItems[0]="AUTO MATRIX OFF";}

      // show menu
      menuSystem.show( display );
    }
  }

  // ------------------------------------------------
  //                                  OLED PROTECTION

  // oled protection:
  if ((ui_cleared == false) && (update_ui == false)) {
    Serial.println("[oled protection] clearing ui");
    display.clear();
    display.clear();
    display.clear();
    ui_cleared=true;
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA

#define I2C_ADDR_CONTROL_PANEL_0 8
#define I2C_ADDR_PORTCONTROLLER_0 9

int I2C_ADDRESSES[] = {
  I2C_ADDR_CONTROL_PANEL_0,
  I2C_ADDR_PORTCONTROLLER_0
};

struct I2CLinkStruct {
  char * token;
  byte OUTPUT_BUFFER[10]; // bytes to be sent
  char INPUT_BUFFER[10];  // chars received
  char TMP_BUFFER_0[10];  // chars of bytes to be sent
  char TMP_BUFFER_1[10];  // some space for type conversions
};
I2CLinkStruct I2CLink;

void writeI2C(int I2C_Address) {
  // compile bytes array
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i] = (byte)I2CLink.TMP_BUFFER_0[i];}
  // begin
  Wire.beginTransmission(I2C_Address);
  // write bytes array
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
  // end
  Wire.endTransmission();
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                I2C PERIPHERALS

/*

I2C peripheral: interrupts us to let us know it has something we need.

SatIO: makes i2c requests (possibly with an address sweep for scalability so that all i2c peripheral interrupt on the same pin).

Note: In the future, you should be able to add new I2C devices after flashing, through the ui and control panel:
      1: add address
      2: add $tag
      3: add custom function name.
      This will make it even simpler than already to build a new I2C peripheral and add it to the system.

      Or auto:
      1: make an i2 address sweep within a certain address range.
      2: if you hear back then you have a device address and the device can tell you:
          1: name. what the device is? so that 'what it is' is not ambiguous.
          2: tags. what have you got and how to parse it (the human name(s) of the data its giving you).
          3: append primitives (over, under, equal, inrange) to the human names.
          4: put new concatinated name(s) in matrix function names.
          ... so you can just make an i2c peripheral/module, plug it in on the i2c bus and it works how we need.
          extra note: the custom i2c peripherals should be designed to think within the realms of 'primitives', so that being
          the ones who created the custom i2c device, we are also the ones who know what a simple integer means if and when we check
          for that simple integer in the matrix in regards to each and any given custom i2c device.
*/

void ISR_I2C_PERIPHERAL() {
  // Serial.println("[ISR] ISR_I2C_PERIPHERAL");
  make_i2c_request = true;
}

void makeI2CRequest() {

  // make i2c request if interrupt flag true 
  if (make_i2c_request == true) {
    make_i2c_request = false;

    // request
    Serial.println("[master] making request");
    Wire.requestFrom(I2C_ADDR_CONTROL_PANEL_0,sizeof(I2CLink.INPUT_BUFFER));

    // receive
    memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
    Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
    Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));

    // 1: record time of any activity from the i2c control panel.
    unixtime_control_panel_request = rtc.now().unixtime();
    Serial.println("[unixtime_control_panel_request] " + String(unixtime_control_panel_request));

    // blind button press protection: ignore button presses when screen is a sleep/off/blank (some buttons may be moved out of this block)
    if (update_ui==true) {

      // parse special interrupt buttons
      if (strcmp(I2CLink.INPUT_BUFFER, "$B,ISR0")==0) {Serial.println("[button] ISR0");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,ISR1")==0) {Serial.println("[button] ISR1");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,ISR2")==0) {Serial.println("[button] ISR2");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,ISR3")==0) {Serial.println("[button] ISR3");}

      // parse numpad buttons
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,0")==0) {Serial.println("[button] 0"); inputChar(digit_0);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,1")==0) {Serial.println("[button] 1"); inputChar(digit_1);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,2")==0) {Serial.println("[button] 2"); inputChar(digit_2);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,3")==0) {Serial.println("[button] 3"); inputChar(digit_3);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,4")==0) {Serial.println("[button] 4"); inputChar(digit_4);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,5")==0) {Serial.println("[button] 5"); inputChar(digit_5);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,6")==0) {Serial.println("[button] 6"); inputChar(digit_6);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,7")==0) {Serial.println("[button] 7"); inputChar(digit_7);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,8")==0) {Serial.println("[button] 8"); inputChar(digit_8);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,9")==0) {Serial.println("[button] 9"); inputChar(digit_9);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,10")==0) {Serial.println("[button] 10: ."); inputChar(period_char);}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,11")==0) {Serial.println("[button] 11: -"); inputChar(hyphen_char);}

      // parse navigation buttons
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,12")==0) {Serial.println("[button] 12: home"); menu_page=0;}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,13")==0) {Serial.println("[button] 13: up"); menuUp();}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,14")==0) {Serial.println("[button] 14: right"); menuRight();}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,15")==0) {Serial.println("[button] 15: down"); menuDown();}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,16")==0) {Serial.println("[button] 16: left"); menuLeft();}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,17")==0) {Serial.println("[button] 17: enter"); menuEnter();}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,18")==0) {Serial.println("[button] 18: back");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,19")==0) {Serial.println("[button] 19: delete");}

      // parse currently spare creative potential buttons: (auto input with set_var_x set_var_y set_var_z) (clear)
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,20")==0) {Serial.println("[button] 20");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,21")==0) {Serial.println("[button] 21");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,22")==0) {Serial.println("[button] 22");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,23")==0) {Serial.println("[button] 23");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,24")==0) {Serial.println("[button] 24");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,25")==0) {Serial.println("[button] 25");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,26")==0) {Serial.println("[button] 26");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,27")==0) {Serial.println("[button] 27");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,28")==0) {Serial.println("[button] 28");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,29")==0) {Serial.println("[button] 29");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,30")==0) {Serial.println("[button] 30");}
      else if (strcmp(I2CLink.INPUT_BUFFER, "$B,31")==0) {Serial.println("[button] 31");}
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                PORT CONTROLLER

void writeToPortController() {

  // Serial.println("[writeToPortController]");

  // Port Map: $P,X,Y
  for (int i=0; i < 20; i++) {
    // Serial.println("[matrix_port_map] " + String(matrixData.matrix_port_map[0][i]) + " [tmp_matrix_port_map] " + String(matrixData.tmp_matrix_port_map[0][i]));
    // check for change
    if (matrixData.matrix_port_map[0][i] != matrixData.tmp_matrix_port_map[0][i]) {
      // update
      matrixData.tmp_matrix_port_map[0][i] = matrixData.matrix_port_map[0][i];

      memset(I2CLink.TMP_BUFFER_0, 0, sizeof(I2CLink.TMP_BUFFER_0));
      // tag
      strcpy(I2CLink.TMP_BUFFER_0, "$P,");
      // index
      itoa(i, I2CLink.TMP_BUFFER_1, 10);
      strcat(I2CLink.TMP_BUFFER_0, I2CLink.TMP_BUFFER_1);
      strcat(I2CLink.TMP_BUFFER_0, ",");
      // port number
      itoa(matrixData.matrix_port_map[0][i], I2CLink.TMP_BUFFER_1, 10);
      strcat(I2CLink.TMP_BUFFER_0, I2CLink.TMP_BUFFER_1);

      writeI2C(I2C_ADDR_PORTCONTROLLER_0);
    }
  }

  // Matrix Switch True/False: $M,X,Y
  for (int i=0; i < 20; i++) {
    // Serial.println("[matrix_switch_state] " + String(matrixData.matrix_switch_state[0][i]) + " [tmp_matrix_switch_state] " + String(matrixData.tmp_matrix_switch_state[0][i]));
    // check for change
    if (matrixData.matrix_switch_state[0][i] != matrixData.tmp_matrix_switch_state[0][i]) {
      // update
      matrixData.tmp_matrix_switch_state[0][i] = matrixData.matrix_switch_state[0][i];

      memset(I2CLink.TMP_BUFFER_0, 0, sizeof(I2CLink.TMP_BUFFER_0));
      // tag
      strcpy(I2CLink.TMP_BUFFER_0, "$M,");
      // index
      itoa(i, I2CLink.TMP_BUFFER_1, 10);
      strcat(I2CLink.TMP_BUFFER_0, I2CLink.TMP_BUFFER_1);
      strcat(I2CLink.TMP_BUFFER_0, ",");
      // true/false
      itoa(matrixData.matrix_switch_state[0][i], I2CLink.TMP_BUFFER_1, 10);
      strcat(I2CLink.TMP_BUFFER_0, I2CLink.TMP_BUFFER_1);

      writeI2C(I2C_ADDR_PORTCONTROLLER_0);
    }
  }

  // Satellite Count and HDOP Precision Factor Indicator
  memset(I2CLink.TMP_BUFFER_0, 0, sizeof(I2CLink.TMP_BUFFER_0));
  // tag
  strcpy(I2CLink.TMP_BUFFER_0, "$GPSSIG,");
  // data
  if (atoi(gnggaData.satellite_count_gngga)==0) {strcat(I2CLink.TMP_BUFFER_0, "0");}
  else if ((atoi(gnggaData.satellite_count_gngga)>0) && (atof(gnggaData.hdop_precision_factor)>1.0)) {strcat(I2CLink.TMP_BUFFER_0, "1");}
  else if ((atoi(gnggaData.satellite_count_gngga)>0) && (atof(gnggaData.hdop_precision_factor)<=1.0)) {strcat(I2CLink.TMP_BUFFER_0, "2");}
  writeI2C(I2C_ADDR_PORTCONTROLLER_0);

  // Overload Indicator
  memset(I2CLink.TMP_BUFFER_0, 0, sizeof(I2CLink.TMP_BUFFER_0));
  // tag
  strcpy(I2CLink.TMP_BUFFER_0, "$OLOAD,");
  // data
  if (systemData.overload==false) {strcat(I2CLink.TMP_BUFFER_0, "0");}
  else {strcat(I2CLink.TMP_BUFFER_0, "1");}
  writeI2C(I2C_ADDR_PORTCONTROLLER_0);

  // Uncomment if and when hearing back from the peripheral is required
  // Serial.println("[master] read data");
  // Wire.requestFrom(I2C_ADDR_PORTCONTROLLER_0,sizeof(input_buffer));
  // memset(input_buffer, 0, sizeof(input_buffer));
  // Wire.readBytesUntil('\n', input_buffer, sizeof(input_buffer));
  // Serial.println("[received] " + String(input_buffer));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                        SDCARD: FULL INITIALIZE

void setupSDCard() {
  /*
  initializes sdcard, attempts to load saved system configuration file and saved matrix file. creates new directory tree, system file
  and matrix file if not exists.
  */

 uint8_t cardType;
  if (SD.begin(SD_CS, sdspi)) {
    cardType = SD.cardType();
    Serial.println("[SDCARD] Initialized.");
    Serial.print("[SDCARD] Type: ");
    if (cardType == CARD_MMC) {Serial.println("MMC");}
    else if (cardType == CARD_SD) {Serial.println("SDSC.");}
    else if (cardType == CARD_SDHC) {Serial.println("SDHC.");}
    else {Serial.println("UNKNOWN.");}
  }
  else {Serial.println("[SDCARD] Failed to initialize.");}

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
//                                                                                                                  SDCARD: CHECK

void sdcardCheck() {
  uint8_t cardType;
  if (SD.begin(SD_CS, sdspi)) {
    cardType = SD.cardType();
    Serial.println("[SDCARD] Initialized.");
    Serial.print("[SDCARD] Type: ");
    if (cardType == CARD_MMC) {Serial.println("MMC");}
    else if (cardType == CARD_SD) {Serial.println("SDSC.");}
    else if (cardType == CARD_SDHC) {Serial.println("SDHC.");}
    else {Serial.println("UNKNOWN.");}
  }
  else {Serial.println("[SDCARD] Failed to initialize.");}
  // create/load system files
  if (!(cardType == CARD_NONE)) {
    Serial.println("[sdcard] initialized");
    // return true;
  }
  // return false;
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

int gps_read_t;
bool gps_done = false;
int gps_done_t = millis();
int i_gps = 0;
bool cs = false;

void readGPS(void * pvParameters) {
  // Serial.println("[readGPS] ");

  while (1) {

    if (gps_done==false) {

      gps_done_t = millis();
      serial1Data.gngga_bool = false;
      serial1Data.gnrmc_bool = false;
      serial1Data.gpatt_bool = false;
      memset(gnggaData.sentence, 0, sizeof(gnggaData.sentence));
      memset(gnrmcData.sentence, 0, sizeof(gnrmcData.sentence));
      memset(gpattData.sentence, 0, sizeof(gpattData.sentence));

      /* 
      this setup should read every sentence (gngga, desbi, gpatt, gnrmc) coming from the WTGPS300P once every 100ms.
       */

      for (i_gps = 0; i_gps < 10; i_gps++) {
        if (Serial2.available()) {

          if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}

          // gps_read_t = millis();

          memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
          
          SerialLink.nbytes = Serial2.readBytesUntil('\n', SerialLink.BUFFER, sizeof(SerialLink.BUFFER));

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
      sensorData.photoresistor_0 = analogRead(CD74HC4067_SIG);
      // Serial.println("[photoresistor_0] " + String(sensorData.photoresistor_0));

      // dht11
      setMultiplexChannel_CD74HC4067(1);
      sensorData.dht11_h_0 = dht.readHumidity();
      sensorData.dht11_c_0 = dht.readTemperature();     // celsius default
      sensorData.dht11_f_0 = dht.readTemperature(true); // fahreheit = true
      if (isnan(sensorData.dht11_h_0) || isnan(sensorData.dht11_c_0) || isnan(sensorData.dht11_f_0)) {
        Serial.println("Failed to read from DHT sensor!");
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
  // rtc_wdt_protect_off();
  // rtc_wdt_disable();

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                SETUP: SERIAL

  Serial.println("[setup] serial");

  Serial.setRxBufferSize(2000); // ensure this is set before begin()
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200); while(!Serial);

  // ESP32 can map hardware serial to alternative pins.
  Serial2.setPins(27, -1, ctsPin, rtsPin); // serial to gps module. ensure this is set before begin()
  Serial2.setRxBufferSize(2000); // ensure this is set before begin()
  Serial2.setTimeout(50); // ensure this is set before begin()
  Serial2.begin(115200);

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                  SETUP: PINS

  Serial.println("[setup] pins");

  pinMode(CD74HC4067_S0, OUTPUT); 
  pinMode(CD74HC4067_S1, OUTPUT); 
  pinMode(CD74HC4067_S2, OUTPUT); 
  pinMode(CD74HC4067_S3, OUTPUT); 
  pinMode(CD74HC4067_SIG, INPUT); 
  digitalWrite(CD74HC4067_S0, LOW);
  digitalWrite(CD74HC4067_S1, LOW);
  digitalWrite(CD74HC4067_S2, LOW);
  digitalWrite(CD74HC4067_S3, LOW);

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

  /*

  Interrupt line: connects one or more I2C peripherals so they can tell us when to make a request.

  */

  pinMode(ISR_I2C_PERIPHERAL_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(ISR_I2C_PERIPHERAL_PIN), ISR_I2C_PERIPHERAL, FALLING);

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                  SETUP: WIRE 

  
  setMultiplexChannel_CD74HC4067(1);
  
  Serial.println("[setup] wire");
  Wire.begin();  // sets up the I2C  

  Serial.println("[setup] selecting i2C channel: 0");
  setMultiplexChannel_TCA9548A(0);  // set i2c multiplexer channel

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
  Serial.println("[CONFIG_ESP32_REV_MIN] " + String(CONFIG_ESP32_REV_MIN));
  Serial.println("[CONFIG_LOG_DEFAULT_LEVEL] " + String(CONFIG_LOG_DEFAULT_LEVEL));
  Serial.println("[CONFIG_BOOTLOADER_LOG_LEVEL] " + String(CONFIG_BOOTLOADER_LOG_LEVEL));
  Serial.println("[CONFIG_ESP_CONSOLE_UART_BAUDRATE] " + String(CONFIG_ESP_CONSOLE_UART_BAUDRATE));
  Serial.println("[CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL] " + String(CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL));
  // IRAM https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html#iram
  Serial.println("[getCpuFrequencyMhz] " + String(getCpuFrequencyMhz()));
  Serial.println("[APB_CLK_FREQ] " + String(getApbFrequency()));

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   SETUP: SPI

  // VSPI: SDCARD
  // SD_SCLK = 18;  // default esp32 VSPI
  // SD_MISO = 19;  // default esp32 VSPI
  // SD_MOSI = 23;  // default esp32 VSPI
  // SD_CS   = 5;   // default esp32 VSPI
  // pinMode(SD_CS, OUTPUT);
  // digitalWrite(SD_CS, HIGH);

  // VSPI: SSD1351 OLED DISPLAY
  // SSD1351_SCLK = 14; // (SCL) default esp32 HSPI
  // SSD1351_MISO = 12; // (DC)  default esp32 HSPI
  // SSD1351_MOSI = 13; // (SDA) default esp32 HSPI
  // SSD1351_CS   = 26; // (CS)   custom esp32 HSPI
  // pinMode(SSD1351_CS, OUTPUT);
  // digitalWrite(SSD1351_CS, HIGH);

  // VSPI: SDCARD
  beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  setupSDCard();
  SD.end();
  endSPIDevice(SD_CS);

  // HSPI: SSD1351 OLED DISPLAY
  beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
  display.begin();
  display.setFixedFont(ssd1306xled_font6x8);
  display.fill( 0x0000 );
  menu_page=0;
  canvas6x8.setFixedFont(ssd1306xled_font6x8);
  canvas8x8.setFixedFont(ssd1306xled_font6x8);
  canvas19x8.setFixedFont(ssd1306xled_font6x8);
  canvas120x8.setFixedFont(ssd1306xled_font6x8);
  canvas120x120.setFixedFont(ssd1306xled_font6x8);
  display.clear();
  // uncomment to debug
  // canvas.printFixed(1, 1, " SATIO ", STYLE_BOLD );
  // display.drawCanvas(1, 1, canvas);
  // display.end();
  // endSPIDevice(SSD1351_CS);

  // delay(2000);

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                            SETUP: CORE TASKS

  // Create task to increase performance (core 0 also found to be best for this task)
  xTaskCreatePinnedToCore(
      readGPS, /* Function to implement the task */
      "Task0", /* Name of the task */
      4096,   /* Stack size in words */
      NULL,    /* Task input parameter */
      2,       /* Priority of the task */
      &Task0,  /* Task handle. */
      0);      /* Core where the task should run */
    
  // Create task to increase performance (core 0 also found to be best for this task)
  xTaskCreatePinnedToCore(
    getSensorData, /* Function to implement the task */
    "Task1",       /* Name of the task */
    4096,         /* Stack size in words */
    NULL,          /* Task input parameter */
    2,             /* Priority of the task */
    &Task1,        /* Task handle. */
    0);            /* Core where the task should run */
  
  // // Create task to increase performance (core 0 also found to be best for this task)
  // xTaskCreatePinnedToCore(
  //   UpdateUI, /* Function to implement the task */
  //   "Task2",       /* Name of the task */
  //   4096,         /* Stack size in words */
  //   NULL,          /* Task input parameter */
  //   1,             /* Priority of the task */
  //   &Task2,        /* Task handle. */
  //   0);            /* Core where the task should run */

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
bool longer_loop = false;

void loop() {

  // Serial.println("----------------------------------------");
  // Serial.println("[loop] ");

  timeData.mainLoopTimeStart = millis();
  i_loops_between_gps_reads++;

  // uncomment to override default values
  // systemData.matrix_enabled = true;
  // systemData.output_matrix_enabled = true;
  // systemData.matrix_run_on_startup = true;

  makeI2CRequest();

  // ---------------------------------------------------------------------
  //                                                                   GPS

  /*
  ensure safe execution each loop. final matrix values for port controller
  must not be altered while instructing port controller. this must be true
  while also instructing port controller every loop and while not blocking
  the loop so that we can utilize the port controller for other instructions
  and do other things if needed until gps data is ready. the WTGPSs300P outputs
  each sentence (gngga, gpatt, gnrmc, desbi) 10 times a second, every 100
  milliseconds.
  */

  // default is false
  longer_loop = false;

  if (gps_done==true) {

    // mark this loop as taking longer
    longer_loop = true;

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
    writeToPortController();
    // Serial.println("[writePortController] " + String(millis()-t0));

    gps_done = false;
    sensors_done=false;
  }

  /*

  this helps keep the system fast accross loops taking different times, by utilizing loops that take less time to complete.
  note that this is currently suitable in this case while timing or other conditions may be more suitable in other cases.
  this currently allows for text to be updated a little more than 10 times a second.

  anything graphically intensive can be placed on a task and be written to areas of the display that are not being written to
  by updateUI function which should be reserved for text.

  updating text procedurally at the end of each loop also means that we avoid any race conditions where if updateUI was running
  on a task then it may try and display values that are currently being overwritten by other tasks/functions.

  update text at the end of each loop possible.

  run graphics on tasks when and if required.

  aim to always stay well below the 100ms looptime for every loop.

  dont wait for anything, get what we can and go again.

  */

  if (longer_loop==false) {
    t0 = millis();
    UpdateUI();
    Serial.println("[UpdateUI] " + String(millis()-t0));
  }

  // ---------------------------------------------------------------------

  if (interrupt_second_counter > 0) {
    portENTER_CRITICAL(&second_timer_mux);
    interrupt_second_counter--;
    track_planets_period = true;
    portEXIT_CRITICAL(&second_timer_mux);
  }

  // delay(1000); // debug test overload: increase loop time
  timeData.mainLoopTimeTaken = (millis() - timeData.mainLoopTimeStart);
  if (timeData.mainLoopTimeTaken>=100) {systemData.overload=true;} // gps module outputs every 100ms
  else {systemData.overload=false;}
  // if (timeData.mainLoopTimeTaken > timeData.mainLoopTimeTakenMax) {timeData.mainLoopTimeTakenMax = timeData.mainLoopTimeTaken;}
  // if (timeData.mainLoopTimeTaken < timeData.mainLoopTimeTakenMin) {timeData.mainLoopTimeTakenMin = timeData.mainLoopTimeTaken;}

  // some data while running headless
  Serial.println("[UTC_Datetime]          " + String(gnrmcData.utc_time) + " " + String(String(gnrmcData.utc_date))); // (at this point stale)
  Serial.println("[RTC Datetime]          " + formatRTCTime()); // fresh from RTC
  // Serial.println("[Satellite Count]       " + String(gnggaData.satellite_count_gngga));
  // Serial.println("[HDOP Precision Factor] " + String(gnggaData.hdop_precision_factor));
  // Serial.println("[gnrmcData.latitude]    " + String(gnrmcData.latitude));
  // Serial.println("[gnrmcData.longitude]   " + String(gnrmcData.longitude));
  // Serial.println("[photoresistor_0]       " + String(sensorData.photoresistor_0));
  // Serial.println("[dht11_hic_0]           " + String(sensorData.dht11_hic_0));
  Serial.println("[Looptime]              " + String(timeData.mainLoopTimeTaken));
  // Serial.println("[Looptime Max] " + String(timeData.mainLoopTimeTakenMax));
  // Serial.println("[Looptime Min] " + String(timeData.mainLoopTimeTakenMin));

  // delay(500);
  
  // ---------------------------------------------------------------------
}