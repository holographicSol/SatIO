/*

                                  SatIO - Written by Benjamin Jack Cullen.

                                            "The GPS Master"

                      A general purpose programmable satellite, sensor and inertial platform.
    Supporting stacks (up to 10 functions per output pin) of logic across 20 output pins on the portcontroller.

                                 SatIO is the system, a matrix is the program.

        Design: Break out all the things and build I2C peripherals as required to orbit the ESP32/Central-MCU.

                                
                                Wiring For Keystudio ESP32 PLUS Development Board

                                ESP32: 1st ATMEGA2560 with shield as Port Controller (not on multiplexer):
                                ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                ESP32: 2nd ATMEGA2560 with shield as Control Panel (not on multiplexer):
                                ESP32: io25    -> ATMEGA2560: io22
                                ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                Other ESP32 i2C Devices (not on multiplexer):
                                ESP32: SDA0 SCL0 -> DS3231 (RTC): SDA, SCL (5v)

                                ESP32: WTGPS300P (5v) (for getting a downlink):
                                ESP32: io27 RXD -> WTGPS300P: TXD
                                ESP32: null TXD -> WTGPS300P: RXD

                                ESP32 i2C: i2C Multiplexing (3.3v) (for peripherals):
                                ESP32: i2C -> TCA9548A: SDA, SCL

                                ESP32: Analog/Digital Multiplexing (3.3v) (for peripherals):
                                ESP32: io4    -> CD74HC4067: SIG
                                ESP32: io32   -> CD74HC4067: S0
                                ESP32: io33   -> CD74HC4067: S1
                                ESP32: io16   -> CD74HC4067: S2
                                ESP32: io17   -> CD74HC4067: S3
                                CD74HC4067 C0 -> DHT11: SIG

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



                                           $SATIO SENTENCE

                                            System Uptime                    
        Tag                  Last Sync      |                               Degrees Longitude        
        |      yyyymmddhhmmss|yyyymmddhhmmss|s|hh.mm|hh.mm|                 |                 |                
        $SATIO,00000000000000,00000000000000,0,00.00,00.00,00.00000000000000,00.00000000000000,*Z
              |              |                |     |     |                 |                 |            
              RTC Datetime                    |     |     Degrees Latitude                    Checksum            
                                              |     Sun Set
                                              Sun Rise



                                          $MATRIX SENTENCE 

                                                                              Matrix Switch Output Port 19
                                                                              |
                                                                              |    Matrix Switch State 0
                                                                              |    |
    $MATRIX,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
           |                                                                                                                                                   |
          Matrix Switch Output Port 0                                                                                                                          Matrix Switch State 19
                                                                                          


                                          $SENSORS SENTENCE

                      Sensor 0
                      |
              $SENSORS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
                                                                                  |
                                                                                  Sensor 15



                                          $SUN SENTENCE
                                                    
                                    Right Ascension 
                                    |       Azimuth 
                                    |       |       Rise
                                    |       |       |
                                $SUN,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
                                        |       |       |
                                        |       |       Set 
                                        |       Altitude
                                        Declination 



                                          $MOON SENTENCE

                                                Rise
                                Right Ascension |
                                |       Azimuth | 
                                |       |       |       Phase
                                |       |       |       |
                          $MOON,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,*CHECKSUM
                                    |       |       |       |
                                    |       |       Set     Luminessence
                                    |       Altitude
                                    Declination



                                        $MERCURY SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
              $MERCURY,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude  



                                         $VENUS SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
                $VENUS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude  



                                        $MARS SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
                 $MARS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude



                                      $JUPITER SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
              $JUPITER,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude



                                      $SATURN SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
               $SATURN,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude



                                      $URANUS SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
               $URANUS,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude 



                                      $NEPTUNE SENTENCE

                                      Rise
                      Right Ascension |       Helio Ecliptic Latitude
                      |       Azimuth |       |       Radius Vector   
                      |       |       |       |       |       Ecliptic Latitude
                      |       |       |       |       |       |
              $NEPTUNE,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0*CHECKSUM
                          |       |       |       |       |       |
                          |       |       Set     |       |       Ecliptic Longitude
                          |       Altitude        |       Distance
                          Declination             Helio Ecliptic Longitude  



  Use case: Its a PLC, use your imagination. Automate all the things. Robots, flying machines, sensor drones
  or to provide data to local LLM's over serial, the list goes on.
  
  Flexibility: The system is designed to be highly flexible, so that input/output/calculations of all kinds can
  be turned on/off.

  Port Controller: Port controller to turn pins high/low according to instructions received from master.

  UI: Allows programming matrix switch logic and tuning for individual use cases. Emphasis to importance, clarity,
  consistency.
  
  Summary: Over one quintillion possible combinations of stackable logic across 20 switches for a general purpose
  part, subsystem or standalone device.

  Whats the point? Working with ESP32 is cheap and from this project I intend to have reusable, general purpose parts
  as modules that can work both together and standalone, creating a platform I can go to when working with ESP32.

  Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
  of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html).
  Additions: 1: doXRiseSetTimes(). This allows for calculating rise and set times of all planets and objects according to time and location.
             2: inRange60(). Ensures minutes and second values are wihin 0-59 for planet/object rise, set times.
             3: inRange24(). Ensures hour values are wihin 0-23 for planet/object rise, set times.
  
  Writes to SSD1351 are now performed on a xTask, allowing for up to around 100 <10ms loops per second that can be utilized
  for other things. There are some pixel artifacts to deal with since migrating updateui to xTask, which were not noticeable when
  updating ui more slowly in main loop.
*/

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      LIBRARIES

#include <Arduino.h>
#include "soc/rtc_wdt.h"
#include "esp_pm.h"
#include "esp_attr.h"
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <iostream>
#include "SdFat.h"
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include <TimeLib.h>          // https://github.com/PaulStoffregen/Time
#include <Timezone.h>         // https://github.com/JChristensen/Timezone
#include <SiderealPlanets.h>  // https://github.com/DavidArmstrong/SiderealPlanets
#include <SiderealObjects.h>  // https://github.com/DavidArmstrong/SiderealObjects
#include <DHT.h>
#include <CD74HC4067.h>
#include "lcdgfx.h"
#include "lcdgfx_gui.h"
#include <assert.h>

void beginSDCARD();
void endSDCARD();
void beginSSD1351();
void endSSD1351();
bool sdcardCheck();
// void UpdateUI();
void readI2C();
void UIIndicators();

bool gps_done = false; // helps avoid any potential race conditions where gps data is collected on another task

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
  Note that this is a preliminary begin to be called before a 'library specific begin' like sd.begin() for example when stacking
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
//                                                                                                                 DISPLAY WIRING

// SSD1351 HSPI pins on esp32 with custom CS
int SSD1351_SCLK = 14; // (SCL)
int SSD1351_MISO = 12; // (DC)
int SSD1351_MOSI = 13; // (SDA)
int SSD1351_CS   = 26; // (CS)

// The parameters are  RST pin, BUS number, CS pin, DC pin, FREQ (0 means default), CLK pin, MOSI pin
DisplaySSD1351_128x128x16_SPI display( (int8_t)-1, {  (int8_t)-1,  (int8_t)SSD1351_CS,  (int8_t)SSD1351_MISO,  (int8_t)0,  (int8_t)-1,  (int8_t)-1  });

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 DISPLAY CANVAS

NanoCanvas<8,8,1> canvas8x8; 
NanoCanvas<19,8,1> canvas19x8;
NanoCanvas<120,8,1> canvas120x8;
NanoCanvas<60,8,1> canvas60x8;
NanoCanvas<80,8,1> canvas80x8;
NanoCanvas<126,24,1> canvas126x24;
NanoCanvas<120,120,1> canvas120x120;
NanoCanvas<128,128,1> canvas128x128;
NanoCanvas<28,8,1> canvas28x8;
NanoCanvas<21,8,1> canvas21x8;
NanoCanvas<42,8,1> canvas42x8;
NanoCanvas<54,8,1> canvas54x8;
NanoCanvas<36,8,1> canvas36x8;
NanoCanvas<92,8,1> canvas92x8;
NanoPoint sprite;
NanoEngine16<DisplaySSD1351_128x128x16_SPI> engine( display );


const uint8_t UnidentifiedStudioBMP[] PROGMEM =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x21, 0x04, 0x21, 0x04, 0x29, 0x25, 0x21, 0x04, 0x18, 0xa2, 0x10, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x29, 0x25, 0x21, 0x24, 0x19, 0x23, 0x18, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x21, 0x04, 0x21, 0x04, 0x29, 0x24, 0x21, 0x04, 0x21, 0x04, 0x21, 0x05, 0x29, 0x25, 0x29, 0x05, 0x18, 0x63, 0x00, 0x00, 0x00, 0x60, 0x01, 0xc2, 0x1b, 0x04, 0x12, 0xa3, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x03, 0x29, 0x04, 0x29, 0x04, 0x29, 0x04, 0x29, 0x04, 0x29, 0x05, 0x21, 0x04, 0x21, 0x03, 0x10, 0x00, 0x00, 0x60, 0x1b, 0xe3, 0x1e, 0xe3, 0x1f, 0x63, 0x1f, 0x43, 0x1d, 0xc3, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x18, 0xa3, 0x21, 0x25, 0x19, 0x05, 0x19, 0x25, 0x19, 0x04, 0x21, 0x25, 0x29, 0x05, 0x18, 0xa3, 0x00, 0x00, 0x12, 0xa0, 0x1f, 0x02, 0x17, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x82, 0x1c, 0x83, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xe0, 0x01, 0x80, 0x01, 0xc2, 0x11, 0xe2, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x01, 0xc0, 0x01, 0x20, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x04, 0x21, 0x05, 0x21, 0x05, 0x21, 0x05, 0x21, 0x24, 0x21, 0x24, 0x18, 0xa3, 0x00, 0x60, 0x13, 0xc2, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xc0, 0x1e, 0x43, 0x00, 0xa0, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x80, 0x13, 0xe2, 0x15, 0x82, 0x1e, 0x43, 0x16, 0x83, 0x1e, 0xc3, 0x1e, 0xc3, 0x1e, 0xe3, 0x1f, 0x03, 0x1e, 0xe3, 0x1e, 0xc3, 0x16, 0x42, 0x1d, 0xc3, 0x14, 0xa2, 0x12, 0xe2, 0x11, 0x82, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x62, 0x29, 0x04, 0x29, 0x04, 0x21, 0x04, 0x21, 0x23, 0x29, 0x24, 0x18, 0xa3, 0x00, 0x00, 0x12, 0xc2, 0x1f, 0x43, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xa0, 0x1d, 0x23, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x13, 0x40, 0x1e, 0x63, 0x17, 0x83, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc0, 0x17, 0xa2, 0x1f, 0x63, 0x1e, 0xa3, 0x1d, 0x43, 0x13, 0xa2, 0x01, 0xe0, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x21, 0x03, 0x21, 0x04, 0x29, 0x24, 0x21, 0x03, 0x20, 0xe3, 0x00, 0x00, 0x00, 0xe0, 0x1c, 0xe3, 0x17, 0x82, 0x07, 0xc0, 0x17, 0xc2, 0x1e, 0x83, 0x11, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x14, 0x82, 0x1f, 0x83, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xe2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe2, 0x07, 0xe0, 0x07, 0xe2, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe2, 0x17, 0x83, 0x1e, 0xe3, 0x1d, 0x43, 0x02, 0xe0, 0x01, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x19, 0x04, 0x21, 0x05, 0x21, 0x05, 0x21, 0x25, 0x21, 0x04, 0x18, 0xa3, 0x00, 0x00, 0x00, 0xe0, 0x13, 0x62, 0x1c, 0xc2, 0x14, 0x02, 0x01, 0xc0, 0x00, 0x60, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x15, 0x43, 0x17, 0xa3, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x83, 0x1e, 0xe4, 0x1e, 0x23, 0x1d, 0xe3, 0x15, 0xe3, 0x1e, 0x43, 0x26, 0xa5, 0x27, 0x25, 0x1f, 0x63, 0x17, 0xa2, 0x07, 0xe2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xc2, 0x1f, 0x83, 0x1e, 0x23, 0x13, 0xa3, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x18, 0xa3, 0x29, 0x05, 0x29, 0x05, 0x19, 0x05, 0x21, 0x24, 0x21, 0x05, 0x18, 0x63, 0x10, 0x02, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xa0, 0x14, 0x82, 0x17, 0xa2, 0x07, 0xe0, 0x17, 0xc2, 0x17, 0xc2, 0x17, 0x63, 0x1d, 0x83, 0x02, 0xe0, 0x01, 0x00, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x01, 0x20, 0x02, 0x20, 0x13, 0x42, 0x14, 0xa3, 0x1e, 0x23, 0x27, 0x04, 0x1f, 0x83, 0x17, 0x82, 0x1f, 0x63, 0x27, 0x24, 0x1e, 0x83, 0x15, 0x83, 0x1c, 0x45, 0x1a, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0xe2, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x40, 0x02, 0x20, 0x02, 0x00, 0x02, 0x40, 0x02, 0x40, 0x01, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x18, 0xa3, 0x29, 0x04, 0x29, 0x05, 0x21, 0x23, 0x21, 0x24, 0x21, 0x05, 0x18, 0xe3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa4, 0x29, 0x24, 0x18, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0xa2, 0x1f, 0x03, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xc0, 0x1e, 0xa3, 0x13, 0x02, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xe0, 0x01, 0xe0, 0x02, 0xe0, 0x03, 0x20, 0x02, 0xa0, 0x01, 0x80, 0x01, 0x00, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x63, 0x18, 0xa3, 0x18, 0xe3, 0x21, 0x04, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x21, 0x04, 0x18, 0xa3, 0x10, 0x62, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x2e, 0x06, 0x1f, 0x23, 0x17, 0x43, 0x17, 0x42, 0x1f, 0x43, 0x1f, 0x23, 0x1f, 0x23, 0x1f, 0x43, 0x1f, 0x43, 0x1f, 0x23, 0x1f, 0x23, 0x1f, 0x43, 0x1f, 0x23, 0x1f, 0x43, 0x1f, 0x23, 0x26, 0xc4, 0x13, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x04, 0x21, 0x23, 0x21, 0x25, 0x21, 0x05, 0x21, 0x24, 0x21, 0x24, 0x29, 0x04, 0x21, 0x05, 0x21, 0x25, 0x29, 0x24, 0x29, 0x04, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1d, 0xa3, 0x17, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0xe2, 0x02, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x21, 0x04, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x18, 0xe3, 0x18, 0xa3, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0xa0, 0x1d, 0x24, 0x1e, 0x62, 0x1e, 0xa3, 0x17, 0x23, 0x17, 0xa2, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xc2, 0x17, 0x03, 0x13, 0x22, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x21, 0x04, 0x29, 0x05, 0x29, 0x05, 0x29, 0x24, 0x21, 0x24, 0x21, 0x23, 0x29, 0x05, 0x21, 0x05, 0x21, 0x04, 0x21, 0x04, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x62, 0x17, 0x23, 0x07, 0xc0, 0x07, 0xc0, 0x17, 0x63, 0x13, 0xa2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x21, 0x04, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x19, 0x24, 0x19, 0x03, 0x10, 0x62, 0x10, 0x02, 0x00, 0x00, 0x01, 0x00, 0x01, 0x60, 0x01, 0x60, 0x02, 0x40, 0x14, 0xe2, 0x1f, 0x23, 0x17, 0x82, 0x17, 0xa2, 0x17, 0xc2, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc2, 0x1e, 0xe3, 0x13, 0x22, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x03, 0x21, 0x04, 0x21, 0x25, 0x21, 0x05, 0x29, 0x04, 0x29, 0x06, 0x21, 0x05, 0x29, 0x24, 0x21, 0x04, 0x21, 0x04, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0x22, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xc0, 0x1d, 0xe3, 0x00, 0xe0, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x26, 0x29, 0x25, 0x20, 0xe4, 0x10, 0xa2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x12, 0x62, 0x13, 0x62, 0x13, 0xc2, 0x04, 0x42, 0x1e, 0x03, 0x1f, 0x62, 0x07, 0xc0, 0x07, 0xe2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x02, 0x13, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa2, 0x29, 0x04, 0x29, 0x05, 0x21, 0x25, 0x21, 0x24, 0x21, 0x05, 0x21, 0x05, 0x29, 0x25, 0x21, 0x04, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x15, 0x62, 0x17, 0xc2, 0x07, 0xe0, 0x07, 0xc0, 0x13, 0xa2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x31, 0x06, 0x29, 0x26, 0x29, 0x25, 0x21, 0x64, 0x19, 0x03, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x01, 0x00, 0x14, 0x03, 0x1e, 0x03, 0x1e, 0x63, 0x1e, 0xa3, 0x1f, 0x02, 0x17, 0xa2, 0x17, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x03, 0x13, 0x22, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x18, 0x62, 0x20, 0xe3, 0x29, 0x24, 0x21, 0x23, 0x21, 0x24, 0x29, 0x05, 0x29, 0x25, 0x21, 0x04, 0x29, 0x25, 0x18, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1e, 0x23, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x82, 0x12, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x18, 0xe3, 0x10, 0x62, 0x18, 0xe3, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x26, 0x31, 0x26, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x18, 0xe3, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0xa0, 0x02, 0x20, 0x1d, 0xc3, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xc0, 0x17, 0xc2, 0x1e, 0xe3, 0x13, 0x22, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x18, 0xa3, 0x21, 0x23, 0x29, 0x24, 0x29, 0x24, 0x21, 0x04, 0x21, 0x04, 0x29, 0x25, 0x18, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x16, 0xa3, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x42, 0x02, 0x40, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x21, 0x45, 0x29, 0x45, 0x31, 0x26, 0x29, 0x26, 0x31, 0x25, 0x29, 0x64, 0x29, 0x45, 0x29, 0x05, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x00, 0x15, 0xa2, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x23, 0x12, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x16, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x23, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x04, 0x29, 0x25, 0x29, 0x45, 0x21, 0x04, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x21, 0x65, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x44, 0x21, 0x64, 0x29, 0x64, 0x29, 0x45, 0x29, 0x45, 0x20, 0xe4, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x40, 0x16, 0x62, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x15, 0x03, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1e, 0x83, 0x07, 0xe0, 0x07, 0xc2, 0x17, 0x23, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x31, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x24, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x60, 0x04, 0x22, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xc2, 0x07, 0xe0, 0x1d, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1e, 0x83, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x42, 0x02, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x10, 0x62, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x13, 0x62, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xe0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1e, 0x63, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x62, 0x02, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x10, 0x62, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x13, 0x02, 0x1f, 0x83, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1d, 0xe3, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xa2, 0x13, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x21, 0x04, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x22, 0x26, 0xe4, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0xe2, 0x17, 0xc2, 0x07, 0xe0, 0x07, 0xc2, 0x14, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x18, 0xa3, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1d, 0x03, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xe0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x13, 0xe2, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xe0, 0x1d, 0x83, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x40, 0x17, 0x22, 0x07, 0xe0, 0x07, 0xe0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x62, 0x1f, 0x43, 0x07, 0xe0, 0x07, 0xe2, 0x1e, 0xe3, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x60, 0x16, 0x62, 0x07, 0xe0, 0x07, 0xe0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1e, 0x63, 0x07, 0xc2, 0x07, 0xe0, 0x17, 0x82, 0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1e, 0x23, 0x07, 0xe0, 0x07, 0xe0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x15, 0x22, 0x17, 0xc2, 0x07, 0xe0, 0x07, 0xe2, 0x1d, 0x03, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x21, 0x24, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1d, 0xc3, 0x07, 0xe0, 0x07, 0xe0, 0x14, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x60, 0x13, 0x22, 0x17, 0x63, 0x07, 0xe0, 0x07, 0xc0, 0x26, 0xe3, 0x01, 0x40, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x18, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1c, 0x03, 0x1f, 0x63, 0x07, 0xe0, 0x15, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x60, 0x1e, 0xa3, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xc2, 0x13, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x21, 0x04, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x18, 0x63, 0x00, 0x00, 0x00, 0x60, 0x01, 0x20, 0x1d, 0xc3, 0x07, 0xe0, 0x15, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x15, 0x02, 0x17, 0xc2, 0x07, 0xc0, 0x07, 0xc0, 0x1e, 0x03, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x46, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x05, 0x10, 0x02, 0x00, 0x00, 0x00, 0x60, 0x14, 0x82, 0x07, 0xc0, 0x1c, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x12, 0xa2, 0x1f, 0x43, 0x07, 0xc0, 0x07, 0xe0, 0x1f, 0x83, 0x02, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x25, 0x29, 0x45, 0x29, 0x44, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x44, 0x29, 0x45, 0x31, 0x26, 0x18, 0xa3, 0x00, 0x02, 0x00, 0x00, 0x14, 0x02, 0x17, 0xc0, 0x1c, 0xe3, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1d, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x15, 0x83, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xe4, 0x29, 0x25, 0x29, 0x44, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x31, 0x25, 0x29, 0x05, 0x10, 0x62, 0x00, 0x00, 0x13, 0xa2, 0x27, 0x43, 0x1c, 0xc3, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xa2, 0x17, 0x83, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x43, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x26, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x26, 0x29, 0x25, 0x29, 0x45, 0x29, 0x44, 0x29, 0x45, 0x29, 0x45, 0x18, 0xe3, 0x00, 0x00, 0x01, 0x80, 0x1b, 0xc3, 0x1a, 0xa3, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40, 0x1e, 0x83, 0x07, 0xc0, 0x07, 0xe0, 0x17, 0xa2, 0x1d, 0x03, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x05, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x26, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x44, 0x29, 0x44, 0x10, 0x62, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x10, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0x43, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x03, 0x02, 0x62, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x44, 0x18, 0xe3, 0x10, 0x02, 0x00, 0x02, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x1f, 0x43, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xc2, 0x1d, 0x23, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x23, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x26, 0x29, 0x26, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x15, 0x62, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x03, 0x12, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x10, 0xa2, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x26, 0x29, 0x26, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x26, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x26, 0x21, 0x45, 0x29, 0x45, 0x31, 0x25, 0x29, 0x26, 0x29, 0x25, 0x29, 0x44, 0x31, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x31, 0x24, 0x31, 0x25, 0x29, 0x25, 0x29, 0x26, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x02, 0xa0, 0x1f, 0x63, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xc0, 0x15, 0x42, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x20, 0xe3, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x31, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x26, 0x29, 0x45, 0x29, 0x64, 0x29, 0x45, 0x29, 0x26, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x21, 0x45, 0x29, 0x44, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x21, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1d, 0xc4, 0x17, 0xc2, 0x07, 0xc0, 0x07, 0xe0, 0x1f, 0x03, 0x12, 0xe2, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x31, 0x25, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x46, 0x21, 0x46, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x21, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x26, 0x29, 0x26, 0x29, 0x26, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x44, 0x29, 0x45, 0x29, 0x46, 0x29, 0x46, 0x29, 0x46, 0x29, 0x45, 0x29, 0x46, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x26, 0x29, 0x46, 0x21, 0x65, 0x29, 0x45, 0x29, 0x26, 0x29, 0x45, 0x29, 0x45, 0x10, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x00, 0x00, 0x13, 0x02, 0x17, 0x63, 0x07, 0xc0, 0x07, 0xe2, 0x17, 0xa2, 0x1e, 0x03, 0x01, 0x20, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x46, 0x29, 0x46, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x44, 0x29, 0x25, 0x29, 0x25, 0x31, 0x25, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x21, 0x64, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x26, 0x29, 0x44, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x21, 0x65, 0x31, 0x24, 0x31, 0x05, 0x29, 0x26, 0x29, 0x45, 0x29, 0x45, 0x29, 0x26, 0x29, 0x45, 0x29, 0x45, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xa2, 0x18, 0xe3, 0x00, 0x00, 0x00, 0xa0, 0x1d, 0x24, 0x1f, 0x45, 0x1f, 0x64, 0x1f, 0x63, 0x2f, 0x05, 0x23, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x21, 0x03, 0x29, 0x45, 0x31, 0x25, 0x31, 0x06, 0x29, 0x46, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x21, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x31, 0x26, 0x29, 0x25, 0x29, 0x25, 0x29, 0x24, 0x31, 0x25, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x31, 0x25, 0x29, 0x45, 0x31, 0x25, 0x29, 0x25, 0x31, 0x26, 0x29, 0x26, 0x29, 0x25, 0x29, 0x45, 0x29, 0x44, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x21, 0x65, 0x29, 0x45, 0x29, 0x43, 0x29, 0x24, 0x29, 0x25, 0x29, 0x43, 0x31, 0x25, 0x31, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x21, 0x46, 0x31, 0x25, 0x31, 0x24, 0x29, 0x25, 0x29, 0x44, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x05, 0x20, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x01, 0xc0, 0x02, 0x00, 0x01, 0xe0, 0x01, 0x80, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x02, 0x10, 0x02, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0x62, 0x18, 0xa3, 0x18, 0xe4, 0x18, 0xe3, 0x10, 0x63, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x00, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0x63, 0x18, 0x63, 0x18, 0x63, 0x10, 0x62, 0x10, 0x62, 0x10, 0x63, 0x10, 0x63, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x63, 0x10, 0x62, 0x10, 0x60, 0x10, 0x62, 0x10, 0x62, 0x00, 0x60, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x60, 0x01, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x01, 0x80, 0x01, 0x40, 0x00, 0x60, 0x01, 0x20, 0x01, 0x80, 0x01, 0x60, 0x01, 0x20, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x01, 0x20, 0x01, 0x80, 0x01, 0x20, 0x00, 0xa0, 0x01, 0x20, 0x01, 0x60, 0x01, 0x80, 0x01, 0x80, 0x01, 0x60, 0x00, 0xe0, 0x00, 0x60, 0x01, 0x00, 0x01, 0x80, 0x01, 0x80, 0x01, 0x60, 0x01, 0x60, 0x01, 0x00, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x40, 0x01, 0x80, 0x01, 0x60, 0x01, 0x80, 0x01, 0xa0, 0x01, 0x80, 0x01, 0x60, 0x01, 0x00, 0x00, 0xe0, 0x01, 0x80, 0x01, 0x80, 0x01, 0x40, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x60, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x40, 0x01, 0xa0, 0x01, 0x40, 0x00, 0x60, 0x01, 0x00, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0xa0, 0x00, 0x60, 0x01, 0x00, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x60, 0x01, 0x40, 0x00, 0xe0, 0x01, 0x40, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x40, 0x00, 0xe0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1b, 0xe3, 0x1e, 0x63, 0x1e, 0x83, 0x14, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1c, 0xe4, 0x1e, 0x63, 0x1d, 0xe4, 0x01, 0xe0, 0x1d, 0x03, 0x1e, 0xa3, 0x1e, 0x83, 0x25, 0xe5, 0x01, 0x60, 0x00, 0x00, 0x01, 0x20, 0x25, 0x83, 0x1e, 0x63, 0x1d, 0xa3, 0x12, 0xa2, 0x26, 0x23, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0x64, 0x2d, 0x26, 0x01, 0xe0, 0x1c, 0xe3, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0x64, 0x1e, 0x63, 0x1e, 0x43, 0x25, 0x44, 0x13, 0x42, 0x00, 0xe0, 0x00, 0x00, 0x01, 0x20, 0x2d, 0xc6, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0x83, 0x1d, 0x23, 0x13, 0xa2, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0x63, 0x13, 0x42, 0x00, 0x60, 0x00, 0x60, 0x1b, 0xe3, 0x1e, 0x63, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0x64, 0x16, 0xc3, 0x1d, 0xe3, 0x01, 0xe0, 0x1d, 0x03, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0x64, 0x01, 0xa0, 0x1c, 0xc3, 0x1e, 0xa3, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0x83, 0x1e, 0x83, 0x2e, 0x25, 0x13, 0x62, 0x1d, 0xc3, 0x1e, 0x83, 0x1e, 0xa3, 0x1e, 0x83, 0x1e, 0x63, 0x1e, 0x24, 0x1d, 0x03, 0x12, 0xc2, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0xa2, 0x17, 0xc2, 0x17, 0xa2, 0x14, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x15, 0xe2, 0x17, 0xa2, 0x17, 0x22, 0x02, 0x40, 0x1d, 0xe3, 0x07, 0xe2, 0x07, 0xc0, 0x17, 0xa2, 0x14, 0xc3, 0x00, 0xa0, 0x01, 0x80, 0x1e, 0x63, 0x17, 0xc2, 0x1e, 0xa3, 0x12, 0x42, 0x1d, 0xa3, 0x17, 0x62, 0x17, 0xc2, 0x07, 0xc0, 0x1f, 0x43, 0x24, 0xe4, 0x01, 0xe0, 0x1d, 0xc3, 0x07, 0xe0, 0x17, 0xc2, 0x16, 0x62, 0x1e, 0x43, 0x1f, 0x23, 0x17, 0xa2, 0x1f, 0x62, 0x1d, 0x03, 0x00, 0xe0, 0x01, 0x40, 0x1f, 0x03, 0x07, 0xe0, 0x17, 0xc0, 0x17, 0x43, 0x1e, 0xe3, 0x1f, 0x03, 0x1e, 0xe3, 0x25, 0x43, 0x14, 0x22, 0x17, 0xa2, 0x07, 0xe2, 0x07, 0xc2, 0x1e, 0x83, 0x01, 0x80, 0x00, 0x60, 0x1c, 0xc3, 0x17, 0x82, 0x07, 0xc2, 0x17, 0x42, 0x1e, 0xe3, 0x1f, 0x03, 0x17, 0x22, 0x17, 0xc2, 0x07, 0xe0, 0x17, 0x62, 0x1e, 0xe3, 0x1e, 0xe3, 0x1e, 0x83, 0x1e, 0x43, 0x16, 0xe2, 0x07, 0xc2, 0x17, 0xa2, 0x17, 0x62, 0x1d, 0x23, 0x01, 0xa0, 0x1d, 0xc3, 0x07, 0xc2, 0x17, 0xc2, 0x1f, 0x22, 0x1e, 0xe3, 0x1e, 0xe3, 0x1e, 0xa2, 0x1e, 0x83, 0x1e, 0x43, 0x16, 0xa2, 0x17, 0xc2, 0x07, 0xe0, 0x17, 0x82, 0x1d, 0x83, 0x01, 0x40, 0x15, 0x82, 0x07, 0xc2, 0x07, 0xc2, 0x17, 0x62, 0x1e, 0xe3, 0x1e, 0xe3, 0x1e, 0xe3, 0x2e, 0x65, 0x13, 0x82, 0x16, 0xe2, 0x17, 0xc2, 0x17, 0x82, 0x15, 0xa2, 0x1e, 0x63, 0x17, 0x63, 0x17, 0xc2, 0x17, 0x42, 0x13, 0xe2, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0xa2, 0x07, 0xc0, 0x17, 0xc2, 0x15, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x16, 0x02, 0x07, 0xc2, 0x17, 0x42, 0x02, 0x40, 0x15, 0xe3, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x24, 0x02, 0xa2, 0x01, 0xa0, 0x1e, 0x83, 0x07, 0xe0, 0x16, 0xa2, 0x00, 0x60, 0x00, 0x60, 0x15, 0x82, 0x07, 0xc0, 0x07, 0xc0, 0x1d, 0x83, 0x01, 0x00, 0x01, 0x00, 0x1d, 0xc3, 0x07, 0xe0, 0x1f, 0x43, 0x02, 0x40, 0x01, 0x00, 0x12, 0xe2, 0x16, 0x63, 0x07, 0xc2, 0x1f, 0x63, 0x14, 0x02, 0x01, 0x60, 0x1f, 0x23, 0x07, 0xc0, 0x1e, 0xc2, 0x13, 0x02, 0x12, 0x02, 0x12, 0x02, 0x12, 0x02, 0x11, 0x82, 0x13, 0xc2, 0x17, 0xc2, 0x07, 0xe0, 0x07, 0xc0, 0x17, 0xa2, 0x15, 0x22, 0x00, 0xe0, 0x1c, 0xc3, 0x17, 0xc2, 0x07, 0xc2, 0x03, 0xc2, 0x12, 0x02, 0x12, 0x22, 0x03, 0x40, 0x1f, 0x63, 0x07, 0xe0, 0x16, 0x02, 0x02, 0x82, 0x02, 0x00, 0x01, 0x80, 0x01, 0x20, 0x1c, 0xa3, 0x17, 0xc2, 0x17, 0xc2, 0x1d, 0xa3, 0x01, 0x00, 0x00, 0xa0, 0x1d, 0xe3, 0x07, 0xe0, 0x17, 0x82, 0x13, 0x82, 0x12, 0x22, 0x01, 0xe2, 0x01, 0xc0, 0x01, 0x40, 0x01, 0x00, 0x14, 0x02, 0x17, 0xa2, 0x07, 0xc0, 0x16, 0x42, 0x01, 0x00, 0x00, 0x60, 0x15, 0x82, 0x17, 0xc0, 0x17, 0x82, 0x14, 0x20, 0x12, 0x42, 0x02, 0x22, 0x12, 0x02, 0x11, 0xe2, 0x02, 0x60, 0x16, 0xe2, 0x07, 0xc2, 0x1e, 0x83, 0x00, 0xa0, 0x01, 0x00, 0x13, 0x62, 0x1e, 0xe3, 0x17, 0xc2, 0x1f, 0x03, 0x12, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0xa2, 0x07, 0xc0, 0x17, 0xa2, 0x14, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0xa0, 0x16, 0x03, 0x17, 0xc2, 0x17, 0x23, 0x02, 0x40, 0x1d, 0xe2, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xc0, 0x1e, 0x03, 0x02, 0xa0, 0x1e, 0xa3, 0x07, 0xe0, 0x1e, 0x83, 0x00, 0x60, 0x00, 0x60, 0x1d, 0x02, 0x17, 0xa2, 0x17, 0xc2, 0x1c, 0xc3, 0x00, 0xa0, 0x00, 0xe0, 0x1d, 0xc3, 0x07, 0xc0, 0x27, 0x25, 0x01, 0x60, 0x00, 0x00, 0x00, 0x60, 0x02, 0xa0, 0x17, 0xa2, 0x07, 0xc0, 0x16, 0xc3, 0x01, 0xe0, 0x17, 0x42, 0x07, 0xc0, 0x1e, 0x43, 0x02, 0x40, 0x01, 0x00, 0x01, 0x00, 0x01, 0x20, 0x00, 0xa0, 0x13, 0xa2, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe2, 0x17, 0x62, 0x13, 0x22, 0x15, 0x03, 0x17, 0xc0, 0x17, 0xa2, 0x12, 0x42, 0x00, 0x00, 0x00, 0x00, 0x01, 0xa0, 0x27, 0x25, 0x07, 0xe0, 0x1d, 0x62, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x24, 0x63, 0x17, 0xc2, 0x17, 0xc0, 0x1d, 0x03, 0x00, 0x60, 0x00, 0xa0, 0x1d, 0xe2, 0x07, 0xe0, 0x17, 0x62, 0x03, 0x00, 0x01, 0x20, 0x01, 0x00, 0x01, 0x00, 0x00, 0xa0, 0x00, 0x60, 0x1b, 0xa3, 0x17, 0xa2, 0x07, 0xc0, 0x1d, 0xa2, 0x00, 0xa0, 0x00, 0x60, 0x15, 0xa2, 0x07, 0xe0, 0x17, 0x42, 0x03, 0x60, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x00, 0xa0, 0x01, 0xe0, 0x17, 0x02, 0x07, 0xc0, 0x16, 0x03, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x14, 0x62, 0x17, 0x83, 0x17, 0xc2, 0x14, 0xe3, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0x62, 0x07, 0xc0, 0x17, 0xc2, 0x14, 0xc2, 0x00, 0x60, 0x10, 0x00, 0x00, 0x60, 0x16, 0x22, 0x17, 0xc2, 0x16, 0xe2, 0x02, 0x20, 0x1d, 0xe2, 0x07, 0xe0, 0x07, 0xa2, 0x16, 0xe2, 0x17, 0xa2, 0x17, 0xa2, 0x05, 0xa2, 0x1e, 0xe3, 0x07, 0xe2, 0x1e, 0x83, 0x00, 0x60, 0x00, 0x60, 0x1c, 0xc2, 0x17, 0xc2, 0x17, 0xa2, 0x1c, 0xc3, 0x00, 0xa0, 0x00, 0xe0, 0x1d, 0xe2, 0x07, 0xe0, 0x27, 0x26, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x17, 0x02, 0x17, 0xc2, 0x1f, 0x83, 0x02, 0xa0, 0x17, 0x43, 0x07, 0xe0, 0x07, 0xe2, 0x17, 0xa2, 0x17, 0x83, 0x1f, 0x83, 0x1f, 0x83, 0x1b, 0xc3, 0x13, 0xa2, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0x00, 0x17, 0x62, 0x17, 0xa2, 0x16, 0xc3, 0x15, 0xc2, 0x17, 0xa0, 0x17, 0xa2, 0x12, 0x62, 0x00, 0x00, 0x00, 0x00, 0x01, 0xa0, 0x27, 0x45, 0x07, 0xe0, 0x1d, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1c, 0x83, 0x17, 0xc2, 0x17, 0xc2, 0x1c, 0xe3, 0x00, 0x60, 0x00, 0xa0, 0x16, 0x03, 0x07, 0xe0, 0x17, 0xc0, 0x17, 0xa2, 0x1f, 0x83, 0x17, 0x63, 0x17, 0x83, 0x1d, 0x43, 0x00, 0x60, 0x13, 0xa3, 0x17, 0xa2, 0x07, 0xe0, 0x15, 0xa2, 0x00, 0xa0, 0x00, 0x60, 0x1d, 0x83, 0x07, 0xc0, 0x07, 0xc0, 0x17, 0xa2, 0x17, 0x83, 0x17, 0x83, 0x17, 0x83, 0x1d, 0x83, 0x02, 0x80, 0x17, 0x02, 0x07, 0xc0, 0x16, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x1b, 0x02, 0x1f, 0x63, 0x17, 0xc0, 0x2e, 0x24, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x02, 0x07, 0xc2, 0x17, 0xc2, 0x15, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x16, 0x82, 0x17, 0xe0, 0x16, 0xc2, 0x02, 0x00, 0x1d, 0xe3, 0x07, 0xe0, 0x17, 0x63, 0x03, 0xa0, 0x26, 0xc3, 0x07, 0xe0, 0x07, 0xc0, 0x17, 0xc2, 0x07, 0xe0, 0x1e, 0x83, 0x00, 0x60, 0x00, 0x60, 0x1c, 0xe3, 0x17, 0xa2, 0x17, 0xc2, 0x1c, 0xc3, 0x00, 0xa0, 0x00, 0xe0, 0x16, 0x02, 0x07, 0xe0, 0x27, 0x05, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x01, 0x60, 0x16, 0xe2, 0x07, 0xc0, 0x17, 0x82, 0x02, 0xc0, 0x17, 0x23, 0x07, 0xe0, 0x17, 0xc2, 0x17, 0xa2, 0x17, 0x82, 0x17, 0x83, 0x1f, 0x63, 0x1b, 0xa3, 0x13, 0x82, 0x17, 0xc2, 0x17, 0xa2, 0x15, 0x22, 0x1c, 0xe3, 0x17, 0xa2, 0x07, 0xc2, 0x07, 0xa2, 0x07, 0xc0, 0x17, 0xa2, 0x12, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0xe0, 0x1f, 0x44, 0x07, 0xe0, 0x1d, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1c, 0x83, 0x17, 0xa2, 0x07, 0xc2, 0x1d, 0x03, 0x00, 0x60, 0x00, 0xa0, 0x15, 0xe3, 0x07, 0xe0, 0x17, 0xc2, 0x17, 0x82, 0x1f, 0x63, 0x1f, 0x83, 0x17, 0x82, 0x1d, 0x63, 0x00, 0x60, 0x13, 0xe3, 0x17, 0xc2, 0x07, 0xe0, 0x15, 0xa3, 0x00, 0xa0, 0x00, 0x60, 0x15, 0x83, 0x07, 0xc0, 0x07, 0xc0, 0x17, 0xa2, 0x1f, 0x63, 0x1f, 0x63, 0x1f, 0x83, 0x1d, 0x83, 0x02, 0x80, 0x17, 0x02, 0x17, 0xc2, 0x15, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x12, 0xe2, 0x1f, 0x63, 0x07, 0xc0, 0x2e, 0x44, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x82, 0x17, 0xc2, 0x17, 0xa2, 0x15, 0x22, 0x00, 0x60, 0x00, 0x60, 0x00, 0xa0, 0x17, 0x03, 0x07, 0xe0, 0x16, 0xa2, 0x02, 0x00, 0x1d, 0xe3, 0x07, 0xc0, 0x1f, 0x23, 0x02, 0x40, 0x13, 0xa2, 0x1f, 0x63, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0xa3, 0x00, 0xa0, 0x00, 0x60, 0x14, 0xe2, 0x17, 0x83, 0x17, 0xc2, 0x1c, 0xa3, 0x00, 0x60, 0x00, 0xe0, 0x15, 0xe3, 0x07, 0xe0, 0x2f, 0x05, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x02, 0x40, 0x17, 0x82, 0x07, 0xe0, 0x1f, 0x43, 0x02, 0x00, 0x1f, 0x22, 0x17, 0xc0, 0x1e, 0x03, 0x01, 0xe0, 0x01, 0x00, 0x01, 0x20, 0x01, 0x00, 0x00, 0xa0, 0x13, 0xa2, 0x07, 0xc0, 0x17, 0xa2, 0x14, 0x62, 0x01, 0xe0, 0x16, 0xc2, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x82, 0x12, 0x62, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x1f, 0x44, 0x07, 0xe0, 0x15, 0x43, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1c, 0x84, 0x17, 0xa3, 0x07, 0xc2, 0x1d, 0x03, 0x00, 0x60, 0x00, 0xa0, 0x1d, 0xc3, 0x07, 0xe0, 0x1f, 0x23, 0x02, 0x60, 0x01, 0x00, 0x01, 0x20, 0x01, 0x20, 0x00, 0xa0, 0x00, 0x60, 0x13, 0xe2, 0x17, 0xa2, 0x17, 0xe2, 0x1d, 0x62, 0x00, 0xa0, 0x00, 0x60, 0x15, 0xc2, 0x07, 0xe0, 0x17, 0x42, 0x03, 0x00, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x00, 0xa0, 0x01, 0xe0, 0x17, 0x02, 0x07, 0xc0, 0x15, 0xc2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x14, 0x42, 0x17, 0xa3, 0x07, 0xe0, 0x1d, 0x83, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xa2, 0x17, 0x42, 0x07, 0xc0, 0x1d, 0xe3, 0x00, 0xe0, 0x00, 0xa0, 0x12, 0x20, 0x1f, 0x82, 0x17, 0xc2, 0x1d, 0xe3, 0x01, 0xc0, 0x15, 0xe3, 0x17, 0xc0, 0x1f, 0x03, 0x01, 0xe0, 0x00, 0xe0, 0x1d, 0xa3, 0x17, 0xa0, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0x63, 0x00, 0x60, 0x00, 0x60, 0x1d, 0x22, 0x17, 0xa2, 0x07, 0xc2, 0x1d, 0x23, 0x00, 0xa0, 0x00, 0xe0, 0x15, 0xe3, 0x07, 0xe0, 0x2f, 0x23, 0x01, 0x40, 0x00, 0x60, 0x01, 0xa0, 0x15, 0x82, 0x17, 0xa2, 0x17, 0xa2, 0x1c, 0xe3, 0x01, 0xa0, 0x17, 0x23, 0x07, 0xc0, 0x1d, 0xe3, 0x01, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x13, 0x82, 0x07, 0xc2, 0x17, 0xa2, 0x14, 0x22, 0x00, 0x60, 0x13, 0x62, 0x17, 0x83, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x83, 0x12, 0x42, 0x00, 0x00, 0x00, 0x00, 0x01, 0xa0, 0x1f, 0x44, 0x07, 0xe0, 0x15, 0x43, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1c, 0xc3, 0x17, 0xc2, 0x07, 0xc2, 0x1d, 0x43, 0x00, 0xa0, 0x00, 0xa0, 0x1d, 0xe3, 0x07, 0xc0, 0x1e, 0xe4, 0x01, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1c, 0x42, 0x17, 0xc2, 0x07, 0xc0, 0x15, 0xe2, 0x00, 0xa0, 0x00, 0x60, 0x15, 0x82, 0x17, 0xc0, 0x17, 0x22, 0x02, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x02, 0x00, 0x16, 0xe3, 0x07, 0xc2, 0x15, 0xe2, 0x00, 0xa0, 0x00, 0xa0, 0x12, 0xa2, 0x16, 0xc3, 0x07, 0xc2, 0x17, 0x62, 0x13, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1d, 0xc3, 0x17, 0xc0, 0x17, 0x82, 0x1d, 0xe3, 0x04, 0xe2, 0x1e, 0xa2, 0x17, 0xa2, 0x17, 0x82, 0x1c, 0x23, 0x01, 0x20, 0x16, 0x02, 0x07, 0xe0, 0x1e, 0xe3, 0x01, 0xa0, 0x00, 0x00, 0x02, 0x40, 0x1e, 0xe3, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0x83, 0x01, 0x40, 0x13, 0x82, 0x16, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0x83, 0x13, 0x02, 0x01, 0x60, 0x1d, 0xc3, 0x07, 0xe0, 0x17, 0x82, 0x04, 0x82, 0x15, 0x22, 0x1e, 0xa3, 0x17, 0x82, 0x17, 0xa2, 0x1e, 0x24, 0x01, 0x80, 0x01, 0x60, 0x17, 0x23, 0x07, 0xc0, 0x17, 0x42, 0x15, 0xe2, 0x1d, 0x83, 0x15, 0xa2, 0x15, 0xa3, 0x1c, 0x43, 0x14, 0x02, 0x07, 0xc0, 0x17, 0xa3, 0x13, 0xe2, 0x00, 0x60, 0x00, 0xa0, 0x15, 0x83, 0x17, 0xc2, 0x07, 0xe0, 0x17, 0xa3, 0x12, 0x62, 0x00, 0x00, 0x00, 0x00, 0x01, 0xa0, 0x27, 0x43, 0x07, 0xe0, 0x15, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0xe0, 0x13, 0x82, 0x1e, 0x63, 0x07, 0xe0, 0x07, 0xc2, 0x16, 0xc2, 0x13, 0x02, 0x01, 0x20, 0x15, 0xe3, 0x07, 0xc0, 0x26, 0xe4, 0x01, 0x60, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xa0, 0x13, 0x02, 0x16, 0x22, 0x07, 0xc0, 0x07, 0xc0, 0x17, 0x23, 0x13, 0x82, 0x00, 0xe0, 0x15, 0x82, 0x07, 0xc0, 0x17, 0xa2, 0x16, 0x22, 0x1d, 0x83, 0x1d, 0x83, 0x15, 0x82, 0x25, 0x23, 0x13, 0x42, 0x17, 0x02, 0x17, 0xc2, 0x06, 0xe0, 0x04, 0xc0, 0x1d, 0xe3, 0x1f, 0x23, 0x17, 0xa2, 0x17, 0x82, 0x1d, 0x03, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x02, 0x60, 0x1e, 0x03, 0x17, 0x62, 0x17, 0xa2, 0x17, 0xa3, 0x17, 0x82, 0x1e, 0xe3, 0x1c, 0xe3, 0x01, 0x40, 0x00, 0xe0, 0x1d, 0xc3, 0x17, 0xc2, 0x26, 0xc5, 0x01, 0x60, 0x00, 0x00, 0x00, 0x60, 0x13, 0xc3, 0x17, 0x03, 0x17, 0xa2, 0x1e, 0x83, 0x1a, 0xe2, 0x27, 0x23, 0x17, 0x82, 0x17, 0xc2, 0x17, 0xc0, 0x1f, 0x83, 0x2e, 0x05, 0x02, 0x40, 0x1d, 0xc3, 0x17, 0xc2, 0x17, 0xc2, 0x17, 0x83, 0x17, 0x83, 0x17, 0x62, 0x17, 0x03, 0x1d, 0xa3, 0x12, 0x02, 0x00, 0x60, 0x01, 0x40, 0x1f, 0x03, 0x17, 0xc2, 0x17, 0xc2, 0x17, 0xa3, 0x17, 0xc2, 0x17, 0xc2, 0x17, 0xa2, 0x1d, 0xc4, 0x14, 0x22, 0x17, 0xa2, 0x1f, 0x63, 0x13, 0x82, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc2, 0x1e, 0x63, 0x17, 0x82, 0x17, 0x83, 0x12, 0x62, 0x00, 0x00, 0x00, 0x00, 0x01, 0xa0, 0x2f, 0x25, 0x17, 0xc2, 0x1d, 0x23, 0x00, 0x60, 0x00, 0x00, 0x12, 0x62, 0x27, 0x05, 0x1f, 0x83, 0x17, 0xc2, 0x17, 0xa3, 0x17, 0xa3, 0x26, 0x64, 0x01, 0xe0, 0x1d, 0xe3, 0x17, 0xc2, 0x26, 0xc4, 0x01, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x02, 0x2e, 0xe5, 0x17, 0x82, 0x17, 0xc2, 0x17, 0xa2, 0x17, 0x83, 0x26, 0xe5, 0x11, 0xc2, 0x1d, 0x83, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0xa2, 0x2f, 0x05, 0x13, 0xc2, 0x17, 0x02, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0x82, 0x17, 0xa2, 0x07, 0x82, 0x1e, 0xc3, 0x1c, 0xe3, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x40, 0x12, 0x82, 0x13, 0x02, 0x13, 0x22, 0x12, 0xa2, 0x02, 0x00, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x12, 0x62, 0x13, 0x42, 0x12, 0xe3, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x12, 0x42, 0x13, 0x02, 0x12, 0xe2, 0x01, 0x00, 0x12, 0xe2, 0x13, 0x22, 0x13, 0x42, 0x13, 0x42, 0x13, 0x22, 0x12, 0x62, 0x01, 0x00, 0x12, 0xc2, 0x13, 0x82, 0x13, 0x42, 0x13, 0x02, 0x12, 0xc2, 0x12, 0x82, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x12, 0xe2, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x12, 0x62, 0x11, 0xe2, 0x13, 0x42, 0x1b, 0x02, 0x01, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x12, 0xc2, 0x13, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1a, 0xe3, 0x13, 0x42, 0x12, 0x22, 0x00, 0x60, 0x00, 0x00, 0x00, 0xe0, 0x12, 0xe2, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x12, 0x62, 0x00, 0xa0, 0x12, 0x62, 0x13, 0x42, 0x12, 0xe3, 0x00, 0xa0, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa2, 0x12, 0xa2, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x13, 0x42, 0x12, 0xa2, 0x00, 0xa0, 0x12, 0x62, 0x13, 0x42, 0x13, 0x22, 0x13, 0x02, 0x13, 0x22, 0x13, 0x42, 0x13, 0x42, 0x1a, 0xe3, 0x01, 0xa0, 0x13, 0x42, 0x13, 0x62, 0x13, 0x22, 0x12, 0xe2, 0x12, 0xc2, 0x12, 0xa2, 0x01, 0xe0, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x18, 0x63, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0x60, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xe0, 0x00, 0xe0, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xe0, 0x00, 0xe0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x20, 0xe3, 0x10, 0xa2, 0x00, 0x02, 0x00, 0x00, 0x00, 0xe0, 0x13, 0x82, 0x1d, 0x43, 0x1d, 0xc3, 0x1d, 0xc3, 0x1d, 0xc3, 0x25, 0xa3, 0x1d, 0xa3, 0x1d, 0x63, 0x1d, 0x83, 0x1d, 0x83, 0x1d, 0x43, 0x1d, 0x43, 0x1d, 0x43, 0x1d, 0x42, 0x1d, 0x22, 0x1d, 0x23, 0x14, 0xe2, 0x14, 0xe2, 0x1c, 0xe2, 0x13, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x13, 0x42, 0x1c, 0xa2, 0x1c, 0x43, 0x01, 0x60, 0x1b, 0x03, 0x14, 0xa2, 0x14, 0xc2, 0x14, 0xa2, 0x14, 0x82, 0x14, 0x22, 0x13, 0x42, 0x01, 0xe0, 0x00, 0x60, 0x00, 0x60, 0x12, 0x42, 0x14, 0x63, 0x14, 0xa3, 0x14, 0xa2, 0x14, 0xc2, 0x14, 0x82, 0x1a, 0xa3, 0x00, 0x60, 0x00, 0x60, 0x12, 0x82, 0x1d, 0x03, 0x26, 0x84, 0x2e, 0xa5, 0x36, 0x85, 0x25, 0xa4, 0x13, 0xe2, 0x01, 0x20, 0x00, 0x02, 0x00, 0x00, 0x01, 0x20, 0x13, 0xe3, 0x1d, 0xa3, 0x26, 0x44, 0x26, 0x65, 0x1e, 0x85, 0x25, 0xe3, 0x1b, 0xc3, 0x00, 0xe0, 0x00, 0x00, 0x10, 0xa2, 0x1b, 0xa3, 0x14, 0xa2, 0x14, 0xa2, 0x1c, 0x43, 0x12, 0x82, 0x1c, 0x43, 0x1c, 0x43, 0x11, 0xe2, 0x01, 0x00, 0x13, 0x62, 0x14, 0xa2, 0x12, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x19, 0x03, 0x10, 0x62, 0x10, 0x02, 0x00, 0x60, 0x1c, 0x83, 0x1f, 0x62, 0x17, 0xa2, 0x1f, 0x63, 0x1e, 0xe3, 0x1f, 0x23, 0x1f, 0x82, 0x07, 0xc0, 0x07, 0xc2, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0xa2, 0x07, 0xc0, 0x07, 0xe0, 0x17, 0xa2, 0x1f, 0x82, 0x17, 0xa2, 0x07, 0xc2, 0x07, 0xc2, 0x17, 0xa2, 0x1d, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x15, 0x62, 0x17, 0xa2, 0x17, 0x42, 0x02, 0x80, 0x1d, 0x63, 0x07, 0xc2, 0x07, 0xc0, 0x17, 0x62, 0x1f, 0x63, 0x17, 0xa2, 0x17, 0xa2, 0x1e, 0xc3, 0x13, 0xc2, 0x00, 0xe0, 0x24, 0x23, 0x27, 0x05, 0x17, 0xa2, 0x17, 0xc2, 0x17, 0xc2, 0x1f, 0x63, 0x2c, 0xc5, 0x00, 0xa0, 0x12, 0xe2, 0x1f, 0x03, 0x17, 0xc0, 0x17, 0xa2, 0x1e, 0xe3, 0x1e, 0xe3, 0x17, 0xa2, 0x17, 0xa2, 0x1d, 0x63, 0x00, 0xa0, 0x00, 0xe0, 0x1d, 0x43, 0x17, 0x82, 0x17, 0xa3, 0x1f, 0x43, 0x1e, 0xe3, 0x1f, 0x43, 0x17, 0x82, 0x27, 0x24, 0x12, 0x42, 0x00, 0x02, 0x00, 0xa0, 0x13, 0xa2, 0x16, 0xe2, 0x17, 0xa2, 0x14, 0x82, 0x03, 0x40, 0x17, 0x23, 0x17, 0x42, 0x25, 0x63, 0x03, 0xe0, 0x1f, 0x63, 0x07, 0xe2, 0x14, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x62, 0x18, 0xe3, 0x10, 0x62, 0x00, 0x02, 0x01, 0xe0, 0x1f, 0x03, 0x17, 0xc0, 0x17, 0x02, 0x13, 0x22, 0x11, 0xe2, 0x12, 0x42, 0x14, 0x62, 0x1e, 0x63, 0x1d, 0xe3, 0x14, 0xa2, 0x04, 0x40, 0x16, 0x82, 0x07, 0xe0, 0x07, 0xc0, 0x05, 0x40, 0x14, 0x22, 0x04, 0x60, 0x06, 0x62, 0x07, 0xe0, 0x07, 0xc0, 0x15, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x15, 0x82, 0x07, 0xc0, 0x17, 0x42, 0x02, 0x80, 0x1d, 0x63, 0x07, 0xe0, 0x17, 0xa2, 0x04, 0x00, 0x13, 0x02, 0x15, 0x22, 0x17, 0x22, 0x17, 0xa2, 0x1f, 0x23, 0x13, 0x42, 0x11, 0x20, 0x12, 0xa2, 0x17, 0x42, 0x07, 0xe0, 0x17, 0xc2, 0x14, 0xe2, 0x01, 0x60, 0x01, 0x80, 0x1e, 0x63, 0x07, 0xe0, 0x17, 0x62, 0x14, 0x82, 0x01, 0xe2, 0x12, 0x22, 0x1d, 0x42, 0x17, 0xa2, 0x17, 0x82, 0x13, 0xa2, 0x1b, 0x03, 0x1f, 0x43, 0x17, 0xc0, 0x16, 0x42, 0x12, 0xc2, 0x02, 0x00, 0x12, 0x82, 0x14, 0xc2, 0x36, 0x25, 0x11, 0xe2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1d, 0xc3, 0x1f, 0x63, 0x01, 0x20, 0x02, 0x40, 0x17, 0x23, 0x15, 0x62, 0x1e, 0xa3, 0x17, 0x02, 0x06, 0xe2, 0x17, 0xa2, 0x14, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x02, 0xa0, 0x1f, 0x43, 0x07, 0xc2, 0x17, 0x22, 0x13, 0xe2, 0x01, 0x80, 0x00, 0xe0, 0x00, 0xa0, 0x11, 0x40, 0x00, 0xe0, 0x00, 0x60, 0x00, 0x60, 0x15, 0x22, 0x07, 0xc0, 0x17, 0x62, 0x02, 0x80, 0x00, 0x00, 0x00, 0x60, 0x14, 0x42, 0x17, 0xc2, 0x07, 0xc2, 0x15, 0x62, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x15, 0x62, 0x17, 0xc2, 0x17, 0x42, 0x02, 0xa0, 0x1d, 0x63, 0x07, 0xc0, 0x1f, 0x63, 0x02, 0x20, 0x00, 0x60, 0x00, 0xa0, 0x13, 0xa2, 0x17, 0x82, 0x17, 0xc0, 0x1e, 0x63, 0x00, 0xa0, 0x00, 0x60, 0x16, 0xe2, 0x07, 0xe0, 0x17, 0xc2, 0x13, 0x42, 0x00, 0x60, 0x13, 0x62, 0x17, 0x83, 0x07, 0xe0, 0x1d, 0xe3, 0x01, 0x00, 0x00, 0x60, 0x00, 0x60, 0x01, 0xc0, 0x16, 0xe2, 0x17, 0xa2, 0x16, 0x63, 0x1c, 0x63, 0x17, 0x82, 0x07, 0xe0, 0x16, 0xa2, 0x13, 0x42, 0x01, 0x80, 0x00, 0xe0, 0x00, 0xa0, 0x11, 0x82, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1d, 0x83, 0x1f, 0x43, 0x00, 0xa0, 0x02, 0x00, 0x1f, 0x03, 0x03, 0x82, 0x1d, 0x43, 0x1e, 0x84, 0x04, 0x00, 0x1f, 0x43, 0x1c, 0xc3, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x63, 0x00, 0x00, 0x00, 0x00, 0x01, 0xa0, 0x1e, 0xc4, 0x17, 0xc2, 0x07, 0xe0, 0x17, 0xa2, 0x1e, 0xc3, 0x1d, 0xa3, 0x14, 0x03, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x15, 0x22, 0x17, 0xc0, 0x17, 0x43, 0x02, 0x60, 0x00, 0x00, 0x00, 0x60, 0x14, 0x02, 0x17, 0xc2, 0x17, 0xc0, 0x15, 0x62, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x15, 0xa2, 0x07, 0xc2, 0x17, 0x23, 0x02, 0x80, 0x1d, 0x63, 0x07, 0xe0, 0x1f, 0x63, 0x01, 0xe0, 0x00, 0x02, 0x00, 0x00, 0x01, 0xc0, 0x1e, 0xc2, 0x17, 0xc0, 0x1f, 0x43, 0x01, 0xe0, 0x00, 0x60, 0x1e, 0xc3, 0x07, 0xc2, 0x17, 0xc2, 0x13, 0x02, 0x00, 0x60, 0x1d, 0x03, 0x07, 0xc0, 0x07, 0xc0, 0x1c, 0xc3, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1d, 0xe3, 0x07, 0xc2, 0x17, 0x82, 0x14, 0x42, 0x17, 0x43, 0x07, 0xc0, 0x17, 0xc2, 0x17, 0x82, 0x1e, 0xa2, 0x1d, 0x83, 0x13, 0x42, 0x01, 0x00, 0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x00, 0xa0, 0x24, 0xe3, 0x2e, 0xc5, 0x00, 0xa0, 0x01, 0xa0, 0x26, 0x65, 0x12, 0x42, 0x01, 0x40, 0x11, 0xe2, 0x01, 0xe0, 0x2e, 0xc5, 0x1c, 0x43, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x13, 0xa3, 0x1e, 0xc3, 0x17, 0x82, 0x17, 0xe2, 0x07, 0xc0, 0x07, 0xc2, 0x17, 0xa2, 0x1e, 0x43, 0x01, 0x80, 0x00, 0x00, 0x00, 0x60, 0x15, 0x22, 0x07, 0xe0, 0x17, 0x42, 0x02, 0x80, 0x00, 0x00, 0x00, 0x60, 0x13, 0xa2, 0x07, 0xc2, 0x07, 0xc0, 0x15, 0x80, 0x00, 0x60, 0x00, 0x00, 0x00, 0x60, 0x1d, 0xc2, 0x17, 0xa2, 0x1e, 0xe2, 0x02, 0x40, 0x1d, 0x63, 0x07, 0xe0, 0x1f, 0x44, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x1e, 0x43, 0x07, 0xe0, 0x1f, 0x62, 0x12, 0xa2, 0x00, 0xa0, 0x1e, 0xa3, 0x07, 0xc0, 0x17, 0xc2, 0x12, 0xe2, 0x00, 0xe0, 0x15, 0xa2, 0x07, 0xe0, 0x07, 0xe0, 0x1c, 0xa2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1d, 0xa3, 0x17, 0xa2, 0x17, 0x82, 0x02, 0xe0, 0x1c, 0x63, 0x1e, 0xe3, 0x1f, 0x83, 0x07, 0xc2, 0x07, 0xc0, 0x17, 0xc2, 0x17, 0xa3, 0x1d, 0x63, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x11, 0xe2, 0x1a, 0xa3, 0x00, 0x00, 0x00, 0x60, 0x1a, 0x83, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1a, 0xa3, 0x11, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x40, 0x1b, 0x63, 0x15, 0x82, 0x17, 0x23, 0x17, 0xc2, 0x07, 0xe0, 0x1f, 0x63, 0x1c, 0x63, 0x00, 0x60, 0x00, 0x60, 0x15, 0x02, 0x07, 0xc2, 0x1f, 0x43, 0x02, 0x80, 0x00, 0x00, 0x00, 0x60, 0x13, 0x42, 0x17, 0xc2, 0x07, 0xe0, 0x15, 0x82, 0x00, 0x62, 0x00, 0x00, 0x00, 0x60, 0x16, 0x22, 0x07, 0xe0, 0x16, 0xe2, 0x02, 0x20, 0x15, 0x82, 0x07, 0xe0, 0x1f, 0x64, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x60, 0x01, 0x60, 0x16, 0x63, 0x07, 0xe0, 0x1f, 0x43, 0x01, 0xe0, 0x00, 0xa0, 0x16, 0xc3, 0x07, 0xe0, 0x17, 0xc0, 0x13, 0x00, 0x00, 0x60, 0x1c, 0xe3, 0x07, 0xc0, 0x07, 0xe0, 0x15, 0x03, 0x00, 0x60, 0x00, 0x00, 0x10, 0x02, 0x01, 0x00, 0x16, 0x42, 0x07, 0xe2, 0x1f, 0x63, 0x01, 0x60, 0x00, 0xa0, 0x01, 0x80, 0x1b, 0xa2, 0x15, 0xc2, 0x1f, 0x42, 0x17, 0xc2, 0x07, 0xe0, 0x1f, 0x63, 0x13, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x62, 0x02, 0x00, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x01, 0x00, 0x14, 0xc2, 0x17, 0xc0, 0x17, 0x83, 0x1d, 0x64, 0x00, 0xa0, 0x00, 0x00, 0x1d, 0x02, 0x17, 0xc2, 0x17, 0x43, 0x02, 0x60, 0x00, 0x00, 0x00, 0x00, 0x12, 0xc2, 0x17, 0xa2, 0x07, 0xe0, 0x16, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0xe0, 0x1e, 0xe2, 0x07, 0xc0, 0x16, 0x82, 0x01, 0xe0, 0x1d, 0x63, 0x07, 0xe0, 0x1f, 0x63, 0x01, 0xe0, 0x00, 0x00, 0x00, 0x60, 0x02, 0xa0, 0x17, 0x42, 0x07, 0xe0, 0x1e, 0x63, 0x00, 0xa0, 0x00, 0x60, 0x1e, 0xe3, 0x07, 0xe0, 0x17, 0xc2, 0x13, 0x22, 0x00, 0x60, 0x03, 0x02, 0x17, 0x83, 0x07, 0xe0, 0x1e, 0xa3, 0x01, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x02, 0x40, 0x17, 0x62, 0x07, 0xc0, 0x1d, 0xe2, 0x12, 0x62, 0x01, 0x62, 0x00, 0x60, 0x00, 0x60, 0x00, 0xa0, 0x01, 0x60, 0x1e, 0x02, 0x07, 0xe0, 0x17, 0xc2, 0x1c, 0x83, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0xc3, 0x17, 0x43, 0x16, 0x22, 0x1c, 0x02, 0x12, 0xa2, 0x13, 0x62, 0x1e, 0x63, 0x07, 0xe0, 0x17, 0x83, 0x13, 0x63, 0x00, 0x60, 0x00, 0x00, 0x15, 0x02, 0x07, 0xc0, 0x17, 0x82, 0x02, 0x80, 0x00, 0x02, 0x00, 0x00, 0x01, 0xa0, 0x1e, 0xa3, 0x07, 0xc0, 0x17, 0x22, 0x13, 0xe2, 0x02, 0x80, 0x1c, 0xc3, 0x17, 0x82, 0x17, 0xa2, 0x1d, 0xa3, 0x01, 0x20, 0x1d, 0x63, 0x07, 0xe0, 0x1f, 0x63, 0x02, 0xe0, 0x12, 0x82, 0x1c, 0x62, 0x16, 0xe3, 0x17, 0xc2, 0x1f, 0x43, 0x13, 0x62, 0x00, 0xa0, 0x02, 0x00, 0x17, 0x43, 0x07, 0xe0, 0x17, 0xa3, 0x14, 0x22, 0x00, 0xa0, 0x01, 0x00, 0x1d, 0xe4, 0x07, 0xe0, 0x17, 0xc0, 0x16, 0x22, 0x13, 0xa3, 0x13, 0xe2, 0x16, 0x83, 0x07, 0xe2, 0x17, 0x83, 0x12, 0xe2, 0x15, 0x23, 0x26, 0xe4, 0x1d, 0xa2, 0x13, 0xc3, 0x12, 0xa2, 0x13, 0xc2, 0x17, 0x22, 0x07, 0xe0, 0x1f, 0x03, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00, 0x10, 0xa3, 0x18, 0xe5, 0x19, 0x03, 0x20, 0xe4, 0x18, 0xe5, 0x18, 0xe5, 0x21, 0x04, 0x21, 0x44, 0x29, 0x25, 0x19, 0x04, 0x00, 0x60, 0x01, 0x20, 0x12, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x43, 0x1f, 0x63, 0x17, 0xa2, 0x1f, 0x62, 0x1f, 0x43, 0x17, 0x43, 0x17, 0x82, 0x17, 0x82, 0x15, 0x02, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x1c, 0xe3, 0x17, 0xc2, 0x1f, 0x42, 0x02, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x13, 0x82, 0x1f, 0x03, 0x17, 0x82, 0x17, 0x83, 0x1f, 0x23, 0x17, 0x83, 0x17, 0xa3, 0x1e, 0xc3, 0x12, 0xa2, 0x00, 0xa0, 0x1d, 0x63, 0x07, 0xc0, 0x17, 0xc2, 0x17, 0x03, 0x1e, 0xe3, 0x1f, 0x63, 0x17, 0xa2, 0x1f, 0x23, 0x14, 0x02, 0x00, 0xe0, 0x1c, 0x03, 0x1e, 0xe3, 0x07, 0xa3, 0x07, 0xe0, 0x17, 0xc0, 0x1f, 0x23, 0x2c, 0x85, 0x00, 0xa0, 0x02, 0x40, 0x1e, 0xa3, 0x17, 0xa2, 0x17, 0xa2, 0x1f, 0x63, 0x17, 0x83, 0x17, 0xa2, 0x17, 0x43, 0x1c, 0x42, 0x00, 0xa0, 0x14, 0xc2, 0x17, 0x82, 0x17, 0xa2, 0x1f, 0x63, 0x1f, 0x24, 0x1f, 0x43, 0x17, 0xa2, 0x1f, 0x63, 0x13, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x10, 0x02, 0x21, 0x04, 0x29, 0x25, 0x29, 0x44, 0x29, 0x25, 0x31, 0x26, 0x29, 0x26, 0x29, 0x44, 0x29, 0x45, 0x29, 0x26, 0x29, 0x25, 0x00, 0x60, 0x12, 0xe2, 0x25, 0xc5, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x13, 0xa2, 0x1c, 0xa3, 0x1c, 0xe3, 0x1d, 0x24, 0x1d, 0x03, 0x1c, 0xa3, 0x13, 0x22, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0xa3, 0x25, 0xa4, 0x2d, 0x45, 0x11, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x12, 0xa2, 0x1c, 0xc4, 0x1d, 0x84, 0x1d, 0x84, 0x1d, 0x04, 0x14, 0x03, 0x02, 0x00, 0x00, 0x60, 0x00, 0x60, 0x1c, 0x03, 0x1d, 0xc3, 0x1d, 0xc4, 0x25, 0x64, 0x25, 0x24, 0x1c, 0xe4, 0x14, 0x43, 0x12, 0x82, 0x00, 0xa0, 0x00, 0x60, 0x1a, 0xe3, 0x25, 0x65, 0x1d, 0xa4, 0x1d, 0xc3, 0x1d, 0xa3, 0x25, 0x84, 0x23, 0x64, 0x00, 0x60, 0x00, 0x60, 0x01, 0xa0, 0x1b, 0xe3, 0x25, 0x24, 0x25, 0x84, 0x1d, 0x83, 0x1c, 0xa3, 0x12, 0xa2, 0x00, 0xa0, 0x00, 0x60, 0x01, 0x60, 0x1b, 0xe2, 0x1c, 0xc3, 0x1d, 0x03, 0x1d, 0x24, 0x24, 0xe3, 0x1c, 0x63, 0x12, 0xa2, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x44, 0x29, 0x25, 0x29, 0x46, 0x29, 0x44, 0x29, 0x44, 0x29, 0x45, 0x29, 0x05, 0x00, 0x60, 0x1c, 0xc2, 0x1f, 0x63, 0x13, 0x82, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00, 0xe0, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0xa0, 0x00, 0xe0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x00, 0xe0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xe0, 0x00, 0xe0, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x10, 0x62, 0x21, 0x24, 0x29, 0x26, 0x29, 0x45, 0x29, 0x46, 0x29, 0x45, 0x29, 0x44, 0x21, 0x46, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x18, 0xe3, 0x00, 0xe0, 0x1e, 0x63, 0x17, 0xc0, 0x1d, 0xc3, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x29, 0x44, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x46, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x10, 0xa2, 0x02, 0x00, 0x1f, 0x63, 0x07, 0xc0, 0x1f, 0x43, 0x12, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xa3, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xa3, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x02, 0x13, 0x62, 0x03, 0x60, 0x03, 0x00, 0x02, 0xa0, 0x01, 0x80, 0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x62, 0x10, 0x63, 0x10, 0x63, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x29, 0x25, 0x29, 0x45, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xa3, 0x18, 0xa3, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0xe3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x10, 0x62, 0x02, 0xa0, 0x1f, 0x63, 0x07, 0xe0, 0x17, 0xa2, 0x1c, 0xc3, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x10, 0x62, 0x00, 0x60, 0x01, 0x40, 0x1d, 0x43, 0x1f, 0x43, 0x1f, 0x83, 0x1f, 0x83, 0x1e, 0xa4, 0x02, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x02, 0x40, 0x1f, 0x63, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0x43, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x10, 0x62, 0x00, 0x00, 0x01, 0x00, 0x1d, 0x03, 0x1f, 0x83, 0x07, 0xe0, 0x07, 0xc2, 0x1f, 0x23, 0x13, 0xe2, 0x00, 0xe0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa2, 0x29, 0x24, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x00, 0xe0, 0x1e, 0xc3, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x63, 0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x21, 0x04, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0x62, 0x00, 0x00, 0x00, 0xe0, 0x14, 0xa2, 0x1f, 0x82, 0x17, 0xc0, 0x07, 0xc0, 0x17, 0x63, 0x1c, 0xa3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x60, 0x21, 0x03, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x00, 0x60, 0x15, 0x23, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xa2, 0x15, 0x22, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x19, 0x04, 0x18, 0xe4, 0x18, 0x63, 0x00, 0x00, 0x00, 0xa0, 0x13, 0xe2, 0x1f, 0x23, 0x07, 0xe0, 0x07, 0xc0, 0x17, 0x83, 0x1d, 0xa3, 0x01, 0xe0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x18, 0xe3, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x13, 0x42, 0x17, 0x62, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0x63, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x20, 0xe5, 0x18, 0xe5, 0x10, 0xa2, 0x00, 0x00, 0x00, 0xa0, 0x13, 0x82, 0x17, 0x03, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xc0, 0x1e, 0x83, 0x02, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x21, 0x04, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x01, 0x80, 0x1e, 0xe3, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x43, 0x12, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x19, 0x03, 0x19, 0x03, 0x19, 0x03, 0x19, 0x03, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x60, 0x12, 0xe2, 0x16, 0x83, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xc0, 0x1f, 0x23, 0x13, 0x62, 0x00, 0xe0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x10, 0x62, 0x00, 0x00, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x1d, 0xa3, 0x07, 0xc0, 0x07, 0xe0, 0x17, 0xa2, 0x14, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x20, 0xe4, 0x19, 0x03, 0x21, 0x04, 0x20, 0xe4, 0x18, 0xe4, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x60, 0x02, 0x40, 0x1e, 0x23, 0x17, 0x82, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x63, 0x1c, 0xc3, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x29, 0x24, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x21, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x14, 0x02, 0x17, 0xc2, 0x07, 0xe0, 0x17, 0xc2, 0x15, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xa3, 0x10, 0x62, 0x00, 0x00, 0x01, 0x80, 0x1d, 0x83, 0x17, 0xa2, 0x07, 0xc0, 0x07, 0xe0, 0x17, 0xa2, 0x1d, 0xe3, 0x12, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x29, 0x25, 0x29, 0x25, 0x31, 0x25, 0x31, 0x26, 0x29, 0x26, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x21, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x80, 0x17, 0x43, 0x07, 0xe2, 0x07, 0xe0, 0x16, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x10, 0x62, 0x00, 0x00, 0x00, 0xe0, 0x14, 0x42, 0x1f, 0x83, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xa2, 0x1f, 0x03, 0x13, 0xa2, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x19, 0x23, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x44, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x04, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x16, 0xe3, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0xa3, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x10, 0xa2, 0x00, 0x00, 0x00, 0x60, 0x13, 0x02, 0x1f, 0x03, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xc0, 0x17, 0xa2, 0x1d, 0xa3, 0x01, 0xe2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x18, 0xa3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x46, 0x29, 0x25, 0x31, 0x05, 0x31, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x29, 0x45, 0x29, 0x25, 0x29, 0x45, 0x29, 0x25, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40, 0x1e, 0xa3, 0x07, 0xe0, 0x07, 0xe0, 0x16, 0xc3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x19, 0x03, 0x19, 0x03, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x60, 0x12, 0x22, 0x1e, 0x03, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xa2, 0x1e, 0xe3, 0x13, 0xc2, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x18, 0xa3, 0x21, 0x05, 0x29, 0x46, 0x29, 0x45, 0x29, 0x24, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x21, 0x04, 0x21, 0x04, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1e, 0x83, 0x07, 0xe0, 0x07, 0xe0, 0x16, 0xc3, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x21, 0x04, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x20, 0xe3, 0x20, 0xe4, 0x20, 0xe4, 0x18, 0xa3, 0x10, 0x60, 0x00, 0x60, 0x01, 0x20, 0x1c, 0xc3, 0x1f, 0x43, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xc2, 0x17, 0x63, 0x1d, 0xa2, 0x12, 0x42, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x18, 0xa3, 0x21, 0x04, 0x21, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x18, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1e, 0x83, 0x07, 0xe0, 0x07, 0xe0, 0x16, 0xe3, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x19, 0x03, 0x19, 0x03, 0x20, 0xe4, 0x28, 0xe4, 0x18, 0xe3, 0x10, 0x62, 0x00, 0x00, 0x00, 0xa0, 0x13, 0x62, 0x16, 0xa3, 0x17, 0xa2, 0x07, 0xc0, 0x07, 0xe0, 0x17, 0xa2, 0x1e, 0xe2, 0x14, 0x62, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x29, 0x25, 0x29, 0x45, 0x29, 0x45, 0x29, 0x25, 0x18, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x1e, 0xa3, 0x07, 0xe0, 0x07, 0xe0, 0x16, 0xc3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x19, 0x03, 0x19, 0x04, 0x20, 0xe4, 0x20, 0xe4, 0x19, 0x03, 0x18, 0xe3, 0x18, 0x63, 0x00, 0x60, 0x00, 0x60, 0x02, 0x00, 0x1d, 0x63, 0x17, 0x62, 0x07, 0xc0, 0x17, 0xc0, 0x07, 0xe2, 0x17, 0xa2, 0x1e, 0x02, 0x13, 0x02, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x18, 0xa3, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xa3, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x1e, 0xe3, 0x07, 0xe0, 0x07, 0xe0, 0x1e, 0xa3, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe4, 0x18, 0xe4, 0x18, 0xe4, 0x20, 0xe4, 0x20, 0xe3, 0x20, 0xe4, 0x18, 0xa3, 0x10, 0x62, 0x00, 0x00, 0x00, 0xe0, 0x13, 0xc2, 0x1e, 0xc3, 0x17, 0xc0, 0x07, 0xc0, 0x07, 0xc2, 0x17, 0xc2, 0x1f, 0x42, 0x1d, 0x63, 0x02, 0x80, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x40, 0x17, 0x22, 0x07, 0xe2, 0x07, 0xe0, 0x1e, 0x83, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x21, 0x04, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x19, 0x03, 0x18, 0xe3, 0x18, 0xe3, 0x20, 0xe4, 0x20, 0xe4, 0x18, 0xe3, 0x20, 0xe4, 0x18, 0xe3, 0x18, 0x62, 0x00, 0x02, 0x00, 0x60, 0x02, 0x00, 0x1d, 0x82, 0x17, 0x62, 0x07, 0xe2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc2, 0x1f, 0x23, 0x14, 0xc2, 0x01, 0xe0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x42, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xc2, 0x16, 0x03, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x19, 0x03, 0x18, 0xa3, 0x10, 0x02, 0x00, 0x60, 0x00, 0xa0, 0x13, 0x42, 0x1e, 0x83, 0x17, 0xa2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xa3, 0x1e, 0xe3, 0x1c, 0xa3, 0x01, 0xe0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1d, 0xe3, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x82, 0x14, 0xc2, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x20, 0xe3, 0x20, 0xe3, 0x18, 0x63, 0x00, 0x02, 0x00, 0x60, 0x01, 0x40, 0x1c, 0xa3, 0x1f, 0x23, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc2, 0x1f, 0x03, 0x1c, 0xc3, 0x02, 0x40, 0x00, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x13, 0xc2, 0x17, 0x82, 0x07, 0xc0, 0x07, 0xe0, 0x1f, 0x43, 0x12, 0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x20, 0xe4, 0x20, 0xe3, 0x20, 0xe3, 0x28, 0xe4, 0x18, 0xa3, 0x10, 0x62, 0x00, 0x00, 0x00, 0xa0, 0x02, 0x40, 0x15, 0x42, 0x1f, 0x23, 0x17, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xc2, 0x1f, 0x43, 0x1d, 0xc3, 0x13, 0xa2, 0x01, 0xa0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x12, 0xc2, 0x1f, 0x03, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xc0, 0x15, 0xe2, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x19, 0x03, 0x19, 0x03, 0x19, 0x03, 0x19, 0x03, 0x20, 0xe3, 0x20, 0xe4, 0x18, 0xa3, 0x00, 0x60, 0x00, 0x60, 0x00, 0xa0, 0x12, 0xc0, 0x1d, 0xc3, 0x1f, 0x63, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0xa2, 0x1e, 0xc3, 0x1d, 0x43, 0x13, 0x62, 0x01, 0xe0, 0x00, 0xe0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x80, 0x14, 0x23, 0x1f, 0x03, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x1f, 0x02, 0x12, 0xa2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x21, 0x04, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe4, 0x19, 0x03, 0x19, 0x03, 0x19, 0x03, 0x18, 0xe3, 0x20, 0xe3, 0x19, 0x04, 0x18, 0xa3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x13, 0x42, 0x1d, 0xe3, 0x1f, 0x63, 0x17, 0xc2, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xe0, 0x07, 0xc0, 0x17, 0x82, 0x16, 0xe3, 0x1d, 0xe4, 0x1c, 0x83, 0x13, 0x42, 0x12, 0x62, 0x12, 0x20, 0x02, 0x00, 0x01, 0xe2, 0x11, 0xe2, 0x12, 0xa2, 0x14, 0x62, 0x1e, 0x83, 0x17, 0x82, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x17, 0x83, 0x1c, 0x22, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x18, 0xe3, 0x20, 0xe4, 0x20, 0xe5, 0x20, 0xe4, 0x19, 0x03, 0x18, 0xe3, 0x18, 0xa3, 0x10, 0xa2, 0x10, 0x62, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x01, 0x00, 0x13, 0x42, 0x1d, 0xc2, 0x1f, 0x03, 0x17, 0x82, 0x17, 0xc0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe2, 0x07, 0xc2, 0x07, 0xc2, 0x17, 0xa2, 0x17, 0xc2, 0x17, 0x82, 0x17, 0x42, 0x17, 0x02, 0x17, 0x22, 0x17, 0x42, 0x17, 0xa2, 0x17, 0xc2, 0x07, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc2, 0x17, 0x63, 0x1c, 0xa3, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x18, 0xa3, 0x18, 0xa3, 0x18, 0xa3, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x62, 0x10, 0x02, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x12, 0x42, 0x14, 0x82, 0x1e, 0x63, 0x1f, 0x43, 0x17, 0xc2, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xc0, 0x07, 0xe0, 0x17, 0xa2, 0x1e, 0xc4, 0x13, 0xe3, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x01, 0x00, 0x12, 0xc2, 0x14, 0xc2, 0x1e, 0xa3, 0x1f, 0x42, 0x17, 0x82, 0x17, 0xa2, 0x17, 0xc2, 0x07, 0xc2, 0x07, 0xc2, 0x07, 0xc0, 0x07, 0xc0, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0xa2, 0x17, 0x42, 0x1d, 0xa3, 0x12, 0x02, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0xa0, 0x01, 0xe0, 0x1b, 0x43, 0x1c, 0x63, 0x15, 0x43, 0x15, 0xa2, 0x15, 0xc2, 0x15, 0xc2, 0x15, 0xa2, 0x15, 0x62, 0x1d, 0x03, 0x1c, 0x63, 0x1b, 0xa2, 0x01, 0xe0, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};

const uint8_t sat16x16_red_signal[] PROGMEM =
{
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00
};

const uint8_t sat16x16_green_signal[] PROGMEM =
{
  0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 
  0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 
  0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 
  0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0, 0x07, 0xe0
};

const uint8_t sat16x16_blue_signal[] PROGMEM =
{
  0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f
};

const uint8_t rtcsync_blue[] = {
  0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 
  0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f, 0x00, 0x1f
};

const uint8_t rtcsync_red[] = {
  0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 
  0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00
};

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                              DISPLAY VARIABLES

/* try to ensure space for developments by leaving a space of 20 pages between each page group */

bool update_ui = true;
bool ui_cleared = false;
int menu_page = 0;
/* HOME */
static int page_home                             = 0;
/* INPUT DATA */
static int page_input_data                       = 20;
/* MAIN MENU */
static int page_main_menu                        = 40;
/* MATRIX LOGIC */
static int page_matrix_logic_main                = 60;
static int page_matrix_logic_select_setup        = 61;
static int page_matrix_logic_setup_function      = 62;
/* OVERVIEW MATRIX SWITCHING */
static int page_overview_matrix_switching        = 63;
/* FILE */
static int page_file_main                        = 80;
static int page_file_save_matrix                 = 81;
static int page_file_load_matrix                 = 83;
static int page_file_delete_matrix               = 84;
static int page_save_system_config_indicator     = 85;
static int page_save_matrix_file_indicator       = 86;
static int page_load_matrix_file_indicator       = 87;
static int page_delete_matrix_file_indicator     = 88;
static int page_restore_default_matrix_indicator = 89;
// bool bool_save_system_config_indicator     = false; // flag indicating indicator has been displayed
// bool bool_save_matrix_file_indicator       = false;
// bool bool_load_matrix_file_indicator       = false;
// bool bool_delete_matrix_file_indicator     = false;
// bool bool_restore_default_matrix_indicator = false;
/* GPS */
static int page_gps_main                         = 100;
static int page_gps_view_gngga                   = 101;
static int page_gps_view_gnrmc                   = 102;
static int page_gps_view_gpatt                   = 103;
static int page_gps_view_satio                   = 104;
/* SERIAL */
static int page_serial_main                      = 120;
/* SYSTEM */
static int page_system_main                      = 140;
/* UNIVERSE */
static int page_universe_main                    = 160;
static int page_universe_view_sun                = 161;
static int page_universe_view_moon               = 162;
static int page_universe_view_mercury            = 163;
static int page_universe_view_venus              = 164;
static int page_universe_view_mars               = 165;
static int page_universe_view_jupiter            = 166;
static int page_universe_view_saturn             = 167;
static int page_universe_view_uranus             = 168;
static int page_universe_view_neptune            = 169;
/* DISPLAY */
static int page_display_main                     = 180;
/* CD74HC4067 */
static int page_CD74HC4067_main                  = 200;

/* compact ui content vertical spacing */
static int ui_content_0 = 16;
static int ui_content_1 = 26;
static int ui_content_2 = 36;
static int ui_content_3 = 46;
static int ui_content_4 = 56;
static int ui_content_5 = 66;
static int ui_content_6 = 76;
static int ui_content_7 = 86;
static int ui_content_8 = 96;
static int ui_content_9 = 106;
static int ui_content_10 = 116;

bool updateui_content = true; // performance and efficiency: make true when content should be updated. can be true for any reason.

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             DISPLAY MENU SETUP

/*
1x1 menu blended into border in top left corner. this allows this menu to take up no space on the home screen.
main menu is activated by pressing enter when on homescreen.
this may be replaced later by a simple function that changes the page however until then this is preferrable in case more items
are added to this menu.
*/

const int max_home_items = 1;
const char *menuHomeItems[max_home_items] =
{
  "",
};
LcdGfxMenu menuHome( menuHomeItems, max_home_items, {{1, 1}, {1, 1}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             DISPLAY MENU SETUP

const int max_main_menu_items = 9;
const char *menuMainItems[max_main_menu_items] =
{
    "    MATRIX       ", // allows matrix configuration
    "    VIEW MATRIX  ", // overview matrix switching
    "    FILE         ", // load/save/delete system and matrix configurations
    "    GPS          ", // enable/disable parsing of sentences from the gps module
    "    SERIAL       ", // enable/disable output of various comma delimited sentences
    "    SYSTEM       ",
    "    UNIVERSE     ", // enable/disable solar tracking, planet tracking and or other celestial calculations
    "    DISPLAY      ",
    "    CD74HC4067   ",
};
//  "                  "
LcdGfxMenu menuMain( menuMainItems, max_main_menu_items, {{2, 34}, {125, 121}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                              DISPLAY MENU MATRIX SWITCH SELECT

const int max_matrix_switch_items = 20;
const char *menuMatrixSwitchSelectItems[max_matrix_switch_items] =
{
    "M0  ",
    "M1  ",
    "M2  ",
    "M3  ",
    "M4  ",
    "M5  ",
    "M6  ",
    "M7  ",
    "M8  ",
    "M9  ",
    "M10 ",
    "M11 ",
    "M12 ",
    "M13 ",
    "M14 ",
    "M15 ",
    "M16 ",
    "M17 ",
    "M18 ",
    "M19 ",
};
LcdGfxMenu menuMatrixSwitchSelect( menuMatrixSwitchSelectItems, max_matrix_switch_items, {{0, 12}, {39, 35}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                     DISPLAY MENU MATRIX SWITCH FUNCTION SELECT

const int max_function_menu_items = 10;
const char *menuMatrixFunctionSelectItems[max_function_menu_items] =
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
};
LcdGfxMenu menuMatrixFunctionSelect( menuMatrixFunctionSelectItems, max_function_menu_items, {{95, 12}, {128, 35}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                               MENU MATRIX SWITCH CONFIGURATION

const int max_matrix_function_configure_items = 5;
const char *menuMatrixConfigureFunctionItems[max_matrix_function_configure_items] =
{
    "SELECT FUNCTION",
    "ENTER  VALUE X",
    "ENTER  VALUE Y",
    "ENTER  VALUE Z",
    "CHANGE EXPRESSION",
};
LcdGfxMenu menuMatrixConfigureFunction( menuMatrixConfigureFunctionItems, max_matrix_function_configure_items, {{2, 70}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MENU FILE

const int max_file_items = 6;
const char *menuFileItems[max_file_items] =
{
    "NEW     MATRIX",
    "SAVE    MATRIX",
    "LOAD    MATRIX",
    "DELETE  MATRIX",
    "SAVE    SYSTEM",
    "RESTORE DEFAULTS",
};
LcdGfxMenu menuFile( menuFileItems, max_file_items, {{2, 62}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                MENU FILE NAMES

const int max_filepath_items = 20;
const char *menuMatrixFilepathItems[max_filepath_items];
LcdGfxMenu menuMatrixFilepath( menuMatrixFilepathItems, max_filepath_items, {{2, 20}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       MENU GPS

const int max_gps_items = 9;
const char *menuGPSItems[max_gps_items];
LcdGfxMenu menuGPS( menuGPSItems, max_gps_items, {{2, 38}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MENU SERIAL

const int max_serial_items = 16;
const char *menuSerialItems[max_serial_items];
LcdGfxMenu menuSerial( menuSerialItems, max_serial_items, {{2, 14}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  MENU UNIVERSE

const int max_universe_items = 18;
const char *menuUniverseItems[max_universe_items];
LcdGfxMenu menuUniverse( menuUniverseItems, max_universe_items, {{2, 14}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   MENU DISPLAY 

const int max_display_items = 8;
const char *menuDisplayItems[max_display_items];
LcdGfxMenu menuDisplay( menuDisplayItems, max_display_items, {{2, 38}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MENU SYSTEM

const int max_system_items = 1;
const char *menuSystemItems[max_system_items];
LcdGfxMenu menuSystem( menuSystemItems, max_system_items, {{2, 64}, {125, 125}} );

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
TaskHandle_t GPSTask;
TaskHandle_t Task1;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                            RTC

RTC_DS3231 rtc;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SIDEREAL PLANETS

SiderealPlanets myAstro;    // for calculating azimuth and altitude
SiderealObjects myAstroObj; // for getting right ascension and declination of objects from star table

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                         SDCARD

// sd VSPI pins on esp32
int SD_SCLK = 18;  // default esp32 VSPI
int SD_MISO = 19;  // default esp32 VSPI
int SD_MOSI = 23;  // default esp32 VSPI
int SD_CS   = 5;   // default esp32 VSPI

#define SD_FAT_TYPE 2
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
const uint8_t SD_CS_PIN = 5;
// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(4)

#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)

SdExFat sd;
ExFile exfile;

bool sdcard_initialized = false;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SPI SWITCHING

void beginSDCARD() {
  beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
 }

 void endSDCARD() {
  sd.end();
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

bool make_i2c_request = false;
int unixtime_control_panel_request;
int previous_menu_page;
char input_data[128];
char tmp_input_data[128];
char allow_input_data = false;
signed int enter_digits_key = -1;
int menu_column_selection=0;
int previous_menu_column_selection;

char cwd[1024] = "/";

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: SYSTEM

struct systemStruct {
  bool debug = false;                // print verbose information over serial
  bool t_bench = false;              // prints bennchmark information for tuning
  bool overload = false;             // false providing main loop time under specified amount of time. useful if we need to know data is accurate to within overload threshhold time.
  bool matrix_run_on_startup = true; // enables/disable matrix switch on startup as specified by system configuration file

  // performace: turn on/off what you need
  bool satio_enabled = true;          // enables/disables data extrapulation from existing GPS data (coordinate degrees, etc)
  bool gngga_enabled = true;          // enables/disables parsing of serial GPS data
  bool gnrmc_enabled = true;          // enables/disables parsing of serial GPS data
  bool gpatt_enabled = true;          // enables/disables parsing of serial GPS data
  bool matrix_enabled = true;         // enables/disables matrix switch

  bool output_satio_enabled = false;   // enables/disables output SatIO sentence over serial
  bool output_gngga_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_gnrmc_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_gpatt_enabled = false;   // enables/disables output GPS sentence over serial
  bool output_matrix_enabled = false;  // enables/disables output matrix switch active/inactive states sentence over serial
  bool output_sensors_enabled = false; // enables/disables output of sensory data sentence over serial

  bool output_sun_enabled = false;     // enables/disables output sentence over serial
  bool output_moon_enabled = false;    // enables/disables output sentence over serial
  bool output_mercury_enabled = false; // enables/disables output sentence over serial
  bool output_venus_enabled = false;   // enables/disables output sentence over serial
  bool output_mars_enabled = false;    // enables/disables output sentence over serial
  bool output_jupiter_enabled = false; // enables/disables output sentence over serial
  bool output_saturn_enabled = false;  // enables/disables output sentence over serial
  bool output_uranus_enabled = false;  // enables/disables output sentence over serial
  bool output_neptune_enabled = false; // enables/disables output sentence over serial

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
  
  bool allow_debug_bridge = false; // allows serial programming and other features (recommended false every startup)

  // oled protection
  bool display_auto_off = true; // recommended
  int index_display_autooff_times = 5; // index of currently used time 
  int max_display_autooff_times = 6; // max available times
  int display_autooff_times[6] = {3, 5, 10, 15, 30, 60}; // available times
  char char_display_autooff_times[6][56] = {
    "AUTO-OFF  3",
    "AUTO-OFF  5",
    "AUTO-OFF  10",
    "AUTO-OFF  15",
    "AUTO-OFF  30",
    "AUTO-OFF  60",
  };
  int display_timeout = display_autooff_times[index_display_autooff_times];
  
  // personalization: color
  int index_display_border_color = 3;
  int index_display_content_color = 4;
  int index_display_menu_content_color = 2;
  int index_display_menu_border_color = 2;
  int index_display_title_color = 2;
  int index_display_color_subtitle = 2;
  int max_color_index = 6;
  // ensure rgb16 values can be equally divided by 8 unless 255 or 0
  int display_color[7] = {
    RGB_COLOR16(255,0,0), // red
    RGB_COLOR16(255,255,0), // yellow
    RGB_COLOR16(0,255,0), // green
    RGB_COLOR16(0,0,255), // blue
    RGB_COLOR16(0,255,255), // light blue
    RGB_COLOR16(255,0,255), // purple
    RGB_COLOR16(255,255,255), // white
  };
  char char_display_border_color[7][56] = {
    "BORDER    RED",
    "BORDER    YELLOW",
    "BORDER    GREEN",
    "BORDER    BLUE",
    "BORDER    L.BLUE",
    "BORDER    PURPLE",
    "BORDER    WHITE",
  };
  char char_display_content_color[7][56] = {
    "CONTENT   RED",
    "CONTENT   YELLOW",
    "CONTENT   GREEN",
    "CONTENT   BLUE",
    "CONTENT   L.BLUE",
    "CONTENT   PURPLE",
    "CONTENT   WHITE",
  };
  char char_display_menu_border_color[7][56] = {
    "MENUB     RED",
    "MENUB     YELLOW",
    "MENUB     GREEN",
    "MENUB     BLUE",
    "MENUB     L.BLUE",
    "MENUB     PURPLE",
    "MENUB     WHITE",
  };
  char char_display_menu_content_color[7][56] = {
    "MENUC     RED",
    "MENUC     YELLOW",
    "MENUC     GREEN",
    "MENUC     BLUE",
    "MENUC     L.BLUE",
    "MENUC     PURPLE",
    "MENUC     WHITE",
  };
  char char_display_title_color[7][56] = {
    "TITLE     RED",
    "TITLE     YELLOW",
    "TITLE     GREEN",
    "TITLE     BLUE",
    "TITLE     L.BLUE",
    "TITLE     PURPLE",
    "TITLE     WHITE",
  };
  char char_display_subtitle_color[7][56] = {
    "SUBTITLE  RED",
    "SUBTITLE  YELLOW",
    "SUBTITLE  GREEN",
    "SUBTITLE  BLUE",
    "SUBTITLE  L.BLUE",
    "SUBTITLE  PURPLE",
    "SUBTITLE  WHITE",
  };
  int color_border = display_color[index_display_border_color];
  int color_content = display_color[index_display_content_color];
  int color_menu_content = display_color[index_display_menu_content_color];
  int color_menu_border = display_color[index_display_menu_border_color];
  int color_title = display_color[index_display_title_color];
  int color_subtitle = display_color[index_display_color_subtitle];

  // conversion maps
  char translate_enable_bool[2][10] = {"DISABLED", "ENABLED"}; // bool used as index selects bool translation
  char translate_plus_minus[2][2]  = {"+", "-"}; // bool used as index selects bool translation
  char translate_am_pm[2][4]  = {"AM", "PM"}; // bool used as index selects bool translation

  char tmp0[56];
  char tmp1[56];
};
systemStruct systemData;

void debug(String x) {if (systemData.debug==true) {Serial.println(x);}}

void bench(String x) {if (systemData.t_bench==true) {Serial.println(x);}}

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
  int max_matrix_filenames = 20; // max matrix file names available 
  char matrix_filenames[20][56] = {  
    "", "", "", "", "",
    "", "", "", "", "",
    "", "", "", "", "",
    "", "", "", "", "",
    };                                                   // matrix filenames created, stored and found by system
  char sysconf[56] = "/SYSTEM/SYSTEM.CONFIG";            // filepath
  char default_matrix_filepath[56] = "/MATRIX/M_0.SAVE"; // filepath
  char matrix_filename[56] = "";                         // filename
  char matrix_filepath[56] = "";                         // current matrix filepath
  char tempmatrixfilepath[56];                           // used for laoding filepaths
  char system_dirs[2][56] = {"/MATRIX/", "/SYSTEM/"};    // root dirs
  char save_ext[56] = ".SAVE";
  char matrix_fname[10] = "M";
  unsigned long iter_token; // count token iterations
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
  char delim[56] = ",";                                        // delimiter char
  char tmp[56];                                                // buffer
  char tag_0[56] = "r";                                        // file line tag
  char tag_1[56] = "e";                                        // file line tag
  ExFile current_file;                                         // file currently handled
  char newfilename[56];
};
SDCardStruct sdcardData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     DATA: TIME

struct TimeStruct {
  double seconds;                      // seconds accumulated since startup
  signed long mainLoopTimeTaken;     // current main loop time
  signed long mainLoopTimeStart;     // time recorded at the start of each iteration of main loop
  signed long mainLoopTimeTakenMax;  // current record of longest main loop time
  signed long mainLoopTimeTakenMin;  // current record of shortest main loop time
  unsigned long t0;                    // micros time 0
  unsigned long t1;                    // micros time 1
  uint32_t uptime_seconds;
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
  char moon_p_name[8][28] = {
    "New Moon",
    "Waxing Crescent",
    "First Quarter",
    "Waxing Gibbous",
    "Full Moon",
    "Waning Gibbous",
    "Third Quarter",
    "Waning Crescent"
  };
  double moon_lum;
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
  char sentence[1024];
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
  // debug("[connected] getCheckSum: " + String(string));
  for (SerialLink.XOR = 0, SerialLink.i_XOR = 0; SerialLink.i_XOR < strlen(string); SerialLink.i_XOR++) {
    SerialLink.c_XOR = (unsigned char)string[SerialLink.i_XOR];
    if (SerialLink.c_XOR == '*') break;
    if (SerialLink.c_XOR != '$') SerialLink.XOR ^= SerialLink.c_XOR;
  }
  // uncomment to debug
  // debug("[connected] getCheckSum: " + String(SerialLink.XOR));
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

  // debug("[validateChecksum]");
  // debug("[validateChecksum] " + String(buffer));

  memset(SerialLink.gotSum, 0, sizeof(SerialLink.gotSum));
  
  SerialLink.gotSum[0] = buffer[strlen(buffer) - 3];
  SerialLink.gotSum[1] = buffer[strlen(buffer) - 2];

  // debug("[checksum_in_buffer] " + String(SerialLink.gotSum));

  SerialLink.checksum_of_buffer =  getCheckSum(buffer);
  // debug("[checksum_of_buffer] " + String(SerialLink.checksum_of_buffer));
  // sprintf(SerialLink.checksum,"%X",SerialLink.checksum_of_buffer);
  // debug("[checksum_of_buffer converted] " + String(SerialLink.checksum));
  SerialLink.checksum_in_buffer = h2d2(SerialLink.gotSum[0], SerialLink.gotSum[1]);
  // debug("[checksum_in_buffer (h2d2)] " + String(SerialLink.checksum_in_buffer));

  if (SerialLink.checksum_in_buffer == SerialLink.checksum_of_buffer) {return true;}
  return false;
}

void createChecksum(char * buffer) {
  SerialLink.checksum_of_buffer = getCheckSum(buffer);

  // debug("[checksum_of_buffer] " + String(SerialLink.checksum_of_buffer));
  // debug("[hexadecimal number] " + String("%X", SerialLink.checksum_of_buffer)); todo

  sprintf(SerialLink.checksum,"%X",SerialLink.checksum_of_buffer);

  // debug("[checksum] " + String(SerialLink.checksum));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               VALIDATION: DATA

/*
checks can be tuned and ellaborated upon individually.
each sentence has a checksum that is for checking if the payload is more or less intact, while in contrast checks below are for
sanitizing each element of a sentence. thorough testing is required to ensure no false negatives/positives.
*/


bool count_digits(char * data, int expected) {
  validData.valid_i = 0;
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 1) {validData.valid_i++;}}
  if (validData.valid_i == expected) {return true;} else {return false;}
}

bool count_alpha(char * data, int expected) {
  validData.valid_i = 0;
  for (int i = 0; i < strlen(data); i++) {if (isalpha(data[i]) == 1) {validData.valid_i++;}}
  if (validData.valid_i == expected) {return true;} else {return false;}
}

bool is_all_digits(char * data) {
  validData.valid_b = true;
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {validData.valid_b = false;}}
  return validData.valid_b;
}

bool is_all_digits_plus_char(char * data, char find_char) {
  /* designed to check all chars are digits except one period and is more general purpose than just accepting a period */
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
  validData.valid_b = true;
  validData.find_char = strchr(data, '.');
  validData.index = (int)(validData.find_char - data);
  for (int i = 0; i < strlen(data); i++) {
    if (isdigit(data[i]) == 0) {if (i != validData.index) {if ((data[i] != '-') && (i > 0)) {validData.valid_b = false;}}}}
  return validData.valid_b;
}

bool is_all_alpha(char * data) {
  validData.valid_b = true;
  for (int i = 0; i < strlen(data); i++) {if (isalpha(data[i]) == 0) {validData.valid_b = false;}}
  return validData.valid_b;
}

bool val_utc_time(char * data) {
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
  bool check_pass = false;
  if (strlen(data) == 6) {
    if (is_all_digits(data) == true) {
      if ((atoi(data) >= 0.0) && (atoi(data) <= 999999)) {check_pass = true;}
    }
  }
  return check_pass;
}

bool val_latitude(char * data) {
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
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "N") == 0) || (strcmp(data, "S") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_longitude_H(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "E") == 0) || (strcmp(data, "W") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_positioning_status_gngga(char * data) {
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
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) >= 0){
      check_pass = true;
      }
  }
  return check_pass;
}

bool val_hdop_precision_factor(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if (atoi(data) >= 0){
      check_pass = true;
  }
  }
  return check_pass;
}

bool val_altitude(char * data) {
  // account for decimal point
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
      check_pass = true;
  }
  return check_pass;
}

bool val_altitude_units(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if (strcmp(data, "M") == 0) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_geoidal(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_geoidal_units(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if (strcmp(data, "M") == 0) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_differential_delay(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_basestation_id(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (strlen(data) == 4) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_positioning_status_gnrmc(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "A") == 0) || (strcmp(data, "V") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_ground_speed(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_ground_heading(char * data) {
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
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if (atoi(data) >= 0) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_installation_angle_direction(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "E") == 0) || (strcmp(data, "W") == 0) || (strcmp(data, "M") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_mode_indication(char * data) {
  bool check_pass = false;
  if (strlen(data) == 1) {
    if ((strcmp(data, "A") == 0) || (strcmp(data, "D") == 0) || (strcmp(data, "E") == 0) || (strcmp(data, "N") == 0)) {
      check_pass = true;
    }
  }
  return check_pass;
}

bool val_pitch_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_roll_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_yaw_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_angle_channle_p_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "p") == 0) {check_pass = true;}
  return check_pass;
}

bool val_angle_channle_r_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "r") == 0) {check_pass = true;}
  return check_pass;
}

bool val_angle_channle_y_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "y") == 0) {check_pass = true;}
  return check_pass;
}

bool val_version_channel_s_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "S") == 0) {check_pass = true;}
  return check_pass;
}

bool val_software_version_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) == 20230219) {check_pass = true;}
  }
  return check_pass;
}

bool val_product_id_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "003E009") == 0) {check_pass = true;}
  return check_pass;
}

bool val_id_channel_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "ID") == 0) {check_pass = true;}
  return check_pass;
}

bool val_ins_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_channel_gpatt(char * data) {
  bool check_pass = false;
  if (strcmp(data, "INS") == 0) {check_pass = true;}
  return check_pass;
}

bool val_hardware_version_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (strcmp(data, "3335") == 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_run_state_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((strcmp(data, "01") == 0) || (strcmp(data, "02") == 0) || (strcmp(data, "03") == 0)) {check_pass = true;}
  }
  return check_pass;
}

// todo
bool val_mis_angle_num_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_static_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

// todo
bool val_user_code_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_gst_data_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if (atoi(data) >= 0) {check_pass = true;}
  }
  return check_pass;
}

bool val_line_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_mis_att_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_imu_kind_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_car_kind_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 1) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_mileage_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_run_inetial_flag_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_speed_enable_gpatt(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) == 0) || (atoi(data) == 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_speed_num_gpatt(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_speed_status(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  }
  return check_pass;
}

bool val_accelleration_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "A") == 0) {check_pass = true;}
  return check_pass;
}

bool val_axis_accelleration(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_angular_velocity_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "G") == 0) {check_pass = true;}
  return check_pass;
}

bool val_gyro_angular_velocity(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_status_delimiter(char * data) {
  bool check_pass = false;
  if (strcmp(data, "S") == 0) {check_pass = true;}
  return check_pass;
}

bool val_ubi_state_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_state_kind_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_code_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_gset_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_sset_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ang_dget_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_run_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_fix_kind_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_fiobject_roll_flag(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_fix_pitch_flag(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_ubi_on_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 8)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_kind_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 2)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_a_set(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_b_set(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 19)) {check_pass = true;}
  }
  return check_pass;
}

bool val_acc_X_data(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_acc_Y_data(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_gyro_Z_data(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_pitch_angle(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_roll_angle(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_yaw_angle(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_car_speed(char * data) {
  bool check_pass = false;
  if (is_all_digits_plus_char(data, '.') == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 100)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ins_flag(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
    if ((atoi(data) >= 0) && (atoi(data) <= 4)) {check_pass = true;}
  }
  return check_pass;
}

bool val_ubi_num(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_ubi_valid(char * data) {
  bool check_pass = false;
  if (is_all_digits(data) == true) {
  if ((atoi(data) >= 0) && (atoi(data) <= 1)) {check_pass = true;}
  }
  return check_pass;
}

bool val_coll_T_data(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_coll_T_heading(char * data) {
  bool check_pass = false;
  if (is_positive_negative_num(data) == true) {
    check_pass = true;
  }
  return check_pass;
}

bool val_custom_flag(char * data) {
  bool check_pass = false;
  if (strlen(data) >= 1) {check_pass = true;}
  return check_pass;
}

bool val_checksum(char * data) {
  bool check_pass = false;
  if (strlen(data) == 3) {check_pass = true;}
  return check_pass;
}

bool val_scalable(char * data) {
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
  String tempStr = "";

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

  // reflects matrix switch inverted logic bool (per function)
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

  /*
  a placeholder for timings when timer functions are selected for a matrix switch.
  allows modulation with second resolution.
  currently intended as one timer per switch so be careful.
  */
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

  char expression[5][16] =
  {
    "", // empty for functions that take no expression
    "Equal",
    "Over",
    "Under",
    "Range",
  };
  int i_expression = 0;

  // number of available base function names that can be used to program a matrix switch
  int max_matrix_function_names = 134;
  char matrix_function_names[134][25] = 
  {
    "None",
    "Enabled",
    "Overload",
    "SwitchLink",
    "SecondsTimer",
    "RTCTime",
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
    "DegLat",
    "DegLon",
    "DegLatLon",
    "UTCTimeGNGGA",
    "PosStatusGNGGA",
    "SatCount",
    "HemiGNGGANorth",
    "HemiGNGGASouth",
    "HemiGNGGAEast",
    "HemiGNGGAWest",
    "GPSPrecision",
    "AltGNGGA",
    "UTCTimeGNRMC",
    "PosStatusGNRMCA",
    "PosStatusGNRMCV",
    "ModeGNRMCA",
    "ModeGNRMCD",
    "ModeGNRMCE",
    "ModeGNRMCN",
    "HemiGNRMCNorth",
    "HemiGNRMCSouth",
    "HemiGNRMCEast",
    "HemiGNRMCWest",
    "GSpeedGNRMC",
    "HeadingGNRMC",
    "UTCDateGNRMC",
    "LFlagGPATT",
    "SFlagGPATT",
    "RSFlagGPATT",
    "INSGPATT",
    "SpeedNumGPATT",
    "MileageGPATT",
    "GSTDataGPATT",
    "YawGPATT",
    "RollGPATT",
    "PitchGPATT",
    "GNGGAValidCS",
    "GNRMCValidCS",
    "GPATTValidCS",
    "GNGGAValidCD",
    "GNRMCValidCD",
    "GPATTValidCD",
    "SunAz",
    "SunAlt",
    "DayTime",
    "NightTime",
    "Sunrise",
    "Sunset",
    "MoonAz",
    "MoonAlt",
    "MoonUp",
    "MoonDown",
    "Moonrise",
    "Moonset",
    "MoonPhase",
    "MercuryAz",
    "MercuryAlt",
    "MercuryUp",
    "MercuryDown",
    "MercuryRise",
    "MercurySet",
    "VenusAz",
    "VenusAlt",
    "VenusUp",
    "VenusDown",
    "VenusRise",
    "VenusSet",
    "MarsAz",
    "MarsAlt",
    "MarsUp",
    "MarsDown",
    "MarsRise",
    "MarsSet",
    "JupiterAz",
    "JupiterAlt",
    "JupiterUp",
    "JupiterDown",
    "JupiterRise",
    "JupiterSet",
    "SaturnAz",
    "SaturnAlt",
    "SaturnUp",
    "SaturnDown",
    "SaturnRise",
    "SaturnSet",
    "UranusAz",
    "UranusAlt",
    "UranusUp",
    "UranusDown",
    "UranusRise",
    "UranusSet",
    "NeptuneAz",
    "NeptuneAlt",
    "NeptuneUp",
    "NeptuneDown",
    "NeptuneRise",
    "NeptuneSet",
    "DHT11H0",
    "DHT11C0",
    "DHT11F",
    "DHT11HIC0",
    "DHT11HIF0",
    "Sensor0",
    "Sensor1",
    "Sensor2",
    "Sensor3",
    "Sensor4",
    "Sensor5",
    "Sensor6",
    "Sensor7",
    "Sensor8",
    "Sensor9",
    "Sensor10",
    "Sensor11",
    "Sensor12",
    "Sensor13",
    "Sensor14",
    "Sensor15",
  };
};
MatrixStruct matrixData;

// note that we could work out of this item list entirely to be more efficient but then our function name items would have a
// display driver dependency so for now we have two instances and with the menu items depending on our actual item list.
const char *menuMatrixSetFunctionNameItems[134] =
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
  // matrixData.matrix_function_names[134],
  // matrixData.matrix_function_names[135],
  // matrixData.matrix_function_names[136],
  // matrixData.matrix_function_names[137],
  // matrixData.matrix_function_names[138],
  // matrixData.matrix_function_names[139],
  // matrixData.matrix_function_names[140],
  // matrixData.matrix_function_names[141],
  // matrixData.matrix_function_names[142],
  // matrixData.matrix_function_names[143],
  // matrixData.matrix_function_names[144],
  // matrixData.matrix_function_names[145],
  // matrixData.matrix_function_names[146],
  // matrixData.matrix_function_names[147],
  // matrixData.matrix_function_names[148],
  // matrixData.matrix_function_names[149],
  // matrixData.matrix_function_names[150],
  // matrixData.matrix_function_names[151],
  // matrixData.matrix_function_names[152],
  // matrixData.matrix_function_names[153],
  // matrixData.matrix_function_names[154],
  // matrixData.matrix_function_names[155],
  // matrixData.matrix_function_names[156],
  // matrixData.matrix_function_names[157],
  // matrixData.matrix_function_names[158],
  // matrixData.matrix_function_names[159],
  // matrixData.matrix_function_names[160],
  // matrixData.matrix_function_names[161],
  // matrixData.matrix_function_names[162],
  // matrixData.matrix_function_names[163],
  // matrixData.matrix_function_names[164],
  // matrixData.matrix_function_names[165],
  // matrixData.matrix_function_names[166],
  // matrixData.matrix_function_names[167],
  // matrixData.matrix_function_names[168],
  // matrixData.matrix_function_names[169],
  // matrixData.matrix_function_names[170],
  // matrixData.matrix_function_names[171],
  // matrixData.matrix_function_names[172],
  // matrixData.matrix_function_names[173],
  // matrixData.matrix_function_names[174],
  // matrixData.matrix_function_names[175],
  // matrixData.matrix_function_names[176],
  // matrixData.matrix_function_names[177],
  // matrixData.matrix_function_names[178],
  // matrixData.matrix_function_names[179],
  // matrixData.matrix_function_names[180],
  // matrixData.matrix_function_names[181],
  // matrixData.matrix_function_names[182],
  // matrixData.matrix_function_names[183],
  // matrixData.matrix_function_names[184],
  // matrixData.matrix_function_names[185],
  // matrixData.matrix_function_names[186],
  // matrixData.matrix_function_names[187],
  // matrixData.matrix_function_names[188],
  // matrixData.matrix_function_names[189],
  // matrixData.matrix_function_names[190],
  // matrixData.matrix_function_names[191],
  // matrixData.matrix_function_names[192],
  // matrixData.matrix_function_names[193],
  // matrixData.matrix_function_names[194],
  // matrixData.matrix_function_names[195],
  // matrixData.matrix_function_names[196],
  // matrixData.matrix_function_names[197],
  // matrixData.matrix_function_names[198],
  // matrixData.matrix_function_names[199],
  // matrixData.matrix_function_names[200],
  // matrixData.matrix_function_names[201],
  // matrixData.matrix_function_names[202],
  // matrixData.matrix_function_names[203],
  // matrixData.matrix_function_names[204],
  // matrixData.matrix_function_names[205],
  // matrixData.matrix_function_names[206],
  // matrixData.matrix_function_names[207],
  // matrixData.matrix_function_names[208],
  // matrixData.matrix_function_names[209],
  // matrixData.matrix_function_names[210],
  // matrixData.matrix_function_names[211],
  // matrixData.matrix_function_names[212],
  // matrixData.matrix_function_names[213],
  // matrixData.matrix_function_names[214],
  // matrixData.matrix_function_names[215],
  // matrixData.matrix_function_names[216],
  // matrixData.matrix_function_names[217],
  // matrixData.matrix_function_names[218],
  // matrixData.matrix_function_names[219],
  // matrixData.matrix_function_names[220],
  // matrixData.matrix_function_names[221],
  // matrixData.matrix_function_names[222],
  // matrixData.matrix_function_names[223],
  // matrixData.matrix_function_names[224],
  // matrixData.matrix_function_names[225],
  // matrixData.matrix_function_names[226],
  // matrixData.matrix_function_names[227],
  // matrixData.matrix_function_names[228],
  // matrixData.matrix_function_names[229],
  // matrixData.matrix_function_names[230],
  // matrixData.matrix_function_names[231],
  // matrixData.matrix_function_names[232],
  // matrixData.matrix_function_names[233],
  // matrixData.matrix_function_names[234],
  // matrixData.matrix_function_names[235],
  // matrixData.matrix_function_names[236],
  // matrixData.matrix_function_names[237],
  // matrixData.matrix_function_names[238],
  // matrixData.matrix_function_names[239],
  // matrixData.matrix_function_names[240],
  // matrixData.matrix_function_names[241],
  // matrixData.matrix_function_names[242],
  // matrixData.matrix_function_names[243],
  // matrixData.matrix_function_names[244],
  // matrixData.matrix_function_names[245],
  // matrixData.matrix_function_names[246],
  // matrixData.matrix_function_names[247],
  // matrixData.matrix_function_names[248],
  // matrixData.matrix_function_names[249],
  // matrixData.matrix_function_names[250],
  // matrixData.matrix_function_names[251],
  // matrixData.matrix_function_names[252],
  // matrixData.matrix_function_names[253],
  // matrixData.matrix_function_names[254],
  // matrixData.matrix_function_names[255],
};
LcdGfxMenu menuMatrixSetFunctionName( menuMatrixSetFunctionNameItems, 134, {{2, 46}, {125, 125}} );

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: GNGGA

struct GNGGAStruct {
  char sentence[200];
  char outsentence[200];
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
  if (systemData.debug == true) {
    Serial.println("[gnggaData.tag] "                     + String(gnggaData.tag));
    Serial.println("[gnggaData.utc_time] "                + String(gnggaData.utc_time));
    Serial.println("[gnggaData.latitude] "                + String(gnggaData.latitude));
    Serial.println("[gnggaData.latitude_hemisphere] "     + String(gnggaData.latitude_hemisphere));
    Serial.println("[gnggaData.longitude] "               + String(gnggaData.longitude));
    Serial.println("[gnggaData.longitude_hemisphere] "    + String(gnggaData.longitude_hemisphere));
    Serial.println("[gnggaData.solution_status] "         + String(gnggaData.solution_status));
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
  char outsentence[200];
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
  if (systemData.debug == true) {
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
  char outsentence[200];
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
  if (systemData.debug == true) {
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
    Serial.println("[gpattData.check_data] "       + String(gpattData.check_data));
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    DATA: SATIO

struct SatDatatruct {
  int checksum_i;                                                  // checksum int
  char satio_sentence[200];                                        // buffer
  char satDataTag[56]                 = "$SATIO";                  // satio sentence tag
  char rtcSyncDatetimeStamp[56]       = "0.0";                     // record last time satellites were seen
  char rtcSyncDatetime[56]            = "0.0";                     // record last time satellites were seen
  char rtcSyncTime[56]                = "0.0";                     // record last time satellites were seen
  char rtcSyncDate[56]                = "0.0";                     // record last time satellites were seen
  bool convert_coordinates            = true;                      // enables/disables coordinate conversion to degrees
  char coordinate_conversion_mode[56] = "GNGGA";                   // sentence coordinates degrees created from
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
  double degrees_latitude;                                         // degrees converted from absolute
  double degrees_longitude;                                        // degrees converted from absolute
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

  /* CD74HC4067 x16 Analog/Digital Multiplexer */

  // specific analog/digital sensor: can be refactored
  float dht11_h_0 = 0.0;
  float dht11_c_0 = 0.0;
  float dht11_f_0 = 0.0;
  float dht11_hif_0 = 0.0;
  float dht11_hic_0 = 0.0;
  bool dht11_0_display_hic = true;

  // general analog/digital sensor: can be refactored
  float sensor_0 = 0.0;
  float sensor_1 = 0.0;
  float sensor_2 = 0.0;
  float sensor_3 = 0.0;
  float sensor_4 = 0.0;
  float sensor_5 = 0.0;
  float sensor_6 = 0.0;
  float sensor_7 = 0.0;
  float sensor_8 = 0.0;
  float sensor_9 = 0.0;
  float sensor_10 = 0.0;
  float sensor_11 = 0.0;
  float sensor_12 = 0.0;
  float sensor_13 = 0.0;
  float sensor_14 = 0.0;
  float sensor_15 = 0.0;

  char sensor_sentence[1024];
  char TMP[1024];
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


String formatRTCDateTime() {
  return 
  String(String(padDigitsZero( rtc.now().hour())) + ":" + String(padDigitsZero(rtc.now().minute())) + ":" + String(padDigitsZero(rtc.now().second())) + " " +
  String(padDigitsZero(rtc.now().day())) + "." + String(padDigitsZero(rtc.now().month())) + "." + String(padDigitsZero(rtc.now().year())));
}

String formatRTCDate() {
  return 
  String(padDigitsZero(rtc.now().day())) + "." + String(padDigitsZero(rtc.now().month())) + "." + String(padDigitsZero(rtc.now().year()));
}

String formatRTCDateAbbreviated() {
  return 
  String(padDigitsZero(rtc.now().day())) + "." + String(padDigitsZero(rtc.now().month())) + "." + String(padDigitsZero(rtc.now().year())[2]) + String(padDigitsZero(rtc.now().year())[3]);
}

String formatRTCTime() {
  return 
  String(String(padDigitsZero( rtc.now().hour())) + ":" + String(padDigitsZero(rtc.now().minute())) + ":" + String(padDigitsZero(rtc.now().second())));
}

String formatRTCDateTimeStamp() {
  /* decend units of time for timestamp */
  return 
  String(padDigitsZero(rtc.now().year())) + String(padDigitsZero(rtc.now().month())) + String(padDigitsZero(rtc.now().day())) +
  String(String(padDigitsZero( rtc.now().hour())) + String(padDigitsZero(rtc.now().minute())) + String(padDigitsZero(rtc.now().second())));
}

String formatRTCDateStamp() {
   /* decend units of time for timestamp */
  return String(padDigitsZero(rtc.now().day())) + String(padDigitsZero(rtc.now().month())) + String(padDigitsZero(rtc.now().year()));
}

String formatRTCTImeStamp() {
   /* decend units of time for timestamp */
  return String(String(padDigitsZero( rtc.now().hour())) + String(padDigitsZero(rtc.now().minute())) + String(padDigitsZero(rtc.now().second())));
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
    satData.degrees_latitude =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnggaData.latitude_hemisphere, "S") == 0) {
      satData.degrees_latitude = 0 - satData.degrees_latitude;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.degrees_latitude);

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
    satData.degrees_longitude =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnggaData.longitude_hemisphere, "W") == 0) {
      satData.degrees_longitude = 0 - satData.degrees_longitude;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.degrees_longitude);
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
    satData.degrees_latitude =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnrmcData.latitude_hemisphere, "S") == 0) {
      satData.degrees_latitude = 0 - satData.degrees_latitude;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.degrees_latitude);

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
    satData.degrees_longitude =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
    // Negate latitude value if it's in the Southern hemisphere (make negative value).
    if (strcmp(gnrmcData.longitude_hemisphere, "W") == 0) {
      satData.degrees_longitude = 0 - satData.degrees_longitude;
    }
    // Save formatted latitude value as a string for later use.
    scanf("%f17", &satData.degrees_longitude);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                        SET LAST SATELLITE TIME

void syncRTCOnDownlink() {
  rtc.adjust(DateTime(satData.lt_year_int, satData.lt_month_int, satData.lt_day_int, satData.lt_hour_int, satData.lt_minute_int, satData.lt_second_int));
  // debug("[synchronized] " + formatRTCDateTime());
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                           CONVERT UTC TO LOCAL

int hoursMinutesSecondsToInt(int hours, int minutes, int seconds) {
  return atoi(String(padDigitsZero(hours) + padDigitsZero(minutes) + padDigitsZero(seconds)).c_str());
}

int hoursMinutesToInt(int hours, int minutes) {
  return atoi(String(padDigitsZero(hours) + padDigitsZero(minutes)).c_str());
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

  // live data from satellites
  // debug("[utc_time] " + String(gnrmcData.utc_time));
  // debug("[utc_date] " + String(gnrmcData.utc_date));

  /*                                     TEMPORARY TIME                                        */
  /* make temporary values that will not disturb final values untiil values whole and complete */

  // temp store date
  satData.tmp_day[0] = gnrmcData.utc_date[0];
  satData.tmp_day[1] = gnrmcData.utc_date[1];
  // debug("[tmp_day] " + String(satData.tmp_day));
  satData.tmp_month[0] = gnrmcData.utc_date[2];
  satData.tmp_month[1] = gnrmcData.utc_date[3];
  // debug("[tmp_month] " + String(satData.tmp_month));
  satData.tmp_year[0] = gnrmcData.utc_date[4];
  satData.tmp_year[1] = gnrmcData.utc_date[5];
  // debug("[tmp_year] " + String(satData.tmp_year));

  // temp store time
  satData.tmp_hour[0] = gnrmcData.utc_time[0];
  satData.tmp_hour[1] = gnrmcData.utc_time[1];
  // debug("[tmp_hour] " + String(satData.tmp_hour));
  satData.tmp_minute[0] = gnrmcData.utc_time[2];
  satData.tmp_minute[1] = gnrmcData.utc_time[3];
  // debug("[tmp_minute] " + String(satData.tmp_minute));
  satData.tmp_second[0] = gnrmcData.utc_time[4];
  satData.tmp_second[1] = gnrmcData.utc_time[5];
  // debug("[tmp_second] " + String(satData.tmp_second));
  satData.tmp_millisecond[0] = gnrmcData.utc_time[7];
  satData.tmp_millisecond[1] = gnrmcData.utc_time[8];
  // debug("[tmp_second] " + String(satData.tmp_millisecond));

  // temporary int time values so that we do not disturb the primary values while converting.
  satData.tmp_day_int = atoi(satData.tmp_day);
  satData.tmp_month_int = atoi(satData.tmp_month);
  satData.tmp_year_int = atoi(satData.tmp_year);
  satData.tmp_hour_int = atoi(satData.tmp_hour);
  satData.tmp_minute_int = atoi(satData.tmp_minute);
  satData.tmp_second_int = atoi(satData.tmp_second);
  satData.tmp_millisecond_int = atoi(satData.tmp_millisecond);

  // before conversion
  // debug("[temp time] " + String(satData.tmp_hour_int) + ":" + String(satData.tmp_minute_int) + "." + String(satData.tmp_second_int));
  // debug("[temp date] " + String(satData.tmp_day_int) + "." + String(satData.tmp_month_int) + "." + String(satData.tmp_year_int));

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
  // debug("[tmp_makeTime] " + String(tmp_makeTime));

  // adjust tmp_makeTime back/forward according to UTC offset
  if      (satData.utc_offset_flag==0) {adjustTime(satData.utc_offset*SECS_PER_HOUR);}
  else                                 {adjustTime(-satData.utc_offset*SECS_PER_HOUR);}

  // before conversion
  // debug("[temp time  +- offset] " + String(hour()) + ":" + String(minute()) + "." + String(second()));
  // debug("[temp date +- offset] " + String(day()) + "." + String(month()) + "." + String(year()));

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
      if (satData.tmp_millisecond_int==00) {
        first_gps_pass=false; // dont drop in here next time
        syncRTCOnDownlink();  // sync rtc

        // set last sync datetime
        memset(satData.rtcSyncDatetimeStamp, 0, sizeof(satData.rtcSyncDatetimeStamp));
        strcpy(satData.rtcSyncDatetimeStamp, formatRTCDateTimeStamp().c_str());

        memset(satData.rtcSyncDatetime, 0, sizeof(satData.rtcSyncDatetime));
        strcpy(satData.rtcSyncDatetime, formatRTCDateTime().c_str());

        memset(satData.rtcSyncTime, 0, sizeof(satData.rtcSyncTime));
        strcpy(satData.rtcSyncTime, formatRTCTime().c_str());

        memset(satData.rtcSyncDate, 0, sizeof(satData.rtcSyncDate));
        strcpy(satData.rtcSyncDate, formatRTCDate().c_str());
      }
    }
    else {
      // sync every minute according to downlinked time. 
      if (satData.lt_second_int == 0) {
        if (satData.tmp_millisecond_int==00) {
          syncRTCOnDownlink(); // sync rtc

          // set last sync datetime
          memset(satData.rtcSyncDatetimeStamp, 0, sizeof(satData.rtcSyncDatetimeStamp));
          strcpy(satData.rtcSyncDatetimeStamp, formatRTCDateTimeStamp().c_str());

          memset(satData.rtcSyncDatetime, 0, sizeof(satData.rtcSyncDatetime));
          strcpy(satData.rtcSyncDatetime, formatRTCDateTime().c_str());

          memset(satData.rtcSyncTime, 0, sizeof(satData.rtcSyncTime));
          strcpy(satData.rtcSyncTime, formatRTCTime().c_str());

          memset(satData.rtcSyncDate, 0, sizeof(satData.rtcSyncDate));
          strcpy(satData.rtcSyncDate, formatRTCDate().c_str());
        }
      }
    }
  }

  // debug("[rtc time] " + formatRTCDateTime()); // debug

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

  // current rtc unixtime
  strcat(satData.satio_sentence, String(formatRTCDateTimeStamp()).c_str());
  strcat(satData.satio_sentence, ",");

  // last downlink sync rtc
  strcat(satData.satio_sentence, satData.rtcSyncDatetimeStamp);
  strcat(satData.satio_sentence, ",");

  // system uptime in seconds (may be preferrable than system startup datetime because datetime is local)
  strcat(satData.satio_sentence, String(timeData.uptime_seconds).c_str());
  strcat(satData.satio_sentence, ",");

  // sun rise time
  strcat(satData.satio_sentence, String(siderealPlanetData.sun_r).c_str());
  strcat(satData.satio_sentence, ",");

  // sun set time
  strcat(satData.satio_sentence, String(siderealPlanetData.sun_s).c_str());
  strcat(satData.satio_sentence, ",");

  // coordinate conversion mode
  // if (satData.convert_coordinates == true) {
  //   if (String(satData.coordinate_conversion_mode) == "GNGGA") {
      // append to satio sentence
  strcat(satData.satio_sentence, String(satData.degrees_latitude, 7).c_str());
  strcat(satData.satio_sentence, ",");
  strcat(satData.satio_sentence, String(satData.degrees_longitude, 7).c_str());
  strcat(satData.satio_sentence, ",");
    // }
    // else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
    //   // append to satio sentence
    //   strcat(satData.satio_sentence, String(satData.degrees_latitude, 7).c_str());
    //   strcat(satData.satio_sentence, ",");
    //   strcat(satData.satio_sentence, String(satData.degrees_longitude, 7).c_str());
    //   strcat(satData.satio_sentence, ",");
    // }
  // }
  // else {strcat(satData.satio_sentence, "0.0,0.0,");}

  // append checksum
  createChecksum(satData.satio_sentence);
  strcat(satData.satio_sentence, "*");
  strcat(satData.satio_sentence, SerialLink.checksum);
  if (systemData.output_satio_enabled == true) {Serial.println(satData.satio_sentence);}
  // debug(satData.satio_sentence);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                          SDCARD: PRINT FILE CONTENTS TO SERIAL

// ls, cat, etc

// void cd(char * data) {
//   memset(cwd, 0, sizeof(cwd));
//   strcpy(cwd, data);
// }

// void printDirectory(File dir) {

//   /* prints files and dirs in current working directory */

//   Serial.println();
//   Serial.println("[ls] " + String(cwd));
//   Serial.println();

//   while (true) {

//     entry =  dir.openNextFile();
    
//     // no more files
//     if (! entry) {break;}

//     // print dirs
//     if (entry.isDirectory()) {
//       Serial.print(entry.name());
//       Serial.println("/");
//     }
//     // print files (files have sizes, directories do not)
//     else {
//       Serial.print(entry.name());
//       Serial.print(" ");
//       Serial.println(entry.size(), DEC);
//     }
//     entry.close();
//   }

//   Serial.println();
// }

// void ls() {
//   endSSD1351();
//   beginSDCARD();
//   root = sd.open(cwd);
//   printDirectory(root);
//   endSDCARD();
//   beginSSD1351();
// }

// bool sdcard_read_to_serial(fs::FS &fs, char * file) {

//   /* prints the contents of a file to serial  */

//   sdcardData.current_file.flush();
//   sdcardData.current_file = exfile.open(file);
//   if (sdcardData.current_file) {
//     while (sdcardData.current_file.available()) {Serial.write(sdcardData.current_file.read());}
//     sdcardData.current_file.close(); return true;
//   }
//   else {sdcardData.current_file.close(); return false;}
// }

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                    SDCARD: UPDATE MATRIX FILEPATH AND FILENAME

void UpdateMatrixFileNameFilePath(char * filepath) {
  memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
  strcpy(sdcardData.matrix_filepath, filepath);
  if (strlen(sdcardData.matrix_filepath)>8) {
    memset(sdcardData.matrix_filename, 0, sizeof(sdcardData.matrix_filename));
    strncpy(sdcardData.matrix_filename, sdcardData.matrix_filepath + 8, strlen(sdcardData.matrix_filepath));
    Serial.println("[sdcardData.matrix_filepath] " + String(sdcardData.matrix_filepath));
    Serial.println("[sdcardData.matrix_filename] " + String(sdcardData.matrix_filename));
  }
  Serial.println("[UpdateMatrixFileNameFilePath] sdcardData.matrix_filepath: " + String(sdcardData.matrix_filepath));
}

void UpdateMatrixFileName(char * filepath) {
  if (strlen(sdcardData.matrix_filepath)>8) {
    memset(sdcardData.matrix_filename, 0, sizeof(sdcardData.matrix_filename));
    strncpy(sdcardData.matrix_filename, sdcardData.matrix_filepath + 8, strlen(sdcardData.matrix_filepath));
    Serial.println("[sdcardData.matrix_filepath] " + String(sdcardData.matrix_filepath));
    Serial.println("[sdcardData.matrix_filename] " + String(sdcardData.matrix_filename));
  }
  Serial.println("[UpdateMatrixFileNameFilePath] sdcardData.matrix_filepath: " + String(sdcardData.matrix_filepath));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                              SDCARD: SAVE SYSTEM CONFIGURATION

void sdcard_save_system_configuration(char * file) {

  /* saves tagged, system configuration data to file */

  // ------------------------------------------------

  Serial.println("[sdcard] attempting to save file: " + String(file));
  exfile.flush();
  exfile = sd.open(file, O_WRITE | O_CREAT);

  // ------------------------------------------------

  if (exfile) {

    // ------------------------------------------------
    Serial.println("[sdcard_save_system_configuration] sdcardData.matrix_filepath: " + String(sdcardData.matrix_filepath));

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "MATRIX_FILEPATH,");
    if (!sdcardData.matrix_filepath) {strcat(sdcardData.file_data, sdcardData.default_matrix_filepath);}
    else {strcat(sdcardData.file_data, sdcardData.matrix_filepath);}
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "AUTO_RESUME,");
    itoa(systemData.matrix_run_on_startup, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "DISPLAY_AUTO_OFF,");
    itoa(systemData.display_auto_off, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_AUTO_OFF,");
    itoa(systemData.index_display_autooff_times, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_BORDER_COLOR,");
    itoa(systemData.index_display_border_color, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_CONTENT_COLOR,");
    itoa(systemData.index_display_content_color, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_MENU_BORDER_COLOR,");
    itoa(systemData.index_display_menu_border_color, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

      // ------------------------------------------------

      memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
      strcat(sdcardData.file_data, "INDEX_DISPLAY_MENU_CONTENT_COLOR,");
      itoa(systemData.index_display_menu_content_color, sdcardData.tmp, 10);
      strcat(sdcardData.file_data, sdcardData.tmp);
      strcat(sdcardData.file_data, ",");
      Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
      exfile.println("");
      exfile.println(sdcardData.file_data);
      exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_TITLE_COLOR,");
    itoa(systemData.index_display_title_color, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "INDEX_DISPLAY_COLOR_SUBTITLE,");
    itoa(systemData.index_display_color_subtitle, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "MATRIX_ENABLED,");
    itoa(systemData.matrix_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "SATIO_ENABLED,");
    itoa(systemData.satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "GNGGA_ENABLED,");
    itoa(systemData.gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "GNRMC_ENABLED,");
    itoa(systemData.gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "GPATT_ENABLED,");
    itoa(systemData.gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_SATIO_SENTENCE,");
    itoa(systemData.output_satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_GNGGA_SENTENCE,");
    itoa(systemData.output_gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_GNRMC_SENTENCE,");
    itoa(systemData.output_gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_GPATT_SENTENCE,");
    itoa(systemData.output_gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_MATRIX_SENTENCE,");
    itoa(systemData.output_matrix_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_SENSORS_SENTENCE,");
    itoa(systemData.output_sensors_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "UTC_OFFSET,");
    itoa(satData.utc_offset, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "UTC_OFFSET_FLAG,");
    itoa(satData.utc_offset_flag, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_SUN,");
    itoa(systemData.sidereal_track_sun, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_MOON,");
    itoa(systemData.sidereal_track_moon, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_MERCURY,");
    itoa(systemData.sidereal_track_mercury, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_VENUS,");
    itoa(systemData.sidereal_track_venus, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_MARS,");
    itoa(systemData.sidereal_track_mars, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------
    
    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_JUPITER,");
    itoa(systemData.sidereal_track_jupiter, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_SATURN,");
    itoa(systemData.sidereal_track_saturn, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_URANUS,");
    itoa(systemData.sidereal_track_uranus, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "TRACK_NEPTUNE,");
    itoa(systemData.sidereal_track_neptune, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_SUN,");
    itoa(systemData.output_sun_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_MOON,");
    itoa(systemData.output_moon_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_MERCURY,");
    itoa(systemData.output_mercury_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_VENUS,");
    itoa(systemData.output_venus_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_MARS,");
    itoa(systemData.output_mars_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------
    
    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_JUPITER,");
    itoa(systemData.output_jupiter_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_SATURN,");
    itoa(systemData.output_saturn_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_URANUS,");
    itoa(systemData.output_uranus_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    memset(sdcardData.file_data, 0, sizeof(sdcardData.file_data));
    strcat(sdcardData.file_data, "OUTPUT_NEPTUNE,");
    itoa(systemData.output_neptune_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    exfile.println("");
    exfile.println(sdcardData.file_data);
    exfile.println("");

    // ------------------------------------------------

    exfile.close();
    Serial.println("[sdcard] saved file successfully: " + String(file));

    // ------------------------------------------------
  }

  // ------------------------------------------------

  else {exfile.close(); Serial.println("[sdcard] failed to save file: " + String(file));}

  // ------------------------------------------------
}

void PrintFileToken() {Serial.println("[sdcard] [reading] " +  String(sdcardData.token));}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                              SDCARD: LOAD SYSTEM CONFIGURATION 

bool sdcard_load_system_configuration(char * file) {

  Serial.println("[sdcard] attempting to load file: " + String(file));
  exfile.flush();
  if (exfile.open(file, O_RDONLY)==1) {

    while (exfile.available()) {

      // read line
      sdcardData.SBUFFER = "";
      memset(sdcardData.BUFFER, 0, sizeof(sdcardData.BUFFER));
      sdcardData.SBUFFER = exfile.readStringUntil('\n');
      sdcardData.SBUFFER.toCharArray(sdcardData.BUFFER, sdcardData.SBUFFER.length()+1);
      Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));

      // ------------------------------------------------

      // check matrix filepath
      if (strncmp(sdcardData.BUFFER, "MATRIX_FILEPATH", 15) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        PrintFileToken();
        // update filename and file path
        UpdateMatrixFileNameFilePath(sdcardData.token);
      }

      // ------------------------------------------------

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

      // ------------------------------------------------

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

      // ------------------------------------------------

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

      // ------------------------------------------------

      // display border color index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_BORDER_COLOR", strlen("INDEX_DISPLAY_BORDER_COLOR")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_border_color = atoi(sdcardData.token);
          systemData.color_border = systemData.display_color[systemData.index_display_border_color];
          // Serial.println("[index_display_border_color] " + String(systemData.index_display_border_color));
        }
      }

      // ------------------------------------------------

      // display content color index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_CONTENT_COLOR", strlen("INDEX_DISPLAY_CONTENT_COLOR")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_content_color = atoi(sdcardData.token);
          systemData.color_content = systemData.display_color[systemData.index_display_content_color];
          // Serial.println("[color_content] " + String(systemData.color_content));
        }
      }

      // ------------------------------------------------

      // display menu border color index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_MENU_BORDER_COLOR", strlen("INDEX_DISPLAY_MENU_BORDER_COLOR")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_menu_border_color= atoi(sdcardData.token);
          systemData.color_menu_border = systemData.display_color[systemData.index_display_menu_border_color];
          // Serial.println("[color_menu_border] " + String(systemData.color_menu_border));
        }
      }

      // ------------------------------------------------

      // display menu content color index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_MENU_CONTENT_COLOR", strlen("INDEX_DISPLAY_MENU_CONTENT_COLOR")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_menu_content_color = atoi(sdcardData.token);
          systemData.color_menu_content = systemData.display_color[systemData.index_display_menu_content_color];
          // Serial.println("[color_menu_content] " + String(systemData.color_menu_content));
        }
      }

      // ------------------------------------------------

      // display title color index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_TITLE_COLOR", strlen("INDEX_DISPLAY_TITLE_COLOR")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_title_color = atoi(sdcardData.token);
          systemData.color_title = systemData.display_color[systemData.index_display_title_color];
          // Serial.println("[color_menu_content] " + String(systemData.color_menu_content));
        }
      }

      // ------------------------------------------------

      // display subtitle color index
      if (strncmp(sdcardData.BUFFER, "INDEX_DISPLAY_COLOR_SUBTITLE", strlen("INDEX_DISPLAY_COLOR_SUBTITLE")) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        PrintFileToken();
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          PrintFileToken();
          systemData.index_display_color_subtitle = atoi(sdcardData.token);
          systemData.color_subtitle = systemData.display_color[systemData.index_display_color_subtitle];
          // Serial.println("[color_menu_content] " + String(systemData.color_menu_content));
        }
      }

      // ------------------------------------------------

      // continue to enable/disable only if auto resume is true
      if (systemData.matrix_run_on_startup == true) {

        // ------------------------------------------------

        if (strncmp(sdcardData.BUFFER, "MATRIX_ENABLED", strlen("MATRIX_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.matrix_enabled = false;} else {systemData.matrix_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "SATIO_ENABLED", strlen("SATIO_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.satio_enabled = false;} else {systemData.satio_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "GNGGA_ENABLED", strlen("GNGGA_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.gngga_enabled = false;} else {systemData.gngga_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "GNRMC_ENABLED", strlen("GNRMC_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.gnrmc_enabled = false;} else {systemData.gnrmc_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "GPATT_ENABLED", strlen("GPATT_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.gpatt_enabled = false;} else {systemData.gpatt_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_SATIO_SENTENCE", strlen("OUTPUT_SATIO_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_satio_enabled = false;} else {systemData.output_satio_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GNGGA_SENTENCE", strlen("OUTPUT_GNGGA_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_gngga_enabled = false;} else {systemData.output_gngga_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GNRMC_SENTENCE", strlen("OUTPUT_GNRMC_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_gnrmc_enabled = false;} else {systemData.output_gnrmc_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GPATT_SENTENCE", strlen("OUTPUT_GPATT_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_gpatt_enabled = false;} else {systemData.output_gpatt_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_MATRIX_SENTENCE", strlen("OUTPUT_MATRIX_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_matrix_enabled = false;} else {systemData.output_matrix_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_SENSORS_SENTENCE", strlen("OUTPUT_SENSORS_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_sensors_enabled = false;} else {systemData.output_sensors_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "UTC_OFFSET,", strlen("UTC_OFFSET,")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            satData.utc_offset = atoi(sdcardData.token);
          }
        }

        // ------------------------------------------------
        
        else if (strncmp(sdcardData.BUFFER, "UTC_OFFSET_FLAG", strlen("UTC_OFFSET_FLAG")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {satData.utc_offset_flag = false;} else {satData.utc_offset_flag = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_SUN", strlen("TRACK_SUN")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_sun = false;} else {systemData.sidereal_track_sun = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_MOON", strlen("TRACK_MOON")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_moon = false;} else {systemData.sidereal_track_moon = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_MERCURY", strlen("TRACK_MERCURY")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_mercury = false;} else {systemData.sidereal_track_mercury = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_VENUS", strlen("TRACK_VENUS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_venus = false;} else {systemData.sidereal_track_venus = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_MARS", strlen("TRACK_MARS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_mars = false;} else {systemData.sidereal_track_mars = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_JUPITER", strlen("TRACK_JUPITER")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_jupiter = false;} else {systemData.sidereal_track_jupiter = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_SATURN", strlen("TRACK_SATURN")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_saturn = false;} else {systemData.sidereal_track_saturn = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_URANUS", strlen("TRACK_URANUS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_uranus = false;} else {systemData.sidereal_track_uranus = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "TRACK_NEPTUNE", strlen("TRACK_NEPTUNE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.sidereal_track_neptune = false;} else {systemData.sidereal_track_neptune = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_SUN", strlen("OUTPUT_SUN")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_sun_enabled = false;} else {systemData.output_sun_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_MOON", strlen("OUTPUT_MOON")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_moon_enabled = false;} else {systemData.output_moon_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_MERCURY", strlen("OUTPUT_MERCURY")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_mercury_enabled = false;} else {systemData.output_mercury_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_VENUS", strlen("OUTPUT_VENUS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_venus_enabled = false;} else {systemData.output_venus_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_MARS", strlen("OUTPUT_MARS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_mars_enabled = false;} else {systemData.output_mars_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_JUPITER", strlen("OUTPUT_JUPITER")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_jupiter_enabled = false;} else {systemData.output_jupiter_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_SATURN", strlen("OUTPUT_SATURN")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_saturn_enabled = false;} else {systemData.output_saturn_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_URANUS", strlen("OUTPUT_URANUS")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_uranus_enabled = false;} else {systemData.output_uranus_enabled = true;}
          }
        }

        // ------------------------------------------------

        else if (strncmp(sdcardData.BUFFER, "OUTPUT_NEPTUNE", strlen("OUTPUT_NEPTUNE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          PrintFileToken();
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            PrintFileToken();
            if (atoi(sdcardData.token) == 0) {systemData.output_neptune_enabled = false;} else {systemData.output_neptune_enabled = true;}
          }
        }

        // ------------------------------------------------
        
      }
    }
    exfile.close();
    Serial.println("[sdcard] loaded file successfully: " + String(file));
    return true;
  }
  else {exfile.close(); Serial.println("[sdcard] failed to load file: " + String(file));
  return false;}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                         SDCARD: MAKE DIRECTORY

/* creates a single directory */

void sdcard_mkdir(char * dir){

  if (!sd.exists(dir)) {
    Serial.println("[sdcard] attempting to create directory: " + String(dir));
    if (!sd.mkdir(dir)) {Serial.println("[sdcard] failed to create directory: " + String(dir));}
    else {Serial.println("[sdcard] found directory: " + String(dir));}}
  else {Serial.println("[sdcard] directory already exists: " + String(dir));}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                       SDCARD: MAKE DIRECTORIES

/* creates root directories required by the system to work properly */

void sdcard_mkdirs() {for (int i = 0; i < 2; i++) {sdcard_mkdir(sdcardData.system_dirs[i]);}}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                      SDCARD: PUT ALL MATRIX FILENAMES IN ARRAY

/* discovers and compiles an array of matrix filenames */

void sdcard_list_matrix_files(char * dir, char * name, char * ext) {
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
    Serial.println("[sdcard] calculating: " + String(temppath));
    if (sd.exists(temppath)) {
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
      // clear function names
      memset(matrixData.matrix_function[Mi][Fi], 0, 56);
      strcpy(matrixData.matrix_function[Mi][Fi], "None");
      // clear function values
      matrixData.matrix_function_xyz[Mi][Fi][0] = 0.0;
      matrixData.matrix_function_xyz[Mi][Fi][1] = 0.0;
      matrixData.matrix_function_xyz[Mi][Fi][2] = 0.0;
      // clear port maps
      matrixData.matrix_port_map[0][Mi] = -1;
      matrixData.tmp_matrix_port_map[0][Mi] = -1;
      // clear inverted logic (default is standard not inverted)
      matrixData.matrix_switch_inverted_logic[Mi][Fi] = false;
      // clear timers
      matrixData.matrix_timers[0][Mi] = 0.0;
      // clear enabled
      matrixData.matrix_switch_enabled[0][Mi] = false;
      // clear states
      matrixData.matrix_switch_state[0][Mi] = false;
      matrixData.tmp_matrix_switch_state[0][Mi] = false;
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                            SDCARD: LOAD MATRIX 

/* loads tagged, comma delimited data from a matrix file */

bool sdcard_load_matrix(char * file) {
  
  Serial.println("[sdcard] attempting to load file: " + String(file));
  exfile.flush();
  exfile = sd.open(file); 
  if (exfile) {
    while (exfile.available()) {
      // read line
      sdcardData.SBUFFER = "";
      memset(sdcardData.BUFFER, 0, sizeof(sdcardData.BUFFER));
      sdcardData.SBUFFER = exfile.readStringUntil('\n');
      sdcardData.SBUFFER.toCharArray(sdcardData.BUFFER, sdcardData.SBUFFER.length()+1);
      Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));
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
          Serial.println("[Mi] [PASS] " +String(sdcardData.data_0));
        }
        else {Serial.println("[Mi] [INVALID] " +String(sdcardData.data_0));}
        // matrix function index
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_1, sdcardData.token);
        if (is_all_digits(sdcardData.data_1) == true) {validData.bool_data_1 = true;
          Serial.println("[Fi] [PASS] " +String(sdcardData.data_1));
        }
        else {Serial.println("[Fi] [INVALID] " +String(sdcardData.data_1));}
        // continue if we have valid index numbers
        if ((validData.bool_data_0 == true) && (validData.bool_data_1 == true)) {
          // matrix function name
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_2, sdcardData.token);
          memset(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], 0, sizeof(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]));
          strcpy(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], sdcardData.data_2);
          Serial.println("[Fn] [MATRIX] " +String(matrixData.matrix_function[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]));
          // matrix function data: x
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_3, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_3) == true) {
            matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0] = atol(sdcardData.data_3);
            Serial.println("[X]  [MATRIX] " +String(matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0]));
          }
          else {Serial.println("[X] [INVALID] " + String(sdcardData.data_3));}
          // matrix function data: y
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_4, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_4) == true) {
            matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1] = atol(sdcardData.data_4);
            Serial.println("[Y]  [MATRIX] " +String(matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1]));
          }
          else {Serial.println("[Y] [INVALID] " + String(sdcardData.data_4));}
          // matrix function data: z
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_5, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_5) == true) {
            matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2] = atol(sdcardData.data_5);
            Serial.println("[Z]  [MATRIX] " +String(matrixData.matrix_function_xyz[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2]));
          }
          else {Serial.println("[Z] [INVALID] " + String(sdcardData.data_5));}
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
          Serial.println("[E]  [MATRIX] " +String(matrixData.matrix_switch_enabled[0][atoi(sdcardData.data_0)]));
          }
        else {Serial.println("[E]  [INVALID] " +String(sdcardData.data_6));}
        // port
        sdcardData.token = strtok(NULL, ",");
        // check
        if (is_all_digits_plus_char(sdcardData.data_7, '-') == true) {
          strcpy(sdcardData.data_7, sdcardData.token);
          matrixData.matrix_port_map[0][atoi(sdcardData.data_0)] = atoi(sdcardData.data_7);
          Serial.println("[E]  [MATRIX] " +String(matrixData.matrix_port_map[0][atoi(sdcardData.data_0)]));
          }
        else {Serial.println("[E]  [INVALID] " +String(sdcardData.data_7));}
      }
    }
    // update filename and file path (this has to work for loading on boot and for any following matrix load file calls)
    strcpy(sdcardData.tempmatrixfilepath, file);
    memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
    strcpy(sdcardData.matrix_filepath, sdcardData.tempmatrixfilepath);
    Serial.println("[sdcard] loaded file successfully:   " + String(file));
    Serial.println("[sdcard] sdcardData.matrix_filepath: " + String(sdcardData.matrix_filepath));
    UpdateMatrixFileName(sdcardData.matrix_filepath);
    exfile.close();
    return true;
  }
  // update matrix filepath (clear)
  else {
    exfile.close();
    Serial.println("[sdcard] failed to load file: " + String(file));
    // update filename and file path
    // memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
    // memset(sdcardData.matrix_filename, 0, sizeof(sdcardData.matrix_filename));
    return false;
    }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                            SDCARD: SAVE MATRIX

/* saves tagged, comma delimited data to a matrix file */

bool sdcard_save_matrix(char * file) {

  Serial.println("[sdcard] attempting to save file: " + String(file));
  // exfile.flush();
  exfile = sd.open(file, O_WRITE | O_CREAT);
  Serial.println("[sdcard exfile] " + String(exfile));
  if (exfile) {
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
        // inverted function logic
        memset(sdcardData.tmp, 0 , sizeof(sdcardData.tmp));
        itoa(matrixData.matrix_switch_inverted_logic[Mi][Fi], sdcardData.tmp, 10);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // write line
        Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
        exfile.println(sdcardData.file_data);
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
      Serial.println("[check] " + String(matrixData.matrix_port_map[0][Mi]));
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // write line
      Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
      exfile.println("");
      exfile.println(sdcardData.file_data);
      exfile.println("");
    }
    exfile.close();
    Serial.println("[sdcard] saved file successfully: " + String(file));
    // update filename and file path
    UpdateMatrixFileNameFilePath(file);
    return true;
  }
  else {exfile.close(); Serial.println("[sdcard] failed to save file: " + String(file));
  return false;}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                     SDCARD: DELETE MATRIX FILE

void sdcard_delete_matrix(char * file) {
  if (sd.exists(file)) {
    Serial.println("[sdcard] attempting to delete file: " + String(file));
    // try remove
    sd.remove(file);
    if (!sd.exists(file)) {
      Serial.println("[sdcard] successfully deleted file: " + String(file));
      Serial.println("attempting to remove filename from filenames.");
      // recreate matrix filenames
      sdcard_list_matrix_files(sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
      // zero the matrix
      zero_matrix();
      // update filename and file path
      memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
      memset(sdcardData.matrix_filename, 0, sizeof(sdcardData.matrix_filename));
    }
    else {Serial.println("[sdcard] failed to deleted file: " + String(file));}
  }
  else {Serial.println("[sdcard] file does not exist: " + String(file));}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   TRACK OBJECT

void trackObject(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second, int object_table_i, int object_i) {

  /*
  requires time, location, object table number and object number.
  sets time and location specific values pertaining to an object.
  object will first need to be identified.
  */

  myAstro.setLatLong(latitude, longitude);
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
  siderealObjectData.object_ra = myAstro.getRAdec();
  siderealObjectData.object_dec = myAstro.getDeclinationDec();
  siderealObjectData.object_az = myAstro.getAzimuth();
  siderealObjectData.object_alt = myAstro.getAltitude();
  myAstro.doXRiseSetTimes();
  siderealObjectData.object_r = myAstro.getRiseTime();
  siderealObjectData.object_s = myAstro.getSetTime();
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                IDENTIFY OBJECT

void IdentifyObject(double object_ra, double object_dec) {

  /*
  requires RA and DEC.
  sets object values according to identified object table and identified object number.
  once we have the object number we can track the object if required.
  */

  // -------------------------------------------------------

  myAstroObj.setRAdec(object_ra, object_dec);
  myAstro.doRAdec2AltAz();
  myAstroObj.identifyObject();

  // -------------------------------------------------------

  // scan tables for the object
  switch(myAstroObj.getIdentifiedObjectTable()) {
    case(1):
    siderealObjectData.object_table_i = 0; break; // Star
    case(2):
    siderealObjectData.object_table_i = 1; break; // NGC
    case(3):
    siderealObjectData.object_table_i = 2;  break; //IC
    case(7):
    siderealObjectData.object_table_i = 3;  break; // Other
  }
  // -------------------------------------------------------

  // object tables
  if (myAstroObj.getIdentifiedObjectTable() == 1) {
    // set table name
    memset(siderealObjectData.object_table_name, 0, 56);
    strcpy(siderealObjectData.object_table_name, siderealObjectData.object_table[siderealObjectData.object_table_i]);
    // set object id name
    memset(siderealObjectData.object_name, 0, 56);
    strcpy(siderealObjectData.object_name, myAstroObj.printStarName(myAstroObj.getIdentifiedObjectNumber()));
    // set object id number
    siderealObjectData.object_number = myAstroObj.getIdentifiedObjectNumber();
  }
  // -------------------------------------------------------

  // alternate object tables
  if (myAstroObj.getAltIdentifiedObjectTable()) {
    switch(myAstroObj.getAltIdentifiedObjectTable()) {
      casematrix_indi_h:
      siderealObjectData.object_table_i = 4;  break; // Messier
      case(5):
      siderealObjectData.object_table_i = 5;  break; // Caldwell
      case(6):
      siderealObjectData.object_table_i = 6;  break; // Herschel 400 number
    }
    // set table name
    memset(siderealObjectData.object_table_name, 0, 56);
    strcpy(siderealObjectData.object_table_name, siderealObjectData.object_table[siderealObjectData.object_table_i]);
    // set object id name
    memset(siderealObjectData.object_name, 0, 56);
    strcpy(siderealObjectData.object_name, myAstroObj.printStarName(myAstroObj.getAltIdentifiedObjectNumber()));
    // set object id number
    siderealObjectData.object_number = myAstroObj.getAltIdentifiedObjectNumber();
  }
  // -------------------------------------------------------
}

void trackSun() {
  siderealPlanetData.sun_ra  = myAstro.getRAdec();
  siderealPlanetData.sun_dec = myAstro.getDeclinationDec();
  myAstro.doRAdec2AltAz();
  siderealPlanetData.sun_az  = myAstro.getAzimuth();
  siderealPlanetData.sun_alt = myAstro.getAltitude();
  myAstro.doSunRiseSetTimes();
  siderealPlanetData.sun_r  = myAstro.getSunriseTime();
  siderealPlanetData.sun_s  = myAstro.getSunsetTime();
  // create and ouptput solar tracking information
  if (systemData.output_sun_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$SUN,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.sun_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.sun_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.sun_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.sun_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.sun_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.sun_s + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
}

void trackMoon() {
  siderealPlanetData.moon_ra  = myAstro.getRAdec();
  siderealPlanetData.moon_dec = myAstro.getDeclinationDec();
  // bench("[ra dec]" + String(millis()-t0));
  // t0 = millis();
  myAstro.doRAdec2AltAz();
  // bench("[doRAdec2AltAz]" + String(millis()-t0));
  // t0 = millis();
  siderealPlanetData.moon_az  = myAstro.getAzimuth();
  siderealPlanetData.moon_alt = myAstro.getAltitude();
  // bench("[azalt]" + String(millis()-t0));
  // t0 = millis();
  myAstro.doMoonRiseSetTimes();
  // bench("[doMoonRiseSetTimes]" + String(millis()-t0));
  // t0 = millis();
  siderealPlanetData.moon_r  = myAstro.getMoonriseTime();
  siderealPlanetData.moon_s  = myAstro.getMoonsetTime();
  // bench("[rs]" + String(millis()-t0));
  // t0 = millis();
  siderealPlanetData.moon_p  = myAstro.getMoonPhase();
  // bench("[p]" + String(millis()-t0));
  siderealPlanetData.moon_lum = myAstro.getLunarLuminance();
  // create and ouptput lunar tracking information
  if (systemData.output_moon_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$MOON,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_p + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.moon_lum + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
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
  // create and ouptput tracking information
  if (systemData.output_mercury_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$MERCURY,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_helio_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_helio_ecliptic_long + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_radius_vector + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_distance + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mercury_ecliptic_long + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
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
  // create and ouptput tracking information
  if (systemData.output_venus_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$VENUS,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_helio_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_helio_ecliptic_long + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_radius_vector + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_distance + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.venus_ecliptic_long + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
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
  // create and ouptput tracking information
  if (systemData.output_mars_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$MARS,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_helio_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_helio_ecliptic_long + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_radius_vector + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_distance + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.mars_ecliptic_long + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
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
  // create and ouptput tracking information
  if (systemData.output_jupiter_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$JUPITER,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_helio_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_helio_ecliptic_long + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_radius_vector + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_distance + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.jupiter_ecliptic_long + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
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
  // create and ouptput tracking information
  if (systemData.output_saturn_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$SATURN,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_helio_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_helio_ecliptic_long + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_radius_vector + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_distance + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.saturn_ecliptic_long + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
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
  // create and ouptput tracking information
  if (systemData.output_uranus_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$URANUS,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_helio_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_helio_ecliptic_long + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_radius_vector + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_distance + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.uranus_ecliptic_long + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
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
  // create and ouptput tracking information
  if (systemData.output_neptune_enabled==true) {
    memset(siderealPlanetData.sentence, 0, sizeof(siderealPlanetData.sentence));
    strcat(siderealPlanetData.sentence, "$NEPTUNE,");
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_ra + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_dec + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_az + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_alt + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_r + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_s + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_helio_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_helio_ecliptic_long + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_radius_vector + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_distance + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_ecliptic_lat + String(",")).c_str());
    strcat(siderealPlanetData.sentence, String(siderealPlanetData.neptune_ecliptic_long + String(",")).c_str());
    // append checksum
    createChecksum(siderealPlanetData.sentence);
    strcat(siderealPlanetData.sentence, "*");
    strcat(siderealPlanetData.sentence, SerialLink.checksum);
    Serial.println(siderealPlanetData.sentence);
    // debug(satData.satio_sentence);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   TASK: PLANETARY CALCULATIONS

void trackPlanets() {
  // do planet elements and do sun before doing other plans
  myAstro.doPlanetElements();
  myAstro.doSun();

  // now do other plans
  if (systemData.sidereal_track_sun == true) {trackSun();}
  else {
    siderealPlanetData.sun_ra=NAN;
    siderealPlanetData.sun_dec=NAN;
    siderealPlanetData.sun_az=NAN;
    siderealPlanetData.sun_alt=NAN;
    siderealPlanetData.sun_r=NAN;
    siderealPlanetData.sun_s=NAN;
  }
  if (systemData.sidereal_track_moon == true) {trackMoon();}
  else {
    siderealPlanetData.moon_ra=NAN;
    siderealPlanetData.moon_dec=NAN;
    siderealPlanetData.moon_az=NAN;
    siderealPlanetData.moon_alt=NAN;
    siderealPlanetData.moon_r=NAN;
    siderealPlanetData.moon_s=NAN;
    siderealPlanetData.moon_p=NAN;
    siderealPlanetData.moon_lum=NAN;
  }
  if (systemData.sidereal_track_mercury == true) {trackMercury();}
  else {
    siderealPlanetData.mercury_ra=NAN;
    siderealPlanetData.mercury_dec=NAN;
    siderealPlanetData.mercury_az=NAN;
    siderealPlanetData.mercury_alt=NAN;
    siderealPlanetData.mercury_r=NAN;
    siderealPlanetData.mercury_s=NAN;
    siderealPlanetData.mercury_helio_ecliptic_lat=NAN;
    siderealPlanetData.mercury_helio_ecliptic_long=NAN;
    siderealPlanetData.mercury_radius_vector=NAN;
    siderealPlanetData.mercury_distance=NAN;
    siderealPlanetData.mercury_ecliptic_lat=NAN;
    siderealPlanetData.mercury_ecliptic_long=NAN;
  }
  if (systemData.sidereal_track_venus == true) {trackVenus();}
  else {
    siderealPlanetData.venus_ra=NAN;
    siderealPlanetData.venus_dec=NAN;
    siderealPlanetData.venus_az=NAN;
    siderealPlanetData.venus_alt=NAN;
    siderealPlanetData.venus_r=NAN;
    siderealPlanetData.venus_s=NAN;
    siderealPlanetData.venus_helio_ecliptic_lat=NAN;
    siderealPlanetData.venus_helio_ecliptic_long=NAN;
    siderealPlanetData.venus_radius_vector=NAN;
    siderealPlanetData.venus_distance=NAN;
    siderealPlanetData.venus_ecliptic_lat=NAN;
    siderealPlanetData.venus_ecliptic_long=NAN;
  }
  if (systemData.sidereal_track_mars == true) {trackMars();}
  else {
    siderealPlanetData.mars_ra=NAN;
    siderealPlanetData.mars_dec=NAN;
    siderealPlanetData.mars_az=NAN;
    siderealPlanetData.mars_alt=NAN;
    siderealPlanetData.mars_r=NAN;
    siderealPlanetData.mars_s=NAN;
    siderealPlanetData.mars_helio_ecliptic_lat=NAN;
    siderealPlanetData.mars_helio_ecliptic_long=NAN;
    siderealPlanetData.mars_radius_vector=NAN;
    siderealPlanetData.mars_distance=NAN;
    siderealPlanetData.mars_ecliptic_lat=NAN;
    siderealPlanetData.mars_ecliptic_long=NAN;
  }
  if (systemData.sidereal_track_jupiter == true) {trackJupiter();}
  else {
    siderealPlanetData.jupiter_ra=NAN;
    siderealPlanetData.jupiter_dec=NAN;
    siderealPlanetData.jupiter_az=NAN;
    siderealPlanetData.jupiter_alt=NAN;
    siderealPlanetData.jupiter_r=NAN;
    siderealPlanetData.jupiter_s=NAN;
    siderealPlanetData.jupiter_helio_ecliptic_lat=NAN;
    siderealPlanetData.jupiter_helio_ecliptic_long=NAN;
    siderealPlanetData.jupiter_radius_vector=NAN;
    siderealPlanetData.jupiter_distance=NAN;
    siderealPlanetData.jupiter_ecliptic_lat=NAN;
    siderealPlanetData.jupiter_ecliptic_long=NAN;
  }
  if (systemData.sidereal_track_saturn == true) {trackSaturn();}
  else {
    siderealPlanetData.saturn_ra=NAN;
    siderealPlanetData.saturn_dec=NAN;
    siderealPlanetData.saturn_az=NAN;
    siderealPlanetData.saturn_alt=NAN;
    siderealPlanetData.saturn_r=NAN;
    siderealPlanetData.saturn_s=NAN;
    siderealPlanetData.saturn_helio_ecliptic_lat=NAN;
    siderealPlanetData.saturn_helio_ecliptic_long=NAN;
    siderealPlanetData.saturn_radius_vector=NAN;
    siderealPlanetData.saturn_distance=NAN;
    siderealPlanetData.saturn_ecliptic_lat=NAN;
    siderealPlanetData.saturn_ecliptic_long=NAN;
  }
  if (systemData.sidereal_track_uranus == true) {trackUranus();}
  else {
    siderealPlanetData.uranus_ra=NAN;
    siderealPlanetData.uranus_dec=NAN;
    siderealPlanetData.uranus_az=NAN;
    siderealPlanetData.uranus_alt=NAN;
    siderealPlanetData.uranus_r=NAN;
    siderealPlanetData.uranus_s=NAN;
    siderealPlanetData.uranus_helio_ecliptic_lat=NAN;
    siderealPlanetData.uranus_helio_ecliptic_long=NAN;
    siderealPlanetData.uranus_radius_vector=NAN;
    siderealPlanetData.uranus_distance=NAN;
    siderealPlanetData.uranus_ecliptic_lat=NAN;
    siderealPlanetData.uranus_ecliptic_long=NAN;
  }
  if (systemData.sidereal_track_neptune == true) {trackNeptune();}
  else {
    siderealPlanetData.neptune_ra=NAN;
    siderealPlanetData.neptune_dec=NAN;
    siderealPlanetData.neptune_az=NAN;
    siderealPlanetData.neptune_alt=NAN;
    siderealPlanetData.neptune_r=NAN;
    siderealPlanetData.neptune_s=NAN;
    siderealPlanetData.neptune_helio_ecliptic_lat=NAN;
    siderealPlanetData.neptune_helio_ecliptic_long=NAN;
    siderealPlanetData.neptune_radius_vector=NAN;
    siderealPlanetData.neptune_distance=NAN;
    siderealPlanetData.neptune_ecliptic_lat=NAN;
    siderealPlanetData.neptune_ecliptic_long=NAN;
  }
}

void setTrackPlanets() {
  
  // int t0 = millis();
  myAstro.setLatLong(satData.degrees_latitude, satData.degrees_longitude);
  // bench("[setLatLong]" + String(millis()-t0));

  myAstro.rejectDST();

  // t0 = millis();
  myAstro.setGMTdate(rtc.now().year(), rtc.now().month(), rtc.now().day());
  // bench("[setGMTdate]" + String(millis()-t0));

  // t0 = millis();
  myAstro.setGMTtime(rtc.now().hour(), rtc.now().minute(), rtc.now().second());
  // bench("[setGMTtime]" + String(millis()-t0));

  // t0 = millis();
  myAstro.setLocalTime(rtc.now().hour(), rtc.now().minute(), rtc.now().second());
  // bench("[setLocalTime]" + String(millis()-t0));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   MATRIX FUNCTIONS: EXPRESSIONS

/*
matrix switch requires all checks to return true for a matrix to be active, therefore checks can be inverted as required, to
return true when otherwise a check would return false, which allows more flexibility.
*/

// calculate if n0 in (+- range/2) of n1
bool in_range_check_true(double n0, double n1, double r) {
  // debug(
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
  // debug(
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

bool in_square_range_check_true(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r) == true) {
    if (in_range_check_true(y0, y1, r) == true) {return true;} else return false;}
  else {return false;}
}

bool in_square_range_check_false(double x0, double x1, double y0, double y1, double r) {
  if (in_range_check_true(x0, x1, r) == true) {
    if (in_range_check_true(y0, y1, r) == true) {return false;} else return true;}
  else {return true;}
}

bool check_over_true(double n0, double n1) {
  // debug("check_over_true: n0 " + String(n0) + " > n1 " + String(n1));
  if (n0 > n1) {return true;}
  else {return false;}
}

bool check_over_false(double n0, double n1) {
  // debug("check_over_false: n0 " + String(n0) + " > n1 " + String(n1));
  if (n0 > n1) {return false;}
  else {return true;}
}

bool check_under_true(double n0, double n1) {
  // debug("check_under_true: n0 " + String(n0) + " < n1 " + String(n1));
  if (n0 < n1) {return true;}
  else {return false;}
}

bool check_under_false(double n0, double n1) {
  // debug("check_under_false: n0 " + String(n0) + " < n1 " + String(n1));
  if (n0 < n1) {return false;}
  else {return true;}
}

bool check_equal_true(double n0, double n1) {
  // debug("check_equal_true: n0 " + String(n0) + " == n1 " + String(n1));
  if (n0 == n1) {return true;}
  else {return false;}
}

bool check_equal_false(double n0, double n1) {
  // debug("check_equal_false: n0 " + String(n0) + " == n1 " + String(n1));
  if (n0 != n1) {return true;}
  else {return false;}
}

bool check_ge_and_le_true(double n0, double n1, double n2) {
  // debug(
  //   "check_ge_and_le_true: n0 " +
  //   String(n0) +
  //   " >= n1 " +
  //   String(n1) +
  //   " && n0 " +
  //   String(n0) +
  //   " <= " +
    // String(n2));
  if ((n0 >= n1) && (n0 <= n2)) {return true;}
  else {return false;}
}

bool check_ge_and_le_false(double n0, double n1, double n2) {
  // debug(
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
  // debug("check_strncmp_true: c0 " + String(c0) + " == c1 " + String(c1) + " (n=" + String(n) + ")");
  if (strncmp(c0, c1, n) == 0) {return true;}
  else {return false;}
}

bool check_strncmp_false(char * c0, char * c1, int n) {
  // debug("check_strncmp_false: c0 " + String(c0) + " == c1 " + String(c1) + " (n=" + String(n) + ")");
  if (strncmp(c0, c1, n) == 0) {return false;}
  else {return true;}
}

bool check_bool_true(bool _bool) {
  // debug("check_bool_true: " + String(_bool));
  if (_bool == true) {return true;} else {return false;}
}

bool check_bool_false(bool _bool) {
  // debug("check_bool_false: " + String(_bool));
  if (_bool == false) {return true;} else {return false;}
}

bool SecondsTimer(double n0, double n1, int Mi) {

  /*

  seconds accumulated by an isr alarm. this does not use satellite data. 
  
  x (n0): off interval
  y (n1): on interval (should not exceed off interval)

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
//                                                                                                                 MATRIX: SWITCH

void matrixSwitch() {

  /*
  compound condition checks, each resulting in zero/one at the final_bool.
  */

  // iterate through matrices
  for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
    // debug("[Mi] " + String(Mi) + " [E] " + String(matrixData.matrix_switch_enabled[0][Mi]));
    if (matrixData.matrix_switch_enabled[0][Mi] == 1) {

      /*
      temporary switch must be zero each time
      */
      bool tmp_matrix[matrixData.max_matrix_functions] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      int count_none_function = 0;

      // iterate over each function name in the current matrix
      for (int Fi = 0; Fi < matrixData.max_matrix_functions; Fi++) {

        // uncomment to debug
        // debug("[Mi] " + String(Mi));
        // debug("[Fi] " + String(Fi));
        // debug("[matrixData.matrix_function[Mi][Fi]] " + String(matrixData.matrix_function[Mi][Fi]));

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
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(matrixData.matrix_switch_state[0][(int)matrixData.matrix_function_xyz[Mi][Fi][0]], 1);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(matrixData.matrix_switch_state[0][(int)matrixData.matrix_function_xyz[Mi][Fi][0]], 1);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                              TIME DATA

        /* allows modulation with an approximate resolution of 1 second: while other stacked logic true, switch state is true/false every Xsec for Ysec */
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SecondsTimer") == 0) {
          tmp_matrix[Fi] = SecondsTimer(matrixData.matrix_function_xyz[Mi][Fi][0],
          matrixData.matrix_function_xyz[Mi][Fi][1], Mi);
        }

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

        // over
        if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_true(satData.degrees_latitude,
              matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // under
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(satData.degrees_longitude,
              matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // equal
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        // range
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLonRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_range_check_true(satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][0], matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_range_check_false(satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][0], matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        // ranges
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "DegLatLonRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = in_square_range_check_true(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][1],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = in_square_range_check_false(satData.degrees_latitude,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            satData.degrees_longitude,
            matrixData.matrix_function_xyz[Mi][Fi][1],
            matrixData.matrix_function_xyz[Mi][Fi][2]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                                  GNGGA


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

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LFlagGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LFlagGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LFlagGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "LFlagGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.line_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "INSGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.ins),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.ins),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "INSGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.ins),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.ins),
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

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "INSGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.ins),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.ins),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RSFlagGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RSFlagGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RSFlagGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "RSFlagGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.run_state_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SFlagGPATTOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SFlagGPATTUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SFlagGPATTEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SFlagGPATTRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(atol(gpattData.static_flag),
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                     SIDEREAL TIME: SUN
        
        // sun azimuth:
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SunAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.sun_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // sun altitude:
        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SunAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.sun_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                                 SIDEREAL TIME: MOON

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.moon_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.moon_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonPhaseOver") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true((int)siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_false((int)siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonPhaseUnder") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true((int)siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_false((int)siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonPhaseEqual") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true((int)siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_false((int)siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MoonPhaseRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.moon_p,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                              SIDEREAL TIME: MERCURY

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercuryAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mercury_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MercuryAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mercury_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                                SIDEREAL TIME: VENUS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.venus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "VenusAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.venus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                                 SIDEREAL TIME: MARS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mars_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "MarsAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.mars_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                              SIDEREAL TIME: JUPITER

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.jupiter_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "JupiterAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.jupiter_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                               SIDEREAL TIME: SATURN

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.saturn_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "SaturnAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.saturn_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                               SIDEREAL TIME: URANUS

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.uranus_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "UranusAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.uranus_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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

        // -------------------------------------------------------------------------------------------------------------------
        //                                                                                              SIDEREAL TIME: NEPTUNE

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneAzRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_az,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.neptune_az,
              matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "NeptuneAltRange") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(siderealPlanetData.neptune_alt,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
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


        /* run the following logic checks providing the sensor data has already been collected (sensor data must be called manually before calling matrix switch) */

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
        //                                                                                                               SENSOR 0

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor0Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor0Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor0Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor0Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_0,
            matrixData.matrix_function_xyz[Mi][Fi][0],
            matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_0,
              matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 1

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor1Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor1Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor1Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor1Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_1,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 2

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor2Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor2Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor2Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor2Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_2,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 3

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor3Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor3Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor3Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor3Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_3,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 4

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor4Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor4Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor4Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor4Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_4,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 5

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor5Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor5Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor5Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor5Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_5,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 6

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor6Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor6Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor6Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor6Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_6,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 7

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor7Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor7Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor7Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor7Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_7,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 8

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor8Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor8Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor8Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor8Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_8,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 9

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor9Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor9Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor9Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor9Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_9,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 10

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor10Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor10Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor10Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor10Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_10,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 11

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor11Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor11Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor11Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor11Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_11,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 12

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor12Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor12Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor12Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor12Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_12,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 13

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor13Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor13Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor13Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor13Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_13,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 14

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor14Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor14Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor14Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor14Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_14,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }

        // ----------------------------------------------------------------------------------------------------------------------
        //                                                                                                               SENSOR 15

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor15Over") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_over_true(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_over_false(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor15Under") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_under_true(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_under_false(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor15Equal") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_equal_true(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_equal_false(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0]);
          }
        }

        else if (strcmp(matrixData.matrix_function[Mi][Fi], "Sensor15Range") == 0) {
          if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==false) {
            tmp_matrix[Fi] = check_ge_and_le_true(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
          else if (matrixData.matrix_switch_inverted_logic[Mi][Fi]==true) {
            tmp_matrix[Fi] = check_ge_and_le_false(sensorData.sensor_15,
            matrixData.matrix_function_xyz[Mi][Fi][0],
              matrixData.matrix_function_xyz[Mi][Fi][1]);
          }
        }
        
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
        if (systemData.debug==true) {
          for (int FC = 0; FC < matrixData.max_matrix_functions-1; FC++) {
            Serial.println ("[tmp_matrix[FC]] " + String(tmp_matrix[FC])); if (tmp_matrix[FC] == 0) {final_bool = false;}
          }
        }

        else {
          for (int FC = 0; FC < matrixData.max_matrix_functions-1; FC++) {if (tmp_matrix[FC] == 0) {final_bool = false; break;}}
        }

        /*
        WARNING: why do you think you can trust the data you are receiving?
                 once you plug something into this, the 'satellites' are in control unless you have a way to override.

                 critical systems: arduino is neither medical nor military grade.
        */

        // debug (same as line below but with output)
        if (systemData.debug==true) {
          if (final_bool == false) {Serial.println("[matrix " + String(Mi) + "] inactive"); matrixData.matrix_switch_state[0][Mi] = 0;}
          else if (final_bool == true) {Serial.println("[matrix " + String(Mi) + "] active"); matrixData.matrix_switch_state[0][Mi] = 1;}
        }
        /* a short call to the port controller each iteration may be made here */
        else {
          if (final_bool == false) {matrixData.matrix_switch_state[0][Mi] = 0;}
          else if (final_bool == true) {matrixData.matrix_switch_state[0][Mi] = 1;}
        }
      }
      // else {debug("[matrix " + String(Mi) + "] WARNING: Matrix checks are enabled for an non configured matrix!");}
    }
    // handle Mi's that are disbaled.
    else {matrixData.matrix_switch_state[0][Mi] = 0;}
  }

  if (systemData.output_matrix_enabled == true) {

    // start building matrix sentence
    memset(matrixData.matrix_sentence, 0, sizeof(matrixData.matrix_sentence));
    strcpy(matrixData.matrix_sentence, "$MATRIX,");

    // append port mapping data
    for (int i=0; i < matrixData.max_matrices; i++) {
      strcat(matrixData.matrix_sentence, String(String(matrixData.matrix_port_map[0][i])+",").c_str());
      }
    
    // append matrix switch state data
    for (int i=0; i < matrixData.max_matrices; i++) {
      strcat(matrixData.matrix_sentence, String(String(matrixData.matrix_switch_state[0][i])+",").c_str());
    }

    // append checksum
    createChecksum(matrixData.matrix_sentence);
    strcat(matrixData.matrix_sentence, "*");
    strcat(matrixData.matrix_sentence, SerialLink.checksum);

    // serial output: switch states.
    Serial.println(matrixData.matrix_sentence);
  }
  debug(matrixData.matrix_sentence);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 STATS COUNTERS

void CountMatrixEnabled() {
  matrixData.matrix_enabled_i = 0;
  matrixData.matrix_disabled_i = 0;
  for (int Mi = 0; Mi < matrixData.max_matrices; Mi++) {
    if (matrixData.matrix_switch_enabled[0][Mi] == 1) {matrixData.matrix_enabled_i++;} else {matrixData.matrix_disabled_i++;}
  }
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
  if (menu_page==page_home) {menuHome.up();}
  else if (menu_page==page_main_menu) {menuMain.up();}
  else if (menu_page==page_matrix_logic_main) {
    if (menu_column_selection==0) {menuMatrixSwitchSelect.up();}
    if (menu_column_selection==1) {}
    if (menu_column_selection==2) {}
    if (menu_column_selection==4) {menuMatrixFunctionSelect.up();}
  }
  else if (menu_page==page_input_data) {}
  else if (menu_page==page_matrix_logic_select_setup) {menuMatrixConfigureFunction.up();}
  else if (menu_page==page_matrix_logic_setup_function) {menuMatrixSetFunctionName.up();}
  else if (menu_page==page_file_main) {menuFile.up();}
  else if (menu_page==page_file_save_matrix) {menuMatrixFilepath.up();}
  else if (menu_page==page_file_load_matrix) {menuMatrixFilepath.up();}
  else if (menu_page==page_file_delete_matrix) {menuMatrixFilepath.up();}
  else if (menu_page==page_gps_main) {menuGPS.up();}
  else if (menu_page==page_serial_main) {menuSerial.up();}
  else if (menu_page==page_universe_main) {menuUniverse.up();}
  else if (menu_page==page_display_main) {menuDisplay.up();}
  else if (menu_page==page_system_main) {menuSystem.up();}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MENU DOWN

void menuDown() {
  if (menu_page==page_home) {menuHome.down();}
  else if (menu_page==page_main_menu) {menuMain.down();}
  else if (menu_page==page_matrix_logic_main) {
    if (menu_column_selection==0) {menuMatrixSwitchSelect.down();}
    if (menu_column_selection==1) {}
    if (menu_column_selection==2) {}
    if (menu_column_selection==4) {menuMatrixFunctionSelect.down();}
  }
  else if (menu_page==page_input_data) {}
  else if (menu_page==page_matrix_logic_select_setup) {menuMatrixConfigureFunction.down();}
  else if (menu_page==page_matrix_logic_setup_function) {menuMatrixSetFunctionName.down();}
  else if (menu_page==page_file_main) {menuFile.down();}
  else if (menu_page==page_file_save_matrix) {menuMatrixFilepath.down();}
  else if (menu_page==page_file_load_matrix) {menuMatrixFilepath.down();}
  else if (menu_page==page_file_delete_matrix) {menuMatrixFilepath.down();}
  else if (menu_page==page_gps_main) {menuGPS.down();}
  else if (menu_page==page_serial_main) {menuSerial.down();}
  else if (menu_page==page_universe_main) {menuUniverse.down();}
  else if (menu_page==page_display_main) {menuDisplay.down();}
  else if (menu_page==page_system_main) {menuSystem.down();}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     MENU RIGHT

void menuRight() {
  if (menu_page==page_home) {}
  else if (menu_page==page_main_menu) {}
  else if (menu_page==page_matrix_logic_main) {menu_column_selection++; if (menu_column_selection>4) {menu_column_selection=0;}}

  // debug("[menu_column_selection] " + String(menu_column_selection));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MENU LEFT

void menuLeft() {
  if (menu_page==page_home) {}
  else if (menu_page==page_main_menu) {}
  else if (menu_page==page_matrix_logic_main) {menu_column_selection--; if (menu_column_selection<0) {menu_column_selection=4;}}
  // debug("[menu_column_selection] " + String(menu_column_selection));
}

void menuBack() {
  /* specify explicity which page to go from each given page */
  // debug("[menuBack] menupage 0: " + String(menu_page));
  if (menu_page==page_main_menu) {menu_page=page_home;}
  else if (menu_page==page_matrix_logic_main) {menu_page=page_main_menu;}
  else if (menu_page==page_input_data) {
    // debug("[menuBack] enter_digits_key: " + String(enter_digits_key));
    // enter port
    if (enter_digits_key == 1) {menu_page=page_matrix_logic_main;}
    // enter function x, enter function y, enter function z
    else if ((enter_digits_key == 2) || (enter_digits_key == 3) || (enter_digits_key == 4)) {menu_page=page_matrix_logic_select_setup;}
  }
  else if (menu_page==page_matrix_logic_select_setup) {menu_page=page_matrix_logic_main;}
  else if (menu_page==page_matrix_logic_setup_function) {menu_page=page_matrix_logic_select_setup;}
  else if (menu_page==page_overview_matrix_switching) {menu_page=page_main_menu;}
  else if (menu_page==page_file_main) {menu_page=page_main_menu;}
  else if (menu_page==page_file_save_matrix) {menu_page=page_file_main;}
  else if (menu_page==page_file_load_matrix) {menu_page=page_file_main;}
  else if (menu_page==page_file_delete_matrix) {menu_page=page_file_main;}
  else if (menu_page==page_gps_main) {menu_page=page_main_menu;}
  else if (menu_page==page_serial_main) {menu_page=page_main_menu;}
  else if (menu_page==page_universe_main) {menu_page=page_main_menu;}
  else if (menu_page==page_display_main) {menu_page=page_main_menu;}
  else if (menu_page==page_system_main) {menu_page=page_main_menu;}
  else if (menu_page==page_CD74HC4067_main) {menu_page=page_main_menu;}
  else if (menu_page==page_gps_view_gngga) {menu_page=page_gps_main;}
  else if (menu_page==page_gps_view_gnrmc) {menu_page=page_gps_main;}
  else if (menu_page==page_gps_view_gpatt) {menu_page=page_gps_main;}
  else if (menu_page==page_gps_view_satio) {menu_page=page_gps_main;}

  else if (menu_page==page_universe_view_sun) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_moon) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_mercury) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_venus) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_mars) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_jupiter) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_saturn) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_uranus) {menu_page=page_universe_main;}
  else if (menu_page==page_universe_view_neptune) {menu_page=page_universe_main;}

  // debug("[menuBack] menupage 1: " + String(menu_page));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     MENU ENTER

void menuEnter() {

  // ----------------------------------------------------------------

  // home page
  if (menu_page==page_home) {

    // go to main menu
    if (menuHome.selection()==0) {menu_page=page_main_menu;}
  }

  // ----------------------------------------------------------------

  // main menu
  else if (menu_page==page_main_menu) {

    // go to matrix menu
    if (menuMain.selection()==0) {
      menu_page=page_matrix_logic_main;
    }

    // go to matrix menu
    if (menuMain.selection()==1) {
      menu_page=page_overview_matrix_switching;
    }

    // go to file menu
    else if (menuMain.selection()==2) {
      menu_page=page_file_main;
    }

    // go to gps menu
    else if (menuMain.selection()==3) {
      menu_page=page_gps_main;
    }

    // go to serial menu
    else if (menuMain.selection()==4) {
      menu_page=page_serial_main;
    }

    // go to system menu
    else if (menuMain.selection()==5) {
      menu_page=page_system_main;
    }

    // go to universe menu
    else if (menuMain.selection()==6) {
      menu_page=page_universe_main;
    }

    // go to display menu
    else if (menuMain.selection()==7) {
      menu_page=page_display_main;
    }

    // go to CD74HC4067 menu
    else if (menuMain.selection()==8) {
      menu_page=page_CD74HC4067_main;
    }

  }

  // ----------------------------------------------------------------

  // matrix switch configuration
  else if (menu_page==page_matrix_logic_main) {

    // go to set port page
    if (menu_column_selection==1) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 1;
      menu_page=page_input_data;
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
      menu_page=page_matrix_logic_select_setup;
    }
  }

  // ----------------------------------------------------------------

  // set digits
  else if (menu_page==page_input_data) {
    allow_input_data=false;
    if (enter_digits_key==1) {matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]=atoi(input_data); menu_page=page_matrix_logic_main;}
    else if (enter_digits_key==2) {matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]=atoi(input_data); menu_page=page_matrix_logic_select_setup;}
    else if (enter_digits_key==3) {matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]=atoi(input_data); menu_page=page_matrix_logic_select_setup;}
    else if (enter_digits_key==4) {matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]=atoi(input_data); menu_page=page_matrix_logic_select_setup;}
    enter_digits_key = -1;
  }

  // matrix switch select function name, x, y, or z
  else if (menu_page==page_matrix_logic_select_setup) {
    if (menuMatrixConfigureFunction.selection()==0) {menu_page=page_matrix_logic_setup_function;}

    // go to set function value x page
    if (menuMatrixConfigureFunction.selection()==1) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 2;
      menu_page=page_input_data;
    }
    // go to set function value y page
    else if (menuMatrixConfigureFunction.selection()==2) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 3;
      menu_page=page_input_data;
    }
    // go to set function value z page
    else if (menuMatrixConfigureFunction.selection()==3) {
      memset(input_data, 0, sizeof(input_data));
      allow_input_data=true;
      enter_digits_key = 4;
      menu_page=page_input_data;
    }
    // set expression
    else if (menuMatrixConfigureFunction.selection()==4) {

      // iterate over expression
      matrixData.i_expression++;
      if (matrixData.i_expression > 4) {matrixData.i_expression=0;}
      // debug("[expression]" + String(matrixData.expression[matrixData.i_expression]));

      // put current str in temp
      memset(matrixData.temp, 0, sizeof(matrixData.temp));
      strcpy(matrixData.temp, matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]);
      
      // remove expression
      matrixData.tempStr = String(matrixData.temp);
      matrixData.tempStr.replace("Under", "");
      matrixData.tempStr.replace("Over", "");
      matrixData.tempStr.replace("Equal", "");
      matrixData.tempStr.replace("Range", "");
      // debug("[temp 0] " + matrixData.tempStr);

      // concatinate base function name with expression
      matrixData.tempStr = matrixData.tempStr + matrixData.expression[matrixData.i_expression];
      // debug("[temp 1] " + matrixData.tempStr);

      // copy new name into matrix
      memset(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()], 0, sizeof(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]));
      strcpy(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()], matrixData.tempStr.c_str());
    }
  }

  // ----------------------------------------------------------------

  // matrix switch set function name
  else if (menu_page==page_matrix_logic_setup_function) {
    memset(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()], 0, sizeof(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]));
    strcpy(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()], matrixData.matrix_function_names[menuMatrixSetFunctionName.selection()]);
    menu_page=page_matrix_logic_select_setup;
  }

  // ----------------------------------------------------------------

  // file menu
  else if (menu_page==page_file_main) {

    // new matrix
    if (menuFile.selection()==0) {
      // disable and turn off all matrix switches
      setAllMatrixSwitchesEnabledFalse();
      setAllMatrixSwitchesStateFalse();
      // zero the matrix and clear current matrix file path
      zero_matrix();
      // update filename and file path
      memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
      memset(sdcardData.matrix_filename, 0, sizeof(sdcardData.matrix_filename));
    }

    // ----------------------------------------------------------------

    // goto save matrix page
    else if (menuFile.selection()==1) {

      /* create list of matrix filespaths and go to save page */

      // END SPI DEVICE
      endSPIDevice(SSD1351_CS);

      // SDCARD
      beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
      sdcard_list_matrix_files(sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
      sd.end();
      endSPIDevice(SD_CS);

      // BEGIN SPI DEVICE
      beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
      display.begin();

      // GO TO
      menu_page=page_file_save_matrix;
    }

    // ----------------------------------------------------------------

    // goto load matrix page
    else if (menuFile.selection()==2) {

      /* create list of matrix filespaths and go to load page */

      // END SPI DEVICE
      endSPIDevice(SSD1351_CS);

      // SDCARD
      beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
      sdcard_list_matrix_files(sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
      sd.end();
      endSPIDevice(SD_CS);

      // BEGIN SPI DEVICE
      beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
      display.begin();

      // GO TO
      menu_page=page_file_load_matrix;
    }

    // ----------------------------------------------------------------

    // goto delete matrix page
    else if (menuFile.selection()==3) {

      /* create list of matrix filespaths and go to delete page */

      // END SPI DEVICE
      endSPIDevice(SSD1351_CS);

      // SDCARD
      beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
      sdcard_list_matrix_files(sdcardData.system_dirs[0], sdcardData.matrix_fname, sdcardData.save_ext);
      sd.end();
      endSPIDevice(SD_CS);

      // BEGIN SPI DEVICE
      beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
      display.begin();

      // GO TO
      menu_page=page_file_delete_matrix;
    }

    // ----------------------------------------------------------------

    // save system settings
    else if (menuFile.selection()==4) {

      // GO TO
      menu_page=page_save_system_config_indicator;
      UIIndicators();

      // END SPI DEVICE
      endSPIDevice(SSD1351_CS);

      // SDCARD
      beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
      sdcard_save_system_configuration(sdcardData.sysconf);
      sd.end();
      endSPIDevice(SD_CS);

      // BEGIN SPI DEVICE
      beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
      display.begin();

      // GO TO
      menu_page=page_file_main;
    }

    // ----------------------------------------------------------------

    // restore default system settings
    else if (menuFile.selection()==5) {

      // GO TO
      menu_page=page_restore_default_matrix_indicator;
      UIIndicators();

      // END SPI DEVICE
      endSPIDevice(SSD1351_CS);

      // SDCARD
      beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
      // ToDo:
      sd.end();
      endSPIDevice(SD_CS);

      // BEGIN SPI DEVICE
      beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS);

      // GO TO
      menu_page=page_file_main;
    }

    // ----------------------------------------------------------------
  }

  // ----------------------------------------------------------------

  // save matrix menu
  else if (menu_page==page_file_save_matrix) {
    // generate filename according to selection index
    memset(sdcardData.newfilename, 0, sizeof(sdcardData.newfilename));
    strcpy(sdcardData.newfilename, "/MATRIX/M_");
    memset(sdcardData.tmp, 0, sizeof(sdcardData.tmp));
    itoa(menuMatrixFilepath.selection(), sdcardData.tmp, 10);
    strcat(sdcardData.newfilename, sdcardData.tmp);
    strcat(sdcardData.newfilename, ".SAVE");

    // GO TO
    menu_page=page_save_matrix_file_indicator;
    UIIndicators();

    // END SPI DEVICE
    endSPIDevice(SSD1351_CS);

    // SDCARD
    beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    sdcard_save_matrix(sdcardData.newfilename);
    sd.end();
    endSPIDevice(SD_CS);

    // BEGIN SPI DEVICE
    beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
    display.begin();

    // GO TO
    menu_page=page_file_main;
  }

  // ----------------------------------------------------------------

  // load matrix menu
  else if (menu_page==page_file_load_matrix) {
    // handle empty slots
    if (!strcmp(sdcardData.matrix_filenames[menuMatrixFilepath.selection()], "EMPTY")==0) {
      // generate filename according to selection index
      memset(sdcardData.newfilename, 0, sizeof(sdcardData.newfilename));
      strcpy(sdcardData.newfilename, "/MATRIX/M_");
      memset(sdcardData.tmp, 0, sizeof(sdcardData.tmp));
      itoa(menuMatrixFilepath.selection(), sdcardData.tmp, 10);
      strcat(sdcardData.newfilename, sdcardData.tmp);
      strcat(sdcardData.newfilename, ".SAVE");

      // GO TO
      menu_page=page_load_matrix_file_indicator;
      UIIndicators();

      // END SPI DEVICE
      endSPIDevice(SSD1351_CS);

      // SDCARD
      beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
      sdcard_load_matrix(sdcardData.newfilename);
      sd.end();
      endSPIDevice(SD_CS);

      // BEGIN SPI DEVICE
      beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
      display.begin();
    }
    // GO TO
    menu_page=page_file_main;
  }

  // ----------------------------------------------------------------

  // delete matrix menu
  else if (menu_page==page_file_delete_matrix) {
    // handle empty slots
    if (!strcmp(sdcardData.matrix_filenames[menuMatrixFilepath.selection()], "EMPTY")==0) {
      // generate filename according to selection index
      memset(sdcardData.newfilename, 0, sizeof(sdcardData.newfilename));
      strcpy(sdcardData.newfilename, "/MATRIX/M_");
      memset(sdcardData.tmp, 0, sizeof(sdcardData.tmp));
      itoa(menuMatrixFilepath.selection(), sdcardData.tmp, 10);
      strcat(sdcardData.newfilename, sdcardData.tmp);
      strcat(sdcardData.newfilename, ".SAVE");
      
      // GO TO
      menu_page=page_delete_matrix_file_indicator;
      UIIndicators();

      // END SPI DEVICE
      endSPIDevice(SSD1351_CS);

      // SDCARD
      beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
      sdcard_delete_matrix(sdcardData.newfilename);
      sd.end();
      endSPIDevice(SD_CS);

      // BEGIN SPI DEVICE
      beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
      display.begin();
    }
    // GO TO
    menu_page=page_file_main;
  }

  // ----------------------------------------------------------------

  // gps page
  else if (menu_page==page_gps_main) {
    if (menuGPS.selection()==0) {systemData.satio_enabled^=true;}
    else if (menuGPS.selection()==1) {systemData.gngga_enabled^=true;}
    else if (menuGPS.selection()==2) {systemData.gnrmc_enabled^=true;}
    else if (menuGPS.selection()==3) {systemData.gpatt_enabled^=true;}
    else if (menuGPS.selection()==4) {
      if (strcmp(satData.coordinate_conversion_mode, "GNGGA")==0) {
        memset(satData.coordinate_conversion_mode, 0, sizeof(satData.coordinate_conversion_mode));
        strcpy(satData.coordinate_conversion_mode, "GNRMC");
      }
      else if (strcmp(satData.coordinate_conversion_mode, "GNRMC")==0) {
        memset(satData.coordinate_conversion_mode, 0, sizeof(satData.coordinate_conversion_mode));
        strcpy(satData.coordinate_conversion_mode, "GNGGA");
      }
    }
    else if (menuGPS.selection()==5) {menu_page=page_gps_view_gngga;}
    else if (menuGPS.selection()==6) {menu_page=page_gps_view_gnrmc;}
    else if (menuGPS.selection()==7) {menu_page=page_gps_view_gpatt;}
    else if (menuGPS.selection()==8) {menu_page=page_gps_view_satio;}
  }

  // ----------------------------------------------------------------

  // serial page
  else if (menu_page==page_serial_main) {
    if (menuSerial.selection()==0) {systemData.output_satio_enabled^=true;}
    else if (menuSerial.selection()==1) {systemData.output_gngga_enabled^=true;}
    else if (menuSerial.selection()==2) {systemData.output_gnrmc_enabled^=true;}
    else if (menuSerial.selection()==3) {systemData.output_gpatt_enabled^=true;}
    else if (menuSerial.selection()==4) {systemData.output_matrix_enabled^=true;}
    else if (menuSerial.selection()==5) {systemData.output_sensors_enabled^=true;}
    else if (menuSerial.selection()==6) {systemData.output_sun_enabled^=true;}
    else if (menuSerial.selection()==7) {systemData.output_moon_enabled^=true;}
    else if (menuSerial.selection()==8) {systemData.output_mercury_enabled^=true;}
    else if (menuSerial.selection()==9) {systemData.output_venus_enabled^=true;}
    else if (menuSerial.selection()==10) {systemData.output_mars_enabled^=true;}
    else if (menuSerial.selection()==11) {systemData.output_jupiter_enabled^=true;}
    else if (menuSerial.selection()==12) {systemData.output_saturn_enabled^=true;}
    else if (menuSerial.selection()==13) {systemData.output_uranus_enabled^=true;}
    else if (menuSerial.selection()==14) {systemData.output_neptune_enabled^=true;}
    else if (menuSerial.selection()==15) {systemData.debug^=true;}
  }

  // ----------------------------------------------------------------

  // universe page
  else if (menu_page==page_universe_main) {
    if (menuUniverse.selection()==0) {systemData.sidereal_track_sun^=true;}
    else if (menuUniverse.selection()==1) {systemData.sidereal_track_mercury^=true;}
    else if (menuUniverse.selection()==2) {systemData.sidereal_track_moon^=true;}
    else if (menuUniverse.selection()==3) {systemData.sidereal_track_venus^=true;}
    else if (menuUniverse.selection()==4) {systemData.sidereal_track_mars^=true;}
    else if (menuUniverse.selection()==5) {systemData.sidereal_track_jupiter^=true;}
    else if (menuUniverse.selection()==6) {systemData.sidereal_track_saturn^=true;}
    else if (menuUniverse.selection()==7) {systemData.sidereal_track_uranus^=true;}
    else if (menuUniverse.selection()==8) {systemData.sidereal_track_neptune^=true;}
    else if (menuUniverse.selection()==9) {menu_page=page_universe_view_sun;}
    else if (menuUniverse.selection()==10) {menu_page=page_universe_view_moon;}
    else if (menuUniverse.selection()==11) {menu_page=page_universe_view_mercury;}
    else if (menuUniverse.selection()==12) {menu_page=page_universe_view_venus;}
    else if (menuUniverse.selection()==13) {menu_page=page_universe_view_mars;}
    else if (menuUniverse.selection()==14) {menu_page=page_universe_view_jupiter;}
    else if (menuUniverse.selection()==15) {menu_page=page_universe_view_saturn;}
    else if (menuUniverse.selection()==16) {menu_page=page_universe_view_uranus;}
    else if (menuUniverse.selection()==17) {menu_page=page_universe_view_neptune;}
  }

  // ----------------------------------------------------------------

  // dispaly page
  else if (menu_page==page_display_main) {

    // display auto off
    if (menuDisplay.selection()==0)  {systemData.display_auto_off^=true;}
    
    // iter display auto off timing
    if (menuDisplay.selection()==1)  {
      systemData.index_display_autooff_times++;
      if (systemData.index_display_autooff_times>systemData.max_display_autooff_times) {systemData.index_display_autooff_times=0;}
      systemData.display_timeout = systemData.display_autooff_times[systemData.index_display_autooff_times];
    }

    // iter display border color
    if (menuDisplay.selection()==2) {systemData.index_display_border_color++;
      if (systemData.index_display_border_color>systemData.max_color_index) {systemData.index_display_border_color=0;}
      systemData.color_border=systemData.display_color[systemData.index_display_border_color];
    }

    // iter display border color
    if (menuDisplay.selection()==3) {systemData.index_display_content_color++;
      if (systemData.index_display_content_color>systemData.max_color_index) {systemData.index_display_content_color=0;}
      systemData.color_content=systemData.display_color[systemData.index_display_content_color];
    }

    // iter display menu border color
    if (menuDisplay.selection()==4) {systemData.index_display_menu_border_color++;
      if (systemData.index_display_menu_border_color>systemData.max_color_index) {systemData.index_display_menu_border_color=0;}
      systemData.color_menu_border=systemData.display_color[systemData.index_display_menu_border_color];
    }

    // iter display menu content color
    if (menuDisplay.selection()==5) {systemData.index_display_menu_content_color++;
      if (systemData.index_display_menu_content_color>systemData.max_color_index) {systemData.index_display_menu_content_color=0;}
      systemData.color_menu_content=systemData.display_color[systemData.index_display_menu_content_color];
    }

    // iter display title color
    if (menuDisplay.selection()==6) {systemData.index_display_title_color++;
      if (systemData.index_display_title_color>systemData.max_color_index) {systemData.index_display_title_color=0;}
      systemData.color_title=systemData.display_color[systemData.index_display_title_color];
    }

    // iter display subtitle color
    if (menuDisplay.selection()==7) {systemData.index_display_color_subtitle++;
      if (systemData.index_display_color_subtitle>systemData.max_color_index) {systemData.index_display_color_subtitle=0;}
      systemData.color_subtitle=systemData.display_color[systemData.index_display_color_subtitle];
    }
  }

  // ----------------------------------------------------------------

  // system page
  else if (menu_page==page_system_main) {

    // startup run matrix
    if (menuSystem.selection()==0) {systemData.matrix_run_on_startup^=true;}

    // consider matrix switch state handling before allowing the below two values to be changed after flashing
    // else if (menuSystem.selection()==1) {systemData.matrix_enabled^=true;}
    // else if (menuSystem.selection()==2) {systemData.port_controller_enabled^=true;}
  }

  // ----------------------------------------------------------------
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                Crude Function Name to Associated Value Mapping 

String getRelatedY(char * data) {
  /*
  returns y value for in ranges checks where x and y are different (not typically required for an in range check where range pertains to x alone).
  in ranges checks are square (check in square range) where z is square range. example x=lat, y=lon, z= squarerange = 0.0000100 = approx 1 meter in latitude.
  if inverted then in square range check becomes is out of square range check.
  */
  if (strcmp("DegLatLonRange", data)==0) {return String(satData.degrees_longitude, 10);}
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
  if (strcmp("DegLatOver", data)==0) {return String(satData.degrees_latitude, 10);}
  if (strcmp("DegLatUnder", data)==0) {return String(satData.degrees_latitude, 10);}
  if (strcmp("DegLatEqual", data)==0) {return String(satData.degrees_latitude, 10);}
  if (strcmp("DegLatRange", data)==0) {return String(satData.degrees_latitude, 10);}
  if (strcmp("DegLonOver", data)==0) {return String(satData.degrees_longitude, 10);}
  if (strcmp("DegLonUnder", data)==0) {return String(satData.degrees_longitude, 10);}
  if (strcmp("DegLonEqual", data)==0) {return String(satData.degrees_longitude, 10);}
  if (strcmp("DegLonRange", data)==0) {return String(satData.degrees_longitude, 10);}
  if (strcmp("DegLatLonRange", data)==0) {return String(satData.degrees_latitude, 10);}
  if (strcmp("UTCTimeGNGGAOver", data)==0) {return String(gnggaData.utc_time);}
  if (strcmp("UTCTimeGNGGAUnder", data)==0) {return String(gnggaData.utc_time);}
  if (strcmp("UTCTimeGNGGAEqual", data)==0) {return String(gnggaData.utc_time);}
  if (strcmp("UTCTimeGNGGARange", data)==0) {return String(gnggaData.utc_time);}
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
  if (strcmp("LFlagGPATTOver", data)==0) {return String(gpattData.line_flag);}
  if (strcmp("LFlagGPATTUnder", data)==0) {return String(gpattData.line_flag);}
  if (strcmp("LFlagGPATTEqual", data)==0) {return String(gpattData.line_flag);}
  if (strcmp("LFlagGPATTRange", data)==0) {return String(gpattData.line_flag);}
  if (strcmp("SFlagGPATTOver", data)==0) {return String(gpattData.static_flag);}
  if (strcmp("SFlagGPATTUnder", data)==0) {return String(gpattData.static_flag);}
  if (strcmp("SFlagGPATTEqual", data)==0) {return String(gpattData.static_flag);}
  if (strcmp("SFlagGPATTRange", data)==0) {return String(gpattData.static_flag);}
  if (strcmp("RSFlagGPATTOver", data)==0) {return String(gpattData.run_state_flag);}
  if (strcmp("RSFlagGPATTUnder", data)==0) {return String(gpattData.run_state_flag);}
  if (strcmp("RSFlagGPATTEqual", data)==0) {return String(gpattData.run_state_flag);}
  if (strcmp("RSFlagGPATTRange", data)==0) {return String(gpattData.run_state_flag);}
  if (strcmp("INSGPATTOver", data)==0) {return String(gpattData.ins);}
  if (strcmp("INSGPATTUnder", data)==0) {return String(gpattData.ins);}
  if (strcmp("INSGPATTEqual", data)==0) {return String(gpattData.ins);}
  if (strcmp("INSGPATTRange", data)==0) {return String(gpattData.ins);}
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
  if (strcmp("DayTime", data)==0) {return String(check_ge_and_le_true(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()), siderealPlanetData.sun_r, siderealPlanetData.sun_s));}
  if (strcmp("NightTime", data)==0) {return String(check_ge_and_le_false(hoursMinutesToInt(rtc.now().hour(), rtc.now().minute()), siderealPlanetData.sun_r, siderealPlanetData.sun_s));}
  if (strcmp("Sunrise", data)==0) {return String(siderealPlanetData.sun_r);}
  if (strcmp("Sunset", data)==0) {return String(siderealPlanetData.sun_s);}
  if (strcmp("MoonAzRange", data)==0) {return String(siderealPlanetData.moon_az);}
  if (strcmp("MoonAltRange", data)==0) {return String(siderealPlanetData.moon_alt);}
  if (strcmp("MoonUp", data)==0) {return String(siderealPlanetData.moon_r);}
  if (strcmp("MoonDown", data)==0) {return String(siderealPlanetData.moon_s);}
  if (strcmp("Moonrise", data)==0) {return String(siderealPlanetData.moon_r);}
  if (strcmp("Moonset", data)==0) {return String(siderealPlanetData.moon_s);}
  if (strcmp("MoonPhaseOver", data)==0) {return String((int)siderealPlanetData.moon_p);}
  if (strcmp("MoonPhaseUnder", data)==0) {return String((int)siderealPlanetData.moon_p);}
  if (strcmp("MoonPhaseEqual", data)==0) {return String((int)siderealPlanetData.moon_p);}
  if (strcmp("MoonPhaseRange", data)==0) {return String((int)siderealPlanetData.moon_p);}
  if (strcmp("MercuryAzRange", data)==0) {return String(siderealPlanetData.mercury_az);}
  if (strcmp("MercuryAltRange", data)==0) {return String(siderealPlanetData.mercury_alt);}
  if (strcmp("MercuryUp", data)==0) {return String(siderealPlanetData.mercury_r);}
  if (strcmp("MercuryDown", data)==0) {return String(siderealPlanetData.mercury_s);}
  if (strcmp("MercuryRise", data)==0) {return String(siderealPlanetData.mercury_r);}
  if (strcmp("MercurySet", data)==0) {return String(siderealPlanetData.mercury_s);}
  if (strcmp("VenusAzRange", data)==0) {return String(siderealPlanetData.venus_az);}
  if (strcmp("VenusAltRange", data)==0) {return String(siderealPlanetData.venus_alt);}
  if (strcmp("VenusUp", data)==0) {return String(siderealPlanetData.venus_r);}
  if (strcmp("VenusDown", data)==0) {return String(siderealPlanetData.venus_s);}
  if (strcmp("VenusRise", data)==0) {return String(siderealPlanetData.venus_r);}
  if (strcmp("VenusSet", data)==0) {return String(siderealPlanetData.venus_s);}
  if (strcmp("MarsAzRange", data)==0) {return String(siderealPlanetData.mars_az);}
  if (strcmp("MarsAltRange", data)==0) {return String(siderealPlanetData.mars_alt);}
  if (strcmp("MarsUp", data)==0) {return String(siderealPlanetData.mars_r);}
  if (strcmp("MarsDown", data)==0) {return String(siderealPlanetData.mars_s);}
  if (strcmp("MarsRise", data)==0) {return String(siderealPlanetData.mars_r);}
  if (strcmp("MarsSet", data)==0) {return String(siderealPlanetData.mars_s);}
  if (strcmp("JupiterAzRange", data)==0) {return String(siderealPlanetData.jupiter_az);}
  if (strcmp("JupiterAltRange", data)==0) {return String(siderealPlanetData.jupiter_alt);}
  if (strcmp("JupiterUp", data)==0) {return String(siderealPlanetData.jupiter_r);}
  if (strcmp("JupiterDown", data)==0) {return String(siderealPlanetData.jupiter_s);}
  if (strcmp("JupiterRise", data)==0) {return String(siderealPlanetData.jupiter_r);}
  if (strcmp("JupiterSet", data)==0) {return String(siderealPlanetData.jupiter_s);}
  if (strcmp("SaturnAzRange", data)==0) {return String(siderealPlanetData.saturn_az);}
  if (strcmp("SaturnAltRange", data)==0) {return String(siderealPlanetData.saturn_alt);}
  if (strcmp("SaturnUp", data)==0) {return String(siderealPlanetData.saturn_r);}
  if (strcmp("SaturnDown", data)==0) {return String(siderealPlanetData.saturn_s);}
  if (strcmp("SaturnRise", data)==0) {return String(siderealPlanetData.saturn_r);}
  if (strcmp("SaturnSet", data)==0) {return String(siderealPlanetData.saturn_s);}
  if (strcmp("UranusAzRange", data)==0) {return String(siderealPlanetData.uranus_az);}
  if (strcmp("UranusAltRange", data)==0) {return String(siderealPlanetData.uranus_alt);}
  if (strcmp("UranusUp", data)==0) {return String(siderealPlanetData.uranus_r);}
  if (strcmp("UranusDown", data)==0) {return String(siderealPlanetData.uranus_s);}
  if (strcmp("UranusRise", data)==0) {return String(siderealPlanetData.uranus_r);}
  if (strcmp("UranusSet", data)==0) {return String(siderealPlanetData.uranus_s);}
  if (strcmp("NeptuneAzRange", data)==0) {return String(siderealPlanetData.neptune_az);}
  if (strcmp("NeptuneAltRange", data)==0) {return String(siderealPlanetData.neptune_alt);}
  if (strcmp("NeptuneUp", data)==0) {return String(siderealPlanetData.neptune_r);}
  if (strcmp("NeptuneDown", data)==0) {return String(siderealPlanetData.neptune_s);}
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
  if (strcmp("Sensor0Over", data)==0) {return String(sensorData.sensor_0);}
  if (strcmp("Sensor0Under", data)==0) {return String(sensorData.sensor_0);}
  if (strcmp("Sensor0Equal", data)==0) {return String(sensorData.sensor_0);}
  if (strcmp("Sensor0Range", data)==0) {return String(sensorData.sensor_0);}
  if (strcmp("Sensor1Over", data)==0) {return String(sensorData.sensor_1);}
  if (strcmp("Sensor1Under", data)==0) {return String(sensorData.sensor_1);}
  if (strcmp("Sensor1Equal", data)==0) {return String(sensorData.sensor_1);}
  if (strcmp("Sensor1Range", data)==0) {return String(sensorData.sensor_1);}
  if (strcmp("Sensor2Over", data)==0) {return String(sensorData.sensor_2);}
  if (strcmp("Sensor2Under", data)==0) {return String(sensorData.sensor_2);}
  if (strcmp("Sensor2Equal", data)==0) {return String(sensorData.sensor_2);}
  if (strcmp("Sensor2Range", data)==0) {return String(sensorData.sensor_2);}
  if (strcmp("Sensor3Over", data)==0) {return String(sensorData.sensor_3);}
  if (strcmp("Sensor3Under", data)==0) {return String(sensorData.sensor_3);}
  if (strcmp("Sensor3Equal", data)==0) {return String(sensorData.sensor_3);}
  if (strcmp("Sensor3Range", data)==0) {return String(sensorData.sensor_3);}
  if (strcmp("Sensor4Over", data)==0) {return String(sensorData.sensor_4);}
  if (strcmp("Sensor4Under", data)==0) {return String(sensorData.sensor_4);}
  if (strcmp("Sensor4Equal", data)==0) {return String(sensorData.sensor_4);}
  if (strcmp("Sensor4Range", data)==0) {return String(sensorData.sensor_4);}
  if (strcmp("Sensor5Over", data)==0) {return String(sensorData.sensor_5);}
  if (strcmp("Sensor5Under", data)==0) {return String(sensorData.sensor_5);}
  if (strcmp("Sensor5Equal", data)==0) {return String(sensorData.sensor_5);}
  if (strcmp("Sensor5Range", data)==0) {return String(sensorData.sensor_5);}
  if (strcmp("Sensor6Over", data)==0) {return String(sensorData.sensor_6);}
  if (strcmp("Sensor6Under", data)==0) {return String(sensorData.sensor_6);}
  if (strcmp("Sensor6Equal", data)==0) {return String(sensorData.sensor_6);}
  if (strcmp("Sensor6Range", data)==0) {return String(sensorData.sensor_6);}
  if (strcmp("Sensor7Over", data)==0) {return String(sensorData.sensor_7);}
  if (strcmp("Sensor7Under", data)==0) {return String(sensorData.sensor_7);}
  if (strcmp("Sensor7Equal", data)==0) {return String(sensorData.sensor_7);}
  if (strcmp("Sensor7Range", data)==0) {return String(sensorData.sensor_7);}
  if (strcmp("Sensor8Over", data)==0) {return String(sensorData.sensor_8);}
  if (strcmp("Sensor8Under", data)==0) {return String(sensorData.sensor_8);}
  if (strcmp("Sensor8Equal", data)==0) {return String(sensorData.sensor_8);}
  if (strcmp("Sensor8Range", data)==0) {return String(sensorData.sensor_8);}
  if (strcmp("Sensor9Over", data)==0) {return String(sensorData.sensor_9);}
  if (strcmp("Sensor9Under", data)==0) {return String(sensorData.sensor_9);}
  if (strcmp("Sensor9Equal", data)==0) {return String(sensorData.sensor_9);}
  if (strcmp("Sensor9Range", data)==0) {return String(sensorData.sensor_9);}
  if (strcmp("Sensor10Over", data)==0) {return String(sensorData.sensor_10);}
  if (strcmp("Sensor10Under", data)==0) {return String(sensorData.sensor_10);}
  if (strcmp("Sensor10Equal", data)==0) {return String(sensorData.sensor_10);}
  if (strcmp("Sensor10Range", data)==0) {return String(sensorData.sensor_10);}
  if (strcmp("Sensor11Over", data)==0) {return String(sensorData.sensor_11);}
  if (strcmp("Sensor11Under", data)==0) {return String(sensorData.sensor_11);}
  if (strcmp("Sensor11Equal", data)==0) {return String(sensorData.sensor_11);}
  if (strcmp("Sensor11Range", data)==0) {return String(sensorData.sensor_11);}
  if (strcmp("Sensor12Over", data)==0) {return String(sensorData.sensor_12);}
  if (strcmp("Sensor12Under", data)==0) {return String(sensorData.sensor_12);}
  if (strcmp("Sensor12Equal", data)==0) {return String(sensorData.sensor_12);}
  if (strcmp("Sensor12Range", data)==0) {return String(sensorData.sensor_12);}
  if (strcmp("Sensor13Over", data)==0) {return String(sensorData.sensor_13);}
  if (strcmp("Sensor13Under", data)==0) {return String(sensorData.sensor_13);}
  if (strcmp("Sensor13Equal", data)==0) {return String(sensorData.sensor_13);}
  if (strcmp("Sensor13Range", data)==0) {return String(sensorData.sensor_13);}
  if (strcmp("Sensor14Over", data)==0) {return String(sensorData.sensor_14);}
  if (strcmp("Sensor14Under", data)==0) {return String(sensorData.sensor_14);}
  if (strcmp("Sensor14Equal", data)==0) {return String(sensorData.sensor_14);}
  if (strcmp("Sensor14Range", data)==0) {return String(sensorData.sensor_14);}
  if (strcmp("Sensor15Over", data)==0) {return String(sensorData.sensor_15);}
  if (strcmp("Sensor15Under", data)==0) {return String(sensorData.sensor_15);}
  if (strcmp("Sensor15Equal", data)==0) {return String(sensorData.sensor_15);}
  if (strcmp("Sensor15Range", data)==0) {return String(sensorData.sensor_15);}
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
  display.drawRect(0, 0, 127, 127);
}

// ------------------------------------------------
//                                    UI BORDER RED

void drawMainBorderRed() {
  display.setColor(RGB_COLOR16(255,0,0));
  display.drawRect(0, 0, 127, 127);
}

void drawMainBorderGreen() {
  display.setColor(RGB_COLOR16(0,255,0));
  display.drawRect(0, 0, 127, 127);
}

void drawGeneralTitle(String title, int color1, int color2) {
  /*
  Prints String title to top horizontal centre of display.
  Color1: Text color.
  Color2: Border coler. 
  */
  display.setColor(color1);
  canvas120x8.clear();
  // center the title at top of screen
  canvas120x8.printFixed((125/2)-((strlen(title.c_str())/2)*6), 1, title.c_str(), STYLE_BOLD );
  display.drawCanvas(1, 2, canvas120x8);
  // border the title
  display.setColor(color2);
  display.drawRect(0, 0, 127, 12);
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

void UIIndicators() {

  /*
  usefull for if we are going to indicate something before SPI switching where we will loose the display temporarily until we are finished with another SPI device.
  these pages should be written procedurally unlike UpdateUI task.
  should not be ran while UpdateUI is writing to display.
  */

  // ------------------------------------------------
  //                            SAVE MATRIX INDICATOR

  // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
  if (menu_page==page_save_matrix_file_indicator) {
    display.setColor(RGB_COLOR16(0,255,0));
    // ------------------------------------------------
    if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
      previous_menu_page=menu_page; display.clear();
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("SAVING")/2)*6), (display.height()/2)-16, "SAVING", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }
    // ------------------------------------------------
  }

  // ------------------------------------------------
  //                            LOAD MATRIX INDICATOR

  // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
  else if (menu_page==page_load_matrix_file_indicator) {
    display.setColor(RGB_COLOR16(0,255,0));
    // ------------------------------------------------
    if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
      previous_menu_page=menu_page; display.clear();
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("LOADING")/2)*6), (display.height()/2)-16, "LOADING", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }
    // ------------------------------------------------
  }

  // ------------------------------------------------
  //                          DELETE MATRIX INDICATOR

  // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
  else if (menu_page==page_delete_matrix_file_indicator) {
    display.setColor(RGB_COLOR16(0,255,0));
    // ------------------------------------------------
    if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
      previous_menu_page=menu_page; display.clear();
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("DELETING")/2)*6), (display.height()/2)-16, "DELETING", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }
    // ------------------------------------------------
  }

  // ------------------------------------------------
  //                   SAVING SYSTEM CONFIG INDICATOR

  // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
  else if (menu_page==page_save_system_config_indicator) {
    display.setColor(RGB_COLOR16(0,255,0));
    // ------------------------------------------------
    if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
      previous_menu_page=menu_page; display.clear();
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("SAVING")/2)*6), (display.height()/2)-16, "SAVING", STYLE_BOLD );
      canvas120x120.printFixed((120/2)-((strlen("SYSTEM CONFIG")/2)*6), (display.height()/2), "SYSTEM CONFIG", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }
    // ------------------------------------------------
  }

  // ------------------------------------------------
  //        RESTORING DEFAULT SYSTEM CONFIG INDICATOR

  // indicator page (to circumvent unwanted input there are no input controls wired up for this page)
  else if (menu_page==page_restore_default_matrix_indicator) {
    display.setColor(RGB_COLOR16(0,255,0));
    // ------------------------------------------------
    if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
      previous_menu_page=menu_page; display.clear();
      canvas120x120.clear();
      canvas120x120.printFixed((120/2)-((strlen("RESTORING")/2)*6), (display.height()/2)-16, "RESTORING", STYLE_BOLD );
      canvas120x120.printFixed((120/2)-((strlen("SYSTEM CONFIG")/2)*6), (display.height()/2), "SYSTEM CONFIG", STYLE_BOLD );
      display.drawCanvas(5, 5, canvas120x120);
      drawMainBorderGreen();
    }
    // ------------------------------------------------
  }
}

// ------------------------------------------------
//                                               UI

bool display_sync;

// void UpdateUI() {
void UpdateUI(void * pvParamters) {

  while (1) {

  // this call should not happen while ui is being updated, ui is updated here on a task, so currently the call is here so that this always happens before writing to display. 
  readI2C();

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
  // menu_page=page_CD74HC4067_main; // uncomment to debug

  // ------------------------------------------------
  //                                  UPDATE UI PAGES

  if (update_ui==true) {
    // debug("[oled protection] allowing ui update");
    // debug("[menu page] " + String(menu_page));

    // ------------------------------------------------
    //                                        HOME PAGE

    if (menu_page==page_home) {
      // ------------------------------------------------
      // performace/efficiency: draw conditionally 
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page;
        display.clear();
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // show datetime
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, formatRTCTime().c_str(), STYLE_BOLD );
      display.drawCanvas(39, 4, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, formatRTCDateAbbreviated().c_str(), STYLE_BOLD );
      display.drawCanvas(39, 14, canvas60x8);
      // ------------------------------------------------
      // performace/efficiency: draw conditionally
      if (updateui_content==true) {
        updateui_content=false;
        // ------------------------------------------------
        display.setColor(systemData.color_menu_border);
        menuHome.showMenuBorder(display);
        display.setColor(systemData.color_menu_content);
        menuHome.showMenuContent(display);
      // ------------------------------------------------
      }
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                    SETTINGS PAGE

    else if (menu_page==page_main_menu) {
      // ------------------------------------------------
      // performace/efficiency: draw conditionally 
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page;
        display.clear();
        drawMainBorder();
        drawGeneralTitle("SETTINGS", systemData.color_title, systemData.color_border);
      }
      // ------------------------------------------------
      // performace/efficiency: draw conditionally 
      if (updateui_content==true) {
        updateui_content=false;
        display.setColor(systemData.color_content);
        // ------------------------------------------------
        display.setColor(systemData.color_menu_border);
        menuMain.showMenuBorder(display);
        display.setColor(systemData.color_menu_content);
        menuMain.showMenuContent(display);
      }
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                         MATRIX SWITCH LOGIC PAGE

    else if (menu_page==page_matrix_logic_main) {
      // ------------------------------------------------
      // performace/efficiency: draw conditionally 
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("MATRIX LOGIC", systemData.color_title, systemData.color_border);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_border);
      // ------------------------------------------------
      display.drawHLine(1, 37, 127); // seperate combo bar from content 0
      display.drawHLine(1, 49, 127); // seperate content 0 from content 1
      display.drawHLine(1, 93, 127); // seperate content 1 from content 2
      display.drawVLine(64, 37, 49); // seperate enabled/disabled from high/low

      display.setColor(systemData.color_content);

      // ------------------------------------------------
      // ENABLED
      if (matrixData.matrix_switch_enabled[0][menuMatrixSwitchSelect.selection()]==true) {
        canvas60x8.clear();
        display.setColor(RGB_COLOR16(0,0,255)); // emphasis
        canvas60x8.printFixed((60/2)-((strlen("ENABLED")/2)*6), 1, "ENABLED", STYLE_BOLD );
        display.drawCanvas(1, 39, canvas60x8);
      }
      // DISABLED
      else {
        canvas60x8.clear();
        display.setColor(RGB_COLOR16(255,0,0));
        canvas60x8.printFixed((60/2)-((strlen("DISABLED")/2)*6), 1, "DISABLED", STYLE_BOLD );
        display.drawCanvas(1, 39, canvas60x8);
      }

      // ------------------------------------------------
      // ACTIVE
      if (matrixData.matrix_switch_state[0][menuMatrixSwitchSelect.selection()]==true) {
        canvas60x8.clear();
        display.setColor(RGB_COLOR16(0,0,255)); // emphasis
        canvas60x8.printFixed((60/2)-((strlen("ACTIVE")/2)*6), 1, "ACTIVE", STYLE_BOLD );
        display.drawCanvas(66, 39, canvas60x8);
      }
      // INACTIVE
      else {
        canvas60x8.clear();
        display.setColor(RGB_COLOR16(255,0,0));
        canvas60x8.printFixed((60/2)-((strlen("INACTIVE")/2)*6), 1, "INACTIVE", STYLE_BOLD );
        display.drawCanvas(66, 39, canvas60x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // function name
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "");
      strcat(TMP_UI_DATA_0, matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]);
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
      display.drawCanvas(3, 52, canvas120x8);

      // ------------------------------------------------
      // function x
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "X ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 62, canvas120x8);

      // ------------------------------------------------
      // function y
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Y ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 72, canvas120x8);

      // ------------------------------------------------
      // function z
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Z ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 82, canvas120x8);

      // ------------------------------------------------
      // real x: display functions associated X
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "X ");
      strcat(TMP_UI_DATA_0, getRelatedX(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 96, canvas120x8);

      // ------------------------------------------------
      // real y: display functions associated Y
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Y ");
      strcat(TMP_UI_DATA_0, getRelatedY(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 106, canvas120x8);

      // ------------------------------------------------
      // real z: display functions associated Z
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Z ");
      strcat(TMP_UI_DATA_0, getRelatedZ(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, 116, canvas120x8);

      // ------------------------------------------------
      display.setColor(systemData.color_content);

      // ------------------------------------------------
      // clear any previously highlighted menus
      if (previous_menu_column_selection!=menu_column_selection) {
        canvas126x24.clear();
        display.drawCanvas(1, 13, canvas126x24);
        previous_menu_column_selection=menu_column_selection;
      }

      // ------------------------------------------------
      // highlight matrix switch select menu
      if (menu_column_selection == 0) {
        display.setColor(systemData.color_menu_border);
        menuMatrixSwitchSelect.showMenuBorder(display);
        display.setColor(systemData.color_menu_content);
        menuMatrixSwitchSelect.showMenuContent(display);
      }
      else {
        // draw currently selected menu item when menu not highlighted
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "");
        display.setColor(systemData.color_menu_content);
        strcat(TMP_UI_DATA_0, menuMatrixSwitchSelectItems[menuMatrixSwitchSelect.selection()]);
        canvas19x8.clear();
        canvas19x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.setColor(systemData.color_border);
        display.drawCanvas(10, 19, canvas19x8);
      }

      // ------------------------------------------------
      // highlight matrix switch port select
      if (menu_column_selection == 1) {
        // port number
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "P");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
        canvas19x8.clear();
        // indicate if port number is -1 (none)
        if (matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]>=0) {display.setColor(RGB_COLOR16(0,0,255));}
        else {display.setColor(RGB_COLOR16(255,0,0));}
        canvas19x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_NORMAL);
        display.drawCanvas(39, 19, canvas19x8);
        display.setColor(systemData.color_menu_content);
        display.drawRect(35, 15, 62, 15+15);
      }
      else {
        // draw currently selected menu item when menu not highlighted
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "P");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
        canvas19x8.clear();
        // indicate if port number is -1 (none)
        if (matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]>=0) {display.setColor(RGB_COLOR16(0,0,255));}
        else {display.setColor(RGB_COLOR16(255,0,0));}
        canvas19x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(39, 19, canvas19x8);
        display.setColor(systemData.color_content);
      }

      // ------------------------------------------------
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
        display.drawCanvas(68, 19, canvas8x8);
        display.setColor(systemData.color_menu_content);
        display.drawRect(66, 15, 79, 15+15);
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
        display.drawCanvas(68, 19, canvas8x8);
        display.setColor(systemData.color_content);
      }

      // ------------------------------------------------
      // highlight matrix switch inverted logic
      if (menu_column_selection == 3) {
        canvas8x8.clear();
        display.setColor(systemData.color_content);
        if (matrixData.matrix_switch_inverted_logic[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]==true) {
          display.setColor(RGB_COLOR16(255,255,0)); // emphasis
          canvas8x8.printFixed(1, 1, "I", STYLE_NORMAL ); // inverted function logic (not switch logic, this is per function on a switch) 
        }
        else if (matrixData.matrix_switch_inverted_logic[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]==false) {
          display.setColor(RGB_COLOR16(0,0,255));
          canvas8x8.printFixed(1, 1, "S", STYLE_NORMAL ); // standard function logic (not switch logic, this is per function on a switch) 
        }
        display.drawCanvas(84, 19, canvas8x8);
        display.setColor(systemData.color_menu_content);
        display.drawRect(83, 15, 93, 15+15);
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
          display.setColor(RGB_COLOR16(0,0,255));
          canvas8x8.printFixed(1, 1, "S", STYLE_BOLD ); // standard function logic (not switch logic, this is per function on a switch) 
        }
        display.drawCanvas(83, 19, canvas8x8);
      }

      // ------------------------------------------------
      // highlight matrix switch function select menu
      if (menu_column_selection == 4) {
        display.setColor(systemData.color_menu_border);
        menuMatrixFunctionSelect.showMenuBorder(display);
        display.setColor(systemData.color_menu_content);
        menuMatrixFunctionSelect.showMenuContent(display);
      }
      else {
        // draw currently selected menu item when menu not highlighted
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "");
        strcat(TMP_UI_DATA_0, menuMatrixFunctionSelectItems[menuMatrixFunctionSelect.selection()]);
        canvas19x8.clear();
        display.setColor(systemData.color_menu_content);
        canvas19x8.printFixed(5, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(91+4, 19, canvas19x8);
      }
    }

    // ------------------------------------------------
    //                        OVERVIEW MATRIX SWITCHING

    else if (menu_page==page_overview_matrix_switching) {
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page;
        display.clear();
        drawMainBorder();
        drawGeneralTitle("MATRIX OVERVIEW", systemData.color_title, systemData.color_border);
        display.drawRect(0, 12, 127, 26);
        display.drawVLine(64, 13, 25);
        // ------------------------------------------------
      }

      int size = 23;
      int start = 2;
      for (int i=0; i<5; i++) {

        // enabled/disabled
        canvas60x8.clear();
        display.setColor(systemData.color_content);
        canvas60x8.printFixed((60/2)-((strlen(String("E" + String(matrixData.matrix_enabled_i) + " D" + String(matrixData.matrix_disabled_i)).c_str())/2)*6), 1, String("E" + String(matrixData.matrix_enabled_i) + " D" + String(matrixData.matrix_disabled_i)).c_str(), STYLE_BOLD );
        display.drawCanvas(1, 15, canvas60x8);

        // on/off
        canvas60x8.clear();
        display.setColor(systemData.color_content);
        canvas60x8.printFixed((60/2)-((strlen(String("A" + String(matrixData.matrix_active_i) + " I" + String(matrixData.matrix_inactive_i)).c_str())/2)*6), 1, String("A" + String(matrixData.matrix_active_i) + " I" + String(matrixData.matrix_inactive_i)).c_str(), STYLE_BOLD );
        display.drawCanvas(65, 15, canvas60x8);

        // ------------------------------------------------

        // 0-4 switch number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_enabled[0][i]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String("S" + String(i)).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 30, canvas19x8);

        // 0-4 port number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_state[0][i]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String(matrixData.matrix_port_map[0][i]).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 40, canvas19x8);

        // ------------------------------------------------

        // 5-9 switch number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_enabled[0][i+5]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String("S" + String(i+5)).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 55, canvas19x8);

        // 5-9 port number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_state[0][i+5]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String(matrixData.matrix_port_map[0][i+5]).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 65, canvas19x8);

        // ------------------------------------------------

        // 10-14 switch number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_enabled[0][i+10]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String("S" + String(i+10)).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 81, canvas19x8);

        // 10-14 port number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_state[0][i+10]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String(matrixData.matrix_port_map[0][i+10]).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 91, canvas19x8);

        // ------------------------------------------------

        // 15-19 switch number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_enabled[0][i+15]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String("S" + String(i+15)).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 106, canvas19x8);

        // 15-19 port number
        canvas19x8.clear();
        display.setColor(RGB_COLOR16(64,64,64));
        if (matrixData.matrix_switch_state[0][i+15]==true) {display.setColor(RGB_COLOR16(0,255,0));}
        canvas19x8.printFixed(1, 1, String(matrixData.matrix_port_map[0][i+15]).c_str(), STYLE_NORMAL );
        display.drawCanvas(start+1, 116, canvas19x8);

        
        // ------------------------------------------------
        // adjust x (end 100+size) +5rem
        start = start+25;
        // ------------------------------------------------
      }
    }

    // ------------------------------------------------
    //                                ENTER DIGITS PAGE

    else if (menu_page==page_input_data) {

      // ------------------------------------------------
      //                                ENTER PORT NUMBER

      if (enter_digits_key==1) {
        if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
          previous_menu_page=menu_page; display.clear();
          drawMainBorder();
          // ------------------------------------------------
          drawGeneralTitle("ENTER PORT NUMBER", RGB_COLOR16(255,0,0), systemData.color_border);
          // ------------------------------------------------
          display.setColor(systemData.color_border);
          // ------------------------------------------------
          display.drawHLine(1, 108, 127);
          display.drawVLine(90, 13, 37);
          display.drawHLine(1, 38, 127);
          // ------------------------------------------------
        }
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas80x8.clear();
        canvas80x8.printFixed(1, 1, String("MATRIX SWITCH" + String(menuMatrixSwitchSelect.selection())).c_str(), STYLE_BOLD );
        display.drawCanvas(3, ui_content_0, canvas80x8);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
        canvas19x8.clear();
        canvas19x8.printFixed(1, 1, String(menuMatrixSwitchSelect.selection()).c_str(), STYLE_BOLD );
        display.drawCanvas(100, ui_content_0, canvas19x8);

        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas80x8.clear();
        canvas80x8.printFixed(1, 1, "CURRENT PORT", STYLE_BOLD );
        display.drawCanvas(3, ui_content_1, canvas80x8);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
        canvas19x8.clear();
        canvas19x8.printFixed(1, 1, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str(), STYLE_BOLD );
        display.drawCanvas(100, ui_content_1, canvas19x8);
      }

      // ------------------------------------------------
      //                             ENTER FUNCTION X,Y,Z

      else if ((enter_digits_key==2) || (enter_digits_key==3) || (enter_digits_key==4)) {
        if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
          previous_menu_page=menu_page; display.clear();
          drawMainBorder();
          if (enter_digits_key==2)      {drawGeneralTitle("ENTER VALUE X", systemData.color_title, systemData.color_border);;}
          else if (enter_digits_key==3) {drawGeneralTitle("ENTER VALUE Y", systemData.color_title, systemData.color_border);;}
          else if (enter_digits_key==4) {drawGeneralTitle("ENTER VALUE Z", systemData.color_title, systemData.color_border);;}
          // ------------------------------------------------
          display.setColor(systemData.color_border);
          // ------------------------------------------------
          display.drawHLine(1, 28, 127);
          // ------------------------------------------------
        }
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        // matrix switch number
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "M");
        strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelect.selection()).c_str());
        strcat(TMP_UI_DATA_0, "   F");
        strcat(TMP_UI_DATA_0, String(menuMatrixFunctionSelect.selection()).c_str());
        strcat(TMP_UI_DATA_0, "   P");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed((125/2)-((strlen(TMP_UI_DATA_0)/2)*6), 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(1, ui_content_0, canvas120x8);

        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
        // function name
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "");
        strcat(TMP_UI_DATA_0, matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]);
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
        display.drawCanvas(3, ui_content_2-2, canvas120x8);
        // function x
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "X ");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, ui_content_3-2, canvas120x8);
        // function y
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Y ");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, ui_content_4-2, canvas120x8);
        // function z
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Z ");
        strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, ui_content_5-2, canvas120x8);
        // real x: display functions X
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "X ");
        strcat(TMP_UI_DATA_0, getRelatedX(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, ui_content_6-2, canvas120x8);
        // real y: display functions Y
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Y ");
        strcat(TMP_UI_DATA_0, getRelatedY(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, ui_content_7-2, canvas120x8);
        // real z: display functions Z
        memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
        strcpy(TMP_UI_DATA_0, "Z ");
        strcat(TMP_UI_DATA_0, getRelatedZ(matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]).c_str());
        canvas120x8.clear();
        canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
        display.drawCanvas(3, ui_content_8-2, canvas120x8);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        // ------------------------------------------------
        display.drawHLine(1, 108, 127);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(RGB_COLOR16(0,255,0));
      // ------------------------------------------------
      // draw input data
      canvas120x8.clear();
      canvas120x8.printFixed((125/2)-((strlen(String(input_data).c_str())/2)*6), 1, input_data, STYLE_BOLD );
      display.drawCanvas(2, 112, canvas120x8);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                     SELECT FUNCTION OPTIONS PAGE

    // select function name, x, y, or z
    else if (menu_page==page_matrix_logic_select_setup) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SETUP SWITCH LOGIC", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        // ------------------------------------------------
        display.drawHLine(1, 28, 127);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      // matrix switch number 
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "M");
      strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelect.selection()).c_str());
      strcat(TMP_UI_DATA_0, "   F");
      strcat(TMP_UI_DATA_0, String(menuMatrixFunctionSelect.selection()).c_str());
      strcat(TMP_UI_DATA_0, "   P");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_port_map[0][menuMatrixSwitchSelect.selection()]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed((125/2)-((strlen(TMP_UI_DATA_0)/2)*6), 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(1, ui_content_0, canvas120x8);

      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // function name
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "");
      strcat(TMP_UI_DATA_0, matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]);
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
      display.drawCanvas(3, ui_content_2-4, canvas120x8);
      // function x
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "X ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][0]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, ui_content_3-4, canvas120x8);
      // function y
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Y ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][1]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, ui_content_4-4, canvas120x8);
      // function z
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "Z ");
      strcat(TMP_UI_DATA_0, String(matrixData.matrix_function_xyz[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()][2]).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, ui_content_5-4, canvas120x8);
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuMatrixConfigureFunction.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuMatrixConfigureFunction.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                        SELECT FUNCTION NAME PAGE

    // matrix switch set function name
    else if (menu_page==page_matrix_logic_setup_function) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SELECT FUNCTION", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        // ------------------------------------------------
        display.drawHLine(1, 28, 127);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      // matrix switch number 
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "M");
      strcat(TMP_UI_DATA_0, String(menuMatrixSwitchSelect.selection()).c_str());
      strcat(TMP_UI_DATA_0, "      F");
      strcat(TMP_UI_DATA_0, String(menuMatrixFunctionSelect.selection()).c_str());
      canvas120x8.clear();
      canvas120x8.printFixed((125/2)-((strlen(TMP_UI_DATA_0)/2)*6), 1, TMP_UI_DATA_0, STYLE_BOLD );
      display.drawCanvas(3, ui_content_0, canvas120x8);

      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // function name
      memset(TMP_UI_DATA_0, 0, sizeof(TMP_UI_DATA_0));
      strcpy(TMP_UI_DATA_0, "");
      strcat(TMP_UI_DATA_0, matrixData.matrix_function[menuMatrixSwitchSelect.selection()][menuMatrixFunctionSelect.selection()]);
      canvas120x8.clear();
      canvas120x8.printFixed(1, 1, TMP_UI_DATA_0, STYLE_BOLD);
      display.drawCanvas(3, ui_content_2-1, canvas120x8);
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuMatrixSetFunctionName.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuMatrixSetFunctionName.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                        FILE MENU

    else if (menu_page==page_file_main) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("FILE", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        // ------------------------------------------------
        display.drawHLine(1, 28, 127);
        display.drawVLine(46, 13, 26);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, "MATRIX", STYLE_BOLD );
      display.drawCanvas(3, ui_content_0, canvas42x8);
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sdcardData.matrix_filename).c_str(), STYLE_BOLD );
      display.drawCanvas(50, ui_content_0, canvas60x8);

      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuFile.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuFile.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                 SAVE MATRIX MENU

    else if (menu_page==page_file_save_matrix) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SAVE", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // set items each iteration so that if changed anywhere will be reflected in ui
      setMenuMatrixFilePathItems();
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuMatrixFilepath.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuMatrixFilepath.showMenuContent(display);
      // ------------------------------------------------
      
    }

    // ------------------------------------------------
    //                                 LOAD MATRIX MENU

    // load matrix
    else if (menu_page==page_file_load_matrix) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("LOAD", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // set items each iteration so that if changed anywhere will be reflected in ui
      setMenuMatrixFilePathItems();
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuMatrixFilepath.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuMatrixFilepath.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                               DELETE MATRIX MENU

    // delete matrix
    else if (menu_page==page_file_delete_matrix) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("DELETE", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // set items each iteration so that if changed anywhere will be reflected in ui
      setMenuMatrixFilePathItems();
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuMatrixFilepath.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuMatrixFilepath.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                         GPS MENU

    else if (menu_page==page_gps_main) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("GPS", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // set items each iteration so that if changed anywhere will be reflected in ui
      if (systemData.satio_enabled==true) {menuGPSItems[0 ]                             ="SATIO   ENABLED";}
      else {menuGPSItems[0]                                                             ="SATIO   DISABLED";}
      if (systemData.gngga_enabled==true) {menuGPSItems[1]                              ="GNGGA   ENABLED";}
      else {menuGPSItems[1]                                                             ="GNGGA   DISABLED";}
      if (systemData.gnrmc_enabled==true) {menuGPSItems[2]                              ="GNRMC   ENABLED";}
      else {menuGPSItems[2]                                                             ="GNRMC   DISABLED";}
      if (systemData.gpatt_enabled==true) {menuGPSItems[3]                              ="GPATT   ENABLED";}
      else {menuGPSItems[3]                                                             ="GPATT   DISABLED";}
      if (strcmp(satData.coordinate_conversion_mode, "GNGGA")==0) {menuGPSItems[4]      ="CONVERT GNGGA";}
      else if (strcmp(satData.coordinate_conversion_mode, "GNRMC")==0) {menuGPSItems[4] ="CONVERT GNRMC";}
      menuGPSItems[5]                                                                   ="VIEW    GNGGA";
      menuGPSItems[6]                                                                   ="VIEW    GNRMC";
      menuGPSItems[7]                                                                   ="VIEW    GPATT";
      menuGPSItems[8]                                                                   ="VIEW    SATIO";
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuGPS.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuGPS.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                      SERIAL MENU

    /* output data to be parsed by other systems or to be read by humans */

    else if (menu_page==page_serial_main) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SERIAL", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // set items each iteration so that if changed anywhere will be reflected in ui
      if (systemData.output_satio_enabled==true) {menuSerialItems[0]    ="SATIO   ENABLED";}
      else {menuSerialItems[0]                                          ="SATIO   DISABLED";}
      if (systemData.output_gngga_enabled==true) {menuSerialItems[1]    ="GNGGA   ENABLED";}
      else {menuSerialItems[1]                                          ="GNGGA   DISABLED";}
      if (systemData.output_gnrmc_enabled==true) {menuSerialItems[2]    ="GNRMC   ENABLED";}
      else {menuSerialItems[2]                                          ="GNRMC   DISABLED";}
      if (systemData.output_gpatt_enabled==true) {menuSerialItems[3]    ="GPATT   ENABLED";}
      else {menuSerialItems[3]                                          ="GPATT   DISABLED";}
      if (systemData.output_matrix_enabled==true) {menuSerialItems[4]   ="MATRIX  ENABLED";}
      else {menuSerialItems[4]                                          ="MATRIX  DISABLED";}
      if (systemData.output_sensors_enabled==true) {menuSerialItems[5]  ="SENSORS ENABLED";}
      else {menuSerialItems[5]                                          ="SENSORS DISABLED";}
      if (systemData.output_sun_enabled==true) {menuSerialItems[6]      ="SUN     ENABLED";}
      else {menuSerialItems[6]                                          ="SUN     DISABLED";}
      if (systemData.output_moon_enabled==true) {menuSerialItems[7]     ="MOON    ENABLED";}
      else {menuSerialItems[7]                                          ="MOON    DISABLED";}
      if (systemData.output_mercury_enabled==true) {menuSerialItems[8]  ="MERCURY ENABLED";}
      else {menuSerialItems[8]                                          ="MERCURY DISABLED";}
      if (systemData.output_venus_enabled==true) {menuSerialItems[9]    ="VENUS   ENABLED";}
      else {menuSerialItems[9]                                          ="VENUS   DISABLED";}
      if (systemData.output_mars_enabled==true) {menuSerialItems[10]    ="MARS    ENABLED";}
      else {menuSerialItems[10]                                         ="MARS    DISABLED";}
      if (systemData.output_jupiter_enabled==true) {menuSerialItems[11] ="JUPITER ENABLED";}
      else {menuSerialItems[11]                                         ="JUPITER DISABLED";}
      if (systemData.output_saturn_enabled==true) {menuSerialItems[12]   ="SATURN  ENABLED";}
      else {menuSerialItems[12]                                         ="SATURN  DISABLED";}
      if (systemData.output_uranus_enabled==true) {menuSerialItems[13]   ="URANUS  ENABLED";}
      else {menuSerialItems[13]                                         ="URANUS  DISABLED";}
      if (systemData.output_neptune_enabled) {menuSerialItems[14]       ="NEPTUNE ENABLED";}
      else {menuSerialItems[14]                                         ="NEPTUNE DISABLED";}
      if (systemData.debug==true) {menuSerialItems[15]                  ="DEBUG   ENABLED";}
      else {menuSerialItems[15]                                         ="DEBUG   DISABLED";}
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuSerial.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuSerial.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                         UNIVERSE

    /* currently solar system tracking */

    else if (menu_page==page_universe_main) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("UNIVERSE", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_content);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // set items each iteration so that if changed anywhere will be reflected in ui
      if (systemData.sidereal_track_sun==true) {menuUniverseItems[0]     ="SUN     ENABLED";} 
      else {menuUniverseItems[0]                                         ="SUN     DISABLED";}
      if (systemData.sidereal_track_mercury==true) {menuUniverseItems[1] ="MERCURY ENABLED";}
      else {menuUniverseItems[1]                                         ="MERCURY DISABLED";}
      if (systemData.sidereal_track_moon==true) {menuUniverseItems[2]    ="MOON    ENABLED";}
      else {menuUniverseItems[2]                                         ="MOON    DISABLED";}
      if (systemData.sidereal_track_venus==true) {menuUniverseItems[3]   ="VENUS   ENABLED";}
      else {menuUniverseItems[3]                                         ="VENUS   DISABLED";}
      if (systemData.sidereal_track_mars==true) {menuUniverseItems[4]    ="MARS    ENABLED";}
      else {menuUniverseItems[4]                                         ="MARS    DISABLED";}
      if (systemData.sidereal_track_jupiter==true) {menuUniverseItems[5] ="JUPITER ENABLED";}
      else {menuUniverseItems[5]                                         ="JUPITER DISABLED";}
      if (systemData.sidereal_track_saturn==true) {menuUniverseItems[6]  ="SATURN  ENABLED";}
      else {menuUniverseItems[6]                                         ="SATURN  DISABLED";}
      if (systemData.sidereal_track_uranus==true) {menuUniverseItems[7]  ="URANUS  ENABLED";}
      else {menuUniverseItems[7]                                         ="URANUS  DISABLED";}
      if (systemData.sidereal_track_neptune==true) {menuUniverseItems[8] ="NEPTUNE ENABLED";}
      else {menuUniverseItems[8]                                         ="NEPTUNE DISABLED";}
      menuUniverseItems[9]                                               ="VIEW    SUN";
      menuUniverseItems[10]                                              ="VIEW    MOON";
      menuUniverseItems[11]                                              ="VIEW    MERCURY";
      menuUniverseItems[12]                                              ="VIEW    VENUS";
      menuUniverseItems[13]                                              ="VIEW    MARS";
      menuUniverseItems[14]                                              ="VIEW    JUPITER";
      menuUniverseItems[15]                                              ="VIEW    SATURN";
      menuUniverseItems[16]                                              ="VIEW    URANUS";
      menuUniverseItems[17]                                              ="VIEW    NEPTUNE";
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuUniverse.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuUniverse.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                     DISPLAY MENU

    else if (menu_page==page_display_main) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
      }
      // this page currently has border and title drawn every frame in case the color is changed
      drawMainBorder();
      drawGeneralTitle("DISPLAY", systemData.color_title, systemData.color_border);
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // auto off
      if (systemData.display_auto_off==true) {menuDisplayItems[0] ="AUTO-OFF  ENABLED";}
      else {menuDisplayItems[0]                                   ="AUTO-OFF  DISABLED";}
      // auto off time
      menuDisplayItems[1] = systemData.char_display_autooff_times[systemData.index_display_autooff_times];
      // border color
      menuDisplayItems[2] = systemData.char_display_border_color[systemData.index_display_border_color];
      // content color
      menuDisplayItems[3] = systemData.char_display_content_color[systemData.index_display_content_color];
      // menu border color
      menuDisplayItems[4] = systemData.char_display_menu_border_color[systemData.index_display_menu_border_color];
      // menu content color
      menuDisplayItems[5] = systemData.char_display_menu_content_color[systemData.index_display_menu_content_color];
      // title color
      menuDisplayItems[6] = systemData.char_display_title_color[systemData.index_display_title_color];
      // subtitle color
      menuDisplayItems[7] = systemData.char_display_subtitle_color[systemData.index_display_color_subtitle];
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuDisplay.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuDisplay.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                      SYSTEM MENU

    else if (menu_page==page_system_main) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SYSTEM", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        // ------------------------------------------------
        display.drawHLine(1, 62, 127);
        display.drawVLine(46, 13, 61);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, "SPEED", STYLE_BOLD );
      display.drawCanvas(3, ui_content_0, canvas42x8);
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas60x8.clear();
      float speed_sec = timeData.mainLoopTimeTaken/1000000;
      canvas60x8.printFixed(1, 1, String(speed_sec, 6).c_str(), STYLE_BOLD );
      display.drawCanvas(50, ui_content_0, canvas60x8);

      // // overload
      // // looptime min
      // // looptime max
      // // looptime current
      // // uptime
      // // manually set overload time
      // manually set rtc

      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      // run matrix on startup
      if (systemData.matrix_run_on_startup==true) {menuSystemItems[0]="AUTO MATRIX ON";}
      else {menuSystemItems[0]="AUTO MATRIX OFF";}
      // // enable/disable matrix
      // if (systemData.matrix_enabled==true) {menuSystemItems[1]="MATRIX ENABLED";}
      // else {menuSystemItems[1]="MATRIX DISABLED";}
      // // enable/disable port controller
      // if (systemData.port_controller_enabled==true) {menuSystemItems[2]="PORT.CON ENABLED";}
      // else {menuSystemItems[2]="PORT.CON DISABLED";}
      // ------------------------------------------------
      display.setColor(systemData.color_menu_border);
      menuSystem.showMenuBorder(display);
      display.setColor(systemData.color_menu_content);
      menuSystem.showMenuContent(display);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                  CD74HC4067 MENU

    /* this may be a menu and is currently a view */

    else if (menu_page==page_CD74HC4067_main) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("CD74HC4067", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 36, 127);
        display.drawVLine(64, 16, 127);
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        /* sensor value column 0 */
        canvas60x8.clear();
        canvas60x8.printFixed((60/2)-((strlen("0-7")/2)*6), 1, "0-7", STYLE_BOLD );
        display.drawCanvas(3, 22, canvas60x8);
        // ------------------------------------------------
        /* sensor value column 1 */
        canvas60x8.clear();
        canvas60x8.printFixed((60/2)-((strlen("8-15")/2)*6), 1, "8-15", STYLE_BOLD );
        display.drawCanvas(66, 22, canvas60x8);
        // ------------------------------------------------
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      /*
      max decimal places can be customized per displayed sensor value.
      raw analog values do not require floats however sensors default data type is float to
      account for many developments made where a raw analog reading would be insufficient,
      like in the case of a digital sensor providing values accurate to N decimal places for example.
      this allows uniformity as default.
      */
     // ------------------------------------------------
      /* sensor value column 0 */
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_0, 4).c_str());
      display.drawCanvas(3, 42, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_1, 4).c_str());
      display.drawCanvas(3, 52, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_2, 4).c_str());
      display.drawCanvas(3, 62, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_3, 4).c_str());
      display.drawCanvas(3, 72, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_4, 4).c_str());
      display.drawCanvas(3, 82, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_5, 4).c_str());
      display.drawCanvas(3, 92, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_6, 4).c_str());
      display.drawCanvas(3, 102, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_7, 4).c_str());
      display.drawCanvas(3, 112, canvas60x8);
      /* sensor value column 1 */
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_8, 4).c_str());
      display.drawCanvas(67, 42, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_9, 4).c_str());
      display.drawCanvas(67, 52, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_10, 4).c_str());
      display.drawCanvas(67, 62, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_11, 4).c_str());
      display.drawCanvas(67, 72, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_12, 4).c_str());
      display.drawCanvas(67, 82, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_13, 4).c_str());
      display.drawCanvas(67, 92, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_14, 4).c_str());
      display.drawCanvas(67, 102, canvas60x8);
      canvas60x8.clear();
      canvas60x8.printFixed(1, 1, String(sensorData.sensor_15, 4).c_str());
      display.drawCanvas(67, 112, canvas60x8);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                       GNGGA MENU

    /* this may be a menu and is currently a view */

    else if (menu_page==page_gps_view_gngga) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("GNGGA", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawVLine(28, 13, 127);
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("UTC").c_str());
        display.drawCanvas(4, ui_content_0, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("SS").c_str());
        display.drawCanvas(4, ui_content_3, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("SC").c_str());
        display.drawCanvas(4, ui_content_4, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("PF").c_str());
        display.drawCanvas(4, ui_content_5, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("A").c_str());
        display.drawCanvas(4, ui_content_6, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("GEO").c_str());
        display.drawCanvas(4, ui_content_7, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("DD").c_str());
        display.drawCanvas(4, ui_content_8, canvas21x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.utc_time).c_str());
      display.drawCanvas(32, ui_content_0, canvas92x8);
      // ----------------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ----------------------------------------------------------
      canvas21x8.clear();
      canvas21x8.printFixed(1, 1, String(gnggaData.latitude_hemisphere).c_str());
      display.drawCanvas(4, ui_content_1, canvas21x8);
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.latitude).c_str());
      display.drawCanvas(32, ui_content_1, canvas92x8);
      // ----------------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ----------------------------------------------------------
      canvas21x8.clear();
      canvas21x8.printFixed(1, 1, String(gnggaData.longitude_hemisphere).c_str());
      display.drawCanvas(4, ui_content_2, canvas21x8);
      // ------------------------------------------------
       display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.longitude).c_str());
      display.drawCanvas(32, ui_content_2, canvas92x8);
      // ----------------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.solution_status).c_str());
      display.drawCanvas(32, ui_content_3, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.satellite_count_gngga).c_str());
      display.drawCanvas(32, ui_content_4, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.hdop_precision_factor).c_str());
      display.drawCanvas(32, ui_content_5, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.altitude).c_str());
      display.drawCanvas(32, ui_content_6, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.geoidal).c_str());
      display.drawCanvas(32, ui_content_7, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnggaData.differential_delay).c_str());
      display.drawCanvas(32, ui_content_8, canvas92x8);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                       GNRMC MENU

    /* this may be a menu and is currently a view */

    else if (menu_page==page_gps_view_gnrmc) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("GNRMC", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawVLine(28, 13, 127);
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("UTC").c_str());
        display.drawCanvas(4, ui_content_0, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("GS").c_str());
        display.drawCanvas(4, ui_content_3, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("GH").c_str());
        display.drawCanvas(4, ui_content_4, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("UTC").c_str());
        display.drawCanvas(4, ui_content_5, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("IA").c_str());
        display.drawCanvas(4, ui_content_6, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("IAD").c_str());
        display.drawCanvas(4, ui_content_7, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("MI").c_str());
        display.drawCanvas(4, ui_content_8, canvas21x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.utc_time).c_str());
      display.drawCanvas(32, ui_content_0, canvas92x8);
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas21x8.clear();
      canvas21x8.printFixed(1, 1, String(gnrmcData.latitude_hemisphere).c_str());
      display.drawCanvas(4, ui_content_1, canvas21x8);
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.latitude).c_str());
      display.drawCanvas(32, ui_content_1, canvas92x8);
      // ----------------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas21x8.clear();
      canvas21x8.printFixed(1, 1, String(gnrmcData.longitude_hemisphere).c_str());
      display.drawCanvas(4, ui_content_2, canvas21x8);
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.longitude).c_str());
      display.drawCanvas(32, ui_content_2, canvas92x8);
      // ----------------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.ground_speed).c_str());
      display.drawCanvas(32, ui_content_3, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.ground_heading).c_str());
      display.drawCanvas(32, ui_content_4, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.utc_date).c_str());
      display.drawCanvas(32, ui_content_5, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.installation_angle).c_str());
      display.drawCanvas(32, ui_content_6, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.installation_angle_direction).c_str());
      display.drawCanvas(32, ui_content_7, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gnrmcData.mode_indication).c_str());
      display.drawCanvas(32, ui_content_8, canvas92x8);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                       GPATT MENU

    /* this may be a menu and is currently a view */

    else if (menu_page==page_gps_view_gpatt) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("GPATT", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawVLine(28, 13, 127);
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("p").c_str());
        display.drawCanvas(4, ui_content_0, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("R").c_str());
        display.drawCanvas(4, ui_content_1, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("Y").c_str());
        display.drawCanvas(4, ui_content_2, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("INS").c_str());
        display.drawCanvas(4, ui_content_3, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("RSF").c_str());
        display.drawCanvas(4, ui_content_4, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("RIF").c_str());
        display.drawCanvas(4, ui_content_5, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("SF").c_str());
        display.drawCanvas(4, ui_content_6, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("GST").c_str());
        display.drawCanvas(4, ui_content_7, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("LF").c_str());
        display.drawCanvas(4, ui_content_8, canvas21x8);
        // ------------------------------------------------
        canvas21x8.clear();
        canvas21x8.printFixed(1, 1, String("M").c_str());
        display.drawCanvas(4, ui_content_9, canvas21x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.pitch).c_str());
      display.drawCanvas(32, ui_content_0, canvas92x8);
      // ----------------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.roll).c_str());
      display.drawCanvas(32, ui_content_1, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.yaw).c_str());
      display.drawCanvas(32, ui_content_2, canvas92x8);
      // ----------------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.ins).c_str());
      display.drawCanvas(32, ui_content_3, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.run_state_flag).c_str());
      display.drawCanvas(32, ui_content_4, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.run_inetial_flag).c_str());
      display.drawCanvas(32, ui_content_5, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.static_flag).c_str());
      display.drawCanvas(32, ui_content_6, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.gst_data).c_str());
      display.drawCanvas(32, ui_content_7, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.line_flag).c_str());
      display.drawCanvas(32, ui_content_8, canvas92x8);
      // ------------------------------------------------
      canvas92x8.clear();
      canvas92x8.printFixed(1, 1, String(gpattData.mileage).c_str());
      display.drawCanvas(32, ui_content_9, canvas92x8);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                                       SATIO MENU

    /* this may be a menu and is currently a view */

    else if (menu_page==page_gps_view_satio) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SATIO", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawVLine(41, 13, 127);
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas36x8.clear();
        canvas36x8.printFixed(1, 1, String("RTCT").c_str());
        display.drawCanvas(4, ui_content_0, canvas36x8);
        // ------------------------------------------------
        canvas36x8.clear();
        canvas36x8.printFixed(1, 1, String("RTCD").c_str());
        display.drawCanvas(4, ui_content_1, canvas36x8);
        // ------------------------------------------------
        canvas36x8.clear();
        canvas36x8.printFixed(1, 1, String("SRTCT").c_str());
        display.drawCanvas(4, ui_content_4, canvas36x8);
        // ------------------------------------------------
        canvas36x8.clear();
        canvas36x8.printFixed(1, 1, String("SRTCD").c_str());
        display.drawCanvas(4, ui_content_5, canvas36x8);
        // ------------------------------------------------
        canvas36x8.clear();
        canvas36x8.printFixed(1, 1, String("SUNR").c_str());
        display.drawCanvas(4, ui_content_6, canvas36x8);
        // ------------------------------------------------
        canvas36x8.clear();
        canvas36x8.printFixed(1, 1, String("SUNS").c_str());
        display.drawCanvas(4, ui_content_7, canvas36x8);
        // ------------------------------------------------
        canvas36x8.clear();
        canvas36x8.printFixed(1, 1, String("DAY").c_str());
        display.drawCanvas(4, ui_content_8, canvas36x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(formatRTCTime()).c_str());
      display.drawCanvas(45, ui_content_0, canvas80x8);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(formatRTCDate()).c_str());
      display.drawCanvas(45, ui_content_1, canvas80x8);
      // ------------------------------------------------
      canvas36x8.clear();
      if (strcmp(satData.coordinate_conversion_mode, "GNGGA")==0) {
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas36x8.printFixed(1, 1, String(gnggaData.latitude_hemisphere).c_str());
        display.drawCanvas(4, ui_content_2, canvas36x8);
      }
      else if (strcmp(satData.coordinate_conversion_mode, "GNRMC")==0) {
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas36x8.printFixed(1, 1, String(gnrmcData.latitude_hemisphere).c_str());
        display.drawCanvas(4, ui_content_2, canvas36x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(satData.degrees_latitude, 7).c_str());
      display.drawCanvas(45, ui_content_2, canvas80x8);
      // ------------------------------------------------
      canvas36x8.clear();
      if (strcmp(satData.coordinate_conversion_mode, "GNGGA")==0) {
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas36x8.printFixed(1, 1, String(gnggaData.longitude_hemisphere).c_str());
        display.drawCanvas(4, ui_content_3, canvas36x8);
      }
      else if (strcmp(satData.coordinate_conversion_mode, "GNRMC")==0) {
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas36x8.printFixed(1, 1, String(gnrmcData.longitude_hemisphere).c_str());
        display.drawCanvas(4, ui_content_3, canvas36x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(satData.degrees_longitude, 7).c_str());
      display.drawCanvas(45, ui_content_3, canvas80x8);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(satData.rtcSyncTime).c_str());
      display.drawCanvas(45, ui_content_4, canvas80x8);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(satData.rtcSyncDate).c_str());
      display.drawCanvas(45, ui_content_5, canvas80x8);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(siderealPlanetData.sun_r).c_str());
      display.drawCanvas(45, ui_content_6, canvas80x8);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(siderealPlanetData.sun_s).c_str());
      display.drawCanvas(45, ui_content_7, canvas80x8);
      // ------------------------------------------------
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(myAstro.HumanDayOfTheWeek(rtc.now().year(), rtc.now().month(), rtc.now().day())).c_str());
      display.drawCanvas(45, ui_content_8, canvas80x8);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                              UNIVERSE VIEWS: SUN

    /* currently solar system tracking */

    else if (menu_page==page_universe_view_sun) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SUN", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        // make three columns
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.sun_r) + String("  SET " + String(siderealPlanetData.sun_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.sun_r) + String("  SET " + String(siderealPlanetData.sun_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.sun_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.sun_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.sun_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.sun_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      // ------------------------------------------------
    }

    // ------------------------------------------------
    //                             UNIVERSE VIEWS: MOON

    else if (menu_page==page_universe_view_moon) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("MOON", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("PH").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("LUM").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.moon_r) + String("  SET " + String(siderealPlanetData.moon_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.moon_r) + String("  SET " + String(siderealPlanetData.moon_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.moon_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.moon_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.moon_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.moon_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas80x8.clear();
      canvas80x8.printFixed(1, 1, String(siderealPlanetData.moon_p_name[(int)siderealPlanetData.moon_p]).c_str());
      display.drawCanvas(40, ui_content_5, canvas80x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.moon_lum).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
    }

    // ------------------------------------------------
    //                          UNIVERSE VIEWS: MERCURY

    else if (menu_page==page_universe_view_mercury) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("MERCURY", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELA").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELO").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RADV").c_str());
        display.drawCanvas(4, ui_content_7, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DIST").c_str());
        display.drawCanvas(4, ui_content_8, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELAT").c_str());
        display.drawCanvas(4, ui_content_9, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELON").c_str());
        display.drawCanvas(4, ui_content_10, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.mercury_r) + String("  SET " + String(siderealPlanetData.mercury_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.mercury_r) + String("  SET " + String(siderealPlanetData.mercury_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_helio_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_5, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_helio_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_radius_vector).c_str());
      display.drawCanvas(40, ui_content_7, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_distance).c_str());
      display.drawCanvas(40, ui_content_8, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_9, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mercury_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_10, canvas42x8);
    }

    // ------------------------------------------------
    //                            UNIVERSE VIEWS: VENUS

    else if (menu_page==page_universe_view_venus) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("VENUS", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELA").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELO").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RADV").c_str());
        display.drawCanvas(4, ui_content_7, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DIST").c_str());
        display.drawCanvas(4, ui_content_8, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELAT").c_str());
        display.drawCanvas(4, ui_content_9, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELON").c_str());
        display.drawCanvas(4, ui_content_10, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.venus_r) + String("  SET " + String(siderealPlanetData.venus_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.venus_r) + String("  SET " + String(siderealPlanetData.venus_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_helio_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_5, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_helio_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_radius_vector).c_str());
      display.drawCanvas(40, ui_content_7, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_distance).c_str());
      display.drawCanvas(40, ui_content_8, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_9, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.venus_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_10, canvas42x8);
    }

    // ------------------------------------------------
    //                             UNIVERSE VIEWS: MARS

    else if (menu_page==page_universe_view_mars) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("MARS", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELA").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELO").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RADV").c_str());
        display.drawCanvas(4, ui_content_7, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DIST").c_str());
        display.drawCanvas(4, ui_content_8, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELAT").c_str());
        display.drawCanvas(4, ui_content_9, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELON").c_str());
        display.drawCanvas(4, ui_content_10, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.mars_r) + String("  SET " + String(siderealPlanetData.mars_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.mars_r) + String("  SET " + String(siderealPlanetData.mars_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_helio_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_5, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_helio_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_radius_vector).c_str());
      display.drawCanvas(40, ui_content_7, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_distance).c_str());
      display.drawCanvas(40, ui_content_8, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_9, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.mars_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_10, canvas42x8);
    }

    // ------------------------------------------------
    //                          UNIVERSE VIEWS: JUPITER

    else if (menu_page==page_universe_view_jupiter) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("JUPITER", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELA").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELO").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RADV").c_str());
        display.drawCanvas(4, ui_content_7, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DIST").c_str());
        display.drawCanvas(4, ui_content_8, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELAT").c_str());
        display.drawCanvas(4, ui_content_9, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELON").c_str());
        display.drawCanvas(4, ui_content_10, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.jupiter_r) + String("  SET " + String(siderealPlanetData.jupiter_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.jupiter_r) + String("  SET " + String(siderealPlanetData.jupiter_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_helio_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_5, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_helio_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_radius_vector).c_str());
      display.drawCanvas(40, ui_content_7, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_distance).c_str());
      display.drawCanvas(40, ui_content_8, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_9, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.jupiter_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_10, canvas42x8);
    }

    // ------------------------------------------------
    //                           UNIVERSE VIEWS: SATURN

    else if (menu_page==page_universe_view_saturn) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("SATURN", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELA").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELO").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RADV").c_str());
        display.drawCanvas(4, ui_content_7, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DIST").c_str());
        display.drawCanvas(4, ui_content_8, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELAT").c_str());
        display.drawCanvas(4, ui_content_9, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELON").c_str());
        display.drawCanvas(4, ui_content_10, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.saturn_r) + String("  SET " + String(siderealPlanetData.saturn_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.saturn_r) + String("  SET " + String(siderealPlanetData.saturn_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_helio_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_5, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_helio_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_radius_vector).c_str());
      display.drawCanvas(40, ui_content_7, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_distance).c_str());
      display.drawCanvas(40, ui_content_8, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_9, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.saturn_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_10, canvas42x8);
    }

    // ------------------------------------------------
    //                           UNIVERSE VIEWS: URANUS

    else if (menu_page==page_universe_view_uranus) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("URANUS", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELA").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELO").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RADV").c_str());
        display.drawCanvas(4, ui_content_7, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DIST").c_str());
        display.drawCanvas(4, ui_content_8, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELAT").c_str());
        display.drawCanvas(4, ui_content_9, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELON").c_str());
        display.drawCanvas(4, ui_content_10, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.uranus_r) + String("  SET " + String(siderealPlanetData.uranus_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.uranus_r) + String("  SET " + String(siderealPlanetData.uranus_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_helio_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_5, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_helio_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_radius_vector).c_str());
      display.drawCanvas(40, ui_content_7, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_distance).c_str());
      display.drawCanvas(40, ui_content_8, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_9, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.uranus_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_10, canvas42x8);
    }

    // ------------------------------------------------
    //                          UNIVERSE VIEWS: NEPTUNE

    else if (menu_page==page_universe_view_neptune) {
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      if ((menu_page != previous_menu_page) || (ui_cleared == true)) {
        previous_menu_page=menu_page; display.clear();
        drawMainBorder();
        drawGeneralTitle("NEPTUNE", systemData.color_title, systemData.color_border);
        // ------------------------------------------------
        display.setColor(systemData.color_border);
        display.drawHLine(1, 24, 127); // seperate rise and set from rest of content
        // ------------------------------------------------
        display.drawVLine(35, 24, 127); // vertical seperator 0
        display.drawVLine(84, 24, 127); // vertical seperator 1
        // ------------------------------------------------
        display.setColor(systemData.color_subtitle);
        // ------------------------------------------------
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RA").c_str());
        display.drawCanvas(4, ui_content_1, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DEC").c_str());
        display.drawCanvas(4, ui_content_2, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("AZ").c_str());
        display.drawCanvas(4, ui_content_3, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ALT").c_str());
        display.drawCanvas(4, ui_content_4, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELA").c_str());
        display.drawCanvas(4, ui_content_5, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("HELO").c_str());
        display.drawCanvas(4, ui_content_6, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("RADV").c_str());
        display.drawCanvas(4, ui_content_7, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("DIST").c_str());
        display.drawCanvas(4, ui_content_8, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELAT").c_str());
        display.drawCanvas(4, ui_content_9, canvas28x8);
        canvas28x8.clear();
        canvas28x8.printFixed(1, 1, String("ELON").c_str());
        display.drawCanvas(4, ui_content_10, canvas28x8);
      }
      // ------------------------------------------------
      display.setColor(systemData.color_subtitle);
      // ------------------------------------------------
      canvas120x8.clear();
      canvas120x8.printFixed((120/2)-((strlen(String("RISE " + String(siderealPlanetData.neptune_r) + String("  SET " + String(siderealPlanetData.neptune_s))).c_str())/2)*6), 1, String("RISE " + String(siderealPlanetData.neptune_r) + String("  SET " + String(siderealPlanetData.neptune_s))).c_str(), STYLE_BOLD );
      display.drawCanvas(4, ui_content_0-2, canvas120x8); // offset for hline
      // ------------------------------------------------
      display.setColor(systemData.color_content);
      // ------------------------------------------------
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_ra).c_str());
      display.drawCanvas(40, ui_content_1, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_dec).c_str());
      display.drawCanvas(40, ui_content_2, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_az).c_str());
      display.drawCanvas(40, ui_content_3, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_alt).c_str());
      display.drawCanvas(40, ui_content_4, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_helio_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_5, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_helio_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_6, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_radius_vector).c_str());
      display.drawCanvas(40, ui_content_7, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_distance).c_str());
      display.drawCanvas(40, ui_content_8, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_ecliptic_lat).c_str());
      display.drawCanvas(40, ui_content_9, canvas42x8);
      canvas42x8.clear();
      canvas42x8.printFixed(1, 1, String(siderealPlanetData.neptune_ecliptic_long).c_str());
      display.drawCanvas(40, ui_content_10, canvas42x8);
    }
    
    // ------------------------------------------------

    // set this flag last so that we can use it to update static ui data upon waking up from oled protection mode
    ui_cleared = false;

    // ------------------------------------------------
  }

  // ------------------------------------------------
  //                                  OLED PROTECTION

  if ((ui_cleared == false) && (update_ui == false)) {
    // debug("[oled protection] clearing ui");
    // uncomment if issues occur using faster option
    // display.clear();
    // display.clear();
    // display.clear();
    // faster
    canvas128x128.clear();
    display.drawCanvas(0, 0, canvas128x128);
    ui_cleared=true;
  }
  delay(1);
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA

#define I2C_ADDR_PORTCONTROLLER_0 9

struct I2CLinkStruct {
  char * token;
  byte OUTPUT_BUFFER[10]; // bytes to be sent
  char INPUT_BUFFER[10];  // chars received
  char TMP_BUFFER_0[10];  // chars of bytes to be sent
  char TMP_BUFFER_1[10];  // some space for type conversions
  int I2CADDRESSINDEX = 0;
  int I2CADDRESSRANGEMIN = 0;   // for performance this should be modifiable, can be min zero
  int I2CADDRESSRANGEMAX = 127; // for performance this should be modifiable, can be max 127
  bool MESSAGE_RECEIVED = false;
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  I2C INTERRUPT

/*

[this setup is for custom i2c peripherals]

1: Master is interrupted.
2: Master sweeps address range.
3: Response is parsed from slaves (strongly recommended to design custom slaves to clear their message buffers after sending message buffer).

Pros:
1: no need for slaves and masters to switch between master/slave mode.
2: no need to poll slaves in case they have a message (which they often may not).
3: requests are made when they need to be (because a slave has a messaage and so interrupted).
4: allows at least 127 custom sensor slaves for all kinds of things that can return data to the master.

Cons:
1: requires 3 wires (over 2 I2C wires) per slave (SDA, SCL, interrupt).

Note:
1: resistors would be required for multiple slaves interrupting on the same pin.
2: care should be taken so that slave messages do not conflict (slaves having the same messages). use of acronym names may be used in message content. 
3: scanning may be preferrable during development but in production may prefer explicitly addressing devices (scanning takes time).
   reducing scanner address range may also be more preferrable as a production solution, so that slaves can still share one interrupt pin and
   scan time is also reduced.
4: consideration should be payed to weather a slave should be on the general I2C extension bus (these devices should be able to interrupt), or
   on the I2C multilpexer (devices that can be polled satisfactorily), in which it is recommended to poll the I2C device on a channel in getSensorData.

*/

void ISR_I2C_PERIPHERAL() {
  // Serial.println("[ISR] ISR_I2C_PERIPHERAL"); // do not ever uncomment this unless when absolutely necessary (no stuff in isr)
  make_i2c_request = true;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      I2C WRITE

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
//                                                                                                                       I2C READ

void readI2C() {

  I2CLink.MESSAGE_RECEIVED = false;

  // make i2c request if interrupt flag true 
  if (make_i2c_request == true) {
    make_i2c_request = false;

    // -------------------------------------------------
    //                            SCAN I2C ADDRESS RANGE

    I2CLink.I2CADDRESSINDEX = I2CLink.I2CADDRESSRANGEMIN;
    for (I2CLink.I2CADDRESSINDEX; I2CLink.I2CADDRESSINDEX<I2CLink.I2CADDRESSRANGEMAX; I2CLink.I2CADDRESSINDEX++) {

      // -------------------------------------------------
      //                           TEST WITH EMPTY MESSAGE

      // compile empty bytes array: custom peripherals read bytes unitl (we should send them something when we test or they will hang)
      memset(I2CLink.TMP_BUFFER_0, 0, sizeof(I2CLink.TMP_BUFFER_0));

      // begin test
      Wire.beginTransmission(I2CLink.I2CADDRESSINDEX);
      for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i] = (byte)I2CLink.TMP_BUFFER_0[i];}
      
      // write bytes array
      Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));

      // end test
      int error = Wire.endTransmission();

      // -------------------------------------------------
      //                                      MAKE REQUEST

      // check test reults
      if (error == 0){

        Serial.println("[I2C] address found: " + String(I2CLink.I2CADDRESSINDEX));

        // make a request from found device
        Serial.println("[master] requesting from address: " + String(I2CLink.I2CADDRESSINDEX));
        Wire.requestFrom(I2CLink.I2CADDRESSINDEX, sizeof(I2CLink.INPUT_BUFFER));

        // receive from found device
        memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
        Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
        Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));

        // break if message or keep scanning if message empty (and dont parse the message in loop)
        if (!strcmp(I2CLink.INPUT_BUFFER, "")==0) {I2CLink.MESSAGE_RECEIVED=true; break;}
      }
    }

    // -------------------------------------------------
    //                             PARSE MESSAGE CONTENT

    if (I2CLink.MESSAGE_RECEIVED==true) {
      I2CLink.MESSAGE_RECEIVED=false;
      updateui_content = true;

      // record time of any activity from the i2c control panel.
      unixtime_control_panel_request = rtc.now().unixtime();
      Serial.println("[unixtime_control_panel_request] " + String(unixtime_control_panel_request));

      // blind button press protection: ignore button presses when screen is a sleep/off/blank (some buttons may be moved out of this block)
      if (update_ui==true) {

        // -------------------------------------------------
        //                                       CONTROL PAD

        // parse special interrupt buttons
        if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,I0")==0) {Serial.println("[button] ISR0");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,I1")==0) {Serial.println("[button] ISR1");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,I2")==0) {Serial.println("[button] ISR2");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,I3")==0) {Serial.println("[button] ISR3");}

        // parse numpad buttons
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,0")==0) {Serial.println("[button] 0"); inputChar(digit_0);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,1")==0) {Serial.println("[button] 1"); inputChar(digit_1);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,2")==0) {Serial.println("[button] 2"); inputChar(digit_2);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,3")==0) {Serial.println("[button] 3"); inputChar(digit_3);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,4")==0) {Serial.println("[button] 4"); inputChar(digit_4);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,5")==0) {Serial.println("[button] 5"); inputChar(digit_5);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,6")==0) {Serial.println("[button] 6"); inputChar(digit_6);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,7")==0) {Serial.println("[button] 7"); inputChar(digit_7);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,8")==0) {Serial.println("[button] 8"); inputChar(digit_8);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,9")==0) {Serial.println("[button] 9"); inputChar(digit_9);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,10")==0) {Serial.println("[button] 10: ."); inputChar(period_char);}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,11")==0) {Serial.println("[button] 11: -"); inputChar(hyphen_char);}

        // parse navigation buttons
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,12")==0) {Serial.println("[button] 12: home"); menu_page=page_home;}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,13")==0) {Serial.println("[button] 13: up"); menuUp();}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,14")==0) {Serial.println("[button] 14: right"); menuRight();}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,15")==0) {Serial.println("[button] 15: down"); menuDown();}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,16")==0) {Serial.println("[button] 16: left"); menuLeft();}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,17")==0) {Serial.println("[button] 17: enter"); menuEnter();}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,18")==0) {Serial.println("[button] 18: delete"); if (allow_input_data==true) {input_data[strlen(input_data)-1]='\0';}}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,19")==0) {Serial.println("[button] 19: back"); menuBack();}

        // parse currently spare creative potential buttons: (auto input with set_var_x set_var_y set_var_z) (clear)
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,20")==0) {Serial.println("[button] 20");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,21")==0) {Serial.println("[button] 21");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,22")==0) {Serial.println("[button] 22");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,23")==0) {Serial.println("[button] 23");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,24")==0) {Serial.println("[button] 24");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,25")==0) {Serial.println("[button] 25");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,26")==0) {Serial.println("[button] 26");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,27")==0) {Serial.println("[button] 27");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,28")==0) {Serial.println("[button] 28");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,29")==0) {Serial.println("[button] 29");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,30")==0) {Serial.println("[button] 30");}
        else if (strcmp(I2CLink.INPUT_BUFFER, "$CP,B,31")==0) {Serial.println("[button] 31");}
      }

      // -------------------------------------------------
      //                               OBJECT DETECTION AI

      // example
      // if      (strcmp(I2CLink.INPUT_BUFFER, "$OD,0,1")==0) {Serial.println("[object detection ai] object 0: true"); object_0=true;}
      // else if (strcmp(I2CLink.INPUT_BUFFER, "$OD,0,0")==0) {Serial.println("[object detection ai] object 0: false"); object_0=false;}

    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                PORT CONTROLLER

void writeToPortController() {

  // debug("[writeToPortController]");

  // Port Map: $P,X,Y
  for (int i=0; i < 20; i++) {
    // debug("[matrix_port_map] " + String(matrixData.matrix_port_map[0][i]) + " [tmp_matrix_port_map] " + String(matrixData.tmp_matrix_port_map[0][i]));
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

      // debug("[matrix_port_map writing] " + String( I2CLink.TMP_BUFFER_0));

      writeI2C(I2C_ADDR_PORTCONTROLLER_0);
    }
  }

  // Matrix Switch True/False: $M,X,Y
  for (int i=0; i < 20; i++) {
    // debug("[matrix_switch_state] " + String(matrixData.matrix_switch_state[0][i]) + " [tmp_matrix_switch_state] " + String(matrixData.tmp_matrix_switch_state[0][i]));
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

      // debug("[matrix_switch_state writing] " + String(I2CLink.TMP_BUFFER_0));

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

  // debug("[overload writing] " + String(I2CLink.TMP_BUFFER_0));

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
  
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("[sdcard] failed to initialize");
  }
  else {
    // debug("[sdcard] initialized");

    // create/load system files
    sdcard_mkdirs();

    // load system configuration file
    if (!sdcard_load_system_configuration(sdcardData.sysconf)) {
      sdcard_save_system_configuration(sdcardData.sysconf);
    }

    // load matrix file specified by configuration file
    if (!sdcard_load_matrix(sdcardData.matrix_filepath)) {
      Serial.println("[sdcard] specified matrix file not found!");

      // is it the the default matrix file that is missing?
      if (strcmp(sdcardData.matrix_filepath, sdcardData.default_matrix_filepath)==0) {
        Serial.println("[sdcard] default matrix file not found!");
        
        // create default matrix file
        if (!sdcard_save_matrix(sdcardData.matrix_filepath)) {Serial.println("[sdcard] failed to write default marix file.");}
        else if (!sdcard_load_matrix(sdcardData.default_matrix_filepath)) {Serial.println("[sdcard] failed to load matrix file");}
      }
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  SDCARD: CHECK

bool sdcardCheck() {
  /* a quick check to see if card can begin. cardBegin should return 1 or 0 */
  Serial.println("[sdcard] cardBegin: " + String(sd.cardBegin(SD_CONFIG)));
  if (!sd.cardBegin(SD_CONFIG)) {
    Serial.println("[sdcard] could not begin card");
  }
  else {
    Serial.println("[sdcard] sdcard began");
    return true;
  }
  return false;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                SERIAL COMMANDS

/* this is where we can accept input over serial in order to program the device or simply return information */

// void readSerial0() {
//   // Serial.println("[readSerial0] ");

//   // 
//   if (Serial.available()) {
//     memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
//     SerialLink.nbytes = Serial.readBytesUntil('\n', SerialLink.BUFFER, sizeof(SerialLink.BUFFER));
//     // Serial.println("$" + String(SerialLink.BUFFER)); // debug

//     if (SerialLink.nbytes > 1) {
//       if (systemData.allow_debug_bridge==true) {
//         if (strncmp(SerialLink.BUFFER, "test", strlen("test")) == 0) {Serial.println("[command] running test");}

//         // // command token
//         // sdcardData.token = strtok(SerialLink.BUFFER, " ");

//         // if (strncmp(sdcardData.token, "ls", strlen("ls")) == 0) {Serial.println("[command] ls");
//         //   sdcardData.token = strtok(NULL, " ");
//         //   if (sdcardData.token!=NULL) {memset(cwd, 0, sizeof(cwd)); strcpy(cwd, sdcardData.token);}
//         //   // ls();
//         //   }
//       }
//       else {Serial.println("[access] denied.");}
//     }
//   }
// }

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  READ GPS DATA

// void check_gngga() {
//   // debug("[check_gngga]");
//   if (systemData.gngga_enabled == true){
//     if (systemData.output_gngga_enabled==true) {Serial.println(gnggaData.sentence);}
//     gnggaData.valid_checksum = validateChecksum(gnggaData.sentence);
//     // debug("[gnggaData.sentence] " + String(gnggaData.sentence));
//     // debug("[gnggaData.valid_checksum] " + String(gnggaData.valid_checksum));
//     if (gnggaData.valid_checksum == true) {GNGGA();}
//     else {gnggaData.bad_checksum_validity++;}
//   }
// }

// void check_gnrmc() {
//   // debug("[check_gnrmc]");
//   if (systemData.gnrmc_enabled == true) {
//     if (systemData.output_gnrmc_enabled == true) {Serial.println(gnrmcData.sentence);}
//     gnrmcData.valid_checksum = validateChecksum(gnrmcData.sentence);
//     // debug("[gnrmcData.sentence] " + String(gnrmcData.sentence));
//     // debug("[gnrmcData.valid_checksum] " + String(gnrmcData.valid_checksum));
//     if (gnrmcData.valid_checksum == true) {GNRMC();}
//     else {gnrmcData.bad_checksum_validity++;}
//   }
// }

// void check_gpatt() {
//   // debug("[check_gpatt]");
//   if (systemData.gpatt_enabled == true) {
//     if (systemData.output_gpatt_enabled == true) {Serial.println(gpattData.sentence);}
//     gpattData.valid_checksum = validateChecksum(gpattData.sentence);
//     // debug("[gpattData.sentence] " + String(gpattData.sentence));
//     // debug("[gpattData.valid_checksum] " + String(gpattData.valid_checksum));
//     if (gpattData.valid_checksum == true) {GPATT();}
//     else {gpattData.bad_checksum_validity++;}
//   }
// }

int gps_done_t0;
int gps_done_t1;

void readGPS(void * pvParameters) {

  while (1) {

    // lock to avoid potential race conditions for yeilded gps data
    if (gps_done==false) {

      // ----------------------------------------------------------------------------------------------------------------------
      
      gps_done_t0 = micros();
      serial1Data.gngga_bool = false;
      serial1Data.gnrmc_bool = false;
      serial1Data.gpatt_bool = false;
      memset(gnggaData.sentence, 0, sizeof(gnggaData.sentence));
      memset(gnrmcData.sentence, 0, sizeof(gnrmcData.sentence));
      memset(gpattData.sentence, 0, sizeof(gpattData.sentence));

      // ----------------------------------------------------------------------------------------------------------------------

      /* this setup should read every sentence (gngga, desbi, gpatt, gnrmc) coming from the WTGPS300P once every 100ms. */

      // read up until attempt limit is reached. the aim here is to keep looping really fast for just what we need before handling the data later.
      // a while loop could be used with clauses to break.
      // for (int i_gps = 0; i_gps < 1000000000; i_gps++) {
      while (1) {
      // while (!serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {
        if (Serial2.available()) {

          // first check break for if we have everything we need
          if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
          // if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true) {break;}

          // read gps module to either terminating character or until the buffer is full
          memset(SerialLink.BUFFER, 0, sizeof(SerialLink.BUFFER));
          SerialLink.nbytes = Serial2.readBytesUntil('\n', SerialLink.BUFFER, sizeof(SerialLink.BUFFER));
          // Serial.println(SerialLink.BUFFER);

          // eliminate potential partial reads
          if (SerialLink.nbytes>10) {

            // ----------------------------------------------------------------------------------------------------------------------

            if (serial1Data.gngga_bool==false) {
              if (strncmp(SerialLink.BUFFER, "$GNGGA", 6) == 0) {
                if (systemData.gngga_enabled == true){
                  strcpy(gnggaData.sentence, SerialLink.BUFFER);
                  serial1Data.gngga_bool = true;
                  if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
                }
              }
            }
            // ----------------------------------------------------------------------------------------------------------------------

            if (serial1Data.gnrmc_bool==false) {
              if (strncmp(SerialLink.BUFFER, "$GNRMC", 6) == 0) {
                if (systemData.gnrmc_enabled == true){
                  strcpy(gnrmcData.sentence, SerialLink.BUFFER);
                  serial1Data.gnrmc_bool = true;
                  if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
                }
              }
            }
            // ----------------------------------------------------------------------------------------------------------------------

            if (serial1Data.gpatt_bool==false) {
              if (strncmp(SerialLink.BUFFER, "$GPATT", 6) == 0) {
                if (systemData.gpatt_enabled == true){
                  strcpy(gpattData.sentence, SerialLink.BUFFER);
                  serial1Data.gpatt_bool = true;
                  if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {break;}
                }
              }
            }
            // ----------------------------------------------------------------------------------------------------------------------
          }
        }
      }

      /* parse the above sentences fast so as not to miss each next sentence. then we can drop in here afterwards if we collected all the sentences */

      if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true && serial1Data.gpatt_bool==true) {
      // if (serial1Data.gngga_bool==true && serial1Data.gnrmc_bool==true) {

        // ----------------------------------------------------------------------------------------------------------------------

        if (systemData.gngga_enabled == true){
          if (systemData.output_gngga_enabled==true) {
            // store a copy before tokenization since this is a task we may prefer not to print here
            memset(gnggaData.outsentence, 0, sizeof(gnggaData.outsentence));
            strcpy(gnggaData.outsentence, gnggaData.sentence);
          }
          gnggaData.valid_checksum = validateChecksum(gnggaData.sentence);
          // debug("[gnggaData.valid_checksum] " + String(gnggaData.valid_checksum));
          if (gnggaData.valid_checksum == true) {GNGGA();}
          else {gnggaData.bad_checksum_validity++;}
        }

        // ----------------------------------------------------------------------------------------------------------------------
        
        if (systemData.gnrmc_enabled == true) {
          if (systemData.output_gnrmc_enabled == true) {
            // store a copy before tokenization since this is a task we may prefer not to print here
            memset(gnrmcData.outsentence, 0, sizeof(gnrmcData.outsentence));
            strcpy(gnrmcData.outsentence, gnrmcData.sentence);
          }
          gnrmcData.valid_checksum = validateChecksum(gnrmcData.sentence);
          // debug("[gnrmcData.valid_checksum] " + String(gnrmcData.valid_checksum));
          if (gnrmcData.valid_checksum == true) {GNRMC();}
          else {gnrmcData.bad_checksum_validity++;}
        }

        // ----------------------------------------------------------------------------------------------------------------------

        if (systemData.gpatt_enabled == true) {
          if (systemData.output_gpatt_enabled == true) {
            // store a copy before tokenization since this is a task we may prefer not to print here
            memset(gpattData.outsentence, 0, sizeof(gpattData.outsentence));
            strcpy(gpattData.outsentence, gpattData.sentence);
          }
          gpattData.valid_checksum = validateChecksum(gpattData.sentence);
          // debug("[gpattData.valid_checksum] " + String(gpattData.valid_checksum));
          if (gpattData.valid_checksum == true) {GPATT();}
          else {gpattData.bad_checksum_validity++;}
        }

        // ----------------------------------------------------------------------------------------------------------------------

        // is everything collected and validated? if so then reset the lock ready for other functions to use the data before going again
        if ((gnggaData.valid_checksum=true) && (gnrmcData.valid_checksum=true) && (gpattData.valid_checksum=true)) {
        // if ((gnggaData.valid_checksum=true) && (gnrmcData.valid_checksum=true)) {
          // debug("[gps_done_t] " + String(millis()-gps_done_t)); // debug
          gps_done_t1 = micros();
          gps_done=true;
        }

        // ----------------------------------------------------------------------------------------------------------------------
      }
    }
    delay(1);
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      SENSORS

void getSensorData() {


  // debug("[getSensorData] ");

  // step over each multiplexer analog/digital channel
  for (int i_chan = 0; i_chan < 16; i_chan++) {

    // set multiplexer channel
    setMultiplexChannel_CD74HC4067(i_chan);

    /*
    analogu read is default hardcode for reading each channel, this is for more general flexibility. more complicated sensors
    can be expanded upon here and new variables created in sensorData where requried.
    ESP32 has a 12bit ADC.
    10bit ADC: 0-1023
    12bit ADC: 0-4095
    14bit ADC: 0-16383
    */

    // sensor 0
    if (i_chan==0) {
      sensorData.dht11_h_0 = dht.readHumidity();
      sensorData.dht11_c_0 = dht.readTemperature();     // celsius default
      sensorData.dht11_f_0 = dht.readTemperature(true); // fahreheit = true
      if (isnan(sensorData.dht11_h_0) || isnan(sensorData.dht11_c_0) || isnan(sensorData.dht11_f_0)) {
        // debug("[dht11_hic_0] Failed to read from dht senor!");
      }
      sensorData.dht11_hif_0 = dht.computeHeatIndex(sensorData.dht11_f_0, sensorData.dht11_h_0);        // fahreheit default
      sensorData.dht11_hic_0 = dht.computeHeatIndex(sensorData.dht11_c_0, sensorData.dht11_h_0, false); // fahreheit = false
      sensorData.sensor_0 = sensorData.dht11_hic_0; // custum sensor 0
      // debug("[dht11_hic_0] " + String(sensorData.dht11_hic_0));
    }

    // sensor 0
    // if (i_chan==0) {
    //   sensorData.sensor_0 = analogRead(CD74HC4067_SIG);
    // }

    // sensor 1
    else if (i_chan==1) {
      sensorData.sensor_1 = analogRead(CD74HC4067_SIG);
    }

    // sensor 2
    else if (i_chan==2) {
      sensorData.sensor_2 = analogRead(CD74HC4067_SIG);
    }

    // sensor 3
    else if (i_chan==3) {
      sensorData.sensor_3 = analogRead(CD74HC4067_SIG);
    }

    // sensor 4
    else if (i_chan==4) {
      sensorData.sensor_4 = analogRead(CD74HC4067_SIG);
    }

    // sensor 5
    else if (i_chan==5) {
      sensorData.sensor_5 = analogRead(CD74HC4067_SIG);
    }

    // sensor 6
    else if (i_chan==6) {
      sensorData.sensor_6 = analogRead(CD74HC4067_SIG);
    }

    // sensor 7
    else if (i_chan==7) {
      sensorData.sensor_7 = analogRead(CD74HC4067_SIG);
    }

    // sensor 8
    else if (i_chan==8) {
      sensorData.sensor_8 = analogRead(CD74HC4067_SIG);
    }

    // sensor 9
    else if (i_chan==9) {
      sensorData.sensor_9 = analogRead(CD74HC4067_SIG);
    }

    // sensor 10
    else if (i_chan==10) {
      sensorData.sensor_10 = analogRead(CD74HC4067_SIG);
    }

    // sensor 11
    else if (i_chan==11) {
      sensorData.sensor_11 = analogRead(CD74HC4067_SIG);
    }

    // sensor 12
    else if (i_chan==12) {
      sensorData.sensor_12 = analogRead(CD74HC4067_SIG);
    }

    // sensor 13
    else if (i_chan==13) {
      sensorData.sensor_13 = analogRead(CD74HC4067_SIG);
    }

    // sensor 14
    else if (i_chan==14) {
      sensorData.sensor_14 = analogRead(CD74HC4067_SIG);
    }

    // sensor 15
    else if (i_chan==15) {
      sensorData.sensor_15 = analogRead(CD74HC4067_SIG);
    }
  }
  // set multiplexer channel back to zero
  setMultiplexChannel_CD74HC4067(0);

  // step over each multiplexer i2C channel
  for (int i_chan = 0; i_chan < 8; i_chan++) {

    // set multiplexer channel
    setMultiplexChannel_TCA9548A(i_chan);

    // i2c channel 0
    if (i_chan==0) {
    }

    // i2c channel 1
    else if (i_chan==1) {
    }

    // i2c channel 2
    else if (i_chan==2) {
    }

    // i2c channel 3
    else if (i_chan==3) {
    }

    // i2c channel 4
    else if (i_chan==4) {
    }

    // i2c channel 5
    else if (i_chan==5) {
    }

    // i2c channel 6
    else if (i_chan==6) {
    }

    // i2c channel 7
    else if (i_chan==7) {
    }
  }
  // set multiplexer channel back to zero
  setMultiplexChannel_TCA9548A(0);

  // build sensory data sentence
  if (systemData.output_sensors_enabled==true) {
    // Serial.println(
    //   "$SENSORS," +
    //   String(sensorData.sensor_0) + "," +
    //   String(sensorData.sensor_1) + "," +
    //   String(sensorData.sensor_2) + "," +
    //   String(sensorData.sensor_3) + "," +
    //   String(sensorData.sensor_4) + "," +
    //   String(sensorData.sensor_5) + "," +
    //   String(sensorData.sensor_6) + "," +
    //   String(sensorData.sensor_7) + "," +
    //   String(sensorData.sensor_8) + "," +
    //   String(sensorData.sensor_9) + "," +
    //   String(sensorData.sensor_10) + "," +
    //   String(sensorData.sensor_10) + "," +
    //   String(sensorData.sensor_11) + "," +
    //   String(sensorData.sensor_12) + "," +
    //   String(sensorData.sensor_13) + "," +
    //   String(sensorData.sensor_14) + "," +
    //   String(sensorData.sensor_15)
    // );
    memset(sensorData.sensor_sentence, 0, sizeof(sensorData.sensor_sentence));
    strcat(sensorData.sensor_sentence, "$SENSORS,");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_0, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_1, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_2, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_3, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_4, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_5, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_6, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_7, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_8, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_9, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_10, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_11, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_12, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_13, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_14, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    memset(sensorData.TMP, 0, sizeof(sensorData.TMP));
    ltoa(sensorData.sensor_15, sensorData.TMP, 10);
    strcat(sensorData.sensor_sentence, sensorData.TMP);
    strcat(sensorData.sensor_sentence, ",");

    // append checksum
    createChecksum(sensorData.sensor_sentence);
    strcat(sensorData.sensor_sentence, "*");
    strcat(sensorData.sensor_sentence, SerialLink.checksum);
    if (systemData.output_sensors_enabled == true) {Serial.println(sensorData.sensor_sentence);}
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
  Serial2.setTimeout(10); // ensure this is set before begin()
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

  
  setMultiplexChannel_CD74HC4067(0);
  
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

  // HSPI: SSD1351 OLED DISPLAY
  beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
  display.begin();
  display.setFixedFont(ssd1306xled_font6x8);
  display.fill( 0x0000 );
  canvas8x8.setFixedFont(ssd1306xled_font6x8);
  canvas19x8.setFixedFont(ssd1306xled_font6x8);
  canvas120x8.setFixedFont(ssd1306xled_font6x8);
  canvas120x120.setFixedFont(ssd1306xled_font6x8);
  canvas60x8.setFixedFont(ssd1306xled_font6x8);
  canvas28x8.setFixedFont(ssd1306xled_font6x8);
  canvas21x8.setFixedFont(ssd1306xled_font6x8);
  canvas36x8.setFixedFont(ssd1306xled_font6x8);
  canvas42x8.setFixedFont(ssd1306xled_font6x8);
  canvas54x8.setFixedFont(ssd1306xled_font6x8);
  canvas80x8.setFixedFont(ssd1306xled_font6x8);
  canvas92x8.setFixedFont(ssd1306xled_font6x8);
  display.clear();
  display.drawBitmap16(0, 0, 128, 128, UnidentifiedStudioBMP);

  // uncomment to test display
  // canvas.printFixed(1, 1, " SATIO ", STYLE_BOLD );
  // display.drawCanvas(1, 1, canvas);
  // display.clear();

  // END SPI DEVICE
  endSPIDevice(SSD1351_CS);

  // VSPI: SDCARD
  beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  setupSDCard();
  sd.end();
  endSPIDevice(SD_CS);

  // BEGIN SPI DEVICE
  beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS); 
  display.begin();
  display.clear();

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                      SETUP: SIDEREAL PLANETS

  myAstro.begin();

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                            SETUP: CORE TASKS

  // Create task to increase performance
  xTaskCreatePinnedToCore(
      readGPS, /* Function to implement the task */
      "GPSTask", /* Name of the task */
      10240,    /* Stack size in words */
      NULL,    /* Task input parameter */
      2,       /* Priority of the task */
      &GPSTask,  /* Task handle. */
      0);      /* Core where the task should run */
  
  // Create task to increase performance
  xTaskCreatePinnedToCore(
    UpdateUI, /* Function to implement the task */
    "Task1", /* Name of the task */
    102400,    /* Stack size in words */
    NULL,    /* Task input parameter */
    2,       /* Priority of the task */
    &Task1,  /* Task handle. */
    0);      /* Core where the task should run */
  
  // wait a moment before entering main loop
  delay(3000);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      MAIN LOOP

int t0 = millis();
bool track_planets_period = false;
bool check_sdcard = false;
bool longer_loop = false;
// int loop_distribution = 0;
/*
determine how many fast loops that may be utilized, occur during longer loops.
these loops will be counted up to every 100 ms and can be multiplied by 10 to get an idea of how many faster loops are available
every second that may be utilized for other things. like a seperate sensor matrix for example.
*/
long count_faster_loops = 0;

void loop() {
  bench("-----");
  timeData.mainLoopTimeStart = micros();
  systemData.t_bench = true; // uncomment to observe timings
  // count_faster_loops++;

  // ------------------------------------------------------------------------------------------
  //                                                                                        GPS
  /*
  Efficiency and performance.
  Only run the following block when new GPS data has been collected.
  GPS data from wtgps300p is every 100ms, aim to keep loop time under 100ms.
  */
  longer_loop = false;
  if (gps_done==true) {
    longer_loop = true; // set load distribution flag
    
    // ---------------------------------------------------------------------
    //                                                 SUSPEND READ GPS TASK
    /*
    Do not allow values to be changed while we use the GPS data!
    Avert race conditions while using GPS data.
    */
    vTaskSuspend(GPSTask);
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    //                                                   GPS SENTENCE OUTPUT
    /*
    Read GPS is running on a task so print the data here for safe output.
    Only run if new GPS data has been collected.
    */
    if (systemData.output_gngga_enabled==true) {Serial.println(gnggaData.outsentence);}
    if (systemData.output_gnrmc_enabled==true) {Serial.println(gnrmcData.outsentence);}
    if (systemData.output_gpatt_enabled==true) {Serial.println(gpattData.outsentence);}

    bench("[gps_done_t] " + String((float)(gps_done_t1-gps_done_t0)/1000000, 4) + "s");
    // bench("[count_faster_loops] " + String(count_faster_loops));
    // count_faster_loops=0;
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    //                                                           CONVERSIONS
    /*
    Convert absolute latitude and longitude to degrees.
    convert UTC to local time.
    Sync RTC.
    Only run if new GPS data has been collected.
    */
    t0 = micros();
    convertUTCToLocal();
    bench("[convertUTCToLocal] " + String((float)(micros()-t0)/1000000, 4) + "s");

    t0 = micros();
    calculateLocation();
    bench("[calculateLocation] " + String((float)(micros()-t0)/1000000, 4) + "s");
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    //                                                         MATRIX SWITCH
    /*
    Check users programmable logic.
    Never run while values are being updated.
    Only run if new GPS data has been collected.
    */
    t0 = micros();
    if (systemData.matrix_enabled == true) {matrixSwitch();}
    MatrixStatsCounter();
    bench("[matrixSwitch] " + String((float)(micros()-t0)/1000000, 4) + "s");
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    //                                                  RESUME READ GPS TASK
    /*
    Aim to set this true as soon as possible and never before we are
    finished using the GPS data.
    */
    gps_done = false;
    vTaskResume(GPSTask);
    // ---------------------------------------------------------------------

    // ---------------------------------------------------------------------
    //                                                        SATIO SENTENCE
    /*
    Create and output special SatIO sentence over serial.
    Only run if new GPS data has been collected.
    */
    t0 = micros();
    if (systemData.satio_enabled == true) {buildSatIOSentence();}
    bench("[buildSatIOSentence] " + String((float)(micros()-t0)/1000000, 4) + "s");
    // ---------------------------------------------------------------------
  }
  // --------------------------------------------------------------------------------------------

  
  // ---------------------------------------------------------------------
  //                                                           SENSOR DATA
  /*
  Collect sensor data that could be utilized every loop.
  Run every loop.
  */
  t0 = micros();
  getSensorData();
  bench("[getSensorData] " + String((float)(micros()-t0)/1000000, 4) + "s");
  // ---------------------------------------------------------------------

  // ---------------------------------------------------------------------
  //                                                       PORT CONTROLLER
  /*
  Port controller to be utilized every loop.
  Run every loop.
  */
  t0 = micros();
  if (systemData.port_controller_enabled == true) {writeToPortController();}
  bench("[writePortController] " + String((float)(micros()-t0)/1000000, 4) + "s");
  // ---------------------------------------------------------------------

  // ---------------------------------------------------------------------
  //                                                                SDCARD
  /*
  Check if sd card is present.
  Currently commented because we are so extensible that we will literally see
  this check happening (ui ending/beginning) due to SPI switching.
  */
  // t0 = micros();
  // endSPIDevice(SSD1351_CS);
  // display.end();
  // beginSPIDevice(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  // sdcard_initialized = sdcardCheck();
  // sd.end();
  // endSPIDevice(SD_CS);
  // beginSPIDevice(SSD1351_SCLK, SSD1351_MISO, SSD1351_MOSI, SSD1351_CS);
  // display.begin();
  // bench("[sdcardCheck] " + String((float)(micros()-t0)/1000000, 4) + "s");
  // delay(50);
  // ---------------------------------------------------------------------

  // ---------------------------------------------------------------------
  //                                                     LOAD DISTRIBUTION
  /*
  Efficiency and performance.
  Distribute functions between faster loops.
  Utilizes loops that did not perform GPS calculations and conversions.
  GPS data from wtgps300p is every 100ms, aim to keep loop time under 100ms.
  Run every loop.
  */
  if (longer_loop==false) {

    // update ui: where possible try to avoid writing a lot of pixels, or take a performance hit
    // if (loop_distribution==0) {
    //   loop_distribution=1;
    //   t0 = micros();
    //   // UpdateUI();
    //   bench("[// UpdateUI] " + String((float)(micros()-t0)/1000000, 4) + "s");
    // }

    // track planets
    // else if (loop_distribution==1) {
    //   loop_distribution=0;
      if (track_planets_period == true) {
        track_planets_period = false;
        t0 = micros();
        setTrackPlanets();
        bench("[setTrackPlanets] " + String((float)(micros()-t0)/1000000, 4) + "s");
        t0 = micros();
        trackPlanets();
        bench("[trackPlanets] " + String((float)(micros()-t0)/1000000, 4) + "s");
      }
    // }
  }
  // ---------------------------------------------------------------------


  // ---------------------------------------------------------------------
  //                                                    ISR SECOND COUNTER
  if (interrupt_second_counter > 0) {
    portENTER_CRITICAL(&second_timer_mux);
    interrupt_second_counter--;
    track_planets_period = true;
    check_sdcard = true;
    timeData.uptime_seconds++;
    portEXIT_CRITICAL(&second_timer_mux);
  }

  // ---------------------------------------------------------------------
  //                                                               TIMINGS
  // delay(100); // debug test overload: increase loop time
  timeData.mainLoopTimeTaken = (micros() - timeData.mainLoopTimeStart);
  if (timeData.mainLoopTimeTaken>=100000) {systemData.overload=true;} // gps module outputs every 100ms (100,000uS)
  else {systemData.overload=false;}
  if (timeData.mainLoopTimeTaken > timeData.mainLoopTimeTakenMax) {timeData.mainLoopTimeTakenMax = timeData.mainLoopTimeTaken;}
  // if ((timeData.mainLoopTimeTaken < timeData.mainLoopTimeTakenMin) && (timeData.mainLoopTimeTaken>0)) {timeData.mainLoopTimeTakenMin = timeData.mainLoopTimeTaken;}
  bench("[overload] " + String(systemData.overload));
  bench("[Looptime] " + String((float)(timeData.mainLoopTimeTaken)/1000000, 4) + "s");
  bench("[Looptime Max] " + String((float)(timeData.mainLoopTimeTakenMax)/1000000, 4) + "s");
  // bench("[Looptime Min] " + String(timeData.mainLoopTimeTakenMin, 6) + " uS");
  // ---------------------------------------------------------------------
}
