/*

                                           SATIO - Written by Benjamin Jack Cullen.

                                  A general purpose programmable satellite and inertial switch. 

                Receives and Processes Transmissions from Satellites and makes the data available for calculations.

                Possible combinations example: 100 checks ^ 10 functions = 100,000,000,000,000,000,000 combinations.
                                                                           100 Quintillion.
            Currently there are over 200 different checks that can be performed using just several small primitive functions and
             currently each relays activation/deactivaion can occur based on up to 10 different checks resulting true or false. 
                                      
                                        Wiring For ESP32-2432S028 development board (CYD)
                  
                  WTGPS300P TX --> CYD io22 as RXD (requires implemented Serial1.setPins() to work on CYD)

                                                      SENTENCE $SATIO
                                                                                    
                      START Tag                Last Sat Time                    Converted Longitude        
                         |                   |               |                   |               |                  
                      $SATIO,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                             |               |               |                 |                              
                               DatetimeStamp                  Converted Latitude                                 


                        Ultimately this system is being built as a unit to turn on/off multiplexed relays,
                     where potentially anything can be plugged in such as simple modules or pre-programmed MCU's, 
               making a foundation for other creative projects that may make use of such satellite and or inertial data.
               The idea is that each relay is controlled by a compound of logic (limited by memory), and the logic itself
               is programmable before and after flashing. Allowing for a reusable and general purpose system for any future
               projects requiring such data. more advanced calculations are intended, such as emphemeris, astronomical etc. now
               that the foundational data has been handled. Then finally when fitted with relays, this system can be a self
               contained unit, behaving as a switch for other devices. Robots and flying machines and everything in between.

               The relays are currently simulated while the system is being built and the simulation is designed to be easily
               relaceable by the actual relays themselves once the logic has been completed to a satisfactory degree and currently
               all logic required to activate and deactivate relays is in place.

               Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
               of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)
*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include <XPT2046_Bitbang.h>
#include <TFT_eSPI.h>
#include <SiderealPlanets.h>  // https://github.com/DavidArmstrong/SiderealPlanets
#include <SiderealObjects.h>
#include "FS.h"
#include "SD.h"

const int8_t ctsPin = -1;
const int8_t rtsPin = -1;
const byte gpsrxpin = 22;
const byte gpstxpin = 27;

// ----------------------------
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

XPT2046_Bitbang ts(XPT2046_MOSI, XPT2046_MISO, XPT2046_CLK, XPT2046_CS);

TFT_eSPI tft = TFT_eSPI();

TFT_eSPI_Button key[6];

bool isTouched = false;

TFT_eSprite hud = TFT_eSprite(&tft);

TaskHandle_t TSTask;
TaskHandle_t UpdateDisplayTask;

#define TFTOBJ_COL0 TFT_DARKGREY

#define TFTTXT_COLF_0 TFT_DARKGREY
#define TFTTXT_COLB_0 TFT_BLACK

#define TFTTXT_COLF_1 TFT_BLACK
#define TFTTXT_COLB_1 TFT_DARKGREY

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             SIDEREAL PLANETS

// SiderealPlanets myAstro;    // for calculating azimuth and altitude
// SiderealObjects myAstroObj; // for getting right ascension and declination of objects from star table

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       SDCARD

SPIClass sdspi = SPIClass(VSPI);

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          ETX
#define ETX             0x03 // end of text character
#define NL              0x0a
unsigned long T0_TXD_2 = 0; // hard throttle current time
unsigned long T1_TXD_2 = 0; // hard throttle previous time
unsigned long TT_TXD_2 = 500; // hard throttle interval

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 DATA: SYSTEM

struct systemStruct {
  bool satio_enabled = true;
  bool gngga_enabled = true;
  bool gnrmc_enabled = true;
  bool gpatt_enabled = true;
  bool matrix_enabled = false;
  bool autoresume_enabled = false;
  bool output_satio_enabled = false;
  bool output_gngga_enabled = false;
  bool output_gnrmc_enabled = false;
  bool output_gpatt_enabled = false;
  bool output_matrix_enabled = false;

  bool sidereal_track_sun = true;
  bool sidereal_track_moon = true;
  bool sidereal_track_mercury = true;
  bool sidereal_track_venus = true;
  bool sidereal_track_mars = true;
  bool sidereal_track_jupiter = true;
  bool sidereal_track_saturn = true;
  bool sidereal_track_uranus = true;
  bool sidereal_track_neptune = true;
  
  bool display_low_light = false;
  bool display_flip_vertically = true;
  bool display_auto_dim = true; // (OLED burn-in protection)
  int           display_auto_dim_p0 = 3000;
  unsigned long display_auto_dim_t0;
  unsigned long display_auto_dim_t1;
  bool          display_dim = false;
  bool display_auto_off = false; // (OLED burn-in protection)
  int           display_auto_off_p0 = 20000;
  unsigned long display_auto_off_t0;
  unsigned long display_auto_off_t1;
  bool          display_on = true;
  char translate_enable_bool[1][2][56] = { {"DISABLED", "ENABLED"} };
  char translate_plus_minus[1][2][56]  = { {"ADD", "DEDUCT"} };
};
systemStruct systemData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  DATA: DEBUG

struct sysDebugStruct {
  bool gngga_sentence = false;
  bool gnrmc_sentence = false;
  bool gpatt_sentence = false;
  bool serial_0_sentence = true;
  bool validation = false;
};
sysDebugStruct sysDebugData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: MENU

struct menuStruct {
  int y = 0;
  int x = 1;
  int menu_max_y0 = 6;
  int menu_max_x0 = 3;
  int numpad_y = 0;
  int numpad_x = 0;
  int menu_numpad_max_y0 = 6;
  int menu_numpad_max_x0 = 3;
  char input[2024];
  int numpad_key = NULL;
  bool select = false;
  int page = 0;
  int backpage = 0;
  int page_max = 8;
  int relay_select = 0;
  int relay_function_select = 0;
  int function_index = 0;
  int matrix_filenames_index = 0;
  bool menu_lock = false;
  int isr_i = 0;
};
menuStruct menuData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               DATA: SERIAL 0

struct Serial0Struct {
  unsigned long nbytes;
  unsigned long iter_token;
  char BUFFER[1024];
  char * token = strtok(BUFFER, ",");
  char data_0[56];
  char data_1[56];
  char data_2[56];
  char data_3[56];
  char data_4[56];
  char data_5[56];
  char data_6[56];
  int check_data_R;
};
Serial0Struct serial0Data;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               DATA: SERIAL 1

struct Serial1Struct {
  unsigned long nbytes;
  unsigned long iter_token;
  char BUFFER[2000];
  char * token = strtok(BUFFER, ",");
  int collected = 0;
  bool gngga_bool = false;
  bool gnrmc_bool = false;
  bool gpatt_bool = false;
};
Serial1Struct serial1Data;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 DATA: SDCARD

struct SDCardStruct {
  int max_matrix_filenames = 20;
  char matrix_filenames[20][56] = {
    "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
    };
  char sysconf[56] = "/SYSTEM/SYSTEM.CONFIG";
  int matrix_filename_i = 0;
  char default_matrix_filepath[56] = "/MATRIX/MATRIX_0.SAVE";
  char matrix_filepath[56] = "";
  char tempmatrixfilepath[56];
  char system_dirs[2][56] = {"/MATRIX", "/SYSTEM"};
  File root;
  unsigned long nbytes;
  unsigned long iter_token;
  char BUFFER[2048];
  String SBUFFER;
  char * token = strtok(BUFFER, ",");
  char data_0[56];
  char data_1[56];
  char data_2[56];
  char data_3[56];
  char data_4[56];
  char data_5[56];
  char data_6[56];
  int check_data_R;
  char file_data[256];
  char delim[2] = ",";
  char tmp[256];
  char tag_0[56] = "r";
  char tag_1[56] = "e";
  File current_file;
};
SDCardStruct sdcardData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   DATA: TIME

struct TimeStruct {
  unsigned long seconds;
  unsigned long mainLoopTimeTaken;
  unsigned long mainLoopTimeStart;
  unsigned long mainLoopTimeTakenMax;
  unsigned long mainLoopTimeTakenMin;
  unsigned long t0;
  unsigned long t1;
};
TimeStruct timeData;

void time_counter() {
  timeData.t0 = micros();
  if (timeData.t0 > (timeData.t1+1000000)) {
    timeData.t1 = timeData.t0;
    timeData.seconds++;
    }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             DATA: VALIDATION

struct validationStruct {
  int  valid_i = 0;
  bool valid_b = true;
  char *find_char;
  int  index;
  bool bool_data_0 = false;
  bool bool_data_1 = false;
};
validationStruct validData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                         VALIDATION: CHECKSUM

int getCheckSum(char * string) {
  if (sysDebugData.validation == true) {Serial.println("[connected] getCheckSum: " + String(string));}
  int i;
  int XOR;
  int c;
  for (XOR = 0, i = 0; i < strlen(string); i++) {
    c = (unsigned char)string[i];
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }
  if (sysDebugData.validation == true) {Serial.println("[connected] getCheckSum: " + String(XOR));}
  return XOR;
}

uint8_t h2d(char hex) {if(hex > 0x39) hex -= 7; return(hex & 0xf);}

uint8_t h2d2(char h1, char h2) {return (h2d(h1)<<4) | h2d(h2);}

bool validateChecksum(char * buffer) {
  if (sysDebugData.validation == true) {Serial.println("[connected] validateChecksum: " + String(buffer));}
  char gotSum[2];
  gotSum[0] = buffer[strlen(buffer) - 3];
  gotSum[1] = buffer[strlen(buffer) - 2];
  uint8_t checksum_of_buffer =  getCheckSum(buffer);
  uint8_t checksum_in_buffer = h2d2(gotSum[0], gotSum[1]);
  if (checksum_of_buffer == checksum_in_buffer) {return true;} else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             VALIDATION: DATA

/*
checks can be ellaborated upon individually.
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

bool is_all_digits_plus_char(char * data, char * find_char) {
  if (sysDebugData.validation == true) {Serial.println("[connected] is_all_digits_plus_char: " + String(data));}
  // designed to check all chars are digits except one period and is more general purpose than just accepting a period
  validData.valid_b = true;
  validData.find_char = strchr(data, * find_char);
  validData.index = (int)(validData.find_char - data);
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {if (i != validData.index) {validData.valid_b = false;}}}
  return validData.valid_b;
}

bool is_positive_negative_num(char * data) {
  if (sysDebugData.validation == true) {Serial.println("[connected] is_positive_negative_num: " + String(data));}
  // designed to check all chars are digits except one period and the signed bit. allows positive/negative floats, doubles and ints
  // allow one period anywhere.
  // allow one minus (-) sign at index zero.
  validData.valid_b = true;
  validData.find_char = strchr(data, '.');
  validData.index = (int)(validData.find_char - data);
  for (int i = 0; i < strlen(data); i++) {if (isdigit(data[i]) == 0) {if (i != validData.index) {if ((data[i] != '-') && (i > 0)) {validData.valid_b = false;}}}}
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
  if (is_all_digits_plus_char(data, ".") == true) {
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
  if (is_all_digits_plus_char(data, ".") == true) {
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
  if (is_all_digits_plus_char(data, ".") == true) {
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
  if (is_all_digits_plus_char(data, ".") == true) {
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
  if (is_all_digits_plus_char(data, ".") == true) {
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

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 DATA: RELAYS

struct RelayStruct {

  int MAobject_RELAYS = 20;
  int MAobject_RELAY_ELEMENTS = 10;

  int relays_enabled_i = 0;
  int relays_disabled_i = 0;
  int relays_active_i = 0;
  int relays_inactive_i = 0;

  char matrix_results_sentence[56];
  char checksum_str[56];
  int checksum_i;

  bool relays_bool[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };

  int relays_enable[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };

  unsigned long relays_timing[1][20] = {
    {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    }
  };
  
  char relays[20][10][100] = {
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 1
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
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 13
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 14
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 15
     },
    {"$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", "$NONE", // 16
     },
    };

  /*
  Matrix containing sets of values per relay.
  X: use with/without  Y,Z.
  Y: necessary if comparing to X.
  Z: necessary if checking X,Y in range of Z.  

          0     1     2     
          X     Y     Z    
  {  {   0.0,  0.0,  0.0   } }
  */
  double relays_data[20][10][3] = {
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
  };

  int FUNCTION_NAMES_MAX = 245;
  // convert function name chars to full human names, now there is a bigger screen, and create brief descriptions.
  // try to keep longest strlen <=23 so that it can always be displayed in full without needing to readjust any displayed/touch column widths.
  char function_names[245][56] = 
  {
    "$NONE",
    "$ENABLED",
    "SecondsTimer",
    "DegreesLatGNGGAOver",
    "DegreesLatGNGGAUnder",
    "DegreesLatGNGGAEqual",
    "DegreesLatGNGGARange",
    "DegreesGNGGARange",
    "DegreesLonGNGGAOver",
    "DegreesLonGNGGAUnder",
    "DegreesLonGNGGAEqual",
    "DegreesLonGNGGARange",
    "DegreesLatGNRMCOver",
    "DegreesLatGNRMCUnder",
    "DegreesLatGNRMCEqual",
    "DegreesLatGNRMCRange",
    "DegreesLonGNRMCOver",
    "DegreesLonGNRMCUnder",
    "DegreesLonGNRMCEqual",
    "DegreesGNRMCRange",
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
    "PositioningStatusGNGGA",
    "SatelliteCountOver",
    "SatelliteCountUnder",
    "SatelliteCountEqual",
    "SatelliteCountRange",
    "HemisphereGNGGANorth",
    "HemisphereGNGGAEast",
    "HemisphereGNGGASouth",
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
    "ModeGNRMCN",
    "LatGNRMCOver",
    "LatGNRMCUnder",
    "LatGNRMCEqual",
    "LatGNRMCRange",
    "LonGNRMCOver",
    "LonGNRMCUnder",
    "LonGNRMCEqual",
    "LonGNRMCRange",
    "HemisphereGNRMCNorth",
    "HemisphereGNRMCEast",
    "HemisphereGNRMCSouth",
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
    "InertialFlagGPATTEqual",
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
    "UBIStateFlagOver",
    "UBIStateFlagUnder",
    "UBIStateFlagEqual",
    "UBIStateKindOver",
    "UBIStateKindUnder",
    "UBIStateKindEqual",
    "UBIStateKindRange",
    "UBIStateValueOver",
    "UBIStateValueUnder",
    "UBIStateValueEqual",
    "UBIStateValueRange",
    "CollisionTHeadingOver",
    "CollisionTHeadingUnder",
    "CollisionTHeadingEqual",
    "CollisionTHeadingRange",
    "CollisionTDataOver",
    "CollisionTDataUnder",
    "CollisionTDataEqual",
    "CollisionTDataRange",
    "UBIValidEqual",
    "INSFlagOver",
    "INSFlagUnder",
    "INSFlagEqual",
    "INSFlagRange",
    "CarSpeedOver",
    "CarSpeedUnder",
    "CarSpeedEqual",
    "CarSpeedRange",
    "YawAngleOver",
    "YawAngleUnder",
    "YawAngleEqual",
    "YawAngleRange",
    "RollAngleOver",
    "RollAngleUnder",
    "RollAngleEqual",
    "RollAngleRange",
    "PitchAngleOver",
    "PitchAngleUnder",
    "PitchAngleEqual",
    "PitchAngleRange",
    "AngDGetFlagEqual",
    "INSRunFlagEqual",
    "FixRollFlagEqual",
    "FixPitchFlagEqual",
    "UBIKindFlagEqual",
    "UBIOnFlagOver",
    "UBIOnFlagUnder",
    "UBIOnFlagEqual",
    "UBIOnFlagRange",
    "UBIASetOver",
    "UBIASetUnder",
    "UBIASetEqual",
    "UBIASetRange",
    "UBIBSetOver",
    "UBIBSetUnder",
    "UBIBSetEqual",
    "UBIBSetRange",
    "AccXDataOver",
    "AccXDataUnder",
    "AccXDataEqual",
    "AccXDataRange",
    "AccYDataOver",
    "AccYDataUnder",
    "AccYDataEqual",
    "AccYDataRange",
    "GyroZDataOver",
    "GyroZDataUnder",
    "GyroZDataEqual",
    "GyroZDataRange",
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
    "SunAzimuth",
    "SunAltitide",
    "DayTime",
    "NightTime",
    "Sunrise",
    "Sunset",
    "MoonAzimuth",
    "MoonAltitide",
    "Moonrise",
    "Moonset",
    "MoonPhase",
    "MercuryAzimuth",
    "MercuryAltitide",
    "MercuryUp",
    "MercuryDown",
    "MercuryRise",
    "MercurySet",
    "VenusAzimuth",
    "VenusAltitide",
    "VenusUp",
    "VenusDown",
    "VenusRise",
    "VenusSet",
    "MarsAzimuth",
    "MarsAltitide",
    "MarsUp",
    "MarsDown",
    "MarsRise",
    "MarsSet",
    "JupiterAzimuth",
    "JupiterAltitide",
    "JupiterUp",
    "JupiterDown",
    "JupiterRise",
    "JupiterSet",
    "SaturnAzimuth",
    "SaturnAltitide",
    "SaturnUp",
    "SaturnDown",
    "SaturnRise",
    "SaturnSet",
    "UranusAzimuth",
    "UranusAltitide",
    "UranusUp",
    "UranusDown",
    "UranusRise",
    "UranusSet",
    "NeptuneAzimuth",
    "NeptuneAltitide",
    "NeptuneUp",
    "NeptuneDown",
    "NeptuneRise",
    "NeptuneSet"
  };

  // default and specifiable value to indicate a relay should not be activated/deactivated if all functions in relays expression are $NONE
  char default_relay_function[56]          = "$NONE";
  char default_enable_relay_function[56]   = "$ENABLED";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   SATIO DATA

  char DegreesLatGNGGAOver[56]             = "DegreesLatGNGGAOver";
  char DegreesLatGNGGAUnder[56]            = "DegreesLatGNGGAUnder";
  char DegreesLatGNGGAEqual[56]            = "DegreesLatGNGGAEqual";
  char DegreesLatGNGGARange[56]            = "DegreesLatGNGGARange";
  char DegreesGNGGARange[56]               = "DegreesGNGGARange";
  char DegreesLonGNGGAOver[56]             = "DegreesLonGNGGAOver";
  char DegreesLonGNGGAUnder[56]            = "DegreesLonGNGGAUnder";
  char DegreesLonGNGGAEqual[56]            = "DegreesLonGNGGAEqual";
  char DegreesLonGNGGARange[56]            = "DegreesLonGNGGARange";
  char DegreesLatGNRMCOver[56]             = "DegreesLatGNRMCOver";
  char DegreesLatGNRMCUnder[56]            = "DegreesLatGNRMCUnder";
  char DegreesLatGNRMCEqual[56]            = "DegreesLatGNRMCEqual";
  char DegreesLatGNRMCRange[56]            = "DegreesLatGNRMCRange";
  char DegreesLonGNRMCOver[56]             = "DegreesLonGNRMCOver";
  char DegreesLonGNRMCUnder[56]            = "DegreesLonGNRMCUnder";
  char DegreesLonGNRMCEqual[56]            = "DegreesLonGNRMCEqual";
  char longitude_satio_gnrmc_in_range[56] = "DegreesGNRMCRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNGGA DATA

  char UTCTimeGNGGAOver[56]       = "UTCTimeGNGGAOver";
  char UTCTimeGNGGAUnder[56]      = "UTCTimeGNGGAUnder";
  char UTCTimeGNGGAEqual[56]      = "UTCTimeGNGGAEqual";
  char UTCTimeGNGGARange[56]      = "UTCTimeGNGGARange";
  char LatGNGGAOver[56]           = "LatGNGGAOver";
  char LatGNGGAUnder[56]          = "LatGNGGAUnder";
  char LatGNGGAEqual[56]          = "LatGNGGAEqual";
  char LatGNGGARange[56]          = "LatGNGGARange";
  char LonGNGGAOver[56]           = "LonGNGGAOver";
  char LonGNGGAUnder[56]          = "LonGNGGAUnder";
  char LonGNGGAEqual[56]          = "LonGNGGAEqual";
  char LonGNGGARange[56]          = "LonGNGGARange";
  char PositioningStatusGNGGA[56] = "PositioningStatusGNGGA";
  char SatelliteCountOver[56]     = "SatelliteCountOver";
  char SatelliteCountUnder[56]    = "SatelliteCountUnder";
  char SatelliteCountEqual[56]    = "SatelliteCountEqual";
  char SatelliteCountRange[56]    = "SatelliteCountRange";
  char HemisphereGNGGANorth[56]   = "HemisphereGNGGANorth";
  char HemisphereGNGGAEast[56]    = "HemisphereGNGGAEast";
  char HemisphereGNGGASouth[56]   = "HemisphereGNGGASouth";
  char HemisphereGNGGAWest[56]    = "HemisphereGNGGAWest";
  char GPSPrecisionOver[56]       = "GPSPrecisionOver";
  char GPSPrecisionUnder[56]      = "GPSPrecisionUnder";
  char GPSPrecisionEqual[56]      = "GPSPrecisionEqual";
  char GPSPrecisionRange[56]      = "GPSPrecisionRange";
  char AltitudeGNGGAOver[56]      = "AltitudeGNGGAOver";
  char AltitudeGNGGAUnder[56]     = "AltitudeGNGGAUnder";
  char AltitudeGNGGAEqual[56]     = "AltitudeGNGGAEqual";
  char AltitudeGNGGARange[56]     = "AltitudeGNGGARange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GNRMC DATA

  char UTCTimeGNRMCOver[56]         = "UTCTimeGNRMCOver";
  char UTCTimeGNRMCUnder[56]        = "UTCTimeGNRMCUnder";
  char UTCTimeGNRMCEqual[56]        = "UTCTimeGNRMCEqual";
  char UTCTimeGNRMCRange[56]        = "UTCTimeGNRMCRange";
  char PositioningStatusGNRMCA[56]  = "PositioningStatusGNRMCA";
  char PositioningStatusGNRMCV[56]  = "PositioningStatusGNRMCV";
  char ModeGNRMCA[56]               = "ModeGNRMCA";
  char ModeGNRMCD[56]               = "ModeGNRMCD";
  char ModeGNRMCN[56]               = "ModeGNRMCN";
  char LatGNRMCOver[56]             = "LatGNRMCOver";
  char LatGNRMCUnder[56]            = "LatGNRMCUnder";
  char LatGNRMCEqual[56]            = "LatGNRMCEqual";
  char LatGNRMCRange[56]            = "LatGNRMCRange";
  char LonGNRMCOver[56]             = "LonGNRMCOver";
  char LonGNRMCUnder[56]            = "LonGNRMCUnder";
  char LonGNRMCEqual[56]            = "LonGNRMCEqual";
  char LonGNRMCRange[56]            = "LonGNRMCRange";
  char HemisphereGNRMCNorth[56]     = "HemisphereGNRMCNorth";
  char HemisphereGNRMCEast[56]      = "HemisphereGNRMCEast";
  char HemisphereGNRMCSouth[56]     = "HemisphereGNRMCSouth";
  char HemisphereGNRMCWest[56]      = "HemisphereGNRMCWest";
  char GroundSpeedGNRMCOver[56]     = "GroundSpeedGNRMCOver";
  char GroundSpeedGNRMCUnder[56]    = "GroundSpeedGNRMCUnder";
  char GroundSpeedGNRMCEqual[56]    = "GroundSpeedGNRMCEqual";
  char GroundSpeedGNRMCRange[56]    = "GroundSpeedGNRMCRange";
  char HeadingGNRMCOver[56]         = "HeadingGNRMCOver";
  char HeadingGNRMCUnder[56]        = "HeadingGNRMCUnder";
  char HeadingGNRMCEqual[56]        = "HeadingGNRMCEqual";
  char HeadingGNRMCRange[56]        = "HeadingGNRMCRange";
  char UTCDateGNRMCOver[56]         = "UTCDateGNRMCOver";
  char UTCDateGNRMCUnder[56]        = "UTCDateGNRMCUnder";
  char UTCDateGNRMCEqual[56]        = "UTCDateGNRMCEqual";
  char UTCDateGNRMCRange[56]        = "UTCDateGNRMCRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                   GPATT DATA

  char InertialFlagGPATTEqual[56] = "InertialFlagGPATTEqual";
  char LineFlagGPATTEqual[56]     = "LineFlagGPATTEqual";
  char StaticFlagGPATTEqual[56]   = "StaticFlagGPATTEqual";
  char RunStateFlagGPATTEqual[56] = "RunStateFlagGPATTEqual";
  char INSGPATTEqual[56]          = "INSGPATTEqual";
  char SpeedNumGPATTOver[56]      = "SpeedNumGPATTOver";
  char SpeedNumGPATTUnder[56]     = "SpeedNumGPATTUnder[";
  char SpeedNumGPATTEqual[56]     = "SpeedNumGPATTEqual";
  char SpeedNumGPATTRange[56]     = "SpeedNumGPATTRange";
  char MileageGPATTOver[56]       = "MileageGPATTOver";
  char MileageGPATTUnder[56]      = "MileageGPATTUnder[";
  char MileageGPATTEqual[56]      = "MileageGPATTEqual";
  char MileageGPATTRange[56]      = "MileageGPATTRange";
  char GSTDataGPATTOver[56]       = "GSTDataGPATTOver";
  char GSTDataGPATTUnder[56]      = "GSTDataGPATTUnder[";
  char GSTDataGPATTEqual[56]      = "GSTDataGPATTEqual";
  char GSTDataGPATTRange[56]      = "GSTDataGPATTRange";
  char YawGPATTOver[56]           = "YawGPATTOver";
  char YawGPATTUnder[56]          = "YawGPATTUnder[";
  char YawGPATTEqual[56]          = "YawGPATTEqual";
  char YawGPATTRange[56]          = "YawGPATTRange";
  char RollGPATTOver[56]          = "RollGPATTOver";
  char RollGPATTUnder[56]         = "RollGPATTUnder[";
  char RollGPATTEqual[56]         = "RollGPATTEqual";
  char RollGPATTRange[56]         = "RollGPATTRange";
  char PitchGPATTOver[56]         = "PitchGPATTOver";
  char PitchGPATTUnder[56]        = "PitchGPATTUnder[";
  char PitchGPATTEqual[56]        = "PitchGPATTEqual";
  char PitchGPATTRange[56]        = "PitchGPATTRange";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                            

  char SecondsTimer[56] = "SecondsTimer";

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                             SIDEREAL PLANETS

  char SunAzimuth[56]  = "SunAzimuth";
  char SunAltitude[56] = "SunAltitude";
  char DayTime[56]   = "DayTime";
  char NightTime[56] = "NightTime";
  char Sunrise[56]   = "Sunrise";
  char Sunset[56]    = "Sunset";

  char MoonAzimuth[56]  = "MoonAzimuth";
  char MoonAltitude[56] = "MoonAltitude";
  char MoonUp[56]    = "MoonUp";
  char MoonDown[56]  = "MoonDown";
  char Moonrise[56]  = "Moonrise";
  char Moonset[56]   = "Moonset";
  char MoonPhase[56] = "MoonPhase";

  char MercuryAzimuth[56]  = "MercuryAzimuth";
  char MercuryAltitude[56] = "MercuryAltitude";
  char MercuryUp[56]    = "MercuryUp";
  char MercuryDown[56]  = "MercuryDown";
  char MercuryRise[56]  = "MercuryRise";
  char MercurySet[56]   = "MercurySet";

  char VenusAzimuth[56]  = "VenusAzimuth";
  char VenusAltitude[56] = "VenusAltitude";
  char VenusUp[56]    = "VenusUp";
  char VenusDown[56]  = "VenusDown";
  char VenusRise[56]  = "VenusRise";
  char VenusSet[56]   = "VenusSet";

  char MarsAzimuth[56]  = "MarsAzimuth";
  char MarsAltitude[56] = "MarsAltitude";
  char MarsUp[56]    = "MarsUp";
  char MarsDown[56]  = "MarsDown";
  char MarsRise[56]  = "MarsRise";
  char MarsSet[56]   = "MarsSet";

  char JupiterAzimuth[56]  = "JupiterAzimuth";
  char JupiterAltitude[56] = "JupiterAltitude";
  char JupiterUp[56]    = "JupiterUp";
  char JupiterDown[56]  = "JupiterDown";
  char JupiterRise[56]  = "JupiterRise";
  char JupiterSet[56]   = "JupiterSet";

  char SaturnAzimuth[56]  = "SaturnAzimuth";
  char SaturnAltitude[56] = "SaturnAltitude";
  char SaturnUp[56]    = "SaturnUp";
  char SaturnDown[56]  = "SaturnDown";
  char SaturnRise[56]  = "SaturnRise";
  char SaturnSet[56]   = "SaturnSet";

  char UranusAzimuth[56]  = "UranusAzimuth";
  char UranusAltitude[56] = "UranusAltitude";
  char UranusUp[56]    = "UranusUp";
  char UranusDown[56]  = "UranusDown";
  char UranusRise[56]  = "UranusRise";
  char UranusSet[56]   = "UranusSet";

  char NeptuneAzimuth[56]  = "NeptuneAzimuth";
  char NeptuneAltitude[56] = "NeptuneAltitude";
  char NeptuneUp[56]    = "NeptuneUp";
  char NeptuneDown[56]  = "NeptuneDown";
  char NeptuneRise[56]  = "NeptuneRise";
  char NeptuneSet[56]   = "NeptuneSet";
  

  // ----------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                VALIDITY DATA

  char GNGGAValidChecksum[56] = "GNGGAValidChecksum";
  char GNRMCValidChecksum[56] = "GNRMCValidChecksum";
  char GPATTValidChecksum[56] = "GPATTValidChecksum";
  char GNGGAInvalidChecksum[56] = "GNGGAInvalidChecksum";
  char GNRMCInvalidChecksum[56] = "GNRMCInvalidChecksum";
  char GPATTInvalidChecksum[56] = "GPATTInvalidChecksum";
  char GNGGAValidCheckData[56] = "GNGGAValidCheckData";
  char GNRMCValidCheckData[56] = "GNRMCValidCheckData";
  char GPATTValidCheckData[56] = "GPATTValidCheckData";
  char GNGGAInvalidCheckData[56] = "GNGGAInvalidCheckData";
  char GNRMCInvalidCheckData[56] = "GNRMCInvalidCheckData";
  char GPATTInvalidCheckData[56] = "GPATTInvalidCheckData";
};
RelayStruct relayData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  DATA: GNGGA

struct GNGGAStruct {
  char sentence[2000];
  char tag[56];                                                                                                           // <0> Log header
  char utc_time[56];                    unsigned long bad_utc_time_i;              bool bad_utc_time = true;              // <1> UTC time, the format is hhmmss.sss
  char latitude[56];                    unsigned long bad_latitude_i;              bool bad_latitude = true;              // <2> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];         unsigned long bad_latitude_hemisphere_i;   bool bad_latitude_hemisphere = true;   // <3> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                   unsigned long bad_longitude_i;             bool bad_longitude = true;             // <4> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];        unsigned long bad_longitude_hemisphere_i;  bool bad_longitude_hemisphere = true;  // <5> Longitude hemisphere, E or W (east longitude or west longitude)
  char positioning_status[56];          unsigned long bad_positioning_status_i;    bool bad_positioning_status = true;    // <6> GNSS positioning status: 0 not positioned, 1 single point positioning, 2: pseudorange difference, 6: pure INS */
  char satellite_count_gngga[56] = "0"; unsigned long bad_satellite_count_gngga_i; bool bad_satellite_count_gngga = true; // <7> Number of satellites used
  char hdop_precision_factor[56];       unsigned long bad_hdop_precision_factor_i; bool bad_hdop_precision_factor = true; // <8> HDOP level precision factor
  char altitude[56];                    unsigned long bad_altitude_i;              bool bad_altitude = true;              // <9> Altitude
  char altitude_units[56];              unsigned long bad_altitude_units_i;        bool bad_altitude_units = true;        // <10> 
  char geoidal[56];                     unsigned long bad_geoidal_i;               bool bad_geoidal = true;               // <11> The height of the earth ellipsoid relative to the geoid 
  char geoidal_units[56];               unsigned long bad_geoidal_units_i;         bool bad_geoidal_units = true;         // <12> 
  char differential_delay[56];          unsigned long bad_differential_delay_i;    bool bad_differential_delay = true;    // <13>
  char id[56];                          unsigned long bad_id_i;                    bool bad_id = true;                    // <14> base station ID
  char check_sum[56];                   unsigned long bad_check_sum_i;             bool bad_check_sum = true;             // <15> XOR check value of all bytes starting from $ to *
  int check_data = 0;                   unsigned long bad_checksum_validity;       bool valid_checksum = false;           // Checksum validity bool, counters and a counter for how many elements passed further testing (gngga check_data should result in 16)
  char temporary_data[56];
  char temporary_data_1[56];
  
};
GNGGAStruct gnggaData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GNGGA

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
    else if (serial1Data.iter_token ==6)  {if (val_positioning_status_gngga(serial1Data.token) == true) {memset(gnggaData.positioning_status, 0, 56);    strcpy(gnggaData.positioning_status, serial1Data.token);    gnggaData.check_data++; gnggaData.bad_positioning_status = false;}    else {gnggaData.bad_positioning_status_i++;    gnggaData.bad_positioning_status = true;}}
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
    Serial.println("[gnggaData.positioning_status] "      + String(gnggaData.positioning_status));
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

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  DATA: GNRMC

struct GNRMCStruct {
  char sentence[2000];
  char tag[56];                                                                                                                            // <0> Log header
  char utc_time[56];                       unsigned long bad_utc_time_i;                     bool bad_utc_time = true;                     // <1> UTC time, the format is hhmmss.sss
  char positioning_status[56];             unsigned long bad_positioning_status_i;           bool bad_positioning_status = true;           // <2> Positioning status, A=effective positioning, V=invalid positioning
  char latitude[56];                       unsigned long bad_latitude_i;                     bool bad_latitude = true;                     // <3> Latitude, the format is  ddmm.mmmmmmm
  char latitude_hemisphere[56];            unsigned long bad_latitude_hemisphere_i;          bool bad_latitude_hemisphere = true;          // <4> Latitude hemisphere, N or S (north latitude or south latitude)
  char longitude[56];                      unsigned long bad_longitude_i;                    bool bad_longitude = true;                    // <5> Longitude, the format is dddmm.mmmmmmm
  char longitude_hemisphere[56];           unsigned long bad_longitude_hemisphere_i;         bool bad_longitude_hemisphere = true;         // <6> Longitude hemisphere, E or W (east longitude or west longitude)
  char ground_speed[56];                   unsigned long bad_ground_speed_i;                 bool bad_ground_speed = true;                 // <7> Ground speed
  char ground_heading[56];                 unsigned long bad_ground_heading_i;               bool bad_ground_heading = true;               // <8> Ground heading (take true north as the reference datum)
  char utc_date[56];                       unsigned long bad_utc_date_i;                     bool bad_utc_date = true;                     // <9> UTC date, the format is ddmmyy (day, month, year)
  char installation_angle[56];             unsigned long bad_installation_angle_i;           bool bad_installation_angle = true;           // <10> Magnetic declination (000.0~180.0 degrees)
  char installation_angle_direction[56];   unsigned long bad_installation_angle_direction_i; bool bad_installation_angle_direction = true; // <11> Magnetic declination direction, E (east) or W (west)
  char mode_indication[56];                unsigned long bad_mode_indication_i;              bool bad_mode_indication = true;              // <12> Mode indication (A=autonomous positioning, D=differential E=estimation, N=invalid data) */
  char check_sum[56];                      unsigned long bad_check_sum_i;                    bool bad_check_sum = true;                    // <13> XOR check value of all bytes starting from $ to *
  int check_data = 0;                      unsigned long bad_checksum_validity;              bool valid_checksum = false;                  // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 14)
  char temporary_data[56];
  char temporary_data_1[56];
};
GNRMCStruct gnrmcData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GNRMC

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

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  DATA: GPATT

struct GPATTStruct {
  char sentence[2000];
  char tag[56];                                                                                      // <0> Log header
  char pitch[56];            unsigned long bad_pitch_i;            bool bad_pitch = true;            // <1> pitch angle
  char angle_channel_0[56];  unsigned long bad_angle_channel_0_i;  bool bad_angle_channel_0 = true;  // <2> P
  char roll[56];             unsigned long bad_roll_i;             bool bad_roll = true;             // <3> Roll angle
  char angle_channel_1[56];  unsigned long bad_angle_channel_1_i;  bool bad_angle_channel_1 = true;  // <4> R
  char yaw[56];              unsigned long bad_yaw_i;              bool bad_yaw = true;              // <5> Yaw angle
  char angle_channel_2[56];  unsigned long bad_angle_channel_2_i;  bool bad_angle_channel_2 = true;  // <6> Y
  char software_version[56]; unsigned long bad_software_version_i; bool bad_software_version = true; // <7> software verion
  char version_channel[56];  unsigned long bad_version_channel_i;  bool bad_version_channel = true;  // <8> S
  char product_id[56];       unsigned long bad_product_id_i;       bool bad_product_id = true;       // <9> Product ID: 96 bit unique ID
  char id_channel[56];       unsigned long bad_id_channel_i;       bool bad_id_channel = true;       // <10> ID 
  char ins[56];              unsigned long bad_ins_i;              bool bad_ins = true;              // <11> INS Default open inertial navigation system
  char ins_channel[56];      unsigned long bad_ins_channel_i;      bool bad_ins_channel = true;      // <12> whether inertial navigation open
  char hardware_version[56]; unsigned long bad_hardware_version_i; bool bad_hardware_version = true; // <13> Named after master chip
  char run_state_flag[56];   unsigned long bad_run_state_flag_i;   bool bad_run_state_flag = true;   // <14> Algorithm status flag: 1->3
  char mis_angle_num[56];    unsigned long bad_mis_angle_num_i;    bool bad_mis_angle_num = true;    // <15> number of Installation
  char custom_logo_0[56];    unsigned long bad_custom_logo_0_i;    bool bad_custom_logo_0 = true;    // <16>
  char custom_logo_1[56];    unsigned long bad_custom_logo_1_i;    bool bad_custom_logo_1 = true;    // <17>
  char custom_logo_2[56];    unsigned long bad_custom_logo_2_i;    bool bad_custom_logo_2 = true;    // <18>
  char static_flag[56];      unsigned long bad_static_flag_i;      bool bad_static_flag = true;      // <19> 1:Static 0：dynamic
  char user_code[56];        unsigned long bad_user_code_i;        bool bad_user_code = true;        // <20> 1：Normal user X：Customuser
  char gst_data[56];         unsigned long bad_gst_data_i;         bool bad_gst_data = true;         // <21> User satellite accuracy
  char line_flag[56];        unsigned long bad_line_flag_i;        bool bad_line_flag = true;        // <22> 1：straight driving，0：curve driving
  char custom_logo_3[56];    unsigned long bad_custom_logo_3_i;    bool bad_custom_logo_3 = true;    // <23>
  char mis_att_flag[56];     unsigned long bad_mis_att_flag_i;     bool bad_mis_att_flag = true;     // <24> 
  char imu_kind[56];         unsigned long bad_imu_kind_i;         bool bad_imu_kind = true;         // <25> Sensor Type: 0->BIms055; 1->BMI160; 2->LSM6DS3TR-C; 3->LSM6DSOW 4->ICM-40607; 5->ICM-40608 6->ICM-42670; 7->LSM6DSR
  char ubi_car_kind[56];     unsigned long bad_ubi_car_kind_i;     bool bad_ubi_car_kind = true;     // <26> 1: small car, 2: big car
  char mileage[56];          unsigned long bad_mileage_i;          bool bad_mileage = true;          // <27> kilometers: max 9999 kilometers
  char custom_logo_4[56];    unsigned long bad_custom_logo_4_i;    bool bad_custom_logo_4 = true;    // <28>
  char custom_logo_5[56];    unsigned long bad_custom_logo_5_i;    bool bad_custom_logo_5 = true;    // <29>
  char run_inetial_flag[56]; unsigned long bad_run_inetial_flag_i; bool bad_run_inetial_flag = true; // <30> 1->4
  char custom_logo_6[56];    unsigned long bad_custom_logo_6_i;    bool bad_custom_logo_6 = true;    // <31>
  char custom_logo_7[56];    unsigned long bad_custom_logo_7_i;    bool bad_custom_logo_7 = true;    // <32>
  char custom_logo_8[56];    unsigned long bad_custom_logo_8_i;    bool bad_custom_logo_8 = true;    // <33>
  char custom_logo_9[56];    unsigned long bad_custom_logo_9_i;    bool bad_custom_logo_9 = true;    // <34>
  char speed_enable[56];     unsigned long bad_speed_enable_i;     bool bad_speed_enable = true;     // <35> 
  char custom_logo_10[56];   unsigned long bad_custom_logo_10_i;   bool bad_custom_logo_10 = true;   // <36>
  char custom_logo_11[56];   unsigned long bad_custom_logo_11_i;   bool bad_custom_logo_11 = true;   // <37>
  char speed_num[56];        unsigned long bad_speed_num_i;        bool bad_speed_num = true;        // <38> 1：fixed setting，0：Self adaptive installation
  char scalable[56];         unsigned long bad_scalable_i;         bool bad_scalable = true;         // <39> 
  char check_sum[56];        unsigned long bad_check_sum_i;        bool bad_check_sum = true;        // <40> XOR check value of all bytes starting from $ to *
  int check_data = 0;        unsigned long bad_checksum_validity;  bool valid_checksum = false;      // Checksum validity bool, counters and a counter for how many elements passed further testing (gnrmc check_data should result in 41)
  char temporary_data[56];
  char temporary_data_1[56];
};
GPATTStruct gpattData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        GPATT

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
    Serial.println("[gpattData.scalable] "         + String(gpattData.scalable)); // intentionally unpopulated
    Serial.println("[gpattData.check_sum] "        + String(gpattData.check_sum));
    Serial.println("[gpattData.check_data] "        + String(gpattData.check_data));
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  DATA: SATIO

struct SatDatatruct {
  char checksum_str[56];
  int checksum_i;
  char satio_sentence[1024];
  char sat_time_stamp_string[56];                                 // datetime timestamp from satellite
  char satDataTag[10]                 = "$SATIO";                 // satio sentence tag
  char last_sat_time_stamp_str[56]    = "00.00";                  // record last time satellites were seen
  bool convert_coordinates            = true;
  char coordinate_conversion_mode[10] = "GNGGA";                   // choose a sentence that degrees/decimal coordinates will be created from
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
  // timezones and daylight saving are subject to geopolitics are therefore subject to change. it may be preferrable to set offset manually, and an automatic option might be added but may be unpreferrable.
  // this system intends to be correct regardless of geopolitical variables, by illiminating those variables. this allows the systems data to be objectively correct long into the future. no maps, no geopolitics.
  int utc_offset = 0;       // can be used to offset hours (+/-) from UTC and can also be used to account for daylight saving. notice this is not called timezone or daylight saving.
  bool utc_offset_flag = 0; // 0: add hours to time, 1: deduct hours from time
  char year_prefix[56] = "20"; // inline with trying to keep everything simple, this value is intended to require one ammendment every 100 years.
  char year_full[56];
  char month[56];
  char day[56];
  char hour[56];
  char minute[56];
  char second[56];
  char millisecond[56];
  char hours_minutes[56];
};
SatDatatruct satData;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       CONVERT COORDINTE DATA
void calculateLocation(){

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                GNGGA COORDINATE CONVERSION

  if (String(satData.coordinate_conversion_mode) == "GNGGA") {
    // convert GNGGA latitude
    satData.temp_latitude_gngga = satData.abs_latitude_gngga_0;
    satData.degreesLat = trunc(satData.temp_latitude_gngga / 100);
    satData.minutesLat = satData.temp_latitude_gngga - (satData.degreesLat * 100);
    satData.secondsLat = (satData.minutesLat - trunc(satData.minutesLat)) * 60;
    satData.millisecondsLat = (satData.secondsLat - trunc(satData.secondsLat)) * 1000;
    satData.minutesLat = trunc(satData.minutesLat);
    satData.secondsLat = trunc(satData.secondsLat);
    satData.location_latitude_gngga =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    if (strcmp(gnggaData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gngga = 0 - satData.location_latitude_gngga;
    }
    scanf("%f17", &satData.location_latitude_gngga);
    sprintf(satData.location_latitude_gngga_str, "%f", satData.location_latitude_gngga);
    // convert GNGGA longitude
    satData.temp_longitude_gngga = satData.abs_longitude_gngga_0;
    satData.degreesLong = trunc(satData.temp_longitude_gngga / 100);
    satData.minutesLong = satData.temp_longitude_gngga - (satData.degreesLong * 100);
    satData.secondsLong = (satData.minutesLong - trunc(satData.minutesLong)) * 60;
    satData.millisecondsLong = (satData.secondsLong - trunc(satData.secondsLong)) * 1000;
    satData.minutesLong = trunc(satData.minutesLong);
    satData.secondsLong = trunc(satData.secondsLong);
    satData.location_longitude_gngga =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
    if (strcmp(gnggaData.longitude_hemisphere, "W") == 0) {
      satData.location_longitude_gngga = 0 - satData.location_longitude_gngga;
    }
    scanf("%f17", &satData.location_longitude_gngga);
    sprintf(satData.location_longitude_gngga_str, "%f", satData.location_longitude_gngga);
  }

  // ------------------------------------------------------------------------------------------------------------------------
  //                                                                                              GNRMC COORDINATE CONVERSION

  else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
    // convert GNRMC latitude
    satData.temp_latitude_gnrmc = satData.abs_latitude_gnrmc_0;
    satData.degreesLat = trunc(satData.temp_latitude_gnrmc / 100);
    satData.minutesLat = satData.temp_latitude_gnrmc - (satData.degreesLat * 100);
    satData.secondsLat = (satData.minutesLat - (satData.minutesLat)) * 60;
    satData.millisecondsLat = (satData.secondsLat - trunc(satData.secondsLat)) * 1000;
    satData.minutesLat = trunc(satData.minutesLat);
    satData.secondsLat = trunc(satData.secondsLat);
    satData.location_latitude_gnrmc =
    satData.degreesLat + satData.minutesLat / 60 + satData.secondsLat / 3600 + satData.millisecondsLat / 3600000;
    if (strcmp(gnrmcData.latitude_hemisphere, "S") == 0) {
      satData.location_latitude_gnrmc = 0 - satData.location_latitude_gnrmc;
    }
    scanf("%f17", &satData.location_latitude_gnrmc);
    sprintf(satData.location_latitude_gnrmc_str, "%f", satData.location_latitude_gnrmc);
    // convert GNRMC longitude
    satData.temp_longitude_gnrmc = satData.abs_longitude_gnrmc_0;
    satData.degreesLong = trunc(satData.temp_longitude_gnrmc / 100);
    satData.minutesLong = satData.temp_longitude_gnrmc - (satData.degreesLong * 100);
    satData.secondsLong = (satData.minutesLong - trunc(satData.minutesLong)) * 60;
    satData.millisecondsLong = (satData.secondsLong - trunc(satData.secondsLong)) * 1000;
    satData.minutesLong = trunc(satData.minutesLong);
    satData.secondsLong = trunc(satData.secondsLong);
    satData.location_longitude_gnrmc =
    satData.degreesLong + satData.minutesLong / 60 + satData.secondsLong / 3600 + satData.millisecondsLong / 3600000;
    if (strcmp(gnrmcData.longitude_hemisphere, "W") == 0) {
      satData.location_longitude_gnrmc = 0 - satData.location_longitude_gnrmc;
    }
    scanf("%f17", &satData.location_longitude_gnrmc);
    sprintf(satData.location_longitude_gnrmc_str, "%f", satData.location_longitude_gnrmc);
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SATIO SENTENCE

void extrapulatedSatData() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                      SATIO SENTENCE: BEGIN

  memset(satData.satio_sentence, 0, 1024);
  strcat(satData.satio_sentence, satData.satDataTag);
  strcat(satData.satio_sentence, ",");

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                    SATIO SENTENCE: TIMESTAMP FROM SAT TIME

  char temp_sat_time_stamp_string[56];
  memset(temp_sat_time_stamp_string, 0, 56);
  strcat(temp_sat_time_stamp_string, gnrmcData.utc_date);
  strcat(temp_sat_time_stamp_string, gnggaData.utc_time);
  strcat(satData.satio_sentence, temp_sat_time_stamp_string);
  strcat(satData.satio_sentence, ",");
  
  memset(satData.year_full, 0, 56);
  strcat(satData.year_full, satData.year_prefix);
  strncat(satData.year_full, &temp_sat_time_stamp_string[4], 1);
  strncat(satData.year_full, &temp_sat_time_stamp_string[5], 1);

  memset(satData.month, 0, 56);
  strncat(satData.month, &temp_sat_time_stamp_string[2], 1);
  strncat(satData.month, &temp_sat_time_stamp_string[3], 1);

  memset(satData.day, 0, 56);
  strncat(satData.day, &temp_sat_time_stamp_string[0], 1);
  strncat(satData.day, &temp_sat_time_stamp_string[1], 1);

  memset(satData.hour, 0, 56);
  strncat(satData.hour, &temp_sat_time_stamp_string[6], 1);
  strncat(satData.hour, &temp_sat_time_stamp_string[7], 1);

  memset(satData.hour, 0, 56);
  strncat(satData.hour, &temp_sat_time_stamp_string[6], 1);
  strncat(satData.hour, &temp_sat_time_stamp_string[7], 1);

  memset(satData.minute, 0, 56);
  strncat(satData.minute, &temp_sat_time_stamp_string[8], 1);
  strncat(satData.minute, &temp_sat_time_stamp_string[9], 1);

  memset(satData.second, 0, 56);
  strncat(satData.second, &temp_sat_time_stamp_string[10], 1);
  strncat(satData.second, &temp_sat_time_stamp_string[11], 1);

  memset(satData.millisecond, 0, 56);
  strncat(satData.millisecond, &temp_sat_time_stamp_string[13], 1);
  strncat(satData.millisecond, &temp_sat_time_stamp_string[14], 1);

  // store hour ready for timezone conversion
  int temp_hour = atoi(satData.hour);
  // add hour(s)
  if (satData.utc_offset_flag == 0) {for (int i = 0; i < satData.utc_offset; i++) {if (temp_hour < 24) {temp_hour++;} else {temp_hour=0;}}}
  // deduct hour(s)
  else if (satData.utc_offset_flag == 1) {for (int i = 0; i < satData.utc_offset; i++) {if (temp_hour > 0) {temp_hour--;} else {temp_hour=23;}}}
  // initialize space for hour string
  char temp_hour_str[56];
  memset(satData.hour, 0, 56);
  // reconstruct hours with leading zero
  if (temp_hour < 10) {strcat(satData.hour, "0"); itoa(temp_hour, temp_hour_str, 10); strcat(satData.hour, temp_hour_str);}
  // reconstruct hours without modification
  else {itoa(temp_hour, satData.hour, 10);}

  // store hours.minutes
  memset(satData.hours_minutes, 0, 56);
  strcat(satData.hours_minutes, satData.hour);
  strcat(satData.hours_minutes, ".");
  strcat(satData.hours_minutes, satData.minute);

  // reconstruct satio datetime timestamp
  memset(satData.sat_time_stamp_string, 0, 56);
  strcat(satData.sat_time_stamp_string, gnrmcData.utc_date);
  strcat(satData.sat_time_stamp_string, satData.hour);
  strcat(satData.sat_time_stamp_string, satData.minute);
  strcat(satData.sat_time_stamp_string, satData.second);
  strcat(satData.sat_time_stamp_string, ".");
  strcat(satData.sat_time_stamp_string, satData.millisecond);

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                       SATIO SENTENCE: LAST KNOWN DOWNLINK
  // sat_time_stamp_string
  if (atoi(gnggaData.satellite_count_gngga) > 0) {
    memset(satData.last_sat_time_stamp_str, 0, 56);
    strcpy(satData.last_sat_time_stamp_str, satData.sat_time_stamp_string);
  }
  strcat(satData.satio_sentence, satData.last_sat_time_stamp_str);
  strcat(satData.satio_sentence, ",");

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                 SATIO SENTENCE: CONVERT DATA FROM STRINGS

  if (satData.convert_coordinates == true) {
    if (String(satData.coordinate_conversion_mode) == "GNGGA") {
      satData.abs_latitude_gngga_0 = atof(String(gnggaData.latitude).c_str());
      satData.abs_longitude_gngga_0 = atof(String(gnggaData.longitude).c_str());
    }
    else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
      satData.abs_latitude_gnrmc_0 = atof(String(gnrmcData.latitude).c_str());
      satData.abs_longitude_gnrmc_0 = atof(String(gnrmcData.longitude).c_str());
    }
    calculateLocation();
    if (String(satData.coordinate_conversion_mode) == "GNGGA") {

      strcat(satData.satio_sentence, satData.location_latitude_gngga_str);
      strcat(satData.satio_sentence, ",");
      strcat(satData.satio_sentence, satData.location_longitude_gngga_str);
      strcat(satData.satio_sentence, ",");
    }
    else if (String(satData.coordinate_conversion_mode) == "GNRMC") {
      strcat(satData.satio_sentence, satData.location_latitude_gnrmc_str);
      strcat(satData.satio_sentence, ",");
      strcat(satData.satio_sentence, satData.location_longitude_gnrmc_str);
      strcat(satData.satio_sentence, ",");
    }
  }
  else {strcat(satData.satio_sentence, "0.0,0.0,");}

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                       SATIO SENTENCE: END
  strcat(satData.satio_sentence, "*");
  satData.checksum_i = getCheckSum(satData.satio_sentence);
  itoa(satData.checksum_i, satData.checksum_str, 10);
  strcat(satData.satio_sentence, satData.checksum_str);
  if (systemData.output_satio_enabled == true) {Serial.println(satData.satio_sentence);}
  }


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                           SDCARD: INITIALIZE

void init_sdcard() {
  if (!SD.begin(SS, sdspi, 80000000)) {
    Serial.println("Card Mount Failed");
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
  }
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
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                        SDCARD: PRINT FILE CONTENTS TO SERIAL

bool sdcard_read_to_serial(fs::FS &fs, char * file) {
  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file);
  if (sdcardData.current_file) {
    while (sdcardData.current_file.available()) {Serial.write(sdcardData.current_file.read());}
    sdcardData.current_file.close(); return true;
  }
  else {sdcardData.current_file.close(); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                            SDCARD: SAVE SYSTEM CONFIGURATION

void sdcard_save_system_configuration(fs::FS &fs, char * file, int return_page) {
  Serial.println("[sdcard] attempting to save file: " + String(file));
  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file, FILE_WRITE);
  if (sdcardData.current_file) {

    // note that this should always be default file.
    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "MATRIX_FILEPATH,");
    if (!sdcardData.matrix_filepath) {strcat(sdcardData.file_data, sdcardData.default_matrix_filepath);}
    else {strcat(sdcardData.file_data, sdcardData.matrix_filepath);}
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "AUTO_RESUME,");
    itoa(systemData.autoresume_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "MATRIX_ENABLED,");
    itoa(systemData.matrix_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "SATIO_ENABLED,");
    itoa(systemData.satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "GNGGA_ENABLED,");
    itoa(systemData.gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "GNRMC_ENABLED,");
    itoa(systemData.gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "GPATT_ENABLED,");
    itoa(systemData.gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_AUTO_OFF,");
    itoa(systemData.display_auto_off, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_AUTO_DIM,");
    itoa(systemData.display_auto_dim, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_LOW_LIGHT,");
    itoa(systemData.display_low_light, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "DISPLAY_FLIP_VERTICALLY,");
    itoa(systemData.display_flip_vertically, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_SATIO_SENTENCE,");
    itoa(systemData.output_satio_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_GNGGA_SENTENCE,");
    itoa(systemData.output_gngga_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_GNRMC_SENTENCE,");
    itoa(systemData.output_gnrmc_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "OUTPUT_GPATT_SENTENCE,");
    itoa(systemData.output_gpatt_enabled, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "UTC_OFFSET,");
    itoa(satData.utc_offset, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "UTC_OFFSET_FLAG,");
    itoa(satData.utc_offset_flag, sdcardData.tmp, 10);
    strcat(sdcardData.file_data, sdcardData.tmp);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    memset(sdcardData.file_data, 0, 256);
    strcat(sdcardData.file_data, "YEAR_PREFIX,");
    strcat(sdcardData.file_data, satData.year_prefix);
    strcat(sdcardData.file_data, ",");
    Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
    sdcardData.current_file.println("");
    sdcardData.current_file.println(sdcardData.file_data);
    sdcardData.current_file.println("");

    sdcardData.current_file.close();
    Serial.println("[sdcard] saved file successfully: " + String(file));
  }
  else {sdcardData.current_file.close(); Serial.println("[sdcard] failed to save file: " + String(file));}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                            SDCARD: LOAD SYSTEM CONFIGURATION 

bool sdcard_load_system_configuration(fs::FS &fs, char * file, int return_page) {
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
      Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));

      // check matrix filepath
      if (strncmp(sdcardData.BUFFER, "MATRIX_FILEPATH", 15) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
        sdcardData.token = strtok(NULL, ",");
        Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
        memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
        strcpy(sdcardData.matrix_filepath, sdcardData.token);
      }
      // check auto resume
      if (strncmp(sdcardData.BUFFER, "AUTO_RESUME", 11) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
        sdcardData.token = strtok(NULL, ",");
        if (is_all_digits(sdcardData.token) == true) {
          Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
          if (atoi(sdcardData.token) == 0) {systemData.autoresume_enabled = false;} else {systemData.autoresume_enabled = true;}
        }
      }
      // continue to enable/disable only if auto resume is true
      if (systemData.autoresume_enabled == true) {
      
        if (strncmp(sdcardData.BUFFER, "MATRIX_ENABLED", strlen("MATRIX_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.matrix_enabled = false;} else {systemData.matrix_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "SATIO_ENABLED", strlen("SATIO_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.satio_enabled = false;} else {systemData.satio_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "GNGGA_ENABLED", strlen("GNGGA_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.gngga_enabled = false;} else {systemData.gngga_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "GNRMC_ENABLED", strlen("GNRMC_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.gnrmc_enabled = false;} else {systemData.gnrmc_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "GPATT_ENABLED", strlen("GPATT_ENABLED")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.gpatt_enabled = false;} else {systemData.gpatt_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_OFF", strlen("DISPLAY_AUTO_OFF")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.display_auto_off = false;} else {systemData.display_auto_off = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_AUTO_DIM", strlen("DISPLAY_AUTO_DIM")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.display_auto_dim = false;} else {systemData.display_auto_dim = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_LOW_LIGHT", strlen("DISPLAY_LOW_LIGHT")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.display_low_light = false;} else {systemData.display_low_light = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "DISPLAY_FLIP_VERTICALLY", strlen("DISPLAY_FLIP_VERTICALLY")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.display_flip_vertically = false;} else {systemData.display_flip_vertically = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_SATIO_SENTENCE", strlen("OUTPUT_SATIO_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.output_satio_enabled = false;} else {systemData.output_satio_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GNGGA_SENTENCE", strlen("OUTPUT_GNGGA_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.output_gngga_enabled = false;} else {systemData.output_gngga_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GNRMC_SENTENCE", strlen("OUTPUT_GNRMC_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.output_gnrmc_enabled = false;} else {systemData.output_gnrmc_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "OUTPUT_GPATT_SENTENCE", strlen("OUTPUT_GPATT_SENTENCE")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {systemData.output_gpatt_enabled = false;} else {systemData.output_gpatt_enabled = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "UTC_OFFSET,", strlen("UTC_OFFSET,")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            satData.utc_offset = atoi(sdcardData.token);
          }
        }
        else if (strncmp(sdcardData.BUFFER, "UTC_OFFSET_FLAG", strlen("UTC_OFFSET_FLAG")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            if (atoi(sdcardData.token) == 0) {satData.utc_offset_flag = false;} else {satData.utc_offset_flag = true;}
          }
        }
        else if (strncmp(sdcardData.BUFFER, "YEAR_PREFIX", strlen("YEAR_PREFIX")) == 0) {
          sdcardData.token = strtok(sdcardData.BUFFER, ",");
          Serial.println("[sdcard] system configuration: " + String(sdcardData.token));
          sdcardData.token = strtok(NULL, ",");
          if (is_all_digits(sdcardData.token) == true) {
            Serial.println("[sdcard] system configuration setting: " + String(sdcardData.token));
            memset(satData.year_prefix, 0, 256);
            strcat(satData.year_prefix, sdcardData.token);
          }
        }
      }
    }
    sdcardData.current_file.close();
    Serial.println("[sdcard] loaded file successfully: " + String(file));
    return true;
  }
  else {sdcardData.current_file.close(); Serial.println("[sdcard] failed to load file: " + String(file)); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       SDCARD: MAKE DIRECTORY

void sdcard_mkdir(fs::FS &fs, char * dir){
  if (!fs.exists(dir)) {
    Serial.println("[sdcard] attempting to create directory: " + String(dir));
    if (!fs.mkdir(dir)) {Serial.println("[sdcard] failed to create directory: " + String(dir));}
    else {Serial.println("[sdcard] found directory: " + String(dir));}}
  else {Serial.println("[sdcard] directory already exists: " + String(dir));}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                     SDCARD: MAKE DIRECTORIES

void sdcard_mkdirs() {for (int i = 0; i < 2; i++) {sdcard_mkdir(SD, sdcardData.system_dirs[i]);}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                         SDCARD: CALCULATE AVAILABLE FILENAME

void sdcard_calculate_filename_create(fs::FS &fs, char * dir, char * name, char * ext) {
  char tempname[1024];
  char temppath[1024];
  char temp_i[16];
  for (int i = 0; i < 100; i++) {
    memset(temppath, 0, 1024); strcpy(temppath, dir); strcat(temppath, name); strcat(temppath, "_"); itoa(i, temp_i, 10); strcat(temppath, temp_i); strcat(temppath, ext);
    memset(tempname, 0, 1024); strcat(tempname, name); strcat(tempname, "_"); strcat(tempname, temp_i); strcat(tempname, ext);
    Serial.println("[sdcard] calculating: " + String(temppath));
    if (!fs.exists(temppath)) {
      Serial.println("[sdcard] calculated new filename: " + String(temppath));
      memset(sdcardData.matrix_filepath, 0, 56); strcpy(sdcardData.matrix_filepath, temppath);
      break;}
    else {Serial.println("[sdcard] skipping filename: " + String(temppath));}
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                    SDCARD: PUT ALL MATRIX FILENAMES IN ARRAY

void sdcard_list_matrix_files(fs::FS &fs, char * dir, char * name, char * ext) {
  char tempname[56];
  char temppath[56];
  char temp_i[4];
  for (int i = 0; i < sdcardData.max_matrix_filenames; i++) {memset(sdcardData.matrix_filenames[i], 0, 56);}
  for (int i = 0; i < sdcardData.max_matrix_filenames; i++) {
    memset(temppath, 0, 56); strcpy(temppath, dir); strcat(temppath, name); strcat(temppath, "_"); itoa(i, temp_i, 10); strcat(temppath, temp_i); strcat(temppath, ext);
    memset(tempname, 0, 56); strcat(tempname, name); strcat(tempname, "_"); strcat(tempname, temp_i); strcat(tempname, ext);
    // Serial.println("[sdcard] calculating: " + String(temppath)); // debug
    if (fs.exists(temppath)) {
      Serial.println("[sdcard] calculated filename found: " + String(temppath));
      memset(sdcardData.matrix_filenames[i], 0, 56); strcpy(sdcardData.matrix_filenames[i], temppath);
      Serial.println("[matrix_filenames] " + String(sdcardData.matrix_filenames[i]));
      }
    // else {Serial.println("[sdcard] skipping filename: " + String(temppath)); Serial.println("[sdcard] matrix_filename_i: " + String(sdcardData.matrix_filename_i));} // debug
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  ZERO MATRIX

void zero_matrix() {
  Serial.println("[matrix] setting all matrix values to zero.");
  // iterate over each relay matrix
  for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) {
    relayData.relays_enable[0][Ri] = 0;
    for (int Fi = 0; Fi < relayData.MAobject_RELAY_ELEMENTS; Fi++) {
      memset(relayData.relays[Ri][Fi], 0, 56);
      strcpy(relayData.relays[Ri][Fi], "$NONE");
      relayData.relays_data[Ri][Fi][0] = 0.0;
      relayData.relays_data[Ri][Fi][1] = 0.0;
      relayData.relays_data[Ri][Fi][2] = 0.0;
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          SDCARD: LOAD MATRIX 

bool sdcard_load_matrix(fs::FS &fs, char * file) {
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
      Serial.println("[sdcard] [reading] " + String(sdcardData.BUFFER));
      if (strncmp(sdcardData.BUFFER, "r", 1) == 0) {
        // ensure cleared
        memset(sdcardData.data_0, 0, 56); memset(sdcardData.data_1, 0, 56); memset(sdcardData.data_2, 0, 56);
        memset(sdcardData.data_3, 0, 56); memset(sdcardData.data_4, 0, 56); memset(sdcardData.data_5, 0, 56);
        memset(sdcardData.data_6, 0, 56);
        validData.bool_data_0 = false;
        validData.bool_data_1 = false;
        // split line on delimiter
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        // relay index
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_0, sdcardData.token);
        if (is_all_digits(sdcardData.data_0) == true) {validData.bool_data_0 = true;
        Serial.println("[Ri] [PASS] " +String(sdcardData.data_0));
        }
        else {Serial.println("[Ri] [INVALID] " +String(sdcardData.data_0));}
        // relay function index
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_1, sdcardData.token);
        if (is_all_digits(sdcardData.data_1) == true) {validData.bool_data_1 = true;
          Serial.println("[Fi] [PASS] " +String(sdcardData.data_1));
        }
        else {Serial.println("[Fi] [INVALID] " +String(sdcardData.data_1));}
        // continue if we have valid index numbers
        if ((validData.bool_data_0 == true) && (validData.bool_data_1 == true)) {
          // relay function name
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_2, sdcardData.token);
          memset(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], 0, 56);
          strcpy(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)], sdcardData.data_2);
          Serial.println("[Fn] [MATRIX] " +String(relayData.relays[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)]));
          // relay function data: x
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_3, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_3) == true) {
            relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0] = atol(sdcardData.data_3);
            Serial.println("[X]  [MATRIX] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][0]));
          }
          else {Serial.println("[X] [INVALID] " + String(sdcardData.data_3));}
          // relay function data: y
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_4, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_4) == true) {
            relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1] = atol(sdcardData.data_4);
            Serial.println("[Y]  [MATRIX] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][1]));
          }
          else {Serial.println("[Y] [INVALID] " + String(sdcardData.data_4));}
          // relay function data: z
          sdcardData.token = strtok(NULL, ",");
          strcpy(sdcardData.data_5, sdcardData.token);
          if (is_positive_negative_num(sdcardData.data_5) == true) {
            relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2] = atol(sdcardData.data_5);
            Serial.println("[Z]  [MATRIX] " +String(relayData.relays_data[atoi(sdcardData.data_0)][atoi(sdcardData.data_1)][2]));
          }
          else {Serial.println("[Z] [INVALID] " + String(sdcardData.data_5));}
        }
      }
      else if (strncmp(sdcardData.BUFFER, "e", 1) == 0) {
        sdcardData.token = strtok(sdcardData.BUFFER, ",");
        sdcardData.token = strtok(NULL, ",");
        sdcardData.token = strtok(NULL, ",");
        strcpy(sdcardData.data_6, sdcardData.token);
        if (is_all_digits(sdcardData.data_6) == true) {
          relayData.relays_enable[0][atoi(sdcardData.data_0)] = atoi(sdcardData.data_6);
          Serial.println("[E]  [MATRIX] " +String(relayData.relays_enable[0][atoi(sdcardData.data_0)]));
          }
        else {Serial.println("[E]  [INVALID] " +String(sdcardData.data_6));}
      }
    }
    strcpy(sdcardData.tempmatrixfilepath, file);
    memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));
    strcpy(sdcardData.matrix_filepath, sdcardData.tempmatrixfilepath);
    Serial.println("[sdcard] loaded file successfully:   " + String(file));
    Serial.println("[sdcard] sdcardData.matrix_filepath: " + String(sdcardData.matrix_filepath));
    sdcardData.current_file.close();
    return true;
  }
  else {sdcardData.current_file.close(); Serial.println("[sdcard] failed to load file: " + String(file)); return false; memset(sdcardData.matrix_filepath, 0, sizeof(sdcardData.matrix_filepath));}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          SDCARD: SAVE MATRIX

bool sdcard_save_matrix(fs::FS &fs, char * file) {
  Serial.println("[sdcard] attempting to save file: " + String(file));
  sdcardData.current_file.flush();
  sdcardData.current_file = fs.open(file, FILE_WRITE);
  if (sdcardData.current_file) {
    for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) {
      for (int Fi = 0; Fi < relayData.MAobject_RELAY_ELEMENTS; Fi++) {
        memset(sdcardData.file_data, 0 , 256);
        // tag: relay (r)
        strcat(sdcardData.file_data, sdcardData.tag_0); strcat(sdcardData.file_data, sdcardData.delim);
        // relay index
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%d", Ri);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // relay function index
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%d", Fi);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function name
        strcat(sdcardData.file_data, relayData.relays[Ri][Fi]); strcat(sdcardData.file_data, sdcardData.delim);
        // function value x
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", relayData.relays_data[Ri][Fi][0]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value y
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", relayData.relays_data[Ri][Fi][1]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // function value z
        memset(sdcardData.tmp, 0 , 256);
        sprintf(sdcardData.tmp, "%f", relayData.relays_data[Ri][Fi][2]);
        strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
        // write line
        Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
        sdcardData.current_file.println(sdcardData.file_data);
      }
      memset(sdcardData.file_data, 0 , 256);
      // tag: enable (e)
      strcat(sdcardData.file_data, sdcardData.tag_1); strcat(sdcardData.file_data, sdcardData.delim);
      // relay index
      memset(sdcardData.tmp, 0 , 256);
      sprintf(sdcardData.tmp, "%d", Ri);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // relay enabled 0/1
      memset(sdcardData.tmp, 0 , 256);
      itoa(relayData.relays_enable[0][Ri], sdcardData.tmp, 10);
      strcat(sdcardData.file_data, sdcardData.tmp); strcat(sdcardData.file_data, sdcardData.delim);
      // write line
      Serial.println("[sdcard] [writing] " + String(sdcardData.file_data));
      sdcardData.current_file.println("");
      sdcardData.current_file.println(sdcardData.file_data);
      sdcardData.current_file.println("");
    }
    sdcardData.current_file.close();
    Serial.println("[sdcard] saved file successfully: " + String(file));
    return true;
  }
  else {sdcardData.current_file.close(); Serial.println("[sdcard] failed to save file: " + String(file)); return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                   SDCARD: DELETE MATRIX FILE

void sdcard_delete_matrix(fs::FS &fs, char * file) {
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
      delay(5000);
    }
    else {Serial.println("[sdcard] failed to deleted file: " + String(file));}
  }
  else {Serial.println("[sdcard] file does not exist: " + String(file));}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            MATRIX: SET ENTRY

/*
example test command: $MATRIX_SET_ENTRY,0,0,SatelliteCountOver,1,0,0
*/

void matriobject_set_entry() {
  Serial.println("[matriobject_set_entry] connected");
  serial0Data.check_data_R = 0;
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
  strcpy(relayData.relays[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)], serial0Data.data_2);      // set function
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]=strtod(serial0Data.data_3, &ptr); // set function value x
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]=strtod(serial0Data.data_4, &ptr); // set function value y
  relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]=strtod(serial0Data.data_5, &ptr); // set function value z
  Serial.println("[Ri] " +String(serial0Data.data_0));
  Serial.println("[Fi] " +String(serial0Data.data_1));
  Serial.println("[Fn] " +String(relayData.relays[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)]));
  Serial.println("[X] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][0]));
  Serial.println("[Y] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][1]));
  Serial.println("[Z] " +String(relayData.relays_data[atoi(serial0Data.data_0)][atoi(serial0Data.data_1)][2]));
}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                 MATRIX: ENABLE/DISABLE ENTRY

void matriobject_set_enabled(bool b) {
  Serial.println("[matriobject_set_enabled] connected");
  serial0Data.check_data_R = 0;
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
  relayData.relays_enable[0][atoi(serial0Data.data_0)] = b; // set enable/disable
  Serial.println("[R] " + String(serial0Data.data_0) + " [E] " + String(relayData.relays_enable[0][atoi(serial0Data.data_0)]));
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          MATRIX: DISABLE ALL

/*
disable all matrix entries. does not directly turn relays off. this allows for overriding the matrix switch without deactivating
anything that may be / should remain activated. automatically deactivating a relay when a relay is made disabled should be
explicitly configured and is not yet a feature. this is explicitly disable matrix switch automatically activating/deactivating relays.
*/
void matrix_disable_all() {for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) {relayData.relays_enable[0][Ri]=0;}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                           MATRIX: ENABLE ALL


// enable all matrix entries. does not directly turn relays on, instead enables matrix switch automatically activating/deactivating relays.
void matrix_enable_all() {for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) {relayData.relays_enable[0][Ri]=1;}}


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       MATRIX: ALL RELAYS OFF

// turn all relays off. recommended to first disable matrix.
void relays_deactivate_all() {for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) {relayData.relays_bool[0][Ri]=0;}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                        MATRIX: ALL RELAYS ON

// turn all relays on. recommended to first disable matrix.
void relays_activate_all() {for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) {relayData.relays_bool[0][Ri]=1;}}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                   SATIO: CONVERT COORDINATES

// enable/disable coordinate conversion. performance/efficiency as required.
void satio_convert_coordinates_on()  {satData.convert_coordinates = true;}
void satio_convert_coordinates_off() {satData.convert_coordinates = false;}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                 MATRIX FUNCTIONS: PRIMITIVES

/*
matrix switch requires all checks to return true for a relay to be active, therefore checks can be inverted as required, to return
true when otherwise a check would return false.
*/

// calculate if n0 in (+- range/2) of n1
bool in_range_check_true(double n0, double n1, double r) {
  // Serial.println("in_range_check_true: (n0 " + String(n0) + " >= n1 (" + String(n1) " - r/2 " + String(r/2) + ")) && (n0 " + String(n0) + " <= n1 (" + String(n1) " + r/2 " + String(r/2) + "))");
  if (n0  >=  n1 - r/2) {if (n0  <= n1 + r/2) {return true;}}
  else {return false;}
}

// calculate if n0 in (+- range/2) of n1
bool in_range_check_false(double n0, double n1, double r) {
  // Serial.println("in_range_check_false: (n0 " + String(n0) + " >= n1 (" + String(n1) " - r/2 " + String(r/2) + ")) && (n0 " + String(n0) + " <= n1 (" + String(n1) " + r/2 " + String(r/2) + "))");
  if (n0  >=  n1 - r/2) {if (n0  <= n1 + r/2) {return false;}}
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
  // Serial.println("check_over_true: n0 " + String(n0) + " > n1 " + String(n1));
  if (n0 > n1) {return true;}
  else {return false;}
}

bool check_over_false(double n0, double n1) {
  // Serial.println("check_over_false: n0 " + String(n0) + " > n1 " + String(n1));
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
  // Serial.println("check_ge_and_le_true: n0 " + String(n0) + " >= n1 " + String(n1) + " && n0 " + String(n0) + " <= " + String(n2));
  if ((n0 >= n1) && (n0 <= n2)) {return true;}
  else {return false;}
}

bool check_ge_and_le_false(double n0, double n1, double n2) {
  // Serial.println("check_ge_and_le_false: n0 " + String(n0) + " >= n1 " + String(n1) + " && n0 " + String(n0) + " <= " + String(n2));
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

bool SecondsTimer(unsigned long n0, unsigned long n1, int Ri) {
  // (requires maintenace now that the main loop time is measured in micros and now that this system is multitasking)
  // max seconds 18446744073709551616 (584942417355.07202148 years)
  // n0: interval
  // n1: on time
  // backend interface example for on 1sec/off 1sec: $MATRIobject_sET_ENTRY,0,0,SecondsTimer,1,1,0
  if ((timeData.seconds - relayData.relays_timing[0][Ri]) > n0) {relayData.relays_timing[0][Ri] = timeData.seconds; return true;}
  else if ((timeData.seconds - relayData.relays_timing[0][Ri]) < n1) {return true;}
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                   MATRIX FUNCTIONS: ADVANCED

// build astronomical, ephemeris and other advanced caculations here

// struct SiderealPlantetsStruct {
//   long sun_ra;
//   long sun_dec;
//   long sun_az;
//   long sun_alt;
//   long sun_r;
//   long sun_s;
//   long moon_ra;
//   long moon_dec;
//   long moon_az;
//   long moon_alt;
//   long moon_r;
//   long moon_s;
//   long moon_p;
//   long mercury_ra;
//   long mercury_dec;
//   long mercury_az;
//   long mercury_alt;
//   long mercury_r;
//   long mercury_s;
//   long mercury_helio_ecliptic_lat;
//   long mercury_helio_ecliptic_long;
//   long mercury_radius_vector;
//   long mercury_distance;
//   long mercury_ecliptic_lat;
//   long mercury_ecliptic_long;
//   long venus_ra;
//   long venus_dec;
//   long venus_az;
//   long venus_alt;
//   long venus_r;
//   long venus_s;
//   long venus_helio_ecliptic_lat;
//   long venus_helio_ecliptic_long;
//   long venus_radius_vector;
//   long venus_distance;
//   long venus_ecliptic_lat;
//   long venus_ecliptic_long;
//   long mars_ra;
//   long mars_dec;
//   long mars_az;
//   long mars_alt;
//   long mars_r;
//   long mars_s;
//   long mars_helio_ecliptic_lat;
//   long mars_helio_ecliptic_long;
//   long mars_radius_vector;
//   long mars_distance;
//   long mars_ecliptic_lat;
//   long mars_ecliptic_long;
//   long jupiter_ra;
//   long jupiter_dec;
//   long jupiter_az;
//   long jupiter_alt;
//   long jupiter_r;
//   long jupiter_s;
//   long jupiter_helio_ecliptic_lat;
//   long jupiter_helio_ecliptic_long;
//   long jupiter_radius_vector;
//   long jupiter_distance;
//   long jupiter_ecliptic_lat;
//   long jupiter_ecliptic_long;
//   long saturn_ra;
//   long saturn_dec;
//   long saturn_az;
//   long saturn_alt;
//   long saturn_r;
//   long saturn_s;
//   long saturn_helio_ecliptic_lat;
//   long saturn_helio_ecliptic_long;
//   long saturn_radius_vector;
//   long saturn_distance;
//   long saturn_ecliptic_lat;
//   long saturn_ecliptic_long;
//   long uranus_ra;
//   long uranus_dec;
//   long uranus_az;
//   long uranus_alt;
//   long uranus_r;
//   long uranus_s;
//   long uranus_helio_ecliptic_lat;
//   long uranus_helio_ecliptic_long;
//   long uranus_radius_vector;
//   long uranus_distance;
//   long uranus_ecliptic_lat;
//   long uranus_ecliptic_long;
//   long neptune_ra;
//   long neptune_dec;
//   long neptune_az;
//   long neptune_alt;
//   long neptune_r;
//   long neptune_s;
//   long neptune_helio_ecliptic_lat;
//   long neptune_helio_ecliptic_long;
//   long neptune_radius_vector;
//   long neptune_distance;
//   long neptune_ecliptic_lat;
//   long neptune_ecliptic_long;
// };
// SiderealPlantetsStruct siderealPlanetData;

// struct SiderealObjectStruct {
//   char object_name[56];
//   char object_table_name[56];
//   int  object_number;
//   int  object_table_i;
//   long object_ra;
//   long object_dec;
//   long object_az;
//   long object_alt;
//   long object_mag;
//   long object_r;
//   long object_s;
//   double objects_data[609][7];
//   char object_table[7][20] =
//   {
//     "Star Table",          // 0
//     "NGC Table",           // 1
//     "IC Table",            // 2
//     "Other Objects Table", // 3
//     "Messier Table",       // 4
//     "Caldwell Table",      // 5
//     "Herschel 400 Table",  // 6
//   };
// };
// SiderealObjectStruct siderealObjectData;

// void trackObject(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second,
//                  int object_table_i, int object_i) {
//   myAstro.setLatLong(latitude, longitude);
//   // myAstro.setTimeZone(tz);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   if (object_table_i == 0) {myAstroObj.selectStarTable(object_i);}
//   if (object_table_i == 1) {myAstroObj.selectNGCTable(object_i);}
//   if (object_table_i == 2) {myAstroObj.selectICTable(object_i);}
//   if (object_table_i == 3) {myAstroObj.selectMessierTable(object_i);}
//   if (object_table_i == 4) {myAstroObj.selectCaldwellTable(object_i);}
//   if (object_table_i == 5) {myAstroObj.selectHershel400Table(object_i);}
//   if (object_table_i == 6) {myAstroObj.selectOtherObjectsTable(object_i);}
//   myAstro.setRAdec(myAstroObj.getRAdec(), myAstroObj.getDeclinationDec());
//   myAstro.doRAdec2AltAz();
//   siderealObjectData.object_az = myAstro.getAzimuth();
//   siderealObjectData.object_alt = myAstro.getAltitude();
//   siderealObjectData.object_r = myAstro.getRiseTime();
//   siderealObjectData.object_s = myAstro.getSetTime();
// }

// void IdentifyObject(double object_ra, double object_dec) {
//   myAstroObj.setRAdec(object_ra, object_dec);
//   myAstro.doRAdec2AltAz();
//   siderealObjectData.object_mag = myAstroObj.getStarMagnitude();
//   myAstroObj.identifyObject();
//   switch(myAstroObj.getIdentifiedObjectTable()) {
//   case(1):
//   siderealObjectData.object_table_i = 0; break;
// 	case(2):
//   siderealObjectData.object_table_i = 1; break;
// 	case(3):
//   siderealObjectData.object_table_i = 2;  break;
// 	case(7):
//   siderealObjectData.object_table_i = 3;  break;
//   }
//   if (myAstroObj.getIdentifiedObjectTable() == 1) {
//     // set table name
//     memset(siderealObjectData.object_table_name, 0, 56);
//     strcpy(siderealObjectData.object_table_name, siderealObjectData.object_table[siderealObjectData.object_table_i]);
//     // set object id name
//     memset(siderealObjectData.object_name, 0, 56);
//     strcpy(siderealObjectData.object_name, myAstroObj.printStarName(myAstroObj.getIdentifiedObjectNumber()));
//     }
//   if (myAstroObj.getAltIdentifiedObjectTable()) {
//     switch(myAstroObj.getAltIdentifiedObjectTable()) {
// 	  casematrix_indi_h:
//     siderealObjectData.object_table_i = 4;  break;
// 	  case(5):
//     siderealObjectData.object_table_i = 5;  break;
// 	  case(6):
//     siderealObjectData.object_table_i = 6;  break;
//     }
//   // set table name
//   memset(siderealObjectData.object_table_name, 0, 56);
//   strcpy(siderealObjectData.object_table_name, siderealObjectData.object_table[siderealObjectData.object_table_i]);
//   // set object id number
//   siderealObjectData.object_number = myAstroObj.getAltIdentifiedObjectNumber();
//   }
// }

// void trackSun(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doSun();
//   siderealPlanetData.sun_ra  = myAstro.getRAdec();
//   siderealPlanetData.sun_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.sun_az  = myAstro.getAzimuth();
//   siderealPlanetData.sun_alt = myAstro.getAltitude();
//   myAstro.doSunRiseSetTimes();
//   siderealPlanetData.sun_r  = myAstro.getSunriseTime();
//   siderealPlanetData.sun_s  = myAstro.getSunsetTime();
// }

// void trackMoon(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doSun();
//   siderealPlanetData.moon_ra  = myAstro.getRAdec();
//   siderealPlanetData.moon_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.moon_az  = myAstro.getAzimuth();
//   siderealPlanetData.moon_alt = myAstro.getAltitude();
//   myAstro.doMoonRiseSetTimes();
//   siderealPlanetData.moon_r  = myAstro.doMoonRiseSetTimes();
//   siderealPlanetData.moon_s  = myAstro.getSunsetTime();
//   siderealPlanetData.moon_p  = myAstro.getMoonPhase();
// }

// void trackMercury(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doMercury();
//   siderealPlanetData.mercury_ra  = myAstro.getRAdec();
//   siderealPlanetData.mercury_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.mercury_az  = myAstro.getAzimuth();
//   siderealPlanetData.mercury_alt = myAstro.getAltitude();
//   siderealPlanetData.mercury_helio_ecliptic_lat = myAstro.getHelioLat();
//   siderealPlanetData.mercury_helio_ecliptic_long = myAstro.getHelioLong();
//   siderealPlanetData.mercury_radius_vector = myAstro.getRadiusVec();
//   siderealPlanetData.mercury_distance = myAstro.getDistance();
//   siderealPlanetData.mercury_ecliptic_lat = myAstro.getEclipticLatitude();
//   siderealPlanetData.mercury_ecliptic_long = myAstro.getEclipticLongitude();
//   myAstro.doXRiseSetTimes();
//   siderealPlanetData.mercury_r = myAstro.getRiseTime();
//   siderealPlanetData.mercury_s = myAstro.getSetTime();
// }

// void trackVenus(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doVenus();
//   siderealPlanetData.venus_ra  = myAstro.getRAdec();
//   siderealPlanetData.venus_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.venus_az  = myAstro.getAzimuth();
//   siderealPlanetData.venus_alt = myAstro.getAltitude();
//   siderealPlanetData.venus_helio_ecliptic_lat = myAstro.getHelioLat();
//   siderealPlanetData.venus_helio_ecliptic_long = myAstro.getHelioLong();
//   siderealPlanetData.venus_radius_vector = myAstro.getRadiusVec();
//   siderealPlanetData.venus_distance = myAstro.getDistance();
//   siderealPlanetData.venus_ecliptic_lat = myAstro.getEclipticLatitude();
//   siderealPlanetData.venus_ecliptic_long = myAstro.getEclipticLongitude();
//   myAstro.doXRiseSetTimes();
//   siderealPlanetData.venus_r = myAstro.getRiseTime();
//   siderealPlanetData.venus_s = myAstro.getSetTime();
// }

// void trackMars(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doMars();
//   siderealPlanetData.mars_ra  = myAstro.getRAdec();
//   siderealPlanetData.mars_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.mars_az  = myAstro.getAzimuth();
//   siderealPlanetData.mars_alt = myAstro.getAltitude();
//   siderealPlanetData.mars_helio_ecliptic_lat = myAstro.getHelioLat();
//   siderealPlanetData.mars_helio_ecliptic_long = myAstro.getHelioLong();
//   siderealPlanetData.mars_radius_vector = myAstro.getRadiusVec();
//   siderealPlanetData.mars_distance = myAstro.getDistance();
//   siderealPlanetData.mars_ecliptic_lat = myAstro.getEclipticLatitude();
//   siderealPlanetData.mars_ecliptic_long = myAstro.getEclipticLongitude();
//   myAstro.doXRiseSetTimes();
//   siderealPlanetData.mars_r = myAstro.getRiseTime();
//   siderealPlanetData.mars_s = myAstro.getSetTime();
// }

// void trackJupiter(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doJupiter();
//   siderealPlanetData.jupiter_ra  = myAstro.getRAdec();
//   siderealPlanetData.jupiter_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.jupiter_az  = myAstro.getAzimuth();
//   siderealPlanetData.jupiter_alt = myAstro.getAltitude();
//   siderealPlanetData.jupiter_helio_ecliptic_lat = myAstro.getHelioLat();
//   siderealPlanetData.jupiter_helio_ecliptic_long = myAstro.getHelioLong();
//   siderealPlanetData.jupiter_radius_vector = myAstro.getRadiusVec();
//   siderealPlanetData.jupiter_distance = myAstro.getDistance();
//   siderealPlanetData.jupiter_ecliptic_lat = myAstro.getEclipticLatitude();
//   siderealPlanetData.jupiter_ecliptic_long = myAstro.getEclipticLongitude();
//   myAstro.doXRiseSetTimes();
//   siderealPlanetData.jupiter_r = myAstro.getRiseTime();
//   siderealPlanetData.jupiter_s = myAstro.getSetTime();
// }

// void trackSaturn(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doSaturn();
//   siderealPlanetData.saturn_ra  = myAstro.getRAdec();
//   siderealPlanetData.saturn_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.saturn_az  = myAstro.getAzimuth();
//   siderealPlanetData.saturn_alt = myAstro.getAltitude();
//   siderealPlanetData.saturn_helio_ecliptic_lat = myAstro.getHelioLat();
//   siderealPlanetData.saturn_helio_ecliptic_long = myAstro.getHelioLong();
//   siderealPlanetData.saturn_radius_vector = myAstro.getRadiusVec();
//   siderealPlanetData.saturn_distance = myAstro.getDistance();
//   siderealPlanetData.saturn_ecliptic_lat = myAstro.getEclipticLatitude();
//   siderealPlanetData.saturn_ecliptic_long = myAstro.getEclipticLongitude();
//   myAstro.doXRiseSetTimes();
//   siderealPlanetData.saturn_r = myAstro.getRiseTime();
//   siderealPlanetData.saturn_s = myAstro.getSetTime();
// }

// void trackUranus(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doUranus();
//   siderealPlanetData.uranus_ra  = myAstro.getRAdec();
//   siderealPlanetData.uranus_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.uranus_az  = myAstro.getAzimuth();
//   siderealPlanetData.uranus_alt = myAstro.getAltitude();
//   siderealPlanetData.uranus_helio_ecliptic_lat = myAstro.getHelioLat();
//   siderealPlanetData.uranus_helio_ecliptic_long = myAstro.getHelioLong();
//   siderealPlanetData.uranus_radius_vector = myAstro.getRadiusVec();
//   siderealPlanetData.uranus_distance = myAstro.getDistance();
//   siderealPlanetData.uranus_ecliptic_lat = myAstro.getEclipticLatitude();
//   siderealPlanetData.uranus_ecliptic_long = myAstro.getEclipticLongitude();
//   myAstro.doXRiseSetTimes();
//   siderealPlanetData.uranus_r = myAstro.getRiseTime();
//   siderealPlanetData.uranus_s = myAstro.getSetTime();
// }

// void trackNeptune(double latitude, double longitude, int year, int month, int day, int hour, int minute, int second) {
//   myAstro.setLatLong(latitude, longitude);
//   myAstro.rejectDST();
//   myAstro.setGMTdate(year, month, day);
//   myAstro.setLocalTime(hour, minute, second);
//   myAstro.setGMTtime(hour, minute, second);
//   myAstro.doNeptune();
//   siderealPlanetData.neptune_ra  = myAstro.getRAdec();
//   siderealPlanetData.neptune_dec = myAstro.getDeclinationDec();
//   myAstro.doRAdec2AltAz();
//   siderealPlanetData.neptune_az  = myAstro.getAzimuth();
//   siderealPlanetData.neptune_alt = myAstro.getAltitude();
//   siderealPlanetData.neptune_helio_ecliptic_lat = myAstro.getHelioLat();
//   siderealPlanetData.neptune_helio_ecliptic_long = myAstro.getHelioLong();
//   siderealPlanetData.neptune_radius_vector = myAstro.getRadiusVec();
//   siderealPlanetData.neptune_distance = myAstro.getDistance();
//   siderealPlanetData.neptune_ecliptic_lat = myAstro.getEclipticLatitude();
//   siderealPlanetData.neptune_ecliptic_long = myAstro.getEclipticLongitude();
//   myAstro.doXRiseSetTimes();
//   siderealPlanetData.neptune_r = myAstro.getRiseTime();
//   siderealPlanetData.neptune_s = myAstro.getSetTime();
// }

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               MATRIX: SWITCH

void matrixSwitch() {

  /*
  compound condition checks, each resulting in zero/one at the final_bool. allowing for sextillions of combinations with the current data
  */

  // iterate over each relay matrix
  for (int Ri = 0; Ri < relayData.MAobject_RELAYS-1; Ri++) {
    // Serial.println("[Ri] " + String(Ri) + " [E] " + String(relayData.relays_enable[0][Ri]));
    if (relayData.relays_enable[0][Ri] == 1) {

      // temporary switch must be zero each time
      bool tmp_matrix[relayData.MAobject_RELAY_ELEMENTS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
      int count_none_function = 0;

      // iterate over each function name in the current relay matrix
      for (int Fi = 0; Fi < relayData.MAobject_RELAY_ELEMENTS; Fi++) {

        // uncomment to debug
        // Serial.println("[Ri] " + String(Ri));
        // Serial.println("[Fi] " + String(Fi));
        // Serial.println("[relayData.relays[Ri][Fi]] " + String(relayData.relays[Ri][Fi]));

        // for perfromance reasons logic may prefer adding functions from position zero else if position zero not populated then break to next inner matrix
        if ((strcmp(relayData.relays[Ri][Fi], relayData.default_relay_function) == 0) && (Fi == 0)) {break;}

        // put true in temporary matrix for functions set to none. there is one check to catch you if you do soft enable with no functions set.
        else if (strcmp(relayData.relays[Ri][Fi], relayData.default_relay_function) == 0) {tmp_matrix[Fi] = 1; count_none_function++;}

        // put true in temporary matrix if switch is enabled regardless of data. allows final bool true with no further requirements, even if all set $ENABLED, unlike if all set $NONE
        else if (strcmp(relayData.relays[Ri][Fi], relayData.default_enable_relay_function) == 0) {tmp_matrix[Fi] = 1;}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                                    TIME DATA

        else if (strcmp(relayData.relays[Ri][Fi], relayData.SecondsTimer) == 0) {tmp_matrix[Fi] = SecondsTimer(relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1], Ri);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                                       SATIO

        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        // check latitude and longitude in range: in_ranges_check_true(degrees_lat, matrx_x, degrees_lon, matrx_y, matrix_z)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesGNGGARange) == 0) {tmp_matrix[Fi] = in_ranges_check_true(satData.location_latitude_gngga, relayData.relays_data[Ri][Fi][0], satData.location_longitude_gngga, relayData.relays_data[Ri][Fi][1], relayData.relays_data[Ri][Fi][2]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLatGNRMCRange) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesLonGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.longitude_satio_gnrmc_in_range) == 0) {tmp_matrix[Fi] = in_range_check_true(satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        // check latitude and longitude in range: in_ranges_check_true(degrees_lat, matrx_x, degrees_lon, matrx_y, matrix_z)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.DegreesGNGGARange) == 0) {tmp_matrix[Fi] = in_ranges_check_true(satData.location_latitude_gnrmc, relayData.relays_data[Ri][Fi][0], satData.location_longitude_gnrmc, relayData.relays_data[Ri][Fi][1], relayData.relays_data[Ri][Fi][2]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                                        GNGGA

        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.latitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNGGARange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnggaData.longitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNGGARange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.utc_time), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PositioningStatusGNGGA) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.positioning_status), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SatelliteCountRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.satellite_count_gngga), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGANorth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "N", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGAEast) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "E", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGASouth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.latitude_hemisphere, "S", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNGGAWest) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnggaData.longitude_hemisphere, "W", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPSPrecisionRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.hdop_precision_factor), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGAOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGAUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGAEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.AltitudeGNGGARange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnggaData.altitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                                        GNRMC

        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCTimeGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_time), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LatGNRMCRange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.latitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LonGNRMCRange) == 0) {tmp_matrix[Fi] = in_range_check_true(atol(gnrmcData.longitude), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);} // is n in [2]range of [0]x (no y required)
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCNorth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "N", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCEast) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "E", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCSouth) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.latitude_hemisphere, "S", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HemisphereGNRMCWest) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.longitude_hemisphere, "W", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GroundSpeedGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_speed), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.HeadingGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.ground_heading), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.UTCDateGNRMCRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gnrmcData.utc_date), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PositioningStatusGNRMCA) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "A", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PositioningStatusGNRMCV) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.positioning_status, "V", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ModeGNRMCA) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "A", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ModeGNRMCD) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "D", 1);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.ModeGNRMCN) == 0) {tmp_matrix[Fi] = check_strncmp_true(gnrmcData.mode_indication, "N", 1);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                                        GPATT

        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.pitch), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.pitch), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.PitchGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.pitch), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RollGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.roll), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.YawGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.yaw), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GSTDataGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.gst_data), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.MileageGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.mileage), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTOver) == 0) {tmp_matrix[Fi] = check_over_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTUnder) == 0) {tmp_matrix[Fi] = check_under_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0]);}  
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.SpeedNumGPATTRange) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atol(gpattData.speed_num), relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][1]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.LineFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.line_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.InertialFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.run_inetial_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.INSGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.ins), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.RunStateFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.run_state_flag), relayData.relays_data[Ri][Fi][0]);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.StaticFlagGPATTEqual) == 0) {tmp_matrix[Fi] = check_equal_true(atol(gpattData.static_flag), relayData.relays_data[Ri][Fi][0]);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                           SIDEREAL TIME: SUN

        // // sun azimuth:
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SunAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // // sun altitude:
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SunAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.sun_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // // daytime: current time in range of sunrise and sunset
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.DayTime) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.sun_r, siderealPlanetData.sun_s);}
        // // nighttime: current time not in range of sunrise and sunset
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.NightTime) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.sun_r, siderealPlanetData.sun_s);}
        // // sunrise time less than current time: true after sunrise until midnight
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.Sunrise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.sun_r, atof(satData.hours_minutes));}
        // // sunset time less than current time: true after sunset until midnight                                                                  
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.Sunset) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.sun_s, atof(satData.hours_minutes));}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                          SIDEREAL TIME: MOON

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MoonAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MoonAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.moon_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.Moonrise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.moon_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.Moonset) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.moon_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MoonUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.moon_r, siderealPlanetData.moon_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MoonDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.moon_r, siderealPlanetData.moon_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MoonPhase) == 0) {tmp_matrix[Fi] = check_equal_true(siderealPlanetData.moon_p, relayData.relays_data[Ri][Fi][0]);}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                       SIDEREAL TIME: MERCURY

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MercuryAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MercuryAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mercury_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MercuryRise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.mercury_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MercurySet) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.mercury_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MercuryUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.mercury_r, siderealPlanetData.mercury_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MercuryDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.mercury_r, siderealPlanetData.mercury_s);}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                         SIDEREAL TIME: VENUS

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.VenusAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.VenusAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.venus_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.VenusRise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.venus_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.VenusSet) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.venus_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.VenusUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.venus_r, siderealPlanetData.venus_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.VenusDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.venus_r, siderealPlanetData.venus_s);}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                          SIDEREAL TIME: MARS

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MarsAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MarsAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.mars_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MarsRise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.mars_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MarsSet) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.mars_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MarsUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.mars_r, siderealPlanetData.mars_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.MarsDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.mars_r, siderealPlanetData.mars_s);}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                       SIDEREAL TIME: JUPITER

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.JupiterAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.JupiterAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.jupiter_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.JupiterRise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.jupiter_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.JupiterSet) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.jupiter_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.JupiterUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.jupiter_r, siderealPlanetData.jupiter_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.JupiterDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.jupiter_r, siderealPlanetData.jupiter_s);}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                        SIDEREAL TIME: SATURN

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SaturnAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SaturnAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.saturn_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SaturnRise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.saturn_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SaturnSet) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.saturn_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SaturnUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.saturn_r, siderealPlanetData.saturn_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.SaturnDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.saturn_r, siderealPlanetData.saturn_s);}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                        SIDEREAL TIME: URANUS

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.UranusAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.UranusAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.uranus_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.UranusRise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.uranus_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.UranusSet) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.uranus_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.UranusUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.uranus_r, siderealPlanetData.uranus_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.UranusDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.uranus_r, siderealPlanetData.uranus_s);}

        // // ----------------------------------------------------------------------------------------------------------------------------
        // //                                                                                                       SIDEREAL TIME: NEPTUNE

        // else if (strcmp(relayData.relays[Ri][Fi], relayData.NeptuneAzimuth) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_az, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.NeptuneAltitude) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(siderealPlanetData.neptune_alt, relayData.relays_data[Ri][Fi][0], relayData.relays_data[Ri][Fi][2]);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.NeptuneRise) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.neptune_r, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.NeptuneSet) == 0) {tmp_matrix[Fi] = check_under_true(siderealPlanetData.neptune_s, atof(satData.hours_minutes));}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.NeptuneUp) == 0) {tmp_matrix[Fi] = check_ge_and_le_true(atof(satData.hours_minutes), siderealPlanetData.neptune_r, siderealPlanetData.neptune_s);}
        // else if (strcmp(relayData.relays[Ri][Fi], relayData.NeptuneDown) == 0) {tmp_matrix[Fi] = check_ge_and_le_false(atof(satData.hours_minutes), siderealPlanetData.neptune_r, siderealPlanetData.neptune_s);}

        // ----------------------------------------------------------------------------------------------------------------------------
        //                                                                                                                     VALIDITY

        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNGGAValidChecksum) == 0) {tmp_matrix[Fi] = check_bool_true(gnggaData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNGGAInvalidChecksum) == 0) {tmp_matrix[Fi] = check_bool_false(gnggaData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNRMCValidChecksum) == 0) {tmp_matrix[Fi] = check_bool_true(gnrmcData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNRMCInvalidChecksum) == 0) {tmp_matrix[Fi] = check_bool_false(gnrmcData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPATTValidChecksum) == 0) {tmp_matrix[Fi] = check_bool_true(gpattData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPATTInvalidChecksum) == 0) {tmp_matrix[Fi] = check_bool_false(gpattData.valid_checksum);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNGGAValidCheckData) == 0) {tmp_matrix[Fi] = check_equal_true(gnggaData.check_data, 16);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNGGAInvalidCheckData) == 0) {tmp_matrix[Fi] = check_equal_false(gnggaData.check_data, 16);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNRMCValidCheckData) == 0) {tmp_matrix[Fi] = check_equal_true(gnrmcData.check_data, 14);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GNRMCInvalidCheckData) == 0) {tmp_matrix[Fi] = check_equal_false(gnrmcData.check_data, 14);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPATTValidCheckData) == 0) {tmp_matrix[Fi] = check_equal_true(gpattData.check_data, 41);}
        else if (strcmp(relayData.relays[Ri][Fi], relayData.GPATTInvalidCheckData) == 0) {tmp_matrix[Fi] = check_equal_false(gpattData.check_data, 41);}
      }
      
      // safety layer: disengage if all entries are $NONE
      if (count_none_function <= relayData.MAobject_RELAY_ELEMENTS-1) {

        // default final bool default is true: if a single false is found then final bool should be set to false and remain false
        bool final_bool = true;

        // debug (same as line below but with output)
        // for (int FC = 0; FC < relayData.MAobject_RELAY_ELEMENTS-1; FC++) {Serial.println("[tmp_matrix[FC]] " + String(tmp_matrix[FC])); if (tmp_matrix[FC] == 0) {final_bool = false;}}

        for (int FC = 0; FC < relayData.MAobject_RELAY_ELEMENTS-1; FC++) {if (tmp_matrix[FC] == 0) {final_bool = false; break;}}

        /*
        WARNING: why do you think you can trust the data you are receiving?
                 once you plug something into this, the 'satellites' are in control unless you have a way to override.
        */

        // debug (same as line below but with output)
        // if (final_bool == false) {Serial.println("[RELAY " + String(Ri) + "] inactive"); relayData.relays_bool[0][Ri] = 0;}
        // else if (final_bool == true) {Serial.println("[RELAY " + String(Ri) + "] active"); relayData.relays_bool[0][Ri] = 1;}

        if (final_bool == false) {relayData.relays_bool[0][Ri] = 0;}
        else if (final_bool == true) {relayData.relays_bool[0][Ri] = 1;}
      }
      else {Serial.println("[RELAY " + String(Ri) + "] WARNING: Matrix checks are enabled for an non configured matrix!");}
    }
    // handle Ri's that are disbaled
    else {relayData.relays_bool[0][Ri] = 0;}

    // Output relays bool matrix as a sentence that can easily be parsed by connected devices. 
    if (systemData.output_matrix_enabled == true) {
      memset(relayData.matrix_results_sentence, 0, sizeof(relayData.matrix_results_sentence));
      strcpy(relayData.matrix_results_sentence, "$MATRIXSWITCH,");
      for (int i=0; i < relayData.MAobject_RELAYS; i++) {
        if      (relayData.relays_bool[0][i] == 0) {strcat(relayData.matrix_results_sentence, "0,");}
        else if (relayData.relays_bool[0][i] == 1) {strcat(relayData.matrix_results_sentence, "1,");}
      }
      strcat(relayData.matrix_results_sentence, "*");
      relayData.checksum_i = getCheckSum(relayData.matrix_results_sentence);
      itoa(relayData.checksum_i, relayData.checksum_str, 10);
      strcat(relayData.matrix_results_sentence, relayData.checksum_str);
      Serial.println(relayData.matrix_results_sentence);
      }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   READ RXD 0

void readSerialCommands() {
    
  if ((Serial.available() > 0) && (menuData.menu_lock == false)){
    
    memset(serial0Data.BUFFER, 0, 1024);
    serial0Data.nbytes = (Serial.readBytesUntil('\n', serial0Data.BUFFER, sizeof(serial0Data.BUFFER)));
    // Serial.println(serial0Data.nbytes); // debug

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                        MATRIX: SET ENTRY

    if (strncmp(serial0Data.BUFFER, "$MATRIX_SET_ENTRY", 17) == 0) {
      matriobject_set_entry();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                             MATRIX: ENABLE/DISABLE ENTRY

    else if (strncmp(serial0Data.BUFFER, "$MATRIX_ENABLE_ENTRY", 19) == 0) {
      matriobject_set_enabled(true);
    }

    else if (strncmp(serial0Data.BUFFER, "$MATRIX_DISABLE_ENTRY", 21) == 0) {
      matriobject_set_enabled(false);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                      MATRIX: DISABLE ALL

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_DISABLE_ALL") == 0) {
      matrix_disable_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                                       MATRIX: ENABLE ALL

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_ENABLE_ALL") == 0) {
      matrix_enable_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                               MATRIX: TURN ALL RELAYS ON

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_RELAYS_ALL_ON") == 0) {
      relays_activate_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                              MATRIX: TURN ALL RELAYS OFF

    else if (strcmp(serial0Data.BUFFER, "$MATRIX_RELAYS_ALL_OFF") == 0) {
      relays_deactivate_all();
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                         SDCARD: SERIAL PRINT MATRIX FILE

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_READ_MATRIX") == 0) {
      sdcard_read_to_serial(SD, sdcardData.matrix_filepath);
    }

    // // ------------------------------------------------------------------------------------------------------------------------
    // //                                                                                            SDCARD: SAVE MATRIX TO SDCARD

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_SAVE_MATRIX") == 0) {
      sdcard_save_matrix(SD, sdcardData.matrix_filepath);
    }

    // // ------------------------------------------------------------------------------------------------------------------------
    // //                                                                                          SDCARD: LOAD MATRIX FROM SDCARD

    else if (strcmp(serial0Data.BUFFER, "$SDCARD_LOAD_MATRIX") == 0) {
      sdcard_load_matrix(SD, sdcardData.matrix_filepath);
    }

    // ------------------------------------------------------------------------------------------------------------------------
    //                                                                                              SATIO: CONVERT COORDINATES

    else if (strcmp(serial0Data.BUFFER, "$SATIO_CONVERT_COORDINATES_ON") == 0) {
      satio_convert_coordinates_on();
    }
    else if (strcmp(serial0Data.BUFFER, "$SATIO_CONVERT_COORDINATES_OFF") == 0) {
      satio_convert_coordinates_off();
    }

    // ------------------------------------------------------------------------------------------------------------------------

    else {
      Serial.println("[unknown] " + String(serial0Data.BUFFER));
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   READ RXD 1

void readGPS() {
  while(1) {
    if (Serial1.available() > 0) {
      memset(serial1Data.BUFFER, 0, sizeof(serial1Data.BUFFER));
      // After migrating to CYD, lets see if the Serial1 set pins will be be stable.
      serial1Data.nbytes = (Serial1.readBytesUntil('\n', serial1Data.BUFFER, sizeof(serial1Data.BUFFER)));
      // Serial.print(serial1Data.nbytes); Serial.print(" "); Serial.println(serial1Data.BUFFER); // debug
      
      
      // ------------------------------------------------------------------------------------------------------------------------
      //                                                                                                                    GNGGA
      
      if ((systemData.gngga_enabled == true) && (serial1Data.gngga_bool==false)){
        if (strncmp(serial1Data.BUFFER, "$GNGGA", 6) == 0) {
          if (systemData.output_gngga_enabled == true) {Serial.println(serial1Data.BUFFER);}
          memset(gnggaData.sentence, 0, sizeof(gnrmcData.sentence));
          strcpy(gnggaData.sentence, serial1Data.BUFFER);
          gnggaData.valid_checksum = validateChecksum(gnggaData.sentence);
          if (gnggaData.valid_checksum == true) {GNGGA(); serial1Data.collected++; serial1Data.gngga_bool = true;}
          else {gnggaData.bad_checksum_validity++;}
        }
      }

      // ------------------------------------------------------------------------------------------------------------------------
      //                                                                                                                    GNRMC

      else if ((systemData.gnrmc_enabled == true) && (serial1Data.gnrmc_bool==false)) {
        if (strncmp(serial1Data.BUFFER, "$GNRMC", 6) == 0) {
          if (systemData.output_gnrmc_enabled == true) {Serial.println(serial1Data.BUFFER);}
          memset(gnrmcData.sentence, 0, sizeof(gnrmcData.sentence));
          strcpy(gnrmcData.sentence, serial1Data.BUFFER);
          gnrmcData.valid_checksum = validateChecksum(gnrmcData.sentence);
          if (gnrmcData.valid_checksum == true) {GNRMC(); serial1Data.collected++; serial1Data.gnrmc_bool = true;}
          else {gnrmcData.bad_checksum_validity++;}
        }
      }

      // ------------------------------------------------------------------------------------------------------------------------
      //                                                                                                                    GPATT

      else if ((systemData.gpatt_enabled == true) && (serial1Data.gpatt_bool==false)) {
        if (strncmp(serial1Data.BUFFER, "$GPATT", 6) == 0) {
            if (systemData.output_gpatt_enabled == true) {Serial.println(serial1Data.BUFFER);}
            memset(gpattData.sentence, 0, sizeof(gpattData.sentence));
            strcpy(gpattData.sentence, serial1Data.BUFFER);
            gpattData.valid_checksum = validateChecksum(gpattData.sentence);
            if (gpattData.valid_checksum == true) {GPATT(); serial1Data.collected++; serial1Data.gpatt_bool = true;}
            else {gpattData.bad_checksum_validity++;}
        }
      }
    }
    // clear and exit
    if (serial1Data.collected==3) {serial1Data.collected=0; serial1Data.gngga_bool=false; serial1Data.gnrmc_bool=false; serial1Data.gpatt_bool=false; break;}
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                          TASK: MATRIX SWITCH

void MatrixSwitchTask() {
  if (systemData.matrix_enabled == true) {matrixSwitch();}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 TASK: SATIO

void getSATIOData() {
  if (systemData.satio_enabled == true) {extrapulatedSatData();}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       TASK: ELEMENTS COUNTER

void countRelaysEnabled(){
  relayData.relays_enabled_i = 0;
  relayData.relays_disabled_i = 0;
  for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) { if (relayData.relays_enable[0][Ri] == 1) {relayData.relays_enabled_i++;} else {relayData.relays_disabled_i++;} }
}

void countRelaysActive(){
  relayData.relays_active_i = 0;
  relayData.relays_inactive_i = 0;
  for (int Ri = 0; Ri < relayData.MAobject_RELAYS; Ri++) { if (relayData.relays_bool[0][Ri] == 1) {relayData.relays_active_i++;} else {relayData.relays_inactive_i++;} }
}

void CountElements() {
  countRelaysEnabled();
  countRelaysActive();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                 TASK: PLANETARY CALCULATIONS

// void trackPlanets() {
//   if (systemData.sidereal_track_sun == true) {trackSun(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_moon == true) {trackMoon(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_mercury == true) {trackMercury(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_venus == true) {trackVenus(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_mars == true) {trackMars(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_jupiter == true) {trackJupiter(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_saturn == true) {trackSaturn(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_uranus == true) {trackUranus(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
//   if (systemData.sidereal_track_neptune == true) {trackNeptune(satData.location_latitude_gngga,
//                                                         satData.location_longitude_gngga,
//                                                         atoi(satData.year_full),
//                                                         atoi(satData.month),
//                                                         atoi(satData.day),
//                                                         atoi(satData.hour),
//                                                         atoi(satData.minute),
//                                                         atoi(satData.second));}
// }


// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               DISPLAY STRUCT

struct VirtualMatrixSwitchStruct {
  int matrix_btn_w = 30;
  int matrix_btn_h = 25;
  int matrix_btn_sp_x = 2;
  int matrix_btn_sp_h = 2;
  int matrix_btn_x = 0;
  int matrix_btn_y = 35;
  int matrix_indi_h = 5;
  int matrix_enable_h = 15;
  int matrix_btn_iter_x = 0;
  int matrix_btn_w_loop = 0;
};
VirtualMatrixSwitchStruct vms;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 TOUCH STRUCT

struct TouchScreenStruct {
  int page0_x[10][2] = {
    {0, 35},
    {35, 60},
    {70, 90},
    {95, 115},
    {125, 145},
    {150, 170},
    {180, 200},
    {210, 230},
    {235, 260},
    {260, 290}
  };
  int page1_y[10][2] = {
    {50, 60},
    {70, 80},
    {90, 100},
    {105, 115},
    {125, 135},
    {145, 155},
    {160, 170},
    {180, 190},
    {195, 205},
    {210, 220}
  };
  int matrix_switch_cfg_r1h0 = 50;
  int matrix_switch_cfg_r1h1 = 70;
  int matrix_switch_ena_r1h0 = 75;
  int matrix_switch_ena_r1h1 = 85;
  int matrix_switch_cfg_r2h0 = 100;
  int matrix_switch_cfg_r2h1 = 120;
  int matrix_switch_ena_r2h0 = 125;
  int matrix_switch_ena_r2h1 = 135;
  int ts_t0 = millis();
  int ts_ti = 200;

  int max_homebtn_pages = 13;
  int homebtn_pages[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 100, 400, 401};
  bool homebutton_bool = false;

  int max_settingsbtn_pages = 3;
  int settingsbtn_pages[3] = {0, 400, 401};
  bool settingsbtn_bool = false;
};
TouchScreenStruct tss;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             DISPLAY TITLEBAR

void drawHomeBar() {
  // saves reiteration under certain conditions
  hud.setCursor(4,4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
  hud.print("Home");
}

void drawBack() {
  hud.setCursor(290,4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
  hud.print("Back");
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               TOUCH TITLEBAR

bool isTouchTitleBar(TouchPoint p) {

  // choose where home button will be registered
  for (int i=0; i<tss.max_homebtn_pages; i++) {if (menuData.page==tss.homebtn_pages[i]) {tss.homebutton_bool=true; break;} else {tss.homebutton_bool=false;}}
  // Serial.println(tss.homebutton_bool);
  if (tss.homebutton_bool==true) {
    if ((p.x >= 0 && p.x <= 40) && (p.y >= 0 && p.y <= 25)) {menuData.page=0; return true;}
  }

  // choose where settings button will be registered
  tss.settingsbtn_bool=false;
  for (int i=0; i<tss.max_settingsbtn_pages; i++) {if (menuData.page==tss.settingsbtn_pages[i]) {tss.settingsbtn_bool=true; break;} else {tss.settingsbtn_bool=false;}}
  // Serial.println(tss.settingsbtn_bool);
  if (tss.settingsbtn_bool==true) {
    if ((p.x >= 0 && p.x <= 40) && (p.y >= 0 && p.y <= 25)) {menuData.page=3; return true;}
  }
  return false;
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               DISPLAY PAGE 0

// todo: add LoadMatrixFileNumber to available matrix functions so that matrix files can be automatically loaded into memory by the matrix switch.

bool DisplayPage0() {
  // settings
  hud.setCursor(4,4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
  hud.print("Config");
  // check page here rather than in calling function so that we can see where we are when we're here
  if (menuData.page == 0) {
    // create virtual matrix switch: row 1
    vms.matrix_btn_w_loop = 0;
    vms.matrix_btn_y = 35;
    vms.matrix_btn_iter_x = 0;
    for (vms.matrix_btn_iter_x=0; vms.matrix_btn_iter_x<10; vms.matrix_btn_iter_x++) {
        // skip multiplying matrix_btn_w initially
        if (vms.matrix_btn_iter_x > 0) {vms.matrix_btn_w_loop=vms.matrix_btn_w;}
        // active/inactive indicator
        if (relayData.relays_bool[0][vms.matrix_btn_iter_x] == true) {
            hud.fillRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
            vms.matrix_btn_y,
            vms.matrix_btn_w,
            vms.matrix_indi_h,
            TFT_GREEN);}
        else {hud.fillRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
            vms.matrix_btn_y,
            vms.matrix_btn_w,
            vms.matrix_indi_h,
            TFTOBJ_COL0);}
    // configuration button
    hud.drawRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
    vms.matrix_btn_y+vms.matrix_indi_h+vms.matrix_btn_sp_h,
    vms.matrix_btn_w,
    vms.matrix_btn_h,
    TFTOBJ_COL0);
    // enable/disable button and indicator
    if (relayData.relays_enable[0][vms.matrix_btn_iter_x] == true) {
        hud.drawRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
        vms.matrix_btn_y+vms.matrix_indi_h+vms.matrix_btn_sp_h*2+vms.matrix_btn_h,
        vms.matrix_btn_w,
        vms.matrix_enable_h,
        TFT_GREEN);
        }
    else {hud.drawRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
        vms.matrix_btn_y+vms.matrix_indi_h+vms.matrix_btn_sp_h*2+vms.matrix_btn_h,
        vms.matrix_btn_w,
        vms.matrix_enable_h,
        TFTOBJ_COL0);}
    }
    // create virtual matrix switch: row 2
    vms.matrix_btn_w_loop = 0;
    vms.matrix_btn_y = 90;
    vms.matrix_btn_iter_x = 0;
    for (vms.matrix_btn_iter_x=0; vms.matrix_btn_iter_x<10; vms.matrix_btn_iter_x++) {
        // skip multiplying matrix_btn_w initially
        if (vms.matrix_btn_iter_x > 10) {vms.matrix_btn_w_loop=vms.matrix_btn_w;}
        // active/inactive indicator
        if (relayData.relays_bool[0][vms.matrix_btn_iter_x+10] == true) {
            hud.fillRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
            vms.matrix_btn_y,
            vms.matrix_btn_w,
            vms.matrix_indi_h,
            TFT_GREEN);}
        else {hud.fillRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
            vms.matrix_btn_y,
            vms.matrix_btn_w,
            vms.matrix_indi_h,
            TFTOBJ_COL0);}
    // configuration button
    hud.drawRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
    vms.matrix_btn_y+vms.matrix_indi_h+vms.matrix_btn_sp_h,
    vms.matrix_btn_w,
    vms.matrix_btn_h,
    TFTOBJ_COL0);
    // enable/disable button and indicator
    if (relayData.relays_enable[0][vms.matrix_btn_iter_x+10] == true) {
        hud.drawRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
        vms.matrix_btn_y+vms.matrix_indi_h+vms.matrix_btn_sp_h*2+vms.matrix_btn_h,
        vms.matrix_btn_w,
        vms.matrix_enable_h,
        TFT_GREEN);
        }
    else {hud.drawRect((vms.matrix_btn_iter_x*vms.matrix_btn_sp_x)+(vms.matrix_btn_x)+(vms.matrix_btn_iter_x*vms.matrix_btn_w),
        vms.matrix_btn_y+vms.matrix_indi_h+vms.matrix_btn_sp_h*2+vms.matrix_btn_h,
        vms.matrix_btn_w,
        vms.matrix_enable_h,
        TFTOBJ_COL0);}
    }
    // upper hud reflects simple on/off states as well as time and precision.
    // upper hud col 1 (connection and time)
    hud.setCursor(50,2); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<15; i++) {hud.print(" ");}
    hud.setCursor(50,2);
    hud.print("S  "); hud.print(gnggaData.satellite_count_gngga);

    hud.setCursor(50,12); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<15; i++) {hud.print(" ");}
    hud.setCursor(50,12);
    hud.print("T  "); hud.print(satData.sat_time_stamp_string);

    hud.setCursor(50,22); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<15; i++) {hud.print(" ");}
    hud.setCursor(50,22);
    hud.print("LT "); hud.print(satData.last_sat_time_stamp_str);

    // upper hud col 2 (precision)
    hud.setCursor(175,2); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(175,2);
    hud.print("PF "); hud.print(gnggaData.hdop_precision_factor);

    hud.setCursor(175,12); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(175,12);
    hud.print("PS "); hud.print(gnggaData.positioning_status);

    hud.setCursor(175,22); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(175,22);
    hud.print("PS "); hud.print(gnrmcData.positioning_status);

    // upper hud col 3 (inertial navigation system)
    hud.setCursor(230,2); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(230,2);
    hud.print("INS "); hud.print(gpattData.ins);

    hud.setCursor(230,12); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(230,12);
    hud.print("RIF "); hud.print(gpattData.run_inetial_flag);

    hud.setCursor(230,22); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(230,22);
    hud.print("RSF "); hud.print(gpattData.run_state_flag);

    // upper hud col 4 (inertial navigation system)
    hud.setCursor(280,2); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(280,2);
    hud.print("GST "); hud.print(gpattData.gst_data);

    hud.setCursor(280,12); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(280,12);
    hud.print("SF  "); hud.print(gpattData.static_flag);

    hud.setCursor(280,22); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<6; i++) {hud.print(" ");}
    hud.setCursor(280,22);
    hud.print("LF  "); hud.print(gpattData.line_flag);

    // lower hud reflects positional and rotational information as well as ground speed and mileage. 
    // lower hud col 1
    hud.setCursor(0,145); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<7; i++) {hud.print(" ");}
    hud.setCursor(0,145);
    hud.print("PIT "); hud.print(gpattData.pitch);

    hud.setCursor(0,155); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<7; i++) {hud.print(" ");}
    hud.setCursor(0,155);
    hud.print("ROL "); hud.print(gpattData.roll);

    hud.setCursor(0,165); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<7; i++) {hud.print(" ");}
    hud.setCursor(0,165);
    hud.print("YAW "); hud.print(gpattData.yaw);

    // lower hud col 2
    hud.setCursor(130,145); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<12; i++) {hud.print(" ");}
    hud.setCursor(130,145);
    hud.print("ALT "); hud.print(gnggaData.altitude);

    hud.setCursor(130,155); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<12; i++) {hud.print(" ");}
    hud.setCursor(130,155);
    hud.print("GS  "); hud.print(gnrmcData.ground_speed);

    hud.setCursor(130,165); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<12; i++) {hud.print(" ");}
    hud.setCursor(130,165);
    hud.print("MIL "); hud.print(gpattData.mileage);

    // lower hud col 3
    hud.setCursor(243,145); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<12; i++) {hud.print(" ");}
    hud.setCursor(243,145);
    hud.print("GH "); hud.print(gnrmcData.ground_heading);

    hud.setCursor(243,155); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<12; i++) {hud.print(" ");}
    hud.setCursor(243,155);
    hud.print(""); hud.print(gnggaData.latitude_hemisphere);
    hud.print("  "); hud.print(satData.location_latitude_gngga_str);

    hud.setCursor(243,165); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    for (int i=0; i<12; i++) {hud.print(" ");}
    hud.setCursor(243,165);
    hud.print(""); hud.print(gnggaData.longitude_hemisphere);
    hud.print("  "); hud.print(satData.location_longitude_gngga_str);

    return true;
    }
    else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 TOUCH PAGE 0

bool isTouchPage0(TouchPoint p) {
  // check page here rather than in calling function so that we can see where we are when we're here
  if (menuData.page == 0) {
    // page 0: matrix main btn (row 0)
    if (p.y >= tss.matrix_switch_cfg_r1h0 && p.y <= tss.matrix_switch_cfg_r1h1) {
      for (int i=0; i<10; i++) {
        if (p.x >= tss.page0_x[i][0] && p.x <= tss.page0_x[i][1]) {
          menuData.relay_select=i;
          Serial.print("[matrix switch setup] matrix "); Serial.println(menuData.relay_select);
          menuData.backpage=0;
          menuData.page=1;
          break;
        }
      }
    }
    // page 0: matrix main btn (row 1)
    else if (p.y >= tss.matrix_switch_cfg_r2h0 && p.y <= tss.matrix_switch_cfg_r2h1) {
      for (int i=0; i<10; i++) {
        if (p.x >= tss.page0_x[i][0] && p.x <= tss.page0_x[i][1]) {
          menuData.relay_select=i+10;
          Serial.print("[matrix switch setup] matrix "); Serial.println(menuData.relay_select);
          menuData.backpage=0;
          menuData.page=1;
          break;
        }
      }
    }
    // page 0: matrix enable disable (row 0)
    else if (p.y >= tss.matrix_switch_ena_r1h0 && p.y <= tss.matrix_switch_ena_r1h1) {
      for (int i=0; i<10; i++) {
        if (p.x >= tss.page0_x[i][0] && p.x <= tss.page0_x[i][1]) {
          relayData.relays_enable[0][i] ^= true;
          Serial.print("[matrix switch toggle] matrix "); Serial.println(i);
          break;
        }
      }
    }
    // page 0: matrix enable disable (row 1)
    else if (p.y >= tss.matrix_switch_ena_r2h0 && p.y <= tss.matrix_switch_ena_r2h1) {
      for (int i=0; i<10; i++) {
        if (p.x >= tss.page0_x[i][0] && p.x <= tss.page0_x[i][1]) {
          relayData.relays_enable[0][i+10] ^= true;
          Serial.print("[matrix switch toggle] matrix "); Serial.println(i+10);
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               DISPLAY PAGE 1

bool DisplayPage1() {
    // check page here rather than in calling function so that we can see where we are when we're here
    if (menuData.page == 1) {
        // display selected matrix switch setup configuration
        hud.fillRect(0, 0, 320, 240, TFT_BLACK);
        drawHomeBar();
        // page header
        hud.setCursor(150, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
        hud.print("Matrix "); hud.print(menuData.relay_select);
        // clear all functions on current matrix switch
        hud.setCursor(283,4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
        hud.print("CLEAR");
        // table headers
        hud.fillRect(0, 22, 320, 16, TFTOBJ_COL0);
        hud.setTextColor(TFTTXT_COLF_1, TFTTXT_COLB_1);
        hud.setCursor(4,   28); hud.print("Function");
        hud.setCursor(164, 28); hud.print("X");
        hud.setCursor(220, 28); hud.print("Y");
        hud.setCursor(278, 28); hud.print("Z");
        // table values
        for (int i=0; i<10; i++) {
        hud.drawRect(0, 43+i*20, 320, 16, TFTOBJ_COL0);
        hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
        hud.print(i); hud.print(" "); hud.print(relayData.relays[menuData.relay_select][i]);
        hud.setCursor(164, 47+i*20);
        hud.print(""); hud.print(relayData.relays_data[menuData.relay_select][i][0]);
        hud.setCursor(220, 47+i*20);
        hud.print(""); hud.print(relayData.relays_data[menuData.relay_select][i][1]);
        hud.setCursor(278, 47+i*20);
        hud.print(""); hud.print(relayData.relays_data[menuData.relay_select][i][2]);
        }
        return true;
    }
    else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 TOUCH PAGE 1

bool isTouchPage1(TouchPoint p) {
  // check page here rather than in calling function so that we can see where we are when we're here
  // it is strongly recommended to first disable a matrix switch before modifying its functionality (selecting functions, changing function values, clearing, etc),
  // this is good practice for when the switches have GPIO/relays. disabling can be done automatically but will limit potential scenarios, this way we can choose.
  if (menuData.page == 1) {
    // clear all. populates all functions for current matrix switch with $NONE.
    if ((p.x >= 260 && p.x <= 290) && (p.y >= 0 && p.y <= 25)) {
      for (int i=0; i<relayData.MAobject_RELAY_ELEMENTS; i++) {
        memset(relayData.relays[menuData.relay_select][i], 0, sizeof(relayData.relays[menuData.relay_select][i]));
        strcpy(relayData.relays[menuData.relay_select][i], relayData.function_names[0]);
        relayData.relays_data[menuData.relay_select][i][0] = 0.00;
        relayData.relays_data[menuData.relay_select][i][1] = 0.00;
        relayData.relays_data[menuData.relay_select][i][2] = 0.00;
        }
    }
    // page 1: Function Select
    if (p.x >= 0 && p.x <= 135) {
      for (int i=0; i<10; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          menuData.page=100;
          menuData.relay_function_select=i;
          menuData.backpage=1;
          break;
        }
      }
    }
    // page 1: select x
    else if (p.x >= 155 && p.x <= 195) {
      for (int i=0; i<10; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          menuData.page=300;
          menuData.backpage=1;
          menuData.relay_function_select=i;
          menuData.numpad_key=0;
          memset(menuData.input, 0, sizeof(menuData.input));
          sprintf(menuData.input, "%f", relayData.relays_data[menuData.relay_select][i][0]);
          break;
        }
      }
    }
    // page 1: select y
    else if (p.x >= 205 && p.x <= 245) {
      for (int i=0; i<10; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          menuData.page=300;
          menuData.backpage=1;
          menuData.relay_function_select=i;
          menuData.numpad_key=1;
          memset(menuData.input, 0, sizeof(menuData.input));
          sprintf(menuData.input, "%f", relayData.relays_data[menuData.relay_select][i][1]);
          break;
        }
      }
    }
    // page 1: select z
    else if (p.x >= 255 && p.x <= 285) {
      for (int i=0; i<10; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          menuData.page=300;
          menuData.backpage=1;
          menuData.relay_function_select=i;
          menuData.numpad_key=2;
          memset(menuData.input, 0, sizeof(menuData.input));
          sprintf(menuData.input, "%f", relayData.relays_data[menuData.relay_select][i][2]);
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                      DISPLAY MATRIX FUNCTION

bool DisplaySelectMatrixFunction() {

    // check page here rather than in calling function so that we can see where we are when we're here
    if (menuData.page == 100) {

        hud.fillRect(0, 0, 320, 240, TFT_BLACK);
        drawHomeBar();
        drawBack();

        // page header
        hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
        hud.print("Matrix "); hud.print(menuData.relay_select); hud.print(" Function "); hud.print(menuData.relay_function_select);

        // scroll buttons
        hud.fillRect(0, 22, 150, 16, TFTOBJ_COL0);
        hud.fillRect(170, 22, 150, 16, TFTOBJ_COL0);
        hud.setCursor(75, 27); hud.setTextColor(TFTTXT_COLF_1, TFTTXT_COLB_1); hud.print("UP");
        hud.setCursor(240, 27); hud.setTextColor(TFTTXT_COLF_1, TFTTXT_COLB_1); hud.print("DOWN");
        
        // values
        for (int i=0; i<10; i++) {
        hud.drawRect(0, 43+i*20, 320, 16, TFTOBJ_COL0);
        hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
        hud.print(menuData.function_index+i); hud.print(" "); hud.print(relayData.function_names[menuData.function_index+i]);
        }

        return true;
    }
    else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                        TOUCH MATRIX FUNCTION

bool isTouchSelectMatrixFunction(TouchPoint p) {
  // check page here rather than in calling function so that we can see where we are when we're here
  if (menuData.page == 100) {
    // back
    if ((p.x >= 260 && p.x <= 290) && (p.y >= 0 && p.y <= 25)) {menuData.page=1;}
    // previous list items
    if ((p.x >= 0 && p.x <= 140) && (p.y >= 35 && p.y <= 45)) {
      menuData.function_index--;
      if (menuData.function_index-10<0) {menuData.function_index=relayData.FUNCTION_NAMES_MAX-10;}
    }
    // next list items
    else if ((p.x >= 160 && p.x <= 290) && (p.y >= 35 && p.y <= 45)) {
      menuData.function_index++;
      if (menuData.function_index+10>relayData.FUNCTION_NAMES_MAX) {menuData.function_index=0;}
    }
    // select list item
    if (p.x >= 0 && p.x <= 320) {
      for (int i=0; i<10; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          memset(relayData.relays[menuData.relay_select][menuData.relay_function_select], 0, sizeof(relayData.relays[menuData.relay_select][menuData.relay_function_select]));
          strcpy(relayData.relays[menuData.relay_select][menuData.relay_function_select], relayData.function_names[i+menuData.function_index]);
          menuData.page=1;
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               DISPLAY NUMPAD

bool DisplayNumpad() {
    // check page here rather than in calling function so that we can see where we are when we're here
    if (menuData.page == 300) {
        hud.fillRect(0, 0, 320, 240, TFT_BLACK);
        drawBack();
        hud.setCursor(150-strlen(menuData.input), 30); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
        hud.print(menuData.input);
        hud.setCursor(0, 60);
        hud.print("               7          8          9");
        hud.setCursor(0, 100);
        hud.print("               4          5          6");
        hud.setCursor(0, 140);
        hud.print("               1          2          3");
        hud.setCursor(0, 180);
        hud.print("ENTER          0          .          -          DEL");
        hud.setCursor(0, 220);
        hud.print("                                                 CL");
        return true;
    }
    else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 TOUCH NUMPAD

bool isTouchNumpad(TouchPoint p) {
  // check page here rather than in calling function so that we can see where we are when we're here
  if (menuData.page == 300) {
    if      ((p.x >=  60 && p.x <= 120) && (p.y >=  55 && p.y <=  85)) {strcat(menuData.input, "7");} // 7
    else if ((p.x >=  60 && p.x <= 120) && (p.y >=  85 && p.y <= 120)) {strcat(menuData.input, "4");} // 4
    else if ((p.x >=  60 && p.x <= 120) && (p.y >= 120 && p.y <= 160)) {strcat(menuData.input, "1");} // 1
    else if ((p.x >=  60 && p.x <= 120) && (p.y >= 120 && p.y <= 160)) {strcat(menuData.input, "0");} // 0
    else if ((p.x >= 120 && p.x <= 180) && (p.y >=  55 && p.y <=  85)) {strcat(menuData.input, "8");} // 8
    else if ((p.x >= 120 && p.x <= 180) && (p.y >=  85 && p.y <= 120)) {strcat(menuData.input, "5");} // 5
    else if ((p.x >= 120 && p.x <= 180) && (p.y >= 120 && p.y <= 160)) {strcat(menuData.input, "2");} // 2
    else if ((p.x >= 120 && p.x <= 180) && (p.y >= 160 && p.y <= 195)) {strcat(menuData.input, ".");} // .
    else if ((p.x >= 180 && p.x <= 240) && (p.y >=  55 && p.y <=  85)) {strcat(menuData.input, "9");} // 9
    else if ((p.x >= 180 && p.x <= 240) && (p.y >=  85 && p.y <= 120)) {strcat(menuData.input, "6");} // 6
    else if ((p.x >= 180 && p.x <= 240) && (p.y >= 120 && p.y <= 160)) {strcat(menuData.input, "3");} // 3
    else if ((p.x >= 180 && p.x <= 240) && (p.y >= 160 && p.y <= 195)) {strcat(menuData.input, "-");} // -
    // enter
    else if ((p.x >= 0 && p.x <= 60) && (p.y >= 160 && p.y <= 195)) {
      if (menuData.numpad_key == 0) {char *ptr; relayData.relays_data[menuData.relay_select][menuData.relay_function_select][0] = strtod(menuData.input, &ptr);} // x
      if (menuData.numpad_key == 1) {char *ptr; relayData.relays_data[menuData.relay_select][menuData.relay_function_select][1] = strtod(menuData.input, &ptr);} // y
      if (menuData.numpad_key == 2) {char *ptr; relayData.relays_data[menuData.relay_select][menuData.relay_function_select][2] = strtod(menuData.input, &ptr);} // z
      menuData.page=1;
    }
    // delete
    else if ((p.x >= 240 && p.x <= 285) && (p.y >= 160 && p.y <= 195)) {menuData.input[strlen(menuData.input)-1] = '\0';}
    // clear
    else if ((p.x >= 240 && p.x <= 285) && (p.y >= 195 && p.y <= 225)) {memset(menuData.input, 0, sizeof(menuData.input));}
    // back
    else if ((p.x >= 260 && p.x <= 290) && (p.y >= 0 && p.y <= 25)) {memset(menuData.input, 0, sizeof(menuData.input)); menuData.page = menuData.backpage;}
    return true;
  }
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     SETTINGS

struct SettingsDataStruct {

  int max_settings0values = 7;
  char settings0values[7][56] = {
    "System",  // p4
    "Matrix",  // p5
    "GPS",     // p6
    "Serial",  // p7
    "File",    // p8
    "Time",    // p9
    "Display", // p10
  };

  int max_settingsystemvalues = 1;
  char settingsystemvalues[1][56] = {
    "AUTO RESUME", // autoresume_enabled
  };

  int max_settingsmatrixvalues = 1;
  char settingsmatrixvalues[1][56] = {
    "ENABLE MATRIX"
  };

  int max_settingsgpsvalues = 4;
  char settingsgpsvalues[4][56] = {
    "ENABLE SATIO",  // enables/disables satio processing of gps data
    "ENABLE GNGGA",  // enables/disables parsing of gngga sentence
    "ENABLE GNRMC",  // enables/disables parsing of gnrmc sentence
    "ENABLE GPATT",  // enables/disables parsing of gpatt sentence
  };

  int max_settingsserialvalues = 5;
  char settingsserialvalues[5][56] = {
    "ENABLE SATIO",  // enables/disables serial output of satio sentence
    "ENABLE GNGGA",  // enables/disables serial output of gngga sentence
    "ENABLE GNRMC",  // enables/disables serial output of gnrmc sentence
    "ENABLE GPATT",  // enables/disables serial output of gpatt sentence
    "ENABLE MATRIX", // enables/disables serial output of gpatt sentence
    // coordinate_conversion_mode (GNGGA/GNRMC)
  };

  int max_settingsfilevalues = 8;
  char settingsfilevalues[8][56] = {
    "SYSTEM FILE",
    "SAVE SYSTEM FILE",
    "",
    "MATRIX FILE",
    "NEW MATRIX FILE",
    "SAVE MATRIX FILE",
    "LOAD MATRIX FILE",
    "DELETE MATRIX FILE",
  };

  int max_settingstimevalues = 2;
  char settingstimevalues[2][56] = {
    "UTC OFFSET",  // can be used to offset hours (+/-) from UTC and can also be used to account for daylight saving. notice this is not called timezone or daylight saving.
    "OFFSET FLAG", // 0: add hours to time, 1: deduct hours from time
  };

  int max_settingsdisplayvalues = 4;
  char settingsdisplayvalues[4][56] = {
    "BRIGHTNESS", //
    "AUTO DIM",   //
    "AUTO OFF",   // 
    "FLIP",       // display_flip_vertically
  };
};
SettingsDataStruct sData;

bool DisplaySettings0() {
  // check page here rather than in calling function so that we can see where we are when we're here
  if (menuData.page == 3) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();

    // page header
    hud.setCursor(150, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("Settings");

    // values
    for (int i=0; i<sData.max_settings0values; i++) {
    hud.drawRect(0, 43+i*20, 320, 16, TFTOBJ_COL0); // 320px/4columns=80 - 4spacing=76
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sData.settings0values[i]);
    }

    return true;
  }
  else {return false;}
}

bool isDisplaySettings0(TouchPoint p) {
  // check page here rather than in calling function so that we can see where we are when we're here
  if (menuData.page == 3) {

    // select list item column 0
    if (p.x >= 0 && p.x <= 295) {
      for (int i=0; i<sData.max_settings0values; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          menuData.page=i+4; // settings page 0 is on page 3 so make subsequent settings pages 4+
          break;
        }
      }
    }

    return true;
  }
  else {return false;}
}

bool DisplaySettingsSystem() {
  if (menuData.page == 4) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("System Settings");
    // values
    for (int i=0; i<sData.max_settingsystemvalues; i++) {
    hud.drawRect(0, 43+i*20, 150, 16, TFTOBJ_COL0);
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sData.settingsystemvalues[i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsSystem(TouchPoint p) {
  if (menuData.page == 4) {
    // select list item column 0
    if (p.x >= 0 && p.x <= 140) {
      for (int i=0; i<sData.max_settingsystemvalues; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          Serial.print("[settings] system item "); Serial.println(sData.settingsystemvalues[i]);
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsMatrix() {
  if (menuData.page == 5) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("Matrix Settings");
    // values
    for (int i=0; i<sData.max_settingsmatrixvalues; i++) {
    hud.drawRect(0, 43+i*20, 150, 16, TFTOBJ_COL0);
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sData.settingsmatrixvalues[i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsMatrix(TouchPoint p) {
  if (menuData.page == 5) {
    // select list item column 0
    if (p.x >= 0 && p.x <= 140) {
      for (int i=0; i<sData.max_settingsmatrixvalues; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          Serial.print("[settings] matrix item "); Serial.println(sData.settingsmatrixvalues[i]);
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsGPS() {
  if (menuData.page == 6) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("GPS Settings");
    // values
    for (int i=0; i<sData.max_settingsgpsvalues; i++) {
    hud.drawRect(0, 43+i*20, 150, 16, TFTOBJ_COL0); // 320px/4columns=80 - 4spacing=76
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    if      (i==0) {if (systemData.satio_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    else if (i==1) {if (systemData.gngga_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    else if (i==2) {if (systemData.gnrmc_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    else if (i==3) {if (systemData.gpatt_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    hud.print(sData.settingsgpsvalues[i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsGPS(TouchPoint p) {
  if (menuData.page == 6) {
    // select list item column 0
    if (p.x >= 0 && p.x <= 140) {
      for (int i=0; i<sData.max_settingsgpsvalues; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          Serial.print("[settings] gps item "); Serial.println(sData.settingsgpsvalues[i]);
          if      (i==0) {systemData.satio_enabled ^= true;}
          else if (i==1) {systemData.gngga_enabled ^= true;}
          else if (i==2) {systemData.gnrmc_enabled ^= true;}
          else if (i==3) {systemData.gpatt_enabled ^= true;}
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsSerial() {
  if (menuData.page == 7) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("Serial Settings");
    // values
    for (int i=0; i<sData.max_settingsserialvalues; i++) {
    hud.drawRect(0, 43+i*20, 150, 16, TFTOBJ_COL0); // 320px/4columns=80 - 4spacing=76
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    if      (i==0) {if (systemData.output_satio_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    else if (i==1) {if (systemData.output_gngga_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    else if (i==2) {if (systemData.output_gnrmc_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    else if (i==3) {if (systemData.output_gpatt_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    else if (i==4) {if (systemData.output_matrix_enabled==true) {hud.setTextColor(TFT_GREEN, TFTTXT_COLB_0);}}
    hud.print(sData.settingsserialvalues[i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsSerial(TouchPoint p) {
  if (menuData.page == 7) {
    // select list item column 0
    if (p.x >= 0 && p.x <= 140) {
      for (int i=0; i<sData.max_settingsserialvalues; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          Serial.print("[settings] serial item "); Serial.println(sData.settingsserialvalues[i]);
          if      (i==0) {systemData.output_satio_enabled ^= true;}
          else if (i==1) {systemData.output_gngga_enabled ^= true;}
          else if (i==2) {systemData.output_gnrmc_enabled ^= true;}
          else if (i==3) {systemData.output_gpatt_enabled ^= true;}
          else if (i==4) {systemData.output_matrix_enabled ^= true;}
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsFile() {
  if (menuData.page == 8) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("File Settings");
    // values
    for (int i=0; i<sData.max_settingsfilevalues; i++) {
    hud.drawRect(0, 43+i*20, 150, 16, TFTOBJ_COL0);
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sData.settingsfilevalues[i]);
    // display system configuration filepath
    if (i==0) {
      hud.drawRect(150, 43+i*20, 170, 16, TFTOBJ_COL0);
      hud.setCursor(154, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
      hud.print(sdcardData.sysconf);
    }
    // display current matrix filepath
    else if (i==3) {
      hud.drawRect(150, 43+i*20, 170, 16, TFTOBJ_COL0);
      hud.setCursor(154, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
      hud.print(sdcardData.matrix_filepath);
    }
    }
    
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsFile(TouchPoint p) {
  if (menuData.page == 8) {
    // select list item column 0
    if (p.x >= 0 && p.x <= 140) {
      for (int i=0; i<sData.max_settingsfilevalues; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          Serial.print("[settings] file item "); Serial.println(sData.settingsfilevalues[i]);
          // values
          if      (i==1) {sdcard_save_system_configuration(SD, sdcardData.sysconf, 0);}
          else if (i==4) {sdcard_calculate_filename_create(SD, "/MATRIX/", "MATRIX", ".SAVE"); zero_matrix();}
          else if (i==5) {sdcard_save_matrix(SD, sdcardData.matrix_filepath);}
          else if (i==6) {sdcard_list_matrix_files(SD, "/MATRIX/", "MATRIX", ".SAVE"); menuData.page=400;}
          else if (i==7) {sdcard_list_matrix_files(SD, "/MATRIX/", "MATRIX", ".SAVE"); menuData.page=401;}
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsLoadMatrix() {
  if (menuData.page == 400) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    drawBack();

    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("Load Matrix File");

    // scroll buttons
    hud.fillRect(0, 22, 150, 16, TFTOBJ_COL0);
    hud.fillRect(170, 22, 150, 16, TFTOBJ_COL0);
    hud.setCursor(75, 27); hud.setTextColor(TFTTXT_COLF_1, TFTTXT_COLB_1); hud.print("UP");
    hud.setCursor(240, 27); hud.setTextColor(TFTTXT_COLF_1, TFTTXT_COLB_1); hud.print("DOWN");
    
    // values
    for (int i=0; i<10; i++) {
    hud.drawRect(0, 43+i*20, 320, 16, TFTOBJ_COL0);
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sdcardData.matrix_filenames[menuData.matrix_filenames_index+i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsLoadMatrix(TouchPoint p) {
  if (menuData.page == 400) {
    // back
    if ((p.x >= 260 && p.x <= 290) && (p.y >= 0 && p.y <= 25)) {menuData.page=8;}
    // previous list items
    if ((p.x >= 0 && p.x <= 140) && (p.y >= 35 && p.y <= 45)) {
      menuData.matrix_filenames_index--;
      if (menuData.matrix_filenames_index-10<0) {menuData.matrix_filenames_index=sdcardData.max_matrix_filenames-10;}
    }
    // next list items
    else if ((p.x >= 160 && p.x <= 290) && (p.y >= 35 && p.y <= 45)) {
      menuData.matrix_filenames_index++;
      if (menuData.matrix_filenames_index+10>sdcardData.max_matrix_filenames) {menuData.matrix_filenames_index=0;}
    }
    // select list item
    if (p.x >= 0 && p.x <= 320) {
      for (int i=0; i<10; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          sdcard_load_matrix(SD, sdcardData.matrix_filenames[i]);
          menuData.page=8;
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsDeleteMatrix() {
  if (menuData.page == 401) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    drawBack();

    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("Delete Matrix File");

    // scroll buttons
    hud.fillRect(0, 22, 150, 16, TFTOBJ_COL0);
    hud.fillRect(170, 22, 150, 16, TFTOBJ_COL0);
    hud.setCursor(75, 27); hud.setTextColor(TFTTXT_COLF_1, TFTTXT_COLB_1); hud.print("UP");
    hud.setCursor(240, 27); hud.setTextColor(TFTTXT_COLF_1, TFTTXT_COLB_1); hud.print("DOWN");
    
    // values
    for (int i=0; i<10; i++) {
    hud.drawRect(0, 43+i*20, 320, 16, TFTOBJ_COL0);
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sdcardData.matrix_filenames[menuData.matrix_filenames_index+i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsDeleteMatrix(TouchPoint p) {
  if (menuData.page == 401) {
    // back
    if ((p.x >= 260 && p.x <= 290) && (p.y >= 0 && p.y <= 25)) {menuData.page=8;}
    // previous list items
    if ((p.x >= 0 && p.x <= 140) && (p.y >= 35 && p.y <= 45)) {
      menuData.matrix_filenames_index--;
      if (menuData.matrix_filenames_index-10<0) {menuData.matrix_filenames_index=sdcardData.max_matrix_filenames-10;}
    }
    // next list items
    else if ((p.x >= 160 && p.x <= 290) && (p.y >= 35 && p.y <= 45)) {
      menuData.matrix_filenames_index++;
      if (menuData.matrix_filenames_index+10>sdcardData.max_matrix_filenames) {menuData.matrix_filenames_index=0;}
    }
    // select list item
    if (p.x >= 0 && p.x <= 320) {
      for (int i=0; i<10; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          sdcard_delete_matrix(SD, sdcardData.matrix_filenames[i]);
          menuData.page=8;
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsTime() {
  if (menuData.page == 9) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("Time Settings");
    // values
    for (int i=0; i<sData.max_settingstimevalues; i++) {
    hud.drawRect(0, 43+i*20, 150, 16, TFTOBJ_COL0);
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sData.settingstimevalues[i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsTime(TouchPoint p) {
  if (menuData.page == 9) {
    // select list item column 0
    if (p.x >= 0 && p.x <= 140) {
      for (int i=0; i<sData.max_settingstimevalues; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          Serial.print("[settings] time item "); Serial.println(sData.settingstimevalues[i]);
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

bool DisplaySettingsDisplay() {
  if (menuData.page == 10) {
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);
    drawHomeBar();
    // page header
    hud.setCursor(100, 4); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print("Display Settings");
    // values
    for (int i=0; i<sData.max_settingsdisplayvalues; i++) {
    hud.drawRect(0, 43+i*20, 150, 16, TFTOBJ_COL0);
    hud.setCursor(4, 47+i*20); hud.setTextColor(TFTTXT_COLF_0, TFTTXT_COLB_0);
    hud.print(sData.settingsdisplayvalues[i]);
    }
    return true;
  }
  else {return false;}
}

bool isDisplaySettingsDisplay(TouchPoint p) {
  if (menuData.page == 10) {
    // select list item column 0
    if (p.x >= 0 && p.x <= 140) {
      for (int i=0; i<sData.max_settingsdisplayvalues; i++) {
        if (p.y >= tss.page1_y[i][0] && p.y <= tss.page1_y[i][1]) {
          Serial.print("[settings] display item "); Serial.println(sData.settingsdisplayvalues[i]);
          break;
        }
      }
    }
    return true;
  }
  else {return false;}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               UPDATE DISPLAY

void UpdateDisplay(void * pvParameters) {
  // populate strite according to page then display
  while (1) {
    delay(1);
    // Create an 8-bit sprite 70x 80 pixels (uses 5600 bytes of RAM)
    hud.setColorDepth(8);
    hud.createSprite(320, 240);
    hud.fillSprite(TFT_TRANSPARENT);
    hud.fillRect(0, 0, 320, 240, TFT_BLACK);

    // menuData.page=8;  // force a specific page to be displayed (dev)

    // determine which sprite to build
    bool checktouch = false;
    if (checktouch == false) {checktouch = DisplayPage0();}
    if (checktouch == false) {checktouch = DisplayPage1();}
    if (checktouch == false) {checktouch = DisplaySelectMatrixFunction();}
    if (checktouch == false) {checktouch = DisplayNumpad();}
    if (checktouch == false) {checktouch = DisplaySettings0();}
    if (checktouch == false) {checktouch = DisplaySettingsSystem();}
    if (checktouch == false) {checktouch = DisplaySettingsMatrix();}
    if (checktouch == false) {checktouch = DisplaySettingsGPS();}
    if (checktouch == false) {checktouch = DisplaySettingsSerial();}
    if (checktouch == false) {checktouch = DisplaySettingsFile();}
    if (checktouch == false) {checktouch = DisplaySettingsTime();}
    if (checktouch == false) {checktouch = DisplaySettingsDisplay();}
    if (checktouch == false) {checktouch = DisplaySettingsLoadMatrix();}
    if (checktouch == false) {checktouch = DisplaySettingsDeleteMatrix();}

    // display the sprite and free memory
    hud.pushSprite(0, 0, TFT_TRANSPARENT);
    hud.deleteSprite();
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                            TOUCHSCREEN INPUT

void TouchScreenInput( void * pvParameters ) {
  // keep looping because this function runs as its own task
  while (1) {
    delay(1);
    // get touch data and only delay if current time in threshold range of previous touch time
    TouchPoint p = ts.getTouch();
    if (p.zRaw > 400) {
      if (millis() >= tss.ts_t0+tss.ts_ti) {
        tss.ts_t0 = millis();
        Serial.print("[ts debug] x:"); Serial.print(p.x); Serial.print(" y:"); Serial.print(p.y); Serial.print(" z:"); Serial.println(p.zRaw);
        // check touchscreen efficiently
        bool checktouch = false;
        if (checktouch == false) {checktouch = isTouchTitleBar(p);}
        if (checktouch == false) {checktouch = isTouchPage0(p);}
        if (checktouch == false) {checktouch = isTouchPage1(p);}
        if (checktouch == false) {checktouch = isTouchNumpad(p);}
        if (checktouch == false) {checktouch = isTouchSelectMatrixFunction(p);}
        if (checktouch == false) {checktouch = isDisplaySettings0(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsSystem(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsMatrix(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsGPS(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsSerial(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsFile(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsTime(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsDisplay(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsLoadMatrix(p);}
        if (checktouch == false) {checktouch = isDisplaySettingsDeleteMatrix(p);}
      }
    }
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SETUP

void setup() {

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                              SETUP: SERIAL

  Serial.begin(115200);
  while(!Serial);
  // ESP32 can map hardware serial to alternative pins. Map Serial1 for GPS module to the following, we will need this on CYD
  Serial1.setPins(gpsrxpin, gpstxpin, ctsPin, rtsPin);
  Serial1.begin(115200);

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                           SETUP: CORE INFO

  delay(1000);
  Serial.println("Running on Core: " + String(xPortGetCoreID()));

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                             SETUP: DISPLAY

  // Start the SPI for the touch screen and init the TS library
  ts.begin();
  // Start the tft display and set it to black
  tft.init();
  tft.setRotation(1); //This is the display in landscape

  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(FONT5X7_H);

  // Create touchscreen task to increase performance (core 0 also found to be best for this task)
  xTaskCreatePinnedToCore(
      TouchScreenInput, /* Function to implement the task */
      "TSTask",         /* Name of the task */
      10000,            /* Stack size in words */
      NULL,             /* Task input parameter */
      0,                /* Priority of the task */
      &TSTask,          /* Task handle. */
      0);               /* Core where the task should run */
  
  // Create display task to increase performance (core 0 also found to be best for this task)
  xTaskCreatePinnedToCore(
      UpdateDisplay,       /* Function to implement the task */
      "UpdateDisplayTask", /* Name of the task */
      10000,               /* Stack size in words */
      NULL,                /* Task input parameter */
      0,                   /* Priority of the task */
      &UpdateDisplayTask,  /* Task handle. */
      0);                  /* Core where the task should run */

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                                SETUP: WIRE

  // Wire.begin();

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                    SETUP: SIDEREAL PLANETS

  // myAstro.begin();

  // --------------------------------------------------------------------------------------------------------------------------
  //                                                                                                              SETUP: SDCARD

  init_sdcard();
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

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MAIN LOOP

void loop() {

  // store current time to measure this loop time
  timeData.mainLoopTimeStart = micros();

  // readSerialCommands();
  readGPS();
  getSATIOData();
  // trackPlanets();
  MatrixSwitchTask();
  CountElements();
  // UpdateDisplay();

  // store time taken to complete
  timeData.mainLoopTimeTaken = micros() - timeData.mainLoopTimeStart;
  if (timeData.mainLoopTimeTaken > timeData.mainLoopTimeTakenMax) {timeData.mainLoopTimeTakenMax = timeData.mainLoopTimeTaken;}
  if (timeData.mainLoopTimeTaken < timeData.mainLoopTimeTakenMin) {timeData.mainLoopTimeTakenMin = timeData.mainLoopTimeTaken;}
  // Serial.print("micros: "); Serial.println(timeData.mainLoopTimeTaken);

  // keep track of time in seconds
  time_counter();

  // delay(1000);
}
