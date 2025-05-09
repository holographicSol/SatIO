/*

9-Axis Gyro Module written by Benjamin Jack Cullen and adapted from official example code written by Witmotion.

The WT901 is a IMU sensor device, detecting acceleration, angular velocity, angle as well as magnetic filed.

Configurable over Serial.
Configurable over IIC.

roll: x9 LEDs
pitch: x9 LEDs
yaw: x1 LED (North)

Data in: x1 LED (yellow)
Data out: x1 LED (red)
baud rate: x1 LED (red, yellow, green, blue, purple, white)

*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES
// ----------------------------------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include "./REG.h"
#include "./wit_c_sdk.h"
#include <Wire.h>
#include <FastLED.h>

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     INDICATORS
// ------------------------------------------------------------------------------------------------------------------------------

/*
Indicators:
0-19: Matrix Switch State
20: MATRIX IO ENABLED/DISABLED
21: DATA
22: OVERLOAD
23: SIGNAL
*/

// INDICATORS
#define NUM_LEDS 18
#define DATA_PIN 12
CRGB leds[NUM_LEDS];

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        WT901
// ----------------------------------------------------------------------------------------------------------------------------

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate=0, s_cCmd=0xff;
char ucData[50];
void CopeCmdData();
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8]={0,4800, 9600, 19200, 38400, 57600, 115200, 230400};
float fAcc[3], fGyro[3], fAngle[3];

// -------------------------------------------------
// configuration flags
// -------------------------------------------------
bool enable_serial_output_content=false;
bool enable_resolution_compensation_acc=false;
bool enable_resolution_compensation_ang=false;
bool enable_resolution_compensation_gyr=false;
bool enable_resolution_compensation_mag=true;

// ----------------------------------------------------------------------------------------------------
// resolution compensation increases probability of catching value spikes.
// this is useful where a master may be occupied and would otherwise miss potentially interesting data.
// enabled: return max spike value (from values stored between each request) to master.
// disabled: return most recent value to master.
// ----------------------------------------------------------------------------------------------------

// -------------------------------------------------
// acceleration resolution compensation
// -------------------------------------------------
float acc_x_low=0;
float acc_x_high=0;
float acc_y_low=0;
float acc_y_high=0;
float acc_z_low=0;
float acc_z_high=0;
float tmp_acc_x0=0;
float tmp_acc_y0=0;
float tmp_acc_z0=0;
// -------------------------------------------------
// angle resolution compensation
// -------------------------------------------------
float ang_x_low=0;
float ang_x_high=0;
float ang_y_low=0;
float ang_y_high=0;
float ang_z_low=0;
float ang_z_high=0;
float tmp_ang_x0=0;
float tmp_ang_y0=0;
float tmp_ang_z0=0;
// -------------------------------------------------
// gyro resolution compensation
// -------------------------------------------------
float gyr_x_low=0;
float gyr_x_high=0;
float gyr_y_low=0;
float gyr_y_high=0;
float gyr_z_low=0;
float gyr_z_high=0;
float tmp_gyr_x0=0;
float tmp_gyr_y0=0;
float tmp_gyr_z0=0;
// -------------------------------------------------
// magnetic field resolution compensation
// -------------------------------------------------
float mag_x_low=0;
float mag_x_high=0;
float mag_y_low=0;
float mag_y_high=0;
float mag_z_low=0;
float mag_z_high=0;
float tmp_mag_x0=0;
float tmp_mag_y0=0;
float tmp_mag_z0=0;

// -------------------------------------------------
// angle percentages 
// -------------------------------------------------
float mapped_ang_x=0;
float mapped_ang_y=0;
float ang_x_percent=0;
float ang_y_percent=0;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA
// ------------------------------------------------------------------------------------------------------------------------------

#define SLAVE_ADDR 16

struct I2CLinkStruct {
  char * token;
  byte OUTPUT_BUFFER[32];
  char INPUT_BUFFER[32];
  char TMP_BUFFER0[32];
  char TMP_BUFFER1[32];
  int request_counter=0;
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               CLEAR TMP BUFFER
// ------------------------------------------------------------------------------------------------------------------------------

void clearTMPBuffer() {
  memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
  memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             I2C RECEIVE EVENTS
// ------------------------------------------------------------------------------------------------------------------------------

void receiveEvent(int) {

  // Serial.println("[receiveEvent]");

  // ------------------------------------------------
  // read incoming data
  // ------------------------------------------------
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  // Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));

  // ------------------------------------------------
  // data: set request counter for next request event
  // ------------------------------------------------
  I2CLink.token=strtok(I2CLink.INPUT_BUFFER, ",");
  if (strcmp(I2CLink.token, "$A")==0) {I2CLink.request_counter=0;}
  else if (strcmp(I2CLink.token, "$B")==0) {I2CLink.request_counter=1;}
  else if (strcmp(I2CLink.token, "$C")==0) {I2CLink.request_counter=2;}
  else if (strcmp(I2CLink.token, "$D")==0) {I2CLink.request_counter=3;}
  // ------------------------------------------------
  // configuration: calibration
  // ------------------------------------------------
  else if (strcmp(I2CLink.token, "$CAL-ACC")==0) {if (WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");}
  else if (strcmp(I2CLink.token, "$CAL-MAG0")==0) {if (WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");}
  else if (strcmp(I2CLink.token, "$CAL-MAG1")==0) {if (WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");}
  // ------------------------------------------------
  // configuration: bandwidth
  // ------------------------------------------------
  else if (strcmp(I2CLink.token, "$BANDWIDTH-INCREASE")==0) {if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");}
  else if (strcmp(I2CLink.token, "$BANDWIDTH-REDUCE")==0) {if (WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");}
  // ------------------------------------------------
  // configuration: baudrate
  // ------------------------------------------------
  else if (strcmp(I2CLink.token, "$BAUDRATE-INCREASE")==0) {
    if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
    else {Serial1.begin(c_uiBaud[WIT_BAUD_115200]); Serial.print(" 115200 Baud rate modified successfully\r\n");}
  }
  else if (strcmp(I2CLink.token, "$BAUDRATE-REDUCE")==0) {
    if (WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
    else {Serial1.begin(c_uiBaud[WIT_BAUD_9600]); Serial.print(" 9600 Baud rate modified successfully\r\n");}
  }
  // ------------------------------------------------
  // configuration: return rate
  // ------------------------------------------------
  else if (strcmp(I2CLink.token, "$RETURNRATE-INCREASE")==0) {
    if (WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
    else Serial.print("\r\nSet Baud Success\r\n");
  }
  else if (strcmp(I2CLink.token, "$RETURNRATE-REDUCE")==0) {
    if (WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)  Serial.print("\r\nSet Baud Error\r\n");
    else Serial.print("\r\nSet Baud Success\r\n");
  }
  // ------------------------------------------------
  // configuration: resolution compensation
  // ------------------------------------------------
  else if (strcmp(ucData, "$RC-ACC")==0) {enable_resolution_compensation_acc=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_acc));}
  else if (strcmp(ucData, "$CV-ACC")==0) {enable_resolution_compensation_acc=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_acc));}
  else if (strcmp(ucData, "$RC-ANG")==0) {enable_resolution_compensation_ang=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_ang));}
  else if (strcmp(ucData, "$CV-ANG")==0) {enable_resolution_compensation_ang=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_ang));}
  else if (strcmp(ucData, "$RC-GYR")==0) {enable_resolution_compensation_gyr=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_gyr));}
  else if (strcmp(ucData, "$CV-GYR")==0) {enable_resolution_compensation_gyr=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_gyr));}
  else if (strcmp(ucData, "$RC-MAG")==0) {enable_resolution_compensation_mag=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_mag));}
  else if (strcmp(ucData, "$CC-MAG")==0) {enable_resolution_compensation_mag=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_mag));}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             I2C REQUEST EVENTS
// ------------------------------------------------------------------------------------------------------------------------------

int loops_between_requests=0;
void requestEvent() {

  // Serial.println("[loops_between_requests] " + String(loops_between_requests));
                                                                                           
  // -----------------------------------------------
  //                                    ACCELERATION
  // -----------------------------------------------
  if (I2CLink.request_counter==0) {
    memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
    memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
    strcat(I2CLink.TMP_BUFFER0, "$A,");
    if (enable_resolution_compensation_acc==true) {
      // ---------------------------------------------
      // Acceleration X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_acc_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Acceleration Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_acc_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Acceleration Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_acc_z0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    else {
      // ---------------------------------------------
      // Acceleration X
      // ---------------------------------------------
      dtostrf(fAcc[0], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Acceleration X
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(fAcc[1], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Acceleration X
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(fAcc[2], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    // ---------------------------------------------
    // normalize stored highs/lows
    // ---------------------------------------------
    acc_x_high=fAcc[0];
    acc_x_low=fAcc[0];
    acc_y_high=fAcc[1];
    acc_y_low=fAcc[1];
    acc_z_high=fAcc[2];
    acc_z_low=fAcc[2];
  }

  // -----------------------------------------------
  //                                           ANGLE
  // -----------------------------------------------
  else if (I2CLink.request_counter==1) {
    memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
    memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
    strcat(I2CLink.TMP_BUFFER0, "$B,");
    if (enable_resolution_compensation_ang==true) {
      // ---------------------------------------------
      // Angle X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_ang_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Angle Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_ang_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Angle Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_ang_z0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    else {
      // ---------------------------------------------
      // Angle X
      // ---------------------------------------------
      dtostrf(fAngle[0], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Angle X
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(fAngle[1], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Angle X
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(fAngle[2], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    // ---------------------------------------------
    // normalize stored highs/lows
    // ---------------------------------------------
    ang_x_high=fAngle[0];
    ang_x_low=fAngle[0];
    ang_y_high=fAngle[1];
    ang_y_low=fAngle[1];
    ang_z_high=fAngle[2];
    ang_z_low=fAngle[2];
  }

  // -----------------------------------------------
  //                                            GYRO
  // -----------------------------------------------
  else if (I2CLink.request_counter==2) {
    memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
    memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
    strcat(I2CLink.TMP_BUFFER0, "$C,");
    if (enable_resolution_compensation_gyr==true) {
      // ---------------------------------------------
      // Gyro X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_gyr_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Gyro Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_gyr_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Gyro Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_gyr_z0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    else {
      // ---------------------------------------------
      // Gyro X
      // ---------------------------------------------
      dtostrf(fGyro[0], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Gyro Y
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(fGyro[1], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Gyro Z
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(fGyro[2], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    // ---------------------------------------------
    // normalize stored highs/lows
    // ---------------------------------------------
    gyr_x_high=fGyro[0];
    gyr_x_low=fGyro[0];
    gyr_y_high=fGyro[1];
    gyr_y_low=fGyro[1];
    gyr_z_high=fGyro[2];
    gyr_z_low=fGyro[2];
  }

  // -----------------------------------------------
  //                                  MAGNETIC FIELD
  // -----------------------------------------------
  else if (I2CLink.request_counter==3) {
    memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
    memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
    strcat(I2CLink.TMP_BUFFER0, "$D,");
    if (enable_resolution_compensation_mag==true) {
      // ---------------------------------------------
      // Mag X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_mag_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Mag Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_mag_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Mag Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(tmp_mag_z0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    else {
      // ---------------------------------------------
      // Mag X
      // ---------------------------------------------
      dtostrf(sReg[HX], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Mag Y
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(sReg[HY], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Mag Z
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      dtostrf(sReg[HZ], 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
    }
    // ---------------------------------------------
    // normalize stored highs/lows
    // ---------------------------------------------
    mag_x_high=sReg[HX];
    mag_x_low=sReg[HX];
    mag_y_high=sReg[HY];
    mag_y_low=sReg[HY];
    mag_z_high=sReg[HZ];
    mag_z_low=sReg[HZ];
  }

  // -----------------------------------------------
  // SEND
  // -----------------------------------------------
  // Serial.println("[sending] " + String(I2CLink.TMP_BUFFER0));
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i]=(byte)I2CLink.TMP_BUFFER0[i];}
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          SETUP
// ------------------------------------------------------------------------------------------------------------------------------

void setup() {

  // -----------------------------------------------
  // serial
  // -----------------------------------------------
  Serial.begin(115200);

  // ------------------------------------------------------------
  // Indicators
  // ------------------------------------------------------------
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // -----------------------------------------------
  // WT901
  // -----------------------------------------------
	Serial.print("[setup] setting up WT901 sensor");
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
	AutoScanSensor();

  // -----------------------------------------------
  // initialize I2C communications as slave
  // -----------------------------------------------
  Wire.begin(SLAVE_ADDR);

  // -----------------------------------------------
  // function to run when data requested from master
  // -----------------------------------------------
  Wire.onRequest(requestEvent);

  // -----------------------------------------------
  // function to run when data received from master
  // -----------------------------------------------
  Wire.onReceive(receiveEvent);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     INDICATORS
// ------------------------------------------------------------------------------------------------------------------------------

void LEDOffExcept(int start, int end, int n) {
  for (int i=start; i<=end; i++) {
    if (i!=n) {leds[i] = CRGB::Black; FastLED.show();}
  }
}

void UpdateLEDs() {
  // roll
  if (ang_x_percent>=0 && ang_x_percent<12.5) {LEDOffExcept(0, 8, 0); leds[0] = CRGB::Red; FastLED.show();}
  if (ang_x_percent>=12.5 && ang_x_percent<25) {LEDOffExcept(0, 8, 1); leds[1] = CRGB::Red; FastLED.show();}
  if (ang_x_percent>=25 && ang_x_percent<37.5) {LEDOffExcept(0, 8, 2); leds[2] = CRGB::Red; FastLED.show();}
  if (ang_x_percent>=37.5 && ang_x_percent<50) {LEDOffExcept(0, 8, 3); leds[3] = CRGB::Red; FastLED.show();}
  if (ang_x_percent==50) {LEDOffExcept(0, 8, 4); leds[4] = CRGB::Red; FastLED.show();}
  if (ang_x_percent>50 && ang_x_percent<=62.5) {LEDOffExcept(0, 8, 5); leds[5] = CRGB::Red; FastLED.show();}
  if (ang_x_percent>62.5 && ang_x_percent<=75) {LEDOffExcept(0, 8, 6); leds[6] = CRGB::Red; FastLED.show();}
  if (ang_x_percent>75 && ang_x_percent<=87.5) {LEDOffExcept(0, 8, 7); leds[7] = CRGB::Red; FastLED.show();}
  if (ang_x_percent>87.5 && ang_x_percent<=100) {LEDOffExcept(0, 8, 8); leds[8] = CRGB::Red; FastLED.show();}
  // pitch
  if (ang_y_percent>=0 && ang_y_percent<12.5) {LEDOffExcept(9, 17, 9); leds[9] = CRGB::Green; FastLED.show();}
  if (ang_y_percent>=12.5 && ang_y_percent<25) {LEDOffExcept(9, 17, 10); leds[10] = CRGB::Green; FastLED.show();}
  if (ang_y_percent>=25 && ang_y_percent<37.5) {LEDOffExcept(9, 17, 11); leds[11] = CRGB::Green; FastLED.show();}
  if (ang_y_percent>=37.5 && ang_y_percent<50) {LEDOffExcept(9, 17, 12); leds[12] = CRGB::Green; FastLED.show();}
  if (ang_y_percent==50) {LEDOffExcept(9, 17, 13); leds[13] = CRGB::Green; FastLED.show();}
  if (ang_y_percent>50 && ang_y_percent<=62.5) {LEDOffExcept(9, 17, 14); leds[14] = CRGB::Green; FastLED.show();}
  if (ang_y_percent>62.5 && ang_y_percent<=75) {LEDOffExcept(9, 17, 15); leds[15] = CRGB::Green; FastLED.show();}
  if (ang_y_percent>75 && ang_y_percent<=87.5) {LEDOffExcept(9, 17, 16); leds[16] = CRGB::Green; FastLED.show();}
  if (ang_y_percent>87.5 && ang_y_percent<=100) {LEDOffExcept(9, 17, 17); leds[17] = CRGB::Green; FastLED.show();}
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                           LOOP
// ------------------------------------------------------------------------------------------------------------------------------

void loop() {
  // for (int i=0; i<NUM_LEDS; i++) {leds[i] = CRGB::White; FastLED.show(); delay(30); leds[i] = CRGB::Black; FastLED.show(); delay(30);}
  UpdateLEDs();

  loops_between_requests++;

  // -----------------------------------------------
  // read WT901
  // -----------------------------------------------
  while (Serial1.available()) {WitSerialDataIn(Serial1.read());}

  // -----------------------------------------------
  // read commands
  // -----------------------------------------------
  while (Serial.available()) {
    memset(ucData, 0, sizeof(ucData));
    Serial.readBytesUntil('\n', ucData, sizeof(ucData));
  }

  // -----------------------------------------------
  // process commands
  // -----------------------------------------------
  CmdProcess();

  // -----------------------------------------------
  // update WT901 data
  // -----------------------------------------------
  if (s_cDataUpdate)
  {
    // ---------------------------------------------
    // update values
    // ---------------------------------------------
    for(int i=0; i < 3; i++)
    {
      fAcc[i]=sReg[AX+i] / 32768.0f * 16.0f;
      fGyro[i]=sReg[GX+i] / 32768.0f * 2000.0f;
      fAngle[i]=sReg[Roll+i] / 32768.0f * 180.0f;
    }
    // ---------------------------------------------
    // process acceleration
    // ---------------------------------------------
    if (s_cDataUpdate & ACC_UPDATE)
    {
      s_cDataUpdate &= ~ACC_UPDATE;
      // -----------------------
      // resolution compensation
      // -----------------------
      if (enable_resolution_compensation_acc==true) {
        if (fAcc[0]>acc_x_high) {acc_x_high=fAcc[0];}
        if (fAcc[0]<acc_x_low) {acc_x_low=fAcc[0];}
        if (fAcc[1]>acc_y_high) {acc_y_high=fAcc[1];}
        if (fAcc[1]<acc_y_low) {acc_y_low=fAcc[1];}
        if (fAcc[2]>acc_z_high) {acc_z_high=fAcc[2];}
        if (fAcc[2]<acc_z_low) {acc_z_low=fAcc[2];}
        tmp_acc_x0=acc_x_high;
        if (abs(acc_x_low) > acc_x_high) {tmp_acc_x0=acc_x_low;}
        tmp_acc_y0=acc_y_high;
        if (abs(acc_y_low) > acc_y_high) {tmp_acc_y0=acc_y_low;}
        tmp_acc_z0=acc_z_high;
        if (abs(acc_z_low) > acc_z_high) {tmp_acc_z0=acc_z_low;}
      }
      // -----------------------
      // current value
      // -----------------------
      if (enable_serial_output_content==true) {
        Serial.print("$ACC,");
        Serial.print(fAcc[0], 3);
        Serial.print(",");
        Serial.print(fAcc[1], 3);
        Serial.print(",");
        Serial.print(fAcc[2], 3);
        Serial.print("\r\n");
      }
    }
    // ---------------------------------------------
    // process angle
    // ---------------------------------------------
    if (s_cDataUpdate & ANGLE_UPDATE)
    {
      // -----------------------
      // update angle
      // -----------------------
      s_cDataUpdate &= ~ANGLE_UPDATE;
      mapped_ang_x = map(fAngle[0], -90.00, 90, 0, 100);
      mapped_ang_y = map(fAngle[1], -90.00, 90, 0, 100);
      // -----------------------
      // resolution compensation
      // -----------------------
      if (enable_resolution_compensation_gyr==true) {
        if (fAngle[0]>ang_x_high) {ang_x_high=fAngle[0];}
        if (fAngle[0]<ang_x_low) {ang_x_low=fAngle[0];}
        if (fAngle[1]>ang_y_high) {ang_y_high=fAngle[1];}
        if (fAngle[1]<ang_y_low) {ang_y_low=fAngle[1];}
        if (fAngle[2]>ang_z_high) {ang_z_high=fAngle[2];}
        if (fAngle[2]<ang_z_low) {ang_z_low=fAngle[2];}
        tmp_ang_x0=ang_x_high;
        if (abs(ang_x_low) > ang_x_high) {tmp_ang_x0=ang_x_low;}
        tmp_ang_y0=ang_y_high;
        if (abs(ang_y_low) > ang_y_high) {tmp_ang_y0=ang_y_low;}
        tmp_ang_z0=ang_z_high;
        if (abs(ang_z_low) > ang_z_high) {tmp_ang_z0=ang_z_low;}
        mapped_ang_x = map(tmp_ang_x0, -90.00, 90, 0, 100);
        mapped_ang_y = map(tmp_ang_y0, -90.00, 90, 0, 100);
      }
      // -----------------------
      // create percentages
      // -----------------------
      ang_x_percent = 100 * ((float)mapped_ang_x / 100);
      ang_y_percent = 100 * ((float)mapped_ang_y / 100);
      // -----------------------
      // serial
      // -----------------------
      if (enable_serial_output_content==true) {
        Serial.print("$ANG,");
        Serial.print(fAngle[0], 3);
        Serial.print(",");
        Serial.print(fAngle[1], 3);
        Serial.print(",");
        Serial.print(fAngle[2], 3);
        Serial.print("\r\n");
      }
    }
    // ---------------------------------------------
    // process gyro
    // ---------------------------------------------
    if (s_cDataUpdate & GYRO_UPDATE)
    {
      s_cDataUpdate &= ~GYRO_UPDATE;
      if (enable_resolution_compensation_ang==true) {
        if (fGyro[0]>gyr_x_high) {gyr_x_high=fGyro[0];}
        if (fGyro[0]<gyr_x_low) {gyr_x_low=fGyro[0];}
        if (fGyro[1]>gyr_y_high) {gyr_y_high=fGyro[1];}
        if (fGyro[1]<gyr_y_low) {gyr_y_low=fGyro[1];}
        if (fGyro[2]>gyr_z_high) {gyr_z_high=fGyro[2];}
        if (fGyro[2]<gyr_z_low) {gyr_z_low=fGyro[2];}
        tmp_gyr_x0=gyr_x_high;
        if (abs(gyr_x_low) > gyr_x_high) {tmp_gyr_x0=gyr_x_low;}
        tmp_gyr_y0=gyr_y_high;
        if (abs(gyr_y_low) > gyr_y_high) {tmp_gyr_y0=gyr_y_low;}
        tmp_gyr_z0=gyr_z_high;
        if (abs(gyr_z_low) > gyr_z_high) {tmp_gyr_z0=gyr_z_low;}
      }
      if (enable_serial_output_content==true) {
        Serial.print("$GYR,");
        Serial.print(fGyro[0], 1);
        Serial.print(",");
        Serial.print(fGyro[1], 1);
        Serial.print(",");
        Serial.print(fGyro[2], 1);
        Serial.print("\r\n");
      }
    }
    // ---------------------------------------------
    // process magnetic field
    // ---------------------------------------------
    if (s_cDataUpdate & MAG_UPDATE)
    {
      s_cDataUpdate &= ~MAG_UPDATE;
      if (enable_resolution_compensation_mag==true) {
        if (sReg[HX]>mag_x_high) {mag_x_high=sReg[HX];}
        if (sReg[HX]<mag_x_low) {mag_x_low=sReg[HX];}
        if (sReg[HY]>mag_y_high) {mag_y_high=sReg[HY];}
        if (sReg[HY]<mag_y_low) {mag_y_low=sReg[HY];}
        if (sReg[HZ]>mag_z_high) {mag_z_high=sReg[HZ];}
        if (sReg[HZ]<mag_z_low) {mag_z_low=sReg[HZ];}
        tmp_mag_x0=mag_x_high;
        if (abs(mag_x_low) > mag_x_high) {tmp_mag_x0=mag_x_low;}
        tmp_mag_y0=mag_y_high;
        if (abs(mag_y_low) > mag_y_high) {tmp_mag_y0=mag_y_low;}
        tmp_mag_z0=mag_z_high;
        if (abs(mag_z_low) > mag_z_high) {tmp_mag_z0=mag_z_low;}
      }
      if (enable_serial_output_content==true) {
        Serial.print("$MAG,");
        Serial.print(sReg[HX]);
        Serial.print(",");
        Serial.print(sReg[HY]);
        Serial.print(",");
        Serial.print(sReg[HZ]);
        Serial.print("\r\n");
      }
    }
    s_cDataUpdate=0;
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       ShowHelp
// ------------------------------------------------------------------------------------------------------------------------------

static void ShowHelp(void)
{
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                               HELP                                                ");
  Serial.println("UART SEND: a        Acceleration calibration.");
  Serial.println("UART SEND: m        Enter Magnetic field calibration. To end calibration UART SEND: e");
  Serial.println("UART SEND: U        Bandwidth increase.");
  Serial.println("UART SEND: u        Bandwidth reduction.");
  Serial.println("UART SEND: B        Baud rate increased to 115200.");
  Serial.println("UART SEND: b        Baud rate reduction to 9600.");
  Serial.println("UART SEND: R        The return rate increases to 10Hz.");
  Serial.println("UART SEND: r        The return rate reduction to 1Hz.");
  Serial.println("UART SEND: C        Basic return content: acceleration, angular velocity, angle, magnetic field.");
  Serial.println("UART SEND: c        Return content: acceleration.");
  Serial.println("UART SEND: O        Enable return content.");
  Serial.println("UART SEND: o        Disable return content.");
  Serial.println("UART SEND: rc-acc   Enable resolution compensation (acceleration).");
  Serial.println("UART SEND: cv-acc   Disable resolution compensation content (acceleration).");
  Serial.println("UART SEND: rc-ang   Enable resolution compensation (angle).");
  Serial.println("UART SEND: cv-ang   Disable resolution compensation (angle).");
  Serial.println("UART SEND: rc-gyr   Enable resolution compensation (gyro).");
  Serial.println("UART SEND: cv-gyr   Disable resolution compensation (gyro).");
  Serial.println("UART SEND: rc-mag   Enable resolution compensation (magnetic field).");
  Serial.println("UART SEND: cv-mag   Disable resolution compensation (magnetic field).");
  Serial.println("UART SEND: h        Display this help messgae.");
  Serial.println("---------------------------------------------------------------------------------------------------");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     CmdProcess
// ------------------------------------------------------------------------------------------------------------------------------

static void CmdProcess(void) {
  if (strcmp(ucData, "h\r")==0) {ShowHelp();}

  else if (strcmp(ucData, "a\r")==0) {if (WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");}
  else if (strcmp(ucData, "m\r")==0) {if (WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");}
  else if (strcmp(ucData, "e\r")==0) {if (WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");}
  else if (strcmp(ucData, "u\r")==0) {if (WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");}
  else if (strcmp(ucData, "U\r")==0) {if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");}
  else if (strcmp(ucData, "B\r")==0) {
    if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
    else {Serial1.begin(c_uiBaud[WIT_BAUD_115200]); Serial.print(" 115200 Baud rate modified successfully\r\n");}
  }
  else if (strcmp(ucData, "b\r")==0) {
    if (WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
    else {Serial1.begin(c_uiBaud[WIT_BAUD_9600]); Serial.print(" 9600 Baud rate modified successfully\r\n");}
  }
  else if (strcmp(ucData, "r\r")==0) {
    if (WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)  Serial.print("\r\nSet Baud Error\r\n");
    else Serial.print("\r\nSet Baud Success\r\n");
  }
  else if (strcmp(ucData, "R\r")==0) {
    if (WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
    else Serial.print("\r\nSet Baud Success\r\n");
  }
  else if (strcmp(ucData, "C\r")==0) {if (WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");}
  else if (strcmp(ucData, "c\r")==0) {if (WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");}
  else if (strcmp(ucData, "O\r")==0) {enable_serial_output_content=true;}
  else if (strcmp(ucData, "o\r")==0) {enable_serial_output_content=false;}
  else if (strcmp(ucData, "rc-acc\r")==0) {enable_resolution_compensation_acc=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_acc));}
  else if (strcmp(ucData, "cv-acc\r")==0) {enable_resolution_compensation_acc=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_acc));}
  else if (strcmp(ucData, "rc-ang\r")==0) {enable_resolution_compensation_ang=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_ang));}
  else if (strcmp(ucData, "cv-ang\r")==0) {enable_resolution_compensation_ang=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_ang));}
  else if (strcmp(ucData, "rc-gyr\r")==0) {enable_resolution_compensation_gyr=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_gyr));}
  else if (strcmp(ucData, "cv-gyr\r")==0) {enable_resolution_compensation_gyr=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_gyr));}
  else if (strcmp(ucData, "rc-mag\r")==0) {enable_resolution_compensation_mag=true; Serial.println("[rc-acc] " + String(enable_resolution_compensation_mag));}
  else if (strcmp(ucData, "cv-mag\r")==0) {enable_resolution_compensation_mag=false; Serial.println("[rc-acc] " + String(enable_resolution_compensation_mag));}
  memset(ucData, 0, sizeof(ucData));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 SensorUartSend
// ------------------------------------------------------------------------------------------------------------------------------

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        Delayms
// ------------------------------------------------------------------------------------------------------------------------------

static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SensorDataUpdata
// ------------------------------------------------------------------------------------------------------------------------------

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {

  for(unsigned int i=0; i<uiRegNum; i++) {
    
    switch(uiReg)
    {
      case AZ: s_cDataUpdate |= ACC_UPDATE; break;
      case GZ: s_cDataUpdate |= GYRO_UPDATE; break;
      case HZ: s_cDataUpdate |= MAG_UPDATE; break;
      case Yaw: s_cDataUpdate |= ANGLE_UPDATE; break;
      default: s_cDataUpdate |= READ_UPDATE; break;
    }
    uiReg++;
  }
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 AutoScanSensor
// ------------------------------------------------------------------------------------------------------------------------------

static void AutoScanSensor(void)
{
	unsigned int i, iRetry;
	
	for(i=0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
	{
		Serial1.begin(c_uiBaud[i]);
    Serial1.flush();
		iRetry=2;
		s_cDataUpdate=0;
		do
		{
			WitReadReg(AX, 3);
			delay(200);
      while (Serial1.available())
      {
        WitSerialDataIn(Serial1.read());
      }
			if (s_cDataUpdate != 0)
			{
				Serial.print(c_uiBaud[i]);
				Serial.print(" baud find sensor\r\n\r\n");
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	Serial.print("can not find sensor\r\n");
	Serial.print("please check your connection\r\n");
}
