/*

WT901 IIC Module written by Benjamin Jack Cullen and adapted from official example code written by Witmotion.

The WT901 is a IMU sensor device, detecting acceleration, angular
velocity, angle as well as magnetic filed.

7 segment displays have been chosen over small OLED displays for brightness and ability to remain on without damage.

WT901CTTL       MEGA 2560
    VCC  <--->  5V/3.3V
    TX   <--->  19(TX1)
    RX   <--->  18(RX1)
    GND  <--->  GND

Display Angle X
TM1637 (0)       MEGA 2560
    VCC   <--->  5V
    CLK   <--->  A0
    DIO   <--->  53
    GND   <--->  GND

Display Angle Y
TM1637 (1)       MEGA 2560
    VCC   <--->  5V
    CLK   <--->  A1
    DIO   <--->  52
    GND   <--->  GND

Display Angle Z
TM1637 (2)       MEGA 2560
    VCC   <--->  5V
    CLK   <--->  A2
    DIO   <--->  51
    GND   <--->  GND

Todo:
Display Acceleration X,Y,Z (pending deliveries).
Display Magnetic Field X,Y,Z (pending deliveries). requies 6 digit 7 segment displays.
Setup this module as IIC slave.

*/

// ----------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "./REG.h"
#include "./wit_c_sdk.h"
// #include <FastLED.h>
#include <AceTMI.h> // SimpleTmi1637Interface
#include <AceSegment.h> // Tm1637Module

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        WT901
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 
void CopeCmdData(unsigned char ucData);
static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};
float fAcc[3], fGyro[3], fAngle[3];

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       TM1637
using ace_tmi::SimpleTmi1637Interface;
using ace_segment::Tm1637Module;
using TmiInterface = SimpleTmi1637Interface;
// Many TM1637 LED modules contain 10 nF capacitors on their DIO and CLK lines
// which are unreasonably high. This forces a 100 microsecond delay between
// bit transitions. If you remove those capacitors, you can set this as low as
// 1-5 micros.
const uint8_t DELAY_MICROS = 100;
// number of digits supported by the seven segment display(s)
const uint8_t NUM_DIGITS = 4;
// display 0
const uint8_t CLK_PIN_0 = A0;
const uint8_t DIO_PIN_0 = 53;
TmiInterface tmiInterface_0(DIO_PIN_0, CLK_PIN_0, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_0(tmiInterface_0);
// display 1
const uint8_t CLK_PIN_1 = A1;
const uint8_t DIO_PIN_1 = 52;
TmiInterface tmiInterface_1(DIO_PIN_1, CLK_PIN_1, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_1(tmiInterface_1);
// display 2
const uint8_t CLK_PIN_2 = A2;
const uint8_t DIO_PIN_2 = 51;
TmiInterface tmiInterface_2(DIO_PIN_2, CLK_PIN_2, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_2(tmiInterface_2);
// display 3
const uint8_t CLK_PIN_3 = A3;
const uint8_t DIO_PIN_3 = 50;
TmiInterface tmiInterface_3(DIO_PIN_3, CLK_PIN_3, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_3(tmiInterface_3);
// display 4
const uint8_t CLK_PIN_4 = A4;
const uint8_t DIO_PIN_4 = 49;
TmiInterface tmiInterface_4(DIO_PIN_4, CLK_PIN_4, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_4(tmiInterface_4);
// display 5
const uint8_t CLK_PIN_5 = A5;
const uint8_t DIO_PIN_5 = 48;
TmiInterface tmiInterface_5(DIO_PIN_5, CLK_PIN_5, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_5(tmiInterface_5);
// display 6
const uint8_t CLK_PIN_6 = A6;
const uint8_t DIO_PIN_6 = 47;
TmiInterface tmiInterface_6(DIO_PIN_6, CLK_PIN_6, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_6(tmiInterface_6);
// display 7
const uint8_t CLK_PIN_7 = A7;
const uint8_t DIO_PIN_7 = 46;
TmiInterface tmiInterface_7(DIO_PIN_7, CLK_PIN_7, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_7(tmiInterface_7);
// display 8
const uint8_t CLK_PIN_8 = A8;
const uint8_t DIO_PIN_8 = 45;
TmiInterface tmiInterface_8(DIO_PIN_8, CLK_PIN_8, DELAY_MICROS);
Tm1637Module<TmiInterface, NUM_DIGITS> ledModule_8(tmiInterface_8);
// add/remove patterns as required
const uint8_t NUM_PATTERNS = 12;
const uint8_t PATTERNS[NUM_PATTERNS] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b00000000, // clear 10
  0b01000000, // - 11
};
int dlen=0;

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        SETUP
void setup() {
  Serial.begin(115200);
  // ------------------------------------------------------------------------------------------------------
  //                                                                                                  WT901
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
	Serial.print("\r\n********************** wit-motion normal example  ************************\r\n");
	AutoScanSensor();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 0
  tmiInterface_0.begin();
  ledModule_0.begin();
  ledModule_0.setPatternAt(0, PATTERNS[11]);
  ledModule_0.setPatternAt(1, PATTERNS[11]);
  ledModule_0.setPatternAt(2, PATTERNS[11]);
  ledModule_0.setPatternAt(3, PATTERNS[11]);
  ledModule_0.setBrightness(2);
  ledModule_0.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 1
  tmiInterface_1.begin();
  ledModule_1.begin();
  ledModule_1.setPatternAt(0, PATTERNS[11]);
  ledModule_1.setPatternAt(1, PATTERNS[11]);
  ledModule_1.setPatternAt(2, PATTERNS[11]);
  ledModule_1.setPatternAt(3, PATTERNS[11]);
  ledModule_1.setBrightness(2);
  ledModule_1.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 2
  tmiInterface_2.begin();
  ledModule_2.begin();
  ledModule_2.setPatternAt(0, PATTERNS[11]);
  ledModule_2.setPatternAt(1, PATTERNS[11]);
  ledModule_2.setPatternAt(2, PATTERNS[11]);
  ledModule_2.setPatternAt(3, PATTERNS[11]);
  ledModule_2.setBrightness(2);
  ledModule_2.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 3
  tmiInterface_3.begin();
  ledModule_3.begin();
  ledModule_3.setPatternAt(0, PATTERNS[11]);
  ledModule_3.setPatternAt(1, PATTERNS[11]);
  ledModule_3.setPatternAt(2, PATTERNS[11]);
  ledModule_3.setPatternAt(3, PATTERNS[11]);
  ledModule_3.setBrightness(2);
  ledModule_3.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 4
  tmiInterface_4.begin();
  ledModule_4.begin();
  ledModule_4.setPatternAt(0, PATTERNS[11]);
  ledModule_4.setPatternAt(1, PATTERNS[11]);
  ledModule_4.setPatternAt(2, PATTERNS[11]);
  ledModule_4.setPatternAt(3, PATTERNS[11]);
  ledModule_4.setBrightness(2);
  ledModule_4.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 5
  tmiInterface_5.begin();
  ledModule_5.begin();
  ledModule_5.setPatternAt(0, PATTERNS[11]);
  ledModule_5.setPatternAt(1, PATTERNS[11]);
  ledModule_5.setPatternAt(2, PATTERNS[11]);
  ledModule_5.setPatternAt(3, PATTERNS[11]);
  ledModule_5.setBrightness(2);
  ledModule_5.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 6
  tmiInterface_6.begin();
  ledModule_6.begin();
  ledModule_6.setPatternAt(0, PATTERNS[11]);
  ledModule_6.setPatternAt(1, PATTERNS[11]);
  ledModule_6.setPatternAt(2, PATTERNS[11]);
  ledModule_6.setPatternAt(3, PATTERNS[11]);
  ledModule_6.setBrightness(2);
  ledModule_6.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 7
  tmiInterface_7.begin();
  ledModule_7.begin();
  ledModule_7.setPatternAt(0, PATTERNS[11]);
  ledModule_7.setPatternAt(1, PATTERNS[11]);
  ledModule_7.setPatternAt(2, PATTERNS[11]);
  ledModule_7.setPatternAt(3, PATTERNS[11]);
  ledModule_7.setBrightness(2);
  ledModule_7.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               TM1637 8
  tmiInterface_8.begin();
  ledModule_8.begin();
  ledModule_8.setPatternAt(0, PATTERNS[11]);
  ledModule_8.setPatternAt(1, PATTERNS[11]);
  ledModule_8.setPatternAt(2, PATTERNS[11]);
  ledModule_8.setPatternAt(3, PATTERNS[11]);
  ledModule_8.setBrightness(2);
  ledModule_8.flush();
  // ------------------------------------------------------------------------------------------------------
  //                                                                                               COMPLETE
  delay(2000);
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                 COUNT DIGITS
void digitLen(int digits) {
  dlen=0;
  for (int i=0; i<4; i++) {if (isDigit(String(digits).c_str()[i])==true) {dlen++;}}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                              DISPLAY ANGLE X
void DisplayaAngleX(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_0.setPatternAt(0, PATTERNS[10]);
    ledModule_0.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_0.setPatternAt(2, PATTERNS[11]);
      ledModule_0.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_0.setPatternAt(2, PATTERNS[10]);
      ledModule_0.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_0.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_0.setPatternAt(1, PATTERNS[11]);
      ledModule_0.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_0.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_0.setPatternAt(1, PATTERNS[10]);
      ledModule_0.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_0.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_0.setPatternAt(0, PATTERNS[11]);
      ledModule_0.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_0.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_0.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_0.setPatternAt(0, PATTERNS[10]);
      ledModule_0.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_0.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_0.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_0.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                              DISPLAY ANGLE Y
void DisplayaAngleY(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_1.setPatternAt(0, PATTERNS[10]);
    ledModule_1.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_1.setPatternAt(2, PATTERNS[11]);
      ledModule_1.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_1.setPatternAt(2, PATTERNS[10]);
      ledModule_1.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_1.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_1.setPatternAt(1, PATTERNS[11]);
      ledModule_1.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_1.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_1.setPatternAt(1, PATTERNS[10]);
      ledModule_1.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_1.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_1.setPatternAt(0, PATTERNS[11]);
      ledModule_1.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_1.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_1.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_1.setPatternAt(0, PATTERNS[10]);
      ledModule_1.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_1.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_1.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_1.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                              DISPLAY ANGLE Z
void DisplayaAngleZ(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_2.setPatternAt(0, PATTERNS[10]);
    ledModule_2.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_2.setPatternAt(2, PATTERNS[11]);
      ledModule_2.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_2.setPatternAt(2, PATTERNS[10]);
      ledModule_2.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_2.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_2.setPatternAt(1, PATTERNS[11]);
      ledModule_2.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_2.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_2.setPatternAt(1, PATTERNS[10]);
      ledModule_2.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_2.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_2.setPatternAt(0, PATTERNS[11]);
      ledModule_2.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_2.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_2.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_2.setPatternAt(0, PATTERNS[10]);
      ledModule_2.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_2.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_2.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_2.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       DISPLAY ACCELERATION X
void DisplayaAccellerationX(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_3.setPatternAt(0, PATTERNS[10]);
    ledModule_3.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_3.setPatternAt(2, PATTERNS[11]);
      ledModule_3.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_3.setPatternAt(2, PATTERNS[10]);
      ledModule_3.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_3.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_3.setPatternAt(1, PATTERNS[11]);
      ledModule_3.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_3.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_3.setPatternAt(1, PATTERNS[10]);
      ledModule_3.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_3.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_3.setPatternAt(0, PATTERNS[11]);
      ledModule_3.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_3.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_3.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_3.setPatternAt(0, PATTERNS[10]);
      ledModule_3.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_3.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_3.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_3.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       DISPLAY ACCELERATION Y
void DisplayaAccellerationY(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_4.setPatternAt(0, PATTERNS[10]);
    ledModule_4.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_4.setPatternAt(2, PATTERNS[11]);
      ledModule_4.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_4.setPatternAt(2, PATTERNS[10]);
      ledModule_4.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_4.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_4.setPatternAt(1, PATTERNS[11]);
      ledModule_4.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_4.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_4.setPatternAt(1, PATTERNS[10]);
      ledModule_4.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_4.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_4.setPatternAt(0, PATTERNS[11]);
      ledModule_4.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_4.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_4.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_4.setPatternAt(0, PATTERNS[10]);
      ledModule_4.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_4.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_4.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_4.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                       DISPLAY ACCELERATION Z
void DisplayaAccellerationZ(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_5.setPatternAt(0, PATTERNS[10]);
    ledModule_5.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_5.setPatternAt(2, PATTERNS[11]);
      ledModule_5.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_5.setPatternAt(2, PATTERNS[10]);
      ledModule_5.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_5.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_5.setPatternAt(1, PATTERNS[11]);
      ledModule_5.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_5.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_5.setPatternAt(1, PATTERNS[10]);
      ledModule_5.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_5.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_5.setPatternAt(0, PATTERNS[11]);
      ledModule_5.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_5.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_5.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_5.setPatternAt(0, PATTERNS[10]);
      ledModule_5.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_5.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_5.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_5.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                     DISPLAY MAGNETIC FIELD X
void DisplayaMagneticFieldX(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_6.setPatternAt(0, PATTERNS[10]);
    ledModule_6.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_6.setPatternAt(2, PATTERNS[11]);
      ledModule_6.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_6.setPatternAt(2, PATTERNS[10]);
      ledModule_6.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_6.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_6.setPatternAt(1, PATTERNS[11]);
      ledModule_6.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_6.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_6.setPatternAt(1, PATTERNS[10]);
      ledModule_6.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_6.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_6.setPatternAt(0, PATTERNS[11]);
      ledModule_6.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_6.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_6.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_6.setPatternAt(0, PATTERNS[10]);
      ledModule_6.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_6.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_6.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_6.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                     DISPLAY MAGNETIC FIELD Y
void DisplayaMagneticFieldY(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_7.setPatternAt(0, PATTERNS[10]);
    ledModule_7.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_7.setPatternAt(2, PATTERNS[11]);
      ledModule_7.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_7.setPatternAt(2, PATTERNS[10]);
      ledModule_7.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_7.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_7.setPatternAt(1, PATTERNS[11]);
      ledModule_7.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_7.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_7.setPatternAt(1, PATTERNS[10]);
      ledModule_7.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_7.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_7.setPatternAt(0, PATTERNS[11]);
      ledModule_7.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_7.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_7.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_7.setPatternAt(0, PATTERNS[10]);
      ledModule_7.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_7.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_7.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_7.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                     DISPLAY MAGNETIC FIELD Z
void DisplayaMagneticFieldZ(signed int value) {
  digitLen(value);
  if (dlen==1) {
    ledModule_8.setPatternAt(0, PATTERNS[10]);
    ledModule_8.setPatternAt(1, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_8.setPatternAt(2, PATTERNS[11]);
      ledModule_8.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
    else {
      ledModule_8.setPatternAt(2, PATTERNS[10]);
      ledModule_8.setPatternAt(3, PATTERNS[atoi(String(String(value)[0]).c_str())]);
    }
  }
  if (dlen==2) {
    ledModule_8.setPatternAt(0, PATTERNS[10]);
    if (value < 0 ) {
      ledModule_5.setPatternAt(1, PATTERNS[11]);
      ledModule_8.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_8.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
    else {
      ledModule_8.setPatternAt(1, PATTERNS[10]);
      ledModule_8.setPatternAt(2, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_8.setPatternAt(3, PATTERNS[atoi(String(String(value)[1]).c_str())]);
    }
  }
  if (dlen==3) {
    if (value < 0 ) {
      ledModule_8.setPatternAt(0, PATTERNS[11]);
      ledModule_8.setPatternAt(1, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_8.setPatternAt(2, PATTERNS[atoi(String(String(value)[2]).c_str())]);
      ledModule_8.setPatternAt(3, PATTERNS[atoi(String(String(value)[3]).c_str())]);
    }
    else {
      ledModule_8.setPatternAt(0, PATTERNS[10]);
      ledModule_8.setPatternAt(1, PATTERNS[atoi(String(String(value)[0]).c_str())]);
      ledModule_8.setPatternAt(2, PATTERNS[atoi(String(String(value)[1]).c_str())]);
      ledModule_8.setPatternAt(3, PATTERNS[atoi(String(String(value)[2]).c_str())]);
    }
  }
  ledModule_8.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                         LOOP
void loop() {
  DisplayaAngleX((int)fAngle[0]);
  DisplayaAngleY((int)fAngle[1]);
  DisplayaAngleZ((int)fAngle[2]);
  DisplayaAccellerationX((int)fAcc[0]);
  DisplayaAccellerationY((int)fAcc[1]);
  DisplayaAccellerationZ((int)fAcc[2]);
  DisplayaMagneticFieldX((int)sReg[HX]);
  DisplayaMagneticFieldY((int)sReg[HY]);
  DisplayaMagneticFieldZ((int)sReg[HZ]);

  while (Serial1.available())
  {
    WitSerialDataIn(Serial1.read());
  }
  while (Serial.available()) 
  {
    CopeCmdData(Serial.read());
  }
  CmdProcess();
  if(s_cDataUpdate)
  {
    for(int i=0; i < 3; i++)
    {
      fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
    }
    if(s_cDataUpdate & ACC_UPDATE)
    {
      Serial.print("acc:");
      Serial.print(fAcc[0], 3);
      Serial.print(" ");
      Serial.print(fAcc[1], 3);
      Serial.print(" ");
      Serial.print(fAcc[2], 3);
      Serial.print("\r\n");
      s_cDataUpdate &= ~ACC_UPDATE;
    }
    if(s_cDataUpdate & GYRO_UPDATE)
    {
      Serial.print("gyro:");
      Serial.print(fGyro[0], 1);
      Serial.print(" ");
      Serial.print(fGyro[1], 1);
      Serial.print(" ");
      Serial.print(fGyro[2], 1);
      Serial.print("\r\n");
      s_cDataUpdate &= ~GYRO_UPDATE;
    }
    if(s_cDataUpdate & ANGLE_UPDATE)
    {
      Serial.print("angle:");
      Serial.print(fAngle[0], 3);
      Serial.print(" ");
      Serial.print(fAngle[1], 3);
      Serial.print(" ");
      Serial.print(fAngle[2], 3);
      Serial.print("\r\n");
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    if(s_cDataUpdate & MAG_UPDATE)
    {
      Serial.print("mag:");
      Serial.print(sReg[HX]);
      Serial.print(" ");
      Serial.print(sReg[HY]);
      Serial.print(" ");
      Serial.print(sReg[HZ]);
      Serial.print("\r\n");
      s_cDataUpdate &= ~MAG_UPDATE;
    }
    s_cDataUpdate = 0;
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                  CopeCmdData
void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     ShowHelp
static void ShowHelp(void)
{
	Serial.print("\r\n************************	 WIT_SDK_DEMO	************************");
	Serial.print("\r\n************************          HELP           ************************\r\n");
	Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	Serial.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  Serial.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  Serial.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  Serial.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
	Serial.print("******************************************************************************\r\n");
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                   CmdProcess
static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	if(WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	if(WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	if(WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial1.begin(c_uiBaud[WIT_BAUD_115200]);
                Serial.print(" 115200 Baud rate modified successfully\r\n");
              }
			break;
		case 'b':	if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial1.begin(c_uiBaud[WIT_BAUD_9600]); 
                Serial.print(" 9600 Baud rate modified successfully\r\n");
              }
			break;
		case 'r': if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)  Serial.print("\r\nSet Baud Error\r\n");
			        else Serial.print("\r\nSet Baud Success\r\n");
			break;
		case 'R':	if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else Serial.print("\r\nSet Baud Success\r\n");
			break;
    case 'C': if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'c': if(WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
		case 'h':	ShowHelp();
			break;
		default :break;
	}
	s_cCmd = 0xff;
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               SensorUartSend
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      Delayms
static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                             SensorDataUpdata
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
  for(i = 0; i < uiRegNum; i++)
  {
      switch(uiReg)
      {
          case AZ:
      s_cDataUpdate |= ACC_UPDATE;
          break;
          case GZ:
      s_cDataUpdate |= GYRO_UPDATE;
          break;
          case HZ:
      s_cDataUpdate |= MAG_UPDATE;
          break;
          case Yaw:
      s_cDataUpdate |= ANGLE_UPDATE;
          break;
          default:
      s_cDataUpdate |= READ_UPDATE;
    break;
      }
  uiReg++;
  }
}

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                               AutoScanSensor
static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
	{
		Serial1.begin(c_uiBaud[i]);
    Serial1.flush();
		iRetry = 2;
		s_cDataUpdate = 0;
		do
		{
			WitReadReg(AX, 3);
			delay(200);
      while (Serial1.available())
      {
        WitSerialDataIn(Serial1.read());
      }
			if(s_cDataUpdate != 0)
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
