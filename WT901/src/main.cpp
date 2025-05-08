/*

9-Axis Gyro Module written by Benjamin Jack Cullen and adapted from official example code written by Witmotion.

The WT901 is a IMU sensor device, detecting acceleration, angular velocity, angle as well as magnetic filed.

Todo:
Enable/disable resolution compensation.
Push buttons:
  Calibrate angle.
  Calibrate magnetic field.
  Enable/disable serial output.
  Enable/disable IIC slave mode.
  Enable/disable LEDs.
  Enable/disable OLED display.
  Enable/disable 7 segment displays.
*/

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    LIBRARIES
// ----------------------------------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include "./REG.h"
#include "./wit_c_sdk.h"
#include <Wire.h>

// ----------------------------------------------------------------------------------------------------------------------------
//                                                                                                                        WT901
// ----------------------------------------------------------------------------------------------------------------------------

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate=0, s_cCmd=0xff; 
void CopeCmdData(unsigned char ucData);
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
bool enable_resolution_compensation_a=false;
bool enable_resolution_compensation_b=false;
bool enable_resolution_compensation_c=false;
bool enable_resolution_compensation_d=true;

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

  // -----------------------------------------------
  // read incoming data
  // -----------------------------------------------
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  // Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));

  // -----------------------------------------------
  // parse incoming data
  // -----------------------------------------------
  I2CLink.token=strtok(I2CLink.INPUT_BUFFER, ",");
  if (strcmp(I2CLink.token, "$A")==0) {I2CLink.request_counter=0;}
  else if (strcmp(I2CLink.token, "$B")==0) {I2CLink.request_counter=1;}
  else if (strcmp(I2CLink.token, "$C")==0) {I2CLink.request_counter=2;}
  else if (strcmp(I2CLink.token, "$D")==0) {I2CLink.request_counter=3;}
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
    if (enable_resolution_compensation_a==true) {
      // ---------------------------------------------
      // Acceleration X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_acc_x0=acc_x_high;
      if (abs(acc_x_low) > acc_x_high) {tmp_acc_x0=acc_x_low;}
      dtostrf(tmp_acc_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Acceleration Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_acc_y0=acc_y_high;
      if (abs(acc_y_low) > acc_y_high) {tmp_acc_y0=acc_y_low;}
      dtostrf(tmp_acc_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Acceleration Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_acc_z0=acc_z_high;
      if (abs(acc_z_low) > acc_z_high) {tmp_acc_z0=acc_z_low;}
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
    if (enable_resolution_compensation_b==true) {
      // ---------------------------------------------
      // Angle X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_ang_x0=ang_x_high;
      if (abs(ang_x_low) > ang_x_high) {tmp_ang_x0=ang_x_low;}
      dtostrf(tmp_ang_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Angle Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_ang_y0=ang_y_high;
      if (abs(ang_y_low) > ang_y_high) {tmp_ang_y0=ang_y_low;}
      dtostrf(tmp_ang_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Angle Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_ang_z0=ang_z_high;
      if (abs(ang_z_low) > ang_z_high) {tmp_ang_z0=ang_z_low;}
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
    if (enable_resolution_compensation_c==true) {
      // ---------------------------------------------
      // Gyro X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_gyr_x0=gyr_x_high;
      if (abs(gyr_x_low) > gyr_x_high) {tmp_gyr_x0=gyr_x_low;}
      dtostrf(tmp_gyr_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Gyro Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_gyr_y0=gyr_y_high;
      if (abs(gyr_y_low) > gyr_y_high) {tmp_gyr_y0=gyr_y_low;}
      dtostrf(tmp_gyr_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Gyro Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_gyr_z0=gyr_z_high;
      if (abs(gyr_z_low) > gyr_z_high) {tmp_gyr_z0=gyr_z_low;}
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
    if (enable_resolution_compensation_d==true) {
      // ---------------------------------------------
      // Mag X (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_mag_x0=mag_x_high;
      if (abs(mag_x_low) > mag_x_high) {tmp_mag_x0=mag_x_low;}
      dtostrf(tmp_mag_x0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Mag Y (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_mag_y0=mag_y_high;
      if (abs(mag_y_low) > mag_y_high) {tmp_mag_y0=mag_y_low;}
      dtostrf(tmp_mag_y0, 1, 3, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      strcat(I2CLink.TMP_BUFFER0, ",");
      // ---------------------------------------------
      // Mag Z (special resolution compensation)
      // ---------------------------------------------
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1));
      tmp_mag_z0=mag_z_high;
      if (abs(mag_z_low) > mag_z_high) {tmp_mag_z0=mag_z_low;}
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
//                                                                                                                           LOOP
// ------------------------------------------------------------------------------------------------------------------------------

void loop() {
  loops_between_requests++;

  while (Serial1.available())
  {
    WitSerialDataIn(Serial1.read());
  }
  while (Serial.available()) 
  {
    CopeCmdData(Serial.read());
  }
  CmdProcess();
  if (s_cDataUpdate)
  {
    for(int i=0; i < 3; i++)
    {
      fAcc[i]=sReg[AX+i] / 32768.0f * 16.0f;
      fGyro[i]=sReg[GX+i] / 32768.0f * 2000.0f;
      fAngle[i]=sReg[Roll+i] / 32768.0f * 180.0f;
    }
    if (s_cDataUpdate & ACC_UPDATE)
    {
      s_cDataUpdate &= ~ACC_UPDATE;
      if (enable_resolution_compensation_a==true) {
        if (fAcc[0]>acc_x_high) {acc_x_high=fAcc[0];}
        if (fAcc[0]<acc_x_low) {acc_x_low=fAcc[0];}
        if (fAcc[1]>acc_y_high) {acc_y_high=fAcc[1];}
        if (fAcc[1]<acc_y_low) {acc_y_low=fAcc[1];}
        if (fAcc[2]>acc_z_high) {acc_z_high=fAcc[2];}
        if (fAcc[2]<acc_z_low) {acc_z_low=fAcc[2];}
      }
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
    if (s_cDataUpdate & GYRO_UPDATE)
    {
      s_cDataUpdate &= ~GYRO_UPDATE;
      if (enable_resolution_compensation_b==true) {
        if (fGyro[0]>gyr_x_high) {gyr_x_high=fGyro[0];}
        if (fGyro[0]<gyr_x_low) {gyr_x_low=fGyro[0];}
        if (fGyro[1]>gyr_y_high) {gyr_y_high=fGyro[1];}
        if (fGyro[1]<gyr_y_low) {gyr_y_low=fGyro[1];}
        if (fGyro[2]>gyr_z_high) {gyr_z_high=fGyro[2];}
        if (fGyro[2]<gyr_z_low) {gyr_z_low=fGyro[2];}
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
    if (s_cDataUpdate & ANGLE_UPDATE)
    {
      s_cDataUpdate &= ~ANGLE_UPDATE;
      if (enable_resolution_compensation_c==true) {
        if (fAngle[0]>ang_x_high) {ang_x_high=fAngle[0];}
        if (fAngle[0]<ang_x_low) {ang_x_low=fAngle[0];}
        if (fAngle[1]>ang_y_high) {ang_y_high=fAngle[1];}
        if (fAngle[1]<ang_y_low) {ang_y_low=fAngle[1];}
        if (fAngle[2]>ang_z_high) {ang_z_high=fAngle[2];}
        if (fAngle[2]<ang_z_low) {ang_z_low=fAngle[2];}
      }
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
    if (s_cDataUpdate & MAG_UPDATE)
    {
      s_cDataUpdate &= ~MAG_UPDATE;
      if (enable_resolution_compensation_d==true) {
        if (sReg[HX]>mag_x_high) {mag_x_high=sReg[HX];}
        if (sReg[HX]<mag_x_low) {mag_x_low=sReg[HX];}
        if (sReg[HY]>mag_y_high) {mag_y_high=sReg[HY];}
        if (sReg[HY]<mag_y_low) {mag_y_low=sReg[HY];}
        if (sReg[HZ]>mag_z_high) {mag_z_high=sReg[HZ];}
        if (sReg[HZ]<mag_z_low) {mag_z_low=sReg[HZ];}
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
//                                                                                                                    CopeCmdData
// ------------------------------------------------------------------------------------------------------------------------------
// read one character from RX buffer each time CopeCmdData is called.
// ------------------------------------------------------------------------------------------------------------------------------

static unsigned char s_ucData[50], s_ucRxCnt=0;

void CopeCmdData(unsigned char ucData) {
  // --------------------------------------------------
  // append current char to chars
  // --------------------------------------------------
  s_ucData[s_ucRxCnt++]=ucData;
  // --------------------------------------------------
  // return now while below expected command length
  // --------------------------------------------------
  if (s_ucRxCnt<3) {return;}
  // --------------------------------------------------
  // handle potential overflow
  // --------------------------------------------------
  if (s_ucRxCnt >= 50) {s_ucRxCnt=0;}

  // --------------------------------------------------
  // continue if all expected chars have been collected
  // --------------------------------------------------
  if (s_ucRxCnt >= 3) {
    // ------------------------------------------------------------------
    // accept one character command ending in carriage return and newline
    // ------------------------------------------------------------------
    if ((s_ucData[1] == '\r') && (s_ucData[2] == '\n')) {
      // ----------------------------------------------
      // isolate command char for command processing
      // ----------------------------------------------
      s_cCmd=s_ucData[0];
      // ----------------------------------------------
      // reset
      // ----------------------------------------------
      memset(s_ucData,0,50);
      s_ucRxCnt=0;
    }

    // ------------------------------------------------
    // shift and continue
    // ------------------------------------------------
    else {
      s_ucData[0]=s_ucData[1];
      s_ucData[1]=s_ucData[2];
      s_ucRxCnt=2;
    }
  }
}
                   
// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       ShowHelp
// ------------------------------------------------------------------------------------------------------------------------------

static void ShowHelp(void)
{
  Serial.println("---------------------------------------------------------------------------------------------------");
  Serial.println("                                               HELP                                                ");
  Serial.println("UART SEND: a\\r\\n   Acceleration calibration.");
  Serial.println("UART SEND: m\\r\\n   Enter Magnetic field calibration. To end calibration send: e\\r\\n");
  Serial.println("UART SEND: U\\r\\n   Bandwidth increase.");
  Serial.println("UART SEND: u\\r\\n   Bandwidth reduction.");
  Serial.println("UART SEND: B\\r\\n   Baud rate increased to 115200.");
  Serial.println("UART SEND: b\\r\\n   Baud rate reduction to 9600.");
  Serial.println("UART SEND: R\\r\\n   The return rate increases to 10Hz.");
  Serial.println("UART SEND: r\\r\\n   The return rate reduction to 1Hz.");
  Serial.println("UART SEND: C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.");
  Serial.println("UART SEND: c\\r\\n   Return content: acceleration.");
  Serial.println("UART SEND: O\\r\\n   Enable return content.");
  Serial.println("UART SEND: o\\r\\n   Disable return content.");
  Serial.println("UART SEND: h\\r\\n   Help.");
  Serial.println("---------------------------------------------------------------------------------------------------");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     CmdProcess
// ------------------------------------------------------------------------------------------------------------------------------

static void CmdProcess(void) {
  switch(s_cCmd)
  {
    case 'a': if (WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n"); break;
    case 'm': if (WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n"); break;
    case 'e': if (WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n"); break;
    case 'u': if (WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n"); break;
    case 'U': if (WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n"); break;
    case 'B': if (WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else {Serial1.begin(c_uiBaud[WIT_BAUD_115200]); Serial.print(" 115200 Baud rate modified successfully\r\n");}
              break;
    case 'b':	if (WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else {Serial1.begin(c_uiBaud[WIT_BAUD_9600]); Serial.print(" 9600 Baud rate modified successfully\r\n");}
              break;
    case 'r': if (WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)  Serial.print("\r\nSet Baud Error\r\n");
              else Serial.print("\r\nSet Baud Success\r\n");
              break;
    case 'R': if (WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else Serial.print("\r\nSet Baud Success\r\n");
              break;
    case 'C': if (WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n"); break;
    case 'c': if (WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n"); break;
    case 'O': enable_serial_output_content=true; break;
    case 'o': enable_serial_output_content=false; break;
    case 'h':	ShowHelp(); break;
    default : break;
  }
  s_cCmd=0xff;
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
//                                                                                                              SensorDataUpdata
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
