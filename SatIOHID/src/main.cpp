/* 

SatIO HID (Human Interface Device) Written by Benjamin Jack Cullen

A large platform allowing for plenty of creative potential for sending user input to SatIO over I2C.

Wiring For Keystudio ATMEGA2560 R3 Development Board with Sheild v1.3:

Interrupt the ESP32 on input:
ESP32: io25 -> ATMEGA2560: io22

Data transmission:
ESP32: I2C SDA -> ATMEGA2560: I2C SDA
ESP32: I2C SCL -> ATMEGA2560: I2C SCL

*/

#include <Arduino.h>
#include <Wire.h>

#define INTERRUPT_ESP32_PIN 22

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                          CONTROL PANEL BUTTONS
// ------------------------------------------------------------------------------------------------------------------------------

/*
digital pins usable for interrupts:	2, 3, 18, 19, 20, 21
if all this microcontroller does is register input then we should get away with polling any buttons that will not fit on digital
interrupt pins.
*/

// -----------------------------------------------------
// interrupt buttons
// -----------------------------------------------------
#define ISRBTN0_PIN 2
#define ISRBTN1_PIN 3
#define ISRBTN2_PIN 18
#define ISRBTN3_PIN 19
bool btnISR0_pressed = false;
bool btnISR1_pressed = false;
bool btnISR2_pressed = false;
bool btnISR3_pressed = false;
// -----------------------------------------------------
// polled buttons
// -----------------------------------------------------
#define BTN23_PIN 23 // 0
#define BTN24_PIN 24 // 1
#define BTN25_PIN 25 // 2
#define BTN26_PIN 26 // 3
#define BTN27_PIN 27 // 4
#define BTN28_PIN 28 // 5
#define BTN29_PIN 29 // 6
#define BTN30_PIN 30 // 7
#define BTN31_PIN 31 // 8
#define BTN32_PIN 32 // 9
#define BTN33_PIN 33 // .
#define BTN34_PIN 34 // -
#define BTN35_PIN 35 // home
#define BTN36_PIN 36 // up
#define BTN37_PIN 37 // right
#define BTN38_PIN 38 // down
#define BTN39_PIN 39 // left
#define BTN40_PIN 40 // enter
#define BTN41_PIN 41 // delete
#define BTN42_PIN 42 // back
// -----------------------------------------------------
#define BTN43_PIN 43 //
#define BTN44_PIN 44 //
#define BTN45_PIN 45 //
// -----------------------------------------------------
// polled analog sticks
// -----------------------------------------------------
#define BTN46_PIN 46 // analog stick 0: x
#define BTN47_PIN 47 // analog stick 0: y
#define BTN48_PIN 48 // analog stick 0: button
#define BTN49_PIN 49 // analog stick 1: x
#define BTN50_PIN 50 // analog stick 1: y
#define BTN51_PIN 51 // analog stick 1: button
// -----------------------------------------------------
#define BTN52_PIN 52 //
#define BTN53_PIN 53 //
// -----------------------------------------------------

bool btnBTN23_pressed = false;
bool btnBTN24_pressed = false;
bool btnBTN25_pressed = false;
bool btnBTN26_pressed = false;
bool btnBTN27_pressed = false;
bool btnBTN28_pressed = false;
bool btnBTN29_pressed = false;
bool btnBTN30_pressed = false;
bool btnBTN31_pressed = false;
bool btnBTN32_pressed = false;
bool btnBTN33_pressed = false;
bool btnBTN34_pressed = false;
bool btnBTN35_pressed = false;
bool btnBTN36_pressed = false;
bool btnBTN37_pressed = false;
bool btnBTN38_pressed = false;
bool btnBTN39_pressed = false;
bool btnBTN40_pressed = false;
bool btnBTN41_pressed = false;
bool btnBTN42_pressed = false;
bool btnBTN43_pressed = false;
bool btnBTN44_pressed = false;
bool btnBTN45_pressed = false;
bool btnBTN46_pressed = false;
bool btnBTN47_pressed = false;
bool btnBTN48_pressed = false;
bool btnBTN49_pressed = false;
bool btnBTN50_pressed = false;
bool btnBTN51_pressed = false;
bool btnBTN52_pressed = false;
bool btnBTN53_pressed = false;
int BTNMATRIX[] = {
    BTN23_PIN,
    BTN24_PIN,
    BTN25_PIN,
    BTN26_PIN,
    BTN27_PIN,
    BTN28_PIN,
    BTN29_PIN,
    BTN30_PIN,
    BTN31_PIN,
    BTN32_PIN,
    BTN33_PIN,
    BTN34_PIN,
    BTN35_PIN,
    BTN36_PIN,
    BTN37_PIN,
    BTN38_PIN,
    BTN39_PIN,
    BTN40_PIN,
    BTN41_PIN,
    BTN42_PIN,
    BTN43_PIN,
    BTN44_PIN,
    BTN45_PIN,
    BTN46_PIN,
    BTN47_PIN,
    BTN48_PIN,
    BTN49_PIN,
    BTN50_PIN,
    BTN51_PIN,
    BTN52_PIN,
    BTN53_PIN
};

// -----------------------------------------
// button value
// -----------------------------------------
int btn_value;
// -----------------------------------------
// analog stick: raw values
// -----------------------------------------
long as_0_x;
long as_0_y;
long as_0_c;
long as_1_x;
long as_1_y;
long as_1_c;
// -----------------------------------------
// analog stick: mapped values
// -----------------------------------------
long asm_0_x;
long asm_0_y;
long asm_0_c;
long asm_1_x;
long asm_1_y;
long asm_1_c;
long asm_map_max=100; // map to 0-100
long asm_map_max_div=asm_map_max/2; // idle value
long stable_breach=0; // test stability at idle (reduce asm_map_max if unstable)
long as_0_u=0;
long as_0_d=0;
long as_0_l=0;
long as_0_r=0;
long as_1_u=0;
long as_1_d=0;
long as_1_l=0;
long as_1_r=0;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA
// ------------------------------------------------------------------------------------------------------------------------------

#define SLAVE_ADDR 8
char device_name_tag[5] = "$CP,"; // control pad

struct I2CLinkStruct {
  char * token;
  byte OUTPUT_BUFFER[32];
  char INPUT_BUFFER[32];
  char TMP_BUFFER0[32];
  char TMP_BUFFER1[32];
  String TMP_BUFFER_STRING;
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     I2C EVENTS
// ------------------------------------------------------------------------------------------------------------------------------

void receiveEvent(int) {
  // read incoming data
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  Serial.println("[received] " + String(I2CLink.INPUT_BUFFER));
  // parse incoming data
  I2CLink.token = strtok(I2CLink.INPUT_BUFFER, ",");
  if (strcmp(I2CLink.token, "$NULL")==0) {}
}

void requestEvent() {
  // Serial.println("[sending] " + String(I2CLink.TMP_BUFFER0));
  // write bytes of chars
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i] = (byte)I2CLink.TMP_BUFFER0[i];}
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
  // clear buffers ready for masters request sweep if multiple i2c devices interrupt on the same masters pin
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                            ISR
// ------------------------------------------------------------------------------------------------------------------------------

void ISR_ISRBTN0() {btnISR0_pressed = true;}
void ISR_ISRBTN1() {btnISR1_pressed = true;}
void ISR_ISRBTN2() {btnISR2_pressed = true;}
void ISR_ISRBTN3() {btnISR3_pressed = true;}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          SETUP
// ------------------------------------------------------------------------------------------------------------------------------

void setup() {
  // --------------------------------------
  // serial
  // --------------------------------------
  Serial.setTimeout(50);
  Serial.begin(115200); while(!Serial);
  // --------------------------------------
  // interrupt buttons
  // --------------------------------------
  pinMode(ISRBTN0_PIN, INPUT_PULLUP);
  pinMode(ISRBTN1_PIN, INPUT_PULLUP);
  pinMode(ISRBTN2_PIN, INPUT_PULLUP);
  pinMode(ISRBTN3_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ISRBTN0_PIN), ISR_ISRBTN0, RISING);
  attachInterrupt(digitalPinToInterrupt(ISRBTN1_PIN), ISR_ISRBTN1, RISING);
  attachInterrupt(digitalPinToInterrupt(ISRBTN2_PIN), ISR_ISRBTN2, RISING);
  attachInterrupt(digitalPinToInterrupt(ISRBTN3_PIN), ISR_ISRBTN3, RISING);
  // --------------------------------------
  // polled buttons
  // --------------------------------------
  pinMode(BTN23_PIN, INPUT_PULLUP);
  pinMode(BTN24_PIN, INPUT_PULLUP);
  pinMode(BTN25_PIN, INPUT_PULLUP);
  pinMode(BTN26_PIN, INPUT_PULLUP);
  pinMode(BTN27_PIN, INPUT_PULLUP);
  pinMode(BTN28_PIN, INPUT_PULLUP);
  pinMode(BTN29_PIN, INPUT_PULLUP);
  pinMode(BTN30_PIN, INPUT_PULLUP);
  pinMode(BTN31_PIN, INPUT_PULLUP);
  pinMode(BTN32_PIN, INPUT_PULLUP);
  pinMode(BTN33_PIN, INPUT_PULLUP);
  pinMode(BTN34_PIN, INPUT_PULLUP);
  pinMode(BTN35_PIN, INPUT_PULLUP);
  pinMode(BTN36_PIN, INPUT_PULLUP);
  pinMode(BTN37_PIN, INPUT_PULLUP);
  pinMode(BTN38_PIN, INPUT_PULLUP);
  pinMode(BTN39_PIN, INPUT_PULLUP);
  pinMode(BTN40_PIN, INPUT_PULLUP);
  pinMode(BTN41_PIN, INPUT_PULLUP);
  pinMode(BTN42_PIN, INPUT_PULLUP);
  pinMode(BTN43_PIN, INPUT_PULLUP);
  pinMode(BTN44_PIN, INPUT_PULLUP);
  pinMode(BTN45_PIN, INPUT_PULLUP);
  // --------------------------------------
  // polled analog
  // --------------------------------------
  pinMode(BTN46_PIN, INPUT_PULLUP);
  pinMode(BTN47_PIN, INPUT_PULLUP);
  pinMode(BTN48_PIN, INPUT_PULLUP);
  pinMode(BTN49_PIN, INPUT_PULLUP);
  pinMode(BTN50_PIN, INPUT_PULLUP);
  pinMode(BTN51_PIN, INPUT_PULLUP);
  pinMode(BTN52_PIN, INPUT_PULLUP);
  pinMode(BTN53_PIN, INPUT_PULLUP);

  // --------------------------------------
  // initialize I2C communications as slave
  // --------------------------------------
  Wire.begin(SLAVE_ADDR);
  // -----------------------------------------------
  // function to run when data requested from master
  // -----------------------------------------------
  Wire.onRequest(requestEvent);
  // -----------------------------------------------
  // function to run when data received from master
  // -----------------------------------------------
  Wire.onReceive(receiveEvent);

  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               INTERRUPT MASTER
// ------------------------------------------------------------------------------------------------------------------------------

void interruptMaster() {
  digitalWrite(INTERRUPT_ESP32_PIN, HIGH);
  digitalWrite(INTERRUPT_ESP32_PIN, LOW);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               CLEAR TMP BUFFER
// ------------------------------------------------------------------------------------------------------------------------------

void clearTMPBuffer() {
  memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      DE-BOUNCE
// ------------------------------------------------------------------------------------------------------------------------------

void deBounce() {
  delay(200);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                           MAIN
// ------------------------------------------------------------------------------------------------------------------------------
bool joy_send_bool=true; // clear by default on startup

void loop() {

  // ----------------------------------------------------------------------------------------------------------------------------
  // Buttons ISR
  // ----------------------------------------------------------------------------------------------------------------------------
  if (btnISR0_pressed==true) {btnISR0_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$CP,B,I0"); interruptMaster();
    Serial.println("[button] ISR0 pressed");
  }
  if (btnISR1_pressed==true) {btnISR1_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$CP,B,I1"); interruptMaster();
    Serial.println("[button] ISR1 pressed");
  }
  if (btnISR2_pressed==true) {btnISR2_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$CP,B,I2"); interruptMaster();
    Serial.println("[button] ISR2 pressed");
  }
  if (btnISR3_pressed==true) {btnISR3_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$CP,B,I3"); interruptMaster();
    Serial.println("[button] ISR3 pressed");
  }

  // ----------------------------------------------------------------------------------------------------------------------------
  // Buttons Polled
  // ----------------------------------------------------------------------------------------------------------------------------
  btn_value=NAN;
  for (int i=0; i<31; i++) {
    btn_value = digitalRead(BTNMATRIX[i]);
    if (btn_value==LOW) {
      Serial.println("[button] " + String(i) + " pressed");
      deBounce();
      clearTMPBuffer();
      strcat(I2CLink.TMP_BUFFER0, device_name_tag);
      strcat(I2CLink.TMP_BUFFER0, "B,");
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1)); itoa(i, I2CLink.TMP_BUFFER1, 10);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      interruptMaster();
      break;
    }
  }

  // ----------------------------------------------------------------------------------------------------------------------------
  // Analog Sticks Polled
  // ----------------------------------------------------------------------------------------------------------------------------
  // -----------------------------------------
  // poll analog stick 0
  // -----------------------------------------
  as_0_x = analogRead(A0);
  as_0_y = analogRead(A1);
  as_0_c = analogRead(A2);
  // -----------------------------------------
  // poll analog stick 1
  // -----------------------------------------
  as_1_x = analogRead(A3);
  as_1_y = analogRead(A4);
  as_1_c = analogRead(A5);
  // Serial.println("[as_0_xyc raw] " + String(as_0_x) + "," + String(as_0_y) + "," + String(as_0_c));
  // Serial.println("[as_1_xyc raw] " + String(as_1_x) + "," + String(as_1_y) + "," + String(as_1_c));
  /*
  // ------------------------------------------------------------------------------------------------
  Optional Processing.
  // ------------------------------------------------------------------------------------------------
  
  Option 0: Send raw data over I2C (6 analog values, xyz+xyz). Fastest to send. Unprocessed.
            Raw values: 0-1024. 
                  tag x0   y0   c0   x1   y1   c1  
            Send: $A0,0000,0000,0000,0000,0000,0000
             
  Option 1: Send processed data over I2C (10 analog values, udlrc+udlrc). Slower to send. Semi-Processed.
            Raw values: 0-512 and 512-1024.
                  tag   u0   d0   l0   r0   c0   u1   d1   l1   r1   c1 
            Send: $0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000

  Option 3: Send processed, mapped data over I2C (10 mapped values, udlrc+udlrc). 2nd Fastest to send. Fully-Processed.
             Mapped values: Account for analog voltage fluctuation and analog voltage balance offsets.
                   tag u0 d0 l0 r0 c0 u1 d1 l1 r1 c1 
             Send: $A0,00,00,00,00,00,00,00,00,00,00
  */
  // -----------------------------------------
  // Option 3: Calibration and Stabalization
  // -----------------------------------------
  // balance: adjust raw values to 512 approx. 
  // -----------------------------------------
  as_0_x = as_0_x+6; if (as_0_x>1024) {as_0_x=1024;} else if (as_0_x<0) {as_0_x=0;}
  as_0_y = as_0_y-4; if (as_0_y>1024) {as_0_y=1024;} else if (as_0_y<0) {as_0_y=0;}
  as_1_x = as_1_x+8; if (as_1_x>1024) {as_1_x=1024;} else if (as_1_x<0) {as_1_x=0;}
  as_1_y = as_1_y+8; if (as_1_y>1024) {as_1_y=1024;} else if (as_1_y<0) {as_1_y=0;}
  // Serial.println("[as_0_xyc bal] " + String(as_0_x) + "," + String(as_0_y) + "," + String(as_0_c));
  // Serial.println("[as_1_xyc bal] " + String(as_1_x) + "," + String(as_1_y) + "," + String(as_1_c));
  // -----------------------------------------
  // map 1: map values to reduced stable values.
  // -----------------------------------------
  asm_0_x = map(as_0_x, 0, 1024, 0, asm_map_max);
  asm_0_y = map(as_0_y, 0, 1024, 0, asm_map_max);
  if (as_0_c>0) {asm_0_c=0;} else {asm_0_c=1;}
  asm_1_x = map(as_1_x, 0, 1024, 0, asm_map_max);
  asm_1_y = map(as_1_y, 0, 1024, 0, asm_map_max);
  if (as_1_c>0) {asm_1_c=0;} else {asm_1_c=1;}
  // Serial.println("[asm_0_xyc map] " + String(asm_0_x) + "," + String(asm_0_y) + "," + String(asm_0_c));
  // Serial.println("[asm_1_xyc map] " + String(asm_1_x) + "," + String(asm_1_y) + "," + String(asm_1_c));
  // -----------------------------------------
  // test: uncomment & do not touch the sticks
  // -----------------------------------------
  // if (asm_0_x!=asm_map_max_div || asm_0_y!=asm_map_max_div || asm_1_x!=asm_map_max_div || asm_1_y!=asm_map_max_div) {Serial.println("[unstable mapping] (adjust balance/mapping)"); stable_breach++; delay(1000);}
  // Serial.println("[instability count] " + String(stable_breach));
  // -----------------------------------------
  // if not idle
  // -----------------------------------------
  if (asm_0_x!=asm_map_max_div || asm_0_y!=asm_map_max_div || asm_1_x!=asm_map_max_div || asm_1_y!=asm_map_max_div || asm_0_c!=0 || asm_1_c!=0) {
      // -----------------------------------------
      // map 2: map values to direction
      // -----------------------------------------
      if (asm_0_x<asm_map_max_div) {as_0_r=0; as_0_l=map(asm_0_x, asm_map_max_div, 0, 0, asm_map_max_div);}
      else {as_0_l=0; as_0_r=map(asm_0_x, asm_map_max_div, asm_map_max, 0, asm_map_max_div);}
      if (asm_0_y<asm_map_max_div) {as_0_d=0; as_0_u=map(asm_0_y, asm_map_max_div, 0, 0, asm_map_max_div);}
      else {as_0_u=0; as_0_d=map(asm_0_y, asm_map_max_div, asm_map_max, 0, asm_map_max_div);}
      if (asm_1_x<asm_map_max_div) {as_1_r=0;as_1_l=map(asm_1_x, asm_map_max_div, 0, 0, asm_map_max_div);}
      else {as_1_l=0;as_1_r=map(asm_1_x, asm_map_max_div, asm_map_max, 0, asm_map_max_div);}
      if (asm_1_y<asm_map_max_div) {as_1_d=0;as_1_u=map(asm_1_y, asm_map_max_div, 0, 0, asm_map_max_div);}
      else {as_1_u=0; as_1_d=map(asm_1_y, asm_map_max_div, asm_map_max, 0, asm_map_max_div);}
      // Serial.println("[as0 up] " + String(as_0_u) + " [as0 down] " + String(as_0_d) + " [as0 left] " + String(as_0_l) + " [as0 right] " + String(as_0_r));
      // Serial.println("[as1 up] " + String(as_1_u) + " [as1 down] " + String(as_1_d) + " [as1 left] " + String(as_1_l) + " [as1 right] " + String(as_1_r));
      // -----------------------------------------
      // send
      // -----------------------------------------
      clearTMPBuffer();
      I2CLink.TMP_BUFFER_STRING = String(device_name_tag) +
                                  String("A,") +
                                  String(as_0_u) + String(",") +
                                  String(as_0_d) + String(",") +
                                  String(as_0_l) + String(",") +
                                  String(as_0_r) + String(",") +
                                  String(asm_0_c) + String(",") +
                                  String(as_1_u) + String(",") +
                                  String(as_1_d) + String(",") +
                                  String(as_1_l) + String(",") +
                                  String(as_1_r) + String(",") +
                                  String(asm_1_c)
                                  ;
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER_STRING.c_str());
      // Serial.println("[I2CLink.TMP_BUFFER0] " + String(I2CLink.TMP_BUFFER0));
      interruptMaster();
      joy_send_bool=true;
      delay(200); // experiment with delay so that we do not interrupt master too many times a second when stick non idle
  }
  else {
    if (joy_send_bool==true) {
      // -----------------------------------------
      // send idle
      // -----------------------------------------
      clearTMPBuffer();
      I2CLink.TMP_BUFFER_STRING = String(device_name_tag) +
                                  String("A,") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0") + String(",") +
                                  String("0")
                                  ;
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER_STRING.c_str());
      // Serial.println("[I2CLink.TMP_BUFFER0] " + String(I2CLink.TMP_BUFFER0));
      interruptMaster();
      delay(200); // experiment with delay so that we do not interrupt master too many times a second when stick non idle
      joy_send_bool=false;
    }
  }
  // Serial.println("---------------------------");
  // delay(100);
}
