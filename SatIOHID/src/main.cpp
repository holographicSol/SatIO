/* 

SatIO HID (Human Interface Device) Written by Benjamin Jack Cullen

A large platform allowing for plenty of creative potential for sending user input to SatIO over I2C.

Wiring For Keystudio ATMEGA2560 R3 Development Board with Sheild v1.3:

Interrupt the ESP32 on input:
ESP32: io25    -> ATMEGA2560: io22

Data transmission:
ESP32: I2C SDA -> ATMEGA2560: I2C SDA
ESP32: I2C SCL -> ATMEGA2560: I2C SCL

*/

// ------------------------------------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#define INTERRUPT_ESP32_PIN 22

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                          CONTROL PANEL BUTTONS

/*

digital pins usable for interrupts:	2, 3, 18, 19, 20, 21
if all this microcontroller does is register input then we should get away with polling any buttons that will not fit on digital
interrupt pins.

*/

// interrupt buttons
#define ISRBTN0_PIN 2
#define ISRBTN1_PIN 3
#define ISRBTN2_PIN 18
#define ISRBTN3_PIN 19
bool btnISR0_pressed = false;
bool btnISR1_pressed = false;
bool btnISR2_pressed = false;
bool btnISR3_pressed = false;

// polled buttons
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
#define BTN36_PIN 36
#define BTN37_PIN 37
#define BTN38_PIN 38
#define BTN39_PIN 39
#define BTN40_PIN 40
#define BTN41_PIN 41
#define BTN42_PIN 42

#define BTN43_PIN 43
#define BTN44_PIN 44
#define BTN45_PIN 45
#define BTN46_PIN 46
#define BTN47_PIN 47
#define BTN48_PIN 48
#define BTN49_PIN 49
#define BTN50_PIN 50
#define BTN51_PIN 51
#define BTN52_PIN 52
#define BTN53_PIN 53
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


// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA

#define SLAVE_ADDR 8

struct I2CLinkStruct {
  char * token;
  byte OUTPUT_BUFFER[10];
  char INPUT_BUFFER[10];
  char TMP_BUFFER0[10];
  char TMP_BUFFER1[10];
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     I2C EVENTS

void receiveEvent(int) {

  // read incoming data
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  // Serial.println("[received] " + String(INPUT_BUFFER));

  // parse incoming data
  I2CLink.token = strtok(I2CLink.INPUT_BUFFER, ",");
  if (strcmp(I2CLink.token, "$NULL")==0) {}
}

void requestEvent() {

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

void ISR_ISRBTN0() {btnISR0_pressed = true;}
void ISR_ISRBTN1() {btnISR1_pressed = true;}
void ISR_ISRBTN2() {btnISR2_pressed = true;}
void ISR_ISRBTN3() {btnISR3_pressed = true;}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          SETUP

void setup() {

  // serial
  Serial.setTimeout(50);
  Serial.begin(115200); while(!Serial);

  // interrupt buttons
  pinMode(ISRBTN0_PIN, INPUT_PULLUP);
  pinMode(ISRBTN1_PIN, INPUT_PULLUP);
  pinMode(ISRBTN2_PIN, INPUT_PULLUP);
  pinMode(ISRBTN3_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ISRBTN0_PIN), ISR_ISRBTN0, RISING);
  attachInterrupt(digitalPinToInterrupt(ISRBTN1_PIN), ISR_ISRBTN1, RISING);
  attachInterrupt(digitalPinToInterrupt(ISRBTN2_PIN), ISR_ISRBTN2, RISING);
  attachInterrupt(digitalPinToInterrupt(ISRBTN3_PIN), ISR_ISRBTN3, RISING);

  // polled buttons
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
  pinMode(BTN46_PIN, INPUT_PULLUP);
  pinMode(BTN47_PIN, INPUT_PULLUP);
  pinMode(BTN48_PIN, INPUT_PULLUP);
  pinMode(BTN49_PIN, INPUT_PULLUP);
  pinMode(BTN50_PIN, INPUT_PULLUP);
  pinMode(BTN51_PIN, INPUT_PULLUP);
  pinMode(BTN52_PIN, INPUT_PULLUP);
  pinMode(BTN53_PIN, INPUT_PULLUP);

  // initialize I2C communications as slave
  Wire.begin(SLAVE_ADDR);

  // function to run when data requested from master
  Wire.onRequest(requestEvent);

  // function to run when data received from master
  Wire.onReceive(receiveEvent);

  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               INTERRUPT MASTER

void interruptMaster() {
  digitalWrite(INTERRUPT_ESP32_PIN, HIGH);
  digitalWrite(INTERRUPT_ESP32_PIN, LOW);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                               CLEAR TMP BUFFER

void clearTMPBuffer() {
  memset(I2CLink.TMP_BUFFER0, 0, sizeof(I2CLink.TMP_BUFFER0));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                      DE-BOUNCE

void deBounce() {
  delay(200);
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                           MAIN

void loop() {

  /* check button pressed interrupt flags */

  if (btnISR0_pressed==true) {btnISR0_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$B,ISR0"); interruptMaster();
    Serial.println("[button] ISR0 pressed");
  }
  if (btnISR1_pressed==true) {btnISR1_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$B,ISR1"); interruptMaster();
    Serial.println("[button] ISR1 pressed");
  }
  if (btnISR2_pressed==true) {btnISR2_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$B,ISR2"); interruptMaster();
    Serial.println("[button] ISR2 pressed");
  }
  if (btnISR3_pressed==true) {btnISR3_pressed=false; clearTMPBuffer(); strcpy(I2CLink.TMP_BUFFER0, "$B,ISR3"); interruptMaster();
    Serial.println("[button] ISR3 pressed");
  }

  /* poll button presses */

  for (int i=0; i<31; i++) {
    if (digitalRead(BTNMATRIX[i])==LOW) {
      Serial.println("[button] " + String(i) + " pressed");
      deBounce();
      clearTMPBuffer();
      strcpy(I2CLink.TMP_BUFFER0, "$B,");
      memset(I2CLink.TMP_BUFFER1, 0, sizeof(I2CLink.TMP_BUFFER1)); itoa(i, I2CLink.TMP_BUFFER1, 10);
      strcat(I2CLink.TMP_BUFFER0, I2CLink.TMP_BUFFER1);
      interruptMaster();
    }
  }
}
