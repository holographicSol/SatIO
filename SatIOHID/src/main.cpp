/* 

SatIO HID (Human Interface Device) Written by Benjamin Jack Cullen

A large platform for sending user input to SatIO over I2C.

Wiring For Keystudio ATMEGA2560 R3 Development Board with Sheild v1.3:

Interrupt the ESP32 on input:
ESP32: io25    -> ATMEGA2560: io22
Data transmission:
ESP32: I2C SDA -> ATMEGA2560: I2C SDA
ESP32: I2C SCL -> ATMEGA2560: I2C SCL

Button 0:
ATMEGA2560: io2 -> Button: SIG

*/

// ------------------------------------------------------------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#define INTERRUPT_ESP32_PIN 22

// Digital Pins Usable For Interrupts:	2, 3, 18, 19, 20, 21
#define BTN0_PIN 2

bool btn0_pressed = false;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA

// Define Slave I2C Address
#define SLAVE_ADDR 8

struct I2CLinkStruct {
  char * token;
  byte OUTPUT_BUFFER[10];
  char INPUT_BUFFER[10];
  char TMP_BUFFER[10];
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     I2C EVENTS

void receiveEvent(int) {

  Serial.println("[slave] read data");
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  // Serial.println("[received] " + String(INPUT_BUFFER)); // (avg. 333 messages a second???)

  // get tag token
  I2CLink.token = strtok(I2CLink.INPUT_BUFFER, ",");

  // Port Map
  if (strcmp(I2CLink.token, "$P")==0) {}
}

void requestEvent() {
  // Write Bytes of Chars
  Serial.println("[slave] write data");
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (int i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i] = (byte)I2CLink.TMP_BUFFER[i];}
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
  memset(I2CLink.TMP_BUFFER, 0, sizeof(I2CLink.TMP_BUFFER));
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                            ISR

void ISR_BTN0() {
  btn0_pressed = true;
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                          SETUP

void setup() {
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200); while(!Serial);

  pinMode(BTN0_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN0_PIN), ISR_BTN0, RISING);

  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(requestEvent);

  // Function to run when data received from master
  Wire.onReceive(receiveEvent);

  Serial.println("starting...");
}

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                           MAIN

void loop() {
  if (btn0_pressed==true) {
    btn0_pressed=false;
    Serial.println("[button] 0 pressed");
    strcpy(I2CLink.TMP_BUFFER, "$B,0");
    digitalWrite(INTERRUPT_ESP32_PIN, HIGH);
    digitalWrite(INTERRUPT_ESP32_PIN, LOW);
  }
}
