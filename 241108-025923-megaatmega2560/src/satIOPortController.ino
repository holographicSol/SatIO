/*
Serial Link - Stable inter-microcontroller serial communication. Written by Benjamin Jack Cullen

SatIO PortController - Written by Benjamin Jack Cullen

Currently waiting for 4P 1.25mm JST connectors to connect this port controller to the CYD, so this is untested and
in development.

*/


#include <Arduino.h>

// ------------------------------------------------------------------------------------------------------------------
#define ETX 0x03  // end of text character

// ------------------------------------------------------------------------------------------------------------------

// a placeholder for matrix switch ports
signed int matrix_port_map[1][20] = {
  {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  }
};

// reflects matrix switch active/inactive states each loop of matrix switch function
bool matrix_switch_state[1][20] = {
  {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  }
};

// SERIAL LINK STRUCT -----------------------------------------------------------------------------------------------
struct SerialLinkStruct {
  char debugData[1024];
  unsigned long nbytes;
  unsigned long i_nbytes;
  char BUFFER[1024];            // read incoming bytes into this buffer
  char DATA[1024];              // buffer refined using ETX
  unsigned long T0_RXD_1 = 0;   // hard throttle current time
  unsigned long T1_RXD_1 = 0;   // hard throttle previous time
  unsigned long TT_RXD_1 = 0;   // hard throttle interval
  unsigned long T0_TXD_1 = 0;   // hard throttle current time
  unsigned long T1_TXD_1 = 0;   // hard throttle previous time
  unsigned long TT_TXD_1 = 0;  // hard throttle interval
  unsigned long TOKEN_i;
  char * token = strtok(BUFFER, ",");
};
SerialLinkStruct SerialLink;

// DEBUG DATA -------------------------------------------------------------------------------------------------------
void DebugData(){
  strcat(SerialLink.debugData, "");
  // ...strcats
}

// DEBUG SERIAL -----------------------------------------------------------------------------------------------------
void DebugSerial(){
  Serial.print("[DEBUG] "); Serial.println(SerialLink.debugData);
}

// SETUP ------------------------------------------------------------------------------------------------------------
void setup(void) {
  Serial.begin(115200); while(!Serial);
  Serial1.begin(115200); while(!Serial1);
  Serial1.setTimeout(5);
  Serial.flush();
  Serial1.flush();

  /* 
  setup ports to be controlled. this setup is for testing. actual ports are dictated by SatIO.
  */
  for (int i=22; i<53; i++) {
    pinMode(i, OUTPUT);
    matrix_port_map[0][i] = i;
    digitalWrite(matrix_port_map[0][i], LOW);
  }
}

// RXD1 THROTTLE CHECKS ---------------------------------------------------------------------------------------------
bool RXD1ThrottleChecks() {
  SerialLink.T0_RXD_1 = millis();
  if (SerialLink.T0_RXD_1 >= SerialLink.T1_RXD_1+SerialLink.TT_RXD_1) {
    SerialLink.T1_RXD_1 = SerialLink.T0_RXD_1;
    return true;
  }
}

// TXD1 THROTTLE CHECKS ---------------------------------------------------------------------------------------------
bool TXD1ThrottleChecks() {
SerialLink.T0_TXD_1 = millis();
if (SerialLink.T0_TXD_1 >= SerialLink.T1_TXD_1+SerialLink.TT_TXD_1) {
  SerialLink.T1_TXD_1 = SerialLink.T0_TXD_1;
  return true;
}
}

// RXD1 READ BYTES UNTIL ETX ----------------------------------------------------------------------------------------
bool readBytesUntilETX() {
  memset(SerialLink.BUFFER, 0, 1024);
  memset(SerialLink.DATA, 0, 1024);
  SerialLink.nbytes = Serial1.readBytes(SerialLink.BUFFER, 1024);
  if (SerialLink.nbytes != 0) {
    for(SerialLink.i_nbytes = 0; SerialLink.i_nbytes < SerialLink.nbytes; SerialLink.i_nbytes++) {
      if (SerialLink.BUFFER[SerialLink.i_nbytes] == ETX)
        return true;
      else {SerialLink.DATA[SerialLink.i_nbytes] = SerialLink.BUFFER[SerialLink.i_nbytes];}
    }
  }
}

// PASSTHROUGH ANY --------------------------------------------------------------------------------------------------
void MatrixBool() {

  /*
  check message integrity then set matrix_switch_state with tokens
  */

  while( SerialLink.token != NULL ) {
    Serial.print("[RXD TOKEN "); Serial.print(SerialLink.TOKEN_i); Serial.print("] "); Serial.println(SerialLink.token);
    SerialLink.token = strtok(NULL, ",");
    SerialLink.TOKEN_i++;
  }
}

// COMPARE TOKENS ---------------------------------------------------------------------------------------------------
void CompareTokens(char * tk) {

  /*
  handle tag: $MATRIXSWITCH
  */

  if (strcmp(SerialLink.token, "$MATRIXSWITCH") == 0) {MatrixBool();}
}

// READ RXD1 METHOD 0 -----------------------------------------------------------------------------------------------
void readRXD1_Method0() {
  if (RXD1ThrottleChecks() == true) {
    if (Serial1.available() > 0) {
      if (readBytesUntilETX() == true) {
        Serial.println("-------------------------------------------");
        Serial.print("[RXD]         "); Serial.println(SerialLink.DATA);
        SerialLink.TOKEN_i = 0;
        SerialLink.token = strtok(SerialLink.DATA, ",");
        CompareTokens(SerialLink.token);
      }
    }
  }
}

// READ RXD1 METHOD 1 -----------------------------------------------------------------------------------------------
void readRXD1_Method1() {
  if (Serial1.available() > 0) {
    memset(SerialLink.BUFFER, 0, 1024);
    SerialLink.nbytes = (Serial1.readBytesUntil(ETX, SerialLink.BUFFER, sizeof(SerialLink.BUFFER)));
    Serial.println(SerialLink.BUFFER);
  }
}

// WRITE TXD1 -------------------------------------------------------------------------------------------------------
void WriteTXD1() {
  if (TXD1ThrottleChecks() == true) {
    if (Serial1.availableForWrite()) {
      Serial.print("[TXD] "); Serial.println(SerialLink.BUFFER);
      Serial1.write(SerialLink.BUFFER);
      Serial1.write(ETX);
    }
  }
}

// TXD2 DATA --------------------------------------------------------------------------------------------------------
void TXD1Data() {
  // send a peripherals state to the display peripheral
  memset(SerialLink.BUFFER, 0, 1024);
  strcat(SerialLink.BUFFER, "");
  WriteTXD1();
}

// LOOP -------------------------------------------------------------------------------------------------------------
void loop() {
  readRXD1_Method0();
}
