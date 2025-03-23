/*
Written by Benjamin Jack Cullen. Source material is in the SiderealObjects examples.

Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html).
Additions: 1: doXRiseSetTimes(). This allows for calculating rise and set times of all planets and objects according to time and location.
           2: inRange60(). Ensures minutes and second values are wihin 0-59 for planet/object rise, set times.
           3: inRange24(). Ensures hour values are wihin 0-23 for planet/object rise, set times.
*/

#include <SiderealPlanets.h>  // https://github.com/DavidArmstrong/SiderealPlanets
#include <SiderealObjects.h>  // https://github.com/DavidArmstrong/SiderealObjects

SiderealPlanets myAstro;    // for calculating azimuth and altitude
SiderealObjects myAstroObj; // for getting right ascension and declination of objects from star table

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
  float objects_data[609][7];
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
  myAstro.doXRiseSetTimes(); // requires modified library
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

void setup() {
  Serial.begin(115200);
}

void scenarioZero() {

  /*
  Identify object before tracking object.
  RA and DEC are known somehow (like a static list or dynamically converted from gyroscopic data).
  We dont know what is in front of us but we can try to find out, then label and map anything that is found.
  */

  // identify object
  IdentifyObject(6.75, -16.72); // hardcoded RA, DEC for Sirius could be values converted from gyroscopic data
  Serial.println("[object_number] " + String(siderealObjectData.object_number));
  Serial.println("[object_name] " + String(siderealObjectData.object_name));
  Serial.println("[object_table_i] " + String(siderealObjectData.object_table_i));
  Serial.println("[object_table_name] " + String(siderealObjectData.object_table_name));

  // track object
  trackObject(
    51.2097664, -2.6489721,
    2025,
    3,
    23,
    3,
    36,
    30,
    siderealObjectData.object_table_i,  // object table
    siderealObjectData.object_number // object id
  );
  Serial.println("[object_ra] " + String(siderealObjectData.object_ra));
  Serial.println("[object_dec] " + String(siderealObjectData.object_dec));
  Serial.println("[object_az] " + String(siderealObjectData.object_az));
  Serial.println("[object_alt] " + String(siderealObjectData.object_alt));
  Serial.println("[object_r] " + String(siderealObjectData.object_r));
  Serial.println("[object_s] " + String(siderealObjectData.object_s));
}

void loop() {

  scenarioZero();

  delay(1000);
}
