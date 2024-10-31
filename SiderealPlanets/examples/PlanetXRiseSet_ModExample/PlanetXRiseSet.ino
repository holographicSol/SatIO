/* Sidereal Planets Library - Sun functions
 * Version 1.2.0 - February 17, 2023
 * Example7_Planets
*/

#include <SiderealPlanets.h>

// Need the following define for SAMD processors
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

SiderealPlanets myAstro;

void setup() {
  Serial.begin(115200);
  delay(2000); //SAMD boards may need a long time to init SerialUSB
  Serial.println("Sidereal Planets Functions");
  myAstro.begin();

  // first set time and coordinate data
  myAstro.setLatLong(51.45259, -2.58745);
  myAstro.setTimeZone(1);
  // myAstro.rejectDST();
  myAstro.setGMTdate(2024,7,18);
  myAstro.setLocalTime(6,40,0);
  myAstro.setGMTtime(6,40,0);

  Serial.println("------------------------------------------------------");

  //--------------------------------------------------------------------------------------------------------------------
  // sun: RA and DEC
  Serial.println("Computing Sun Parameters");
  myAstro.doSun();
  Serial.print("Geo Ecliptic Long: "); myAstro.printDegMinSecs(myAstro.getEclipticLongitude()); Serial.println("");
  Serial.print("RA: "); myAstro.printDegMinSecs(myAstro.getRAdec()); Serial.println("");
  Serial.print("Dec: "); myAstro.printDegMinSecs(myAstro.getDeclinationDec()); Serial.println("");
  // convert RA and DEC to AZ and Alt.
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doSunRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getSunriseTime()));
  Serial.println("Set Time: " + String(myAstro.getSunsetTime()));
  //--------------------------------------------------------------------------------------------------------------------

  Serial.println("------------------------------------------------------");

  //--------------------------------------------------------------------------------------------------------------------
  // moon: RA and DEC
  Serial.println("Computing Moon Parameters");
  myAstro.doMoon();
  Serial.print("Geo Ecliptic Long: "); myAstro.printDegMinSecs(myAstro.getEclipticLongitude()); Serial.println("");
  Serial.print("RA: "); myAstro.printDegMinSecs(myAstro.getRAdec()); Serial.println("");
  Serial.print("Dec: "); myAstro.printDegMinSecs(myAstro.getDeclinationDec()); Serial.println("");
  // convert RA and DEC to AZ and Alt.
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));  
  // rise/set time
  myAstro.doMoonRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getMoonriseTime()));
  Serial.println("Set Time: " + String(myAstro.getMoonsetTime()));
  //--------------------------------------------------------------------------------------------------------------------

  Serial.println("------------------------------------------------------");

  //--------------------------------------------------------------------------------------------------------------------
  // mercury: RA and DEC
  Serial.println("Computing Mercury Parameters");
  myAstro.doMercury();
  Serial.print("Geo Ecliptic Long: "); myAstro.printDegMinSecs(myAstro.getEclipticLongitude()); Serial.println("");
  Serial.print("RA: "); myAstro.printDegMinSecs(myAstro.getRAdec()); Serial.println("");
  Serial.print("Dec: "); myAstro.printDegMinSecs(myAstro.getDeclinationDec()); Serial.println("");
  // convert RA and DEC to AZ and Alt.
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));
  // mod for any planet/star rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  //--------------------------------------------------------------------------------------------------------------------

  Serial.println("------------------------------------------------------");

  //--------------------------------------------------------------------------------------------------------------------
  // venus: RA and DEC
  Serial.println("Computing Venus Parameters");
  myAstro.doVenus();
  Serial.print("Geo Ecliptic Long: "); myAstro.printDegMinSecs(myAstro.getEclipticLongitude()); Serial.println("");
  Serial.print("RA: "); myAstro.printDegMinSecs(myAstro.getRAdec()); Serial.println("");
  Serial.print("Dec: "); myAstro.printDegMinSecs(myAstro.getDeclinationDec()); Serial.println("");
  // convert RA and DEC to AZ and Alt.
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));
  // mod for any planet/star rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  //--------------------------------------------------------------------------------------------------------------------

  Serial.println("------------------------------------------------------");

  //--------------------------------------------------------------------------------------------------------------------
  // mars: RA and DEC
  Serial.println("Computing Mars Parameters");
  myAstro.doMars();
  Serial.print("Geo Ecliptic Long: "); myAstro.printDegMinSecs(myAstro.getEclipticLongitude()); Serial.println("");
  Serial.print("RA: "); myAstro.printDegMinSecs(myAstro.getRAdec()); Serial.println("");
  Serial.print("Dec: "); myAstro.printDegMinSecs(myAstro.getDeclinationDec()); Serial.println("");
  // convert RA and DEC to AZ and Alt.
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));
  // mod for any planet/star rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  //--------------------------------------------------------------------------------------------------------------------

  //--------------------------------------------------------------------------------------------------------------------
  // jupiter: RA and DEC
  Serial.println("Computing Mars Parameters");
  myAstro.doJupiter();
  Serial.print("Geo Ecliptic Long: "); myAstro.printDegMinSecs(myAstro.getEclipticLongitude()); Serial.println("");
  Serial.print("RA: "); myAstro.printDegMinSecs(myAstro.getRAdec()); Serial.println("");
  Serial.print("Dec: "); myAstro.printDegMinSecs(myAstro.getDeclinationDec()); Serial.println("");
  // convert RA and DEC to AZ and Alt.
  myAstro.doRAdec2AltAz();
  Serial.println("Azimuth: " + String(myAstro.getAzimuth()));
  Serial.println("Altitude: " + String(myAstro.getAltitude()));
  // mod for any planet/star rise/set time
  myAstro.doXRiseSetTimes();
  Serial.println("Rise Time: " + String(myAstro.getRiseTime()));
  Serial.println("Set Time: " + String(myAstro.getSetTime()));
  //--------------------------------------------------------------------------------------------------------------------

  Serial.println("------------------------------------------------------");

}

void loop() {
  while(1); //Freeze
}