#                                              SatIO - Written by Benjamin Jack Cullen.

                                                   "The GPS MAster"

                              A general purpose programmable satellite and inertial switch.

                                      SatIO is the system, a matrix is the program.

            Receives and Processes Transmissions from Satellites and makes the data available for calculations.

            Possible combinations example: 

10=digit characters   15=lenght of double   3=doubles per function   10=functions per switch   20=switches  190=available functions
                                              (((10^15 * 3) * 10) * 20) ^ 190

        Currently there are over 200 different checks that can be performed using just several small primitive functions and
currently each matrix activation/deactivaion can occur based on up to 10 different checks per 20 switches resulting true or false per switch. 

                                  
                                Wiring For ESP32 keystudio dev module (with keystudio sheild V1.3)

                                      Serial Communication Between ESP32 & ATMEGA2560:
                                      ESP32 io25 (TXD) -> ATMEGA2560 Serial1 (RXD)
                                      ESP32 io26 (RXD) -> ATMEGA2560 Serial1 (TXD)

                                      Serial Communication Between ESP32 & WTGPS300P:
                                      ESP32 io27 (RXD) -> WTGPS300P (TXD) (5v)

                                      ESP32 i2C Multiplexing:
                                      ESP32 i2C          -> TCA9548A i2c Multiplexer: SDA, SCL (3.3v)
                                      TCA9548A SDA0 SCL0 -> RTC DS3231 SDA, SCL (5v)

                                      ESP32 Analog+Digital Multiplexing:
                                      ESP32 io4  -> CD74HC4067 Analog/Digital Multiplexer: SIG
                                      ESP32 io32 -> CD74HC4067 Analog/Digital Multiplexer: S0
                                      ESP32 io33 -> CD74HC4067 Analog/Digital Multiplexer: S1
                                      ESP32 io12 -> CD74HC4067 Analog/Digital Multiplexer: S2
                                      ESP32 io13 -> CD74HC4067 Analog/Digital Multiplexer: S3
                                      CD74HC4067 C0 -> Photo Resistor SIG
                                      CD74HC4067 C1 -> DHT11 SIG

                                      ESP32 SDCARD:
                                      ESP32 io5  -> HW-125 Micro SD Card Module CS (SS)
                                      ESP32 io23 -> HW-125 Micro SD Card Module DI (MOSI)
                                      ESP32 io19 -> HW-125 Micro SD Card Module DO (MISO)
                                      ESP32 io18 -> HW-125 Micro SD Card Module SCK (SCLK)

                                                        SENTENCE $SATIO
                                                                                
                        START Tag                Last Sat Time                    Converted Longitude        
                          |                   |               |                   |               |                  
                        $SATIO,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                              |               |               |                 |                              
                                DatetimeStamp                  Converted Latitude                                 


        Ultimately this system is being built as a unit to turn on/off IO/GPIO/relays and or send messages to other controllers,
                  where potentially anything can be plugged in such as simple modules or pre-programmed MCU's, 
            making a foundation for other creative projects that may make use of such satellite and or inertial data.
                The idea is that each matrix is a compound of logic (limited by memory), and the logic itself
            is programmable before and after flashing. Allowing for a reusable and general purpose system for any future
                                            projects requiring use of GPS data. 
                                        Robots and flying machines and automation.

                                         The IO is virtualized as well as physical! 

            Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
                of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)

                    Use case from a syncronized clock to riding the INS (roll, pitch, yaw) on a fine line to within a certain degree of
                    expected drift, if GPS data is stale or unavailable.

                       Note: This project is now temporarily headless while architecture & performance are the focus.
  

---

![plot](./Extras/images/vlcsnap-2024-11-08-11h00m47s505.png)

---

![plot](./Extras/images/vlcsnap-2024-11-08-10h58m42s127.png)

---

![plot](./Extras/images/DSC_0004_BURST20241108194742300_COVER.JPG)

---
  
       Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
       of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)

---
