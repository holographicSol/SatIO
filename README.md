# SatIO
Programmable Satellite &amp; Inertial Navigation IO For The CYD (Cheap Yellow Display)

---

    SATIO - Written by Benjamin Jack Cullen.

    A general purpose programmable satellite and inertial switch.
    Continuing project development here with PlatformIO.
    (Early development and migration to PlatformIO) 

---

Receives and Processes Transmissions from Satellites and makes the data available for calculations.

    Possible combinations example: 100 checks ^ 10 functions = 100,000,000,000,000,000,000 combinations.
    100 Quintillion.

---

Currently there are over 200 different checks that can be performed using just several small primitive
functions and currently each relays activation/deactivaion can occur based on up to 10 different checks
resulting true or false. 

---

Wiring For ESP32-2432S028 development board (CYD):

WTGPS300P TX --> CYD io22
ATMEGA2560 RX1 --> CYD io27 TXD

---

  
                                              SENTENCE $SATIO
                                                                            
              START Tag                Last Sat Time                    Converted Longitude        
                 |                   |               |                   |               |                  
              $SATIO,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                     |               |               |                 |                              
                       DatetimeStamp                  Converted Latitude                                 
  

---

![plot](./images/vlcsnap-2024-11-08-11h00m47s505.png)

---

---

![plot](./images/vlcsnap-2024-11-08-10h58m42s127.png)

---


Ultimately this system is being built as a unit to make IO high/low, to control other devices/microcontrollers.

---


The relays are currently simulated while the system is being built and the simulation is designed to be easily
relaceable by the actual relays themselves once the logic has been completed to a satisfactory degree and currently
all logic required to activate and deactivate relays is in place.

---
  
       Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
       of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)

---
