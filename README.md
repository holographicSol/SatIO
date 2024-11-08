#                                                        SatIO

---

                                                   "The GPS MAster"

                              A general purpose programmable satellite and inertial switch.

                                      SatIO is the system, a matrix is the program.

            Receives and Processes Transmissions from Satellites and makes the data available for calculations.

            Possible combinations example: 

    10=digit characters   15=lenght of double   3=doubles per function   10=functions per switch   20=switches  190=available functions
                                              (((10^15 * 3) * 10) * 20) ^ 190

        Currently there are over 200 different checks that can be performed using just several small primitive functions and
          currently each matrix activation/deactivaion can occur based on up to 10 different checks resulting true or false. 
                                  
                                        Wiring For ESP32-2432S028 development board (CYD)
              
                           WTGPS300P TX -> CYD io22 remapped as RXD (already remapped in setup funtion)
                     ATMEGA2560 RX1 (pin 19) -> CYD io27 remapped as TXD (already remapped in setup function)

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

                            The IO is virtualized as well as physical (requires SatIO Port Controller)

            Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
                of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)

                     ToDo: RTC + IO expansion + black rubber case with waterproof IO ports, and potentially a server hosting
                                                             an RSS Feed.                       
  

---

![plot](./images/vlcsnap-2024-11-08-11h00m47s505.png)

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
