# SatIO
Programmable Satellite &amp; Inertial Navigation IO For The CYD (Cheap Yellow Display)


                                           SATIO - Written by Benjamin Jack Cullen.

                                  A general purpose programmable satellite and inertial switch. 

                Receives and Processes Transmissions from Satellites and makes the data available for calculations.

                Possible combinations example: 100 checks ^ 10 functions = 100,000,000,000,000,000,000 combinations.
                                                                           100 Quintillion.
            Currently there are over 200 different checks that can be performed using just several small primitive functions and
             currently each relays activation/deactivaion can occur based on up to 10 different checks resulting true or false. 
                                      
                                        Wiring For ESP32-2432S028 development board (CYD)
                  
                  WTGPS300P TX --> CYD io22 as RXD (requires implemented Serial1.setPins() to work on CYD)

                                                      SENTENCE $SATIO
                                                                                    
                      START Tag                Last Sat Time                    Converted Longitude        
                         |                   |               |                   |               |                  
                      $SATIO,000000000000.00,000000000000.00,00.00000000000000,00.00000000000000,*Z
                             |               |               |                 |                              
                               DatetimeStamp                  Converted Latitude                                 


                        Ultimately this system is being built as a unit to turn on/off multiplexed relays,
                     where potentially anything can be plugged in such as simple modules or pre-programmed MCU's, 
               making a foundation for other creative projects that may make use of such satellite and or inertial data.
               The idea is that each relay is controlled by a compound of logic (limited by memory), and the logic itself
               is programmable before and after flashing. Allowing for a reusable and general purpose system for any future
               projects requiring such data. more advanced calculations are intended, such as emphemeris, astronomical etc. now
               that the foundational data has been handled. Then finally when fitted with relays, this system can be a self
               contained unit, behaving as a switch for other devices. Robots and flying machines and everything in between.

               The relays are currently simulated while the system is being built and the simulation is designed to be easily
               relaceable by the actual relays themselves once the logic has been completed to a satisfactory degree and currently
               all logic required to activate and deactivate relays is in place.

               Requires using modified SiderealPlanets library (hopefully thats okay as the modifications allow calculating rise/set
               of potentially any celestial body as described in this paper: https://stjarnhimlen.se/comp/riset.html)
