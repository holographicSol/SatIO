                                        SatIO - Written by Benjamin Jack Cullen.

                                                   "The GPS Master"

                          A general purpose programmable satellite, sensor and inertial platform.
        Supporting stacks (up to 10 functions per output pin) of logic across 20 output pins on the portcontroller.

                                     SatIO is the system, a matrix is the program.

            Design: Break out all the things and build I2C peripherals as required to orbit the ESP32/Central-MCU.

                                    
                                    Wiring For Keystudio ESP32 PLUS Development Board

                                          ESP32: 1st ATMEGA2560 with shield as Port Controller (not on multiplexer):
                                          ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                          ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                          ESP32: 2nd ATMEGA2560 with shield as Control Panel (not on multiplexer):
                                          ESP32: io25    -> ATMEGA2560: io22
                                          ESP32: I2C SDA -> ATMEGA2560: I2C SDA
                                          ESP32: I2C SCL -> ATMEGA2560: I2C SCL

                                          Other ESP32 i2C Devices (not on multiplexer):
                                          ESP32: SDA0 SCL0 -> DS3231 (RTC): SDA, SCL (5v)

                                          ESP32: WTGPS300P (5v) (for getting a downlink):
                                          ESP32: io27 RXD -> WTGPS300P: TXD
                                          ESP32: null TXD -> WTGPS300P: RXD

                                          ESP32 i2C: i2C Multiplexing (3.3v) (for peripherals):
                                          ESP32: i2C -> TCA9548A: SDA, SCL

                                          ESP32: Analog/Digital Multiplexing (3.3v) (for peripherals):
                                          ESP32: io4    -> CD74HC4067: SIG
                                          ESP32: io32   -> CD74HC4067: S0
                                          ESP32: io33   -> CD74HC4067: S1
                                          ESP32: io16   -> CD74HC4067: S2
                                          ESP32: io17   -> CD74HC4067: S3
                                          CD74HC4067 C0 -> DHT11: SIG

                                          ESP32 VSPI: SDCARD (5v) (for matrix and system data):
                                          ESP32: io5  -> HW-125: CS (SS)
                                          ESP32: io23 -> HW-125: DI (MOSI)
                                          ESP32: io19 -> HW-125: DO (MISO)
                                          ESP32: io18 -> HW-125: SCK (SCLK)

                                          ESP32 HSPI: SSD1351 OLED (5v) (for interfacing):
                                          ESP32: io14 -> SSD1351: SCL/SCLK
                                          ESP32: io12 -> SSD1351: MISO/DC
                                          ESP32: io13 -> SSD1351: SDA
                                          ESP32: io26 -> SSD1351: CS


                                                  SENTENCE $SATIO


                                                      System Uptime                    
                  Tag                  Last Sync      |                               Degrees Longitude        
                  |      yyyymmddhhmmss|yyyymmddhhmmss|s|hh.mm|hh.mm|                 |                 |                
                  $SATIO,00000000000000,00000000000000,0,00.00,00.00,00.00000000000000,00.00000000000000,*Z
                        |              |                |     |     |                 |                 |            
                        RTC Datetime                    |     |     Degrees Latitude                    Checksum            
                                                        |     Sun Set
                                                        Sun Rise


      Use case: Its a PLC, use your imagination. Automate all the things. Robots, flying machines, or to provide data
      to local LLM's over serial, the list goes on.
      
      Flexibility: The system is designed to be highly flexible, so that input/output/calculations of all kinds can
      be turned on/off.

      Port Controller: Port controller to turn pins high/low according to instructions received from master.

      UI: Allows programming matrix switch logic and tuning for individual use cases. Emphasis to importance, clarity,
      consistency.
      
      Summary: Over one quintillion possible combinations of stackable logic across 20 switches for a general purpose
      part, subsystem or standalone device.

      Whats to gain? From this project I intend to have reusable, general purpose parts, namely a programmable navigation
      system, control pad and port controller that I can use for other projects in the future. For now I imagine each part#
      will be an I2C device and some parts like SatIO will have both master and slave modes for flexibility across differnt
      project requirements.

-----

![plot](./Extras/images/DSC_0001_BURST20250312163521251_COVER_Doc.JPG)

-----

![plot](./Extras/images/DSC_0001_BURST20250312163521251_COVER.JPG)

-----

![plot](./Extras/images/DSC_0000_BURST20250312163643601.JPG)

-----

![plot](./Extras/images/UnidentifiedStudios.png)

-----

Current Hardware Setup (Semi-Modular):

[Master] SatIO
(Private I2C Slaves) Port Controller
(Private I2C Slaves) Control Pad
(SPI)                Display
(I2C)                Multiplexer
(I2C)                Extension
(A/D)                Multiplexer


Idea Hardware Setup (Fully Modular):

[Master] Master Module
(Global I2C Slaves)  Shared SatIO Module
(Global I2C Slaves)  Shared Matrix Module
(Global I2C Slaves)  Shared Port Controller Module
(Private I2C Slaves) Private Port Controller (GPIO exclusive to this module)
(Private I2C Slaves) Private Control Pad
(SPI)                Private Display
(I2C)                Private Multiplexer
(I2C)                Private Extension
(A/D)                Private Multiplexer

[Slave] Shared Matrix Module
(Global I2C Masters) Master Module
(Private I2C Slaves) Private Port Controller (shared GPIO module)
(SPI)                Private Display
(I2C)                Private Multiplexer
(I2C)                Private Extension
(A/D)                Private Multiplexer
(Note)               Can be more than 1

[Slave] Shared SatIO Module
(Global I2C Masters)  Master Module
(Private I2C Masters) Private Port Controller (shared GPIO module)
(UART)                Private GPS
(SPI)                 Private Display
(I2C)                 Private Multiplexer
(I2C)                 Private Extension
(A/D)                 Private Multiplexer
(Note)                Can be more than 1

[Slave] Shared Port Controller Module
(Global I2C Masters)  Master Module
(Private I2C Masters) Port Controller (shared GPIO module)
(SPI)                 Private Display
(I2C)                 Private Multiplexer
(I2C)                 Private Extension
(A/D)                 Private Multiplexer
(Note)                Can be more than 1



[Thoughts on semi/fully modular]

    The hardware platform remains the same and the software would be similar.
    
    [Semi-Modular]
    	Matrix size limited.
    	Can have more Port Controllers.
    
    [Fully Modular]
    	Can have more Matrix modules.
    	Can have more Port Controllers.
    	Every module can perform faster.


     [Taking the platform fully modular]

    [Problem]
    * Keyestudio ATMEGA2560 Shields are now mostly unavailable.
    * Each pair of ATMEGA2560 Dev boards and Shields cost around 20GBP+.
    * Building I2C extension bus every time a new module is required takes a lot of soldering.
    * Building equivalent shields also requires a lot of soldering (voltage lanes, ground lanes, etc.).
    
    [Solution]
    Begin fabrication of the current development platform.
    * Reduces required wires/wiring to build the platform each time a new module is required.
    * Reduces cost because money can instead be built on PCB fab and components.
    * Makes the platform more easy to work with overall.
    * Able to flash new firmware to the board (to both MCU's individually).
    
    [The platform (PCB)]

  	*Fabricate a development board from the current platform built around ESP32.
  	*The PBC should integrate both the ESP32 and ATMEGA250, and integrate some other soon non-modular components.
  	*This board provides a development platform with the high clock speeds and radio capabilities of ESP32, while
  	also utilizing an integrated ATMEGA250 for a large amount of GPIO.
  	Currently these boards are intended for use as standalone systems or modules that can work together, 1 as master
  	and the rest as slaves, all built on the same platform and each with the same compute power and GPIO capacity.
  
  	ESP32: Traced to Analogue digital multiplexer (Analogue digital multiplexer should have exposed channel headers).
  	       Traced to I2C multiplexer (I2C multiplexer should have exposed channel headers).
  	       Traced to RTC.
  	       Traced to Shared I2C extension bus.
  	       Exposed headers for everything else.
  
  	ATMEGA2560:
  	       Traced to Shared I2C extension bus.
  	       Exposed headers for everything else.
          
-----
