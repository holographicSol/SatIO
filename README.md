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
