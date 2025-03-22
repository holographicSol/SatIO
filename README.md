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


    Use case: Its a PLC, use your imagination. Automate all the things. Robots, flying machines, sensor drones
    or to provide data to local LLM's over serial, the list goes on.
    
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

    [ MATRIX SWITCH LOGIC ]

    Logic may require or not require values X,Y,Z.
    
    [Typical rules]
    0: no arguments.
    1: Is value greater than, less than or equal to X.
    2: Is value greater than X and less than Y.
    3: Is value in Z square range of X and Y. 

    [Special functions]
    None: Returns false, takes no further arguments.
    Enabled: Returns true, takes no further arguments.
    Overload: Returns bool, takes no further arguments.
    SwitchLink: Returns bool. Requires argument X as matrix switch number to be linked.
    SecondsTimer: Returns bool. Requires argumnent X as total time and Y as time on within X total time.


    [RTC]
    RTCTimeOver: Returns bool. Requires X.
    RTCTimeUnder: Returns bool. Requires X.
    RTCTimeEqual: Returns bool. Requires X.
    RTCTimeRange: Returns bool. Requires X.
    DaySunday: Returns bool, takes no further arguments.
    DayMonday: Returns bool, takes no further arguments.
    DayTuesday: Returns bool, takes no further arguments.
    DayWednesday: Returns bool, takes no further arguments.
    DayThursday: Returns bool, takes no further arguments.
    DayFriday: Returns bool, takes no further arguments.
    DaySaturday: Returns bool, takes no further arguments.
    DateDayX: Returns bool, Requires X.
    DateMonthX: Returns bool, Requires X.
    DateYearX: Returns bool, Requires X.


    [Coordinates (degrees)]
    DegLatOver: Returns bool, Requires X.
    DegLonOver: Returns bool, Requires X.
    DegLonUnder: Returns bool, Requires X.
    DegLatUnder: Returns bool, Requires X.
    DegLatEqual: Returns bool, Requires X.
    DegLonEqual: Returns bool, Requires X.
    DegLatRange: Returns bool, Requires X (latitude) to Y (latitude).
    DegLonRange: Returns bool, Requires X (longitude) to Y (longitude).
    DegLatLonRange: Returns bool, Requires X (latitude), Y (longitude) in Z (meters range).


    [UTC Time (GNGGA)]
    UTCTimeGNGGAOver: Returns bool, Requires X.
    UTCTimeGNGGAUnder: Returns bool, Requires X.
    UTCTimeGNGGAEqual: Returns bool, Requires X.
    UTCTimeGNGGARange: Returns bool, Requires X (UTCTimeGNGGA) to Y (UTCTimeGNGGA).


    [Positioning Status]
    PosStatusGNGGA: Returns bool, Requires X.
    SatCountOver: Returns bool, Requires X.
    SatCountUnder: Returns bool, Requires X.
    SatCountEqual: Returns bool, Requires X.
    SatCountRange: Returns bool, Requires X (SatCount) to Y (SatCount).


    [Hemisphere (GNGGA)]
    HemiGNGGANorth: Returns bool, takes no further arguments.
    HemiGNGGAEast: Returns bool, takes no further arguments.
    HemiGNGGASouth: Returns bool, takes no further arguments.
    HemiGNGGAWest: Returns bool, takes no further arguments.


    [GPS Precision Factor]
    GPSPrecisionOver: Returns bool, Requires X.
    GPSPrecisionUnder: Returns bool, Requires X.
    GPSPrecisionEqual: Returns bool, Requires X.
    GPSPrecisionRange: Returns bool, Requires X (GPSPrecision) to Y (GPSPrecision).


    [Altitude (above sea level)]
    AltGNGGAOver: Returns bool, Requires X.
    AltGNGGAUnder: Returns bool, Requires X.
    AltGNGGAEqual: Returns bool, Requires X.
    AltGNGGARange: Returns bool, Requires X (Alt) to Y (Alt).


    [UTC Time (GNRMC)]
    UTCTimeGNRMCOver: Returns bool, Requires X.
    UTCTimeGNRMCUnder: Returns bool, Requires X.
    UTCTimeGNRMCEqual: Returns bool, Requires X.
    UTCTimeGNRMCRange: Returns bool, Requires X.


    [Hemisphere (GNRMC)]
    HemiGNRMCNorth: Returns bool, takes no further arguments.
    HemiGNRMCEast: Returns bool, takes no further arguments.
    HemiGNRMCSouth: Returns bool, takes no further arguments.
    HemiGNRMCWest: Returns bool, takes no further arguments.


    [Ground Speed]
    GSpeedGNRMCOver: Returns bool, Requires X.
    GSpeedGNRMCUnder: Returns bool, Requires X.
    GSpeedGNRMCEqual: Returns bool, Requires X.
    GSpeedGNRMCRange: Returns bool, Requires X (GSpeed) to Y (GSpeed).


    [Ground Heading]
    HeadingGNRMCOver: Returns bool, Requires X.
    HeadingGNRMCUnder: Returns bool, Requires X.
    HeadingGNRMCEqual: Returns bool, Requires X.
    HeadingGNRMCRange: Returns bool, Requires X (Heading) to Y (Heading).


    [UTC Date (GNRMC)]
    UTCDateGNRMCOver: Returns bool, Requires X.
    UTCDateGNRMCUnder: Returns bool, Requires X.
    UTCDateGNRMCEqual: Returns bool, Requires X.
    UTCDateGNRMCRange: Returns bool, Requires X (UTCDateGNRMC) to Y (UTCDateGNRMC).


    [Positioning Status]
    PosStatusGNRMCA: Returns bool, takes no further arguments.
    PosStatusGNRMCV: Returns bool, takes no further arguments.


    [Mode Indication]
    ModeGNRMCA: Returns bool, takes no further arguments.
    ModeGNRMCD: Returns bool, takes no further arguments.
    ModeGNRMCE: Returns bool, takes no further arguments.
    ModeGNRMCN: Returns bool, takes no further arguments.


    [Pitch]
    PitchGPATTOver: Returns bool, Requires X.
    PitchGPATTUnder: Returns bool, Requires X.
    PitchGPATTEqual: Returns bool, Requires X.
    PitchGPATTRange: Returns bool, Requires X (PitchGPATT) to Y (PitchGPATT).


    [Roll]
    RollGPATTOver: Returns bool, Requires X.
    RollGPATTUnder: Returns bool, Requires X.
    RollGPATTEqual: Returns bool, Requires X.
    RollGPATTRange: Returns bool, Requires X (RollGPATTRange) to Y (RollGPATTRange).


    [Yaw]
    YawGPATTOver: Returns bool, Requires X.
    YawGPATTUnder: Returns bool, Requires X.
    YawGPATTEqual: Returns bool, Requires X.
    YawGPATTRange: Returns bool, Requires X (YawGPATT) to Y (YawGPATT).


    [GST Data]
    GSTDataGPATTOver: Returns bool, Requires X.
    GSTDataGPATTUnder: Returns bool, Requires X.
    GSTDataGPATTEqual: Returns bool, Requires X.
    GSTDataGPATTRange: Returns bool, Requires X (GSTDataGPATT) to Y (GSTDataGPATT).


    [Mileage]
    MileageGPATTOver: Returns bool, Requires X.
    MileageGPATTUnder: Returns bool, Requires X.
    MileageGPATTEqual: Returns bool, Requires X.
    MileageGPATTRange: Returns bool, Requires X (MileageGPATT) to Y (MileageGPATT).


    [Speed Num]
    SpeedNumGPATTOver: Returns bool, Requires X.
    SpeedNumGPATTUnder: Returns bool, Requires X.
    SpeedNumGPATTEqual: Returns bool, Requires X.
    SpeedNumGPATTRange: Returns bool, Requires X (SpeedNumGPATT) to Y (SpeedNumGPATT).


    [Line Flag]
    LFlagGPATTOver: Returns bool, Requires X.
    LFlagGPATTUnder: Returns bool, Requires X.
    LFlagGPATTEqual: Returns bool, Requires X.
    LFlagGPATTRange: Returns bool, Requires X (LFlagGPATT) to Y (LFlagGPATT).


    [INS]
    INSGPATTOver: Returns bool, Requires X.
    INSGPATTUnder: Returns bool, Requires X.
    INSGPATTEqual: Returns bool, Requires X.
    INSGPATTRange: Returns bool, Requires X (INSGPATT) to Y (INSGPATT).


    [Run State Flag]
    RSFlagGPATTOver: Returns bool, Requires X.
    RSFlagGPATTUnder: Returns bool, Requires X.
    RSFlagGPATTEqual: Returns bool, Requires X.
    RSFlagGPATTRange:


    [Static Flag]
    SFlagGPATTOver: Returns bool, Requires X.
    SFlagGPATTUnder: Returns bool, Requires X.
    SFlagGPATTEqual: Returns bool, Requires X.
    SFlagGPATTRange: Returns bool, Requires X (SFlagGPATT) to Y (SFlagGPATT).


    [Sun]
    SunAzRange: Returns bool, Requires X (SunAz) to Y (SunAz).
    SunAltRange: Returns bool, Requires X (SunAlt) to Y (SunAlt).
    DayTime: Returns bool, takes no further arguments.
    NightTime: Returns bool, takes no further arguments.
    Sunrise: Returns bool, Requires X.
    Sunset: Returns bool, Requires X.


    [Moon]
    MoonAzRange: 
    MoonAltRange:
    Moonrise: Returns bool, Requires X.
    Moonset: Returns bool, Requires X.
    MoonUp: Returns bool, takes no further arguments.
    MoonDown: Returns bool, takes no further arguments.
    MoonPhaseOver: Returns bool, Requires X.
    MoonPhaseUnder: Returns bool, Requires X.
    MoonPhaseEqual: Returns bool, Requires X.
    MoonPhaseRange: Returns bool, Requires X (MoonPhase) to Y (MoonPhase).


    [Mercury]
    MercuryAzRange: Returns bool, Requires X (MercuryAz) to Y (MercuryAz).
    MercuryAltRange: Returns bool, Requires X (MercuryAlt) to Y (MercuryAlt).
    MercuryRise: Returns bool, Requires X.
    MercurySet: Returns bool, Requires X.
    MercuryUp: Returns bool, takes no further arguments.
    MercuryDown: Returns bool, takes no further arguments.


    [Venus]
    VenusAzRange: Returns bool, Requires X (VenusAz) to Y (VenusAz).
    VenusAltRange: Returns bool, Requires X (VenusAlt) to Y (VenusAlt).
    VenusRise: Returns bool, Requires X.
    VenusSet: Returns bool, Requires X.
    VenusUp: Returns bool, takes no further arguments.
    VenusDown: Returns bool, takes no further arguments.


    [Mars]
    MarsAzRange: Returns bool, Requires X (MarsAz) to Y (MarsAz).
    MarsAltRange: Returns bool, Requires X (MarsAlt) to Y (MarsAlt).
    MarsRise: Returns bool, Requires X.
    MarsSet: Returns bool, Requires X.
    MarsUp: Returns bool, takes no further arguments.
    MarsDown: Returns bool, takes no further arguments.


    [Jupiter]
    JupiterAzRange: Returns bool, Requires X (JupiterAz) to Y (JupiterAz).
    JupiterAltRange: Returns bool, Requires X (JupiterAlt) to Y (JupiterAlt).
    JupiterRise: Returns bool, Requires X.
    JupiterSet: Returns bool, Requires X.
    JupiterUp: Returns bool, takes no further arguments.
    JupiterDown: Returns bool, takes no further arguments.


    [Saturn]
    SaturnAzRange: Returns bool, Requires X (SaturnAz) to Y (SaturnAz).
    SaturnAltRange: Returns bool, Requires X (SaturnAlt) to Y (SaturnAlt).
    SaturnRise: Returns bool, Requires X.
    SaturnSet: Returns bool, Requires X.
    SaturnUp: Returns bool, takes no further arguments.
    SaturnDown: Returns bool, takes no further arguments.


    [Uranus]
    UranusAzRange: Returns bool, Requires X (UranusAz) to Y (UranusAz).
    UranusAltRange: Returns bool, Requires X (UranusAlt) to Y (UranusAlt).
    UranusRise: Returns bool, Requires X.
    UranusSet: Returns bool, Requires X.
    UranusUp: Returns bool, takes no further arguments.
    UranusDown: Returns bool, takes no further arguments.


    [Neptune]
    NeptuneAzRange: Returns bool, Requires X (NeptuneAz) to Y (NeptuneAz).
    NeptuneAltRange: Returns bool, Requires X (NeptuneAlt) to Y (NeptuneAlt).
    NeptuneRise: Returns bool, Requires X.
    NeptuneSet: Returns bool, Requires X.
    NeptuneUp: Returns bool, takes no further arguments.
    NeptuneDown: Returns bool, takes no further arguments.


    [Checksums]
    GNGGAValidCS: Returns bool, takes no further arguments.
    GNRMCValidCS: Returns bool, takes no further arguments.
    GPATTValidCS: Returns bool, takes no further arguments.


    [Check Data]
    GNGGAValidCD: Returns bool, takes no further arguments.
    GNRMCValidCD: Returns bool, takes no further arguments.
    GPATTValidCD: Returns bool, takes no further arguments.
    

    [Humidity]
    DHT11H0Over: Returns bool, Requires X.
    DHT11H0Under: Returns bool, Requires X.
    DHT11H0Equal: Returns bool, Requires X.
    DHT11H0Range: Returns bool, Requires X (DHT11H0) to Y (DHT11H0).


    [Celsius]
    DHT11C0Over: Returns bool, Requires X.
    DHT11C0Under: Returns bool, Requires X.
    DHT11C0Equal: Returns bool, Requires X.
    DHT11C0Range: Returns bool, Requires X (DHT11C0) to Y (DHT11C0).


    [Fahrenheit]
    DHT11F0Over: Returns bool, Requires X.
    DHT11F0Under: Returns bool, Requires X.
    DHT11F0Equal: Returns bool, Requires X.
    DHT11F0Range: Returns bool, Requires X (DHT11F0) to Y (DHT11F0).


    [Heat Index Celsius]
    DHT11HIC0Over: Returns bool, Requires X.
    DHT11HIC0Under: Returns bool, Requires X.
    DHT11HIC0Equal: Returns bool, Requires X.
    DHT11HIC0Range: Returns bool, Requires X (DHT11HIC0) to Y (DHT11HIC0).


    [Heat Index Fahrenheit]
    DHT11HIF0Over: Returns bool, Requires X.
    DHT11HIF0Under: Returns bool, Requires X.
    DHT11HIF0Equal: Returns bool, Requires X.
    DHT11HIF0Range: Returns bool, Requires X (DHT11HIF0) to Y (DHT11HIF0).


    [Sensor 0]
    Sensor0Over: Returns bool, Requires X.
    Sensor0Under: Returns bool, Requires X.
    Sensor0Equal: Returns bool, Requires X.
    Sensor0Range: Returns bool, Requires X (Sensor0) to Y (Sensor0).


    [Sensor 1]
    Sensor1Over: Returns bool, Requires X.
    Sensor1Under: Returns bool, Requires X.
    Sensor1Equal: Returns bool, Requires X.
    Sensor1Range: Returns bool, Requires X (Sensor1) to Y (Sensor1).


    [Sensor 2]
    Sensor2Over: Returns bool, Requires X.
    Sensor2Under: Returns bool, Requires X.
    Sensor2Equal: Returns bool, Requires X.
    Sensor2Range: Returns bool, Requires X (Sensor2) to Y (Sensor2).


    [Sensor 3]
    Sensor3Over: Returns bool, Requires X.
    Sensor3Under: Returns bool, Requires X.
    Sensor3Equal: Returns bool, Requires X.
    Sensor3Range: Returns bool, Requires X (Sensor3) to Y (Sensor3).


    [Sensor 4]
    Sensor4Over: Returns bool, Requires X.
    Sensor4Under: Returns bool, Requires X.
    Sensor4Equal: Returns bool, Requires X.
    Sensor4Range: Returns bool, Requires X (Sensor4) to Y (Sensor4).


    [Sensor 5]
    Sensor5Over: Returns bool, Requires X.
    Sensor5Under: Returns bool, Requires X.
    Sensor5Equal: Returns bool, Requires X.
    Sensor5Range: Returns bool, Requires X (Sensor5) to Y (Sensor5).


    [Sensor 6]
    Sensor6Over: Returns bool, Requires X.
    Sensor6Under: Returns bool, Requires X.
    Sensor6Equal: Returns bool, Requires X.
    Sensor6Range: Returns bool, Requires X (Sensor6) to Y (Sensor6).


    [Sensor 7]
    Sensor7Over: Returns bool, Requires X.
    Sensor7Under: Returns bool, Requires X.
    Sensor7Equal: Returns bool, Requires X.
    Sensor7Range: Returns bool, Requires X (Sensor7) to Y (Sensor7).


    [Sensor 8]
    Sensor8Over: Returns bool, Requires X.
    Sensor8Under: Returns bool, Requires X.
    Sensor8Equal: Returns bool, Requires X.
    Sensor8Range: Returns bool, Requires X (Sensor8) to Y (Sensor8).


    [Sensor 9]
    Sensor9Over: Returns bool, Requires X.
    Sensor9Under: Returns bool, Requires X.
    Sensor9Equal: Returns bool, Requires X.
    Sensor9Range: Returns bool, Requires X (Sensor9) to Y (Sensor9).


    [Sensor 10]
    Sensor10Over: Returns bool, Requires X.
    Sensor10Under: Returns bool, Requires X.
    Sensor10Equal: Returns bool, Requires X.
    Sensor10Range: Returns bool, Requires X (Sensor10) to Y (Sensor10).


    [Sensor 11]
    Sensor11Over: Returns bool, Requires X.
    Sensor11Under: Returns bool, Requires X.
    Sensor11Equal: Returns bool, Requires X.
    Sensor11Range: Returns bool, Requires X (Sensor11) to Y (Sensor11).


    [Sensor 12]
    Sensor12Over: Returns bool, Requires X.
    Sensor12Under: Returns bool, Requires X.
    Sensor12Equal: Returns bool, Requires X.
    Sensor12Range: Returns bool, Requires X (Sensor12) to Y (Sensor12).


    [Sensor 13]
    Sensor13Over: Returns bool, Requires X.
    Sensor13Under: Returns bool, Requires X.
    Sensor13Equal: Returns bool, Requires X.
    Sensor13Range: Returns bool, Requires X (Sensor13) to Y (Sensor13).


    [Sensor 14]
    Sensor14Over: Returns bool, Requires X.
    Sensor14Under: Returns bool, Requires X.
    Sensor14Equal: Returns bool, Requires X.
    Sensor14Range: Returns bool, Requires X (Sensor14) to Y (Sensor14).


    [Sensor 15]
    Sensor15Over: Returns bool, Requires X.
    Sensor15Under: Returns bool, Requires X.
    Sensor15Equal: Returns bool, Requires X.
    Sensor15Range: Returns bool, Requires X (Sensor15) to Y (Sensor15).


-----

    [SERIAL]

    All data sentences are NMEA sentence inspired, complete with checksums and can be enabled/disabled individually.

    Output GNGGA.

    Output GNRMC.

    Output GPATT.

    Output SATIO.

    Output MATRIX.

    Output SENSORS.

    Output Sun: Enable/disable. $SUN,RA,DEC,AZ,ALT,RISE,SET,*CHECKSUM

    Output Moon: Enable/disable. $MOON,RA,DEC,AZ,ALT,RISE,SET,PHASE,LUMINANCE*CHECKSUM

    Output Mercury: Enable/disable. $MERCURY,RA,DEC,AZ,ALT,RISE,SET,HELIO_ECLIPTIC_LAT,HELIO_ECLIPTIC_LON,RADIUS_VECTOR,DISTANCE,ECLIPTIC_LAT,ECLIPTIC_LON*CHECKSUM

    Output Venus: Enable/disable. $VENUS,RA,DEC,AZ,ALT,RISE,SET,HELIO_ECLIPTIC_LAT,HELIO_ECLIPTIC_LON,RADIUS_VECTOR,DISTANCE,ECLIPTIC_LAT,ECLIPTIC_LON*CHECKSUM

    Output Mars: Enable/disable. $MARS,RA,DEC,AZ,ALT,RISE,SET,HELIO_ECLIPTIC_LAT,HELIO_ECLIPTIC_LON,RADIUS_VECTOR,DISTANCE,ECLIPTIC_LAT,ECLIPTIC_LON*CHECKSUM

    Output Jupiter: Enable/disable. $JUPITER,RA,DEC,AZ,ALT,RISE,SET,HELIO_ECLIPTIC_LAT,HELIO_ECLIPTIC_LON,RADIUS_VECTOR,DISTANCE,ECLIPTIC_LAT,ECLIPTIC_LON*CHECKSUM

    Output Saturn: Enable/disable. $SATURN,RA,DEC,AZ,ALT,RISE,SET,HELIO_ECLIPTIC_LAT,HELIO_ECLIPTIC_LON,RADIUS_VECTOR,DISTANCE,ECLIPTIC_LAT,ECLIPTIC_LON*CHECKSUM

    Output Uranus: Enable/disable. $URANUS,RA,DEC,AZ,ALT,RISE,SET,HELIO_ECLIPTIC_LAT,HELIO_ECLIPTIC_LON,RADIUS_VECTOR,DISTANCE,ECLIPTIC_LAT,ECLIPTIC_LON*CHECKSUM

    Output Neptune: Enable/disable. $NEPTUNE,RA,DEC,AZ,ALT,RISE,SET,HELIO_ECLIPTIC_LAT,HELIO_ECLIPTIC_LON,RADIUS_VECTOR,DISTANCE,ECLIPTIC_LAT,ECLIPTIC_LON*CHECKSUM

-----


    [PERFORMANCE]

    Built in functions can be enabled/disabled for application specific tuning.

    SatIO: Enable/disable. Converts UTC from GPS to local time.
                           Converts absolute latitude and longitude from GPS to degrees.
                           Syncronizes RTC with local time from GPS UTC.
                           Records last RTC syncronization datetime.
    
    GNGGA: Enable/disable. Parse GNGGA data from GPS module.

    GNRMC: Enable/disable. Parse GNRMC data from GPS module.

    GPATT: Enable/disable. Parse GPATT data from GPS module.

    Matrix: Enable/disable. Enable/disable programmable matrix logic function.

    Port Controller: Enable/sisable. Enable/disable Port Controller IO.

    Output SatIO: Enable/disable. Print SatIO sentence over serial.

    Output GNGGA: Enable/disable. Print GNGGA sentence over serial.

    Output GNRMC: Enable/disable. Print GNRMC sentence over serial.

    Output GPATT: Enable/disable. Print GPATT sentence over serial.

    Output Matrix: Enable/disable. Print Matrix sentence over serial.

    Output Sensors: Enable/disable. Print Sensors sentence over serial.

    Track Sun: Enable/disable.

    Track Moon: Enable/disable.

    Track Mercury: Enable/disable.

    Track Venus: Enable/disable.

    Track Mars: Enable/disable.

    Track Jupiter: Enable/disable.

    Track Saturn: Enable/disable.

    Track Uranus: Enable/disable.

    Track Neptune: Enable/disable.


-----


    [Current Hardware Setup (Semi-Modular)]
    
    [Master] SatIO
    (Private I2C Slaves) Port Controller
    (Private I2C Slaves) Control Pad
    (SPI)                Display
    (I2C)                Multiplexer
    (I2C)                Extension
    (A/D)                Multiplexer
    
    
    [Idea Hardware Setup (Fully Modular)]
    
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
