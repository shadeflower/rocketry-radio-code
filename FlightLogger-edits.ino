// FlightLogger
// Compatible with FlightComputer 0.1.4

//#define USE_BME280 // 13% memory
//#define USE_SD // 33% memory
//#ifndef USE_SD
  #define USE_LORA // 20% memory
//#endif
//#define USE_GPS // 26% memory
//#ifdef USE_BME280
  //#define USE_TRIGGERS // 0% memory
// #endif
#define USE_LOGGING // 1% memory
//#define USE_LOGGING_VERBOSE // TBD% memory

#include <SPI.h>

// pin definitions
// unused digital: 3
// unused analog: A3, A6, A7
#define RFM95_INT       3
#define RFM95_RST       2
#define RFM95_CS        4
// #define TRIGGER_DROGUE  6
// #define TRIGGER_MAIN    7
// #define GPS_RX          9
// #define GPS_TX          8
#define SD_CS           10
#define XPIN            A0
#define YPIN            A1
#define ZPIN            A2
#define APIN            A3
#define BPIN            A4
#define CPIN            A5

// everything about the current flight data
struct PACKET {
  char  DROGUE_DEPLOYED;// flag to see whether the drogue is deployed  (1 byte)
  char  MAIN_DEPLOYED;  // flag to see whether the main is deployed    (1 byte)
  int   ALTITUDE;       // m                                           (2 bytes)
  float GPS_LATITUDE;   // degrees                                     (4 bytes)
  float GPS_LONGITUDE;  // degrees                                     (4 bytes)
  long  TIME;           // milliseconds                                (4 bytes)
  int   X_RAW;          // ADC counts                                  (2 bytes)
  int   Y_RAW;          // ADC counts                                  (2 bytes)
  int   Z_RAW;          // ADC counts                                  (2 bytes)
  int   A_RAW;        
  int   B_RAW;
  int   C_RAW; 
};

// make everybody look at the same 22-byte spot in memory
typedef union {
  PACKET data;
  byte raw[22];
} PACKET_UNION;

// global holder for all flight data
PACKET_UNION FLIGHT_DATA;

//#ifdef USE_BME280
  //#include <Wire.h>
  
  // Adafruit libraries for BME280
  //#include <Adafruit_Sensor.h>
  //#include <Adafruit_BME280.h>
  
  // base pressure
  //#define SEA_LEVEL_PRESSURE_HPA 1013.25
  //#define BME_280_ADDRESS 0x77

  // altimeter object
  //Adafruit_BME280 bme; // I2C

  //int REF_ALTITUDE = -1; // m
  //int MAX_ALTITUDE = -1000; // m
//#endif

#ifdef USE_LORA
  // RadioHead library
  #include <RH_RF95.h>
 
  // LORA frequency
  #define RF95_FREQ 915.0

  // radio object
  RH_RF95 rf95(RFM95_CS, RFM95_INT);

  // packet timing
  long LAST_PACKET = 0;
  int LORA_INTERVAL = 5000;
#endif

//#ifdef USE_TRIGGERS
  // triggers
  //int DROGUE_ALTITUDE = 50; // m below maximum
  //int MAIN_ALTITUDE = 300; // m above reference (set at runtime)

  //#define TRIGGER_DURATION 2000
//#endif

//#ifdef USE_GPS
  // include
  //#include <Adafruit_GPS.h>
  //#include <SoftwareSerial.h>
  
  //SoftwareSerial softSerial(GPS_TX, GPS_RX);
  //Adafruit_GPS GPS(&softSerial);
//#endif

//#ifdef USE_SD
  // default Arduino SD library
  //#include <SD.h>
  
  // packet timing
  //long LAST_LOG = 0;
  //int SD_INTERVAL = 100;
//#endif

void setup() {

    // initialize flight data
    FLIGHT_DATA.data.DROGUE_DEPLOYED = 0;
    FLIGHT_DATA.data.MAIN_DEPLOYED   = 0;
    FLIGHT_DATA.data.ALTITUDE        = 0;
    FLIGHT_DATA.data.GPS_LATITUDE    = 0;
    FLIGHT_DATA.data.GPS_LONGITUDE   = 0;
    FLIGHT_DATA.data.TIME            = 0;
    FLIGHT_DATA.data.X_RAW           = 0;
    FLIGHT_DATA.data.Y_RAW           = 0;
    FLIGHT_DATA.data.Z_RAW           = 0;
    FLIGHT_DATA.data.A_RAW           = 0;
    FLIGHT_DATA.data.B_RAW           = 0;
    FLIGHT_DATA.data.C_RAW           = 0;


    #ifdef USE_LOGGING
      Serial.begin(9600);
      Serial.println(F("Booting!"));
    #endif
    
    // set CS pins as outputs
    pinMode(SD_CS, OUTPUT);
    pinMode(RFM95_CS, OUTPUT);
    
    // set CS pins to disable
    digitalWrite(SD_CS, HIGH);
    digitalWrite(RFM95_CS, HIGH);

    // setup functions for each component
    //setup_BME280();
    //setup_SD();
    setup_LORA();
    //setup_GPS();
    
    //#ifdef USE_TRIGGERS
      // set pins
      //pinMode(TRIGGER_DROGUE, OUTPUT);
      //digitalWrite(TRIGGER_DROGUE, LOW);
      
      //pinMode(TRIGGER_MAIN, OUTPUT);
      //digitalWrite(TRIGGER_MAIN, LOW);
    //#endif

    #ifdef USE_LOGGING
      Serial.println(F("OK!"));
    #endif
}

void loop() {

    //#ifdef USE_BME280
      // in meters
      //FLIGHT_DATA.data.ALTITUDE = (int)bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);

      // on first pass through set base altitude
      //if(REF_ALTITUDE == -1) {
        //REF_ALTITUDE = FLIGHT_DATA.data.ALTITUDE;
      //}

      // set maximum altitude
      //if(FLIGHT_DATA.data.ALTITUDE > MAX_ALTITUDE) {
        //MAX_ALTITUDE = FLIGHT_DATA.data.ALTITUDE;
      //}

      //#ifdef USE_LOGGING_VERBOSE
        //Serial.print(F("Altitude: "));
        //Serial.println(FLIGHT_DATA.data.ALTITUDE);
      //#endif
    //#endif

    //#ifdef USE_GPS
      // read data from the GPS in the 'main loop'
      //char c = GPS.read();

      //#ifdef USE_LOGGING_VERBOSE
        //if (c) Serial.print(c);
      //#endif
    
      // if a sentence is received, we can check the checksum, parse it...
      //if (GPS.newNMEAreceived()) {
        //#ifdef USE_LOGGING
          //Serial.println(F("Got a GPS packet!"));
        //#endif
        
        //if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
          //return;  // we can fail to parse a sentence in which case we should just wait for another
                  
        //#ifdef USE_LOGGING
          //Serial.println(F("Parsed a GPS packet!"));
          //Serial.print(F("Location: "));
          //Serial.print(FLIGHT_DATA.data.GPS_LATITUDE, 4);
          //Serial.print(F(", ")); 
          //Serial.println(FLIGHT_DATA.data.GPS_LONGITUDE, 4);
        //#endif
      //}

      //FLIGHT_DATA.data.GPS_LATITUDE = GPS.latitudeDegrees;
      //FLIGHT_DATA.data.GPS_LONGITUDE = GPS.longitudeDegrees;

      //#ifdef USE_LOGGING_VERBOSE
        //Serial.print(F("Location: "));
        //Serial.print(FLIGHT_DATA.data.GPS_LATITUDE, 4);
        //Serial.print(F(", ")); 
        //Serial.println(FLIGHT_DATA.data.GPS_LONGITUDE, 4);
      //#endif
    //#endif

    //#ifdef USE_TRIGGERS
      // drogue chute
      //if(FLIGHT_DATA.data.ALTITUDE < (MAX_ALTITUDE - DROGUE_ALTITUDE) && !FLIGHT_DATA.data.DROGUE_DEPLOYED) {

        // move to async at some point
        //digitalWrite(TRIGGER_DROGUE, HIGH);
        //delay(TRIGGER_DURATION);
        //digitalWrite(TRIGGER_DROGUE, LOW);

        // mark drogue as deployed
        //FLIGHT_DATA.data.DROGUE_DEPLOYED = 1;
        
        //#ifdef USE_LOGGING
          //Serial.println(F("Drogue deployed!"));
        //#endif
      //}

      // main chute
      //if(FLIGHT_DATA.data.ALTITUDE < (REF_ALTITUDE + MAIN_ALTITUDE) && !FLIGHT_DATA.data.MAIN_DEPLOYED) {
        //if(FLIGHT_DATA.data.DROGUE_DEPLOYED) {
   
          // move to async at some point
          //digitalWrite(TRIGGER_MAIN, HIGH);
          //delay(TRIGGER_DURATION);
          //digitalWrite(TRIGGER_MAIN, LOW);
          
          // mark main as deployed
          //FLIGHT_DATA.data.MAIN_DEPLOYED = 1;
          
          //#ifdef USE_LOGGING
            //Serial.println(F("Main deployed!"));
          //#endif
        //}
      //} 
    //#endif

    // 0-5 V -> 0-1023 counts; 1.5V = 0G, 3V = 16G, 3.0V = 614 counts, 1.5V = 307 counts
    // 0V = 0 counts
    // 1.65V = 338 counts
    // 3.3V = 675 counts

    // ~0.06V/g = ~60mV/g

    // Count examples
    // +1g     = 351
    // Neutral = 338
    // -1g     = 325
    
    FLIGHT_DATA.data.X_RAW = analogRead(XPIN);
    FLIGHT_DATA.data.Y_RAW = analogRead(YPIN);
    FLIGHT_DATA.data.Z_RAW = analogRead(ZPIN);
    FLIGHT_DATA.data.A_RAW = analogRead(APIN);
    FLIGHT_DATA.data.B_RAW = analogRead(BPIN);
    FLIGHT_DATA.data.C_RAW = analogRead(CPIN);


    // grab time for logging
    FLIGHT_DATA.data.TIME = millis();

    //#ifdef USE_LOGGING_VERBOSE
      //Serial.print(F("X: "));
      //Serial.print(FLIGHT_DATA.data.X_RAW);
      //Serial.print(F(" Y: "));
      //Serial.print(FLIGHT_DATA.data.Y_RAW);
      //Serial.print(F(" Z: "));
      //Serial.println(FLIGHT_DATA.data.Z_RAW);
    //#endif
    
    // print SD/radio packet to the serial port too
    #ifdef USE_LOGGING_VERBOSE
      Serial.println((char*)FLIGHT_DATA.raw);
    #endif

    // filtered at function level
    // write the current FLIGHT_DATA object
    //write_SD();
    send_LORA();
}

// separated function to keep local memory use low
//void setup_BME280() {

  //#ifdef USE_BME280
    // boot up altimeter
    //bool status = bme.begin(BME_280_ADDRESS);  
    //if (!status) {
      
       // #ifdef USE_LOGGING
         // Serial.println(BME_280_ADDRESS);
          //Serial.println(F("BME error."));
        //#endif
        
        //while (1);
    //}
  //#endif
//}

// separated function to keep local memory use low
//void setup_GPS() {

  //#ifdef USE_GPS
    // configure GPS serial
    //GPS.begin(9600);
    
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    
    // set the update rate
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

    //Serial.println(F("GPS initialized."));
 // #endif
//b}

// separated function to keep local memory use low
void setup_LORA() {

  #ifdef USE_LORA
    // set pins
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
   
    // manual reset
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
   
    while (!rf95.init()) {
  
      #ifdef USE_LOGGING
        Serial.println(F("LORA error."));
      #endif
      
      while (1);
    }
    
    if (!rf95.setFrequency(RF95_FREQ)) {
  
      #ifdef USE_LOGGING
        Serial.println(F("Frequency error."));
      #endif
      
      while (1);
    }
  #endif
}

// separated function to keep local memory use low
// pins already initiated
//void setup_SD() {

  //#ifdef USE_SD
    // boot up SD card
    //if (!SD.begin(SD_CS)) {
  
      //  #ifdef USE_LOGGING
        //  Serial.println(F("SD error."));
        //#endif
        
        //while (1);
   // }
//  #endif
//}

void send_LORA() {
    
  #ifdef USE_LORA
    if(millis() - LAST_PACKET > LORA_INTERVAL) {
      // send packet
      rf95.send(FLIGHT_DATA.raw, sizeof(FLIGHT_DATA.raw));
  
      // wait for packet to finish sending
      rf95.waitPacketSent();
      LAST_PACKET = millis();
    }
  #endif
}

//void write_SD() {
  
  //#ifdef USE_SD
    //if(millis() - LAST_LOG > SD_INTERVAL) {
      // write to file
      //File dataFile = SD.open(F("flt.log"), FILE_WRITE);
      
      // if the file is available, write to it:
      //if (dataFile) {
        //dataFile.write(FLIGHT_DATA.raw, 22);
        //dataFile.close();
        
        //LAST_LOG = millis();
      //}
      // if the file isn't open, error
      //else {
  
        //#ifdef USE_LOGGING
          //Serial.println(F("File error."));
      //  #endif
     // }
    //}
  //#endif
//}
