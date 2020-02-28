//Receiver Code

#include <RadioHead.h>
#include <radio_config_Si4460.h>
#include <RHGenericDriver.h>
#include <RHGenericSPI.h>
#include <RHHardwareSPI.h>
#include <RHSPIDriver.h>
#include <RH_RF95.h>

// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

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
};

// make everybody look at the same 22-byte spot in memory
typedef union {
  PACKET data;
  byte raw[22];
} PACKET_UNION;

// global holder for all flight data
PACKET_UNION FLIGHT_DATA;

void setup() 
{   
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23);
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      for (int i=0; i<len; i++)
      {
        FLIGHT_DATA.raw[i] = buf[i];
      }
      
      Serial.print(F("Altitude: "));
      Serial.println(FLIGHT_DATA.data.ALTITUDE);
        
      Serial.print(F("Location: "));
      Serial.print(FLIGHT_DATA.data.GPS_LATITUDE, 4);
      Serial.print(F(", ")); 
      Serial.println(FLIGHT_DATA.data.GPS_LONGITUDE, 4);
      
      Serial.print(F("X: "));
      Serial.print(FLIGHT_DATA.data.X_RAW);
      Serial.print(F(" Y: "));
      Serial.print(FLIGHT_DATA.data.Y_RAW);
      Serial.print(F(" Z: "));
      Serial.println(FLIGHT_DATA.data.Z_RAW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
