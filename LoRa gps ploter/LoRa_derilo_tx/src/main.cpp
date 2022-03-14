/* LoRa derilo- peer to peer LoRa connection sending gps coordinates and shows them on screen
TX part. Using TTGO-T-Beam v1.1 .  by Aco */


#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include <SSD1306.h> 
#include <images.h>
#include <SparkFun_Ublox_Arduino_Library.h> 
#include <axp20x.h>
#include <MicroNMEA.h> 


// Pins
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  868E6


#define I2C_SDA         21
#define I2C_SCL         22
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
//


SFE_UBLOX_GPS myGPS;
int state = 0; // steps through states
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

HardwareSerial SerialGPS(1);

AXP20X_Class axp;


SSD1306 display(0x3c, 21, 22);
String rssi = "RSSI --";
String packSize = "--";
String packet;



String Latitude;
String Longitude;

String v = ","; 

float lat;
float lon;
unsigned int counter = 0;


struct LoRa_config
{
  long Frequency;
  int SpreadingFactor;
  long SignalBandwidth;
  int CodingRate4;
  bool enableCrc;
  bool invertIQ;
  int SyncWord;
  int PreambleLength;
};

void LoRa_setConfig(struct LoRa_config config)
{
  LoRa.setFrequency(config.Frequency);
  LoRa.setSpreadingFactor(config.SpreadingFactor);
  LoRa.setSignalBandwidth(config.SignalBandwidth);
  LoRa.setCodingRate4(config.CodingRate4);
  if (config.enableCrc)
    LoRa.enableCrc();
  else
    LoRa.disableCrc();
  if (config.invertIQ)
    LoRa.enableInvertIQ();
  else
    LoRa.disableInvertIQ();
  LoRa.setSyncWord(config.SyncWord);
  LoRa.setPreambleLength(config.PreambleLength);
}

// LoRa setup frq, SP, BW, coding rate, corection, invert chanels, SyncWord, PreambleLength

static LoRa_config txLoRa = {868000000, 12, 250000, 5, false, false, 0x8e, 8};

void LoRa_TxMode()
{
  LoRa_setConfig(txLoRa);
  LoRa.idle();
}



boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}

void setup() {
  pinMode(16,OUTPUT);
  pinMode(14,OUTPUT);
  
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("LoRa Sender Test");

  Wire.begin(I2C_SDA, I2C_SCL); 

  
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
        Serial.println("AXP192 Begin PASS");
    } else {
        Serial.println("AXP192 Begin FAIL");
    }
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // provides power to GPS backup battery
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // enables power to ESP32 on T-beam
  axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON); // I foresee similar benefit for restting T-watch 
                                                 // where ESP32 is on DCDC3 but remember to change I2C pins and GPS pins!
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("All comms started");
  delay(100);


  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("init ok");
  display.init();
  display.flipScreenVertically();  
  display.setFont(ArialMT_Plain_10);

  delay(1500);

  display.clear();
  display.drawFastImage(0, 0 ,logo_width, logo_height, logo_bits);
  display.display();

   
  delay(1500);

  display.clear();

  do {
    if (myGPS.begin(SerialGPS)) {
      Serial.println("Connected to GPS");
      myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("GPS serial connected, output set to NMEA");
      myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
      myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("Enabled/disabled NMEA sentences");
      break;
    }
    delay(1000);
  } while(1);

}  // endofsetup



void loop() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);


  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if(nmea.isValid() == true)
  {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();
    
    lat = latitude_mdeg ;
    lon = longitude_mdeg;
    
    lat = lat / 1000000; 
    lon = lon / 1000000;

   
  
    Longitude = String(lon, 6);
    Latitude  = String(lat, 6);

    Serial.print("Latitude (deg): ");
    Serial.println(Latitude);
    Serial.print("Longitude (deg): ");
    Serial.println(Longitude);
  }
  else
  {
    Serial.print("No Fix - ");
    Serial.print("Num. satellites: ");
    Serial.println(nmea.getNumSatellites());
  }

  delay(500); //Don't pound too hard on the I2C bus


  if (runEvery(5000)) { // repeat every 5000 millis

    Serial.println("Sending packet non-blocking: ");
     
    packet = Latitude;
    packet += v;
    packet += Longitude;
    Serial.println(packet);

    // send in async / non-blocking mode
    LoRa_TxMode();
    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket(true); // true = async / non-blocking mode
  }



 
 
  display.drawString(0, 0, "Location");
  display.drawString(0, 10, String(Latitude));
  display.drawString(55, 10, "N");
  display.drawString(0, 20, String(Longitude));
  display.drawString(55, 20, "E");
  display.drawString(0, 30, "Number of satellites");
  display.drawString(0, 40, String(nmea.getNumSatellites()));
  display.display();


  // send packet
  
    
  //delay(5000);                                      

}