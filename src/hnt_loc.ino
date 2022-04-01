/*
This is code provide to a HelTec LoraWan board
to send GPS data on Helium console.
Is not power optimized. 
*/

#include <ESP32_LoRaWAN.h>
#include "Arduino.h"

#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define MYPORT_TX 12
#define MYPORT_RX 13
SoftwareSerial myPort;
TinyGPS gps;


/////////////////////////////////////////////////////////////
////////////////////USE YOUR DATA HERE///////////////////////
/*license for Heltec ESP32 LoRaWan, quary your ChipID relevant license: http://resource.heltec.cn/search */
uint32_t  license[4] = {0x00000000, 0x00000000, 0x00000000, 0x00000000};
/* OTAA para*/
uint8_t DevEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //
uint8_t AppEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/* ABP para*/
uint8_t NwkSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t AppSKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint32_t DevAddr =  ( uint32_t )0x00000000;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 3*60*60000; //3 hours

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = false;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };


/* Application port */
uint8_t appPort = 2;

/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)
  4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 8;

/*LoraWan debug level, select in arduino IDE tools.
  None : print basic info.
  Freq : print Tx and Rx freq, DR info.
  Freq && DIO : print Tx and Rx freq, DR, DIO0 interrupt and DIO1 interrupt info.
  Freq && DIO && PW: print Tx and Rx freq, DR, DIO0 interrupt, DIO1 interrupt, MCU sleep and MCU wake info.
*/

uint8_t debugLevel = LoRaWAN_DEBUG_LEVEL;

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
bool test = false;
unsigned char lonlat[8];


static void prepareTxFrame( uint8_t port )
{
  appDataSize = 8;//AppDataSize max value is 64
  appData[0] = lonlat[0];
  appData[1] = lonlat[1];
  appData[2] = lonlat[2];
  appData[3] = lonlat[3];
  appData[4] = lonlat[4];
  appData[5] = lonlat[5];
  appData[6] = lonlat[6];
  appData[7] = lonlat[7];
  
}

void create_position_message(float lat, float lon){
   int unit_lon = int(lon);
   lonlat[0] = ((unsigned char)unit_lon);

   int cents_lon = int((lon - float(unit_lon)) * 100);
   lonlat[1] = ((unsigned char)cents_lon);

   int ten_thousandths_lon = int((lon - float(unit_lon) - (float(cents_lon)/100) )* 10000);
   lonlat[2] = ((unsigned char)iiilon);

   int millionths_lon = int((lon - float(unit_lon) - (float(cents_lon)/100) - (float(ten_thousandths_lon)/10000)) * 1000000);
   lonlat[3] = ((unsigned char)millionths_lon);

   int unit_lat = int(lat);
   lonlat[4] = ((unsigned char)unit_lat);
   
   int cents_lat = int((lat - float(unit_lat)) * 100);
   lonlat[5] = ((unsigned char)cents_lat);

   int ten_thousandths_lat = int((lat - float(unit_lat) - (float(cents_lat)/100) )* 10000);
   lonlat[6] = ((unsigned char)ten_thousandths_lat);

   int millionths_lat = int((lat - float(unit_lat) - (float(cents_lat)/100) - (float(ten_thousandths_lat)/10000)) * 1000000);
   lonlat[7] = ((unsigned char)millionths_lat);
}

void gps_test() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 5000;)
  {
    while (myPort.available())
    {
      char c = myPort.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    create_position_message(flat, flon);
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}

// Add your initialization code here
void setup()
{
  if (mcuStarted == 0)
  {
    LoRaWAN.displayMcuInit();
  }
  Serial.begin(115200);
  while (!Serial);
  SPI.begin(SCK, MISO, MOSI, SS);
  Mcu.init(SS, RST_LoRa, DIO0, DIO1, license);
  deviceState = DEVICE_STATE_INIT;


  myPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  Serial.println(TinyGPS::library_version());
}

// The loop function is called in an endless loop
void loop()
{

  if (test) {
    gps_test();
    delay(20 * 1000);
  }
  else {
    switch ( deviceState )
    {
      case DEVICE_STATE_INIT:
        { 
          LoRaWAN.init(loraWanClass, loraWanRegion);
          break;
        }
      case DEVICE_STATE_JOIN:
        {
          gps_test();
          LoRaWAN.displayJoining();
          LoRaWAN.join();
          deviceState = DEVICE_STATE_SEND;
          break;
        }
      case DEVICE_STATE_SEND:
        {
          gps_test();

          LoRaWAN.displaySending();
          prepareTxFrame( appPort );
          LoRaWAN.send(loraWanClass);
          deviceState = DEVICE_STATE_CYCLE;
          break;
        }
      case DEVICE_STATE_CYCLE:
        {
          // Schedule next packet transmission
          txDutyCycleTime = appTxDutyCycle + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
          LoRaWAN.cycle(txDutyCycleTime);
          deviceState = DEVICE_STATE_SLEEP;
          break;
        }
      case DEVICE_STATE_SLEEP:
        {
          LoRaWAN.displayAck();
          LoRaWAN.sleep(loraWanClass, debugLevel);
          break;
        }
      default:
        {
          deviceState = DEVICE_STATE_INIT;
          break;
        }
    }
  }
}