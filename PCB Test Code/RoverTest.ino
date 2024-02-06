/*
Test for Equine Rover
Intended to be used with a LoRA Thing Plus expLoRaBLE
Tested Devices:
  1. Micro SD Card Reader
  2. GPS Sensor
  3. IMU
  4. LoRa Radio
  5. LCD Screen
*/
#include <LiquidCrystal.h>
#include <Wire.h>
#include <RadioLib.h>
#include <SD.h>
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include <SparkFun_ISM330DHCX.h>


SFE_UBLOX_GNSS myGNSS;

Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 2;

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

SX1262 radio = new Module(D36, D40, D44, D39, SPI1);

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 




void setup() {
  // put your setup code here, to run once:
  lcd.begin(16, 2); // initialize the lcd 
  //lcd.setBacklight(1);
  lcd.print("Receive Test");


  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("If you see nothing on LCD; Check Schematic/Code");
  delay(1000);
}

void loop() {
  int startFlag = 0;
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  { 
    startFlag = 1;
    lcd.setCursor(0,0);
    lcd.print("GPS ");
  }

  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    if (startFlag == 0) {
      lcd.setCursor(0,0);
      startFlag = 1;
      }
    lcd.print("SD ");
  }

  if( !myISM.begin() ){
    if (startFlag == 0) {
      lcd.setCursor(0,0);
      startFlag = 1;
      }
		lcd.print("IMU ");
	}
  int state = radio.begin(915.0, 250.0, 7, 5, 0x34, 20, 10, 0, false);
  if (state != RADIOLIB_ERR_NONE) {
    if (startFlag == 0) {
      lcd.setCursor(0,0);
      startFlag = 1;
      }
    lcd.print("Radio ");
  }
  if (startFlag == 0) {
    lcd.setCursor(0,0);
    lcd.print("All Working");
  }
  else {lcd.print("Failed       ");}

if (state == RADIOLIB_ERR_NONE) {
  lcd.setCursor(0,1);
  lcd.print("Transmitting...");
  byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
  int state = radio.startTransmit(byteArr, 8);
    if (state == RADIOLIB_ERR_NONE) {
                delay(100);
                radio.finishTransmit();
    }
}
} 
