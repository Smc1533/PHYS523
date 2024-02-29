#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include <RadioLib.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SparkFun_ISM330DHCX.h>

Sd2Card card;
SdVolume volume;
SdFile root;
File myFile;
const int chipSelect = 2;

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

SFE_UBLOX_GNSS_SERIAL myGNSS;
SX1262 radio = new Module(D36, D40, D44, D39, SPI1);

SparkFun_ISM330DHCX myISM; 
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 

void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  double latitude = ubxDataStruct->lat; // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000.0, 7);

  double longitude = ubxDataStruct->lon; // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(longitude / 10000000.0, 7);

  double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  Serial.print(F("  Height: "));
  Serial.print(altitude / 1000.0, 3);

  uint8_t fixType = ubxDataStruct->fixType; // Print the fix type
  Serial.print(F("  Fix: "));
  Serial.print(fixType);
  if (fixType == 0)
    Serial.print(F(" (None)"));
  else if (fixType == 1)
    Serial.print(F(" (Dead Reckoning)"));
  else if (fixType == 2)
    Serial.print(F(" (2D)"));
  else if (fixType == 3)
    Serial.print(F(" (3D)"));
  else if (fixType == 3)
    Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixType == 5)
    Serial.print(F(" (Time Only)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution
  Serial.print(F("  Carrier Solution: "));
  Serial.print(carrSoln);
  if (carrSoln == 0)
    Serial.print(F(" (None)"));
  else if (carrSoln == 1)
    Serial.print(F(" (Floating)"));
  else if (carrSoln == 2)
    Serial.print(F(" (Fixed)"));
  else
    Serial.print(F(" (UNKNOWN)"));

uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
  Serial.print(F("  Horizontal Accuracy Estimate: "));
  Serial.print(hAcc);
  Serial.print(F(" (mm)"));
uint32_t vAcc = ubxDataStruct->vAcc; // Print the horizontal accuracy estimate
  Serial.print(F("  Vertical Accuracy Estimate: "));
  Serial.print(vAcc);
  Serial.print(F(" (mm)"));
  Serial.println();
int32_t ECEFX = myGNSS.getHighResECEFX();
int8_t ECEFXHp = myGNSS.getHighResECEFXHp();
int32_t ECEFY = myGNSS.getHighResECEFY();
int8_t ECEFYHp = myGNSS.getHighResECEFYHp();
int32_t ECEFZ = myGNSS.getHighResECEFZ();
int8_t ECEFZHp = myGNSS.getHighResECEFZHp();
uint32_t accuracy = myGNSS.getPositionAccuracy();
double d_ECEFX;
double d_ECEFY;
double d_ECEFZ;
d_ECEFX = ((double)ECEFX) / 100.0; // Convert from cm to m
d_ECEFX += ((double)ECEFXHp) / 10000.0; // Now add the high resolution component ( mm * 10^-1 = m * 10^-4 )
d_ECEFY = ((double)ECEFY) / 100.0; // Convert from cm to m
d_ECEFY += ((double)ECEFYHp) / 10000.0; // Now add the high resolution component ( mm * 10^-1 = m * 10^-4 )
d_ECEFZ = ((double)ECEFZ) / 100.0; // Convert from cm to m
d_ECEFZ += ((double)ECEFZHp) / 10000.0; // Now add the high resolution component ( mm * 10^-1 = m * 10^-4 )
myFile = SD.open("test.txt", FILE_WRITE);
myISM.checkStatus();
myISM.getAccel(&accelData);
myISM.getGyro(&gyroData);
int time = millis();
char X[200];
sprintf(X, "%.8d %.8d %.8d %.8d %.8d %.8d %.8f %.8f %.8f %.8f %.8f %.8f %.8d %.8d", d_ECEFX, d_ECEFY,d_ECEFZ,hAcc,hAcc,vAcc,accelData.xData,accelData.yData,accelData.zData,gyroData.xData,gyroData.yData,gyroData.zData,time,carrSoln);
//myFile.println(X + " " + String(d_ECEFY) + " " + String(d_ECEFZ) + " " + String(hAcc) + " " + String(hAcc) + " " + String(vAcc) + " " + String(accelData.xData) + " " + String(accelData.yData) + " " + String(accelData.zData) + " " + String(gyroData.xData) + " " + String(gyroData.yData) + " " + String(gyroData.zData) + " " + String(millis()) + " "+ String(fixType));
myFile.println(X);
memset(X, 0, sizeof X);
myFile.close();
}
void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Wire.begin();
  Wire1.begin();
  Serial.begin(115200);
  Serial1.begin(38400); //ZED-F9 modules default to 38400 baud
  
  if( !myISM.begin(Wire1) ){
		Serial.println("IMU Did not begin.");
		//while(1);
	}
  myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
		delay(1);
	} 
  myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
	myISM.setAccelFullScale(ISM_4g); 

	// Set the output data rate and precision of the gyroscope
	myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
	myISM.setGyroFullScale(ISM_500dps); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	myISM.setGyroFilterLP1();
	myISM.setGyroLP1Bandwidth(ISM_MEDIUM);





  if (!SD.begin(chipSelect)) {
    Serial.println("SD initialization failed!");
    while (1);
  }
  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.println("X Y Z eX eY eZ aX aY aZ gX gY gZ t Fix");
  myFile.close();
  while (myGNSS.begin(Serial1) == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("u-blox GPS not detected. Please check wiring."));
  }
  Serial.println(F("u-blox module connected"));
  int state = radio.begin(915.0, 250.0, 7, 5, 0x34, 20, 10, 0, false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  myGNSS.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the UART1 port to output both NMEA and UBX messages
  myGNSS.setUART1Input(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX messages only - disable NMEA to reduce the load on the module

  myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible

  myGNSS.setNavigationFrequency(1); //Set output in Hz.

  // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);

  //myGNSS.setNMEAGPGGAcallbackPtr(&pushGPGGA); // Set up the callback for GPGGA

  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, 10); // Tell the module to output GGA every 10 seconds
  
  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1, 0); // Disable GLL to reduce the load on UART1
  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, 0); // Disable GSA to reduce the load on UART1
  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, 0); // Disable GSV to reduce the load on UART1
  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, 0); // Disable RMC to reduce the load on UART1
  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, 0); // Disable VTG to reduce the load on UART1

  myGNSS.setAutoPVTcallbackPtr(&printPVTdata);

}
void loop() {
  // put your main code here, to run repeatedly:
  myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
  myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.
   // Create a byte array to store the RTCM data
  int state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
  if (state == RADIOLIB_ERR_NONE) {
    int numBytes = radio.getPacketLength();
    if (numBytes !=0){
    uint8_t rtcData[numBytes];
    int state = radio.readData(rtcData, numBytes);
    if (state == RADIOLIB_ERR_NONE) {
    myGNSS.pushRawData(rtcData, numBytes);
      }
    }
  }
}
