#include <Wire.h> //Needed for I2C to GNSS
#include <RadioLib.h>
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;
SX1262 radio = new Module(D36, D40, D44, D39, SPI1);
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

  Serial.println();    
}
void printRTCMdata1005(RTCM_1005_data_t *rtcmData1005)
{
  double x = rtcmData1005->AntennaReferencePointECEFX;
  x /= 10000.0; // Convert to m
  double y = rtcmData1005->AntennaReferencePointECEFY;
  y /= 10000.0; // Convert to m
  double z = rtcmData1005->AntennaReferencePointECEFZ;
  z /= 10000.0; // Convert to m

  Serial.print(F("NTRIP Server RTCM 1005:  ARP ECEF-X: "));
  Serial.print(x, 4); // 4 decimal places
  Serial.print(F("  Y: "));
  Serial.print(y, 4); // 4 decimal places
  Serial.print(F("  Z: "));
  Serial.println(z, 4); // 4 decimal places
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printRTCMdata1006 will be called when new RTCM 1006 data has been parsed from pushRawData
// See u-blox_structs.h for the full definition of RTCM_1006_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRTCM1006InputcallbackPtr
//        /                 _____  This _must_ be RTCM_1006_data_t
//        |                /                   _____ You can use any name you like for the struct
//        |                |                  /
//        |                |                  |
void printRTCMdata1006(RTCM_1006_data_t *rtcmData1006)
{
  double x = rtcmData1006->AntennaReferencePointECEFX;
  x /= 10000.0; // Convert to m
  double y = rtcmData1006->AntennaReferencePointECEFY;
  y /= 10000.0; // Convert to m
  double z = rtcmData1006->AntennaReferencePointECEFZ;
  z /= 10000.0; // Convert to m
  double h = rtcmData1006->AntennaHeight;
  h /= 10000.0; // Convert to m

  Serial.print(F("NTRIP Server RTCM 1006:  ARP ECEF-X: "));
  Serial.print(x, 4); // 4 decimal places
  Serial.print(F("  Y: "));
  Serial.print(y, 4); // 4 decimal places
  Serial.print(F("  Z: "));
  Serial.print(z, 4); // 4 decimal places
  Serial.print(F("  Height: "));
  Serial.println(h, 4); // 4 decimal places
}

void setup()
{
  delay(1000);
  Wire.begin();
  Serial.begin(38400);
  Serial1.begin(38400);
  
  Serial.println(F("SparkFun u-blox GNSS example"));
  int state = radio.begin(915.0, 250.0, 7, 5, 0x34, 20, 10, 0, false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  //myGNSS.enableDebugging(Serial); // Uncomment this line to enable debug messages

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Check that this platform supports 64-bit (8 byte) double
  if (sizeof(double) < 8)
  {
    Serial.println(F("Warning! Your platform does not support 64-bit double."));
    Serial.println(F("The latitude and longitude will be inaccurate."));
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);
  myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
  myGNSS.setVal8(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 10);
  //myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED);
  
  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS.setNavigationFrequency(1); //Set output to 5 times a second. Change the RAM layer only.

  byte rate;
  if (myGNSS.getNavigationFrequency(&rate)) //Get the update rate of this module
  {
    Serial.print("Current update rate: ");
    Serial.println(rate);
  }
  else
  {
    Serial.println(F("getNavigationFrequency failed!"));
  }
  myGNSS.setAutoPVTcallbackPtr(&printPVTdata);
  myGNSS.setRTCM1005InputcallbackPtr(&printRTCMdata1005); // Set up a callback to print the RTCM 1005 Antenna Reference Position from the correction data
  myGNSS.setRTCM1006InputcallbackPtr(&printRTCMdata1006);

  
}



void loop()
{
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

    
      Serial1.write(rtcData,numBytes);
      //ubx_send_rtcm3_i2c(numBytes,rtcData);
      //Wire.write(rtcData,numBytes);
    myGNSS.pushRawData(rtcData, numBytes);
    //Serial.println("RTCM Data Recieved");
    
    
    
  

    
    
 
    
  
  
  
  /*
  int state = radio.receive(rtcData,RTCM_DATA_SIZE); // Receive data into the byte array
  myGNSS.pushRawData(rtcData, RTCM_DATA_SIZE,false);
  
    
  // Data received successfully
  // Push the RTCM data to the GNSS module
  
    
  
  //Query module. The module only responds when a new position is available.
  
    // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getElipsoid: returns the height above ellipsoid as an int32_t in mm
    // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
    // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
    // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
    // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

    // First, let's collect the position data
    int32_t latitude = myGNSS.getHighResLatitude();
    int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
    int32_t longitude = myGNSS.getHighResLongitude();
    int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
    int32_t ellipsoid = myGNSS.getElipsoid();
    int8_t ellipsoidHp = myGNSS.getElipsoidHp();
    int32_t msl = myGNSS.getMeanSeaLevel();
    int8_t mslHp = myGNSS.getMeanSeaLevelHp();
    uint32_t accuracy = myGNSS.getHorizontalAccuracy();

    // Defines storage for the lat and lon as double
    double d_lat; // latitude
    double d_lon; // longitude

    // Assemble the high precision latitude and longitude
    d_lat = ((double)latitude) / 10000000.0; // Convert latitude from degrees * 10^-7 to degrees
    d_lat += ((double)latitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )
    d_lon = ((double)longitude) / 10000000.0; // Convert longitude from degrees * 10^-7 to degrees
    d_lon += ((double)longitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )

   // Print the lat and lon
    Serial.print("Lat (deg): ");
    Serial.print(d_lat, 9);
    Serial.print(", Lon (deg): ");
    Serial.print(d_lon, 9);

    // Now define float storage for the heights and accuracy
    float f_ellipsoid;
    float f_msl;
    float f_accuracy;

    // Calculate the height above ellipsoid in mm * 10^-1
    f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
    // Now convert to m
    f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

    // Calculate the height above mean sea level in mm * 10^-1
    f_msl = (msl * 10) + mslHp;
    // Now convert to m
    f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

    // Convert the horizontal accuracy (mm * 10^-1) to a float
    f_accuracy = accuracy;
    // Now convert to m
    f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

    // Finally, do the printing
    Serial.print(", Ellipsoid (m): ");
    Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

    Serial.print(", Mean Sea Level (m): ");
    Serial.print(f_msl, 4); // Print the mean sea level with 4 decimal places

    Serial.print(", Accuracy (m): ");
    Serial.println(f_accuracy, 4); // Print the accuracy with 4 decimal places
  */
    }
  }
  }
}

