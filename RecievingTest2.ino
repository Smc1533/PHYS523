#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
#include <RadioLib.h>
SFE_UBLOX_GNSS_SERIAL myGNSS;
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
void setup() {
  // put your setup code here, to run once:
  delay(1000);
  
  Serial.begin(115200);
  Serial1.begin(38400); //ZED-F9 modules default to 38400 baud

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

    
      //Serial1.write(rtcData,numBytes);
      //ubx_send_rtcm3_i2c(numBytes,rtcData);
      //Wire.write(rtcData,numBytes);
    myGNSS.pushRawData(rtcData, numBytes);
}
    }}}
