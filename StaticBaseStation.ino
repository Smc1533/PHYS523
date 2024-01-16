#include <Wire.h> //Needed for I2C to GNSS
#include <RadioLib.h>
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS;


SX1262 radio = new Module(D36, D40, D44, D39, SPI1);
void setup()
{
  delay(1000);
  
  Serial.begin(38400);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("u-blox Base Station example"));
  int state = radio.begin(915.0, 250.0, 7, 5, 0x34, 20, 10, 0, false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
#ifdef SERIAL_OUTPUT
  Serial1.begin(115200);
  myGNSS.setRTCMOutputPort(Serial1);
#endif

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Ensure RTCM3 is enabled
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
  myGNSS.setNavigationFrequency(1);
  while (Serial.available()) Serial.read(); //Clear any latent chars in serial buffer
  
  bool response = myGNSS.newCfgValset(); // Create a new Configuration Item VALSET message
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C, 0);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 0);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 0);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GST_I2C, 0);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C, 0);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C, 0);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 0);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_I2C, 0);
  response &= myGNSS.sendCfgValset(); // Send the configuration VALSET
  
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); //Enable message 1005 to output through I2C port, message every second
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C, 1);
  response &= myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10); // Enable message 1230 every 10 seconds
  response &= myGNSS.sendCfgValset(); // Send the VALSET

  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("RTCM failed to enable. Are you sure you have an ZED-F9P?"));
    while (1); //Freeze
  }

  response &= myGNSS.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10, false, VAL_LAYER_RAM);
  if (response == false)
  {
    Serial.println(F("Failed to enter static position. Freezing..."));
    while (1)
      ;
  }
  else
    Serial.println(F("Static position set"));
  
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)
}

void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.
  delay(250); //Don't pound too hard on the I2C bus
}

//This function gets called from the SparkFun u-blox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Sends RTCM Data over radio in byte array packages

#define RTCM_DATA_SIZE 256 // Define the size of the RTCM data array

uint8_t rtcData[RTCM_DATA_SIZE]; // Create a byte array to store the RTCM data

void DevUBLOXGNSS::processRTCM(uint8_t incoming)
{
  static uint16_t byteCounter = 0;
  if ((myGNSS.rtcmFrameCounter == 1 && byteCounter > 0) || byteCounter == 256)
  { 
    int state = radio.startTransmit(rtcData, byteCounter);
    if (state == RADIOLIB_ERR_NONE) {
                delay(100);
                radio.finishTransmit();
                Serial.print(byteCounter);
                Serial.println(" byte RTCM Data Sent");
            }
    byteCounter = 0;
  } 
    rtcData[byteCounter] = incoming;
    byteCounter++; 
}
