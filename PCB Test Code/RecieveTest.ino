/*
Test for Receiving Board
Intended to be used with a LoRA Thing Plus expLoRaBLE
Tested Devices:
  1. Micro SD Card Reader
  2. Hexadecimal Rotary Encoder
  3. Push Button
  4. LoRa Radio
  5. LCD Screen
*/
#include <LiquidCrystal.h>
#include <Wire.h>
#include <RadioLib.h>
#include <SD.h>
#include <SPI.h>

Sd2Card card;
SdVolume volume;
SdFile root;
const int chipSelect = 2;

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

SX1262 radio = new Module(D36, D40, D44, D39, SPI1);

int getEncoder(int pin1, int pin2, int pin4, int pin8){
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin4, INPUT);
  pinMode(pin8, INPUT);
  int one = 0;
  int two = 0;
  int four = 0;
  int eight = 0;
  int total = 0;
  if (digitalRead(pin1) == HIGH) {
    one = 1;
  }
  if (digitalRead(pin2) == HIGH) {
    two = 2;
  }
  if (digitalRead(pin4) == HIGH) {
    four = 4;
  }
  if (digitalRead(pin8) == HIGH) {
    eight = 8;
  }
  total = one + two + four + eight;
  return total;
}

void setup() {
  lcd.begin(16, 2); // initialize the lcd 
  //lcd.setBacklight(1);
  lcd.print("Receive Test");


  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("If you see nothing on LCD; Check Schematic/Code");
  delay(100);
}

void loop() {
  pinMode(0, INPUT);
  int option = getEncoder(1,16,17,3);
  if (option == 0) {
    lcd.setCursor(0,0);
    lcd.print("Test SD Card          ");
    lcd.setCursor(0,1);
    lcd.print("Press to Select      ");
  }
  else if (option == 1) {
    lcd.setCursor(0,0);
    lcd.print("Test Radio          ");
    lcd.setCursor(0,1);
    lcd.print("Press to Select       ");
  }
  else {
    lcd.setCursor(0,0);
    lcd.print("End Options            ");
    lcd.setCursor(0,1);
    lcd.print("Scroll To See Options       ");
  }
  if (digitalRead(0) == HIGH){
     if (option == 0) {
       if (!card.init(SPI_HALF_SPEED, chipSelect)) {
         lcd.setCursor(0,0);
         lcd.print("SD Card Failed       ");

      }    
       else {
        lcd.setCursor(0,0);
        lcd.print("SD Card Connected       ");
            }
      lcd.setCursor(0,1);
      lcd.print("Press to go back          ");
      delay(100);
      while(digitalRead(0) == LOW);
      delay(100);

     }
     if (option == 1) {
       int state = radio.begin(915.0, 250.0, 7, 5, 0x34, 20, 10, 0, false);
       if (state == RADIOLIB_ERR_NONE) {
         lcd.setCursor(0,0);
         lcd.print("Radio Initilized         ");
         lcd.setCursor(0,1);
         lcd.print("wait for signal...        ");
         int state = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_NONE);
         delay(500);
         
         int numBytes = radio.getPacketLength();
        if (numBytes == 0){
           lcd.setCursor(0,1);
           lcd.print("Reception Failed        ");
           delay(5000);
          }
        else{
           lcd.setCursor(0,1);
           lcd.print("Reception Success         ");
           delay(5000);
        }
      }    
       else {
        lcd.setCursor(0,0);
        lcd.print("Radio Failed                  ");
            }
      lcd.setCursor(0,1);
      lcd.print("Press to go back                ");
      delay(100);
      while(digitalRead(0) == LOW);
      delay(500);
     }
    }


}
