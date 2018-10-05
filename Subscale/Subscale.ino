
#include <sensors.h>
#include <SD.h>

unsigned long time = 0;
int16_t accX, accY,accZ;
double altitude;


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  //setup sd card
  SDcardSetup();
  SDcardWriteSetup();
  
  //setup accelerometer here

}

void loop() {
  // put your main code here, to run repeatedly:

  
  time = millis();  //current time in milliseconds
  //take time, acceleration and altitude measurements

  
  //save them to sd card
  WriteData();

}
