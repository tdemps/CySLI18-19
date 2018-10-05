//--------------------------------------------------SD-CARD-FUNCTIONS-----------------------------------
void SDcardSetup(){
  /*Set up sd card to read RC data*/
  
  pinMode(SD_PIN, OUTPUT);
  if(!SD.begin(SD_PIN)){
    Serial.println("Card init failed, is the card inserted?");
    return;
  }
  Serial.println("SD Card initialized");
}

void SDcardWriteSetup(){
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  dataFile.println(F("Time(ms),Height(ft),AccX(ft/s^2),AccY(ft/s^2),AccZ(ft/s^2)"));
  dataFile.close(); 
}

void LogWrite(short reason){
  
  /*Called to write important checkpoints during flight*/
  
  //reason  //determines which event to write to log  //input
  
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  if(dataFile){
    switch(reason){
      case 1: dataFile.println(F("LAUNCH DETECTED"));
        break;
      case 2:dataFile.println(F("ABNORMAL FLIGHT DETECTED"));
        break;
      case 3:dataFile.println(F("FREEFALL DETECTED"));
        break;
      case 4:dataFile.println(F("BRAKE OPENED 5 DEG"));
        break;
      case 5: dataFile.println(F("BRAKE CLOSED"));
        break;
      case 6: dataFile.println(F("TARGET APOGEE REACHED, VEL > 0, BRAKING UNTIL FREEFALL"));
        break;
      case 7: dataFile.println(F("MOTOR BURNOUT"));
        break;
      default: break;
    }
  }else
    Serial.println("LogWrite: Error opening dataFile");
  
  dataFile.close();
}

void WriteData(){
 File dataFile = SD.open("Data.txt", FILE_WRITE);
 if(!dataFile)
      Serial.println("WriteData: Error opening dataFile");
 dataFile.print(time);
 dataFile.print(",");
 dataFile.print(altitude);
// dataFile.print(",");
// dataFile.print(altRefine);
 dataFile.print(",");
 dataFile.print(accX);  
 dataFile.print(",");
 dataFile.print(accY);
 dataFile.print(",");
 dataFile.print(accZ);
 dataFile.print(",");
// dataFile.print(getPos() - 35);
// dataFile.print(",");
// dataFile.print(accRefine);
// dataFile.print(",");
// dataFile.print(velocity);
// dataFile.print(",");  
// dataFile.println(projHeight);
 dataFile.close();
}
