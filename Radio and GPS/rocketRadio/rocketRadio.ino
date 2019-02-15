#include<SPI.h>
#include<RH_RF95.h>
#include<Adafruit_GPS.h>
#include<SD.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define GPSSerial Serial1

//Initialize objects
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_GPS GPS(&GPSSerial);

//turn off echoing of GPS data to the Serial console
#define GPSECHO false

//Blinky on receipt
#define LED 13

//Password for deployment
uint8_t DEPLOY[] = "deploy";
uint8_t timer = millis();

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while(!Serial) {
    delay(1);
  }
  delay(100);

  Serial.println("CySLI Onboard Radio & GPS");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while(!rf95.init()) {
    Serial.println("CySLI Onboard radio init failed");
    while(1);
  }
  Serial.println("CySLI Onboard radio init OK!");

  if(!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while(1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23,false);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

void loop() 
{
  if(rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    //GPS initialization stuff
//    char c = GPS.read();
//    if (GPSECHO)
//      if (c) Serial.print(c);
//    if (GPS.newNMEAreceived()){
//      Serial.println(GPS.lastNMEA());
//      if(!GPS.parse(GPS.lastNMEA()))
//        return;
//    }
//    if (timer > millis()) timer = millis();

    if(rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      if(cmdCheck(buf,DEPLOY)==1){
        deploy();
        uint8_t reply[] = "Deploying payload...";
        rf95.send(reply, sizeof(reply));
        rf95.waitPacketSent();
        Serial.println("Sent deployment confirmation.");
        digitalWrite(LED, LOW);
      }
      else
      {
        uint8_t reply[] = "Command not recognized.";
        rf95.send(reply,sizeof(reply));
        rf95.waitPacketSent();
        Serial.println("Sent command error.");
        digitalWrite(LED,LOW);
      }
//      if (millis() - timer > 2000) {
//        timer = millis(); // reset the timer
//        Serial.print("\nTime: ");
//        Serial.print(GPS.hour, DEC); Serial.print(':');
//        Serial.print(GPS.minute, DEC); Serial.print(':');
//        Serial.print(GPS.seconds, DEC); Serial.print('.');
//        Serial.println(GPS.milliseconds);
//        Serial.print("Date: ");
//        Serial.print(GPS.day, DEC); Serial.print('/');
//        Serial.print(GPS.month, DEC); Serial.print("/20");
//        Serial.println(GPS.year, DEC);
//        Serial.print("Fix: "); Serial.print((int)GPS.fix);
//        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//        if (GPS.fix) {
//          float latitude = GPS.latitude;
//          float longitude = GPS.longitude;
//          char lat = GPS.lat;
//          char lon = GPS.lon;
//          rf95.send((uint8_t*)&longitude,sizeof(longitude));
//          rf95.waitPacketSent();
//          rf95.send((uint8_t*)&lon, sizeof(lon));
//          rf95.waitPacketSent();
//          rf95.send((uint8_t*)&latitude,sizeof(latitude));
//          rf95.waitPacketSent();
//          rf95.send((uint8_t*)&lat,sizeof(lat));
//          rf95.waitPacketSent();
//          Serial.println("Sent GPS data:");
//          Serial.println("\n Location: ");
//          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//          Serial.print(", ");
//          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//          Serial.print("Angle: "); Serial.println(GPS.angle);
//          Serial.print("Altitude: "); Serial.println(GPS.altitude);
//          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
//        }
  }
    //}
    else
    {
      Serial.println("Receive failed");
    }
  }
}

int cmdCheck(uint8_t *buf, uint8_t *ref)
{
  if(sizeof(buf)!=sizeof(ref))
  {
    return 0;
  }
  int len = sizeof(buf);
  for(int i = 0; i<len; i++){
    if(buf[i]!=ref[i])
    {
      return 0;
    }
  }
  return 1;
}

void deploy(){
  //Deploy payload by turning servo
}
