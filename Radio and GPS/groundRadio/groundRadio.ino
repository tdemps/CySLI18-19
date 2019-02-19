#include<SPI.h>
#include<RH_RF95.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915.0

//Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }

  delay(100);

  Serial.println("CySLI Ground Radio");

  while(!rf95.init()){
    Serial.println("CySLI Ground Radio init failed");
    while(1);
  }
  Serial.println("CySLI Ground Radio init OK!");

  if(!rf95.setFrequency(RF95_FREQ)){
    Serial.println("setFrequency failed");
    while(1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23,false);
}

void loop() {
  // why do we need to delay a second?
  delay(1000);
  Serial.println("Transmitting...");
  // what's up with the user commands
  Serial.println("Enter command: ");
  //This needs to be user input
  uint8_t cmd[] = "deploy";
  Serial.println("Sending deploy command...");
  // why the delays?
  delay(10);
  rf95.send(cmd,sizeof(cmd));
  delay(10);
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if(!rf95.waitAvailableTimeout(1000)){
    if(rf95.recv(buf, &len)){
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.println((char*)buf);
    }
    else{
      Serial.println("Receive failed");
    }
  }
  else{
    Serial.println("No reply, is CySLI Onboard Radio on?");
  }

}
