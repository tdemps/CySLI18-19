#include <SPI.h>
#include <RH_RF95.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM95_CS      8
  #define RFM95_INT     3
  #define RFM95_RST     4
  #define LED           13
#endif
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  // initializing serial and pins
  Serial.begin(115200);
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  // don't know why it needs to be set low to be set low again,
  // but it works this way so no point in screwing it up,
  // maybe if we have time
  digitalWrite(RFM95_RST, LOW);

  // manual reset, need to keep so init doesn't fail
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // initializing the radio and frequency
  if (!rf95.init()) {
    Serial.println("RFM95 radio init failed");
    while (1);
  } 
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // range from ?-23 for power
  rf95.setTxPower(23, false);  
  
  Serial.print("Transmitter Ready");
}



void loop() {
  delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!
  if (Serial.available()){
    // reads single char inputs from the serial monitor
    char input = Serial.read();

    // if it's 1, disengage the retention system
    if (input == '1'){
      char radiopacket[20] = "in";
      
      // pretty sure we don't need this line but I haven't tested it yet
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending "); 
      Serial.println(radiopacket);  
      // Send a message!
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();

    // if it's 3, engage the retention system  
    } else if (input =='3'){
      char radiopacket[20] = "out";
      
      // pretty sure we don't need this line but I haven't tested it yet
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending "); 
      Serial.println(radiopacket);
      // Send a message!
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();

    // if it's five, fire the ematch for the deployment system
    } else if (input == '5'){
      char radiopacket[20] = "fire";

      // pretty sure we don't need this line but I haven't tested it yet
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending "); 
      Serial.println(radiopacket);
      // Send a message!
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();

    // if it's seven, turn on the GPS output
    } else if (input == '7'){
      char radiopacket[20] = "gpsOn";
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending ");
      Serial.println(radiopacket);
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();

    // if it's nine, turn off GPS output
    } else if (input == '9'){
      char radiopacket[20] = "gpsOff";
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending ");
      Serial.println(radiopacket);
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();
    }
  }

  // this is where GPS would be received, just need to write the other side.
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(500))  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
      Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}
