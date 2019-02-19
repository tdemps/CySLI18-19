// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF69 class. RH_RF69 class does not provide for addressing or
// reliability, so you should only use RH_RF69  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf69_server.
// Demonstrates the use of AES encryption, setting the frequency and modem 
// configuration

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
  Serial.begin(115200);

  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");

  // manual reset, need to keep so init doesn't fail
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    Serial.println("RFM95 radio init failed");
    while (1);
  }
  Serial.println("RFM95 radio init OK!");
 
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf95.setTxPower(23, true);  // range from 14-23 for power, 2nd arg must be true for 69HCW
  
  pinMode(LED, OUTPUT);

  Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");
}



void loop() {
  delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!
  if (Serial.available()){
  char input = Serial.read();
  
    if (input == '1'){
      char radiopacket[20] = "in";
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending "); 
      Serial.println(radiopacket);  
      // Send a message!
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();
      
    } else if (input =='3'){
      char radiopacket[20] = "out";
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending "); 
      Serial.println(radiopacket);
      // Send a message!
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();
      
    } else if (input == '5'){
      char radiopacket[20] = "fire";
      itoa(packetnum++, radiopacket+13, 10);
      Serial.print("Sending "); 
      Serial.println(radiopacket);
      // Send a message!
      rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
      rf95.waitPacketSent();
      
    }
  }
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
