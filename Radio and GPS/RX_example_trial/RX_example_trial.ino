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
#include <Servo.h>
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM95_CS      8
  #define RFM95_INT     3
  #define RFM95_RST     4
  #define LED           13
#endif

#define EMATCH 11

// create instances of sensors
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Servo servo;

//int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup() 
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");

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
  servo.attach(5);
  pinMode(LED, OUTPUT);
  pinMode(EMATCH, OUTPUT);
  Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");
}


void loop() {
 if (rf95.available()) {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      if (!len) return;
      buf[len] = 0;
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
//      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);

      if (strstr((char *)buf, "in")) {
        // Send a reply!
        // todo: send back GPS here
        uint8_t data[] = "received in";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("turned pins in");
//        Blink(LED, 1000, 3); //blink LED 3 times, 40ms between blinks
        servo.write(0);
      } else if (strstr((char *)buf, "out")) {
        // send acknowledgement
        // todo: send back GPS here

        uint8_t data[] = "moved pins out";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
//        Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
        servo.write(60);
      } else if (strstr((char *)buf, "fire")) {
        // acknowledge
        // todo: send back GPS here

        uint8_t data[] = "stand back, firing";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        digitalWrite(EMATCH, HIGH);
        
      }
    } else {
      Serial.println("Receive failed");
    }
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
