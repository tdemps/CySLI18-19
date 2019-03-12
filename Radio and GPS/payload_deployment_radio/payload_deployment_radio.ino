/*
   CYSLI RF95 RADIO PROGRAM (PAYLOAD)
   This program controls the operation of the radio in the payload bay.
   The radio is designed to accept commands from and communicate back to the ground radio.
   The radio is designed to do the following:
      - Engage/Disengage retention system
      - Fire ematch for deployment system
      - Send rocket location from GPS
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>
#include <Servo.h>
/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Adafruit Feather M0 w/ LoRa Radio
#define RFM95_CS      8
#define RFM95_INT     3
#define RFM95_RST     4

// ematch, servo, and GPS
#define EMATCH 13

// create instances of sensors
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // RF95 radio

//int16_t packetnum = 0;  // packet counter, we increment per xmission

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting setup...");
  radioSetup();  // Setup radio
  pinMode(EMATCH, OUTPUT);
  digitalWrite(EMATCH, LOW);
  Serial.println("Setup complete!");
}


void loop() {
  // Check if a message is available
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
      if (strstr((char *)buf, "fire")) {
        // acknowledge
        uint8_t data[] = "stand back, firing";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        digitalWrite(EMATCH, HIGH);
        delay(1000);
        digitalWrite(EMATCH, LOW);
      }
    } else {
      Serial.println("Receive failed");
    }
  }
}
void radioSetup() {

//  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);

  Serial.println("CySLI RX Test!");

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
  rf95.setTxPower(23, false);  // range from 14-23 for power, 2nd arg must be true for 69HCW
  Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");

//  pinMode(LED, OUTPUT);
}
