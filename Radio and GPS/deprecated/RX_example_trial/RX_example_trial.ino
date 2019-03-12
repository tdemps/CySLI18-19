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
  // initializing all pins and any servos
  Serial.begin(115200);
  pinMode(LED, OUTPUT); 
  pinMode(EMATCH, OUTPUT);    
  pinMode(RFM95_RST, OUTPUT);
  // don't know why it's set low here only to be set low again later
  // but I know it doesn't want to work when it's set high and
  // low again so I'm leaving this in
  digitalWrite(RFM95_RST, LOW);
  servo.attach(5);

  // manual reset, need to keep so init doesn't fail
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(EMATCH,LOW);
  // starting the rf95 and setting its frequency
  if (!rf95.init()) {
    Serial.println("RFM95 radio init failed");
    while (1);
  }  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // range from ?-23 for power
  rf95.setTxPower(23, true);  

  // basically leaving this in as a proof of a successful init
  Serial.print("Receiver Ready");
}


void loop() {
  // checking if we received anything
  if (rf95.available()) {

    // setting the max size of the message
    // I think it's defined in the library
    // not sure what it is but I know it's 
    // less than 20 so we should stay under
    // that on the transmitter side. 
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // if recv returns with a message, let's look at it
    if (rf95.recv(buf, &len)) {

      // if it's empty, leave
      if (!len) return;

      // not sure why it's set to 0, maybe try taking out and seeing what happens
      buf[len] = 0;

      // won't really need these on launch day because can't see the serial port
      // on this end, should delete sooner than later. 
      Serial.print("Received [");
      Serial.print(len);
      Serial.print("]: ");
//      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);

      // if the message was "in", move the pins in towards the center releasing
      // the retention system
      if (strstr((char *)buf, "in")) {
        // Send a reply!
        uint8_t data[] = "received in";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("turned pins in");
        servo.write(0);

      // if message was "out", move the pins out engaging the retention system
      } else if (strstr((char *)buf, "out")) {
        // send acknowledgement
        uint8_t data[] = "moved pins out";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        servo.write(60);

      // if the message was fire, send current to the ematch, we'll eventually have
      // to move this to a new script specifically for the rover deployment feather
      // could leave it in since it wouldn't hurt anything as long as nothing is in
      // pin 11 on the rover retention feather.
      } else if (strstr((char *)buf, "fire")) {
        // acknowledge
        // todo: send back GPS here

        uint8_t data[] = "stand back, firing";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        digitalWrite(EMATCH, HIGH);
        delay(2000);
        digitalWrite(EMATCH, LOW);
      }
    } else {
      Serial.println("Receive failed");
    }
  }
}
