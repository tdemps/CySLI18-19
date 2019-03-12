#include <SPI.h>
#include <Servo.h>
#include <RH_RF95.h>

#define RF95_FREQ 915.0

// Adafruit Feather M0 w/ LoRa Radio
#define RFM95_CS      8
#define RFM95_INT     3
#define RFM95_RST     4
#define SERVO         5

RH_RF95 rf95(RFM95_CS, RFM95_INT);  // RF95 radio
Servo servo;  //servo

void setup() {
  Serial.begin(115200);
  radioSetup();
  servoSetup();

}

void loop() {
  if (Serial.available()){
    char input = Serial.read();
    if (input == '1'){
      Serial.println("turning to 0");
      servo.write(0);
    }else if (input == '3'){
      Serial.println("turning to 50");
      servo.write(50);
    }
    
  }
}

void servoSetup() {
    Serial.println("Attaching servo...");
    servo.attach(SERVO);
}

void radioSetup() {

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

}
