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
#define LED           13

// ematch, servo, and GPS
#define EMATCH 11
#define SERVO 5
#define GPSSerial Serial1

// create instances of sensors
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // RF95 radio
Adafruit_GPS GPS(&GPSSerial);  // Adafruit Ultimate GPS
Servo servo;  //servo

//int16_t packetnum = 0;  // packet counter, we increment per xmission

// turn off echoing of GPS data to the Serial console
#define GPSECHO false

// GPS transmit status (ON/OFF)
bool gps_transmit = false;  //Default GPS output to OFF

uint32_t timer = millis();

void setup()
{
  Serial.begin(115200);

  //This can be commented out or removed. It is here so we can see startup messages/errors
  while (!Serial) {
    delay(1);
  }

  Serial.println("Starting setup...");
  radioSetup();  // Setup radio
  servoSetup();  // Setup servo
  gpsSetup();  // Setup GPS
  pinMode(EMATCH, OUTPUT);
  Serial.println("Setup complete!");
}


void loop() {

  // Begin reading GPS
  char c = GPS.read();
  if (GPSECHO)
    if (c) Serial.print(c);

  // Check if we have a new chunk of data
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA());
    GPS.parse(GPS.lastNMEA());  // Parse new GPS NMEA sentence
  }

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

      } else if (strstr((char *)buf, "gpsOn")) {
        uint8_t data[] = "Turning ON GPS";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        gps_transmit = true;

      } else if (strstr((char *)buf, "gpsOff")) {
        uint8_t data[] = "Turning OFF GPS";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
        Serial.println("Sent a reply");
        gps_transmit = false;
      }

    } else {
      Serial.println("Receive failed");
    }
  }

  // Check if output is turned on
  if (gps_transmit == true && !rf95.available()) {
    // Check if the GPS has a fix
    if (millis() - timer > 1000) {
      timer = millis();
      if (GPS.fix) {
        char location[100];
        float lat = GPS.latitudeDegrees;  // Latitude in degrees
        float lon = GPS.longitudeDegrees;  // Longitude in degrees
        char latC = GPS.lat;  // (N or S)
        char lonC = GPS.lon;  // (E or W)
        sprintf(location, "Location: %f%c, %f%c", lat, latC, lon, lonC);  // Format outgoing GPS location
        delay(1000);
        rf95.send((uint8_t *)location, 100);
        rf95.waitPacketSent();
        Serial.println("Sent GPS location");
      }
      else {
        uint8_t location[] = "GPS error: unable to get a fix";
        delay(1000);
        rf95.send(location, sizeof(location));
        rf95.waitPacketSent();
        Serial.println("Sent GPS fix error");
      }
    }
  }
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN, LOW);
    delay(DELAY_MS);
  }
}


void radioSetup() {

  pinMode(LED, OUTPUT);
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

  pinMode(LED, OUTPUT);
}

void servoSetup() {
  Serial.println("Attaching servo...");
  servo.attach(SERVO);
}

void gpsSetup() {
  // Print startup message
  Serial.println("Setting up GPS module...");

  // Default baud rate
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Set update rate to 1Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
}
