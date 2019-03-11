#include <SPI.h>

#define ARDUINO_EMATCH 8
#define COMM 10

void setup() {
  Serial.begin(115200);
  pinMode(ARDUINO_EMATCH, OUTPUT);
  digitalWrite(ARDUINO_EMATCH,LOW);
  
  pinMode(COMM, INPUT);
}

void loop() {
  if (digitalRead(COMM) == HIGH){
    Serial.println("firing"); 
    digitalWrite(ARDUINO_EMATCH, HIGH);
    
    delay(1000);
    
    digitalWrite(ARDUINO_EMATCH, LOW);
  }
  
}
