
#define FEATHER_EMATCH 12
#define COMM 10

void setup() {
  Serial.begin(115200);
  pinMode(FEATHER_EMATCH, OUTPUT);
  pinMode(COMM, OUTPUT);

  digitalWrite(FEATHER_EMATCH,LOW);
  digitalWrite(COMM, OUTPUT);
}

void loop() {
  if(Serial.available()){
    char input = Serial.read();
    if (input == '5'){
      Serial.println("firing");
      digitalWrite(COMM,HIGH);
      digitalWrite(FEATHER_EMATCH,HIGH);
      digitalWrite(COMM,LOW);
      delay(1000);
      digitalWrite(FEATHER_EMATCH,LOW);
    }
  }
}
