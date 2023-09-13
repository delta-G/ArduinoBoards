void setup() {
  // Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  Serial.println("Hello World");
  pinMode(7, INPUT_PULLUP);

  pinMode(21, OUTPUT);

  pinMode(13, OUTPUT);

}

void loop() {

  if(digitalRead(7) == LOW){
    digitalWrite(21, HIGH);
  }
  else{
    digitalWrite(21, LOW);
  }

  static uint32_t pm = millis();
  uint32_t cm = millis();
  if(cm-pm>500){
    Serial.println("Serial - ");
    Serial1.println("Serial1 - ");
    Serial2.println("Serial2 -");
    digitalWrite(13, !digitalRead(13));
    pm = cm;
  }

}
