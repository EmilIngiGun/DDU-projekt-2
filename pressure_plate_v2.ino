void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long starttime;
  unsigned long dt;
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
  delay(50);
  digitalWrite(A0, LOW);
  starttime = micros();
  int reading = analogRead(A0);
  while(reading > 100){
    reading = analogRead(A0);
   // Serial.println(reading);
  }
  dt = micros()-starttime;
  Serial.println(dt);
  delay(50);
}
