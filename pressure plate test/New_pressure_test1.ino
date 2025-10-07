  const int sensorPin = A2;
  int sensorMin = 1700
  int sensorMax = 2700;

void setup() {
  serial.begin(9600);
}

void loop() {
  int sensorvalue = analogRead(sensorPin);
  
  // Tryk i bar , 0-100 procent scale
  int pressure = map(sensorValue, sensorMin, SensorMax, 0, 100);

  // udskriv data
  Serial.print("Raw: ")
  Serial.println(sensorValue);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  delay(200)
  }
