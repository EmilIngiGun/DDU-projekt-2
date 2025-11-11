// ----- CONFIG -----
const byte numSensors = 2;                   // number of pressure sensors
const byte sensorPins[numSensors] = {A2, A1}; // analog input pins

// ----- FILTER AND THRESHOLD SETTINGS -----
float filterAlpha = 16.0;
float thresholdOffset[numSensors] = {50.0, 50.0}; // can be tuned per sensor

// ----- PER-SENSOR STATE -----
float filtered[numSensors] = {390.0, 390.0};
float movingThreshold[numSensors] = {440.0, 800.0};
float minimumVal[numSensors] = {350.0, 700.0};
float maximumVal[numSensors] = {450.0, 1000.0};

unsigned long loopCounter = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting multi-sensor pressure plot...");
}

void loop() {
  // Update all sensors
  for (byte i = 0; i < numSensors; i++) {
    int val = analogRead(sensorPins[i]);

    // Update min/max adaptively
    if (val > maximumVal[i]) maximumVal[i] = val;
    if (val < minimumVal[i]) minimumVal[i] = val;

    // First-order filter (responsive)
    filtered[i] = (filtered[i] * filterAlpha + (float)val) / (filterAlpha + 1.0);

    // Second-order moving threshold (slower)
    movingThreshold[i] =
      (movingThreshold[i] * filterAlpha * 32 + filtered[i] * 1.1) /
      (filterAlpha * 32 + 1.1);

    if (movingThreshold[i] > filtered[i] * 1.1)
      movingThreshold[i] = filtered[i]; // drop quickly

    // Slowly creep min/max toward new center
    if (maximumVal[i] > movingThreshold[i] + thresholdOffset[i] / 4)
      maximumVal[i] -= 0.01;
    if (minimumVal[i] < movingThreshold[i] - thresholdOffset[i] / 4)
      minimumVal[i] += 0.01;
  }

  // ----- SERIAL PLOTTER OUTPUT -----
  // Each variable name + value separated by spaces
  for (byte i = 0; i < numSensors; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print("_speed:");
    Serial.print((filtered[i]-minimumVal[i])/(maximumVal[i]-minimumVal[i]));
    
    Serial.print(" S");
    Serial.print(i);
    Serial.print("_min:");
    Serial.print(0);

    Serial.print(" S");
    Serial.print(i);
    Serial.print("_max:");
    Serial.print(1);
    if (i < numSensors - 1) Serial.print(" ");
    

  }
  Serial.println();

  delay(10); // Small delay for stable plotting
}
