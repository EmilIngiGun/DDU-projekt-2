// ----- CONFIG -----
const byte numSensors = 8;                   // number of pressure sensors
const byte sensorPins[numSensors] = {A0, A1, A2, A3, A5, A10, A12, A14}; // analog input pins

// ----- FILTER AND THRESHOLD SETTINGS -----
float filterAlpha = 16.0;
float thresholdOffset[numSensors] = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0}; // can be tuned per sensor

// ----- PER-SENSOR STATE -----
float filtered[numSensors] = {372.0, 500.0, 370.0, 820.0, 260.0, 850.0, 550.0, 735.0};
//float movingThreshold[numSensors] = {372.0, 500.0, 370.0, 820.0, 260.0, 850.0, 550.0, 735.0};
float movingThreshold[numSensors] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float minimumVal[numSensors] = {362.0, 480.0, 350.0, 800.0, 240.0, 830.0, 530.0, 715.0};
float maximumVal[numSensors] = {415.0, 690.0, 700.0, 960.0, 400.0, 966.0, 730.0, 960.0};

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
//    Serial.print((filtered[i]-minimumVal[i])/(maximumVal[i]-minimumVal[i]));
  Serial.print(filtered[i]);  
    Serial.print(" S");
    /*
    Serial.print(i);
    Serial.print("_min:");
    Serial.print(0);

    Serial.print(" S");
    Serial.print(i);
    Serial.print("_max:");
    Serial.print(1);
  */
    if (i < numSensors - 1) Serial.print(" ");
    

  }

  Serial.println();

  delay(10); // Small delay for stable plotting
}
