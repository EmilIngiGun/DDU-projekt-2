float filterAlpha = 16.0;

float filtered = 390.0;
float thresholdOffset=50.0;
float movingThreshold = filtered+thresholdOffset;

unsigned long loopCounter=0;
void setup() {
Serial.begin(115200);
}
float minimum = 350.0;
float maximum = 450.0;



void loop() {
  int val = analogRead(A0);

  if(val>maximum) maximum=val;
  if(val<minimum) minimum=val;

  

  filtered = (filtered*filterAlpha+(float)val)/(filterAlpha+1.0); //First order filter is fairly responsive

  movingThreshold = (movingThreshold*filterAlpha*32+filtered*1.1)/(filterAlpha*32+1.1); //second order filter lags behind quite a bit on purpose...

  if(movingThreshold>filtered*1.1) movingThreshold=filtered; //Drop fast

  if(maximum>movingThreshold+thresholdOffset/4) maximum-=0.01; //creeping down the maximum
  if(minimum<movingThreshold-thresholdOffset/4) minimum+=0.01; //creeping up the minimum
  Serial.print("Raw_ADC:");
  Serial.print(val);
  Serial.print(",Filtered_ADC_(1st-order-filter):");
  Serial.print(filtered);
  Serial.print(",movingThreshold_(2nd-order-filter):");
  Serial.print(movingThreshold);
  Serial.print(",Autoscale_Bottom:");
  Serial.print(minimum);
  Serial.print(",Autoscale_Top:");
  Serial.print(maximum);
  Serial.println();
  
  
  delay(5);

}
