// Sparkfun Spectrum Shield Okudagram Visulizer
// Written using Arduino IDE v1.0.5 and Processing v2.0.3 on Nov 3rd, 2013. by Tom Flock
// Based on Example 48.1 - tronixstuff.com/tutorials > chapter 48 - 30 Jan 2013
// "MSGEQ7 spectrum analyser shield - basic demonstration"

// This code receives multiplexed data from the Spectrum Shield's two MSGEQ7 ICs
// and transmits the values via serial at a 115200 baud rate.
// The fourteen values are seperated by commas and are terminated by a newline,
// which is the format expected by the accompanying processing code.

// Added 'band'-specific baseline correction based on 100 sample burst when program first runs

// MSGEQ7 Control
int strobe = 4; // strobe pins on digital 4
int res = 5; // reset pins on digital 5
static const byte smoothP = 100;  // Number of samples to compute rolling average over

int left[7]; // store band values in these arrays
int right[7];
int baselineR[7] = { 0, 0, 0, 0, 0, 0, 0}; //initialize baseline corrections to be zero
int baselineL[7] = { 0, 0, 0, 0, 0, 0, 0};
int band;

inline void reduce(int &aByte, int aAmount, int aMin = 0)
{
  int r = aByte-aAmount;
  if (r<aMin)
    aByte = aMin;
  else
    aByte = (byte)r;
}

inline void increase(int &aByte, int aAmount, int aMax = 1023)
{
  int r = aByte+aAmount;
  if (r>aMax)
    aByte = aMax;
  else
    aByte = (byte)r;
}

void baselineAdjust(int stopPos, int *rChannel, int *lChannel) { // Use Welford's algorithm
  for (int i = 0; i < stopPos; i++) { 
    readMSGEQ7();                     // read the spectrum 100 times
    for (band = 0; band < 7; band++) {  
      lChannel[band] += ((left[band] - lChannel[band])/i);   // compute running average
      rChannel[band] += ((right[band] - rChannel[band])/i);  // compute running average
    }
  }
}  
void readMSGEQ7() {
// Function to read 7 band equalizers
  digitalWrite(res, HIGH);
  digitalWrite(res, LOW);
  for(band=0; band <7; band++) {
    digitalWrite(strobe,LOW); // strobe pin on the shield - kicks the IC up to the next band
    delayMicroseconds(30); //
    left[band] = analogRead(0); // store left band reading
    right[band] = analogRead(1); // ... and the right
    digitalWrite(strobe,HIGH);
  }
}
void readMSGEQ7c() {
// Function to read 7 band equalizers (w/ baseline correction applied)
  digitalWrite(res, HIGH);
  digitalWrite(res, LOW);
  for(band=0; band <7; band++) {
    digitalWrite(strobe,LOW); // strobe pin on the shield - kicks the IC up to the next band
    delayMicroseconds(30); //
    left[band] = analogRead(0); // store left band reading
    reduce(left[band], baselineL[band], 0); // baseline correction
    right[band] = analogRead(1); // ... and the right
    reduce(right[band], baselineR[band], 0); // baseline corection
    digitalWrite(strobe,HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(res, OUTPUT); // reset
  pinMode(strobe, OUTPUT); // strobe
  digitalWrite(res,LOW); // reset low
  digitalWrite(strobe,HIGH); //pin 5 is RESET on the shield
  baselineAdjust(smoothP, baselineR, baselineL); // grab band-specific baseline adjustments (assumes no audio on initialization)
}

void loop() {
  readMSGEQ7c();
  // display values of left channel on serial monitor
  for (band = 0; band < 7; band++) {
    Serial.print(left[band]);
    Serial.print(",");
  }
  // display values of right channel on serial monitor
  for (band = 0; band < 7; band++) {
    Serial.print(right[band]);
    Serial.print(",");
  }
  Serial.println();
  delay(1);
}
