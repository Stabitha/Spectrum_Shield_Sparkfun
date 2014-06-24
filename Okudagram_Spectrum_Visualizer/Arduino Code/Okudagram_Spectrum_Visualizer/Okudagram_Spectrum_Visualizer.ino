// Sparkfun Spectrum Shield Okudagram Visulizer
// Written using Arduino IDE v1.0.5 and Processing v2.0.3 on Nov 3rd, 2013. by Tom Flock
// Based on Example 48.1 - tronixstuff.com/tutorials > chapter 48 - 30 Jan 2013
// "MSGEQ7 spectrum analyser shield - basic demonstration"

// This code receives multiplexed data from the Spectrum Shield's two MSGEQ7 ICs
// and transmits the values via serial at a 115200 baud rate.
// The fourteen values are seperated by commas and are terminated by a newline,
// which is the format expected by the accompanying processing code.

// MSGEQ7 Control
int strobe = 4; // strobe pins on digital 4
int res = 5; // reset pins on digital 5
static const byte smoothP = 20;  // Number of samples to compute rolling average over (empirically set)
int count = 0;

int initRootR = 25;
int initRootL = 25;
int left[7]; // store band values in these arrays
int right[7];
int bslR[7] = { 0, 0, 0, 0, 0, 0, 0};
int bslL[7] = { 0, 0, 0, 0, 0, 0, 0};
uint16_t varianceR[7] = { 0, 0, 0, 0, 0, 0, 0};
uint16_t varianceL[7] = { 0, 0, 0, 0, 0, 0, 0};
int baselineRight[7] = { 0, 0, 0, 0, 0, 0, 0};
int baselineLeft[7] = { 0, 0, 0, 0, 0, 0, 0};
int stdDevRight[7] = { 0, 0, 0, 0, 0, 0, 0};
int stdDevLeft[7] = { 0, 0, 0, 0, 0, 0, 0};
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

// this routine takes a variance as input and gives an initial estimate of the standard deviation to ensure convergence of the secant method
int initialRootValue(uint16_t &aVariance) {
  byte powerOfTwo = 1;
  uint32_t twoN = 1;
  int r = 2;
  for (int i = 1; i < 17; i++) { 
    if (aVariance < (twoN *= 2)) {  // if S < 2^i for i = (1,...,16)
      powerOfTwo++;                 // increase powerOfTwo (n)
      powerOfTwo /= 2;              // assign n/2 to powerOfTwo
    } else {
      powerOfTwo++;                 // otherwise, just increment powerOfTwo
    }
  }
  for (int i = 0; i < powerOfTwo; i++) {  // compute 2^powerOfTwo to return as the second estimate of the square root
    r *= r;
  }
  
  return r;                         // since powerOfTwo is 8 or less, r can be an int
}

int secantRoot(uint16_t &aVariance, int &aMean, int binaryEst) {
  /* The first point is chosen to be the mean, which is assumed to be greater than the standard deviation for a normally distributed
     random variable with range (0, ..., 1023) and mean > 1. 
     The second point is chosen by representing S as a x 2^(2n) in binary, and estimating sqrt(S) as 2^n with n chosen such that 
     1 x 2^(2n) is greater than S. This ensures the second point is also greater than S and that the algorithm converges. */
  int x_n2 = aMean;  
  int x_n1 = binaryEst; 
  int x_n = (x_n2*((x_n1*x_n1)-aVariance) - x_n1*((x_n2*x_n2)-aVariance)) / (((x_n1*x_n1)-aVariance) - ((x_n2*x_n2)-aVariance));
  x_n2 = x_n1;
  while ((x_n - x_n1) > 1) { // no use interating if the difference is smaller than one integer
    x_n1 = x_n;
    x_n = (x_n2*((x_n1*x_n1)-aVariance) - x_n1*((x_n2*x_n2)-aVariance)) / (((x_n1*x_n1)-aVariance) - ((x_n2*x_n2)-aVariance));
    x_n2 = x_n1;
  }
  
  return x_n;
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

void baselineAdjust(int stopPos, int *rChannel, int *lChannel) { // Use Welford's algorithm
  for (int i = 1; i <= stopPos; i++) { // 100 represents a 100-point moving average, taking one sample every 7*(30us) or so, every 210us or 4761Hz.
    readMSGEQ7();                      // read the spectrum 100 times
    for (band = 0; band < 7; band++) {  //and for each readout add the l/rChannel values up
      bslL[band] = lChannel[band];
      bslR[band] = rChannel[band];
      lChannel[band] += ((left[band] - lChannel[band])/i);  // M_k = M_k-1 + (x_k - M_k-1)/k
      rChannel[band] += ((right[band] - rChannel[band])/i); // Moving 'stopPos' average of left and right channels
      if (i > 1) {
        varianceL[band] += (((left[band] - bslL[band]) * (left[band] - lChannel[band]))/(i-1));  // Moving 'stopPos' variance of left and right channels
        varianceR[band] += (((right[band] - bslR[band]) * (right[band] - rChannel[band]))/(i-1));
      }
    }
  }
  for (band = 0; band < 7; band++) {
    initRootL = initialRootValue(varianceL[band]);
    initRootR = initialRootValue(varianceR[band]);
    // here I am giving the baseline correction as the standard deviation, but I want to use average for this
    // then use standard deviation to limit the power spectrum that goes out for post-processing so it is not jittery
    stdDevLeft[band] = secantRoot(varianceL[band], lChannel[band], initRootL);   
    stdDevRight[band] = secantRoot(varianceR[band], rChannel[band], initRootR);  
  }
}                           

void readMSGEQ7c() {
// Function to apply baseline correction & clip power spectra to +/- 1s.d. from the average in any given burst of samples
  baselineAdjust(smoothP, baselineRight, baselineLeft); //baselineAdjust() calls readMSGEQ7() and gets left[band]/right[band]
  for(band=0; band <7; band++) {
    right[band] = left[band]; // show uncorrected left band in the right-hand side
    left[band] = max(left[band], baselineLeft[band]-stdDevLeft[band]);
    left[band] = min(left[band], baselineLeft[band]+stdDevLeft[band]);
//    reduce(left[band], baselineLeft[band], 0); // baseline correction (maybe don't do this since it mostly just removes the signal mean>>s.d.)
//    right[band] = max(right[band], baselineRight[band]-stdDevRight[band]);
//    right[band] = min(right[band], baselineRight[band]+stdDevRight[band]);
//    reduce(right[band], baselineRight[band], 0); // baseline corection (maybe don't do this since it mostly just removes the signal mean>>s.d.)
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(res, OUTPUT); // reset
  pinMode(strobe, OUTPUT); // strobe
  digitalWrite(res,LOW); // reset low
  digitalWrite(strobe,HIGH); //pin 5 is RESET on the shield
  baselineAdjust(smoothP, baselineRight, baselineLeft); // grab band-specific baseline adjustments (assumes no audio on initialization)
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
}

