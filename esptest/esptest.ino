#include <Wire.h>
#include <MPU9255.h>   // Works fine for MPU-6500
#include "driver/dac.h"
#include <math.h>
#include "FS.h"
#include "SPIFFS.h"
#include "wav.h"

#include <SPI.h>
#include <SD.h>

#define SD_CS 5
File wavFile;


const int audioPin = 25;
const int button = 27; 
bool but, butP; 
const int sampleRate = 7000; // samples per second
const int freq = 100;      
float phase = 0.0;
unsigned long lastSample = 0;
float defDelay = 500; 
float del; 


MPU9255 mpu;

float vx = 0.0, vy = 0.0;
float axNorm, ayNorm; 
unsigned long tPrev = 0;

void setup() {
  Serial.begin(115200);
  
  delay(100);

  Wire.begin(21, 22);

  if (!mpu.init()) {
    Serial.println("MPU initialization failed!");
    while (1);
  }

  Serial.println("MPU initialized!");
  mpu.set_acc_scale(scale_2g);
  pinMode(audioPin, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  tPrev = millis();

  Serial.begin(115200);

  SPI.begin(18, 19, 23, SD_CS);
  if (!SD.begin(SD_CS, SPI, 1000000)) {
    Serial.println("SD init failed");
    return;
  }

  wavFile = SD.open("/aero_car_noise.wav");
  if (!wavFile) {
    Serial.println("Failed to open file");
    return;
  }
  wavFile.seek(44);

}

void loop() {
  unsigned long tNow = millis();
  but = digitalRead(button);
  mpu.read_acc();
  float ax = (mpu.ax / 16384.0) * 9.80665;  // m/s²
  float ay = (mpu.ay / 16384.0) * 9.80665;
  static bool run1 = true; 
  if (run1){ 
    run1 = false; 
    axNorm = ax; 
    ayNorm = ay; 
  }
if (but == LOW && but != butP) {
  vx = 0;
  vy = 0;

  // --- Take 20 samples for stable normalization ---
  float sumAx = 0, sumAy = 0;
  for (int i = 0; i < 20; i++) {
    mpu.read_acc();
    float axNow = (mpu.ax / 16384.0) * 9.80665;
    float ayNow = (mpu.ay / 16384.0) * 9.80665;
    sumAx += axNow;
    sumAy += ayNow;
    delay(10);  // small pause between readings
  }
  axNorm = sumAx / 20.0;
  ayNorm = sumAy / 20.0;

  //Serial.println("Velocity reset + normalization complete!");
  }
  ax = ax - axNorm; 
  ay = ay - ayNorm; 

  float dt = (tNow - tPrev) / 1000.0;  // convert to seconds
  tPrev = tNow;

  if (abs(ax) < 0.1) ax = 0;
  if (abs(ay) < 0.1) ay = 0;

  vx += ax * dt;
  vy += ay * dt;

  float aMag = sqrt(ax * ax + ay * ay);
  float vMag = sqrt(vx * vx + vy * vy);


  Serial.println(vMag, 4); 




  // delay based on sample rate (example: 8kHz)
  //del = defDelay * aMag;
  phase += 0.5f;        // base pitch increment
  if (phase >= 2 * M_PI) phase -= 2 * M_PI;

  del = map(aMag, 0, 10, 200, 1.83);
  uint8_t sample = 128 + 100 * sinf(phase);
  dacWrite(audioPin, sample);


  delayMicroseconds(del); // 1 / 8000 sec = 125 µs // 20.83

}

/*
  if (!wavFile.available()) {
    Serial.println("Done playing!");
    wavFile.close();
    while (1); // stop
  }

  // read one sample
  uint8_t sample = wavFile.read();

  // output to DAC pin 25 (0–255)
  dacWrite(audioPin, .5 * sample);

  // delay based on sample rate (example: 8kHz)
  //del = defDelay * aMag;

  del = map(aMag, 0, 50, 200, 10.83);
  delayMicroseconds(1); // 1 / 8000 sec = 125 µs // 20.83

    int newTone = freq; 
  if (aMag !=0){ 
    newTone *= aMag;
  }

  butP = but; 
                  // 1 kHz
  unsigned long nowMicros = micros();
  if (nowMicros - lastSample >= (1000000 / sampleRate)) {
      lastSample = nowMicros;

      phase += 2 * M_PI * newTone / sampleRate;
      if (phase >= 2 * M_PI) phase -= 2 * M_PI;

      uint8_t val = 128 + 100 * sinf(phase);   // reduce amplitude for headroom
      uint8_t unclip = map(val, 0, 255, 0, 125);
      dacWrite(audioPin, val / 4);                 // correct DAC range
  }


*/


  //Serial.print("ax, ay: ");
  //Serial.print(ax); Serial.print(", ");
  //Serial.print(ay);
  //Serial.print("   Vx, Vy: ");
  //Serial.print(vx); Serial.print(", ");
  //Serial.println(vy);

  
