#include "SPIFFS.h"
#include <driver/dac.h>

#define WAV_FILE "/aero_car_noise.wav"

// ~22kHz playback rate
#define SAMPLE_DELAY_US 45  

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== SPIFFS AUDIO PLAYBACK ===");

  // Mount SPIFFS (NO formatting!)
  if (!SPIFFS.begin(false)) {
    Serial.println("SPIFFS Mount Failed!");
}

  Serial.println("SPIFFS mounted.");

  // Try opening the WAV file
  File f = SPIFFS.open(WAV_FILE, "r");
  if (!f) {
    Serial.println("ERROR: WAV file not found!");
    return;
  }

  Serial.printf("Opened %s   (%u bytes)\n", WAV_FILE, f.size());

  // Enable DAC output on GPIO25
  dac_output_enable(DAC_CHANNEL_1);

  Serial.println("Playing WAV...");

  // Stream file to DAC
  while (f.available()) {
    uint8_t sample = f.read();           // read next byte
    dac_output_voltage(DAC_CHANNEL_1, sample);
    delayMicroseconds(SAMPLE_DELAY_US);  // playback rate
  }

  f.close();
  Serial.println("Playback finished.");
}

void loop() {}
