
#include <Audio.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SerialFlash.h>

AudioSynthWaveform    waveform1;
AudioOutputAnalog           dac;
AudioConnection       patchCord1(waveform1, dac);

void setup() {
//    pinMode(OUTPUT_CTR_5V,OUTPUT);
//  digitalWrite(OUTPUT_CTR_5V,LOW);//open 5V
//  pinMode(OUTPUT_CTR_3V3,OUTPUT);
//  digitalWrite(OUTPUT_CTR_3V3,LOW);//open 3v3
  AudioMemory(10);
  Serial.begin(115200);
  waveform1.begin(WAVEFORM_SINE);
}

void loop() {
  Serial.print("Beep #");
  waveform1.frequency(440);
  waveform1.amplitude(0.9);
  delay(200);
  waveform1.amplitude(0);
  delay(200);
}
