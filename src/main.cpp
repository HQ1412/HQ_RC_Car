#include <iostream>
#include "Wire.h"
#include <Arduino.h>

const int buzzerPin = 32; // buzzer is on GPIO 32

void setup() {
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Plays a C4 note for 300ms
  tone(buzzerPin, 261.6, 300);
  tone(buzzerPin, 293.7, 300);
  tone(buzzerPin, 329.6, 300);
  tone(buzzerPin, 349.2, 300);
  tone(buzzerPin, 392.0, 300);
  tone(buzzerPin, 440.0, 300);
  tone(buzzerPin, 493.9, 300);
  tone(buzzerPin, 523.3, 300);
  delay(350); // Wait for sound to finish and a short pause
}