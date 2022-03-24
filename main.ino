#include <Arduino.h>
#include <HardwareSerial.h>
#include <Math.h>


// Define global variables
uint8_t serveRequests = 0;
uint8_t impulsesLogged = 0;
uint8_t firstImpulse = 0;
uint8_t secondImpulse = 0;
uint32_t firstTimeStamp = 0;
uint32_t secondTimeStamp = 0;
const uint8_t micDistance = 5; //inches
const double speedOfSound = 0.0135039; //inches per microsecond

// Define macro function for updating global variables upon interrupt events
#define ISR_MACRO(pin) {\
  if(serveRequests) {\
    if(impulsesLogged == 0) {\
      firstTimeStamp = micros();\
      firstImpulse = pin;\
      impulsesLogged += 1;\
    } else if(impulsesLogged == 1) {\
      serveRequests = 0;\
      secondTimeStamp = micros();\
      secondImpulse = pin;\
      impulsesLogged += 1;\
    }\
    logISR(pin);\
  }\
}

// Define hardware-arduino connections
const uint8_t gatePins[] = {2, 3, 21};

// Interrupt Service Routines specific to the sound detector gate pins
void soundISR0() {
  ISR_MACRO(gatePins[0]);
}
void soundISR1() {
  ISR_MACRO(gatePins[1]);
}
void soundISR2() {
  ISR_MACRO(gatePins[2]);
}

// Print pin number and digital pin value to serial monitor upon interrupt
void logISR(byte pin) {
  int pin_val = digitalRead(pin);
//  char buffer[12];
//  sprintf(buffer, "Pin %d: %d", pin, pin_val);
//  Serial.println(buffer);
}

// Set up the sound detector pins for input and attach respective ISRs upon digital rising edge
// and begin serving requests for interrupts
void setup() {
  Serial.begin(9600);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(gatePins[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(gatePins[0]), soundISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(gatePins[1]), soundISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(gatePins[2]), soundISR2, RISING);
  serveRequests = 1;
  Serial.println("Ready to serve requests");
}

// When two impulses are logged, print the time difference between the first and second impulse
// and reset the global variables
void loop() { 
  while(impulsesLogged < 2) {
    Serial.println("Waiting for requests");
    delay(1000);
  }
  uint32_t timeDiff;
  if(firstTimeStamp > secondTimeStamp) timeDiff = UINT32_MAX - firstTimeStamp + secondTimeStamp;
  else timeDiff = secondTimeStamp - firstTimeStamp;

  uint8_t theta = (180/M_PI) * asin((timeDiff * speedOfSound) / micDistance);
  char buffer[150];
  sprintf(buffer, "Impulse 1: %hu\nImpulse 2: %hu\nTime difference (us): %lu\nTheta: %hu", firstImpulse, secondImpulse, timeDiff, theta);
  Serial.println(buffer);
  
  clearLog();
  serveRequests = 1;
}

void clearLog() {
  impulsesLogged = 0;
  firstImpulse = 0;
  secondImpulse = 0;
  firstTimeStamp = 0;
  secondTimeStamp = 0;
}