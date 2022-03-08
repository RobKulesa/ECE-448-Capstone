#include <Arduino.h>
#include <HardwareSerial.h>


uint8_t serveRequests = 0;
uint8_t impulsesLogged = 0;
uint8_t firstImpulse = 0;
uint8_t secondImpulse = 0;
uint32_t firstTimeStamp = 0;
uint32_t secondTimeStamp = 0;


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

// Define hardware connections
const uint8_t gatePins[] = {2, 3, 21};

void soundISR0() {
  ISR_MACRO(gatePins[0]);
}
void soundISR1() {
  ISR_MACRO(gatePins[1]);
}
void soundISR2() {
  ISR_MACRO(gatePins[2]);
}

void logISR(byte pin) {
  int pin_val = digitalRead(pin);
  char buffer[12];
  sprintf(buffer, "Pin %d: %d", pin, pin_val);
  Serial.println(buffer);
}

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

void loop() { 
  while(impulsesLogged < 2) {
    Serial.println("Waiting for requests");
    delay(1000);
  }
  uint32_t timeDiff;
  if(firstTimeStamp > secondTimeStamp) timeDiff = UINT32_MAX - firstTimeStamp + secondTimeStamp;
  else timeDiff = secondTimeStamp - firstTimeStamp;

  char buffer[33];
  sprintf(buffer, "Time difference (us): %lu", timeDiff);
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