#include <Arduino.h>
#include <HardwareSerial.h>


byte serveRequests = 0;
byte impulsesLogged = 0;
byte firstImpulse = 0;
byte secondImpulse = 0;
int firstTimeStamp = 0;
int secondTimeStamp = 0;


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
const byte gatePins[] = {2, 3, 21};

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
  sprintf_s(buffer, 12, "Pin %d: %d", gatePins[0], pin_val);
  Serial.println(buffer);
}

void setup() {
  Serial.begin(9600);
  for (byte i = 0; i < 3; i++) {
    pinMode(gatePins[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(gatePins[0]), soundISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gatePins[1]), soundISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gatePins[2]), soundISR2, CHANGE);
  serveRequests = 1;
  Serial.println("Ready to serve requests");
}

void loop() {
  Serial.println("Waiting for requests");
  delay(1000);
}