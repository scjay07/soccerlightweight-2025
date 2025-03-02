#include "Arduino.h"
#include "RPC.h"
#include <Servo.h>

Servo myservo;

void setup() {
  RPC.begin();
  myservo.attach(5); //attach servo to pin 5

  Serial.begin(115200);

  //Bind the servoMove() function on the M7
  RPC.bind("servoMove", servoMove);
}

void loop() {
  // On M7, let's print everything that is received over the RPC1 stream interface
  // Buffer it, otherwise all characters will be interleaved by other prints
}

/*
Function on the M7 that returns an analog reading (A0)
*/
int servoMove(int angle) {
  myservo.write(angle);
  delay(10);
  return angle;
  /*
  After the operation is done, return angle to the client.
  The value passed to this function does not change, but this
  verifies it has been passed correctly.
  */
}
