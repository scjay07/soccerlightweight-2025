https://github.com/scjay07/soccerlightweight-2025/tree/teste/M4_codigos/M4_teste1#include "Arduino.h"
#include "RPC.h"

using namespace rtos;

Thread servoThread;

void setup() {
  RPC.begin();
  Serial.begin(115200);

  /*
  Starts a new thread that loops the requestServoMove() function
  */
  servoThread.start(requestServoMove);
}

void loop() {
}

/*
This thread calls the servoMove() function remotely
every second, passing the angle variable (0-180).
*/
void requestServoMove() {
  while (true) {
    //Read a pot meter
    //Map value to 180
    delay(1000);
    auto result = RPC.call("servoMove", 90).as<int>();
    RPC.println("Servo angle is: " + String(result));
  }
}
