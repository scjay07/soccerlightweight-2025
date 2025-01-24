#include "Arduino.h"
#include "RPC.h"

using namespace rtos;


void setup() {
  RPC.begin();
  Serial.begin(9600);
  pinMode(6, OUTPUT);
  pinMode(7,INPUT_PULLDOWN);

  /*
  Starts a new thread that loops the requestServoMove() function
  */
}

void loop() {
  
  digitalWrite(6,HIGH);
  if((digitalRead(7)) == HIGH)
  {
  RPC.call("ligarled");
  RPC.println("oi");
  }
  else{
    RPC.call("desligarled");
    }
}

/*
This thread calls the servoMove() function remotely
every second, passing the angle variable (0-180).
*/
