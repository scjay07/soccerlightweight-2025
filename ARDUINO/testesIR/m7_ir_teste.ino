//M7
#include "Arduino.h"
#include "RPC.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //pinMode(13, INPUT);
  RPC.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
  int msg = RPC.call("send_ir").as<int>();
  if (msg > -1) {
    Serial.print("Ir: ");

    Serial.println(msg);
    delay(100);
  }
}
