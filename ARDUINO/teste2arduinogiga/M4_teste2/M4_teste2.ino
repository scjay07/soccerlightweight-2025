//M4
#include "Arduino.h"
#include "RPC.h"

void ligarled()
{
  digitalWrite(4, HIGH);
  delay(1000);
  digitalWrite(4, LOW);
  delay(1000);
}
void desligarled()
{
  digitalWrite(4, LOW);
}
void setup() {
  // put your setup code here, to run once:
  RPC.begin();
  Serial.begin(9600);
  RPC.bind("ligarled", ligarled);
  RPC.bind("desligarled", desligarled);
  pinMode(4, OUTPUT);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  String buffer = "";
  while (RPC.available()) 
  {
    buffer += (char)RPC.read();  // Fill the buffer with characters
  }
  if (buffer.length() > 0) {
    Serial.print(buffer);
  }
}
