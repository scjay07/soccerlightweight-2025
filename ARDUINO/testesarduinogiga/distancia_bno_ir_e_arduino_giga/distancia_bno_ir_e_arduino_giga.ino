#include "Arduino.h"
#include "RPC.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Ultrasonic.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Ultrasonic ultrasonic(48, 46);
Ultrasonic ultrasonic2(45, 47);
Ultrasonic ultrasonic3(25, 27);
int distance;
int distance2;
int distance3;

int getAngle_()
{
  sensors_event_t event;
  bno.getEvent(&event);

  // Leitura bruta do eixo Z
  float angulogiro = event.orientation.x;

  if (angulogiro > 180) angulogiro -= 360;

  return (int)angulogiro;
}

void setup() {
  // put your setup code here, to run once:
  RPC.begin();
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("Erro ao inicializar o BNO055!");
    while (1); // Se o sensor não inicializar, trava a execução
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  int angle = getAngle_();
  Serial.print("Angle: ");
  Serial.println(angle);

  distance = ultrasonic.read();
  distance2 = ultrasonic2.read();
  distance3 = ultrasonic3.read();

  int msg = RPC.call("send_ir").as<int>();
  if (msg > -1) {
    Serial.print("Ir: ");

    Serial.println(msg);
    delay(100);
  }

  Serial.print("Distance in CM 1: ");
  Serial.println(distance);
  Serial.print("Distance in CM 2: ");
  Serial.println(distance2);
  Serial.print("Distance in CM 3: ");
  Serial.println(distance3);
  delay(50);
}
