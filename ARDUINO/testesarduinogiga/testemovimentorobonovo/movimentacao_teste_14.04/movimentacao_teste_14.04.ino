//m7
#include "Arduino.h"
#include "RPC.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Ultrasonic.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int erromover = 0;
int target = 0;
float kpm = 0.9;
float erro = 0;
int ir;
int cx;
int cy;


int getAngle_()
{
  sensors_event_t event;
  bno.getEvent(&event);

  // Leitura bruta do eixo Z
  float angulogiro = event.orientation.x;

  if (angulogiro > 180) angulogiro -= 360;

  return (int)angulogiro;
}


// Motor PWM and Direction pin assignments
const int DIR_PIN_FL1 = 37;  // Front-left motor
const int DIR_PIN_FR1 = 22;  // Front-right motor
const int DIR_PIN_RL1 = 35;  // Rear-left motor
const int DIR_PIN_RR1 = 30;  // Rear-right motor
const int DIR_PIN_FL2 = 39;
const int DIR_PIN_FR2 = 24;
const int DIR_PIN_RL2 = 33;
const int DIR_PIN_RR2 = 32;
const int PWM_PIN_FL = 4;
const int PWM_PIN_FR = 8;
const int PWM_PIN_RL = 6;
const int PWM_PIN_RR = 5;


const float sen45 = sqrt(2) / 2;
const float cos45 = sqrt(2) / 2;


void setMotor(int pwmPin, int dirPin1, int dirPin2, float velocidade, int sentido) {
  if (velocidade == 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    analogWrite(pwmPin, 0);
    return;
  }
  if (!sentido) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  }
  analogWrite(pwmPin, abs(velocidade) > 255 ? 255 : abs(velocidade));
}


int sentidomotor(float w) {
  return (w > 0) ? 1 : 0;
}

void mover(int vel, int angulo) {
  float wfe, wfd, wte, wtd;
  int wfe_v, wfd_v, wte_v, wtd_v;

  float angulorad = radians(angulo);
  float vx = vel * cos(angulorad);
  float vy = vel * sin(angulorad);


  wfe = (cos45 * vx + sen45 * vy);
  wfd = (-cos45 * vx + sen45 * vy);
  wte = (cos45 * vx - sen45 * vy);
  wtd = (-cos45 * vx - sen45 * vy);

  wfe_v = wfe + erromover;
  wfd_v = wfd - erromover; //daquele jeito o erro so diminuia a velocidade igual em todos os motores, tem que ser assim
  wte_v = wte + erromover;
  wtd_v = wtd - erromover;

  Serial.print(wfe_v);
  Serial.print(" ");
  Serial.print(wfd_v);
  Serial.print(" ");
  Serial.print(wte_v);
  Serial.print(" ");
  Serial.print(wtd_v);
  Serial.println(" ");

  setMotor(PWM_PIN_FL, DIR_PIN_FL1, DIR_PIN_FL2, wfe_v, sentidomotor(wfe_v));
  setMotor(PWM_PIN_FR, DIR_PIN_FR1, DIR_PIN_FR2, wfd_v, sentidomotor(wfd_v));
  setMotor(PWM_PIN_RL, DIR_PIN_RL1, DIR_PIN_RL2, wte_v, sentidomotor(wte_v));
  setMotor(PWM_PIN_RR, DIR_PIN_RR1, DIR_PIN_RR2, wtd_v, sentidomotor(wtd_v));
}

void parar() {
  digitalWrite(DIR_PIN_FL1, HIGH);
  digitalWrite(DIR_PIN_FL2, HIGH);
  digitalWrite(DIR_PIN_FR1, HIGH);
  digitalWrite(DIR_PIN_FR2, HIGH);
  digitalWrite(DIR_PIN_RL1, HIGH);
  digitalWrite(DIR_PIN_RL2, HIGH);
  digitalWrite(DIR_PIN_RR1, HIGH);
  digitalWrite(DIR_PIN_RR2, HIGH);
  analogWrite(PWM_PIN_FL, 0);
  analogWrite(PWM_PIN_FR, 0);
  analogWrite(PWM_PIN_RL, 0);
  analogWrite(PWM_PIN_RR, 0);
}

void pegarDirecao()
{
  if (ir != 1 && ir != 0) {
    cx = cos(radians((ir - 1) * 22));
    cy = sin(radians((ir - 1) * 22));
  }
  else {
    cx = cos(0);
    cy = sin(0);
  }
}

void moverAtras()
{
  if (ir >= 2 && ir <= 4 )
  {
    mover(200, 90);
    delay (25);
    parar();
  }
  if (ir >= 5 && ir <= 8 || ir >= 12 && ir <= 14 )
  {

    mover(200, 210); //180
    delay (40);

  }
  if (ir >= 17 && ir <= 15)
  {

    if (cx < cos(radians(22)))
    {
      mover(200, 270); // velocidade vai mudar de acordo com a disrancia do sensor ate a bola, depois a gente muda
      delay (25);
      parar();
      //delay (2);
      //return;
    }
    else
    {

      mover(200, 0);
      delay(35);//ir mais para frente
      //parar();
      //return;
    }
  }
  if (ir >= 9 && ir <= 11)
  {

    if (cx < cos(radians(22)))// 180 - 50
    {


      mover(200, 90); // velocidade vai mudar de acordo com a disrancia do sensor ate a bola, depois a gente muda
      delay(30);
      parar();
      delay (2);
      //return;
    }
    else
    {
      mover(200, 0);
      delay(35);//ir mais para frente
      //parar();
      //return;
    }
  }
}

void setup() {

  pinMode(PWM_PIN_FL, OUTPUT);
  pinMode(PWM_PIN_FR, OUTPUT);
  pinMode(PWM_PIN_RL, OUTPUT);
  pinMode(PWM_PIN_RR, OUTPUT);
  pinMode(DIR_PIN_FL1, OUTPUT);
  pinMode(DIR_PIN_FR1, OUTPUT);
  pinMode(DIR_PIN_RL1, OUTPUT);
  pinMode(DIR_PIN_RR1, OUTPUT);
  pinMode(DIR_PIN_FL2, OUTPUT);
  pinMode(DIR_PIN_FR2, OUTPUT);
  pinMode(DIR_PIN_RL2, OUTPUT);
  pinMode(DIR_PIN_RR2, OUTPUT);


  RPC.begin();
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.println("Erro ao inicializar o BNO055!");
    while (1); // Se o sensor não inicializar, trava a execução
  }
  mover(50, 0);
  delay(15);
  parar();
}

void loop() {
  float angulolido = getAngle_();
  Serial.println(angulolido);
  erro = target - angulolido;
  erromover = kpm * erro;
  if (erromover > 50) erromover = 50;
  if (erromover < -50) erromover = -50;
  /*  ir = RPC.call("send_ir").as<int>();
    if (ir > -1) {
      Serial.print("Ir: ");

      Serial.println(ir);
      delay(100);

      pegarDirecao();
      moverAtras();
    }*/
  mover(200, 0);
}
