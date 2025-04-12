

///A MARI NAO SABE SALVAR

#include "Arduino.h"
#include "RPC.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Ultrasonic.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int angulolido;
int angulocorrecao;
int erromover;
int target = 0;
float kp = 0.285;
float kpm = 4;
int erro = 0;
int ir;
int bola;
int gol;
unsigned long tempoinicial = 0;
bool alinhando = false;
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
float wfe, wfd, wte, wtd;
int wfe_v, wfd_v, wte_v, wtd_v;
int s[4] = { 1, 1, 1, 1 };


void setMotor(int pwmPin, int dirPin1, int dirPin2, float velocidade, int sentido) {
  if (velocidade == 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    analogWrite(pwmPin, 0);
    return;
  }
  if (sentido == 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
  }
  analogWrite(pwmPin, abs(velocidade));
}


int sentidomotor(float w, int indice) {
  s[indice] = (w > 0) ? 1 : 0;
  return s[indice];
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


void mover(int vel, int angulo) {
  float angulorad = radians(angulo);
  float vx = vel * cos(angulorad);
  float vy = vel * sin(angulorad);


  wfe = (cos45 * vx + sen45 * vy);
  wfd = (-cos45 * vx + sen45 * vy);
  wte = (cos45 * vx - sen45 * vy);
  wtd = (-cos45 * vx - sen45 * vy);

  float maxVel = max(max(abs(wfe), abs(wfd)), max(abs(wte), abs(wtd))) + 20;


  wfe_v = (map(abs(wfe), 0, maxVel, 0, 255))- erromover; //mudar de zero para 70
  wfd_v = (map(abs(wfd), 0, maxVel, 0, 255))- erromover;
  wte_v = (map(abs(wte), 0, maxVel, 0, 255))- erromover;
  wtd_v = (map(abs(wtd), 0, maxVel, 0, 255))- erromover;

  
  sentidomotor(wfe, 0);
  sentidomotor(wfd, 1);
  sentidomotor(wte, 2);
  sentidomotor(wtd, 3);

  
  if(abs(wfe_v) > 255) wfe_v = 255;
  if(abs(wfd_v) > 255) wfd_v = 255;
  if(abs(wte_v) > 255) wte_v = 255;
  if(abs(wtd_v) > 255) wtd_v = 255;

  //Serial.println (erromover);
  
    Serial.print(wfe_v);
    Serial.print(" ");
    Serial.print(wfd_v);
    Serial.print(" ");
    Serial.print(wte_v);
    Serial.print(" ");
    Serial.print(wtd_v);
    Serial.println(" ");
  

  setMotor(PWM_PIN_FL, DIR_PIN_FL1, DIR_PIN_FL2, abs(wfe_v), s[0]);
  setMotor(PWM_PIN_FR, DIR_PIN_FR1, DIR_PIN_FR2, abs(wfd_v), s[1]);
  setMotor(PWM_PIN_RL, DIR_PIN_RL1, DIR_PIN_RL2, abs(wte_v), s[2]);
  setMotor(PWM_PIN_RR, DIR_PIN_RR1, DIR_PIN_RR2, abs(wtd_v), s[3]);
}

void pegarDirecao()
{
  if (ir != 16 && ir != 1) {
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
    mover(30, 90);
    delay (25);
    parar();
  }
  if (ir >= 5 && ir <= 8 || ir >= 12 && ir <= 14 )
  {

    mover (30, 210); //180
    delay (40);

  }
  if (ir >= 17 && ir <= 15)
  {

    if (cx < cos(radians(22)))
    {
      mover(30, 270); // velocidade vai mudar de acordo com a disrancia do sensor ate a bola, depois a gente muda
      delay (25);
      parar();
      //delay (2);
      //return;
    }
    else
    {

      mover(30, 0);
      delay(35);//ir mais para frente
      //parar();
      //return;
    }
  }
  if (ir >= 9 && ir <= 11)
  {

    if (cx < cos(radians(22)))// 180 - 50
    {


      mover(30, 90); // velocidade vai mudar de acordo com a disrancia do sensor ate a bola, depois a gente muda
      delay(30);
      parar();
      delay (2);
      //return;
    }
    else
    {
      mover(30, 0);
      delay(35);//ir mais para frente
      //parar();
      //return;
    }
  }
}



void alinhar() {

  if (abs(angulolido) > 20 && abs(angulolido) <= 25) {
    kp *= 1.19;
  }
  //    else if (angulocorrecao < 0 && angulocorrecao >= -2) {
  //      kp *= 1.02;
  //    }
  //    else if (angulocorrecao >= 6 || angulocorrecao <= -6) {
  //      kp *= 0.2;
  //    }
  else {
    kp = 0.285;
  }

  wte = angulocorrecao;
  wfe = angulocorrecao;
  wfd = angulocorrecao;
  wtd = angulocorrecao;
  wfe_v = (byte)map(abs(wfe), 0, 180, 27, 200);
  wfd_v = (byte)map(abs(wfd), 0, 180, 27, 200);
  wte_v = (byte)map(abs(wte), 0, 180, 27, 200);
  wtd_v = (byte)map(abs(wtd), 0, 180, 27, 200);
  sentidomotor(wfe, 0);
  sentidomotor(wfd, 1);
  sentidomotor(wte, 2);
  sentidomotor(wtd, 3);
  setMotor(PWM_PIN_FL, DIR_PIN_FL1, DIR_PIN_FL2, wfe_v, s[0]);
  setMotor(PWM_PIN_FR, DIR_PIN_FR1, DIR_PIN_FR2, wfd_v, s[1]);
  setMotor(PWM_PIN_RL, DIR_PIN_RL1, DIR_PIN_RL2, wte_v, s[2]);
  setMotor(PWM_PIN_RR, DIR_PIN_RR1, DIR_PIN_RR2, wtd_v, s[3]);
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

}

void loop() {

  angulolido = getAngle_();
  //Serial.println(angulolido);
  erro = target - angulolido;
  angulocorrecao = kp * erro;
  erromover = kpm * erro;

  mover (70, 0);

  //  Serial.println (angulolido);
  //
  //  ir = RPC.call("send_ir").as<int>();
  //  if (ir > -1) {
  //    Serial.print("Ir: ");
  //
  //    Serial.println(ir);
  //    delay(100);
  //  }
  //
  //
  //   if (abs(angulolido) > 26)
  //    { //se o robô estiver desalinhado em até 50 graus, então alinha);
  //     tempoinicial = millis();
  //     while ((abs(angulolido) > 26 && abs(angulolido) < 334 ) && (millis() - tempoinicial) <= 75)
  //     {
  //       angulolido = getAngle_();
  //       erro = target - angulolido;
  //       angulocorrecao = kp * erro;
  //       alinhar();
  //     }
  //    }
  //
  //    else
  //    {
  //     pegarDirecao();
  //     moverAtras();
  //    }
}
