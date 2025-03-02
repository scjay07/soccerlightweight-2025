//M7
#include "Arduino.h"
#include "RPC.h"
#include <LCD_I2C.h>
#include <Wire.h>    //Include wire library 
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


//BNO
int angulocorrecao;
int erromover;
int angulolido;
int anguloyaw;
int target;
float kp = 0.27;
float kpm = 1.3;
int erro = 0;
float angleZ = 0;          // Estimativa do ângulo
float biasZ = 0;           // Desvio de bias
float rateZ = 0;           // Taxa de mudança angular
float P = 1;               // Matriz de covariância
float Q = 0.01;            // Process noise (incerteza do processo)
float R = 0.01;             // Medição de ruído (incerteza das medições)
float K = 0;               // Ganho de Kalman
unsigned long tempoinicial = 0;
bool alinhando = false;

//IR
int ir;
int bola;
int gol;
int IR[16];
int valor[16];
const int portas[16] =  {15, 4, 5, 18, 19, 36, 39, 23, 13, 14, 26, 25, 33, 32, 35, 34};
const int minimo[16] = {4026, 4114, 4302, 4617, 4413, 4241, 4310, 4664, 4928, 4030, 4330, 4867, 4082, 4502, 4128, 4280};
//4332,4173,4341,4493,-1,2791,4398,4194,4283,4167,4161,4097,4098,4422,4255,4847
const int MAX = 10000;

// Motor PWM and Direction pin assignments
const int DIR_PIN_FL1 = 28;  // Front-left motor
const int DIR_PIN_FR1 = 34;  // Front-right motor
const int DIR_PIN_RL1 = 26;  // Rear-left motor
const int DIR_PIN_RR1 = 36;  // Rear-right motor
const int DIR_PIN_FL2 = 29;
const int DIR_PIN_FR2 = 35;
const int DIR_PIN_RL2 = 27;
const int DIR_PIN_RR2 = 37;
const int PWM_PIN_FL = 12;
const int PWM_PIN_FR = 8;
const int PWM_PIN_RL = 11;
const int PWM_PIN_RR = 6;

//MOVER1
const float sen45 = sqrt(2) / 2;
const float cos45 = sqrt(2) / 2;
float wfe, wfd, wte, wtd;
byte wfe_v, wfd_v, wte_v, wtd_v;
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


void girareixoH() {
  digitalWrite(DIR_PIN_FL1, HIGH);
  digitalWrite(DIR_PIN_FL2, LOW);
  analogWrite(PWM_PIN_FL, 40);
  digitalWrite(DIR_PIN_FR1, HIGH);
  digitalWrite(DIR_PIN_FR2, LOW);
  analogWrite(PWM_PIN_FR, 40);
  digitalWrite(DIR_PIN_RL1, HIGH);
  digitalWrite(DIR_PIN_RL2, LOW);
  analogWrite(PWM_PIN_RL, 40);
  digitalWrite(DIR_PIN_RR1, HIGH);
  digitalWrite(DIR_PIN_RR2, LOW);
  analogWrite(PWM_PIN_RR, 40);
}


void girareixoA() {
  digitalWrite(DIR_PIN_FL1, LOW);
  digitalWrite(DIR_PIN_FL2, HIGH);
  analogWrite(PWM_PIN_FL, 40);
  digitalWrite(DIR_PIN_FR1, LOW);
  digitalWrite(DIR_PIN_FR2, HIGH);
  analogWrite(PWM_PIN_FR, 40);
  digitalWrite(DIR_PIN_RL1, LOW);
  digitalWrite(DIR_PIN_RL2, HIGH);
  analogWrite(PWM_PIN_RL, 40);
  digitalWrite(DIR_PIN_RR1, LOW);
  digitalWrite(DIR_PIN_RR2, HIGH);
  analogWrite(PWM_PIN_RR, 40);
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


  sentidomotor(wfe, 0);
  sentidomotor(wfd, 1);
  sentidomotor(wte, 2);
  sentidomotor(wtd, 3);


  float maxVel = max(max(abs(wfe), abs(wfd)), max(abs(wte), abs(wtd))) + 20;


  wfe_v = (byte)map(abs(wfe), 0, maxVel, 80, 255) - erromover;
  wfd_v = (byte)map(abs(wfd), 0, maxVel, 100, 255) - erromover;
  wte_v = (byte)map(abs(wte), 0, maxVel, 80, 255) - erromover;
  wtd_v = (byte)map(abs(wtd), 0, maxVel, 80, 255) - erromover;


  setMotor(PWM_PIN_FL, DIR_PIN_FL1, DIR_PIN_FL2, wfe_v, s[0]);
  setMotor(PWM_PIN_FR, DIR_PIN_FR1, DIR_PIN_FR2, wfd_v, s[1]);
  setMotor(PWM_PIN_RL, DIR_PIN_RL1, DIR_PIN_RL2, wte_v, s[2]);
  setMotor(PWM_PIN_RR, DIR_PIN_RR1, DIR_PIN_RR2, wtd_v, s[3]);
}



void girar_bola()
{
  digitalWrite(DIR_PIN_FL1, LOW);
  digitalWrite(DIR_PIN_FL2, HIGH);
  analogWrite(PWM_PIN_FL, 200);
  digitalWrite(DIR_PIN_RR1, HIGH);
  digitalWrite(DIR_PIN_RR2, LOW);
  analogWrite(PWM_PIN_RR, 200);
  delay(150);
  parar();
  delay(10);
  digitalWrite(DIR_PIN_FR1, LOW);
  digitalWrite(DIR_PIN_FR2, HIGH);
  analogWrite(PWM_PIN_FR, 200);
  digitalWrite(DIR_PIN_RL1, HIGH);
  digitalWrite(DIR_PIN_RL2, LOW);
  analogWrite(PWM_PIN_RL, 200);
  delay(150);
  parar();
  delay(100);
  digitalWrite(DIR_PIN_FR1, LOW);
  digitalWrite(DIR_PIN_FR2, LOW);
  analogWrite(PWM_PIN_FR, 0);
  digitalWrite(DIR_PIN_RL1, HIGH);
  digitalWrite(DIR_PIN_RL2, LOW);
  analogWrite(PWM_PIN_RL, 200);
  digitalWrite(DIR_PIN_RR1, LOW);
  digitalWrite(DIR_PIN_RR2, LOW);
  analogWrite(PWM_PIN_RR, 0);
  digitalWrite(DIR_PIN_FL1, LOW);
  digitalWrite(DIR_PIN_FL2, LOW);
  analogWrite(PWM_PIN_FL, 0);
  delay(600);
}



void alinhar() {
  //
  //  if (abs(angulolido) > 0 && abs(angulolido) <= 2) {
  //    kp *= 0;
  //  }
  //  else if (angulocorrecao < 0 && angulocorrecao >= -2) {
  //    kp *= 1.02;
  //  }
  //  else if (angulocorrecao >= 6 || angulocorrecao <= -6) {
  //    kp *= 0.2;
  //  }
  //  else {
  //    kp = 0.134;
  //  }
  wfe = angulocorrecao;
  wte = angulocorrecao;
  wfd = angulocorrecao;
  wtd = angulocorrecao;
  wfe_v = (byte)map(abs(wfe), 0, 180, 18, 200);
  wfd_v = (byte)map(abs(wfd), 0, 180, 18, 200);
  wte_v = (byte)map(abs(wte), 0, 180, 18, 200);
  wtd_v = (byte)map(abs(wtd), 0, 180, 18, 200);
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
  // put your setup code here, to run once:
  RPC.begin();
  Serial.begin(9600);

  for (int i = 0; i < 16; i++)
  {
    //    if (i == 9)
    //      pinMode(12, INPUT_PULLDOWN); //porta12 (sensor 9)
    //    else
    pinMode(portas[i], INPUT);
  }
  for (int i = 0; i < 16; i++)
  {
    for (int j = 0; j < MAX; j++) IR[i] = 0;
  }

  Wire.begin();
  if (!bno.begin()) {
    Serial.print("Não foi possível encontrar o sensor BNO055");
    while (1);
  }

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

  Serial.println("INICIADO");
  //mover(100, 45);
  delay(300);

}

void loop() {
  // put your main code here, to run repeatedly:
  //leitura de ir
  int index = -1;
  for (int i = 0, min_ = 10000; i < 16; i++)
  {
    for (int j = 0; j <= MAX; j++) IR[i] += digitalRead(portas[i]);
    IR[i] = map(IR[i], minimo[i], 10000, 0, 100);

    if (IR[i] < min_ && IR[i] >= 0)
    {
      min_ = IR[i];
      index = i;
    }
  }
  sensors_event_t event;
  bno.getEvent(&event);

  // Leitura bruta do eixo Z
  float rawZ = event.orientation.x;
  // Taxa de mudança angular
  rateZ = rawZ - angleZ;
  // Previsão do próximo valor de ângulo
  angleZ += rateZ;
  // Atualizar a covariância P
  P += Q;
  // Ganho de Kalman
  K = P / (P + R);
  // Correção com a medição (rawZ)
  angleZ += K * (rawZ - angleZ);
  // Atualiza a covariância P após a correção
  P = (1 - K) * P;

  angulolido =  angleZ;
  if (angulolido > 180) angulolido -= 360;
  erro = target - angulolido;
  angulocorrecao = kp * erro;
  erromover = kpm * erro;

  if (abs(angulolido) > 15)
  { //se o robô estiver desalinhado em até 50 graus, então alinha);
    tempoinicial = millis();
    while ((abs(angulolido) > 15 && abs(angulolido) < 345) && (millis() - tempoinicial) <= 75)
    {
      mudar_cor(0, 0, 255);
      msg = Serial2.parseInt();
      if (msg >= 2640 && msg <= 3360) {
        angulolido = (msg - 3000);
        erro = target - angulolido;
        angulocorrecao = kp * erro;
        alinhar();
        //parar();
      }
    }
  }
  else {
    switch (ir) {
      case 1: //IR1
        Serial.println("0 graus");
        parar();
        mover(30, 0);
        delay(20);
        break;
      case 2:
        Serial.println("315 graus");
        mover(30, 315);
        delay(20);
        break;
      case 3:
        Serial.println("292 graus");
        mover(30, 292);
        delay(20);
        break;
      case 4:
        Serial.println("270 graus");
        mover(30, 270);
        delay(20);
        break;
      case 5:
        Serial.println("247 graus");
        mover(30, 247);
        delay(20);
        break;
      case 6:
        Serial.println("225 graus");
        mover(30, 225);
        delay(20);
        break;
      case 7:
        Serial.println("202 graus");
        mover(30, 202);
        delay(20);
        break;
      case 8:
        Serial.println("180 graus");
        mover(30, 180);
        delay(20);
        break;
      case 9:
        Serial.println("157 graus");
        mover(30, 157);
        delay(20);
        break;
      case 10:
        Serial.println("135 graus");
        mover(30, 135);
        delay(20);
        break;
      case 11:
        Serial.println("112 graus");
        mover(30, 112);
        delay(20);
        break;
      case 12:
        Serial.println("90 graus");
        mover(30, 90);
        delay(20);
        break;
      case 13:
        Serial.println("67 graus");
        mover(30, 67);
        delay(20);
        break;
      case 14:
        Serial.println("45 graus");
        mover(30, 45);
        delay(20);
        break;
      case 15:
        Serial.println("22 graus");
        mover(30, 22);
        delay(20);
        break;
      case 16:
        Serial.println("0 graus");
        mover(30, 0);
        delay(20);
        break;
      default:
        Serial.println("Invalido");
    }
  }
  for (int k = 0; k < 16; k++)
  {
    IR[k] = 0;
  }
}
