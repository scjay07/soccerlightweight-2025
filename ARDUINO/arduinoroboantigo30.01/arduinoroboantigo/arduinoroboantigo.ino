//GK ARDUINO JOGO 2

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led(32, A11, NEO_GBR + NEO_KHZ800);


//const int minluz[] = {16,8,19,14,7,15,11,3,16,8,16};
//const int maxluz[] = {74,44,108,98,49,101,87,7,120,87,118};

const int minluz[] = {11, 5, 15, 11, 4, 13, 7, 0, 16, 7, 13};
const int maxluz[] = {40, 39, 26, 20, 13, 27, 15, 1, 35, 13, 39};


int x;
int IR[10];
const int tetocor = 49;
const int tetocor1 = 37;
const int tetocor3 = 50;
const int portasluz[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10 };
float valorluz[11];
int angulolido;
int angulocorrecao;
int target;
float kp = 0.5;
int erro = 0;
int ir;
int bola;
int gol;


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


  wfe = (cos45 * vx + sen45 * vy) - angulocorrecao;
  wfd = (-cos45 * vx + sen45 * vy) - angulocorrecao;
  wte = (cos45 * vx - sen45 * vy) - angulocorrecao;
  wtd = (-cos45 * vx - sen45 * vy) - angulocorrecao;


  sentidomotor(wfe + angulocorrecao, 0);
  sentidomotor(wfd + angulocorrecao, 1);
  sentidomotor(wte + angulocorrecao, 2);
  sentidomotor(wtd + angulocorrecao, 3);


  float maxVel = max(max(abs(wfe), abs(wfd)), max(abs(wte), abs(wtd))) + 20;


  wfe_v = (byte)map(abs(wfe), 0, maxVel, 60, 255);
  wfd_v = (byte)map(abs(wfd), 0, maxVel, 60, 255);
  wte_v = (byte)map(abs(wte), 0, maxVel, 60, 255);
  wtd_v = (byte)map(abs(wtd), 0, maxVel, 60, 255);


  setMotor(PWM_PIN_FL, DIR_PIN_FL1, DIR_PIN_FL2, wfe_v, s[0]);
  setMotor(PWM_PIN_FR, DIR_PIN_FR1, DIR_PIN_FR2, wfd_v, s[1]);
  setMotor(PWM_PIN_RL, DIR_PIN_RL1, DIR_PIN_RL2, wte_v, s[2]);
  setMotor(PWM_PIN_RR, DIR_PIN_RR1, DIR_PIN_RR2, wtd_v, s[3]);
}


void mudarluz(int b, int g, int r) {
  for (int k = 0; k < 32; k++) {
    led.setPixelColor(k, led.Color(g, b, r));
  }
  led.show();
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
  led.begin();
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial2.setTimeout(5);
  for (int k = 0; k < 32; k++) {
    led.setPixelColor(k, led.Color(0, 0, 255));
  }
  led.show();
  for (int i = 0; i < 11; i++) {
    pinMode(portasluz[i], INPUT);
  }
  Serial.println("INICIADO");
  mover (100, 45);
  delay (600);
}


void loop() {
  erro = target - angulolido;
  angulocorrecao = kp * erro;
  for (int i = 0; i < 11; i++) {
    valorluz[i] = analogRead(portasluz[i]);
    valorluz[i] = (int)map(valorluz[i], minluz[i], maxluz[i], 0, 50);
    valorluz[i] = constrain(valorluz[i], 0, 50);
  }
  //  if (valorluz[2] >= tetocor3) //|| valorluz[3] >= tetocor3 || valorluz[4] >= tetocor3) // o sensor 4 ta aumentando muito quando liga o robo
  //  {
  //
  //    //    //vai para leste
  //    target = angulolido;
  //    //    for (int k = 0; k <= 32; k++) {
  //    //      led.setPixelColor(k, led.Color(0, 255, 0));
  //    //    }
  //    //    led.show();
  //    Serial.println("leste");
  //    parar();
  //    delay(100);
  //    mover(20, 90);
  //    delay(300);
  //    parar();
  //    delay(100);
  //  }
  //  else if (valorluz[5] >= tetocor || valorluz[6] >= tetocor || valorluz[7] >= tetocor)
  //  {
  //    //vai para norte
  //    mover(50, 0);
  //    delay(300);
  //    parar();
  //    delay(100);
  //    Serial.println("norte");
  //  }
  //  else if (//valorluz[8] >= tetocor ||valorluz[9] >= tetocor ||
  //    valorluz[10] >= tetocor)
  //  {
  //    //vai para oeste
  //    target = angulolido;
  //    //    for (int k = 0; k <= 32; k++) {
  //    //      led.setPixelColor(k, led.Color(255, 0, 0));
  //    //    }
  //    //    led.show();
  //    Serial.println("oeste");
  //    parar();
  //    delay(100);
  //    mover(20, 270);
  //    delay(300);
  //    parar();
  //    delay(100);
  //  }
  //  if (valorluz[0] >= tetocor || valorluz[1] >= tetocor1) {
  //    target = angulolido;
  //    Serial.println("sul");
  //    for (int k = 0; k < 32; k++) {
  //      led.setPixelColor(k, led.Color(0, 255, 0));
  //    }
  //    parar();
  //    delay(100);
  //    mover(20, 180);
  //    delay(300);
  //    parar();
  //    delay(100);
  //  } else {
  //Serial.println("seguir");
  if (Serial2.available()) {
    int msg = Serial2.parseInt();
    if (msg != 0) {
      if (msg >= 1023 && msg <= 1025) {
        bola = msg;
      } else if (msg >= 2640 && msg <= 3360) {
        angulolido = (msg - 3000);
      } else if (msg >= 1001 && msg <= 1017) {
        ir = (msg - 1000);
        Serial.println(ir);
      } else if (msg == 333 || msg == 444 || msg == 666) {
        gol = msg;
      }
      if (bola == 1024) {
        for (int k = 0; k < 24; k++) {
          led.setPixelColor(k, led.Color(0, 0, 255));
        }
        led.show();
        parar();
        if (gol == 333) {
          Serial.println("esquerda");
          girareixoA();
        } else if (gol == 666) {
          Serial.println("direita");
          //            parar();
          //            delay(50);
          //            girar_bola();
        } else if (gol == 444)
        {
          Serial.println("frente");
          parar();
          for (int k = 0; k < 32; k++) {
            led.setPixelColor(k, led.Color(0, 0, 255));
          }
          led.show();
        }
      } else if (bola == 1023) {
        if (ir >= 2 && ir <= 8) {
          girareixoA();
          target = angulolido;
        } else if (ir >= 9 && ir <= 15) {
          girareixoH();
          target = angulolido;
        } else if (ir == 1 || ir == 16) {


          for (int k = 0; k < 32; k++) {
            led.setPixelColor(k, led.Color(0, 0, 255));
          }
          led.show();
          target = angulolido;
          mover(150, 0);
          delay(3);
        }
      }
    }
  }
}
