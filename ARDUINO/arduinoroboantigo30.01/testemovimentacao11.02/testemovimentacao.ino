
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led(32, A11, NEO_GBR + NEO_KHZ800);


//const int minluz[] = {16,8,19,14,7,15,11,3,16,8,16};
//const int maxluz[] = {74,44,108,98,49,101,87,7,120,87,118};

const int minluz[] = {11, 5, 15, 11, 4, 13, 7, 0, 16, 7, 13};
const int maxluz[] = {35, 39, 26, 20, 13, 27, 15, 1, 35, 13, 39};


int x;
int IR[10];
const int tetocor = 49;
const int tetocor2 = 46;
const int tetocor3 = 50;
const int portasluz[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10 };
float valorluz[11];
int angulolido;
int angulocorrecao;
int erromover;
int target;
float kp = 0.27;
float kpm = 1.3;
int erro = 0;
int ir;
int bola;
int gol;
unsigned long tempoinicial = 0;
bool alinhando = false;


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

void mudar_cor(int r, int g, int b)
{
  for (int k = 0; k < 32; k++) {
    led.setPixelColor(k, led.Color(r, g, b));
  }
  led.show();
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
  //mover(100, 45);
  delay(600);
}


void loop() {
  erro = target - angulolido;
  angulocorrecao = kp * erro;
  erromover = kpm * erro;
  for (int i = 0; i < 11; i++) {
    valorluz[i] = analogRead(portasluz[i]);
    valorluz[i] = (int)map(valorluz[i], minluz[i], maxluz[i], 0, 50);
    valorluz[i] = constrain(valorluz[i], 0, 50);
  }

  //Serial.println("seguir");
  if (Serial2.available()) {
    int msg = Serial2.parseInt();
    if (msg != 0) {
      if (msg >= 1023 && msg <= 1025) {
        bola = msg;
      }
      else if (msg >= 2640 && msg <= 3360) {
        angulolido = (msg - 3000);
      }
      else if (msg >= 1001 && msg <= 1017) {
        ir = (msg - 1000);
        Serial.println(ir);
      }
      else if (msg == 333 || msg == 444 || msg == 666) {
        gol = msg;
      }
      if (abs(angulolido) > 20)
      { //se o robô estiver desalinhado em até 50 graus, então alinha);
        tempoinicial = millis();
        while ((abs(angulolido) > 20 && abs(angulolido) < 340) && (millis() - tempoinicial) <= 75)
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
        parar();
        if (bola == 1024) {
          for (int k = 0; k < 24; k++) {
            led.setPixelColor(k, led.Color(0, 0, 255));
          }
          led.show();
          parar();
        }
        else if (bola == 1023)
        {
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
      }
    }
  }
}
