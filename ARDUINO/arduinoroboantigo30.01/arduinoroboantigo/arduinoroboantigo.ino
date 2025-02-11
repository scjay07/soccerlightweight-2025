//GK
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led(32, A11, NEO_GBR + NEO_KHZ800);
#include <MPU6050_light.h>  //Include library for MPU communication

/*TODOS

   ADICIONAR LINHAS BRANCAS NO CODIGO
   ARRUMAR PEGAR BOLA POR TRAS, VERIFICAR QUE O SENSOR GIROSCOPIO FORCA O ROBO A FICAR ´PARA FRENTE, LOGO OS PARAMETROS QUE FAZEM O ROBO ANDAR LATERALMENTE NÃO FUNCIONAM CORRETAMENTE
   DIGA



*/
//const int minluz[] = { 8, 15, 10, 8, 2, 5, 1, 0, 6, 1, 12 };
//const int maxluz[] = { 38, 46, 29, 28, 11, 19, 7, 0, 23, 14, 42 };
//const int minluz[] = {11,5,15,11,4,13,7,0,16,7,13};
//const int maxluz[] = {40,39,26,20,13,27,15,1,35,13,39};
//const int minluz[] = {11,11,8,10,7,4,0,0,10,4,11};
//const int maxluz[] = {29,30,16,24,15,10,4,0,18,12,31};
//const int minluz[] = {9, 10, 15, 11, 4, 13, 7, 0, 16, 7, 13};
//const int maxluz[] = {35, 40, 26, 20, 13, 27, 15, 1, 35, 13, 39};
const int minluz[] = {8,11,9,10,6,6,0,0,9,5,8};
const int maxluz[] = {40,39,14,19,10,25,9,0,33,21,35};



int x;
int IR[10];
const int tetocor = 49;
const int tetocor2 = 46;
const int tetocor3 = 50;

const int portasluz[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10 };
float valorluz[11];
int ir, bola, gol;
int s[4] = { 1, 1, 1, 1 };


int angulolido;
int angulocorrecao;
int target = 0;
int erro = 0;
unsigned long tempoinicial = 0;
bool alinhando = false;


const int DIR_PIN_FL1 = 28, DIR_PIN_FR1 = 34, DIR_PIN_RL1 = 26, DIR_PIN_RR1 = 36;
const int DIR_PIN_FL2 = 29, DIR_PIN_FR2 = 35, DIR_PIN_RL2 = 27, DIR_PIN_RR2 = 37;
const int PWM_PIN_FL = 12, PWM_PIN_FR = 8, PWM_PIN_RL = 11, PWM_PIN_RR = 6;


const float sen45 = sqrt(2) / 2, cos45 = sqrt(2) / 2;
float wfe, wfd, wte, wtd;
byte wfe_v, wfd_v, wte_v, wtd_v;


float kp = 0.8;  // Ajuste fino do ganho proporcional


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


  sentidomotor(wfe, 0);
  sentidomotor(wfd, 1);
  sentidomotor(wte, 2);
  sentidomotor(wtd, 3);


  float maxVel = max(max(abs(wfe), abs(wfd)), max(abs(wte), abs(wtd))) + 20;


  wfe_v = (byte)map(abs(wfe), 0, maxVel, 45, 255);
  wfd_v = (byte)map(abs(wfd), 0, maxVel, 40, 255);
  wte_v = (byte)map(abs(wte), 0, maxVel, 40, 255);
  wtd_v = (byte)map(abs(wtd), 0, maxVel, 40, 255);


  setMotor(PWM_PIN_FL, DIR_PIN_FL1, DIR_PIN_FL2, wfe_v, s[0]);
  setMotor(PWM_PIN_FR, DIR_PIN_FR1, DIR_PIN_FR2, wfd_v, s[1]);
  setMotor(PWM_PIN_RL, DIR_PIN_RL1, DIR_PIN_RL2, wte_v, s[2]);
  setMotor(PWM_PIN_RR, DIR_PIN_RR1, DIR_PIN_RR2, wtd_v, s[3]);
}


int mediaSensorIR(int leituraAtual) {
  static int buffer[5] = { 0 };
  static int indice = 0;
  buffer[indice] = leituraAtual;
  indice = (indice + 1) % 5;


  int soma = 0;
  for (int i = 0; i < 5; i++) {
    soma += buffer[i];
  }
  return soma / 5;
}
void alinhar() {

  if (angulocorrecao > 0 && angulocorrecao <= 100) angulocorrecao = 300;
  if (angulocorrecao < 0 && angulocorrecao >= -100) angulocorrecao = -300;
  if (angulocorrecao >= 400 || angulocorrecao <= -400) kp *= 0.5;
  wfe = 0 - angulocorrecao;
  wte = 0 - angulocorrecao;
  wfd = 0 - angulocorrecao;
  wtd = 0 - angulocorrecao;
  wfe_v = (byte)map(abs(wfe), 0, 1500, 0, 255);
  wfd_v = (byte)map(abs(wfd), 0, 1500, 0, 255);
  wte_v = (byte)map(abs(wte), 0, 1500, 0, 255);
  wtd_v = (byte)map(abs(wtd), 0, 1500, 0, 255);
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
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.setTimeout(5);
  for (int k = 0; k < 32; k++) {
    led.setPixelColor(k, led.Color(0, 0, 255));
  }
  led.show();
  for (int i = 0; i < 11; i++) {
    pinMode(portasluz[i], INPUT);
  }
  Serial.println("INICIADO");
//  mover(70, 0);
//  delay (600);

}


void mudar_cor(int r, int g, int b)
{
  for (int k = 0; k < 32; k++) {
    led.setPixelColor(k, led.Color(r, g, b));
  }
  led.show();

}

void loop() 
{
//  for (int i = 0; i < 11; i++) {
//    valorluz[i] = analogRead(portasluz[i]);
//    valorluz[i] = (int)map(valorluz[i], minluz[i], maxluz[i], 0, 50);
//    valorluz[i] = constrain(valorluz[i], 0, 50);
//  }
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
  if (valorluz[0] >= tetocor //|| valorluz[1] >= tetocor
  ) {
    target = angulolido;
    Serial.println("sul");
    for (int k = 0; k < 32; k++) {
      led.setPixelColor(k, led.Color(0, 255, 0));
    }
    parar();
    delay(100);
    mover(20, 180);
    delay(300);
    parar();
    delay(100);
  }
//  else if (valorluz[5] >= tetocor //|| valorluz[6] >= tetocor || valorluz[7] >= tetocor
//          )
//  {
//    target = angulolido;
//    Serial.println("norte");
//    for (int k = 0; k < 32; k++) {
//      led.setPixelColor(k, led.Color(0, 255, 0));
//    }
//    parar();
//    delay(100);
//    mover(20, 0);
//    delay(300);
//    parar();
//    delay(100);
//  }
  else {

    if (Serial2.available()) {
      int msg = Serial2.parseInt();
      if (msg >= 2640 && msg <= 3360) {
        angulolido = (msg - 3000);
        //Serial.println(angulolido);
      }

      if (msg >= 1001 && msg <= 1017) {
        ir = msg - 1000;  
        Serial.println(ir);
      }
      if (abs(angulolido) > 30) {//se o rob|ô estiver desalinhado em até 50 graus, então alinha

        tempoinicial = millis();

        while (abs(angulolido) > 30 && (millis() - tempoinicial) <= 75)
        {
          mudar_cor(0, 0, 255);
          msg = Serial2.parseInt();
          if (msg >= 2640 && msg <= 3360) {
            angulolido = (msg - 3000);
            erro = target - angulolido;
            angulocorrecao = kp * erro;
            //Serial.println(angulolido);
            alinhar();
            //parar();
          }
        }
      }
      else {//se o robÔ estiver alinhado para frente, então vai ler os sensores IR

        int velocidadeBase = 40;  // Velocidade base ajustada para estabilidade
        if (msg >= 1001 && msg <= 1017) {
          ir = msg - 1000;  // Suavização da leitura do sensor IR
        }
        //      if (ir == 1 || ir == 16) {
        //        mudar_cor(255, 255, 255);
        //        mover(velocidadeBase, 0);
        //        delay(200);
        //      }
        //      if (ir >= 2 && ir <= 5)
        //      {
        //        mudar_cor(255, 0, 0);
        //        mover(velocidadeBase, 280);
        //      }
        //      else if (ir >= 5 && ir <= 7)
        //      {
        //        mudar_cor(255, 255, 0);
        //        mover(velocidadeBase, 140);
        //        delay(500);
        //        parar();
        //        delay(5000);
        //        mover(velocidadeBase, 220);
        //        delay(500);
        //        parar();
        //        delay(20000);
        //        mudar_cor(255, 255, 255);
        //      }
        //      else if (ir >= 7 && ir <= 9)
        //      {
        //        mudar_cor(255, 0, 255);
        //        mover(velocidadeBase, 250);
        //        delay(300);
        //        parar();
        //        delay(2000);
        //        mover(velocidadeBase, 115);
        //        delay(300);
        //        parar();
        //        delay(2000);
        //        mudar_cor(255, 255, 255);
        //      }
        if (ir >= 2 && ir <= 4) {
          mover(velocidadeBase, 240);  // Move para a esquerda (considerando sensores traseiros)
        }
        else if (ir >= 5 && ir <= 8)
        {
          mover(velocidadeBase, 190);
        }
        else if (ir == 9 || ir ==  10)
        {
          mover(velocidadeBase, 255);
        }
        else if (ir >= 11 && ir <= 12) {
          mover(velocidadeBase, 170);  // Move para a direita
        }
        else if (ir >= 13 && ir <= 15)
        {
          mover(velocidadeBase, 140);
        }
        else if (ir == 1 || ir == 16) {
          mover(velocidadeBase, 0);  // Move para frente//
        }
      }
    }
  }
}



