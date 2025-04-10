//GK
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led(32, A11, NEO_GBR + NEO_KHZ800);
#include <MPU6050_light.h>  //Include library for MPU communication

/*TODOS

   ADICIONAR LINHAS BRANCAS NO CODIGO
   ARRUMAR PEGAR BOLA POR TRAS, VERIFICAR QUE O SENSOR GIROSCOPIO FORCA O ROBO A FICAR ´PARA FRENTE, LOGO OS PARAMETROS QUE FAZEM O ROBO ANDAR LATERALMENTE NÃO FUNCIONAM CORRETAMENTE
   DIGA

*/
const int minluz[] = {8, 11, 9, 10, 6, 6, 0, 0, 9, 5, 8};
const int maxluz[] = {40, 39, 14, 19, 10, 25, 9, 0, 33, 21, 35};


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

int margem = 140;
float kp = 0.27;  // Ajuste fino do ganho proporcional



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
  if (Serial2.available())
  {
    int msg = Serial2.parseInt();
    if (msg >= 2640 && msg <= 3360)
    {
      angulolido = (msg - 3000);
      Serial.print("angulolido:   ");
      Serial.println(angulolido);
      erro = target - angulolido;
      angulocorrecao = kp * erro;
    }
    if (msg >= 1001 && msg <= 1017)
    {
      ir = msg - 1000;
      //Serial.println(ir);
    }
    Serial.print("angulocorrecao:  ");
    Serial.print(angulocorrecao);
    Serial.print("      ");
    Serial.print("angulolido:   ");
    Serial.println(angulolido);
    if (abs(angulolido) > 10)
    { //se o robô estiver desalinhado em até 50 graus, então alinha);
      tempoinicial = millis();
      while ((abs(angulolido) > 10 && abs(angulolido) < 350) && (millis() - tempoinicial) <= 75)
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
    else parar();
  }
}
