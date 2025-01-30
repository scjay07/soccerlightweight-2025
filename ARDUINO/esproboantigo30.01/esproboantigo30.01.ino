//GK TA COM -1 NO 6 IR
//ESP GK JOGO 2

#include <HardwareSerial.h>
#include <LCD_I2C.h>
HardwareSerial mySerial(2); // define a Serial for UART2
//ESP
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // O endereço padrão é 0x28, mas pode variar

const int mySerialRX = 16; // na esp32 isso eh o rx2 e o tx2
const int mySerialTX = 17;

LCD_I2C lcd(0x27, 16, 2); // Default address of most PCF8574 modules, change according

int angulogiro;
float angleZ = 0;          // Estimativa do ângulo
float biasZ = 0;           // Desvio de bias
float rateZ = 0;           // Taxa de mudança angular
float P = 1;               // Matriz de covariância
float Q = 0.01;            // Process noise (incerteza do processo)
float R = 0.01;             // Medição de ruído (incerteza das medições)
float K = 0;               // Ganho de Kalman


float Kp = 6;
float Kd = 1.25;
float Ki = 10;

float target = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;

float angle = 0;
int x;

int IR[16];
int valor[16];
int ori[5] = {0, 0, 0, 0, 0};
const int portas[16] =  {15, 4, 5, 18, 19, 36, 39, 23, 13, 14, 26, 25, 33, 32, 35, 34};
//const int minimo[16] = {3700,4532,4260,4442,-1,4357,4249,4247,4694,4242,4527,4605,4468,4290,4605,4064};
//const int minimo[16] = {5201,4841,4587,4725,-1,4418,4511,5421,4551,4612,4621,5915,4802,5588,5934,4583};
const int minimo[16] = {4801, 4841, 4587, 4725, -1, 4418, 4511, 5421, 4551, 4612, 4621, 5915, 4802, 5588, 5934, 4583};
//5253,6766,5952,5617,-1,5968,5362,6182,6566,5545,5858,5686,5601,8001,5767,4803

//
//int valor1;
//int valor2;
//int valor3;
//int valor4;
//int valor5;
//int valor6;
//int valor7;
//int valor8;

int bola;

const int MAX = 10000;

int16_t ax, ay, az, gy, gx, gz;

int lista_min[16];
int contagem[16];
int moda;
int qtd_leituras = 3;
int leitura()
{
  int sensor_min;
  for (int i = 0, min_ = MAX; i < 16; i++)
  {
    for (int j = 0; j < MAX; j++) valor[i] += digitalRead(portas[i]);
    int aux = map(valor[i], minimo[i], 10000, 0, 100);
    if (aux < 0)
    {
      aux = 0;
    }
    if (aux < min_ && aux >= 0)
    {
      min_ = aux;
      sensor_min = i;
    }

  }
  return sensor_min;
}

void setup() {
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
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, mySerialRX, mySerialTX);
  mySerial.setTimeout(5);
  Wire.begin();
  if (!bno.begin()) {
    Serial.print("Não foi possível encontrar o sensor BNO055");
    while (1);
  }

  bno.setMode(OPERATION_MODE_COMPASS);


  Serial.println("Iniciando o BNO055...");
  

}

//bool direita = 1;
//bool esquerda = 1;
//bool frente = 1;

void loop() {
  // put your main code here, to run repeatedly:
  int index = -1;

  for (int w = 0; w < 16; w++)
  {
    lista_min[w] = -1;
  }
  for (int i = 0; i < 16; i++)
  {
    for (int j = 0; j < MAX; j++) valor[i] = 0;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  // Leitura bruta do eixo Z
  float rawZ = event.orientation.x;
  
//  Serial.print("raw: ");
//  Serial.println(rawZ);

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

  angulogiro =  angleZ;
  //
  //  if (anguloyaw >= 360)
  //  {
  //    anguloyaw = anguloyaw % 360;
  //  }
  //  if (anguloyaw < 0 && anguloyaw >= -360);
  //  {
  //    anguloyaw = anguloyaw % 360;
  //  }

  //  error = target - anguloyaw;// proportional
  //  integral = integral + error; //integral
  //  derivative = error - last_error; //derivative
  //  last_error = error;


  //  angle = (error * Kp); // + (derivative* Kd);w


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

   if (angulogiro >= 180 ) angulogiro -= 360;
   
  //    for (int i = 0; i < qtd_leituras; i++)
  //    {
  //      lista_min[i] = leitura();
  //    }
  //
  //    for (int i = 0; i < qtd_leituras; i++) {
  //      contagem[i] = 0;
  //    }
  //
  //    for (int i = 0; i < qtd_leituras; i++) {
  //      contagem[lista_min[i]]++;
  //    }
  //
  //    int maxContagem = 0;
  //    for (int i = 0; i < qtd_leituras; i++) {
  //      if (contagem[lista_min[i]] > maxContagem) {
  //        maxContagem = contagem[lista_min[i]];
  //        index = lista_min[i];
  //      }
  //    }
  //  Serial.println(index);
  //  Serial.println(bola);

  if ((IR[0] < 2 && IR[15] < 2))
  {
    bola = 1024; //true
    //while (!mySerial.available());
    if (mySerial.available())
    {
      x = mySerial.parseInt();
      //Serial.println(x);
      if (x != 0)
      {

        //Serial.println(x);
        if (x == 333)
        {
          //          if (direita == 1 || frente == 1) {
          // Serial.println("esquerda");
          mySerial.println(333); //esquerda
          //            esquerda = 1;
          //            direita = 0;
          //            frente = 0;
          // }

        }
        else if (x == 666) {
          // if (esquerda == 1 || frente == 1) {
          //  Serial.println("direita");
          mySerial.println(666); //direita
          //            direita = 1;
          //            esquerda = 0;
          //            frente = 0;
          //          }
        }
        else if (x == 444)
        {
          //   if (direita == 1 || esquerda == 1) {
          //    Serial.println("frente");
          mySerial.println(444); //frente
          //            frente = 1;
          //            direita = 0;
          //            esquerda = 0;
          //          }
        }
        mySerial.flush();
      }
    }

  }
  else
  {
    bola = 1023; //false
  }
  switch (index)
  {
    case 0: //IR1
      //Serial.println(1002);
      mySerial.println(1001);
      break;
    case 1:
      //Serial.println(1003);
      mySerial.println(1002);
      break;
    case 2:
      //Serial.println(1004);
      mySerial.println(1003);
      break;
    case 3:
      //Serial.println(1005);
      mySerial.println(1004);
      break;
    case 4:
      //Serial.println(1006);
      mySerial.println(1005);
      break;
    case 5:
      //Serial.println(1007);
      mySerial.println(1006);
      break;
    case 6:
      //Serial.println(1008);
      mySerial.println(1007);
      break;
    case 7:
      //Serial.println(1009);
      mySerial.println(1008);
      break;
    case 8:
      //Serial.println(1010);
      mySerial.println(1009);
      break;
    case 9:
      //Serial.println(1011);
      mySerial.println(1010);
      break;
    case 10:
      //Serial.println(1012);
      mySerial.println(1011);
      break;
    case 11:
      //Serial.println(1013);
      mySerial.println(1012);
      break;
    case 12:
      //Serial.println(1014);
      mySerial.println(1013);
      break;
    case 13:
      //Serial.println(1015);
      mySerial.println(1014);
      break;
    case 14:
      //Serial.println(1016);
      mySerial.println(1015);
      break;
    case 15:
      //Serial.println(1017);
      mySerial.println(1016);
      break;
    case 16:
      //Serial.println(1017);
      mySerial.println(1017);
      break;
    default:
      Serial.println("Invalido");

  }
  for (int k = 0; k < 16; k++)
  {
    IR[k] = 0;
  }

  //mySerial.println(bola);
  Serial.println (angulogiro);
  mySerial.println(angulogiro + 3000);
  //mySerial.println(angle + 6000);
  //Serial.println(angle);

}
