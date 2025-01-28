#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Defina o pino de comunicação I2C
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // O endereço padrão é 0x28, mas pode variar

// Variáveis para o Filtro de Kalman
float angleZ = 0;          // Estimativa do ângulo
float biasZ = 0;           // Desvio de bias
float rateZ = 0;           // Taxa de mudança angular
float P = 1;               // Matriz de covariância
float Q = 0.01;            // Process noise (incerteza do processo)
float R = 0.1;             // Medição de ruído (incerteza das medições)
float K = 0;               // Ganho de Kalman

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.print("Não foi possível encontrar o sensor BNO055");
    while (1);
  }

  Serial.println("Iniciando o BNO055...");
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Leitura bruta do eixo Z
  float rawZ = event.orientation.z;

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

  // Exibe o ângulo suavizado
  Serial.print("Leitura do eixo Z (suavizada com Kalman): ");
  Serial.println(angleZ);

  delay(10);  // Atraso para estabilizar as leituras
}
