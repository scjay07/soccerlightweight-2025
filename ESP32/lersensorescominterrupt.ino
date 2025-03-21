
uint16_t ir_values[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t ir_addr[16] = {4, 34, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


void read_sensor_0();
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 0; i < 16; ++i) pinMode(ir_addr[i], INPUT);
  attachInterrupt(digitalPinToInterrupt(ir_addr[0]), read_sensor_0, FALLING);
  attachInterrupt(digitalPinToInterrupt(ir_addr[1]), read_sensor_1, FALLING);
  /*attachInterrupt(digitalPinToInterrupt(ir_addr[2]), read_sensor_2, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[3]), read_sensor_3, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[4]), read_sensor_4, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[5]), read_sensor_5, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[6]), read_sensor_6, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[7]), read_sensor_7, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[8]), read_sensor_8, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[9]), read_sensor_9, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[10]), read_sensor_10, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[11]), read_sensor_11, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[12]), read_sensor_12, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[13]), read_sensor_13, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[14]), read_sensor_14, FALLING);
    attachInterrupt(digitalPinToInterrupt(ir_addr[15]), read_sensor_15, FALLING);*/


}

void loop() {
  // put your main code here, to run repeatedly:
  int pin = -1;
  int valueB = 0;
  char strBuff[150];
  for (int i = 0; i < 16; ++i) {
    sprintf(strBuff, "%d: %d ", i, ir_values[i]);
    if (ir_values[i] > valueB) {
      pin = i;
      valueB = ir_values[i];
    }
  }
  if (pin != -1) {
    Serial.print("valores: ");
    //Serial.println(strBuff);
    Serial.println(pin);
  }
  for (int i = 0; i < 16; ++i) ir_values[i] = 0;
  //delay(100);
}

void read_sensor_0() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) {
    if (digitalRead(ir_addr[0])) break;
  }
  ir_values[0] = micros() - start_time;
}

void read_sensor_1() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[1])) break;
  ir_values[1] = micros() - start_time;
}

void read_sensor_2() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[2])) break;
  ir_values[2] = micros() - start_time;
}

void read_sensor_3() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[3])) break;
  ir_values[3] = micros() - start_time;
}

void read_sensor_4() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[4])) break;
  ir_values[4] = micros() - start_time;
}

void read_sensor_5() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[5])) break;
  ir_values[5] = micros() - start_time;
}

void read_sensor_6() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[6])) break;
  ir_values[6] = micros() - start_time;
}

void read_sensor_7() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[7])) break;
  ir_values[7] = micros() - start_time;
}

void read_sensor_8() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[8])) break;
  ir_values[8] = micros() - start_time;
}

void read_sensor_9() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[9])) break;
  ir_values[9] = micros() - start_time;
}

void read_sensor_10() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[10])) break;
  ir_values[10] = micros() - start_time;
}

void read_sensor_11() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[11])) break;
  ir_values[11] = micros() - start_time;
}

void read_sensor_12() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[12])) break;
  ir_values[12] = micros() - start_time;
}

void read_sensor_13() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[13])) break;
  ir_values[13] = micros() - start_time;
}

void read_sensor_14() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[14])) break;
  ir_values[14] = micros() - start_time;
}


void read_sensor_15() {
  unsigned long start_time = micros();
  for (int i = 0; i < 2000; ++i) if (digitalRead(ir_addr[15])) break;
  ir_values[15] = micros() - start_time;
}
