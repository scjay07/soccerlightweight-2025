//M4
#include "Arduino.h"
#include "RPC.h"

using namespace rtos;

Thread ir_threads[9];
int ir_ports[] = { 10, 11, 12, 13, 44, 50, 52, 53, 51, 49, 43, 41, 31, 29, 23, 2, 3,9};
//int ir_ports[] = { 10, 11, 12, 13, 44, 50, 52, 53, 51, 49, 43, 41, 31, 29, 2, 3 };
int ir_values[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// functions
void read_ir_sensor_0();
void read_ir_sensor_1();
void read_ir_sensor_2();
void read_ir_sensor_3();
void read_ir_sensor_4();
void read_ir_sensor_5();
void read_ir_sensor_6();
void read_ir_sensor_7();
void read_ir_sensor_8();
void read_ir_sensor_9();
void read_ir_sensor_10();
void read_ir_sensor_11();
void read_ir_sensor_12();
void read_ir_sensor_13();
void read_ir_sensor_14();
void read_ir_sensor_15();
void read_ir_sensor_16();
void read_ir_sensor_17();


void setup() {
  RPC.begin();

  for (int i = 0; i < 18; i++) pinMode(ir_ports[i], INPUT);

  ir_threads[0].start(read_ir_sensor_0);
  ir_threads[1].start(read_ir_sensor_1);
  ir_threads[2].start(read_ir_sensor_2);
  ir_threads[3].start(read_ir_sensor_3);
  ir_threads[4].start(read_ir_sensor_4);
  ir_threads[5].start(read_ir_sensor_5);
  ir_threads[6].start(read_ir_sensor_6);
  ir_threads[7].start(read_ir_sensor_7);
  ir_threads[8].start(read_ir_sensor_8);

  RPC.bind("send_ir", send_ir);
}

void loop() {
}

int send_ir() {
  int pin = -1;
  int max = 0;
  //for (int i = 0; i < 18; i++) if (ir_values[i] > 400) ir_values[i] = 400;
  for (int i = 0; i < 18; i++) {
    if (ir_values[i] > +max) {
      max = ir_values[i];
      pin = i;
    }
  }

  for (int i = 0; i < 18; i++) ir_values[i] = 0;
  return pin;
}

void read_ir_sensor_0() {
  while (true) {
    ir_values[0] = pulseIn(ir_ports[0], LOW);
    ir_values[1] = pulseIn(ir_ports[1], LOW);
  }
}


void read_ir_sensor_1() {
  while (true) {
    ir_values[2] = pulseIn(ir_ports[2], LOW);
    ir_values[3] = pulseIn(ir_ports[3], LOW);
  }
}


void read_ir_sensor_2() {
  while (true) {
    ir_values[4] = pulseIn(ir_ports[4], LOW);
    ir_values[5] = pulseIn(ir_ports[5], LOW);
  }
}


void read_ir_sensor_3() {
  while (true) {
    ir_values[6] = pulseIn(ir_ports[6], LOW);
    ir_values[7] = pulseIn(ir_ports[7], LOW);
  }
}


void read_ir_sensor_4() {
  while (true) {
    ir_values[8] = pulseIn(ir_ports[8], LOW);
    ir_values[9] = pulseIn(ir_ports[9], LOW);
  }
}


void read_ir_sensor_5() {
  while (true) {
    ir_values[10] = pulseIn(ir_ports[10], LOW);
    ir_values[11] = pulseIn(ir_ports[11], LOW);
  }
}

void read_ir_sensor_6() {
  while (true) {
    ir_values[12] = pulseIn(ir_ports[12], LOW);
    ir_values[13] = pulseIn(ir_ports[13], LOW);
  }
}

void read_ir_sensor_7() {
  while (true) {
    ir_values[14] = pulseIn(ir_ports[14], LOW);
    ir_values[15] = pulseIn(ir_ports[15], LOW);
  }
}

void read_ir_sensor_8() {
  while (true) {
    ir_values[16] = pulseIn(ir_ports[16], LOW);
    ir_values[17] = pulseIn(ir_ports[17], LOW);
  }
}
