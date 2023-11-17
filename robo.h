#pragma once

// Коэффициенты ПИДа
#define Kp 8
#define Kd 0
#define Ki 0

//7.5 0 0.01

// Пины драйвера моторов
#define RD 4
#define RS 5
#define LS 6
#define LD 7

int dat1 = 0;   // Правый датчик
int dat2 = 0;   // Левый датчик
long int I;     // Для ПИДа
int E_old = 0;  // Для ПИДа
int KpE = Kp;   // Для отработки танковых поворотов
int bs = 150;   // Базовая скорость
int clck = -3;

int d1_mi = 30;
int d2_mi = 30;
int d1_ma = 480;
int d2_ma = 290;

void initialize() {
  pinMode(RD, OUTPUT);
  pinMode(RS, OUTPUT);
  pinMode(LD, OUTPUT);
  pinMode(LS, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(2, INPUT);
}  // Инициализация библиотеки

void update() {
  dat1 = map(analogRead(A0), d1_mi, d1_ma, 0, 100);
  dat2 = map(analogRead(A1), d2_mi, d2_ma, 0, 100);
}  // Обновление датчиков

void go(int left, int right) {
  if (left < 0) {
    digitalWrite(LD, 0);
    analogWrite(LS, abs(left));
  } else {
    digitalWrite(LD, 1);
    analogWrite(LS, left);
  }
  if (right < 0) {
    digitalWrite(RD, 0);
    analogWrite(RS, abs(right));
  } else {
    digitalWrite(RD, 1);
    analogWrite(RS, right);
  }
}  // Крутит моторы
void pid(int par1, int par2) {
  /* ПИД */
  int E = par1 - par2;
  I = E + I * 0.95;
  int PID = E * KpE + (E - E_old) * Kd + I * Ki;
  PID = constrain(PID, -bs, bs);
  E_old = E;
  int M1 = bs + PID;
  int M2 = bs - PID;
  M1 = constrain(M1, -255, 255);
  M2 = constrain(M2, -255, 255);
  go(M1, M2);
}  // Сам ПИД
void left() {
  unsigned long long int timer = millis();
  while (dat2 > 20) {
    if (millis() - timer < 100) {
      go(-200, 200);
    } else {
      go(-120, 100);
    }
    update();
  }
  delay(300);
  while (dat2 < 20) {
    if (millis() - timer < 100) {
      go(-200, 200);
    } else {
      go(-120, 100);
    }
    update();
  }
  delay(300);
  while (dat2 > 20) {
    if (millis() - timer < 100) {
      go(-200, 200);
    } else {
      go(-120, 100);
    }
    update();
  }
  int time = millis();
  bs = 50;
  while (millis() - time < 200) {
    update();
    pid(dat1, dat2);
  }
  bs = 200;
  go(0, 0);
}  // Танковый поворот налево
void right() {
  while (dat1 > 20) {
    go(120, -100);
    update();
  }
  delay(300);
  while (dat1 < 20) {
    go(120, -100);
    update();
  }
  delay(300);
  while (dat1 > 20) {
    go(120, -100);
    update();
  }
  int time = millis();
  bs = 50;
  while (millis() - time < 200) {
    update();
    pid(dat1, dat2);
  }
  bs = 200;
  go(0, 0);
}  // Танковый поворот направо
void rleft() {
  go(0, 100);
  delay(2150);
  go(0, 0);
}  // Поворот налево, левое колесо заблокировано
void rright() {
  go(100, 0);
  delay(3330);
  go(0, 0);
}  // Поворот направо, правое колесо заблокировано
void move() {
  bool flag = 1;
  while (flag == 1) {
    update();
    if (analogRead(A0) > 100 && analogRead(A1) > 100) {
      go(100, 100);
      delay(200);
      flag = 0;
      go(-50, -50);
      delay(100);
      go(0, 0);
      clck = 1;
    } else pid(dat1, dat2);
  }
}
