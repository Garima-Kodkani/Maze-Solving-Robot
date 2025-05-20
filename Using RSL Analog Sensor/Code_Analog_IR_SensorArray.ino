// 1. Auto calibrate
// 2. Button 11 for restart
// 3. End box placement


//WORKING DRY RUN AT 12V
// Dry RUN 11.8 - 35 secs
// CHECK wheel

#include "ArrayList.h"
ArrayList<char> myList;
ArrayList<char> pathList;
const int hardCSize=30;
char arr[hardCSize] =
{'R','R','L','R','L','L','R','L','R','R','R','R','L','S','L','R','L','L','S','L','R','S','S','E'};

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 10
#define PWMB 9


bool isBlackLine = 0;  //keep 1 in case of black line. In case of white line change this to 0
unsigned int numSensors = 8;
int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 150;
int currentSpeed = 30;
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };
int activeSensors;
int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];
int flag = 0;
int k = 0;
int allwhite = 0;
float Kp = 0.03;  //0.08
float Kd = 0.3;   //0.15
float Ki = 0;
bool jpulse = false;
int del = 0;

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);  //Pushbutton
  pinMode(12, INPUT_PULLUP);  //Pushbutton
  pinMode(13, OUTPUT);        //LED
  pinMode(5, OUTPUT);         //standby for older carrier boards
  digitalWrite(5, HIGH);
 
//   for(int i=0;i<hardCSize;i++){
//     pathList.add(arr[i]);
// }
}

void linefollow() {
  //Serial.println("PID");
  error = 0;
  activeSensors = 0;
  for (int i = 0; i < 8; i++) {
    error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    activeSensors += sensorArray[i];
  }
  error = error / activeSensors;
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed + PIDvalue;
  rsp = currentSpeed - PIDvalue;
  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < -70) {
    rsp = -70;
  }
  //Serial.print(rsp);
  //Serial.print("   ");
  //Serial.println(lsp);
  motor1run(-rsp);
  motor2run(-lsp);
}

void calibrate() {
  for (int i = 0; i < 8; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  for (int i = 0; i < 7000; i++) {
    motor1run(70);
    motor2run(-70);
    for (int i = 0; i < 8; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }
  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    // Serial.print(threshold[i]);
    // Serial.print(" ");
  }
  //Serial.println();
  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  int count = 0;
  allwhite = 0;
  for (int i = 0; i < 8; i++) {
    if (isBlackLine) {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    // Serial.print(sensorArray[i]);
    // Serial.print(" ");
    if (sensorArray[i]) {
      onLine = 1;
      count++;
    }
    if (count >= 7) allwhite = 1;
    jpulse = (count >= 5);
  }
  // Serial.print("count: ");
  // Serial.println();
}

void readSensor(int i) {
  if (isBlackLine) {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
  } else {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
  }
  sensorValue[i] = constrain(sensorValue[i], 0, 1000);
  sensorArray[i] = sensorValue[i] > 500;
}

void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

void right(int l) {
  motor1run(-l);
  motor2run(l);
}

void left(int r) {
  motor1run(r);
  motor2run(-r);
}

void backward(int f) {
  motor1run(f);
  motor2run(f);
}

void forward(int b) {
  motor1run(-b);
  motor2run(-b);
}

void rightFollow() {
  forward(150);
  delay(100);
  readSensor(0);
  while (sensorArray[0] == 0) {
    right(130);
    readSensor(0);
  }
  readSensor(3);
  while (sensorArray[3] == 0) {
    right(80);
    // readSensor(4);
    readSensor(3);
    // Serial.println(sensorArray[3]);
  }
}

void leftFollow() {
  forward(150);
  delay(100);
  readSensor(7);
  while (sensorArray[7] == 0) {
    //Serial.println(sensorArray[7]);
    left(130);
    readSensor(7);
  }
  readSensor(4);
  while (sensorArray[4] == 0) {
    left(100);
    // readSensor(4);
    readSensor(4);
    // Serial.println(sensorArray[3]);
  }
}

void forwardFollow() {
  forward(130);
  delay(100);
}

void basecode() {
  forward(100);
  delay(5000);
  wait();
  delay(1000);




  backward(100);
  delay(5000);
  wait();
  delay(1000);




  left(50);
  delay(5000);
  wait();
  delay(1000);




  right(100);
  delay(5000);
  wait();
  delay(1000);
}

void uturn(int u) {
  // readLine();
  readSensor(4);
  while (sensorArray[4] == 0) {
    left(u);
    readSensor(4);
  }
  //linefollow();
}

void wait() {
  motor1run(0);
  motor2run(0);
}

void solvemaze() {
   Serial.println("Solve Maze function");
  for (int i = 0; i < pathList.size(); i++) { Serial.print(pathList.get(i)); }
  Serial.println();

  while (pathList.contains('B')) {
    int i = pathList.indexOf('B');
    if (pathList.get(i - 1) == 'R' && pathList.get(i + 1) == 'L') {  //RBL = B
      pathList.set(i - 1, 'B');
    } else if (pathList.get(i - 1) == 'R' && pathList.get(i + 1) == 'S') {  //RBS=L
      pathList.set(i - 1, 'L');
    } else if (pathList.get(i - 1) == 'L' && pathList.get(i + 1) == 'R') {  //LBR=B
      pathList.set(i - 1, 'B');
    } else if (pathList.get(i - 1) == 'S' && pathList.get(i + 1) == 'R') {  //SBL=R
      pathList.set(i - 1, 'L');
    } else if (pathList.get(i - 1) == 'S' && pathList.get(i + 1) == 'S') {  //SBS=B
      pathList.set(i - 1, 'B');
    } else if (pathList.get(i - 1) == 'R' && pathList.get(i + 1) == 'R') {  //LBL=S
      pathList.set(i - 1, 'S');
    }
    pathList.remove(i);
    pathList.remove(i);
    for (int i = 0; i < pathList.size(); i++) { Serial.print(pathList.get(i)); }
    Serial.println();
  }
  Serial.println("Final Array  \n");
  for (int i = 0; i < pathList.size(); i++) { Serial.print(pathList.get(i)); }
  Serial.println();
}

bool jpulseF() {
  int jwidth = 5;
  int onLine = 0;
  for (int i = 0; i < 8; i++) {
    if (isBlackLine) {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;
    // Serial.print(sensorValue[i]);
    // Serial.print("  ");
    if (sensorArray[i]) onLine++;
  }
  // Serial.println();
  if (onLine >= jwidth) return true;
  return false;
}

void loop() {

  while (flag == 0) { //calibrate
   delay(1000);
   calibrate();
   delay(1000);
   flag=1;
  }
  while (flag == 1) {  //DRY RUN
    digitalWrite(13, LOW);
    //while(digitalRead(11)){ del=0; }
    // if(del==0){
    //   Serial.println("Before: ");
    //   for (int i = 0; i < pathList.size(); i++) { Serial.print(pathList.get(i)); }
    //   Serial.println();
    //   pathList.remove((int)pathList.size()-1);
    //   del=1;
    //   Serial.println("After: ");
    //   for (int i = 0; i < pathList.size(); i++) { Serial.print(pathList.get(i)); }
    //   Serial.println();
    // }
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  //PID LINE FOLLOW
      linefollow();

      //RIGHT
      if (sensorArray[0] || (allwhite == 1)) {
        forward(150);
        delay(100);
        // Serial.print(sensorArray[0]);
        // Serial.print(" ");
        // Serial.println(sensorArray[4]);
        readSensor(0);
        // readSensor(4);
        if (sensorArray[0] == 1 && sensorArray[3] == 1) {
          // right(150);
          // delay(100);
          forward(150);
          delay(200);
          right(150);
          delay(300);
          forward(150);
          delay(200);
          Serial.println("End");
          pathList.add('E');
          digitalWrite(13, HIGH);
          wait();
          Serial.println("End-2");
          delay(7000);
          solvemaze();
          Serial.println("SOLVED");
          digitalWrite(13, LOW);
          flag = 2;
        }

        while (sensorArray[0] == 0) {
          right(130);
          // readSensor(4);
          readSensor(0);
          // Serial.println(sensorArray[0]);
          // Serial.print(" ");
          // Serial.println(sensorArray[4]);
        }
        readSensor(3);
        while (sensorArray[3] == 0) {
          right(80);
          // readSensor(4);
          readSensor(3);
          // Serial.println(sensorArray[3]);
        }
        Serial.println("only right");
        pathList.add('R');
        digitalWrite(13, HIGH);
      }
      //LEFT
      else if (!sensorArray[1] && sensorArray[7] && sensorArray[6] && sensorArray[4]) {
        digitalWrite(13, HIGH);
        //Serial.println("See");
        forward(130);
        delay(100);
        digitalWrite(13, LOW);
        readLine();
        if (!sensorArray[7] && !sensorArray[3] && !sensorArray[4]) {
          while (sensorArray[4] == 0) {
            left(100);
            readSensor(4);
          }
          Serial.println("only left");
          pathList.add('L');
          digitalWrite(13, HIGH);
        }
        else if (sensorArray[5] && sensorArray[6] && sensorArray[3] && sensorArray[4]) {
          forward(150);
          delay(200);
          left(150);
          delay(300);
          forward(150);
          delay(200);
          digitalWrite(13, HIGH);
          Serial.println("End");
          pathList.add('E');
          wait();
          Serial.println("End-2");
          delay(7000);
          solvemaze();
          Serial.println("SOLVED");
          digitalWrite(13, LOW);
          flag = 2;
        } else {
          Serial.println("Straight2");
          pathList.add('S');
          digitalWrite(13, HIGH);
        }
      } else {
        linefollow();
      }
    }
    // UTURN
    else {
      // digitalWrite(13, HIGH);
      forward(150);
      delay(100);
      Serial.println("Uturn");
      uturn(90);
      pathList.add('B');
      digitalWrite(13, HIGH);
      Serial.println("All black");
      // digitalWrite(13, LOW);
    }
    // if(digitalRead(11)==0){ Serial.println("Before: ");
    //   for (int i = 0; i < pathList.size(); i++) { Serial.print(pathList.get(i)); }
    //   Serial.println();
    //   pathList.remove((int)pathList.size()-1);
    //   del=1;
    //   Serial.println("After: ");
    //   for (int i = 0; i < pathList.size(); i++) { Serial.print(pathList.get(i)); }
    //   Serial.println(); }
  }
  while (flag == 2) { //Actual Run
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    // Serial.println(jpulse);
    if (jpulse==1) {
      digitalWrite(13, HIGH);
      if (pathList.get(k) == 'L') {
        Serial.println("L1");
        leftFollow();
        Serial.println("L2");
      } else if (pathList.get(k) == 'S') {
        Serial.println("S1");
        forwardFollow();
        Serial.println("S2");
      } else if (pathList.get(k) == 'R') {
        Serial.println("R1");
        rightFollow();
        Serial.println("R2");
      } else if (pathList.get(k) == 'E') {
        Serial.println("end");
        forward(130);
        delay(300);
        digitalWrite(13, HIGH);
        wait();
        delay(5000);
        flag = 2;
        exit(0);
      }
      else{
        Serial.println("none");
      }
      k++;
    } else {
      linefollow();
      digitalWrite(13, LOW);
    }
  }
}
