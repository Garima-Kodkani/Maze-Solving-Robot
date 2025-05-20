#include <PID_v1_bc.h>

int DIR1 = 8;
int DIR2 = 7;
int PWM1 = 5;
int PWM2 = 6;

//LSA08 pins
int ir0 = 2;
int ir1 = 3;//Leftmost
int ir2 = 4;
int ir3 = 9;
int ir4 = 10;
int ir5 = 11;
int ir6 = 12;//Rightmost
int  LED=13;
int flag=0;
int i=0;

//Define Variables we'll be connecting to
double Setpoint= 40;
double  motorSpeed=60; 
double InputL, OutputL;
double InputR, OutputR;
char full_array[50];
char opti_array[50];

int dry_run=0;
int len=50;

//Specify the links and initial tuning parameters
double Kp=1.1, Ki=0, Kd=0.0343;
PID myPIDL(&InputL, &OutputL, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPIDR(&InputR, &OutputR, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{ 
  Serial.begin(9600);
  //motor pins setup
  for(int i = 5; i<=8; i++)
  {
    pinMode(i, OUTPUT);
  }
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);

  //LSA08 Pins Setup
  for(int j = 2; j<=4; j++)
  {
    pinMode(j, INPUT);
  }
  pinMode(13, OUTPUT);
  for(int j = 9; j<13; j++)
  {
      pinMode(j, INPUT);
  }
  
  // Turn the PID on 
  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetOutputLimits(-150, 150);
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetOutputLimits(-150, 150);
}

void loop() 
{
   // put your main code here, to run repeatedly:
   int i = 0;
   if(dry_run == 0)
   {
      //Only Straight
      if((digitalRead(ir3)== 1 || digitalRead(ir4)== 1)  && digitalRead(ir1) == 0 && digitalRead(ir2) == 0 && digitalRead(ir5) == 0 && digitalRead(ir6) == 0 )
      {  
        full_array[i] = "S";
        computePID();
      }
      //Only Left Path
      if( digitalRead(ir1) == 1 || digitalRead(ir2) == 1)
      {  
        full_array[i] = "L";
        left(70);
        computePID();
      }
      //Only Right Path
      if(digitalRead(ir5) == 1 || digitalRead(ir6)== 1)
      {
        full_array[i] = "R";
        right(70);
        computePID();
      }
      //T with straight and right
      if(digitalRead(ir3)== 1 && digitalRead(ir4)== 1 && digitalRead(ir5) == 1 && digitalRead(ir6)== 1)
      {
        full_array[i] = "S";
        straight(70);
        computePID(); 
      }
      //T with straight and left
      if(digitalRead(ir3)== 1 && digitalRead(ir4)== 1 && digitalRead(ir2) == 1 && digitalRead(ir1)== 1)
      {
        full_array[i] = "L";
        left(70);
        delay(50);
        computePID(); 
      }
      //All black
      if(digitalRead(ir1) == 0 && digitalRead(ir2) == 0 && digitalRead(ir3) == 0 && digitalRead(ir4) == 0 && digitalRead(ir5) == 0 && digitalRead(ir6) == 0 )
      {
        full_array[i] = "B";
        uturn(70);
      }
      //If T Junction or White box at end
      if(allwhite())
      {    
        full_array[i] = "L";
        left(70);
        delay(200);
        computePID(); 
        delay(500);
    
        if(digitalRead(ir5) == 1 && digitalRead(ir6)== 1)
        {   
          //white box
          full_array[i] = "E";
          right(70);
          delay(500);
          straight(70);
          delay(500);
          wait();
          digitalWrite(LED, HIGH);
          delay(5000);
          dry_run = 1;
        }
     }
     i++;
  }
  else
  {
    for(i=0;i<len;i++)
    {
     opti_array[i]=full_array[i];
    }
    CALCULATE_SHORTEST_PATH();
    followShortestPath();
  }
}

void CALCULATE_SHORTEST_PATH()
{
  /*
  LBL = S
  LBR = B
  LBS = R
  RBL = B
  SBL = R
  SBS = B
  LBL = S */
  int j;
  char action;
  for(j = 0; j < len; j++)
  {
    action = opti_array[j];
    Serial.print(opti_array[j]);
    if(action == 'B')
    {
      Serial.print("\n B ");
      if(opti_array[j-1]== 'L' && opti_array[j+1] == 'R')
      {
        opti_array[j-1] = 'B';
        opti_array[j] = '0';
        opti_array[j+1] = '0';
        REARRANGE(j-1,2); 
      }
      
      if(opti_array[j-1]== 'L' && opti_array[j+1] == 'S')
      {
        opti_array[j-1] = 'R';
        opti_array[j] = '0';
        opti_array[j+1] = '0';
        REARRANGE(j-1,2);
      }
      
      if(opti_array[j-1]== 'R' && opti_array[j+1] == 'L')
      {
        opti_array[j-1] = 'B';
        opti_array[j] = '0';
        opti_array[j+1] = '0';
        REARRANGE(j-1,2);         
      }

      if(opti_array[j-1]== 'S' && opti_array[j+1] == 'L')
      {
        opti_array[j-1] = 'R';
        opti_array[j] = '0';
        opti_array[j+1] = '0';
        REARRANGE(j-1,2);             
      }

      if(opti_array[j-1]== 'S' && opti_array[j+1] == 'S')
      {
        opti_array[j-1] = 'B';
        opti_array[j] = '0';
        opti_array[j+1] = '0';
        REARRANGE(j-1,2);             
      }

      if(opti_array[j-1]== 'L' && opti_array[j+1] == 'L')
      {
        opti_array[j-1] = 'S';
        opti_array[j] = '0';
        opti_array[j+1] = '0';
        REARRANGE(j,2);
      }

      if(opti_array[i-2]=='L' && opti_array[i-1]=='S' && opti_array[i+1]=='R')
      {
        opti_array[j-2] = 'B';
        opti_array[j-1] = '0';
        opti_array[j] = '0';
        opti_array[j+1] = '0';
        REARRANGE(j-1,3);
      }
      j = -1;
    }
  }
}

void followShortestPath()
{
  for(i=0;i<len;i++)
   {
    switch(opti_array[i])
    {
      case 'L':
      left(60);
      delay(10);
      forward();
      computePID();
      break;

      case 'S':
      forward();
      computePID();
      break;

      case 'R':
      right(60);
      delay(10);
      forward();
      computePID();
      break;

      case 'B':
      uturn(60);
      forward();
      computePID();
      break;

      case 'E':
      forward();
      computePID();
      delay(500);
      wait();
      digitalWrite(LED, HIGH);
      delay(5000);
      break;

      case '0':
      forward();
      computePID();
      break;

      default:
      wait();
      break;
    }
  }
  wait();
  digitalWrite(LED, HIGH);
}

void REARRANGE(int s, int inc)
{
    for(int i = s; i < len - inc; i++) 
    {
        opti_array[i] = opti_array[i + inc];
    }
    
    // Fill the tail with '0's
    for(int i = len - inc; i < len; i++) {
        opti_array[i] = '0';
    }
}

void forward()
{
  digitalWrite(DIR1, 0);
  digitalWrite(DIR2, 0);
}

void wait()
{
  analogWrite(PWM1,  0);
  analogWrite(PWM2, 0);
}

void straight(int s)
{
  digitalWrite(DIR1, 0);
  digitalWrite(DIR2, 0);
  analogWrite(PWM1,  s);
  analogWrite(PWM2, s);
}

void left(int q)
{
  digitalWrite(DIR1, 0);
  digitalWrite(DIR2, 1);
  analogWrite(PWM1,  q);
  analogWrite(PWM2, q);
  if(!(digitalRead(ir3)==1||digitalRead(ir3)==1))
  {
    left(q);
  }
}

void right(int r)
{
  digitalWrite(DIR1, 1);
  digitalWrite(DIR2, 0);
  analogWrite(PWM1, r);
  analogWrite(PWM2, r);
  if(!(digitalRead(ir3)==1||digitalRead(ir3)==1))
  {
    right(r);
  }
}

void uturn(int s){
  while(!(digitalRead(ir3)== 1 && digitalRead(ir4)== 1)){
    left(s);
  }
  computePID();
}

bool allwhite()
{
  if( digitalRead(ir1) == 1 && digitalRead(ir2) == 1 && digitalRead(ir3) == 1 && digitalRead(ir4) == 1 && digitalRead(ir5) == 1 && digitalRead(ir6) == 1 )
  {
    return true;
  }
  else
  {
    return false;
  }
}

void computePID()
{
  InputL= digitalRead(ir3);
  InputR = digitalRead(ir4);
  forward();
  // Compute PID control output
  myPIDL.Compute();
  myPIDR.Compute();
  analogWrite(PWM1, OutputR);
  analogWrite(PWM2, OutputL);
}
