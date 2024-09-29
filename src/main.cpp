#include <Arduino.h>

float Kp=40,Ki=0,Kd=150;
float error;
float P;
float I;
float D; 
float pre_error = 0;
int sensor[7] = {0,0,0,0,0,0,0};
int gt_dau=70;
int PID_value = 0;
int PID_P,PID_T;
int dem = 0;

const int EnA = 9;  // phải
const int EnB = 10; // trái
const int InA = 7;
const int InB = 6;
const int InC = 5;
const int InD = 4;

void setup()
{
  Serial.begin(9600); 

  pinMode(InA, OUTPUT);   // bánh phải
  pinMode(InB, OUTPUT);
  pinMode(InC, OUTPUT);   // bánh trái
  pinMode(InD, OUTPUT);

  P=0;
  I=0;
  D=0;
  error=0;

  digitalWrite(InA, HIGH);
  digitalWrite(InB, LOW);
  digitalWrite(InC, HIGH);
  digitalWrite(InD,LOW);
}

void read_sensor()
{
  sensor[0] = digitalRead(A0);
  sensor[1] = digitalRead(A1);
  sensor[2] = digitalRead(A2);
  sensor[3] = digitalRead(A3);
  sensor[4] = digitalRead(A4);
  sensor[5] = digitalRead(A5);
  sensor[6] = digitalRead(12);

  //LỆCH VỀ PHẢI
  if( (sensor[5]==0) && (sensor[4]==0) && (sensor[3]==0) && (sensor[2]==0) && (sensor[1]==1) )
  {
    error = 25; 
  }
  else if( (sensor[5]==0) && (sensor[4]==0) && (sensor[3]==0) && (sensor[2]==1 ) && (sensor[1]==1) )
  {
    error = 20;
  }
  else if( (sensor[5]==0) && (sensor[4]==0) && (sensor[3]==0) && (sensor[2]==1 ) && (sensor[1]==0) )
  {
    error = 15;
  }
  else if( (sensor[5]==0) && (sensor[4]==0) && (sensor[3]==1) && (sensor[2]==1 ) && (sensor[1]==0) )
  {
    error = 10;
  }

  //GIỮA LINE
  else if( (sensor[5]==0) && (sensor[4]==0) && (sensor[3]==1) && (sensor[2]==0) && (sensor[1]==0) ) 
  {
    error = 0;
  }

  //LỆCH VỀ TRÁI 
  else if( (sensor[5]==0) && (sensor[4]==1) && (sensor[3]==1) && (sensor[2]==0) && (sensor[1]==0) )
  {
    error = -10; 
  }
  else if( (sensor[5]==0) && (sensor[4]==1) && (sensor[3]==0) && (sensor[2]==0 ) && (sensor[1]==0) )
  {
    error = -15;
  }
  else if( (sensor[5]==1) && (sensor[4]==1) && (sensor[3]==0) && (sensor[2]==0 ) && (sensor[1]==0) )
  {
    error = -20;
  }
  else if( (sensor[5]==1) && (sensor[4]==0) && (sensor[3]==0) && (sensor[2]==0 ) && (sensor[1]==0) )
  {
    error = -25;
  }
  else if( (sensor[6]==1) && (sensor[0] == 0) )                                
  {
    error = -250;
    delay(50);
  }
  else if( (sensor[6] == 0) && (sensor[0] == 1) )                    
  {
    error = 200;
    delay(50);
  }
}

// /*------------------Cài đặt trạng thái---------------------*/
void vechieu()  // 2 bánh cùng chiều tiến
{
  digitalWrite(InA, HIGH);
  digitalWrite(InB, LOW);
  digitalWrite(InC, HIGH);
  digitalWrite(InD,LOW);
}

void stop(int x)
{ 
  vechieu();
  analogWrite(EnA,0);
  analogWrite(EnB,0);
  delay(x);
}

void run()
{
  read_sensor();
  P = error;
  I = I + error;
  D = error-pre_error;
  PID_value= P*Kp + I*Ki + D*Kd;
  pre_error=error;
   
  float tgt,tgp;
  tgt=gt_dau+PID_value;
  tgp=gt_dau-PID_value;

  PID_T=constrain(tgt,0,150);
  PID_P=constrain(tgp,0,180);

  vechieu();
  analogWrite(EnA,PID_P);
  analogWrite(EnB,PID_T);
}

void loop()
{
  run();
}


