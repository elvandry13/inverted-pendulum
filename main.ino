#include <digitalWriteFast.h>

volatile int temp, counter = 0;
unsigned long int milli_time;
float jarak, jarak2;
float val, Angle, Angle2;

int RPWM = 5;
int LPWM = 6;
int L_EN = 7;
int R_EN = 8;

float output1, output2, output3;
float P, I, D, KP, KI, KD;
float P2, KP2;
float error, lasterror;
float error2;
float PWM, fpwm, sp;

void setup()
{
  Serial.begin(115200);
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Computer Time,Time (Mili Sec.),Sudut");
  for(int i=5;i<9;i++)
  {
    pinMode(i,OUTPUT);
  }
  for(int i=5;i<9;i++)
  {
   digitalWrite(i,LOW);
  }
  pinMode(2, INPUT_PULLUP); 
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0, RISING);   
  attachInterrupt(1, ai1, RISING);
  for(int i=30;i<40;i++)
  {
    pinModeFast(i, INPUT);
  }
  for(int i=30;i<40;i++)
  {
    digitalWriteFast(i, LOW);
  }
  fpwm = 10;
  sp = 0;
  KP = 40;
  KI = 0.2;
  KD = 100; 
  KP2 = 15;
}

void Rotary()
{
  const int b0 = digitalReadFast(30);
  const int b1 = digitalReadFast(31);
  const int b2 = digitalReadFast(32);
  const int b3 = digitalReadFast(33);
  const int b4 = digitalReadFast(34);
  const int b5 = digitalReadFast(35);
  const int b6 = digitalReadFast(36);
  const int b7 = digitalReadFast(37);
  const int b8 = digitalReadFast(38);
  const int b9 = digitalReadFast(39);
  byte c0 = b0;
  byte c1 = c0 + b1;
    if (c1==2){c1= 0;}
  byte c2 = c1 + b2;
    if (c2==2){c2= 0;}
  byte c3 = c2 + b3;
    if (c3==2){c3= 0;}
  byte c4 = c3 + b4;
    if (c4==2){c4= 0;}
  byte c5 = c4 + b5;
    if (c5==2){c5= 0;}
  byte c6 = c5 + b6;
    if (c6==2){c6= 0;}
  byte c7 = c6 + b7;
    if (c7==2){c7= 0;}
  byte c8 = c7 + b8;
    if (c8==2){c8= 0;}
  byte c9 = c8 + b9;
    if (c9==2){c9= 0;}
  val = c0*512 + c1*256 + c2*128 + c3*64 + c4*32 + c5*16 + c6*8 + c7*4 + c8*2 + c9*1 ;
  Angle = 360.0/1024*val;
  
  if(Angle >= 0)
  {
    Angle2 = Angle - 180;
  }

  if(Angle < 0)
  {
    Angle2 = Angle + 180;
  }
  
  if (Angle > 180)
  {
    Angle = Angle - 360;
    Angle2 = Angle + 180;
  }

  if (Angle < -180) 
  {
    Angle = Angle + 360;
    Angle2 = Angle - 180;
  }
}

void PID()
{
  error = sp + Angle2;
  P = KP * error;
  I = I + (KI*error);
  D = KD * (error - lasterror);
  lasterror = error;
  output1 = P + I + D;
}

void posisi() 
{
  if( counter != temp )
  {
    jarak = counter * (38.0/3000.0);
    jarak2 = (-1)*jarak;
    temp = counter;
  }
}

void ai0() 
{
  if(digitalRead(3)==LOW) 
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void ai1() 
{
  if(digitalRead(2)==LOW) 
  {
    counter--;
  }
  else
  {
    counter++;
  }
}

void PID2()
{
  error2 = jarak2;
  P2 = KP2 * error2;
  output2 = P2;
}

void loop() {
  milli_time = millis();  
  Rotary();
  PID();
  posisi();
  PID2();
  output3 = output2 + output1;

  if (output3 > 0 )
  {
    PWM = fpwm + output3;
    if (PWM > 255 ) PWM = 255;
    digitalWrite(R_EN,HIGH);
    digitalWrite(L_EN,HIGH);
    analogWrite(RPWM,PWM);
    analogWrite(LPWM,0);
  }
  
  if (output3 < 0 )
  {
    PWM = fpwm - output3;
    if (PWM > 255 ) PWM = 255;
    digitalWrite(R_EN,HIGH);
    digitalWrite(L_EN,HIGH);
    analogWrite(RPWM,0);
    analogWrite(LPWM,PWM);
  }
  Serial.print("DATA,TIME,");
  Serial.print(milli_time);
  Serial.print(",");
  Serial.print(Angle2);
  Serial.print(",");
  Serial.println(jarak2);
}

