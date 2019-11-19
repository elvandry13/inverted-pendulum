/*
Kode program untuk menjalankan inverted pendulum
Tugas Akhir yang berjudul "Perancangan dan Implementasi Pendulum Terbalik Menggunakan Kendali PID".
Baca perancangan sistem pada BAB III terlebih dahulu untuk mendapatkan informasi
cara kerja sistem dan spesifikasi hardware yang dipakai
Author : Elvandry Ghiffary Rachman (1102140179)
*/

#include <digitalWriteFast.h>

volatile int temp, counter = 0; //parameter incremental rotary encoder
unsigned long int milli_time;   //waktu dalam millisecond
float jarak, jarak2;            //variabel posisi
float val, Angle, Angle2;       //variabel sudut

//pin motor driver
int RPWM = 5;
int LPWM = 6;
int L_EN = 7;
int R_EN = 8;

//variabel kendali PID sudut dan posisi
float output1, output2, output3;
float P, I, D, KP, KI, KD;
float P2, KP2;
float error, lasterror;
float error2;
float PWM, fpwm, sp;

void setup()
{
  Serial.begin(115200); //baudrate
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Computer Time,Time (Mili Sec.),Sudut");

  //setup pin motor driver
  for(int i=5;i<9;i++)
  {
    pinMode(i,OUTPUT);
  }
  for(int i=5;i<9;i++)
  {
    digitalWrite(i,LOW);
  }

  //setup pin incremental rotary encoder
  pinMode(2, INPUT_PULLUP); 
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0, RISING);   
  attachInterrupt(1, ai1, RISING);

  //setup pin absolute rotary encoder
  for(int i=30;i<40;i++)
  {
    pinModeFast(i, INPUT);
  }
  for(int i=30;i<40;i++)
  {
    digitalWriteFast(i, LOW);
  }

  //setup parameter PID, setpoint, PWM
  fpwm = 10;  //inisialisasi PWM awal
  sp = 0;     //setpoint pada 0 derajat
  KP = 40;    //gain proporsional PID kendali sudut
  KI = 0.2;   //gain integral PID kendali sudut
  KD = 100;   //gain turunan PID kendali sudut
  KP2 = 15;   //gain proporsional PID kendali posisi
}

//membaca sudut pendulum
void Rotary()
{
  //membaca gray code
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

  //konversi gray code ke biner code
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
  
  //konversi biner code ke desimal
  val = c0*512 + c1*256 + c2*128 + c3*64 + c4*32 + c5*16 + c6*8 + c7*4 + c8*2 + c9*1 ;
  
  //konversi deismal ke satuan sudut (derajat)
  Angle = 360.0/1024*val;
  
  //penyetaraan nilai sudut untuk kendali swing-up
  // if(Angle >= 0)
  // {
  //   Angle2 = Angle - 180;
  // }

  // if(Angle < 0)
  // {
  //   Angle2 = Angle + 180;
  // }
  
  // if (Angle > 180)
  // {
  //   Angle = Angle - 360;
  //   Angle2 = Angle + 180;
  // }

  // if (Angle < -180) 
  // {
  //   Angle = Angle + 360;
  //   Angle2 = Angle - 180;
  // }
}

//algoritma PID untuk kendali sudut
void PID()
{
  error = sp + Angle2;
  P = KP * error;
  I = I + (KI*error);
  D = KD * (error - lasterror);
  lasterror = error;
  output1 = P + I + D;
}

//mengukur posisi cart terhadap ujung lintasan dengan incremental rotary encoder
void posisi() 
{
  if( counter != temp )
  {
    jarak = counter * (38.0/3000.0); //counter = output encoder; 38.0 = jarak titik tengah ke ujung lintasan (cm); 3000 = jumlah counter dari titik tengah ke ujung lintasan
    jarak2 = (-1)*jarak;
    temp = counter;
  }
}

//output incremental rotary encoder
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

//output incremental rotary encoder
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

//algoritma PID untuk kendali posisi
void PID2()
{
  error2 = jarak2;
  P2 = KP2 * error2;
  output2 = P2;
}

//main function
void loop() {
  milli_time = millis();        //memulai menghitung waktu
  Rotary();                     //mengukur sudut pendulum
  PID();                        //mengendalikan sudut pendulum
  posisi();                     //mengukur posisi cart
  PID2();                       //mengendalikan posisi cart
  output3 = output2 + output1;  //penjumlahan output kendali sudut pendulum dan kendali posisi cart

  //output PWM Motor DC
  if (output3 > 0 )
  {
    //berputar searah jarum jam
    PWM = fpwm + output3;
    if (PWM > 255 ) PWM = 255;
    digitalWrite(R_EN,HIGH);
    digitalWrite(L_EN,HIGH);
    analogWrite(RPWM,PWM);
    analogWrite(LPWM,0);
  }
  
  if (output3 < 0 )
  {
    //berputar berlawanan arah jarum jam
    PWM = fpwm - output3;
    if (PWM > 255 ) PWM = 255;
    digitalWrite(R_EN,HIGH);
    digitalWrite(L_EN,HIGH);
    analogWrite(RPWM,0);
    analogWrite(LPWM,PWM);
  }

  //parameter-parameter yang ditampilkan pada serial monitor
  Serial.print("DATA,TIME,");
  Serial.print(milli_time); //waktu dalam millisecond
  Serial.print(",");
  Serial.print(Angle);     //sudut pendulum
  Serial.print(",");
  Serial.println(jarak2);  //posisi cart
}

