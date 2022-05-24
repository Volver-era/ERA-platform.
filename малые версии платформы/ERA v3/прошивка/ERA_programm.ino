#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <ServoDriverSmooth.h>
#include <ServoSmooth.h>
#include <smoothUtil.h>
#include "giro_acsel.cpp"

incline1 giro;
ServoSmooth s[12];
int ports[12] = {PA0, PA1, PA2,        PA3, PA4, PA5,       PB10, PB2, PB1,      PB0, PA7,PA6,}; //for stm32
int zeroPositions[12] = {90, 86, 139,      90, 87, 24,      90, 86, 145,      95, 93, 30};
int startPositions[12] = {90, 0, 0,      90, 158, 160,      90, 6, 0,      95, 164, 162};
int directions[12] = {-1, -1, -1,   -1, +1, +1,      +1, -1, -1,     +1, +1, +1};
const float L1 = 7; 
const float L2 = 7.5; 

void pos(float x, float y, float z, int leg){
  // pos(x,y,z,нога)
  // pos((+назад, -вперед),(+влево, -вправо),(+вниз, -вверх),(номер ноги))
  
  float groinRadians = atan(y/z);
  float groinDegrees = groinRadians * (180/PI);

  float hipRadians1 = atan(x/z);

  float z2 = sqrt(sq(z) + sq(y) + sq(x)); 
  
  float hipRadians2 = acos((sq(L1)+sq(z2)-sq(L2))/(2*L1*z2));
  float hipDegrees = (hipRadians1 + hipRadians2) * (180/PI);
  
  float kneeRadians = acos((sq(L1)+sq(L2)-sq(z2))/(2*L1*L2));

  float kneeDegrees = 180 - kneeRadians * (180/PI);

  
  s[3*leg+2].setTargetDeg(zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees);
  s[3*leg+1].setTargetDeg(zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
  s[3*leg].setTargetDeg(zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
  
  


}
void translate(float x, float y, float z){
  for (int leg = 0; leg<=3; leg++){
    pos(x,y,z,leg);
  }
}
int legServoAngles[12];
void PositionCalculation(float x, float y, float z, int leg){
float groinRadians = atan(y/z);
float groinDegrees = groinRadians * (180/PI);

float hipRadians1 = atan(x/z);

float z2 = sqrt(sq(z) + sq(y) + sq(x));

float hipRadians2 = acos((sq(L1)+sq(z2)-sq(L2))/(2*L1*z2));
float hipDegrees = (hipRadians1 + hipRadians2) * (180/PI);

float kneeRadians = acos((sq(L1)+sq(L2)-sq(z2))/(2*L1*L2));

float kneeDegrees = 180 - kneeRadians * (180/PI);

legServoAngles[3*leg] = (zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
legServoAngles[3*leg+1] = (zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
legServoAngles[3*leg+2] = (zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees);

}
void ApplyPosition(int leg){
s[3*leg].setTargetDeg(legServoAngles[3*leg]);
s[3*leg+1].setTargetDeg(legServoAngles[3*leg+1]);
s[3*leg+2].setTargetDeg(legServoAngles[3*leg+2]);
}
void ApplyAllPosition(){
for(int leg=0;leg<4;leg++)
{
s[3*leg].setTargetDeg(legServoAngles[3*leg]);
s[3*leg+1].setTargetDeg(legServoAngles[3*leg+1]);
s[3*leg+2].setTargetDeg(legServoAngles[3*leg+2]);
}
}
void SetAccel(int Accel){
  for(int i=0; i<12; i++){
    s[i].setAccel(Accel);
  }

  s[1].setAccel((int)(Accel*0.5));
  s[4].setAccel((int)(Accel*0.5));
  s[7].setAccel((int)(Accel*0.5));
  s[10].setAccel((int)(Accel*0.5));
}
void setup() {
  giro.initialize();
  Serial.begin(9600);
  Serial1.begin(9600);
  //Инициализируем сервы, настраиваем скорости, стартовые позиции и ускорения
  for(int i=0; i<12; i++){
     s[i].attach(ports[i],startPositions[i]);
     s[i].setAutoDetach(false); // вырубаем оключения сервы, чтобы робот не упал
     s[i].setSpeed(360);
     //s[i].setAccel(0);
 //    Serial.print("Port: ");
   //   Serial.println(i);
    // Test_delay(5000);
  //s[i].setMaxAngle(270);
  }
  SetAccel(600);

/*  Timer3.pause();
  Timer3.setPrescaleFactor(1800); // 40 кГц
  Timer3.attachInterrupt(TIMER_UPDATE_INTERRUPT, callback);
  Timer3.refresh(); // обнулить таймер
  Timer3.resume(); // запускаем таймер таймер*/


  delay(1000);
  translate(0,0,11); //Сгибаем ноги
  Test_delay(5000);
 // GoDistance(20);

  PositionCalculation(0,0,11,0);
  PositionCalculation(0,0,11,1);
  PositionCalculation(0,0,11,2);
  PositionCalculation(0,0,11,3); 
}
void Test_delay(int p){
  if(p==0) return; //Если пауза нулевая, выходим из функции
  int minDelay=10;
  while(p>minDelay)
  {
    for(int i=0; i<12; i++){
     s[i].tick();
    }
    delay(9);
    p-=10;
  }
  delay(10);
}
bool CommonTick(){
  bool f = true;
  int degAccurancy = 1;
  for(int i=0; i<12; i++)
  {
    //if(s[i].tick()) f=true;
    s[i].tick();
    if(abs(s[i].getCurrentDeg()-s[i].getTargetDeg())>degAccurancy)f=false;
  }
  delay(20);
  return f;
}
bool OldCommonTick(){
  bool f = true;
 
  for(int i=0; i<12; i++)
  {
    if(!s[i].tick()) f=false;
  }
  delay(20);
  return f;
}
void hodba(int stepLength,int Accel=1500) {
  float lpnoga=11;
  float rpnoga=11;
  float lznoga=11;
  float rznoga=11;
  //float p=-5.5;
  float p=-2.5;
  float d=3.5;
  if (stepLength<0){
    lpnoga+=2;
    rpnoga+=2;
    lznoga-=1;
    rznoga-=1;
  }
  SetAccel(Accel);
  pos(-stepLength,-p,lpnoga-d,0);
  pos(0,-p,lznoga,1);
  pos(0,-p,rpnoga,2);
  pos(+stepLength,-p,rznoga,3); 
  while(!CommonTick());
  //Левая передняя нога вперёд и вверх, правая задняя нога назад, корпус смещён вправо
  
  pos(0,0,lpnoga,0);
  pos(+stepLength,0,lznoga,1);
  pos(+stepLength,0,rpnoga,2);
  pos(+2*stepLength,0,rznoga,3);
  while(!CommonTick());
  //Правая передняя и правая задняя ноги назад
  
  pos(0,p,lpnoga,0);
  pos(+stepLength,p,lznoga,1);
  pos(+stepLength,p,rpnoga,2);
  pos(0,p,rznoga-d,3);
  while(!CommonTick());
  //Левая передняя нога вниз, правая задняя нога вперёд и вверх, корпус вперёд
  
  pos(0,p,lpnoga,0);
  pos(+stepLength,p,lznoga,1);
  pos(+stepLength,p,rpnoga,2);
  pos(0,p,rznoga,3);
  while(!CommonTick());
  //Левая задняя и правая передняя нога вперёд
  
  pos(0,p,lpnoga,0);
  pos(-stepLength,p,lznoga-d,1);
  pos(+stepLength,p,rpnoga,2);
  pos(0,p,rznoga,3);
  while(!CommonTick());
  //Левая задняя нога назад и вверх, правая передняя нога вперёд
  
  pos(+stepLength,0,lpnoga,0);
  pos(0,0,lznoga,1);
  pos(+2*stepLength,0,rpnoga,2);
  pos(+stepLength,0,rznoga,3);
  while(!CommonTick());
  //Левая передняя и правая задняя вперёд, правая передняя нога дважды вперёд
  
  pos(+stepLength,-p,lpnoga,0);
  pos(0,-p,lznoga,1);
  pos(0,-p,rpnoga-d,2);
  pos(+stepLength,-p,rznoga ,3); 
  while(!CommonTick());
  //Правая передняя нога вверх, левая передняя и правая задняя назад
  
  pos(+stepLength,-p,lpnoga,0);
  pos(0,-p,lznoga,1);
  pos(0,-p,rpnoga,2);
  pos(+stepLength,-p,rznoga,3);
  while(!CommonTick());
  //Левая передняя нога и правая задняя нога вперёд
  
}
void ran(float lStep,float rStep,int Accel=1500) {
  float lpnoga=9;
  float rpnoga=9;
  float lznoga=11;  
  float rznoga=11;
  float p=-2.5;
  float d=2.5;
  SetAccel(Accel);
  if (lStep<0&& rStep<0){
    lpnoga+=2;
    rpnoga+=2;
    lznoga-=1;
    rznoga-=1;
  }
  pos(-lStep,0,lpnoga-d,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(-rStep,0,rznoga-d,3); 
  while(!CommonTick());
  //левая передняя и правая задняя нога вперед и вверх

  pos(-lStep,0,lpnoga,0);
  pos(rStep,0,rpnoga,1);
  pos(lStep,0,lznoga,2);
  pos(-rStep,0,rznoga,3); 
  while(!CommonTick());
  //левая передняя и правая задняя нога вниз

  pos(0,0,lpnoga,0);
  pos(-rStep,0,rpnoga-d,1);
  pos(-lStep,0,lznoga-d,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());
  //правая передняя и левая задняя нога вперед и вверх

  pos(lStep,0,lpnoga,0);
  pos(-rStep,0,rpnoga,1);
  pos(-lStep,0,lznoga,2);
  pos(rStep,0,rznoga,3); 
  while(!CommonTick());
  //правая передняя и левая задняя нога вниз

}
void v_storony(int stepLength,int Accel=1500) {
  float lpnoga=11;
  float rpnoga=11;
  float lznoga=11;  
  float rznoga=11;
  float d=2;
  SetAccel(Accel);
  
  pos(0,-0.5*stepLength,lpnoga-d,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(0,-0.5*stepLength,rznoga-d,3); 
  while(!CommonTick());
  //левая передняя и правая задняя нога вбок и вверх

  pos(0,-stepLength,lpnoga,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(0,-stepLength,rznoga,3); 
  while(!CommonTick());

  pos(0,0,lpnoga,0);
  pos(0,-0.5*stepLength,rpnoga-d,1);
  pos(0,-0.5*stepLength,lznoga-d,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  pos(0,0,lpnoga,0);
  pos(0,-stepLength,rpnoga,1);
  pos(0,-stepLength,lznoga,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  

  //Test_delay(10000);

}
void povorot(int stepLength,int Accel=1500){
  float lpnoga=11;
  float rpnoga=11;
  float lznoga=11;  
  float rznoga=11;
  float d=2;
  SetAccel(Accel);
  
  pos(0,0,lpnoga,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  pos(0,0,lpnoga,0);
  pos(0.5*stepLength,-0.5*stepLength,rpnoga-d,1);
  pos(-0.5*stepLength,0.5*stepLength,lznoga-d,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  pos(0,0,lpnoga,0);
  pos(stepLength,-stepLength,rpnoga,1);
  pos(-stepLength,stepLength,lznoga,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  pos(-0.5*stepLength,-0.5*stepLength-d,lpnoga,0);
  pos(stepLength,-stepLength,rpnoga,1);
  pos(-stepLength,stepLength,lznoga,2);
  pos(0.5*stepLength,0.5*stepLength-d,rznoga,3); 
  while(!CommonTick());

  pos(-stepLength,-stepLength,lpnoga,0);
  pos(stepLength,-stepLength,rpnoga,1);
  pos(-stepLength,stepLength,lznoga,2);
  pos(stepLength,stepLength,rznoga,3); 
  while(!CommonTick());
  
  pos(0,0,lpnoga,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  Test_delay(10000);
}
void GoDistance(float distance, float Speed = 5){
  Serial.print("GoDistance: ");
  Serial.println(distance);
  int maxRunSpeed=100;
  int maxWalkSpeed=40;
  int sign=1;
  if(Speed>maxRunSpeed)Speed=maxRunSpeed;
  if (distance<0) {
    distance*=-1;
    sign=-1;
  }
  
  while(distance>0)
  {
    if(Speed<maxWalkSpeed)
    {
      int CommonSpeed = Speed*37;
      hodba(3*sign,CommonSpeed);
    }
    else
    {
      int CommonSpeed = Speed*15;
      if(CommonSpeed==1500) CommonSpeed=0;
      ran(3*sign,3*sign,CommonSpeed);
    }
    
    distance-=6;
  }
}
void prised() {
translate(0,0,13);
while(!CommonTick());
translate(0,0,8);
while(!CommonTick());
}
void Hi(){
  float lpnoga=11;
  float rpnoga=11;
  float lznoga=11;
  float rznoga=11;
  float p=-2.5;
  float d=2.5;
  // pos(x(вперед),y(в сторону),z(вверх),нога);
  SetAccel(600);
  pos(0,0,lpnoga,0);
  pos(0,0,rpnoga+0.5*d,1);
  pos(0,0,lznoga-d,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  s[4].setTargetDeg(40);
  s[5].setTargetDeg(140);
  while(!CommonTick());

  s[3].setTargetDeg(110);
  while(!CommonTick());

  s[3].setTargetDeg(70);
  while(!CommonTick());

  s[3].setTargetDeg(110);
  while(!CommonTick());

  s[3].setTargetDeg(70);
  while(!CommonTick());

  s[3].setTargetDeg(110);
  while(!CommonTick());

  s[3].setTargetDeg(70);
  while(!CommonTick());

  pos(0,0,lpnoga,0);
  pos(0,0,rpnoga+0.5*d,1);
  pos(0,0,lznoga-d,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());

  pos(0,0,lpnoga,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());
  
  Test_delay(2000);

}
void Jump(){
  SetAccel(1000);
  translate(0,0,9);
  while(!CommonTick());
  //SetAccel(0);
  translate(0,0,12);
  while(!CommonTick());
  //SetAccel(1400);
}
int g=1;
void hod(int f) {
  if(f>=g) {
    hodba(3);
    g+=1;
  }
  else {
    translate(0,0,13);
    while(!CommonTick());
  }
}
int ReadNumber(){
  int n=0;
  int sign=1;
  
  char c = Serial.read();
  if(c=='-') sign = -1;
  else n= c-'0';
  
  while(Serial.available())
  {
    c = Serial.read();
    if(c!=10) n+=n*10+c-'0';
  }
  return n*sign;
}
int myReadNumber(){
  int n=0;
  int sign=1;
  
  char c = Serial1.read();
  if(c=='-') sign = -1;
  else n= c-'0';
  
  while(Serial1.available())
  {
    c = Serial1.read();
    if(c!=10) n+=n*10+c-'0';
  }
  return n*sign;
}
int mainSpeed = 20;
void ControlSerial(){
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if(c=='S'){
      int sp = ReadNumber();
      if (sp<0) sp = 0;
      if (sp>100) sp=100;
      mainSpeed = sp;
      Serial.println ("speed OK");
      }
    if(c=='G'){
      int sp = ReadNumber();
      if (sp<-100) sp = -100;
      if (sp>100) sp=100;
      GoDistance(sp,mainSpeed);
      Serial.println ("distance OK");
    }
    if(c=='H'){
      Hi();
      Serial.println ("Hi OK");
    }
    if(c=='R'){
      int sp = ReadNumber();
      if (sp==1)
      {
        for (int i=0;i <8;i++) ran(-1,3,mainSpeed*15);
      }
      else 
      {
        for (int i=0;i <8;i++) ran(3,-2,mainSpeed*15);
      }
      Serial.println ("ran OK");
  
    }
    if(c=='P'){
      prised();
      Serial.println ("prised OK");
    }
    if(c=='J'){
      Jump();
      Serial.println ("Jump OK");
    }

    
  }
}
void ControlSerialBl(){
  if (Serial1.available() > 0)
  {
    char c = Serial1.read();
    if(c=='S'){
      int sp = myReadNumber();
      if (sp<0) sp = 0;
      if (sp>100) sp=100;
      mainSpeed = sp;
      Serial1.println ("speed OK");
      }
    if(c=='G'){
      int sp = myReadNumber();
      if (sp<-100) sp = -100;
      if (sp>100) sp=100;
      GoDistance(sp,mainSpeed);
      Serial1.println ("distance OK");
    }
    if(c=='H'){
      Hi();
      Serial1.println ("Hi OK");
    }
    if(c=='R'){
      int sp = myReadNumber();
      if (sp==1)
      {/*
        for (int i=0;i <1;i++) */ran(-1,3,mainSpeed*15);
      }
      else 
      {/*
        for (int i=0;i <8;i++)*/ ran(3,-2,mainSpeed*15);
      }
      Serial1.println ("ran OK");
  
    }
    if(c=='P'){
      prised();
      Serial1.println ("prised OK");
    }
    if(c=='J'){
      Jump();
      Serial1.println ("Jump OK");
    }

    
  }
}

void loop(){
  translate(0,0,11);
  while(!CommonTick());
  delay(10);
  
  giro.tick();
  Serial.print(giro.anglex);Serial.print('\t');
  Serial.print(giro.angley);Serial.print('\t');
  Serial.println(giro.anglez);Serial.print('\t');
  ControlSerial();
  ControlSerialBl();
}
