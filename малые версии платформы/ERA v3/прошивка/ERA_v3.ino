#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <ServoDriverSmooth.h>
#include <ServoSmooth.h>
#include <smoothUtil.h>
ServoSmooth s[12];
int ports[12] = {PA0, PA1, PA2,        PA3, PA4, PA5,       PB10, PB2, PB1,      PB0, PA7,PA6,}; //for stm32
int zeroPositions[12] = {90, 86, 139,      90, 87, 24,      90, 86, 145,      95, 93, 30};
int directions[12] = {-1, -1, -1,   -1, +1, +1,      +1, -1, -1,     +1, +1, +1};
const float L1 = 7; 
const float L2 = 7.5; 
void pos(float x, float y, float z, int leg){
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
void GoDistance(float distance, float Speed = 5)
{
  while(distance>0)
  {
  //  hodba(0,Speed);
    distance-=Speed;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  //Инициализируем сервы, настраиваем скорости, стартовые позиции и ускорения
  for(int i=0; i<12; i++){
     s[i].attach(ports[i],zeroPositions[i]);
     s[i].setAutoDetach(false); // вырубаем оключения сервы, чтобы робот не упал
     s[i].setSpeed(360);
     //s[i].setAccel(0.9);
     s[i].setAccel(1400);
 //    Serial.print("Port: ");
   //   Serial.println(i);
    // Test_delay(5000);
  //s[i].setMaxAngle(270);
  }
  
/*  s[0].setAccel(0);//Уменьшаем скорости самых верхних серв, т.к. на высокой скорости слишком большая инерция и робота вбок сносит
  s[3].setAccel(0);
  s[6].setAccel(0);
  s[9].setAccel(0);*/
  delay(1000);
  translate(0,0,9); //Сгибаем ноги
  Test_delay(5000);
 // GoDistance(20);
}
//ФУНКЦИЯ КОТОРАЯ ВЫЗЫВАЕТ МЕТОД TICK У СЕРВ ВО ПРЕМЯ ПАУЗ. ИСПОТЛЬЗУЕТСЯ ВМЕСТО ОБЫЧНОГО delay
void Test_delay(int p)
{
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






//ФУНКЦИЯ КОТОРАЯ ПРОВЕРЯЕТ ДОШЛИ ЛИ ДО КОНЦА СЕРВ, она же вызывает tick
//Сделано чтобы вместо delay использовать while(!CommonTick()) 
bool CommonTick()
{
  bool f = true;
  for(int i=0; i<12; i++)
  {
    if(!s[i].tick()) f=false;
  }
  delay(20);
  return f;
}

//Далее идёт функция самой ходьбы: stepLength=6, del=1000
void hodba(int stepLength,int del) {
  float lpnoga=13;
  float lznoga=13;
  float rpnoga=13;
  float rznoga=13;
  //float p=-5.5;
  float p=-2.5;
  float d=3.5;

  //pos(x,y,z,0);
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
void ran(int stepLength) {
  float lpnoga=12;
  float lznoga=13;
  float rpnoga=12;
  float rznoga=13;
  float p=-2.5;
  float d=2.5;
  // pos(x,y,z,нога);
  
  pos(-stepLength,0,lpnoga-d,0);
  pos(0,0,rpnoga,1);
  pos(0,0,lznoga,2);
  pos(-stepLength,0,rznoga-d,3); 
  while(!CommonTick());
  //левая передняя и правая задняя нога вперед и вверх

  pos(-stepLength,0,lpnoga,0);
  pos(stepLength,0,rpnoga,1);
  pos(stepLength,0,lznoga,2);
  pos(-stepLength,0,rznoga,3); 
  while(!CommonTick());
  //левая передняя и правая задняя нога вниз

  pos(0,0,lpnoga,0);
  pos(-stepLength,0,rpnoga-d,1);
  pos(-stepLength,0,lznoga-d,2);
  pos(0,0,rznoga,3); 
  while(!CommonTick());
  //правая передняя и левая задняя нога вперед и вверх

  pos(stepLength,0,lpnoga,0);
  pos(-stepLength,0,rpnoga,1);
  pos(-stepLength,0,lznoga,2);
  pos(stepLength,0,rznoga,3); 
  while(!CommonTick());
  //правая передняя и левая задняя нога вниз

}
void prised() {
translate(0,0,13);
while(!CommonTick());
translate(0,0,8);
while(!CommonTick());
}

int g=1;
void hod(int f) {
  if(f>=g) {
    hodba(6,1000);
    g+=1;
  }
  else {
    translate(0,0,18);
    while(!CommonTick());
  }
}


//сюда пропиываются фунцкии, которые нужно выполнить
void loop(){
//hodba(3,1000);
//prised();
ran(-3);
}
