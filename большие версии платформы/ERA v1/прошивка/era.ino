#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <ServoDriverSmooth.h>
#include <ServoSmooth.h>
#include <smoothUtil.h>
ServoSmooth s[12];
//int ports[12] = {2,3,4,5,6,7,8,9,10,11,12,13,14}; //for arduino mega/для ардуино мега
int ports[12] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB10}; //for stm32/для стм32
//int zeroPositions[12] = {88, 97, 131,    95, 80, 36,     95, 98, 142,    96, 88, 38};
int zeroPositions[12] = {88, 93, 131,    95, 83, 40,     95, 91, 100,    96, 92, 73}; //нулевые позиции ног робота
int directions[12] = {-1, +1, -1, -1, -1, +1, +1, +1, -1, +1, -1, +1};
const float L1 = 11.2; //11.2 длина бедра
const float L2 = 13.2; //13.2 длина голени используются в расчетах
//Сама функция с расчетом по инверсной киниматике, передаетё сюда координаты где должна оказаться пятка,
//а функция сама считает углы и двигает сервы
void pos(float x, float y, float z, int leg){
  float l = 12.2;
  float groinRadians = atan(y/z);
  float groinDegrees = groinRadians * (180/PI);
  float hipRadians1 = atan(x/z);
  float z1 = sqrt(sq(z) + sq(y) + sq(x)); 
  float hipRadians2 = acos(z1/(2*l));
  float hipDegrees = (hipRadians1 + hipRadians2) * (180/PI);
  float kneeRadians = PI - 2*hipRadians2;
  float kneeDegrees = 180 - kneeRadians * (180/PI);
  s[3*leg].setTargetDeg(zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
  s[3*leg+1].setTargetDeg(zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
  s[3*leg+2].setTargetDeg(zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees); 
}

//функция для перевода всех ног в одинаковое полоение
void translate(float x, float y, float z){
  for (int leg = 0; leg<=3; leg++){
    pos(x,y,z,leg);
  }
}


void GoDistance(float distance, float Speed = 5)
{
  while(distance>0)
  {
    hodba(0,Speed);
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
     s[i].setSpeed(180);
     //s[i].setAccel(0.9);
     s[i].setAccel(0.7);
     
  }
  s[0].setAccel(0.5);//Уменьшаем скорости самых верхних серв, т.к. на высокой скорости слишком большая инерция и робота вбок сносит
  s[3].setAccel(0.5);
  s[6].setAccel(0.5);
  s[9].setAccel(0.5);
  delay(1000);
  translate(0,0,18); //Сгибаем ноги
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
    delay(7);
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
  float lpnoga=18;
  float lznoga=18;
  float rpnoga=18;
  float rznoga=18;
  //float p=-5.5;
  float p=-3.5;
  float d=3;

  // 1) Левая передняя нога вперёд и вверх, правая задняя нога назад, корпус смещён вправо
  pos(-stepLength,-p,lpnoga-d,0);
  pos(0,-p,rpnoga,1);
  pos(0,-p,lznoga,2);
  pos(+stepLength,-p,rznoga,3); 
  while(!CommonTick());


  
  // 2) Правая передняя и правая задняя ноги назад 
  pos(0,0,lpnoga,0);
  pos(+stepLength,0,rpnoga,1);
  pos(+stepLength,0,lznoga,2);
  pos(+2*stepLength,0,rznoga,3);
  while(!CommonTick());
  


  // 3) Левая передняя нога вниз, правая задняя нога вперёд и вверх, корпус вперёд
  pos(0,p,lpnoga,0);
  pos(+stepLength,p,rpnoga,1);
  pos(+stepLength,p,lznoga,2);
  pos(0,p,rznoga-d,3);
  while(!CommonTick());
  


  // 4) Левая задняя и правая передняя нога вперёд
  pos(0,p,lpnoga,0);
  pos(+stepLength,p,rpnoga,1);
  pos(+stepLength,p,lznoga,2);
  pos(0,p,rznoga,3);
  while(!CommonTick());


  
  // 5) Левая задняя нога назад и вверх, правая передняя нога вперёд
  pos(0,p,lpnoga,0);
  pos(-stepLength,p,rpnoga-d,1);
  pos(+stepLength,p,lznoga,2);
  pos(0,p,rznoga,3);
  while(!CommonTick());
  


  // 6) Левая передняя и правая задняя вперёд, правая передняя нога дважды вперёд
  pos(+stepLength,0,lpnoga,0);
  pos(0,0,rpnoga,1);
  pos(+2*stepLength,0,lznoga,2);
  pos(+stepLength,0,rznoga,3);
  while(!CommonTick());
  


  // 7) Правая передняя нога вверх, левая передняя и правая задняя назад
  pos(+stepLength,-p,lpnoga,0);
  pos(0,-p,rpnoga,1);
  pos(0,-p,lznoga-d,2);
  pos(+stepLength,-p,rznoga ,3); 
  while(!CommonTick());


  
  // 8) Левая передняя нога и правая задняя нога вперёд
  pos(+stepLength,-p,lpnoga,0);
  pos(0,-p,rpnoga,1);
  pos(0,-p,lznoga,2);
  pos(+stepLength,-p,rznoga,3);
  while(!CommonTick());
  
  
}



// данная функция представляет собой первую часть единой функции поворота
void pov1(int stepLength,int del, int p){

  float d=3;
  float h0=18;
  float h1=17;
  float h2=18;
  float h3=17;

   
  pos(-stepLength,-p,h0-d,0);
  pos(0,-p,h1,1);
  pos(0,-p,h2,2);
  pos(0,-p,h3,3);
  
  Test_delay(del); 
  while(!CommonTick());

  pos(-stepLength,-p,h0,0);
  pos(0,-p,h1,1);
  pos(0,-p,h2,2);
  pos(0,-p,h3,3);
  
  Test_delay(del); 
  while(!CommonTick());

  pos(+stepLength,-p,h0,0);
  pos(0,-p,h1,1);
  pos(+2*stepLength,-p,h2,2);
  pos(0,-p,h3,3);
  
  Test_delay(del); 
  while(!CommonTick());

  pos(0,-p-1,h0,0);
  pos(0,-p-1,h1-2,1);
  pos(0,-p-1,h2-d,2);
  pos(0,-p-1,h3,3);
  
  Test_delay(del); 
  while(!CommonTick());
  
  pos(0,-p,h0,0);
  pos(0,-p,h1,1);
  pos(0,-p,h2,2);
  pos(0,-p,h3,3);
  
  Test_delay(del); 
  while(!CommonTick());

  
  }


  // это вторая функция поворота, которая в общей функции поворота будет действовать сразу после первой
  void pov2(int stepLength,int del, int p) {

  float d=3;
  float h0=18;
  float h1=17;
  float h2=18;
  float h3=17;

  pos(0,p,h0,0);
  pos(+stepLength,p,h1-d,1);
  pos(0,p,h2,2);
  pos(0,p,h3,3); 
  while(!CommonTick());

  pos(0,p,h0,0);
  pos(+stepLength,p,h1,1);
  pos(0,p,h2,2);
  pos(0,p,h3,3); 
  while(!CommonTick());

  pos(0,0,h0,0);
  pos(-stepLength,0,h1,1);
  pos(0,0,h2,2);
  pos(-stepLength,0,h3,3); 
  while(!CommonTick());

  pos(0,p+1,h0-2,0);
  pos(0,p+1,h1,1);
  pos(0,p+1,h2,2);
  pos(0,p+1,h3-d,3); 
  while(!CommonTick());

  pos(0,p,h0,0);
  pos(0,p,h1,1);
  pos(0,p,h2,2);
  pos(0,p,h3,3);
  while(!CommonTick());

}



//общая функция поворота s=6, f=-5.5
void pov(int s, int f) {
 pov1(s,1000, f);
 pov2(s,1000, f);
  
}




int g=1;
// функция ходьбы по шагам (сейчас один шаг считается за два), значение переменной f определяет количество шагов (по два шага), которые сделает робот
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


//сюда пропиываются функции, которые нужно выполнить
//функции вызываются так:
//  название_функции(значения_которые_указываются_для_работы_функции);
void loop(){
  hodba(4,1000);
}
