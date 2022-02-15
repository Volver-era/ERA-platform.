#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <ServoDriverSmooth.h>
#include <ServoSmooth.h>
#include <smoothUtil.h>


ServoSmooth s[12];
//int zeroPositions[12] = {93, 102, 85, 83, 90, 85, 92, 82, 85, 90, 85, 90};
//int zeroPositions[12] = {92, 95, 141, 95, 80, 36, 95, 91, 142, 96, 89, 45};
int zeroPositions[12] = {105,70,30 ,80,113 ,160 ,95 ,75 ,10 ,103 ,103 ,160 };
int directions[12] = {+1, +1, +1, +1, -1, -1, -1, +1, +1, -1, -1, -1};



const float L1 = 7; //11.2 длинна бедра
const float L2 = 7; //13.2 длинна голени используются в расчетах

//Сама функция с расчетом по инверсной киниматике, передаетё сюда координаты где должна оказаться пятка,
//а функция сама считает углы и двигает сервы
/*
void pos(float x, float y, float z, int leg){

  Serial.print("Servo ");
  Serial.print(leg);
  Serial.print(" X=");
  Serial.print(x);
  Serial.print(" Y=");
  Serial.print(y);
  Serial.print(" Z=");
  Serial.println(z);
  
  float groinRadians = atan(y/z);
  float groinDegrees = groinRadians * (180/PI);

  float hipRadians1 = atan(x/z);

  float z2 = sqrt(sq(z) + sq(y) + sq(x)); 
  
  float hipRadians2 = acos((sq(L1)+sq(z2)-sq(L2))/(2*L1*z2));
  float hipDegrees = (hipRadians1 + hipRadians2) * (180/PI);
  
  float kneeRadians = acos((sq(L1)+sq(L2)-sq(z2))/(2*L1*L2));

  float kneeDegrees = 180 - kneeRadians * (180/PI);
  Serial.print(groinDegrees);
  Serial.print(" ");
  Serial.print(hipDegrees);
  Serial.print(" ");
  Serial.println(kneeDegrees);

  s[3*leg].setTargetDeg(zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
  s[3*leg+1].setTargetDeg(zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
  s[3*leg+2].setTargetDeg(zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees);

  Serial.print(zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
  Serial.print(" ");
  Serial.print(zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
  Serial.print(" ");
  Serial.println(zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees);
}
*/

//Сама функция с расчетом по инверсной киниматике, передаетё сюда координаты где должна оказаться пятка,
//а функция сама считает углы и двигает сервы  (Сокращенная, считает хуже, большая погрешность но немного быстрее)

void pos(float x, float y, float z, int leg){

  float l = 7;
  Serial.print("Servo ");
  Serial.print(leg);
  Serial.print(" X=");
  Serial.print(x);
  Serial.print(" Y=");
  Serial.print(y);
  Serial.print(" Z=");
  Serial.println(z);
  
  float groinRadians = atan(y/z);
  float groinDegrees = groinRadians * (180/PI);

  float hipRadians1 = atan(x/z);

  float z1 = sqrt(sq(z) + sq(y) + sq(x)); 
  
  float hipRadians2 = acos(z1/(2*l));
  float hipDegrees = (hipRadians1 + hipRadians2) * (180/PI);
  
  float kneeRadians = PI - 2*hipRadians2;
  float kneeDegrees = 180 - kneeRadians * (180/PI);
 
  Serial.println(groinDegrees);
  Serial.println(hipDegrees);
  Serial.println(kneeDegrees);

  s[3*leg].setTargetDeg(zeroPositions[3*leg] + directions[3*leg]*groinDegrees);
  s[3*leg+1].setTargetDeg(zeroPositions[3*leg+1] + directions[3*leg+1]*hipDegrees);
  s[3*leg+2].setTargetDeg(zeroPositions[3*leg+2] + directions[3*leg+2]*kneeDegrees);
  
}


//Сдвигает все ноги в нужную позицию
void translate(float x, float y, float z){
  for (int leg = 0; leg<=3; leg++){
    pos(x,y,z,leg);
    
  }
}

//вроде поворот, но как-то не очень работает
void rotate(float theta){
  float thetaRadians = theta * (PI/180);
  float d = tan(thetaRadians) * (L2/2);

  Serial.println(d);

  pos(0,0,8.0+d,0);
  pos(0,0,8.0+d,1);
  pos(0,0,8.0-d,2);
  pos(0,0,8.0-d,3);
}


//немного более тормозная ходьба в 3 движения на ногу
void walk2(float stepSpeed, float stepLength)
{
  int p=-6;
Serial.println("1 position");
  pos(0,-p,14,0);
  pos(0,-p,18,1);
  pos(0,-p,18,2);
  pos(0,-p,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("2 position");
  pos(-stepLength,-p,14,0);
  pos(0,-p,18,1);
  pos(0,-p,18,2);
  pos(0,-p,18,3);
  Test_delay(stepSpeed); 
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("3 position");
  pos(-stepLength,p,18,0);
  pos(+stepLength,p,18,1);
  pos(+stepLength,p,18,2);
  pos(+stepLength,p,18,3);
  Test_delay(stepSpeed); 
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
  
Serial.println("4 position");
  pos(-stepLength,p,18,0);
  pos(+stepLength,p,18,1);
  pos(+stepLength,p,18,2);
  pos(+stepLength,p,14,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("5 position");
  pos(-stepLength,p,18,0);
  pos(+stepLength,p,18,1);
  pos(+stepLength,p,18,2);
  pos(-stepLength,p,14,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("6 position");
  pos(-stepLength,p,18,0);
  pos(+stepLength,p,18,1);
  pos(+stepLength,p,18,2);
  pos(-stepLength,p,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись

Serial.println("7 position");
  pos(-stepLength,p,18,0);
  pos(+stepLength,p,14,1);
  pos(+stepLength,p,18,2);
  pos(-stepLength,p,18,3);
  Test_delay(stepSpeed);  
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись 
Serial.println("8 position");
  pos(-stepLength,p,18,0);
  pos(-stepLength,p,14,1);
  pos(+stepLength,p,18,2);
  pos(-stepLength,p,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("9 position");
  pos(0,-p,18,0);
  pos(0,-p,18,1);
  pos(+stepLength,-p,18,2);
  pos(0,-p,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись

Serial.println("10 position");
  pos(0,-p,18,0);
  pos(0,-p,18,1);
  pos(+stepLength,-p,14,2);
  pos(0,-p,18,3);
  Test_delay(stepSpeed); 
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("11 position");
  pos(0,-p,18,0);
  pos(0,-p,18,1);
  pos(0,-p,14,2);
  pos(0,-p,18 ,3);
  Test_delay(stepSpeed);  
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("12 position");
  pos(0,-p,18,0);
  pos(0,-p,18,1);
  pos(0,-p,18,2);
  pos(0,-p,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
 // delay(10000);
}

//Самая не нормальная на данны й момент ходьба в 2 движения на ногу
void walk3(float stepSpeed, float stepLength)
{
  int p=-5;

Serial.println("2 position");
  pos(-stepLength,-p,14,0);
  pos(0,-p,18,1);
  pos(0,-p,18,2);
  pos(0,-p,18,3);
  Test_delay(stepSpeed); 
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("3 position");
  pos(-stepLength,0,18,0);
  pos(+stepLength,0,18,1);
  pos(+stepLength,0,18,2);
  pos(+stepLength,0,18,3);
  Test_delay(stepSpeed); 
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
  

Serial.println("5 position");
  pos(-stepLength,p,18,0);
  pos(+stepLength,p,18,1);
  pos(+stepLength,p,18,2);
  pos(-stepLength,p,14,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("6 position");
  pos(-stepLength,p,18,0);
  pos(+stepLength,p,18,1);
  pos(+stepLength,p,18,2);
  pos(-stepLength,p,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись

float l = 1;
Serial.println("8 position");
  pos(-stepLength,p*l,18,0);
  pos(-stepLength,p*l,14,1);
  pos(+stepLength,p*l,18,2);
  pos(-stepLength,p*l,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("9 position");
  pos(0,0,18,0);
  pos(0,0,18,1);
  pos(+stepLength,0,18,2);
  pos(0,0,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись


Serial.println("11 position");
  pos(0,-p*l,18,0);
  pos(0,-p*l,18,1);
  pos(0,-p*l,14,2);
  pos(0,-p*l,18 ,3);
  Test_delay(stepSpeed);  
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("12 position");
  pos(0,-p,18,0);
  pos(0,-p,18,1);
  pos(0,-p,18,2);
  pos(0,-p,18,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
 // delay(10000);
}

//Правильная ходьба!!! остальные в принципе можно убирать
void walk4(float stepSpeed, float stepLength)
{
  float p=-2.5; //то на сколько робот будет двигать корпус для смещения центра тяжести
  int uppoz=12;
  int downpoz=10;

Serial.println("2 position");
  pos(-stepLength,-p,downpoz,0);
  pos(0,-p,uppoz,1);
  pos(0,-p,uppoz,2);
  pos(+stepLength,-p,uppoz,3);
  Test_delay(stepSpeed); 
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("3 position");
  pos(0,0,uppoz,0);
  pos(+stepLength,0,uppoz,1);
  pos(+stepLength,0,uppoz,2);
  pos(+2*stepLength,0,uppoz,3);
  Test_delay(stepSpeed); 
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
  

Serial.println("5 position");
  pos(0,p,uppoz,0);
  pos(+stepLength,p,uppoz,1);
  pos(+stepLength,p,uppoz,2);
  pos(0,p,downpoz,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("6 position");
  pos(0,p,uppoz,0);
  pos(+stepLength,p,uppoz,1);
  pos(+stepLength,p,uppoz,2);
  pos(0,p,uppoz,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись


Serial.println("8 position");
  pos(0,p,uppoz,0);
  pos(-stepLength,p,downpoz,1);
  pos(+stepLength,p,uppoz,2);
  pos(0,p,uppoz,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("9 position");
  pos(+stepLength,0,uppoz,0);
  pos(0,0,uppoz,1);
  pos(+2*stepLength,0,uppoz,2);
  pos(+stepLength,0,uppoz,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись


Serial.println("11 position");
  pos(+stepLength,-p,uppoz,0);
  pos(0,-p,uppoz,1);
  pos(0,-p,downpoz,2);
  pos(+stepLength,-p,uppoz,3);
  Test_delay(stepSpeed);  
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
Serial.println("12 position");
  pos(+stepLength,-p,uppoz,0);
  pos(0,-p,uppoz,1);
  pos(0,-p,uppoz,2);
  pos(+stepLength,-p,uppoz,3);
  Test_delay(stepSpeed);
  while(!CommonTick()) delay(5);  //чтобы сервы точно довернулись
}

//ходьба, работает криво, какая-то проблема с координатами
/*void takeStep(float stepSpeed, float stepLength){
  pos(+stepLength,0,18,0);
  pos(-stepLength,0,18,1);
  pos(-stepLength,0,18,2);
  pos(+stepLength,0,18,3);
  Test_delay(stepSpeed);
  pos(+stepLength,0,17,0);
  pos(-stepLength,0,18,1);
  pos(-stepLength,0,18,2);
  pos(+stepLength,0,17,3);
  Test_delay(stepSpeed);
  pos(-stepLength,0,17,0);
  pos(+stepLength,0,18,1);
  pos(+stepLength,0,18,2);
  pos(-stepLength,0,17,3);
  Test_delay(stepSpeed);
  pos(-stepLength,0,18,0);
  pos(+stepLength,0,18,1);
  pos(+stepLength,0,18,2);
  pos(-stepLength,0,18,3);
  Test_delay(stepSpeed);
  pos(-stepLength,0,18,0);
  pos(+stepLength,0,17,1);
  pos(+stepLength,0,17,2);
  pos(-stepLength,0,18,3);
  Test_delay(stepSpeed);
  pos(+stepLength,0,18,0);
  pos(-stepLength,0,17,1);
  pos(-stepLength,0,17,2);
  pos(+stepLength,0,18,3);
  Test_delay(stepSpeed);
}
*/

//странное движение влево-вправо, пока не разбирал
void sideStep(){
  pos(0,0,16,0);
  pos(0,0,16,1);
  pos(0,0,16,2);
  pos(0,0,16,3); 
  while(!CommonTick());
  pos(0,0,20,0);
  pos(0,3,16,1);
  pos(0,0,20,2);
  pos(0,3,16,3);
  while(!CommonTick());
  pos(0,4,18,0);
  pos(0,-4,18,1);
  pos(0,4,18,2);
  pos(0,-4,18,3);
  while(!CommonTick());
  pos(0,0,18,0);
  pos(0,-4,18,1);
  pos(0,0,18,2);
  pos(0,-4,18,3);
  while(!CommonTick());
  pos(0,0,18,0);
  pos(0,0,18,1);
  pos(0,0,18,2);
  pos(0,0,18,3);
  while(!CommonTick());
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
    
  //Инициализируем сервы, настраиваем скорости, стартовые позиции и ускорения
  for(int i=0; i<12; i++){
     s[i].attach(i+22,zeroPositions[i]);
     s[i].setAutoDetach(false); // вырубаем оключения сервы, чтобы робот не упал
     s[i].setSpeed(100);
     s[i].setAccel(1.5);
  }
  s[0].setAccel(0.7);//Уменьшаем скорости самых верхних серв, т.к. на высокой скорости слишком большая инерция и робота вбок сносит
  s[3].setAccel(0.7);
  s[6].setAccel(0.7);
  s[9].setAccel(0.7);
  delay(1000);

  //translate(0,0,10); //Сгибаем ноги
  //Test_delay(5000);
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
  delay(5);
  return f;
}


void loop() 
{
  //walk4(1000,3); 

  
 //translate(0,0,10); //Сгибаем ноги
 //while(!CommonTick());
 //translate(0,0,5); //Сгибаем ноги сильнее (приседания для тестирования погрешности)
 //while(!CommonTick());
 
 // sideStep();
}
