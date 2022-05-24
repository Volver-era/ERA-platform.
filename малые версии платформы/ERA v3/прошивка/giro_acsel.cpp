#include "MPU6050.h"
#define TO_DEG 57.2957f
class incline1 {
    MPU6050 mpu;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    public: float anglex;
    float angley;
    float anglez;
     void initialize() {
      Wire.begin();
      Serial.begin(9600);
      mpu.initialize();
      // состояние соединения
      Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");
      delay(1000);
      mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

    }
    void tick() {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      float accy, accx, accz;
      // преобразование в единицы гравитации при настройках 1G = 4096
      accx = ax / 4096.0;
      accy = ay / 4096.0;
      accz = az / 4096.0;
      // границы от -1G до +1G
      accx = constrain(accx, -1.0, 1.0);
      accy = constrain(accy, -1.0, 1.0);
      accz = constrain(accz, -1.0, 1.0);
      // получить значение в градусах
      if ( accy >= 0) {
        anglex = 90 - TO_DEG * acos(accy);
      } else {
        anglex = TO_DEG * acos(-accy) - 90;
      }
      if ( accx >= 0) {
        angley = 90 - TO_DEG * acos(accx);
      } else {
        angley = TO_DEG * acos(-accx) - 90;
      }
      if ( accz >= 0) {
        anglez = 90 - TO_DEG * acos(accz);
      } else {
        anglez = TO_DEG * acos(-accz) - 90;
      }
      /* Serial.print(anglex);Serial.print('\t');
        Serial.print(angley);Serial.print('\t');
        Serial.println(anglez);
        delay(500);*/
    }
};
