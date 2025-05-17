#ifndef BNO085_H
#define BNO085_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

struct euler_t
{
  float yaw;
  float pitch;
  float roll;
  uint8_t status;
};

class BNO085 : public Adafruit_BNO08x {
public:
  BNO085(int resetPin = BNO08X_RESET):  Adafruit_BNO08x(resetPin) {}
  bool begin();
  bool setup();
  bool readSensor(euler_t &pitch);
  void calculatePID(float &output, float setPoint, float Kp, float Ki, float Kd);

private:
  Adafruit_BNO08x bno08x;
  sh2_SensorValue_t sensorValue;
  float integral = 0;
  float preError = 0;
  bool wasReset();
  void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees);
  void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees);
  void quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees);
};

extern BNO085 bno085;
#endif // BNO085_H
