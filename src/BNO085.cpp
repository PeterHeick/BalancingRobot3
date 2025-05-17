#include <Arduino.h>
#include "BNO085.h"

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif

bool bno085Initialized = false;

bool BNO085::begin()
{
  if (!Adafruit_BNO08x::begin_I2C())
  {
    return false;
  }
  setup();
  bno085Initialized = true;
  return true;
}

bool BNO085::setup()
{
  if (bno085Initialized)
  {
    return Adafruit_BNO08x::enableReport(reportType, reportIntervalUs);
  }
  return false;
}

bool BNO085::wasReset()
{
  if (bno085Initialized)
  {
  return Adafruit_BNO08x::wasReset();
  }
  return false;
}

void BNO085::quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees = false)
{

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void BNO085::quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void BNO085::quaternionToEulerGI(sh2_GyroIntegratedRV_t *rotational_vector, euler_t *ypr, bool degrees = false)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

bool BNO085::readSensor(euler_t &pitch)
{
  euler_t ypr;
  sh2_SensorValue_t sensorValue;
  if (!bno085Initialized)
  {
    return false;
  }
  if (Adafruit_BNO08x::wasReset())
  {
    setup();
  }
  if (!Adafruit_BNO08x::getSensorEvent(&sensorValue))
  {
    return false;
  }

  switch (sensorValue.sensorId)
  {
  case SH2_ARVR_STABILIZED_RV:
    quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
    break;
  case SH2_GYRO_INTEGRATED_RV:
    // faster (more noise?)
    quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
    break;
  }
  ypr.status = sensorValue.status;
  pitch = ypr;
  return true;
}