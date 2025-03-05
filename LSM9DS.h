#include <Adafruit_LSM9DS0.h>
Adafruit_LSM9DS0 lsm9ds = Adafruit_LSM9DS0();

bool init_sensors(void)
{
  if (!lsm9ds.begin())
  {
    return false;
  }
  accelerometer = &lsm9ds.getAccel();
  gyroscope = &lsm9ds.getGyro();
  magnetometer = &lsm9ds.getMag();

  return true;
}

void setup_sensors(void)
{
#ifdef __LSM9DS0_H__
  lsm9ds.setupAccel(lsm9ds.LSM9DS0_ACCELRANGE_6G);
  lsm9ds.setupMag(lsm9ds.LSM9DS0_MAGGAIN_4GAUSS);
  lsm9ds.setupGyro(lsm9ds.LSM9DS0_GYROSCALE_2000DPS);
#else
  lsm9ds.setupAccel(lsm9ds.LSM9DS1_ACCELRANGE_6G);
  lsm9ds.setupMag(lsm9ds.LSM9DS1_MAGGAIN_4GAUSS);
  lsm9ds.setupGyro(lsm9ds.LSM9DS1_GYROSCALE_2000DPS);
#endif
}
