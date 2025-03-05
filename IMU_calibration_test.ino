// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_Sensor_Calibration.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// uncomment one combo 9-DoF!
//#include "LSM6DS_LIS3MDL.h"  // see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
#include "LSM9DS.h"           // LSM9DS1 or LSM9DS0
//#include "NXP_FXOS_FXAS.h"  // NXP 9-DoF breakout

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define PRINT_EVERY_N_UPDATES 100

float gx, gy, gz;

void setup()
{
  Serial.begin(115200);
  while (!Serial) yield();

  if (!cal.begin())
  {
    Serial.println("Failed to initialize calibration helper");
  }
  
  else if (! cal.loadCalibration())
  {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors())
  {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();

  Wire.setClock(400000); // 400KHz
}


void loop()
{
  static uint16_t counter = 0;

  sensors_event_t accel, gyro, mag;

  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
  /*
  Serial.print("Raw Accel: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4);
  Serial.print("      Gyro: ");
  Serial.print(gyro.gyro.x, 4); Serial.print(", ");
  Serial.print(gyro.gyro.y, 4); Serial.print(", ");
  Serial.print(gyro.gyro.z, 4);
  //Serial.print(", ");
  //Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  //Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  //Serial.print(mag.magnetic.z, 4);
  Serial.println();

  Serial.print("Accel Offsets (x,y,z): ");
  Serial.print(cal.accel_zerog[0]); Serial.print(", ");
  Serial.print(cal.accel_zerog[1]); Serial.print(", ");
  Serial.print(cal.accel_zerog[2]);
  Serial.print("      ");
  Serial.print("Gyro Offsets (x,y,z): ");
  Serial.print(cal.gyro_zerorate[0]); Serial.print(", ");
  Serial.print(cal.gyro_zerorate[1]); Serial.print(", ");
  Serial.print(cal.gyro_zerorate[2]);
  //Serial.print("Mag Offsets: ");
  Serial.println();
  */
  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  gx += gyro.gyro.x;
  gy += gyro.gyro.y;
  gz += gyro.gyro.z;

  if (counter++ <= PRINT_EVERY_N_UPDATES)
  {
    delay(1);
    return;
  }

  counter = 0;

  Serial.print(gx);
  Serial.print(" ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.println(gz);
  /*
  Serial.print("Calibrated Accel: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4);
  Serial.print("      Gyro: ");
  Serial.print(gyro.gyro.x, 4); Serial.print(", ");
  Serial.print(gyro.gyro.y, 4); Serial.print(", ");
  Serial.print(gyro.gyro.z, 4);
  //Serial.print(", ");
  //Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  //Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  //Serial.println(mag.magnetic.z, 4);
  Serial.println();
  Serial.println();
  */
}