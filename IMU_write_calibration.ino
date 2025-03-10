#include <Adafruit_Sensor_Calibration.h>

// select either EEPROM or SPI FLASH storage:
#ifdef ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  delay(100);
  Serial.println("Calibration filesys test");
  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
    while (1) yield();
  }
  Serial.print("Has EEPROM: "); Serial.println(cal.hasEEPROM());
  Serial.print("Has FLASH: "); Serial.println(cal.hasFLASH());

  if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found... will start with defaults");
  } else {
    Serial.println("Loaded existing calibration");
  }

  // in uTesla
  cal.mag_hardiron[0] = -10.00;
  cal.mag_hardiron[1] = 8.37;
  cal.mag_hardiron[2] = 3.20;

  // in uTesla
  cal.mag_softiron[0] = 0.994;
  cal.mag_softiron[1] = -0.069;
  cal.mag_softiron[2] = -0.005;  
  cal.mag_softiron[3] = -0.069;
  cal.mag_softiron[4] = 0.971;
  cal.mag_softiron[5] = 0.019;  
  cal.mag_softiron[6] = -0.005;
  cal.mag_softiron[7] = 0.019;
  cal.mag_softiron[8] = 1.041;
  // Earth total magnetic field strength in uTesla (dependent on location and time of the year),
  // visit: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm)
  cal.mag_field = 41.52; // approximate value for locations along the equator

  // in Radians/s
  cal.gyro_zerorate[0] = 0.023;
  cal.gyro_zerorate[1] = -0.031;
  cal.gyro_zerorate[2] = -0.078;

  // in G's
  cal.accel_zerog[0] = 0.00;
  cal.accel_zerog[1] = 0.00;
  cal.accel_zerog[2] = 0.00;

  if (! cal.saveCalibration()) {
    Serial.println("**WARNING** Couldn't save calibration");
  } else {
    Serial.println("Wrote calibration");    
  }

  cal.printSavedCalibration();
}

void loop() {

}
