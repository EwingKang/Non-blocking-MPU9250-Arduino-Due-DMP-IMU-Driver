/*****************************************************************************
* [I2C DMA]
* 		https://forum.arduino.cc/index.php?topic=605127.0
* [I2C + DMA]
* 		https://forum.arduino.cc/index.php?topic=152643.0
* [Interrupt misconception on Due (ATSAM3X)]
* 		https://forum.arduino.cc/index.php?topic=621506.0
* [Hanging I2C on DUE, SDA Low SCL High permanent]
* 		https://forum.arduino.cc/index.php?topic=288573.0
******************************************************************************/

#include <Arduino.h>
#include "include/pdc_mpu9250_dmp.h"

MPU9250_DMP imu;
void setup() {
	SerialUSB.begin(115200);
	Serial.begin(115200);
	Serial.println("=========Start 9250DMP with Ardu Due DMA test==========");
	delay(2000);
	
	if (imu.begin() != INV_SUCCESS)
	{
		while (1)
		{
		  Serial.println("Unable to communicate with MPU-9250");
		  Serial.println("Check connections, and try again.");
		  Serial.println();
		  delay(5000);
		}
	}

	imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

	// Use setGyroFSR() and setAccelFSR() to configure the
	// gyroscope and accelerometer full scale ranges.
	// Gyro options are +/- 250, 500, 1000, or 2000 dps
	imu.setGyroFSR(2000); // Set gyro to 2000 dps
	// Accel options are +/- 2, 4, 8, or 16 g
	imu.setAccelFSR(2); // Set accel to +/-2g
	// Note: the MPU-9250's magnetometer FSR is set at 
	// +/- 4912 uT (micro-tesla's)

	// setLPF() can be used to set the digital low-pass filter
	// of the accelerometer and gyroscope.
	// Can be any of the following: 188, 98, 42, 20, 10, 5
	// (values are in Hz).
	imu.setLPF(5); // Set LPF corner frequency to 5Hz

	// The sample rate of the accel/gyro can be set using
	// setSampleRate. Acceptable values range from 4Hz to 1kHz
	imu.setSampleRate(100); // Set sample rate to 10Hz

	// Likewise, the compass (magnetometer) sample rate can be
	// set using the setCompassSampleRate() function.
	// This value can range between: 1-100Hz
	imu.setCompassSampleRate(10); // Set mag rate to 10Hz
}

void loop() {
	// dataReady() checks to see if new accel/gyro data
	// is available. It will return a boolean true or false
	// (New magnetometer data cannot be checked, as the library
	//  runs that sensor in single-conversion mode.)
	if ( imu.dataReady() )
	{
		// Call update() to update the imu objects sensor data.
		// You can specify which sensors to update by combining
		// UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
		// UPDATE_TEMPERATURE.
		// (The update function defaults to accel, gyro, compass,
		//  so you don't have to specify these values.)
		imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
		printIMUData();
	}
}
void printIMUData(void)
{  
  // After calling update() the ax, ay, az, gx, gy, gz, mx,
  // my, mz, time, and/or temerature class variables are all
  // updated. Access them by placing the object. in front:

  // Use the calcAccel, calcGyro, and calcMag functions to
  // convert the raw sensor readings (signed 16-bit values)
  // to their respective units.
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx);
  float gyroY = imu.calcGyro(imu.gy);
  float gyroZ = imu.calcGyro(imu.gz);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);
  
  Serial.println("Accel: " + String(accelX) + ", " +
              String(accelY) + ", " + String(accelZ) + " g");
  Serial.println("Gyro: " + String(gyroX) + ", " +
              String(gyroY) + ", " + String(gyroZ) + " dps");
  Serial.println("Mag: " + String(magX) + ", " +
              String(magY) + ", " + String(magZ) + " uT");
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

