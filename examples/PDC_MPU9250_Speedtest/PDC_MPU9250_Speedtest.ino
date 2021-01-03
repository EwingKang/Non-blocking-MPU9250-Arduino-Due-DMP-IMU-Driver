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
#include <pdc_mpu9250_dmp.hpp>
#include <pdc_bmp280.hpp>

//#define BLOCKED_LOOP; // Comment out this line for realtime version

MPU9250_DMP imu;
BMP280 barometer;
void setup() {
	SerialUSB.begin(115200);
	Serial.begin(115200);
	Serial.println("=========Start 9250DMP with Ardu Due DMA test==========");
	delay(2000);
	
	pinMode(22,INPUT);
	
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

	Serial.println("Set IMU");
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
	
	delay(100);
	Serial.println("Set barometer");
	if(	!barometer.begin()) { Serial.println("FAILED!"); }
}

int i=0;
unsigned long now_us, last_us = 0, imu_us = 0, baro_us = 0;
unsigned long imu_sum_us=0, baro_sum_us = 0;
void loop() {
	now_us = micros();

#ifdef BLOCKED_LOOP
	// dataReady() checks to see if new accel/gyro data
	// is available. It will return a boolean true or false
	// (New magnetometer data cannot be checked, as the library
	//  runs that sensor in single-conversion mode.)
	Serial.print("Loop: ");
	Serial.println(i);
	i++;
	
	if ( imu.dataReady() )
	{
		// Call update() to update the imu objects sensor data.
		// You can specify which sensors to update by combining
		// UPDATE_ACCEL, UPDATE_GYRO, UPDATE_COMPASS, and/or
		// UPDATE_TEMPERATURE.
		// (The update function defaults to accel, gyro, compass,
		//  so you don't have to specify these values.)
		unsigned long t1, t2;
		int update_res;
		t1 = micros();
		//imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
		update_res = imu.updateAll();
		t2 = micros();
		
		Serial.print("Update() duration: ");
		Serial.print( ((float)t2-t1)/1000.0 );
		Serial.print(" ms\n");
		if(update_res == INV_SUCCESS)
		{
			printIMUData();
		}
		else
		{
			Serial.println("Update error");
		}
	}
	delay(750);
#else
	if(now_us - last_us > 100)
	{
		// Super fast loop here @10,000 Hz
		
		// 1 Hz since last success
		if(now_us - imu_us > 1000000)
		{
			unsigned long t1, t2;
			t1 = micros();
			int update_res = imu.updateAllUnblocked();
			t2 = micros();
			if(update_res == INV_PENDING)
			{
				//Serial.print("Ipend: " + String(t2-t1) + "us\n");
				imu_sum_us += t2-t1;
			}else if(update_res == INV_SUCCESS)
			{
				imu_sum_us += t2-t1;
				Serial.print("IMU Success: " + String(t2-t1) + 
							 "us, total: " + String(imu_sum_us) + "us\n");
				printIMUData();
				imu_sum_us = 0;
				imu_us = now_us;
			}else
			{
				Serial.println("Update error");
				imu_us = now_us;
			}
		}
		
		// 1 Hz since last success
		if(now_us+500000 - baro_us > 1000000)
		{
			unsigned long t1, t2;
			t1 = micros();
			int update_res = barometer.UpdateAllNonBlocked();
			t2 = micros();
			if(update_res == 0)
			{
				Serial.print("Spent: " + String(t2-t1) + "us\n");
				baro_sum_us += t2-t1;
			}else if(update_res >= 1 )
			{
				baro_sum_us += t2-t1;
				Serial.print("Baro success: " + String(t2-t1) + 
							 "us, total: " + String(baro_sum_us) + "us\n");
				float p, t, alt;
				barometer.GetData( &p, &t, &alt);
				Serial.println("P (pa):   " + String(p) +
							 ", T (degC):   " + String(t) +
							 ", A (m):   " + String(alt) );
				Serial.println();
				baro_sum_us = 0;
				baro_us = now_us+500000;
			}else
			{
				Serial.println("Baro update error");
				baro_us = now_us+500000;
			}
		}
		
		last_us = now_us;
	}
#endif
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
	float temp = imu.calcTemp();

	Serial.println("Accel: " + String(accelX) + ", " +
			  String(accelY) + ", " + String(accelZ) + " g");
	Serial.println("Gyro: " + String(gyroX) + ", " +
			  String(gyroY) + ", " + String(gyroZ) + " dps");
	Serial.println("Mag: " + String(magX) + ", " +
			  String(magY) + ", " + String(magZ) + " uT");
	Serial.println("Temp: " + String(temp));
	//Serial.println("Time: " + String(imu.time) + " ms");
	Serial.println();
}
