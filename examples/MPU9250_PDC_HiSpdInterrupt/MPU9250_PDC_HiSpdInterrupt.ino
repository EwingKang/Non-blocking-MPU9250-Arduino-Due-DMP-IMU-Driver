/*****************************************************************************
* High speed interrupt based communication with MPU 9250, using Arduino Due 
* PDC functionality.
* TODO
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

#define IMU_INT_PIN 19

MPU9250_DMP imu;
BMP280 barometer;

// Prototype
void ImuIntRising();

void setup() {
	SerialUSB.begin(115200);
	Serial.begin(115200);
	Serial.println("========= Start 9250-DMP with Interrupt ==========");
	delay(2000);
	
	pinMode(IMU_INT_PIN, INPUT);
	
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
	imu.setNonblockTimeout(1100);
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
	imu.setLPF(188); // Set LPF corner frequency to 5Hz

	// The sample rate of the accel/gyro can be set using
	// setSampleRate. Acceptable values range from 4Hz to 1kHz
	imu.setSampleRate(1000); // Set sample rate to 10Hz
	Serial.println( "Sampling rate is " + String(imu.getSampleRate()) );

	// Likewise, the compass (magnetometer) sample rate can be
	// set using the setCompassSampleRate() function.
	// This value can range between: 1-100Hz
	imu.setCompassSampleRate(100); // Set mag rate to 10Hz
	
	
	inv_error_t imu_set_res;
	// Input: 0 for active-high, 1 for active-low
	imu_set_res = imu.setIntLevel(0);
	// Configure the MPU-9250's interrupt to latch or operate as a 50us pulse.
	// Input: 0 for 50us pulse, 1 for latched
	imu_set_res = imu.setIntLatched(0); // TODO changed
	imu_set_res = imu.enableInterrupt();
	
	// Reads the MPU-9250's INT_STATUS register, which can indicate what 
	// (if anything) caused an interrupt (e.g. FIFO overflow or data read).
	// Output: contents of the INT_STATUS register
	short int_status = imu.getIntStatus();
	Serial.println( "Interrupt status: " + String(int_status) );
	
	attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), ImuIntRising, RISING);

	
	//delay(100);
	//Serial.println("Set barometer");
	//if(	!barometer.begin()) { Serial.println("FAILED!"); }
}

enum class ImuStateMachine
{
	ISM_IDLE,
	ISM_DATARDY,
	ISM_READING,
};
volatile ImuStateMachine imu_stmh;
volatile bool itrp_when_reading = false;
volatile unsigned long itrp_imu_us = 0;
volatile int tick = 0;

int imu_data_cnt=0;
int imu_err_cnt = 0, last_print_imu_err_cnt = 0;
unsigned long now_us, last_us = 0, imu_us = 0, first_itrp_imu_us = 0, baro_us = 0;
unsigned long imu_sum_us = 0, baro_sum_us = 0;
void loop() {
	int update_res;
	bool new_imu_data = false;
	if(imu_stmh == ImuStateMachine::ISM_DATARDY )
	{
		now_us = micros();
		update_res = imu.updateAllUnblocked(now_us);
		imu_stmh = ImuStateMachine::ISM_READING;
	}
	else if (imu_stmh == ImuStateMachine::ISM_READING )
	{
		now_us = micros();
		update_res = imu.updateAllUnblocked(now_us);
		if(update_res == INV_PENDING)
		{
			//keep waiting, do nothing
		}else if(update_res == INV_SUCCESS)
		{
			imu_us = micros();
			imu_stmh = ImuStateMachine::ISM_IDLE;
			itrp_when_reading = false;
			new_imu_data = true;
		}else if(update_res == INV_TIMEOUT)
		{
			imu_err_cnt++;
			imu_stmh = ImuStateMachine::ISM_IDLE;	
		} else
		{
			// ERROR, probably due to timeout
			imu_stmh = ImuStateMachine::ISM_IDLE;			
		}
	}
	
	if(new_imu_data)
	{
		imu_data_cnt++;
		if(imu_data_cnt % 50 == 0)
		{
			int diff = itrp_imu_us - first_itrp_imu_us;
			Serial.println( "Start: "+ String(first_itrp_imu_us) + 
						    ", ends: " + String(itrp_imu_us) +
						    ", diff: " + String(diff) + 
						    ", avg: " + String((float)diff/50) );
			first_itrp_imu_us = itrp_imu_us;
		}
	}
	if( (imu_err_cnt-1) % 1000 == 0 &&
		imu_err_cnt != last_print_imu_err_cnt)
	{
		Serial.println("Err: " + String(imu_err_cnt) );
		last_print_imu_err_cnt = imu_err_cnt;
	}
	
	/*
	if(now_us - last_us > 100)
	{
		// Super fast loop here @10,000 Hz
		
		// 1 Hz since last success
		if(now_us - imu_us > 1000000)
		{
			unsigned long t1, t2;
			t1 = micros();
			int 
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
		/
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
	}*/
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

void ImuIntRising()
{
	itrp_imu_us = micros();
	if(imu_stmh == ImuStateMachine::ISM_READING) {
		itrp_when_reading = true;
		return; // Do not set data ready
	}
	imu_stmh = ImuStateMachine::ISM_DATARDY;
	tick++;
}