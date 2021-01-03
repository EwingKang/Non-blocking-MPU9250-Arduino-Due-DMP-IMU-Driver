/******************************************************************************
arduino_due_pdc_i2c.cpp 
HAL of MPU-9250 Digital Motion Processor Arduino Library for Arduino Due using
PDC I2C communication

Ewing Kang
2020.1.5

Originaed:
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library
This library implements motion processing functions of Invensense's MPU-9250.
It is based on their Emedded MotionDriver 6.12 library.
	https://www.invensense.com/developers/software-downloads/

Development environment specifics:
Arduino IDE 
IMU 9250 x.x.x

Supported Platforms:
- xxxxx (Arduino Due, ???? IMU)
******************************************************************************/
#include "arduino_due_pdc_i2c.hpp"
#include <i2c_bus.hpp>

I2cBus i2c_bus;		// actual instance in pdc_mpu9250_dmp.cpp

int arduino_pdci2c_blocked_write(unsigned char slave_addr, unsigned char reg_addr,
								 unsigned char length, unsigned char * data)
{
	//Serial.print("Blked write ");
	//Serial.println(length);
	i2c_bus.WriteRegBlocked( slave_addr, reg_addr, data, length, -1 );
	
	return 0;
}

int arduino_pdci2c_blocked_read(unsigned char slave_addr, unsigned char reg_addr,
								unsigned char length, unsigned char * data)
{
	//Serial.print("Blked read ");
	//Serial.print(length);
	int res = i2c_bus.ReadRegBlocked(slave_addr, reg_addr, length, data, 100);
	if(res != 0)
	{
		//Serial.print(" failed: ");
		//Serial.println(res);
		return -1;
	}else
	{
		//Serial.println(" Done!");
		return 0;
	}
}

int arduino_pdci2c_ask(unsigned char slave_addr, unsigned char reg_addr,
								unsigned char length)
{
	//Serial.print("Ask: ");
	//Serial.print(length);
	int res = i2c_bus.ReadReg(slave_addr, reg_addr, length);
	if(res != 0)
	{
		//Serial.print(" failed: ");
		//Serial.println(res);
		return -1;
	}else
	{
		//Serial.println(" Done!");
		return 0;
	}
}


int arduino_pdci2c_hear(unsigned char length, char * data)
{
	//Serial.print("Hear: ");
	//Serial.print(length);
	int res = i2c_bus.FetchRegReadData((unsigned char *)data, length);
	if(res != 0)
	{
		//Serial.print(" failed: ");
		//Serial.println(res);
		return -1;
	}else
	{
		//Serial.println(" Done!");
		return 0;
	}
}