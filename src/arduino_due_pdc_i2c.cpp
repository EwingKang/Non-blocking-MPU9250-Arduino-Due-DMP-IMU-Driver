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
#include "../include/arduino_due_pdc_i2c.h"
#include "../include/i2c_bus.hpp"
//#include <Arduino.h>
//#include <Wire.h>
//I2cBus i2c_bus;  EWING do this?
//int accelerometer, gyro_id;

int arduino_pdci2c_blocked_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	/*
	Wire.beginTransmission(slave_addr);
	Wire.write(reg_addr);
	for (unsigned char i = 0; i < length; i++)
	{
		Wire.write(data[i]);
	}
	Wire.endTransmission(true);
	*/
	i2c_bus.SetRegBlocked
	return 0;
}

int arduino_pdci2c_blocked_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	Wire.beginTransmission(slave_addr);
	Wire.write(reg_addr);
	Wire.endTransmission(false);
	Wire.requestFrom(slave_addr, length);
	for (unsigned char i = 0; i < length; i++)
	{
		data[i] = Wire.read();
	}
	
	return 0;
}
