/******************************************************************************
arduino_due_pdc_i2c.h
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
#ifndef _ARDUINO_MPU9250_I2C_H_
#define _ARDUINO_MPU9250_I2C_H_

I2cBus i2c_bus;

#if defined(__cplusplus) 
extern "C" {
#endif

int arduino_pdci2c_blocked_write(unsigned char slave_addr, unsigned char reg_addr,
							  unsigned char length, unsigned char * data);
int arduino_pdci2c_blocked_read(unsigned char slave_addr, unsigned char reg_addr,
							 unsigned char length, unsigned char * data);

#if defined(__cplusplus) 
}
#endif


void TWI1_Handler() {
	i2c_bus.IsrHandler();
}

#endif // _ARDUINO_MPU9250_I2C_H_