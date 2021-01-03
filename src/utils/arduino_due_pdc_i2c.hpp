/******************************************************************************
arduino_due_pdc_i2c.h
HAL of MPU-9250 Digital Motion Processor Arduino Library for Arduino Due using
PDC I2C communication

Ewing Kang
2020.1.5

Original source:
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

#if defined(__cplusplus) 
extern "C" {
#endif

int arduino_pdci2c_blocked_write(unsigned char slave_addr, 
								 unsigned char reg_addr,
								 unsigned char length, 
								 unsigned char * data       );
								 
int arduino_pdci2c_blocked_read(unsigned char slave_addr, 
								unsigned char reg_addr,
								unsigned char length, 
								unsigned char * data     );
								
int arduino_pdci2c_ask(unsigned char slave_addr, 
					   unsigned char reg_addr,
					   unsigned char length      );
					   
int arduino_pdci2c_hear(unsigned char length, char * data);

#if defined(__cplusplus) 
}
#endif


#endif // _ARDUINO_MPU9250_I2C_H_