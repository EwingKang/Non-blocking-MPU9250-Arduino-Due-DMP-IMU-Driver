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
#include "include/i2c_bus.hpp"

//#define DeviceID                0x34
//#define DeviceAddress           0x68		//MPU6050 
#define MPU6050_RA_WHO_AM_I     0x75
#define MPU6050_WHO_AM_I_BIT    6
#define MPU6050_WHO_AM_I_LENGTH 6

#define ADXL234         0x53		//ADXL345
#define HMC5883 		0x1E         //gyro
// ADXL234 register map
#define ADXL345_WHO_AM_I    	 0x00
#define ADXL345_FIFO_STATUS     0x39
#define ADXL345_PWR_CTL_RA		0x2D
#define ADXL345_DATAX0  		(0x32)         //X-Axis Data 0
#define DATA_LEN 6			// 6-byte

I2cBus i2c_bus;
int accelerometer, gyro_id;

void setup() {
	SerialUSB.begin(115200);
	Serial.begin(115200);
	Serial.println("=========Start Dma test==========");
	i2c_bus.Begin();
	delay(2000);
		
	
	//============ test1: single write read ================
	Serial.println("Test1 Start");
	accelerometer = i2c_bus.Add_Device(ADXL234);
	delay(200); 							// for serial to write
	
	uint8_t rtn = 3;
	int res;
	
	res = i2c_bus.GetReg(accelerometer, ADXL345_WHO_AM_I, 1);
	Serial.print(res);
	do
	{
		res = i2c_bus.UpdateGetReg();
		//Serial.println(res);
	}while(res<4);
	
	Serial.print("fetch: ");
	Serial.print( i2c_bus.FetchRegData(&rtn, 1) );
	Serial.print("Test1 rtn: ");
	Serial.println(rtn, HEX);
	Serial.println("---------------------");
	delay(1000);
	
	
	//============ test2: multi write ================
	Serial.println("Test2 Start");
	Serial.println("Enable ADXL234 measurement (multi-write)");
	delay(200); // for serial to write
	
	
	//Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
	i2c_bus.SetRegBlocked(accelerometer, 0x31, 0x01, 2);	// 2ms block

	static uint8_t rtn1[10];	
	i2c_bus.SetReg(accelerometer, ADXL345_PWR_CTL_RA, 0x08); // register address , EN_MEAS
	while( !i2c_bus.SetRegIsFinished() );		// block when transmitting
	Serial.println("ADXL234 Started");
	Serial.println("---------------------");
	delay(2000); // for ADXL345 to start (enable measurement)
	
	
	//============ test3: multi read ================
	Serial.println("Test3 Start");
	Serial.println("Multi read");
	delay(200); // for serial to write
	
	res = i2c_bus.GetReg(accelerometer, ADXL345_DATAX0, 6);
	Serial.print(res);
	do{
		res = i2c_bus.UpdateGetReg();
		//Serial.println(res);
	}while(res<4);
	
	uint8_t rtn2[10];
	Serial.print("fetch: ");
	Serial.print( i2c_bus.FetchRegData(rtn2, 6) );
	
	for(int k=0;k<6;k++) {
		Serial.print(rtn2[k], HEX);
		Serial.print(", ");
	}
	Serial.println("");
	
	int16_t ax = (int16_t)rtn2[0] + ((int16_t)rtn2[1] << 8);
	int16_t ay = (int16_t)rtn2[2] + ((int16_t)rtn2[3] << 8);
	int16_t az = (int16_t)rtn2[4] + ((int16_t)rtn2[5] << 8);
	String accel_res = " accel read: [";
	accel_res += String( (float)ax*8/1024 ) + ", ";
	accel_res += String( (float)ay*8/1024 ) + ", ";
	accel_res += String( (float)az*8/1024 ) + "]";
	Serial.println(accel_res.c_str());
	
	Serial.println("---------------------");
	Serial.println("========= End Dma test ==========");
}

int i;
void loop() {
	i++;
	delay(1000);
	
	// read accel data
	uint8_t rtn3[6];
	if( i2c_bus.GetRegBlocked(accelerometer, ADXL345_DATAX0, 6, rtn3, 2) == 0)
	{
		Serial.print("i: ");
		
		int16_t ax = (int16_t)rtn3[0] + ((int16_t)rtn3[1] << 8);
		int16_t ay = (int16_t)rtn3[2] + ((int16_t)rtn3[3] << 8);
		int16_t az = (int16_t)rtn3[4] + ((int16_t)rtn3[5] << 8);
		Serial.print(i);
		Serial.print(" accel read: [");
		Serial.print((float)ax*8/1024);
		Serial.print(", ");
		Serial.print((float)ay*8/1024);
		Serial.print(", ");
		Serial.print((float)az*8/1024);
		Serial.println("] ");
	}else
	{
		Serial.println("something failed");
	}
}

void TWI1_Handler() {
	i2c_bus.IsrHandler();
}
