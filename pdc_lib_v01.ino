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
#include "pdc_twi.hpp"

//#define DeviceID                0x34
//#define DeviceAddress           0x68		//MPU6050 
#define MPU6050_RA_WHO_AM_I     0x75
#define MPU6050_WHO_AM_I_BIT    6
#define MPU6050_WHO_AM_I_LENGTH 6

#define ADXL234         0x53		//ADXL345
#define HMC5883 		0x1E         //gyro
// ADXL234 register map
#define ADXL345_WHO_AM_I     0x00
#define ADXL345_FIFO_STATUS     0x39
#define ADXL345_PWR_CTL_RA		0x2D
#define ADXL345_DATAX0  		(0x32)         //X-Axis Data 0
#define DATA_LEN 6			// 6-byte

PdcTwi i2c;
volatile PdcTwi::PdcTwoWireStatus PdcTwi::master_state = PdcTwoWireStatus::PDC_UNINIT;

void setup() {
	SerialUSB.begin(115200);
	Serial.begin(115200);
	Serial.println("=========Start Dma test==========");
	i2c.Init();
	delay(2000);
	
	
	
	//============ test1: single write read ================
	Serial.print("Test1 Start");
	Serial.print(", initial TWI_SR: ");
	Serial.println(WIRE_INTERFACE->TWI_SR, BIN);
	delay(200); 							// for serial to write
	
	static uint8_t wai_reg = ADXL345_WHO_AM_I;
	static uint8_t rtn = 0;

	i2c.WriteTo(ADXL234, &wai_reg, 1);
	while( !i2c.TxComplete() );				// blocked while transfering	
	i2c.ReadFrom(ADXL234, &rtn, 1);
	while( !i2c.RxComplete() );
	
	Serial.print("Test1 rtn: ");
	Serial.println(i2c.rxdata, HEX);
	Serial.println("---------------------");
	delay(2000);
	
	/*//============ test2: reset all ================
	Serial.println("Test2 Start");
	Serial.println("Reset all");
	i2c.Reset();
	Serial.println("Reset done");*/
	
	//============ test2: multi write ================
	Serial.println("Test2 Start");
	Serial.println("Enable ADXL234 measurement (multi-write)");
	delay(200); // for serial to write
	static uint8_t rtn1[10];
	static uint8_t en_meas_pkt[3]= {ADXL345_PWR_CTL_RA, 0x08}; // register address , EN_MEAS
	
	i2c.WriteTo(ADXL234, en_meas_pkt, 2);
	while( !i2c.TxComplete() );				// blocked while transfering	
	Serial.println("ADXL234 Started");
	Serial.println("---------------------");
	delay(2000); // for ADXL345 to start (enable measurement)
	
	
	//============ test3: multi read ================
	Serial.println("Test3 Start");
	Serial.println("Multi read (single-write, multi-read)");
	delay(200); // for serial to write
	
	
	// reading action: [write desired address] -> [read]
	static uint8_t data0_addr = ADXL345_DATAX0;
	
	i2c.WriteTo(ADXL234, &data0_addr, 1);
	while( !i2c.TxComplete() );			// blocked while transfering
	Serial.println("read");
	i2c.ReadFrom(ADXL234, rtn1, 4);
	long us1 = micros();
	while( !i2c.RxComplete() );			// blocked while recieving
	long us2 = micros();	
	Serial.println("Test3 rtn: ");
	for(int k=0;k<10;k++) {
		Serial.print(rtn1[k], HEX);
		Serial.print(", ");
	}
	Serial.println("");
	Serial.print("End Test2");
	Serial.print(", TWI_SR: ");
	Serial.println(WIRE_INTERFACE->TWI_SR, BIN);
	
	Serial.print("t1: ");
	Serial.print(us1);
	Serial.print(", t2:");
	Serial.println(us2);
	/*long us3 = micros();
	Serial.print(", t3:");
	Serial.println(us3);*/	
	Serial.println("---------------------");
	delay(2000);
	
	//Serial.println("========= End Dma test ==========");
}

int i;
uint8_t rtn2[10];
uint8_t data0_addr = ADXL345_DATAX0;
void loop() {
	i++;
	delay(1000);
	
	// read accel data
	i2c.WriteTo(ADXL234, &data0_addr, 1);
	while( !i2c.TxComplete() );			// blocked while transfering
	i2c.ReadFrom(ADXL234, rtn2, 6);
	while( !i2c.RxComplete() );			// blocked while recieving
	int16_t ax = (int16_t)rtn2[0] + ((int16_t)rtn2[1] << 8);
	int16_t ay = (int16_t)rtn2[2] + ((int16_t)rtn2[3] << 8);
	int16_t az = (int16_t)rtn2[4] + ((int16_t)rtn2[5] << 8);
	Serial.print(i);
	Serial.print(" accel read: [");
	Serial.print((float)ax*8/1024);
	Serial.print(", ");
	Serial.print((float)ay*8/1024);
	Serial.print(", ");
	Serial.print((float)az*8/1024);
	Serial.println("] ");

}

void TWI1_Handler() {
	i2c.IsrHandler();
}
