/******************************************************************************
pdc_mpu9250_dmp.cpp - MPU-9250 Digital Motion Processor Arduino Library, 
communicate and controlled with Arduino Due PDC(Peripheral DMA Controller). By  
utilizing PDC, the communication nolonger requires the loop to hold at the 
moment of transaction as the original Arduino Wire library did.

Supported Platforms:
- ATSAM Arduino Due
Author: Ewing Kang
- Contact: f039281310@yahoo.com.tw
-------------------------------------------------------------------------------
/******************************************************************************
SparkFunMPU9250-DMP.h - MPU-9250 Digital Motion Processor Arduino Library 
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This library implements motion processing functions of Invensense's MPU-9250.
It is based on their Emedded MotionDriver 6.12 library.
	https://www.invensense.com/developers/software-downloads/

Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
******************************************************************************/
#ifndef _PDC_MPU9250_DMP_H_
#define _PDC_MPU9250_DMP_H_

#include <Arduino.h>

// Optimally, these defines would be passed as compiler options, but Arduino
// doesn't give us a great way to do that.
#define MPU9250
#define AK8963_SECONDARY
#define COMPASS_ENABLED

// Include the Invensense MPU9250 driver and DMP keys:
extern "C" {
#include "utils/inv_mpu.h"
#include "utils/inv_mpu_dmp_motion_driver.h"
}

typedef int inv_error_t;
#define INV_SUCCESS 0
#define INV_ERROR 0x20
#define INV_PENDING 0x30
#define INV_TIMEOUT 0x40

enum t_axisOrder {
	X_AXIS, // 0
	Y_AXIS, // 1
	Z_AXIS  // 2
};

// Define's passed to update(), to request a specific sensor (or multiple):
#define UPDATE_ACCEL   (1<<1)
#define UPDATE_GYRO    (1<<2)
#define UPDATE_COMPASS (1<<3)
#define UPDATE_TEMP    (1<<4)

#define INT_ACTIVE_HIGH 0
#define INT_ACTIVE_LOW  1
#define INT_LATCHED     1
#define INT_50US_PULSE  0

#define MAX_DMP_SAMPLE_RATE 200 // Maximum sample rate for the DMP FIFO (200Hz)
#define FIFO_BUFFER_SIZE 512 	// Max FIFO buffer size

const signed char defaultOrientation[9] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};
#define ORIENT_PORTRAIT          0
#define ORIENT_LANDSCAPE         1
#define ORIENT_REVERSE_PORTRAIT  2
#define ORIENT_REVERSE_LANDSCAPE 3


class MPU9250_DMP 
{
public:
	int ax, ay, az;
	int gx, gy, gz;
	int mx, my, mz;
	long qw, qx, qy, qz;
	long temperature;
	unsigned long time;
	float pitch, roll, yaw;
	float heading;

	bool is_reading;
	unsigned long start_us;
	unsigned long timeout_us;  // Default to 1 ms in constructor
	
	MPU9250_DMP();
	
	/**************************************************************************
	* begin(void) -- Verifies communication with the MPU-9250 and the AK8963,
	* and initializes them to the default state:
	* All sensors enabled
	* Gyro FSR: +/- 2000 dps
	* Accel FSR: +/- 2g
	* LPF: 42 Hz
	* FIFO: 50 Hz, disabled
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t begin(void);
	
	/**************************************************************************
	* setSensors(unsigned char) -- Turn on or off MPU-9250 sensors. Any of the 
	* following defines can be combined: INV_XYZ_GYRO, INV_XYZ_ACCEL, 
	* INV_XYZ_COMPASS, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
	* Input: Combination of enabled sensors. Unless specified a sensor will be
	*  disabled.
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t setSensors(unsigned char sensors);
	
	/**************************************************************************
	*                  ---------- setGyroFSR() ----------
	* Sets the full-scale range of the gyroscope.
	* Input: Gyro DPS - 250, 500, 1000, or 2000
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                  ---------- getGyroFSR() ----------
	* Returns the current gyroscope FSR
	* Output: Current Gyro DPS - 250, 500, 1000, or 2000
	*                  ---------- getGyroSens() ----------
	* Returns current gyroscope sensitivity. 
	* The FSR divided by the resolution of the sensor (signed 16-bit).
	* Output: Currently set gyroscope sensitivity (e.g. 131, 65.5, 32.8, 16.4)
	**************************************************************************/
	inv_error_t setGyroFSR(unsigned short fsr);
	unsigned short getGyroFSR(void);
	float getGyroSens(void);
	
	/**************************************************************************
	*                   ---------- setAccelFSR() ----------
	* Sets the FSR of the accelerometer
	* Input: Accel g range - 2, 4, 8, or 16
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                   ---------- getAccelFSR() ----------
	* Returns the current accelerometer FSR
	* Output: Current Accel g - 2, 4, 8, or 16
	*                   ---------- getAccelSens() ----------
	* Returns current accelerometer sensitivity. 
	* The FSR divided by the resolution of the sensor (signed 16-bit).
	* Output: Currently set accel sensitivity (e.g. 16384, 8192, 4096, 2048)
	**************************************************************************/
	inv_error_t setAccelFSR(unsigned char fsr);
	unsigned char getAccelFSR(void);
	unsigned short getAccelSens(void);
	
	/**************************************************************************
	*                   ---------- getMagFSR() ----------
	* Returns the current magnetometer FSR
	* Output: Current mag uT range - +/-1450 uT
	*                   ---------- getMagSens() ----------
	* Returns current magnetometer sensitivity. 
	* The FSR divided by the resolution of the sensor (signed 16-bit).
	* Output: Currently set mag sensitivity (e.g. 0.15)
	**************************************************************************/
	unsigned short getMagFSR(void);
	float getMagSens(void);
	
	/**************************************************************************
	*                  ---------- setLPF() ----------
	* Sets the digital low-pass filter of the accel and gyro.
	* Can be any of the following: 188, 98, 42, 20, 10, 5 (value in Hz)
	* Input: 188, 98, 42, 20, 10, or 5 (defaults to 5 if incorrectly set)
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                  ---------- getLPF() ----------
	* Returns the set value of the LPF.
	* Output: 5, 10, 20, 42, 98, or 188 if set. 0 if the LPF is disabled.
	**************************************************************************/
	inv_error_t setLPF(unsigned short lpf);
	unsigned short getLPF(void);
	
	/**************************************************************************
	*                   ---------- setSampleRate() ----------
	* Set the gyroscope and accelerometer sample rate to a value between 
	* 4Hz and 1000Hz (1kHz).
	* The library will make an attempt to get as close as possible to the
	* requested sample rate.
	* Input: Value between 4 and 1000, indicating the desired sample rate
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                  ---------- getSampleRate() ----------
	* Get the currently set sample rate.
	* May differ slightly from what was set in setSampleRate.
	* Output: set sample rate of the accel/gyro. A value between 4-1000.
	**************************************************************************/
	inv_error_t setSampleRate(unsigned short rate);
	unsigned short getSampleRate(void);
	
	/**************************************************************************
	*               ---------- setCompassSampleRate() ----------
	* Set the magnetometer sample rate to a value between 1Hz and 100 Hz.
	* The library will make an attempt to get as close as possible to the
	* requested sample rate.
	* Input: Value between 1 and 100, indicating the desired sample rate
	* Output: INV_SUCCESS (0) on success, otherwise error
	*              ---------- getCompassSampleRate() ----------
	* Get the currently set magnetometer sample rate.
	* May differ slightly from what was set in setCompassSampleRate.
	* Output: set sample rate of the magnetometer. A value between 1-100
	**************************************************************************/
	inv_error_t setCompassSampleRate(unsigned short rate);
	unsigned short getCompassSampleRate(void);
	
	/**************************************************************************
	*                   ---------- dataReady() ----------
	* Checks to see if new accel/gyro data is available.
	* (New magnetometer data cannot be checked, as the library runs that sensor 
	*  in single-conversion mode.)
	* Output: true if new accel/gyro data is available
	**************************************************************************/
	bool dataReady();
	
	/**************************************************************************
	*                       ---------- update() ----------
	* Reads latest data from the MPU-9250's data registers.
	* Sensors to be updated can be set using the [sensors] parameter.
	* Input: [sensors] can be any combination of UPDATE_ACCEL, UPDATE_GYRO,
	*        UPDATE_COMPASS, and UPDATE_TEMP.
	* Output: INV_SUCCESS (0) on success, otherwise error
	* Note: after a successful update the public sensor variables 
	*       (e.g. ax, ay, az, gx, gy, gz) will be updated with new data 
	**************************************************************************/
	inv_error_t update(unsigned char sensors = 
	                   UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
	
	/**************************************************************************
	* --- updateAccel(), updateGyro(), updateCompass(), updateTemperature() ---
	* These four functions are called by the update() public method. They 
	* read from their respective sensor and update the class variable 
	* (e.g. ax, ay, az)
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t updateAccel(void);
	inv_error_t updateGyro(void);
	inv_error_t updateCompass(void);
	inv_error_t updateTemperature(void);
	inv_error_t updateAll(void);
	
	/**************************************************************************
	*                 --- Non-blocking update functions ---
	* It is recommanded to call updateAllUnblocked() with a micros() input,
	* since micros() function have a higher cost then millis() function. You 
	* may want to acquire your own timer at the start of each loop.
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t setNonblockTimeout(const unsigned long & timeout_us);
	inv_error_t updateAllUnblocked(const unsigned long & now_us);
	inv_error_t updateAllUnblocked(void);
	
	/**************************************************************************
	*                  ---------- configureFifo() ----------
	* Initialize the FIFO, set it to read from a select set of sensors.
	* Input: [sensors] sensors to be read into FIFO, any of the following 
	*        defines can be combined:
	*        INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                  ---------- getFifoConfig() ----------
	* Returns the sensors configured to be read into the FIFO
	* Output: combination of INV_XYZ_GYRO, INV_XYZ_ACCEL, INV_Y_GYRO,
	*         INV_X_GYRO, or INV_Z_GYRO
	*                 ---------- fifoAvailable() ----------
	* Returns the number of bytes currently filled in the FIFO
	* Outputs: Number of bytes filled in the FIFO (up to 512)
	*                   ---------- updateFifo() ----------
	* Reads from the top of the FIFO, and stores the new data in 
	* ax, ay, az, gx, gy, or gz (depending on how the FIFO is configured).
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                   ---------- resetFifo() ----------
	* Resets the FIFO's read/write pointers
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t configureFifo(unsigned char sensors);
	unsigned char getFifoConfig(void);
	unsigned short fifoAvailable(void);
	inv_error_t updateFifo(void);
	inv_error_t resetFifo(void);
	
	/**************************************************************************
	*               ---------- enableInterrupt() ---------- 
	* Configure the MPU-9250's interrupt output to indicate when new data is 
	* ready.
	* Input: 0 to disable, >=1 to enable
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                 ---------- setIntLevel() ----------
	* Configure the MPU-9250's interrupt to be either active-high or active-low.
	* Input: 0 for active-high, 1 for active-low
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                ---------- setIntLatched() ----------
	* Configure the MPU-9250's interrupt to latch or operate as a 50us pulse.
	* Input: 0 for 50us pulse, 1 for latched
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                ---------- getIntStatus() ----------
	* Reads the MPU-9250's INT_STATUS register, which can indicate what 
	* (if anything) caused an interrupt (e.g. FIFO overflow or data read).
	* Output: contents of the INT_STATUS register
	**************************************************************************/
	inv_error_t enableInterrupt(unsigned char enable = 1);
	inv_error_t setIntLevel(unsigned char active_low);
	inv_error_t setIntLatched(unsigned char enable);
	short getIntStatus(void);
	
	/**************************************************************************
	*                       ---------- dmpBegin ---------- 
	* Initialize the DMP, enable one or more features, and set the FIFO's 
	* sample rate features can be any one of:
	*  DMP_FEATURE_TAP -- Tap detection
	*  DMP_FEATURE_ANDROID_ORIENT -- Orientation (portrait/landscape) detection
	*  DMP_FEATURE_LP_QUAT -- Accelerometer, low-power quaternion calculation
	*  DMP_FEATURE_PEDOMETER -- Pedometer (always enabled)
	*  DMP_FEATURE_6X_LP_QUAT -- 6-axis (accel/gyro) quaternion calculation
	*  DMP_FEATURE_GYRO_CAL -- Gyroscope calibration (0's out after 8 seconds 
	*                          of no motion)
	*  DMP_FEATURE_SEND_RAW_ACCEL -- Send raw accelerometer values to FIFO
	*  DMP_FEATURE_SEND_RAW_GYRO -- Send raw gyroscope values to FIFO
	*  DMP_FEATURE_SEND_CAL_GYRO -- Send calibrated gyroscop values to FIFO
	* fifoRate can be anywhere between 4 and 200Hz.
	* Input: OR'd list of features and requested FIFO sampling rate
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t dmpBegin( unsigned short features = 0, 
	                      unsigned short fifoRate = MAX_DMP_SAMPLE_RATE);
	
	/**************************************************************************
	*                    ---------- dmpLoad() ---------- 
	* Loads the DMP with 3062-byte image memory. Must be called to begin DMP.
	* This function is called by the dmpBegin function.
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t dmpLoad(void);
	
	/**************************************************************************
	*                  ---------- dmpGetFifoRate() ---------- 
	* Returns the sample rate of the FIFO
	* Output: Set sample rate, in Hz, of the FIFO
	*                  ---------- dmpSetFiFoRate() ---------- 
	* Sets the rate of the FIFO.
	* Input: Requested sample rate in Hz (range: 4-200)
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	unsigned short dmpGetFifoRate(void);
	inv_error_t dmpSetFifoRate(unsigned short rate);
	
	/**************************************************************************
	*                ---------- dmpUpdateFifo() ---------- 
	* Reads from the top of the FIFO and fills accelerometer, gyroscope,
	* quaternion, and time public variables (depending on how the DMP is
	* configured).
	* Should be called whenever an MPU interrupt is detected.
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t dmpUpdateFifo(void); 
	
	/**************************************************************************
	*                 ---------- dmpEnableFeatures() ---------- 
	* Enable one, or multiple DMP features.
	* Input: An OR'd list of features (see dmpBegin)
	* Output: INV_SUCCESS (0) on success, otherwise error
	*               ---------- dmpGetEnabledFeatures() ---------- 
	* Returns the OR'd list of enabled DMP features
	* Output: OR'd list of DMP feature's (see dmpBegin for list)
	**************************************************************************/
	inv_error_t dmpEnableFeatures(unsigned short mask);
	unsigned short dmpGetEnabledFeatures(void);
	
	/**************************************************************************
	*                   ---------- dmpSetTap() ---------- 
	* Enable tap detection and configure threshold, tap time, and minimum tap 
	* count.
	* Inputs: x/y/zThresh - accelerometer threshold on each axis. 
	*           Range(mg/ms): 0 to 1600. 0 disables tap detection on that axis.
	*         taps - minimum number of taps to create a tap event (Range: 1-4)
	*         tapTime - Minimum number of milliseconds between separate taps
	*         tapMulti - Maximum number of milliseconds combined taps
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                 ---------- tapAvailable() ---------- 
	* Returns true if a new tap is available
	* Output: True if new tap data is available. Cleared on getTapDir or getTapCount.
	*                   ---------- getTapDir() ---------- 
	* Returns the tap direction.
	* Output: One of the following: TAP_X_UP, TAP_X_DOWN, TAP_Y_UP,
	*                               TAP_Y_DOWN, TAP_Z_UP, or TAP_Z_DOWN
	*                   ---------- getTapCount() ---------- 
	* Returns the number of taps in the sensed direction
	* Output: Value between 1-8 indicating successive number of taps sensed.
	**************************************************************************/
	inv_error_t dmpSetTap(unsigned short xThresh = 250, 
	                      unsigned short yThresh = 250, 
	                      unsigned short zThresh = 250,
						  unsigned char taps = 1, 
						  unsigned short tapTime = 100,
						  unsigned short tapMulti = 500);
	bool tapAvailable(void);
	unsigned char getTapDir(void);
	unsigned char getTapCount(void);

	/**************************************************************************
	*                 ---------- dmpSetOrientation() ---------- 
	* Set orientation matrix, used for orientation sensing.
	* Use defaultOrientation matrix as an example input.
	* Input: Gyro and accel orientation in body frame (9-byte array)
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                 ---------- dmpGetOrientation() ---------- 
	* Get the orientation, if any.
	* Output: If an orientation is detected, one of ORIENT_LANDSCAPE, 
	*         ORIENT_PORTRAIT, ORIENT_REVERSE_LANDSCAPE, 
	*         or ORIENT_REVERSE_PORTRAIT.
	**************************************************************************/
	inv_error_t dmpSetOrientation(const signed char * orientationMatrix = defaultOrientation);
	unsigned char dmpGetOrientation(void);
	
	/**************************************************************************
	*                  ---------- dmpEnable3Quat() ----------
	* Enable 3-axis quaternion calculation
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                  ---------- dmpEnable6Quat() ---------- 
	* Enable 6-axis quaternion calculation
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t dmpEnable3Quat(void);
	inv_error_t dmpEnable6Quat(void);
	
	/**************************************************************************
	*                ---------- dmpGetPedometerSteps() ----------
	* Get number of steps in pedometer register
	* Output: Number of steps sensed
	*                ---------- dmpSetPedometerSteps() ---------- 
	* Set number of steps to a value
	* Input: Desired number of steps to begin incrementing from
	* Output: INV_SUCCESS (0) on success, otherwise error
	*                ---------- dmpGetPedometerTime() ---------- 
	* Get number of milliseconds ellapsed over stepping
	* Output: Number of milliseconds where steps were detected
	*                ---------- dmpSetPedometerTime() ---------- 
	* Set number time to begin incrementing step time counter from
	* Input: Desired number of milliseconds
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	unsigned long dmpGetPedometerSteps(void);
	inv_error_t dmpSetPedometerSteps(unsigned long steps);
	unsigned long dmpGetPedometerTime(void);
	inv_error_t dmpSetPedometerTime(unsigned long time);
	
	/**************************************************************************
	*             ---------- dmpSetInterruptMode() ----------
	* Output: INV_SUCCESS (0) on success, otherwise error
	*              ---------- dmpSetGyroBias() ----------
	* Output: INV_SUCCESS (0) on success, otherwise error
	*             ---------- dmpSetAccelBias() ----------
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t dmpSetInterruptMode(unsigned char mode);
	inv_error_t dmpSetGyroBias(long * bias);
	inv_error_t dmpSetAccelBias(long * bias);
	
	/**************************************************************************
	*                 ---------- lowPowerAccel() ----------
	* Output: INV_SUCCESS (0) on success, otherwise error
	**************************************************************************/
	inv_error_t lowPowerAccel(unsigned short rate);

	/**************************************************************************
	*                 ---------- calcAccel() ---------- 
	* Convert 16-bit signed acceleration value to g's
	*                 ---------- calcGyro() ---------- 
	* Convert 16-bit signed gyroscope value to degree's per second
	*                  ---------- calcMag() ---------- 
	* Convert 16-bit signed magnetometer value to microtesla (uT)
	*                 ---------- calcQuat() ---------- 
	* Convert Q30-format quaternion to a vector between +/- 1
	*                 ---------- calcTemp() ---------- 
	* Convert Q16-format temperature to temperature member variable(deg-C) 
	**************************************************************************/
	float calcAccel(int axis);
	float calcGyro(int axis);
	float calcMag(int axis);
	float calcQuat(long axis);
	float calcTemp(void);
	
	/**************************************************************************
	*              ---------- computeEulerAngles() ---------- 
	* Compute euler angles based on most recently read qw, qx, qy, and qz
	* Input: boolean indicating whether angle results are presented in degrees 
	*        or radians
	* Output: class variables roll, pitch, and yaw will be updated on exit.	
	**************************************************************************/
	void computeEulerAngles(bool degrees = true);
	
	/**************************************************************************
	*               ----------  computeCompassHeading() ---------- 
	* Compute heading based on most recently read mx, my, and mz values
	* Output: class variable heading will be updated on exit
	**************************************************************************/
	float computeCompassHeading(void);
	
	/**************************************************************************
	*                     ---------- selfTest() ---------- 
	* Run gyro and accel self-test.
	* Output: Returns bit mask, 1 indicates success. A 0x7 is success on all sensors.
	*         Bit pos 0: gyro
	*         Bit pos 1: accel
	*         Bit pos 2: mag
	**************************************************************************/
	int selfTest(unsigned char debug = 0);
	
private:
	unsigned short _aSense;
	float _gSense, _mSense;
	
	// Convert a QN-format number to a float
	float qToFloat(long number, unsigned char q);
	unsigned short orientation_row_2_scale(const signed char *row);
};   //class MPU9250_DMP 


#endif // _SPARKFUN_MPU9250_DMP_H_