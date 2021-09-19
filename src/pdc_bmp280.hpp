/*!
 *  @file BMP280.h
 *
 *  This is a library for the Adafruit BMP280 Breakout.
 *
 *  Designed specifically to work with the Adafruit BMP280 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2651
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  BSD license, all text above must be included in any redistribution
 */
#ifndef __BMP280_H__
#define __BMP280_H__

#include "Arduino.h"
#include "i2c_bus.hpp"

/************************************************
 *  I2C ADDRESS/BITS/SETTINGS
 ************************************************/
#define BMP280_ADDRESS (0x76) 		// The default I2C address for the sensor.
#define BMP280_ADDRESS_ALT (0x77)	// Alternative I2C address
#define BMP280_CHIPID (0x58) // Default chip ID.

//  Forward declarations of I2C bus object
extern I2cBus i2c_bus;

/************************************************!
 * Registers available on the sensor.
 ************************************************/
enum {
	BMP280_REGISTER_DIG_T1 = 0x88,
	BMP280_REGISTER_DIG_T2 = 0x8A,
	BMP280_REGISTER_DIG_T3 = 0x8C,
	BMP280_REGISTER_DIG_P1 = 0x8E,
	BMP280_REGISTER_DIG_P2 = 0x90,
	BMP280_REGISTER_DIG_P3 = 0x92,
	BMP280_REGISTER_DIG_P4 = 0x94,
	BMP280_REGISTER_DIG_P5 = 0x96,
	BMP280_REGISTER_DIG_P6 = 0x98,
	BMP280_REGISTER_DIG_P7 = 0x9A,
	BMP280_REGISTER_DIG_P8 = 0x9C,
	BMP280_REGISTER_DIG_P9 = 0x9E,
	BMP280_REGISTER_CHIPID = 0xD0,
	BMP280_REGISTER_VERSION = 0xD1,
	BMP280_REGISTER_SOFTRESET = 0xE0,
	BMP280_REGISTER_CAL26 = 0xE1, /**< R calibration = 0xE1-0xF0 */
	BMP280_REGISTER_STATUS = 0xF3,
	BMP280_REGISTER_CONTROL = 0xF4,
	BMP280_REGISTER_CONFIG = 0xF5,
	BMP280_REGISTER_PRESSUREDATA = 0xF7,
	BMP280_REGISTER_TEMPDATA = 0xFA,
};

/************************************************
 *  Struct to hold calibration data.
 ************************************************/
typedef struct {
	int32_t dig_T1; // dig_T1 cal register.
	int32_t dig_T2;  // dig_T2 cal register.
	int32_t dig_T3;  // dig_T3 cal register.

	int64_t dig_P1; // dig_P1 cal register.
	int64_t dig_P2;  // dig_P2 cal register.
	int64_t dig_P3;  // dig_P3 cal register.
	int64_t dig_P4;  // dig_P4 cal register.
	int64_t dig_P5;  // dig_P5 cal register.
	int64_t dig_P6;  // dig_P6 cal register.
	int64_t dig_P7;  // dig_P7 cal register.
	int64_t dig_P8;  // dig_P8 cal register.
	int64_t dig_P9;  // dig_P9 cal register.
} Bmp280CalibData;

/************************************************
 * Driver for the Adafruit BMP280 barometric pressure sensor.
 ************************************************/
class BMP280 {
public:
	/** Oversampling rate for the sensor. */
	enum sensor_sampling {
		SAMPLING_NONE = 0x00,	// No over-sampling
		SAMPLING_X1 = 0x01,		// 1x over-sampling
		SAMPLING_X2 = 0x02,		// 2x over-sampling
		SAMPLING_X4 = 0x03,		// 4x over-sampling
		SAMPLING_X8 = 0x04,		// 8x over-sampling
		SAMPLING_X16 = 0x05		// 16x over-sampling
	};

	/** Operating mode for the sensor. */
	enum sensor_mode {
		MODE_SLEEP = 0x00,			// Sleep mode
		MODE_FORCED = 0x01,			// Forced mode
		MODE_NORMAL = 0x03,			// Normal mode
		MODE_SOFT_RESET_CODE = 0xB6	// Software reset
	};

	/** Filtering level for sensor data. */
	enum sensor_filter {
		FILTER_OFF = 0x00,	// No filtering
		FILTER_X2 = 0x01,	// 2x filtering
		FILTER_X4 = 0x02,	// 4x filtering
		FILTER_X8 = 0x03,	// 8x filtering
		FILTER_X16 = 0x04	// 16x filtering
	};

	/** Standby duration in ms */
	enum standby_duration {
		STANDBY_MS_1 = 0x00,		// 1 ms
		STANDBY_MS_63 = 0x01,		// 62.5 ms
		STANDBY_MS_125 = 0x02,		// 125 ms
		STANDBY_MS_250 = 0x03,		// 250 ms
		STANDBY_MS_500 = 0x04,		// 500 ms
		STANDBY_MS_1000 = 0x05,		// 1000 ms
		STANDBY_MS_2000 = 0x06,		// 2000 ms
		STANDBY_MS_4000 = 0x07		// 4000 ms
	};

	BMP280(void);
	~BMP280(void);

	bool begin(uint8_t addr = BMP280_ADDRESS, 
			   uint8_t chipid = BMP280_CHIPID);
	void reset(void);
	uint8_t getStatus(void);

	float readTemperature();
	float readPressure(void);
	float readAltitude(float seaLevelhPa = 1013.25);
	float seaLevelForAltitude(float altitude, float atmospheric);
	float waterBoilingPoint(float pressure);
	void SetSeaLevelPressure(float seaLevelhPa);
	void GetSeaLevelPressure(float *seaLevelhPa);
	int UpdateAllNonBlocked(void);
	void GetData(float * pres, float * temp, float * alt);

	// void takeForcedMeasurement();
	void setSampling(sensor_mode mode = MODE_NORMAL,
				   sensor_sampling tempSampling = SAMPLING_X16,
				   sensor_sampling pressSampling = SAMPLING_X16,
				   sensor_filter filter = FILTER_OFF,
				   standby_duration duration = STANDBY_MS_1);

private:

	/** Encapsulates the config register */
	struct config {
		/** Inactive duration (standby time) in normal mode */
		unsigned int t_sb : 3;
		/** Filter settings */
		unsigned int filter : 3;
		/** Unused - don't set */
		unsigned int none : 1;
		/** Enables 3-wire SPI */
		unsigned int spi3w_en : 1;	// TODO: enable spi?!
		/** Used to retrieve the assembled config register's byte value. */
		unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
	};

	/** Encapsulates trhe ctrl_meas register */
	struct ctrl_meas {
		/** Temperature oversampling. */
		unsigned int osrs_t : 3;
		/** Pressure oversampling. */
		unsigned int osrs_p : 3;
		/** Device mode */
		unsigned int mode : 2;
		/** Used to retrieve the assembled ctrl_meas register's byte value. */
		unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
	};

	void readCoefficients(void);

	void write8(byte reg, byte value);
	uint8_t read8(byte reg);
	uint16_t read16(byte reg);
	uint32_t read24(byte reg);
	int16_t readS16(byte reg);
	uint16_t read16_LE(byte reg);
	int16_t readS16_LE(byte reg);

	uint8_t _i2caddr;
	int32_t _sensorID;
	
	bool _is_reading;
	unsigned long _start_us;
	
	
	int32_t _t_fine;
	
	float _temp;    // degress celcius
	float _pres;	// Pa (N/m^2)
	float _alt;		// meters
	float _qnh;		// hPa (100*N/m^2)
	
	Bmp280CalibData _bmp280_calib;
	config _configReg;
	ctrl_meas _measReg;
	
	
};

#endif
