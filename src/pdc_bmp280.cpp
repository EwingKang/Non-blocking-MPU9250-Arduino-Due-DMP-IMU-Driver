/*!
 *  @file BMP280.cpp
 *
 *  This is a library for the BMP280 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BMP280 Sensor.
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
#include "Arduino.h"
#include <pdc_bmp280.hpp>


/************************************************
 * @brief  BMP280 constructor using i2c
 * @param  
 ************************************************/
BMP280::BMP280(void)
{
	_is_reading = false;
	_qnh = 1013.2;
}

BMP280::~BMP280(void) 
{
}


/************************************************
 *  Initialises the sensor.
 *  @param addr
 *         The I2C address to use (default = 0x77)
 *  @param chipid
 *         The expected chip ID (used to validate connection).
 *  @return True if the init was successful, otherwise false.
 ************************************************/
bool BMP280::begin(uint8_t addr, uint8_t chipid) {
	_i2caddr = addr;

	// i2c
	i2c_bus.Begin();
	if (read8(BMP280_REGISTER_CHIPID) != chipid)
	return false;

	readCoefficients();
	// write8(BMP280_REGISTER_CONTROL, 0x3F); /* needed? */
	setSampling();
	delay(100);
	return true;
}

/************************************************
 * Sets the sampling config for the device.
 * @param mode
 *        The operating mode of the sensor.
 * @param tempSampling
 *        The sampling scheme for temp readings.
 * @param pressSampling
 *        The sampling scheme for pressure readings.
 * @param filter
 *        The filtering mode to apply (if any).
 * @param duration
 *        The sampling duration.
 ************************************************/
void BMP280::setSampling(sensor_mode mode,
						 sensor_sampling tempSampling,
						 sensor_sampling pressSampling,
						 sensor_filter filter,
						 standby_duration duration  ) 
{
	_measReg.mode = mode;
	_measReg.osrs_t = tempSampling;
	_measReg.osrs_p = pressSampling;

	_configReg.filter = filter;
	_configReg.t_sb = duration;
	
	write8(BMP280_REGISTER_CONFIG, _configReg.get());
	write8(BMP280_REGISTER_CONTROL, _measReg.get());
}


/************************************************
 *	@brief  Writes an 8 bit value over I2C/SPI
 ************************************************/
void BMP280::write8(byte reg, byte value) {
	i2c_bus.WriteRegBlocked( _i2caddr, reg, value, 10);
}

/************************************************
 *  @brief  Reads an 8 bit value over I2C/SPI
 *  @param  reg
 *          selected register
 *  @return value from selected register
 ************************************************/
uint8_t BMP280::read8(byte reg) {
	uint8_t value = 0;
	i2c_bus.ReadRegBlocked(_i2caddr, reg, 1, &value, 10);

	return value;
}

/************************************************!
 *  @brief  Reads a 16 bit value over I2C/SPI
 ************************************************/
uint16_t BMP280::read16(byte reg) {
	uint16_t temp = read16_LE(reg);
	return (temp >> 8) | (temp << 8);
}

uint16_t BMP280::read16_LE(byte reg) {
	// Arduino Due ARM Cortex-M3 ATSAM3X8E is set to little endian.
	// That is, the natural order of reading starts from LSB, 
	// which matches the order in the I2C definition.
	uint16_t value;
	int res = i2c_bus.ReadRegBlocked(_i2caddr, reg, 2, (uint8_t *)(&value), 10);
	if(res)
	{
		Serial.print("r16 failed: " + String(res) + " @ ");
		Serial.print(reg, HEX);
	}
	return value;
}

/************************************************
 *   @brief  Reads a signed 16 bit value over I2C/SPI
 ************************************************/
int16_t BMP280::readS16(byte reg) { 
	return (int16_t)read16(reg); 
}

int16_t BMP280::readS16_LE(byte reg) {
	return (int16_t)read16_LE(reg);
}

/************************************************
 *  @brief  Reads a 24 bit value over I2C/SPI
 *  Little endian. 
 ************************************************/
uint32_t BMP280::read24(byte reg) {
	uint32_t value=0;
	/*uint8_t bfr[3];
	i2c_bus.ReadRegBlocked(_i2caddr, reg, 3, bfr, 10);
	
	value = (uint32_t)bfr[0] | 
			((uint32_t)bfr[1]) << 8 |
			((uint32_t)bfr[2]) << 16;*/
	i2c_bus.ReadRegBlocked(_i2caddr, reg, 3, (uint8_t *)(&value), 10);
	return value;
}

/************************************************
 *  @brief  Reads the factory-set coefficients
 ************************************************/
void BMP280::readCoefficients() {
	_bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
	_bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
	_bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

	_bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
	_bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
	_bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
	_bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
	_bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
	_bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
	_bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
	_bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
	_bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

/************************************************
 * Reads the temperature from the device.
 * @return The temperature in degress celcius.
 ************************************************/
float BMP280::readTemperature() {
	int32_t var1, var2;

	int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
	adc_T >>= 4;

	var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
		  ((int32_t)_bmp280_calib.dig_T2)) >>
		 11;

	var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
			((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
		   12) *
		  ((int32_t)_bmp280_calib.dig_T3)) >>
		 14;

	_t_fine = var1 + var2;

	float T = (_t_fine * 5 + 128) >> 8;
	_temp = T/100.0f;
	return _temp;
}

/************************************************
 * Reads the barometric pressure from the device.
 * @return Barometric pressure in Pa.
 ************************************************/
float BMP280::readPressure() {
	int64_t var1, var2, p;

	// Must be done first to get the _t_fine variable set up
	readTemperature();

	int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
	adc_P >>= 4;

	var1 = ((int64_t)_t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
		 ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
	var1 =
	  (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

	if (var1 == 0) {
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
	_pres = (float)p/256.0f;
	return _pres;
}

/************************************************
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 ************************************************/
float BMP280::readAltitude(float seaLevelhPa) {
	float altitude;

	float pressure = readPressure(); // in Si units for Pascal
	pressure /= 100;

	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

	return altitude;
}

/*****************************************************
 * @brief 
 * @param 
 * @return -1: error
 *          0: pending
 *		    1: success
 *          2: pressure unavailable
 *****************************************************/
int BMP280::UpdateAllNonBlocked() 
{
	uint8_t bfr[6];
	
	// ===== I2C communication =====
	if(!_is_reading) 
	{
		if(i2c_bus.ReadReg(_i2caddr, BMP280_REGISTER_PRESSUREDATA, 6))
			return -1;
		_start_us = micros();
		_is_reading = true;
		return 0;
	}
	else {
		int ret = i2c_bus.UpdateReadReg();
		if(ret >= 1 && ret < 4 && (micros() - _start_us)< 1000)
			return 0;	// still spining
		
		else if( ret != 4 )  
			return -1;	// timeout
		
		else {
			// success, Fetch data
			if( i2c_bus.FetchRegReadData(bfr, 6) )
				return -1;

			_is_reading = false;
		}
	}
	
	// ===== Calculate temperature =====
	// Handles temperature data before pressure for correction
	// to get the _t_fine variable set up
	int32_t var_t1, var_t2;
	int32_t adc_T = (int32_t)bfr[5] | 
					(int32_t)bfr[4] << 8 | 
					(int32_t)bfr[3] << 16;
	adc_T >>= 4;
	var_t1 = ((((adc_T >> 3) - (_bmp280_calib.dig_T1 << 1))) *
			  (_bmp280_calib.dig_T2)) >> 11;

	var_t2 = (((((adc_T >> 4) - (_bmp280_calib.dig_T1)) *
			    ((adc_T >> 4) - (_bmp280_calib.dig_T1))) >> 12) *
			  (_bmp280_calib.dig_T3)) >> 14;

	_t_fine = var_t1 + var_t2;
	_temp = (float)((_t_fine * 5 + 128) >> 8)/100.0f;
	
	// ===== Calculate pressure =====
	int64_t var_p1, var_p2, p;
	int32_t adc_P = (int32_t)bfr[2] | 
					(int32_t)bfr[1] << 8 |
					(int32_t)bfr[0] << 16;
	adc_P >>= 4;
	var_p1 = _t_fine - 128000;
	var_p2 = var_p1 * var_p1 * _bmp280_calib.dig_P6;
	var_p2 = var_p2 + ((var_p1 * _bmp280_calib.dig_P5) << 17);
	var_p2 = var_p2 + ((_bmp280_calib.dig_P4) << 35);
	var_p1 = ((var_p1 * var_p1 * _bmp280_calib.dig_P3) >> 8) +
		 ((var_p1 * _bmp280_calib.dig_P2) << 12);
	var_p1 =
	  (((((int64_t)1) << 47) + var_p1)) * (_bmp280_calib.dig_P1) >> 33;

	if (var_p1 == 0) {
		return 2; // avoid exception caused by division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var_p2) * 3125) / var_p1;
	var_p1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var_p2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

	p = ((p + var_p1 + var_p2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
	_pres = (float)p/256.0f;
	
	// ===== Calculate altitude =====
	_alt = 44330 * (1.0 - pow(_pres/100 / _qnh, 0.1903));

	return 1;
}

/**********************************************************
 * @brief Set internal reference for sea level barometric 
 *        pressure. The default value is 
 * @param seaLevelhPa [hPa] Current hPa at sea level.
 * @return None
 *********************************************************/
void BMP280::SetSeaLevelPressure(float seaLevelhPa)
{
	_qnh = seaLevelhPa;
}
void BMP280::GetSeaLevelPressure(float *seaLevelhPa)
{
	(*seaLevelhPa) = _qnh;
}

void BMP280::GetData(float * pres, float * temp, float * alt)
{
	(*pres) = _pres;
	(*temp) = _temp;
	(*alt) = _alt;
}

/************************************************
 * Calculates the pressure at sea level (QFH) from the specified altitude,
 * and atmospheric pressure (QFE).
 * @param  altitude      Altitude in m
 * @param  atmospheric   Atmospheric pressure in hPa
 * @return The approximate pressure in hPa
 ************************************************/
float BMP280::seaLevelForAltitude(float altitude, float atmospheric) {
	// Equation taken from BMP180 datasheet (page 17):
	// http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude.  See this thread for more information:
	// http://forums.adafruit.com/viewtopic.php?f=22&t=58064
	return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/************************************************
	@brief  calculates the boiling point  of water by a given pressure
	@param pressure pressure in hPa
	@return temperature in °C
************************************************/

float BMP280::waterBoilingPoint(float pressure) {
  // Magnusformular for calculation of the boiling point of water at a given
  // pressure
	return (234.175 * log(pressure / 6.1078)) /
		   (17.08085 - log(pressure / 6.1078));
}

/************************************************
 *  @brief  Take a new measurement (only possible in forced mode)
 *  !!!todo!!!
 ************************************************/
/*
void BMP280::takeForcedMeasurement()
{
	// If we are in forced mode, the BME sensor goes back to sleep after each
	// measurement and we need to set it to forced mode once at this point, so
	// it will take the next measurement and then return to sleep again.
	// In normal mode simply does new measurements periodically.
	if (_measReg.mode == MODE_FORCED) {
		// set to forced mode, i.e. "take next measurement"
		write8(BMP280_REGISTER_CONTROL, _measReg.get());
		// wait until measurement has been completed, otherwise we would read
		// the values from the last measurement
		while (read8(BMP280_REGISTER_STATUS) & 0x08)
				delay(1);
	}
}
*/

/************************************************
 *  @brief  Resets the chip via soft reset
 ************************************************/
void BMP280::reset(void) {
	write8(BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

/************************************************
	@brief  Gets the most recent sensor event from the hardware status register.
	@return Sensor status as a byte.
 ************************************************/
uint8_t BMP280::getStatus(void) {
	return read8(BMP280_REGISTER_STATUS);
}

/************************************************
	@brief  Gets an Adafruit Unified Sensor object for the temp sensor component
	@return Adafruit_Sensor pointer to temperature sensor
 ************************************************/
/*Adafruit_Sensor *BMP280::getTemperatureSensor(void) {
  return temp_sensor;
}
*/
/************************************************
	@brief  Gets an Adafruit Unified Sensor object for the pressure sensor
   component
	@return Adafruit_Sensor pointer to pressure sensor
 ************************************************/
 /*
Adafruit_Sensor *BMP280::getPressureSensor(void) {
  return pressure_sensor;
}
*/

/************************************************
	@brief  Gets the sensor_t data for the BMP280's temperature sensor
************************************************/

/*
void BMP280_Temp::getSensor(sensor_t *sensor) {
  // Clear the sensor_t object
  memset(sensor, 0, sizeof(sensor_t));

  // Insert the sensor name in the fixed length char array
  strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->max_value = -40.0; // Temperature range -40 ~ +85 C
  sensor->min_value = +85.0;
  sensor->resolution = 0.01; //  0.01 C
}*/

/**************************************************************************/
/*!
	@brief  Gets the temperature as a standard sensor event
	@param  event Sensor event object that will be populated
	@returns True
*/
/**************************************************************************/
/*
bool BMP280_Temp::getEvent(sensors_event_t *event) {
  // Clear the event
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = millis();
  event->temperature = _theBMP280->readTemperature();
  return true;
}
*/

/*!
	@brief  Gets the sensor_t data for the BMP280's pressure sensor
*/
/**************************************************************************/
/*
void BMP280_Pressure::getSensor(sensor_t *sensor) {
  // Clear the sensor_t object 
  memset(sensor, 0, sizeof(sensor_t));

  // Insert the sensor name in the fixed length char array
  strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->max_value = 300.0; // 300 ~ 1100 hPa
  sensor->min_value = 1100.0;
  sensor->resolution = 0.012; // 0.12 hPa relative
}
*/
/**************************************************************************/
/*!
	@brief  Gets the pressure as a standard sensor event
	@param  event Sensor event object that will be populated
	@returns True
*/
/**************************************************************************/
/*
bool BMP280_Pressure::getEvent(sensors_event_t *event) {
  // Clear the event
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_PRESSURE;
  event->timestamp = millis();
  event->pressure = _theBMP280->readPressure() / 100; // convert Pa to hPa
  return true;
}*/
