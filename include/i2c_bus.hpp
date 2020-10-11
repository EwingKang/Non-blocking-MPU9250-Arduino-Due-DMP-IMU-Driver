#ifndef I2C_BUS_HPP_
#define I2C_BUS_HPP_

#include "pdc_twi.hpp"
//#include <vector>

#define I2C_BFR_SIZE 128

//Future plan:
//	1. Add different TWI interface handling in the future 
class I2cBus {
public:
	enum class I2cBusStatus {
		BUS_UNINIT,
		BUS_STDBY,
		BUS_WRITING,
		BUS_ASK_WRITING,
		BUS_ASK_READING,
		BUS_ASK_FINISH,
		BUS_LAST_FAILED
	};

	/*TODO static const char* I2cBusStatusStr[] = 
	{
		"BUS_UNINIT",
		"BUS_STDBY",
		"BUS_WRITING",
		"BUS_ASK_WRITING",
		"BUS_ASK_READING",
		"BUS_ASK_FINISH",
		"BUS_LAST_FAILED"
	};*/
	
	I2cBusStatus _bus_st;
	
	int Begin();
	void Reset();
	
	/****************************************************************
						Set data in slave register
	*****************************************************************/
	int WriteReg( const uint8_t dev_addr, const uint8_t reg, 
				const uint8_t *data, unsigned int len      );
	int WriteReg(const uint8_t dev_addr, const uint8_t reg, 
				 const uint8_t data);
	
	bool WriteRegIsFinished();
	
	int WriteRegBlocked( uint8_t dev_addr, const uint8_t reg, 
						 const uint8_t* data, const unsigned int len, 
						 unsigned long t_out_ms                       );
	int WriteRegBlocked(uint8_t dev_addr, const uint8_t reg, 
						const uint8_t data, unsigned long t_out_ms);

	
	/****************************************************************
						Read data from slave register
		Currently, after ReadReg() call, you have to poll 
		UpdateReadReg() by yourself. The alternative is to make
		hardware interface library to trigger next stage in 
		comunication within the interrupt. However, this will mean 
		the manipulation of various interrupt and hardware register 
		within the ISR handler, which is untested and will dramatically
		increase the complexity of the library.
	*****************************************************************/
	int ReadReg(uint8_t dev_addr, const uint8_t reg, unsigned int len);
	int UpdateReadReg();
	
	int FetchRegReadData(uint8_t* bfr, unsigned int len);
	int ReadRegBlocked(uint8_t dev_id, const uint8_t reg, 
					   unsigned int len, uint8_t* bfr, unsigned long t_out_ms);

	inline void IsrHandler()
	{
		_i2c.IsrHandler();
	};
		
private:
	static uint8_t tx_bfr[I2C_BFR_SIZE];
	static uint8_t rx_bfr[I2C_BFR_SIZE];
	PdcTwi _i2c;
	
	uint8_t _curr_read_dev_addr;
	unsigned int _curr_read_len;
	
	//std::queue<unsigned int> _tx_size_q;
	//unsigned int _tx_queue_len;
};

#endif  // I2C_DEVICES_H_