#ifndef I2C_DEVICES_H_
#define I2C_DEVICES_H_

#include "pdc_twi.hpp"
#include <vector>

PdcTwi i2c;
#define I2C_BFR_SIZE 128
uint8_t tx_bfr[I2C_BFR_SIZE];
uint8_t rx_bfr[I2C_BFR_SIZE];

//Future plan:
//	1. Add different TWI interface handling in the future 
class I2cBus {
public:

	enum I2cBusStatus {
		BUS_UNINIT,
		BUS_STDBY,
		BUS_WRITING,
		BUS_ASK_WRITING,
		BUS_ASK_READING,
		BUS_ASK_FINISH,
		BUS_LAST_FAILED
	};
	I2cBusStatus _bus_st;
	
	int Begin()
	{
		i2c.Init();			// this will block for ~10ms
		_bus_st = I2cBusStatus::BUS_STDBY;
	}
	void Reset()
	{
		i2c.Reset();		// this will block for ~15ms
	}
	
	int Add_Device(const uint8_t dev_addr)
	{	
		_dev_addr.push_back(dev_addr);
		_tx_bfr_p.push_back(new uint8_t[I2C_BFR_SIZE]);
		_tx_bfr_len.push_back(0);
		
		_rx_bfr_p.push_back(new uint8_t[I2C_BFR_SIZE]);
		_rx_bfr_len.pushback(0);
		return ( _dev_addr.size()-1 );	// the index for the device
	}
	
	
	/****************************************************************
						Set data in slave register
	*****************************************************************/
	int SetReg(uint8_t dev_id, const uint8_t reg, const uint8_t *data, unsigned int len)
	{
		if( dev_id >= _dev_addr.size() ) 
		{
			return -1;		// no such device!
		}
		if( ( _bus_st != I2cBusStatus::BUS_LAST_FAILED) ||
			( _bus_st != I2cBusStatus::BUS_STDBY)       ||
		    ( _bus_st != I2cBusStatus::BUS_ASK_FINISH			) 
		{
			return -2; 		// bus busy!
		}
		_curr_dev_id = dev_id;
		
		_tx_bfr_p[dev_id][1] = reg;						// slave register
		memcpy( (_tx_bfr_p[dev_id])+1 , data, len);		// data
		_tx_bfr_len[dev_id] = len + 1;
		i2c.WriteTo(_dev_addr[dev_id], _tx_bfr_p[dev_id], _tx_bfr_len[dev_id]);
		_bus_st = I2cBusStatus::BUS_WRITING;
		return 0;
	}
	int SetReg(uint8_t dev_id, const uint8_t reg, const uint8_t data)
	{
		return WriteTo(dev_id, reg, &_ask_dummy_bfr, 1);	// 1-byte data write
	}
	bool SetRegIsFinished() {
		if(_bus_st == I2cBusStatus::BUS_WRITING)
		{
			if( !i2c.TxComplete() )
			{
				return false;
			}else
			{
				_bus_st == I2cBusStatus::BUS_STDBY;
				return true;
			}
		}
		return false;
	}
	
	/*int LineUpWriteTo(uint8_t dev_id, const uint8_t reg, const uint8_t *data, unsigned int len)
	{
		if((_tx_queue_len + len) >= I2C_BFR_SIZE) {
			return -1; 		// buffer not enough
		}
		_tx_size_q.push(len);
		memcpy(&tx_bfr[_tx_queue_len],, data, len);
		_tx_queue_len += I2C_BFR_SIZE;
	}*/
	
	/****************************************************************
						Read data from slave register
		Currently, after GetReg() call, you have to poll 
		UpdateGetReg() by yourself. The alternative is to make
		hardware interface library to trigger next stage in 
		comunication within the interrupt. However, this will mean 
		the manipulation of various interrupt and hardware register 
		within the ISR handler, which is untested and will dramatically
		increase the complexity of the library.
	*****************************************************************/
	int GetReg(uint8_t dev_id, const uint8_t reg, unsigned int len)
	{
		if( dev_id >= _dev_addr.size() ) 
		{
			return -1;		// no such device!
		}
		if( ( _bus_st != I2cBusStatus::BUS_LAST_FAILED) ||
			( _bus_st != I2cBusStatus::BUS_STDBY)       ||
		    ( _bus_st != I2cBusStatus::BUS_ASK_FINISH	     ) 
		{
			return -2; 		// bus busy!
		}
		_curr_dev_id = dev_id;
		_rx_bfr_len[dev_id] = len;
		_bus_st = I2cBusStatus::BUS_ASK_WRITING;
		SetReg(dev_id, reg, _tx_bfr_p[dev_id], 0);	// only access register
	}
	int UpdateGetReg()
	{
		if(_bus_st != I2cBusStatus::BUS_ASK_WRITING) ||
		  (_bus_st != I2cBusStatus::BUS_ASK_READING)         )
		{
			return -1;		// not doing GetReg atm
		}
		
		if(_bus_st == I2cBusStatus::BUS_ASK_WRITING) {
			if( !i2c.TxComplete() )
			{
				return 1;		// tx not ready
			}else 
			{
				_bus_st = I2cBusStatus::BUS_ASK_READING
				i2c.ReadFrom( _dev_addr[_curr_dev_id], 
							  _rx_bfr_p[_curr_dev_id], 
							  _rx_bfr_len[_curr_dev_id]      );
				return 2;		// start reading
			}
		}
		if(_bus_st == I2cBusStatus::BUS_ASK_READING) {
			if( !i2c.RxComplete() )
			{
				return 3;		// rx not ready
			}else
			{
				_bus_st == I2cBusStatus::BUS_ASK_FINISH;
				return 4;		// finished
			}
		}
	}
	int FetchRegData(const uint8_t* bfr, unsigned int len)
	{
		if(_bus_st != I2cBusStatus::BUS_ASK_FINISH) 
		{
			return -1;		// haven't received anything
		}		
		if(len != _rx_bfr_len[_curr_dev_id])
		{
			return -2;		// copy length not correct
		}
		
		memcpy(bfr, _rx_bfr_p[_curr_dev_id], _rx_bfr_len[_curr_dev_id]);
		_bus_st == I2cBusStatus::BUS_STDBY;
		return 0;		// success
	}
	
private:
	
	uint8_t _curr_dev_id;
	std::vector<uint8_t> _dev_addr;
	std::vector<uint8_t *> _tx_bfr_p;	// pointer to buffer for each device
	std::vector<uint8_t> _tx_bfr_len;		// remaining data within dev buffer
	
	uint8_t _ask_dummy_bfr;				// just a precausion
	std::vector<uint8_t *> _rx_bfr_p;	// pointer to rx buffer for each device
	std::vector<uint8_t> _rx_bfr_len;	// asking reading length
	
	
	//std::queue<unsigned int> _tx_size_q;
	//unsigned int _tx_queue_len;
};

#endif