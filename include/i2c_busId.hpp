#ifndef I2C_DEVICES_H_
#define I2C_DEVICES_H_

#include "pdc_twi.hpp"
#include <vector>

#define I2C_BFR_SIZE 128
//uint8_t tx_bfr[I2C_BFR_SIZE];
//uint8_t rx_bfr[I2C_BFR_SIZE];

//Future plan TODO:
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
	
	// return  the id for the device
	int Add_Device(const uint8_t dev_addr)
	{	
		_dev_addr.push_back(dev_addr);
		_tx_bfr_p.push_back(new uint8_t[I2C_BFR_SIZE]);
		_tx_bfr_len.push_back(0);
		
		_rx_bfr_p.push_back(new uint8_t[I2C_BFR_SIZE]);
		_rx_bfr_len.push_back(0);
		return ( _dev_addr.size()-1 );	// the id for the device
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
		if( _bus_st == I2cBusStatus::BUS_WRITING) {
			if( !SetRegIsFinished() ) return -3;
		}
		if( ( _bus_st != I2cBusStatus::BUS_LAST_FAILED) &&
			( _bus_st != I2cBusStatus::BUS_STDBY )      &&
		    ( _bus_st != I2cBusStatus::BUS_ASK_FINISH )			) 
		{
			return -2; 		// bus busy!
		}
		_curr_dev_id = dev_id;
		
		_tx_bfr_p[dev_id][0] = reg;						// slave register
		memcpy( (_tx_bfr_p[dev_id])+1 , data, len);		// data
		_tx_bfr_len[dev_id] = len + 1;
		// DEBUG
		//String str = "set reg:" + String( _dev_addr[dev_id], HEX) + ",";
		//str += String( *_tx_bfr_p[dev_id], HEX) + ",";
		//str += String( _tx_bfr_len[dev_id]) + ";";
		//Serial.println(str.c_str());
		i2c.WriteTo(_dev_addr[dev_id], _tx_bfr_p[dev_id], _tx_bfr_len[dev_id]);
		_bus_st = I2cBusStatus::BUS_WRITING;
		return 0;
	}
	int SetReg(uint8_t dev_id, const uint8_t reg, const uint8_t data)
	{
		return SetReg(dev_id, reg, &data, 1);	// 1-byte data write
	}
	bool SetRegIsFinished() {
		if(_bus_st == I2cBusStatus::BUS_WRITING)
		{
			if( !i2c.TxComplete() )
			{
				return false;
			}else
			{
				_bus_st = I2cBusStatus::BUS_STDBY;
				return true;
			}
		}
		return false;
	}
	
	int SetRegBlocked( uint8_t dev_id, const uint8_t reg, 
	                   const uint8_t* data, const unsigned int len, 
					   unsigned long t_out                            )
	{
		unsigned long t_start = millis();
		unsigned long now;
		int ret = SetReg(dev_id, reg, data, len);
		if( ret )  return ret; 		// something went wrong
		do {
			ret = SetRegIsFinished();
			now = millis();
		}while( (!ret) && (now-t_start < t_out) );
		if(now-t_start >t_out) return -10;		// timeout
		return ret;
	}
	int SetRegBlocked(uint8_t dev_id, const uint8_t reg, const uint8_t data, unsigned long t_out)
	{
		return SetRegBlocked(dev_id, reg, &data, 1, t_out);
	}

	
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
			Serial.print("dev_id out of range: ");
			Serial.println(dev_id);
			return -1;		// no such device!
		}
		if( _bus_st == I2cBusStatus::BUS_WRITING) {
			if( !SetRegIsFinished() ) return -3;
		}
		if( ( _bus_st != I2cBusStatus::BUS_LAST_FAILED) &&
			( _bus_st != I2cBusStatus::BUS_STDBY)       &&
		    ( _bus_st != I2cBusStatus::BUS_ASK_FINISH)	     ) 
		{
			Serial.print("GetReg busy: ");
			Serial.println(_bus_st);
			return -2; 		// bus busy!
		}
		_curr_dev_id = dev_id;
		_rx_bfr_len[dev_id] = len;
		if( SetReg(dev_id, reg, _tx_bfr_p[dev_id], 0) == 0)
		{
			_bus_st = I2cBusStatus::BUS_ASK_WRITING;
			return 0;
		}else{
			return -3;		// write failed
		}
	}
	int UpdateGetReg()
	{
		int rtn = 0;
		if( (_bus_st != I2cBusStatus::BUS_ASK_WRITING) &&
		    (_bus_st != I2cBusStatus::BUS_ASK_READING)         )
		{
			return -1;			// not doing GetReg atm
		}
		
		if(_bus_st == I2cBusStatus::BUS_ASK_WRITING) {
			if( !i2c.TxComplete() )
			{
				return 1;		// tx ungoing
			}else 
			{
				_bus_st = I2cBusStatus::BUS_ASK_READING;
				i2c.ReadFrom( _dev_addr[_curr_dev_id], 
							  _rx_bfr_p[_curr_dev_id], 
							  _rx_bfr_len[_curr_dev_id]      );
				rtn = 2;		// start reading
			}
		}
		if(_bus_st == I2cBusStatus::BUS_ASK_READING) {
			if( !i2c.RxComplete() )
			{
				return 3;		// rx ungoing
			}else
			{
				_bus_st = I2cBusStatus::BUS_ASK_FINISH;
				rtn = 4;		// finished
			}
		}
		return rtn;
	}
	int FetchRegData(uint8_t* bfr, unsigned int len)
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
	
	int GetRegBlocked(uint8_t dev_id, const uint8_t reg, unsigned int len, uint8_t* bfr, unsigned long t_out)
	{
		unsigned long t_start = millis();
		unsigned long now;
		int ret = GetReg(dev_id, reg, len);
		if( ret )  return ret; 		// something went wrong
		do {
			ret = UpdateGetReg();
			now = millis();
		}while( (ret>0) && (ret<4) && (now-t_start < t_out) );
		if(now-t_start >t_out) return -10;		// timeout
		if(ret != 4) return ret;				// error code
		
		return FetchRegData(bfr, len);		
	}
	
	inline void IsrHandler() 
	{
		i2c.IsrHandler();
	}
		
private:
	PdcTwi i2c;
	
	uint8_t _curr_dev_id;
	std::vector<uint8_t> _dev_addr;
	std::vector<uint8_t *> _tx_bfr_p;	// pointer to buffer for each device
	std::vector<uint8_t> _tx_bfr_len;		// remaining data within dev buffer
	
	std::vector<uint8_t *> _rx_bfr_p;	// pointer to rx buffer for each device
	std::vector<uint8_t> _rx_bfr_len;	// asking reading length
	
	
	//std::queue<unsigned int> _tx_size_q;
	//unsigned int _tx_queue_len;
};

namespace std {
  void __throw_length_error(char const*) {
  }
}

#endif