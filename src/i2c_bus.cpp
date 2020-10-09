#include "../include/i2c_bus.hpp"

uint8_t I2cBus::tx_bfr[I2C_BFR_SIZE];
uint8_t I2cBus::rx_bfr[I2C_BFR_SIZE];
	
int I2cBus::Begin()
{
	_i2c.Init();			// this will block for ~10ms
	_bus_st = I2cBusStatus::BUS_STDBY;
}
void I2cBus::Reset()
{
	_i2c.Reset();		// this will block for ~15ms
}

/****************************************************************
					Set data in slave register
*****************************************************************/
int I2cBus::WriteReg( const uint8_t dev_addr, const uint8_t reg, 
					  const uint8_t *data, unsigned int len      )
{

	if( len > I2C_BFR_SIZE-1) {
		return -4;			// Data larger then buffer size
	}
		
	if( _bus_st == I2cBusStatus::BUS_WRITING) {
		if( !WriteRegIsFinished() ) return -3;
	}
	if( ( _bus_st != I2cBusStatus::BUS_LAST_FAILED) &&
		( _bus_st != I2cBusStatus::BUS_STDBY )      &&
		( _bus_st != I2cBusStatus::BUS_ASK_FINISH )			) 
	{
		return -2; 		// bus busy!
	}
	
	tx_bfr[0] = reg;						// slave register
	memcpy( (tx_bfr)+1 , data, len);		// data
	
	// DEBUG
	//String str = "set reg:" + String(dev_addr, HEX) + ",";
	//str += String(*tx_bfr, HEX) + ",";
	//str += String(len) + ";";
	//Serial.println(str.c_str());
	_i2c.WriteTo(dev_addr, tx_bfr, len+1);
	_bus_st = I2cBusStatus::BUS_WRITING;
	return 0;
}
int I2cBus::WriteReg(const uint8_t dev_addr, const uint8_t reg, const uint8_t data)
{
	return WriteReg(dev_addr, reg, &data, 1);	// 1-byte data write
}

bool I2cBus::WriteRegIsFinished() 
{
	if(_bus_st == I2cBusStatus::BUS_WRITING)
	{
		if( !_i2c.TxComplete() )
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

int I2cBus::WriteRegBlocked( uint8_t dev_addr, const uint8_t reg, 
					 const uint8_t* data, const unsigned int len, 
					 unsigned long t_out_ms                       )
{
	unsigned long t_start = millis();
	unsigned long now;
	int ret = WriteReg(dev_addr, reg, data, len);
	if( ret )  return ret; 		// something went wrong
	do {
		ret = WriteRegIsFinished();
		now = millis();
	}while( (!ret) && (now-t_start < t_out_ms) );
	if(now-t_start >t_out_ms) return -10;		// timeout
	return ret;
}
int I2cBus::WriteRegBlocked(uint8_t dev_addr, const uint8_t reg, const uint8_t data, unsigned long t_out_ms)
{
	return WriteRegBlocked(dev_addr, reg, &data, 1, t_out_ms);
}


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
int I2cBus::ReadReg(uint8_t dev_addr, const uint8_t reg, unsigned int len)
{
	if( len > I2C_BFR_SIZE-1 )
	{
		Serial.print("Not enough Rx buffer:");
		Serial.println(len);
		return -1;
	}
	if( _bus_st == I2cBusStatus::BUS_WRITING) {
		if( !WriteRegIsFinished() ) return -3;
	}
	if( ( _bus_st != I2cBusStatus::BUS_LAST_FAILED) &&
		( _bus_st != I2cBusStatus::BUS_STDBY)       &&
		( _bus_st != I2cBusStatus::BUS_ASK_FINISH)	     ) 
	{
		Serial.print("ReadReg busy: ");
		Serial.println((unsigned int)_bus_st);
		return -2; 		// bus busy!
	}
	
	//Write the address into slave device
	if( WriteReg(dev_addr, reg, 0, 0) == 0)
	{
		_bus_st = I2cBusStatus::BUS_ASK_WRITING;
		_curr_read_dev_addr = dev_addr;
		_curr_read_len = len;
		return 0;
	}else{
		return -3;		// write failed
	}
}

int I2cBus::UpdateReadReg()
{
	int rtn = 0;
	if( (_bus_st != I2cBusStatus::BUS_ASK_WRITING) &&
		(_bus_st != I2cBusStatus::BUS_ASK_READING)         )
	{
		return -1;			// not doing ReadReg atm
	}
	
	if(_bus_st == I2cBusStatus::BUS_ASK_WRITING) {
		if( !_i2c.TxComplete() )
		{
			return 1;		// tx ongoing
		}else 
		{
			_bus_st = I2cBusStatus::BUS_ASK_READING;
			_i2c.ReadFrom( _curr_read_dev_addr, rx_bfr, _curr_read_len );
			rtn = 2;		// start reading
		}
	}
	if(_bus_st == I2cBusStatus::BUS_ASK_READING) {
		if( !_i2c.RxComplete() )
		{
			return 3;		// rx ongoing
		}else
		{
			_bus_st = I2cBusStatus::BUS_ASK_FINISH;
			rtn = 4;		// finished
		}
	}
	return rtn;
}

int I2cBus::FetchRegReadData(uint8_t* bfr, unsigned int len)
{
	if(_bus_st != I2cBusStatus::BUS_ASK_FINISH) 
	{
		return -1;		// haven't received anything
	}		
	if(len != _curr_read_len)
	{
		return -2;		// copy length not correct
	}
	
	memcpy(bfr, rx_bfr, len);
	_bus_st == I2cBusStatus::BUS_STDBY;
	return 0;		// success
}

int I2cBus::ReadRegBlocked(uint8_t dev_id, const uint8_t reg, 
				   unsigned int len, uint8_t* bfr, unsigned long t_out_ms)
{
	unsigned long t_start = millis();
	unsigned long now;
	int ret = ReadReg(dev_id, reg, len);
	if( ret )  return ret; 		// something went wrong
	do {
		ret = UpdateReadReg();
		now = millis();
	}while( (ret>0) && (ret<4) && (now-t_start < t_out_ms) );
	if(now-t_start >=t_out_ms) return -10;		// timeout
	if(ret != 4) return ret;				// error code
	
	return FetchRegReadData(bfr, len);		
}

	
namespace std {
  void __throw_length_error(char const*) {
  };
};
