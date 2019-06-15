/******************************************************************************
* PDC: Peripheral DMA Controller, while DMA stands for Direct Memory Access
* TWI: Two Wire Interface, same as I2C as far as our regards.
* Author: Ewing Kang
* Contact: f039281310@yahoo.com.tw
* References:
* Liscense:
******************************************************************************/
#ifdef TwoWire_h
#error PDC_TWI is likely to conflict with Wire.h library
#endif 

#ifndef PDC_TWI_H_
#define PDC_TWI_H_

#include <Arduino.h>

#define TWI_CLOCK    100000 	// 100K Hz

class PdcTwi {
public:
	byte rxdata;
	
	void Init() {
		BusReset();
		pmc_enable_periph_clk(WIRE_INTERFACE_ID);
		PIO_Configure(
					   g_APinDescription[PIN_WIRE_SDA].pPort,
					   g_APinDescription[PIN_WIRE_SDA].ulPinType,
					   g_APinDescription[PIN_WIRE_SDA].ulPin,
					   g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
		PIO_Configure(
					   g_APinDescription[PIN_WIRE_SCL].pPort,
					   g_APinDescription[PIN_WIRE_SCL].ulPinType,
					   g_APinDescription[PIN_WIRE_SCL].ulPin,
					   g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

		// setup vector interrupt controller
		NVIC_DisableIRQ(TWI1_IRQn);
		NVIC_ClearPendingIRQ(TWI1_IRQn);
		NVIC_SetPriority(TWI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 3));
		NVIC_EnableIRQ(TWI1_IRQn);
		
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;	// Disable PDC channel
		TWI_ConfigureMaster(WIRE_INTERFACE, TWI_CLOCK, VARIANT_MCK);	// set to master mode
		
		mode = PdcTwoWireMode::MODE_STDBY;
		master_state = PdcTwoWireStatus::PDC_OFF;
		WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
	}

	/****************************************************************
						Public read/write access
		[MMR] Master Mode register (p.719)
		[IADR] Internal ADdress Register, for non-7 bit I2C device
		       (p.713)
	*****************************************************************/
	void WriteTo(uint8_t dev_addr, uint8_t *data_ptr, uint16_t len) {
		if(len == 0) return;
		mode = PdcTwoWireMode::MASTER_SEND;
		if( len == 1) {
			master_state = PdcTwoWireStatus::PDC_SINGLE_TX;			
			// P.713 IADR is for extended addressing for non-7 bit address
			WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;	// set master mode
			WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE;
			WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;		// set stop for one byte
			WIRE_INTERFACE->TWI_THR = (*data_ptr);		// start the transmission
			
			// interrupt enable register, this *needs* to be set AFTER THR
			WIRE_INTERFACE->TWI_IER = TWI_IER_TXCOMP | TWI_IER_NACK;	
			return;
		}else {
			//p.718 start PDC procedure
			SetPdcWriteAddr(data_ptr, len);
			SetTwiMasterWrite(dev_addr);
			master_state = PdcTwoWireStatus::PDC_MULTI_TX;
			WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTEN;					// enable TX
			WIRE_INTERFACE->TWI_IER = TWI_IER_ENDTX | TWI_IER_NACK ;	// interrupt enable register
		}
	}

	void ReadFrom(uint8_t dev_addr, uint8_t *data_ptr, uint16_t len) {
		if(len == 0) return;
		mode = PdcTwoWireMode::MASTER_RECV;
		_rx_stop_set = false;
		if( len == 1) {
			// p.722 & p.715 procedure
			master_state = PdcTwoWireStatus::PDC_SINGLE_RX;
			WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
			WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD; // master read
			WIRE_INTERFACE->TWI_CR = TWI_CR_START | TWI_CR_STOP;
			WIRE_INTERFACE->TWI_IER = TWI_IER_RXRDY;
		}else {
			SetPdcReadAddr(data_ptr, len);
			SetTwiMasterRead(dev_addr);
			master_state = PdcTwoWireStatus::PDC_MULTI_RX;
			WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTEN;
			WIRE_INTERFACE->TWI_CR = TWI_CR_START;
			WIRE_INTERFACE->TWI_IER = TWI_IER_RXRDY | TWI_IER_NACK;
		}
	}
	
	bool TxComplete() {
		if( master_state == PdcTwoWireStatus::PDC_TX_SUCCESS ) return true;
		else return false;
	}
	bool RxComplete() {
		if( master_state == PdcTwoWireStatus::PDC_RX_SUCCESS ) return true;
		else return false;
	}
	

	/****************************************************************
							Set PDC addresses
		Set relative address and counter into PDC register
		Note:
			[PTCR] Peripheral Transfer Control Register
			[TPR]/[RPR] Transmit/Receive Pointer Register
			[TCR]/[RCR] Transmit/Receive Counter Register
			[TNPR]/[RNPR] Transmit/Receive Next Pointer Register
			[TNCR]/[RNCR] Transmit Next Pointer Counter
	*****************************************************************/
	static inline int SetPdcWriteAddr(uint8_t *data, uint16_t count) {
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
		WIRE_INTERFACE->TWI_TPR = (RwReg)data;		
		WIRE_INTERFACE->TWI_TCR = count;
		WIRE_INTERFACE->TWI_TNPR = 0;		
		WIRE_INTERFACE->TWI_TNCR = 0;				
		master_state = PdcTwoWireStatus::PDC_ADD_SET;
	}
	static inline int SetPdcReadAddr(uint8_t *data, uint16_t count) {
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
		WIRE_INTERFACE->TWI_RPR = (RwReg)data;		
		WIRE_INTERFACE->TWI_RCR = count;
		WIRE_INTERFACE->TWI_RNPR = 0;
		WIRE_INTERFACE->TWI_RNCR = 0;
		master_state = PdcTwoWireStatus::PDC_ADD_SET;
	}
	
	/***************************************************************
							Set TWI control mode
		Note:
			MMR-Master Mode register
			[P.713] IADR is only for extended addressing, i.e. 
			non-7 bit I2C device address
	****************************************************************/
	static inline int SetTwiMasterWrite(uint8_t dev_addr) {
		WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE;
		WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;	// set master mode
	}
	
	static inline int SetTwiMasterRead(uint8_t dev_addr) {
		WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD; // master read
		WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
	}

	/****************************************************************
							ISR handling
		Handles IRQ. Should be called in TWI1_Handler() in global 
		scope.
		Note:
	*****************************************************************/
	inline void IsrHandler() {
		int sr = WIRE_INTERFACE->TWI_SR;
		int rcr = WIRE_INTERFACE->TWI_RCR; // Receive Counter Register
		//Serial.print("isr: ");
		//Serial.println(sr, BIN);
		
		//============= SINGLE transmitting =============//
		if(master_state == PdcTwoWireStatus::PDC_SINGLE_TX) {
			if (sr & TWI_SR_NACK){
				// failed, no ack on I2C bus
				master_state = PdcTwoWireStatus::PDC_TX_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				Serial.println("txSclNAck");
				return;
			}
			else if( sr & TWI_SR_TXCOMP) {
					master_state = PdcTwoWireStatus::PDC_TX_SUCCESS;
					WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
					Serial.println("txSglComp");
					return;
			}else {
				master_state = PdcTwoWireStatus::PDC_TX_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				Serial.println("txSglUNKwn");
				return;
			}
		}
		
		//============= SINGLE recieving =============//
		if(master_state == PdcTwoWireStatus::PDC_SINGLE_RX) {
			if (sr & TWI_SR_NACK) {
				// failed, no ack on I2C bus
				master_state = PdcTwoWireStatus::PDC_RX_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				Serial.println("txSclNAck");
				return;
			}
			if( sr & TWI_SR_RXRDY) {
				master_state = PdcTwoWireStatus::PDC_RX_SUCCESS;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				rxdata = WIRE_INTERFACE->TWI_RHR;
				Serial.println("rxSglComp");
				return;
			}else {
				master_state =  PdcTwoWireStatus::PDC_RX_FAILED;
				Serial.println("rxSglNotTxComp");
				return;
			}
		}
		
		//============= MULTI transmitting =============//
		if(master_state == PdcTwoWireStatus::PDC_MULTI_TX) {
			if (sr & TWI_SR_NACK){			// No ack, probably something is disconnected
				master_state = PdcTwoWireStatus::PDC_TX_FAILED;
				WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
				Serial.println("txNAck");
				return;
			}
			if( sr & TWI_SR_TXCOMP) {
				master_state = PdcTwoWireStatus::PDC_TX_SUCCESS;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
				Serial.println("txComp");
				return;
			}
			if( sr & TWI_SR_ENDTX ) {
				// p.744: The Receive Counter Register has reached 0 since the last write in TWI_RCR or TWI_RNCR.
				// p.507 [Transmit Transfer End] This flag is set when PERIPH_TCR register reaches zero and the last
				//       data has been written into peripheral THR. It is reset by writing a non zero value in 
				//       PERIPH_TCR or PERIPH_TNCR.
				WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;		// This is necessary or SCL will hang low
				WIRE_INTERFACE->TWI_IDR = TWI_SR_ENDTX;		// Disable PDC end isr
				WIRE_INTERFACE->TWI_IER = TWI_IER_TXCOMP;	// so it'll trigger comp next time for multi-byte write
				Serial.println("txLast");
				return;	
			}else{
				Serial.println("txPng");
				return;  // transmittion
			}
		}
		
		//============= MULTI receiving =============//
		if(master_state == PdcTwoWireStatus::PDC_MULTI_RX) {
			if( sr & TWI_SR_NACK ) {			// no ack, when slave not response to initial address call
				master_state = PdcTwoWireStatus::PDC_RX_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable pdc
				Serial.println("rxNAck");
				return;
			}
			
			//Serial.print("RCR: ");
			//Serial.println(rcr);
			if( sr & TWI_IER_ENDRX ) {
				master_state = PdcTwoWireStatus::PDC_RX_SUCCESS;
				if( !_rx_stop_set)	WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;		// Stop case missed
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
				Serial.println("rxEnd");
				return;
			}else {
				// This is actually triggered by RXRDY, however, the PDC has already taken the data, 
				// so SR will show RXRDY as 0.
				// p.716 This should be fired when ever RHR is ready, i.e evertime a new data is ready.
				//       check if we're ending the RX
				// p.506 Receive Counter Register is counting down
				if( rcr <= 1) {
					WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
					_rx_stop_set = true;
					Serial.println("rxLastOne");
				}
				return;
			}
		}
		
		//============= Uncatched interrupt =============//
		Serial.print("Unknown ISR, SR: ");
		Serial.println(sr, BIN);
		Serial.print("status: ");
		Serial.print(master_state);
		Serial.print(", PTSR: ");		// PDC Transfer Status Register
		Serial.println(WIRE_INTERFACE->TWI_PTSR, BIN);
		
		WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;							// make sure things is stooping
		WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
		master_state = PdcTwoWireStatus::PDC_UNKOWN_FAILED;
		return;
	}	// IsrHandler()
	
	
	
	/****************************************************************
							Reset
		A hard, manual reset by BusReset() to clear slave pull down 
		issue.
	*****************************************************************/
	void Reset()
	{
		uint32_t i;
		
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDCs
		WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
		
		// TWI software reset 
		WIRE_INTERFACE->TWI_CR = TWI_CR_SWRST;
		WIRE_INTERFACE->TWI_RHR;

		// Wait at least 10 ms 
		for (i=0; i < 1000000; i++);

		// TWI Slave Mode Disabled, TWI Master Mode Disabled
		WIRE_INTERFACE->TWI_CR = TWI_CR_SVDIS | TWI_CR_MSDIS;
		
		// Disable interrupts
		NVIC_DisableIRQ(TWI1_IRQn);
		NVIC_ClearPendingIRQ(TWI1_IRQn);
		
		BusReset();
		master_state = PdcTwoWireStatus::PDC_UNINIT;
	}
	
	void BusReset()
	{
		// Sent 9 pulses over CLK pin, should free-up any slave SDA hangs
		pinMode(21, OUTPUT);
		for (int i = 0; i < 9; i++) {
			// I2C 100K Hz is ~10us per clock cycle. 
			// The delay value gives ~84us/8 clk cycle
			digitalWrite(21, HIGH);
			delayMicroseconds(3);	
			digitalWrite(21, LOW);
			delayMicroseconds(3);
		}
		pinMode(21, INPUT);
	}
	
private:
	bool _rx_stop_set;

	enum PdcTwoWireMode {
		MODE_UNINIT,
		MODE_STDBY,
		MASTER_SEND,
		MASTER_RECV,
		SLAVE_RECV,
		SLAVE_SEND
	};
	PdcTwoWireMode mode;
	
	enum PdcTwoWireStatus {
		PDC_UNINIT,
		PDC_OFF,
		PDC_ADD_SET,
		PDC_RUNNING,
		PDC_SINGLE_TX,
		PDC_SINGLE_RX,
		PDC_MULTI_TX,
		PDC_MULTI_RX,
		PDC_TX_SUCCESS,
		PDC_RX_SUCCESS,
		PDC_TX_FAILED,
		PDC_RX_FAILED,
		PDC_UNKOWN_FAILED
	};
	volatile static PdcTwoWireStatus master_state;  ///TODO: static is dirty 
	//static uint16_t rx_len;
};	//class PdcTwi

#endif