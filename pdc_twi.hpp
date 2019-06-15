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

		NVIC_DisableIRQ(TWI1_IRQn);
		NVIC_ClearPendingIRQ(TWI1_IRQn);
		NVIC_SetPriority(TWI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 3));
		NVIC_EnableIRQ(TWI1_IRQn);

		// Disable PDC channel
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;

		TWI_ConfigureMaster(WIRE_INTERFACE, TWI_CLOCK, VARIANT_MCK);
		
		mode = PdcTwoWireMode::MODE_STDBY;
		mst_tx_status = PdcTwoWireStatus::PDC_OFF;
		mst_rx_status = PdcTwoWireStatus::PDC_OFF;
		WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;	// disable all interrupts
	}

	/******************************************************
		Public access
	*******************************************************/
	void WriteTo(uint8_t dev_addr, uint8_t *data_ptr, uint16_t len) {
		if(len == 0) return;
		mode = PdcTwoWireMode::MASTER_SEND;
			
		if( len == 1) {
			mst_tx_status = PdcTwoWireStatus::PDC_SINGLE_TX;
			//_tx_started = false;
			//p.719
			//MMR: Master Mode register
			// P.713 IADR is for extended addressing for non-7 bit address
			WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;	// set master mode
			WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE;
			WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;		// set stop for one byte
			WIRE_INTERFACE->TWI_THR = (*data_ptr);		// start the transmission
			
			// interrupt enable register, this *needs* to be set AFTER THR
			WIRE_INTERFACE->TWI_IER = TWI_IER_TXCOMP | TWI_IER_NACK;	
			return;
		}else {
			//p.718 start
			SetPdcWriteAddr(data_ptr, len);
			SetTwiMasterWrite(dev_addr);
			mst_tx_status = PdcTwoWireStatus::PDC_RUNNING;
			WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTEN;					// enable TX
			WIRE_INTERFACE->TWI_IER = TWI_IER_ENDTX | TWI_IER_NACK ;	// interrupt enable register
		}
	}

	void ReadFrom(uint8_t dev_addr, uint8_t *data_ptr, uint16_t len) {
		if(len == 0) return;
		mode = PdcTwoWireMode::MASTER_RECV;
		_rx_stop_set = false;
		if( len == 1) {
			// p.722 & p.715
			mst_rx_status = PdcTwoWireStatus::PDC_SINGLE_RX;
			WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
			WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD; // master read
			WIRE_INTERFACE->TWI_CR = TWI_CR_START | TWI_CR_STOP;
			WIRE_INTERFACE->TWI_IER = TWI_IER_RXRDY;
		}else {
			SetPdcReadAddr(data_ptr, len);
			SetTwiMasterRead(dev_addr);
			mst_rx_status = PdcTwoWireStatus::PDC_RUNNING;
			WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTEN;
			WIRE_INTERFACE->TWI_CR = TWI_CR_START;
			WIRE_INTERFACE->TWI_IER = TWI_IER_RXRDY | TWI_IER_NACK;
		}
	}
	
	bool TxComplete() {
		if( mst_tx_status == PdcTwoWireStatus::PDC_SUCCESS ) return true;
		else return false;
	}
	bool RxComplete() {
		if( mst_rx_status == PdcTwoWireStatus::PDC_SUCCESS ) return true;
		else return false;
	}
	/*
	bool TxRunning() {
		if(( mst_tx_status == PdcTwoWireStatus::PDC_RUNNING) || ( mst_tx_status == PdcTwoWireStatus::PDC_STDBY) 
			|| ( mst_tx_status == PdcTwoWireStatus::PDC_SINGLE_TX)) {
			return true;
		}else { 	return false;	}
	}
	bool RxRunning() {
		if(( mst_rx_status == PdcTwoWireStatus::PDC_RUNNING)  || ( mst_rx_status == PdcTwoWireStatus::PDC_STDBY)) {
			return true;
		}else{ return false;	}
	}*/

	/******************************************************
		Set relative address and counter into PDC register
	*******************************************************/
	static inline int WriteTHR(uint8_t data) {
		WIRE_INTERFACE->TWI_THR = data;
		//WIRE_INTERFACE->TWI_THR = 0x00;
	}
	
	static inline int SetPdcWriteAddr(uint8_t *data, uint16_t count) {
		///TODO: add validation
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
		WIRE_INTERFACE->TWI_TPR = (RwReg)data;		//Transmit Pointer Register
		WIRE_INTERFACE->TWI_TCR = count;			//Transmit Counter Register
		WIRE_INTERFACE->TWI_TNPR = 0;				//Transmit Next Pointer Register
		WIRE_INTERFACE->TWI_TNCR = 0;				//Transmit Next Pointer Counter
		mst_tx_status = PdcTwoWireStatus::PDC_STDBY;
		mst_rx_status = PdcTwoWireStatus::PDC_NOTME;
	}
	static inline int SetPdcReadAddr(uint8_t *data, uint16_t count) {
		// seems like the last data will be move into the RHR, however, it will not 
		// be moved by the PDC because the STOP is already set. This is quite weird.
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_RXTDIS | TWI_PTCR_TXTDIS;
		WIRE_INTERFACE->TWI_RPR = (RwReg)data;		//Receive Pointer Register
		WIRE_INTERFACE->TWI_RCR = count;			//Receive Couter Register
		WIRE_INTERFACE->TWI_RNPR = 0;
		WIRE_INTERFACE->TWI_RNCR = 0;
		mst_tx_status = PdcTwoWireStatus::PDC_NOTME;
		mst_rx_status = PdcTwoWireStatus::PDC_STDBY;
	}
	
	/******************************************************
		Set TWI control mode
	*******************************************************/
	static inline int SetTwiMasterWrite(uint8_t dev_addr) {
		//MMR: Master Mode register
		// P.713 IADR is for extended addressing for non-7 bit address
		WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE;
		WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;	// set master mode
	}
	
	static inline int SetTwiMasterRead(uint8_t dev_addr) {
		WIRE_INTERFACE->TWI_MMR = TWI_MMR_DADR(dev_addr) | TWI_MMR_IADRSZ_NONE | TWI_MMR_MREAD; // master read
		WIRE_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;
	}

	
	inline void IsrHandler() {
		int sr = WIRE_INTERFACE->TWI_SR;
		int rcr = WIRE_INTERFACE->TWI_RCR; // Receive Counter Register
		//Serial.print("isr: ");
		//Serial.println(sr, BIN);
		
		//============= SINGLE transmitting =============//
		if(mst_tx_status == PdcTwoWireStatus::PDC_SINGLE_TX) {
			if (sr & TWI_SR_NACK){
				// failed, no ack on I2C bus
				mst_tx_status = PdcTwoWireStatus::PDC_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				Serial.println("txSclNAck");
				return;
			}
			else if( sr & TWI_SR_TXCOMP) {
					mst_tx_status = PdcTwoWireStatus::PDC_SUCCESS;
					WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
					Serial.println("txSglComp");
					return;
			}else {
				mst_tx_status = PdcTwoWireStatus::PDC_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				Serial.println("txSglUNKwn");
				return;
			}
		}
		
		//============= SINGLE recieving =============//
		if(mst_rx_status == PdcTwoWireStatus::PDC_SINGLE_RX) {
			if (sr & TWI_SR_NACK) {
				// failed, no ack on I2C bus
				mst_rx_status = PdcTwoWireStatus::PDC_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				Serial.println("txSclNAck");
				return;
			}
			if( sr & TWI_SR_RXRDY) {
				mst_rx_status = PdcTwoWireStatus::PDC_SUCCESS;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				rxdata = WIRE_INTERFACE->TWI_RHR;
				Serial.println("rxSglComp");
				return;
			}else {
				mst_tx_status =  PdcTwoWireStatus::PDC_FAILED;
				Serial.println("rxSglNotTxComp");
				return;
			}
		}
		
		//============= MULTI transmitting =============//
		if(mst_tx_status == PdcTwoWireStatus::PDC_RUNNING) {
			if (sr & TWI_SR_NACK){			// No ack, probably something is disconnected
				mst_tx_status = PdcTwoWireStatus::PDC_FAILED;
				WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
				Serial.println("txNAck");
				return;
			}
			if( sr & TWI_SR_TXCOMP) {
				mst_tx_status = PdcTwoWireStatus::PDC_SUCCESS;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// Peripheral Transfer Counter Register
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
		if(mst_rx_status == PdcTwoWireStatus::PDC_RUNNING) {
			if( sr & TWI_SR_NACK ) {			// no ack, when slave not response to initial address call
				mst_rx_status = PdcTwoWireStatus::PDC_FAILED;
				WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
				WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable pdc
				Serial.println("rxNAck");
				return;
			}
			
			//Serial.print("RCR: ");
			//Serial.println(rcr);
			if( sr & TWI_IER_ENDRX ) {
				mst_rx_status = PdcTwoWireStatus::PDC_SUCCESS;
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
		Serial.print("tx_st: ");
		Serial.print(mst_tx_status);
		Serial.print(", rx st: ");
		Serial.print(mst_rx_status);
		Serial.print(", PTSR: ");		// PDC Transfer Status Register
		Serial.println(WIRE_INTERFACE->TWI_PTSR, BIN);
		
		WIRE_INTERFACE->TWI_CR = TWI_CR_STOP;							// make sure things is stooping
		WIRE_INTERFACE->TWI_IDR = WIRE_INTERFACE -> TWI_IMR;			// disable all interrupts
		WIRE_INTERFACE->TWI_PTCR = TWI_PTCR_TXTDIS | TWI_PTCR_RXTDIS;	// disable PDC
		mst_tx_status = PdcTwoWireStatus::PDC_FAILED;
		mst_rx_status = PdcTwoWireStatus::PDC_FAILED;
		return;
	}	// IsrHandler
	
	
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
		PDC_NOTME,
		PDC_STDBY,
		PDC_RUNNING,
		PDC_SINGLE_TX,
		PDC_SINGLE_RX,
		PDC_SUCCESS,
		PDC_FAILED
	};
	volatile static PdcTwoWireStatus mst_tx_status, mst_rx_status;  ///TODO: static is dirty 
	//static uint16_t rx_len;
};	//class PdcTwi

#endif