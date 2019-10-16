/**************************************************************************/
/*!
    @file     LXSAMD21DMX.cpp
    @author   Claude Heintz
    @license  BSD (see SAMD21DMX.h or http://lx.claudeheintzdesign.com/opensource.html)
    @copyright 2016 by Claude Heintz

    DMX Driver for Arduino MKR1000

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include "LXSAMD21DMX.h"
#include "Arduino.h"
#include "wiring_private.h"
#include "variant.h"
#include <inttypes.h>
#include <stdlib.h>
#include <rdm/rdm_utility.h>

//**************************************************************************************
// single instance and shared interrupt status

LXSAMD21DMX SAMD21DMX;

UID LXSAMD21DMX::THIS_DEVICE_ID(0x6C, 0x78, 0x00, 0x00, 0x00, 0x04);

uint8_t  _interrupt_mode;

// **************************** SERCOMn_Handler  ***************
// 
// DMX_SERCOM_HANDLER_FUNC macro points to handler name

void DMX_SERCOM_HANDLER_FUNC()
{
	switch ( _interrupt_mode ) {
		case ISR_OUTPUT_ENABLED:
			SAMD21DMX.outputIRQHandler();
			break;
		case ISR_INPUT_ENABLED:
			SAMD21DMX.inputIRQHandler();
			break;
		case ISR_RDM_ENABLED:
			SAMD21DMX.rdmIRQHandler();
			break;
	}
}


// **************************** global data (can be accessed in ISR)  ***************

Uart SerialDMX (&DMX_sercom, PIN_DMX_RX, PIN_DMX_TX, PAD_DMX_RX, PAD_DMX_TX);


void setBaudRate(uint32_t baudrate) {
	DMX_SERCOM->USART.CTRLA.bit.ENABLE = 0x0u; // must be disabled before writing to USART.BAUD or USART.CTRLA
	uint16_t sampleRateValue = 16;

    // see SERCOM.initUART
    uint32_t baudTimes8 = (SystemCoreClock * 8) / (sampleRateValue * baudrate);

    DMX_SERCOM->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    DMX_SERCOM->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
    DMX_SERCOM->USART.CTRLA.bit.ENABLE = 0x1u; // re-enable
}

//************************************************************************************
// ************************  LXSAMD21DMXOutput member functions  ********************

LXSAMD21DMX::LXSAMD21DMX ( void ) {
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_interrupt_mode = ISR_DISABLED;
	
	//zero buffer including _dmxData[0] which is start code
    memset(_dmxData, 0, DMX_MAX_SLOTS+1);
    //_dre_flag = 0;
}
    

LXSAMD21DMX::~LXSAMD21DMX ( void ) {
    stop();
    _receive_callback = NULL;
}


void LXSAMD21DMX::startOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}

	if ( _interrupt_mode == ISR_INPUT_ENABLED ) {
		stop();
	}
	
	if ( _interrupt_mode == ISR_DISABLED ) {	//prevent messing up sequence if already started...
	  SerialDMX.begin(DMX_BREAK_BAUD, (uint8_t)SERIAL_8N2);
  
	  // Assign pin mux to SERCOM functionality (must come after SerialDMX.begin)
	  pinPeripheral(PIN_DMX_RX, MUX_DMX_RX);
	  pinPeripheral(PIN_DMX_TX, MUX_DMX_TX);

	  _interrupt_mode = ISR_OUTPUT_ENABLED;
	  _rdm_task_mode = DMX_TASK_SEND;     
	  _dmx_send_state = DMX_STATE_BREAK;

	  transmissionComplete();	//sets TXC interrupt and sends break
	}
}

void LXSAMD21DMX::startInput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _interrupt_mode == ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_mode == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		SerialDMX.begin(DMX_DATA_BAUD, (uint8_t)SERIAL_8N2);
  
	   // Assign pin mux to SERCOM functionality (must come after SerialDMX.begin)
	   pinPeripheral(PIN_DMX_RX, MUX_DMX_RX);
	   pinPeripheral(PIN_DMX_TX, MUX_DMX_TX);

		_next_read_slot = 0;              
		_dmx_read_state = DMX_STATE_IDLE;

		_interrupt_mode = ISR_INPUT_ENABLED;
	}
}

void LXSAMD21DMX::startRDM( uint8_t pin, uint8_t direction ) {
	pinMode(pin, OUTPUT);
	_direction_pin = pin;
	if ( direction ) {
		startOutput();							//enables transmit interrupt
		_next_read_slot = 0;              
		_dmx_read_state = DMX_STATE_IDLE;
		_rdm_task_mode = DMX_TASK_SEND;
	} else {
		startInput();
		DMX_SERCOM->USART.INTENSET.reg =  SERCOM_USART_INTENSET_TXC | SERCOM_USART_INTENSET_ERROR;
	}
	_interrupt_mode = ISR_RDM_ENABLED;
}

void LXSAMD21DMX::stop ( void ) {
   SerialDMX.end();
	_interrupt_mode = ISR_DISABLED;
}

void LXSAMD21DMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

void LXSAMD21DMX::setMaxSlots (int slots) {
	if ( slots > DMX_MIN_SLOTS ) {
		_slots = slots;
	} else {
		_slots = DMX_MIN_SLOTS;
	}
}

uint8_t LXSAMD21DMX::getSlot (int slot) {
	return _dmxData[slot];
}

void LXSAMD21DMX::setSlot (int slot, uint8_t value) {
	_dmxData[slot] = value;
}

uint8_t* LXSAMD21DMX::dmxData(void) {
	return &_dmxData[0];
}

uint8_t* LXSAMD21DMX::rdmData( void ) {
	return _rdmPacket;
}

uint8_t* LXSAMD21DMX::receivedData( void ) {
	return _receivedData;
}

uint8_t* LXSAMD21DMX::receivedRDMData( void ) {
	return _rdmData;
}

//************************************************************************************

void LXSAMD21DMX::transmissionComplete( void ) {
	if ( _dmx_send_state == DMX_STATE_BREAK ) {
		setBaudRate(DMX_BREAK_BAUD);
        _dmx_send_state = DMX_STATE_START;
        _next_send_slot = 0;
        DMX_SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
        DMX_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC;
        DMX_SERCOM->USART.DATA.reg = 0;	//break
	} else if ( _dmx_send_state == DMX_STATE_IDLE ) {		//after data completely sent
		if ( _rdm_task_mode == 	DMX_TASK_SEND_RDM ) {
			DMX_SERCOM->USART.INTFLAG.bit.TXC = 1;						// clear txc interrupt !!!
			DMX_SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_TXC;	// shut off interrupt
			_rdm_task_mode = DMX_TASK_RECEIVE;
			if ( _rdm_read_handled ) {
				_dmx_read_state = DMX_READ_STATE_START;
				_next_read_slot = 0;
			} else {
				_dmx_read_state = DMX_READ_STATE_IDLE;
			}
			digitalWrite(_direction_pin, LOW);
		} else {
			_dmx_send_state = DMX_STATE_BREAK;
			// txc interrupt not cleared so it will fire again...
			// if necessary, change mode
			if ( _rdm_task_mode == 	DMX_TASK_SET_SEND_RDM ) {
				_rdm_task_mode = DMX_TASK_SEND_RDM;
			} else if ( _rdm_task_mode == DMX_TASK_SET_SEND ) {
				_rdm_task_mode = DMX_TASK_SEND;
			}
		}
	} else if ( _dmx_send_state == DMX_STATE_START ) {
		setBaudRate(DMX_DATA_BAUD);
        _dmx_send_state = DMX_STATE_DATA;
        DMX_SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_TXC;
        DMX_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
        	//rdm task (?)
        if ( _rdm_task_mode == DMX_TASK_SEND_RDM ) {
        	DMX_SERCOM->USART.DATA.reg = _rdmPacket[_next_send_slot++];
        } else {
        	DMX_SERCOM->USART.DATA.reg = _dmxData[_next_send_slot++];
        }
	}
}

void LXSAMD21DMX::dataRegisterEmpty( void ) {
	if ( _dmx_send_state == DMX_STATE_DATA ) {
		if ( _rdm_task_mode == 	DMX_TASK_SEND_RDM ) {
			DMX_SERCOM->USART.DATA.reg = _rdmPacket[_next_send_slot++];	//send next slot;
			if ( _next_send_slot > _rdm_len ) {
				_dmx_send_state = DMX_STATE_IDLE;
				DMX_SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
				DMX_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC;
				// switch to wait for last byte transmission to complete
			}
		} else {
			DMX_SERCOM->USART.DATA.reg = _dmxData[_next_send_slot++];	//send next slot;
			if ( _next_send_slot > _slots ) {
				_dmx_send_state = DMX_STATE_IDLE;
				DMX_SERCOM->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
				DMX_SERCOM->USART.INTENSET.reg = SERCOM_USART_INTENSET_TXC;
				// switch to wait for last byte transmission to complete
			}
		}
		
	}
}

//************************************************************************************

void LXSAMD21DMX::printReceivedData( void ) {
	for(int j=0; j<_next_read_slot; j++) {
		Serial.println(_receivedData[j]);
	}
}

void LXSAMD21DMX::packetComplete( void ) {
	if ( _receivedData[0] == 0 ) {				//zero start code is DMX
		if ( _rdm_read_handled == 0 ) {			// not handled by specific method
			if ( _next_read_slot > DMX_MIN_SLOTS ) {
				_slots = _next_read_slot - 1;				//_next_read_slot represents next slot so subtract one
				for(int j=0; j<_next_read_slot; j++) {	//copy dmx values from read buffer
					_dmxData[j] = _receivedData[j];
				}
	
				if ( _receive_callback != NULL ) {
					_receive_callback(_slots);
				}
			}
		}
	} else {
		if ( _receivedData[0] == RDM_START_CODE ) {			//zero start code is RDM
			if ( _rdm_read_handled == 0 ) {					// not handled by specific method
				if ( validateRDMPacket(_receivedData) ) {	// evaluate checksum
					uint8_t plen = _receivedData[2] + 2;
					for(int j=0; j<plen; j++) {
						_rdmData[j] = _receivedData[j];
					}
					if ( _receive_callback != NULL ) {
						_rdm_receive_callback(plen);
					}
				}
			}
		} else {
#if defined LXSAMD21DMX_DEBUG
			Serial.println("________________ unknown data packet ________________");
			printReceivedData();
#endif
		}
	}
	resetFrame();
}

void LXSAMD21DMX::resetFrame( void ) {
	_dmx_read_state = DMX_READ_STATE_IDLE;						// insure wait for next break
	//_dmx_send_state????
}

void LXSAMD21DMX::breakReceived( void ) {
	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {	// break has already been detected
		if ( _next_read_slot > 1 ) {						// break before end of maximum frame
			if ( _receivedData[0] == 0 ) {				// zero start code is DMX
				packetComplete();						// packet terminated with slots<512
			}
		}
	}
	_dmx_read_state = DMX_READ_STATE_START;		//break causes spurious 0 byte on next interrupt, ignore...
	_next_read_slot = 0;
	_packet_length = DMX_MAX_FRAME;						// default to receive complete frame
}

void LXSAMD21DMX::byteReceived(uint8_t c) {

	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {
	//digitalWrite(6, LOW);//<- debug pin
		_receivedData[_next_read_slot] = c;
		if ( _next_read_slot == 2 ) {						//RDM length slot
			if ( _receivedData[0] == RDM_START_CODE ) {			//RDM start code
				if ( _rdm_read_handled == 0 ) {
					_packet_length = c + 2;				//add two bytes for checksum
				}
			} else if ( _receivedData[0] == 0xFE ) {	//RDM Discovery Response
				_packet_length = DMX_MAX_FRAME;
			} else if ( _receivedData[0] != 0 ) {		// if Not Null Start Code
				_dmx_read_state = DMX_STATE_IDLE;			//unrecognized, ignore packet
			}
		}
	
		_next_read_slot++;
		if ( _next_read_slot >= _packet_length ) {		//reached expected end of packet
			packetComplete();
		}
		//digitalWrite(6, HIGH);//<- debug pin
	} else if ( _dmx_read_state == DMX_READ_STATE_START ) {
		_dmx_read_state = DMX_READ_STATE_RECEIVING;
	}
}

void LXSAMD21DMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}

/************************************ RDM Methods **************************************/

void LXSAMD21DMX::setRDMReceivedCallback(LXRecvCallback callback) {
	_rdm_receive_callback = callback;
}


void LXSAMD21DMX::outputIRQHandler(void) {
    if ( DMX_SERCOM->USART.INTFLAG.bit.TXC ) {
    	transmissionComplete();
    } else if ( DMX_SERCOM->USART.INTFLAG.bit.DRE ) {
        dataRegisterEmpty();
    }
}

void LXSAMD21DMX::inputIRQHandler(void) {
	
		if ( DMX_SERCOM->USART.INTFLAG.bit.ERROR ) {
		   DMX_SERCOM->USART.INTFLAG.bit.ERROR = 1;		//acknowledge error, clear interrupt
		   
			if ( DMX_SERCOM->USART.STATUS.bit.FERR ) {	//framing error happens when break is sent
				breakReceived();
				return;
			}
			// other error flags?
			//return;
		}	//ERR
	
		if ( DMX_SERCOM->USART.INTFLAG.bit.RXC ) {
			uint8_t incoming_byte = DMX_SERCOM->USART.DATA.reg;				// read buffer to clear interrupt flag
			byteReceived(incoming_byte);
		} // RXC
	
}		  // <-inputIRQHandler(void)

void LXSAMD21DMX::rdmIRQHandler(void) {
	if ( _rdm_task_mode ) {
		if ( DMX_SERCOM->USART.INTFLAG.bit.TXC ) {
			transmissionComplete();
		} else if ( DMX_SERCOM->USART.INTFLAG.bit.DRE ) {
			dataRegisterEmpty();
		}
	} else {
		inputIRQHandler();
	}
}		  //--IrqHandler(void)

/*********************************** RDM *****************************************/

uint8_t LXSAMD21DMX::rdmTaskMode( void ) {		// applies to bidirectional RDM connection
	return _rdm_task_mode;
}

void LXSAMD21DMX::setTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	 _rdm_task_mode = DMX_TASK_SEND;
}


void LXSAMD21DMX::restoreTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	_dmx_send_state = DMX_STATE_BREAK;
	_rdm_task_mode = DMX_TASK_SET_SEND;
	 transmissionComplete();		// sends break and sets interrupt
	 delay(1);
	 while ( _rdm_task_mode != DMX_TASK_SEND ) {
	 	delay(1);
	 }
}

void LXSAMD21DMX::setTaskReceive( void ) {		// only valid if connection started using startRDM()
	_next_read_slot = 0;
	_packet_length = DMX_MAX_FRAME;
    _dmx_send_state = DMX_STATE_IDLE;
    _rdm_task_mode = DMX_TASK_RECEIVE;
    _rdm_read_handled = 0;
    
    
    digitalWrite(_direction_pin, LOW);
}

void LXSAMD21DMX::sendRawRDMPacket( uint8_t len ) {		// only valid if connection started using startRDM()
	_rdm_len = len;
	// calculate checksum:  len should include 2 bytes for checksum at the end
	uint16_t checksum = rdmChecksum(_rdmPacket, _rdm_len-2);
	_rdmPacket[_rdm_len-2] = checksum >> 8;
	_rdmPacket[_rdm_len-1] = checksum & 0xFF;

	if ( _rdm_task_mode ) {						//already sending, flag to send RDM
		_rdm_task_mode = DMX_TASK_SET_SEND_RDM;
	} else {
		_rdm_task_mode = DMX_TASK_SEND_RDM;
		digitalWrite(_direction_pin, HIGH);
		_dmx_send_state = DMX_STATE_BREAK;
		 //set the interrupts
		DMX_SERCOM->USART.INTENSET.reg =  SERCOM_USART_INTENSET_TXC | SERCOM_USART_INTENSET_ERROR;
	}
	
	while ( _rdm_task_mode ) {	//wait for packet to be sent and listening to start
		delay(2);				//_rdm_task_mode is set to 0 (receive) after RDM packet is completely sent
	}
}

void  LXSAMD21DMX::setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice) {
	pdata[RDM_IDX_START_CODE]		= RDM_START_CODE;
  	pdata[RDM_IDX_SUB_START_CODE]	= RDM_SUB_START_CODE;
  	pdata[RDM_IDX_PACKET_SIZE]		= msglen;
  	
  	// must set target UID outside this method
  	UID::copyFromUID(THIS_DEVICE_ID, pdata, RDM_IDX_SOURCE_UID);
  	
  	pdata[RDM_IDX_TRANSACTION_NUM]	= _transaction++;
  	pdata[RDM_IDX_PORT]				= port;
  	pdata[RDM_IDX_MSG_COUNT]		= 0x00;		//(always zero for controller msgs)
  	pdata[RDM_IDX_SUB_DEV_MSB] 		= subdevice >> 8;
  	pdata[RDM_IDX_SUB_DEV_LSB] 		= subdevice & 0xFF;
  	// total always 20 bytes
}

void  LXSAMD21DMX::setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl) {
	pdata[RDM_IDX_CMD_CLASS] 		= cmdclass;
  	pdata[RDM_IDX_PID_MSB] 			= (pid >> 8) & 0xFF;
  	pdata[RDM_IDX_PID_LSB]			 = pid & 0xFF;
  	pdata[RDM_IDX_PARAM_DATA_LEN] 	= pdl;
  	// total always 4 bytes
}

uint8_t LXSAMD21DMX::sendRDMDiscoveryPacket(UID lower, UID upper, UID* single) {
	uint8_t rv = RDM_NO_DISCOVERY;
	uint8_t j;
	
	//Build RDM packet
	setupRDMControllerPacket(_rdmPacket, RDM_DISC_UNIQUE_BRANCH_MSGL, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(BROADCAST_ALL_DEVICES_ID, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, RDM_DISC_UNIQUE_BRANCH, RDM_DISC_UNIQUE_BRANCH_PDL);
  	UID::copyFromUID(lower, _rdmPacket, 24);
  	UID::copyFromUID(upper, _rdmPacket, 30);
	
	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_DISC_UNIQUE_BRANCH_PKTL);
	delay(2);

	// any bytes read indicate response to discovery packet
	// check if a single, complete, uncorrupted packet has been received
	// otherwise, refine discovery search
	
	if ( _next_read_slot ) {
		rv = RDM_PARTIAL_DISCOVERY;
		
		// find preamble separator
		for(j=0; j<8; j++) {
			if ( _receivedData[j] == RDM_DISC_PREAMBLE_SEPARATOR ) {
				break;
			}
		}
		// 0-7 bytes preamble
		if ( j < 8 ) {
			if ( _next_read_slot == j + 17 ) { //preamble separator plus 16 byte payload
				uint8_t bindex = j + 1;
				
				//calculate checksum of 12 slots representing UID
				uint16_t checksum = rdmChecksum(&_receivedData[bindex], 12);
				
				//convert dual bytes to payload of single bytes
				uint8_t payload[8];
				for (j=0; j<8; j++) {
					payload[j] = _receivedData[bindex] & _receivedData[bindex+1];
					bindex += 2;
				}

				if ( testRDMChecksum( checksum, payload, 6 ) ) {
					//copy UID into uldata
					rv = RDM_DID_DISCOVER;
					*single = payload;
				}
			}
		}			// j<8
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}

	restoreTaskSendDMX();
	return rv;
}

uint8_t LXSAMD21DMX::sendRDMDiscoveryMute(UID target, uint8_t cmd) {
	uint8_t rv = 0;

	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, cmd, 0x00);

	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_PKT_BASE_TOTAL_LEN);
	delay(3);
	
	if ( _next_read_slot >= (RDM_PKT_BASE_TOTAL_LEN+2) ) {				//expected pdl 2 or 8
		if ( validateRDMPacket(_receivedData) ) {
			if ( _receivedData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
				if ( _receivedData[RDM_IDX_CMD_CLASS] == RDM_DISC_COMMAND_RESPONSE ) {
					if ( THIS_DEVICE_ID == UID(&_receivedData[RDM_IDX_DESTINATION_UID]) ) {
						rv = 1;
					}
				}
			}
		}
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	restoreTaskSendDMX();
	return rv;
}

uint8_t LXSAMD21DMX::sendRDMControllerPacket( void ) {
	uint8_t rv = 0;
	_rdm_read_handled = 1;
	sendRawRDMPacket(_rdmPacket[2]+2);
	delay(3);
	
	if ( _next_read_slot > 0 ) {
		if ( validateRDMPacket(_receivedData) ) {
			uint8_t plen = _receivedData[2] + 2;
			for(int rv=0; rv<plen; rv++) {
				_rdmData[rv] = _receivedData[rv];
			}
			rv = 1;
		}
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LXSAMD21DMX::sendRDMControllerPacket( uint8_t* bytes, uint8_t len ) {
	for (uint8_t j=0; j<len; j++) {
		_rdmPacket[j] = bytes[j];
	}
	return sendRDMControllerPacket();
}

uint8_t LXSAMD21DMX::sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND, pid, 0x00);
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_GET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
					for(int j=0; j<len; j++) {
						info[j] = _rdmData[24+j];
					}
				}
			}
		}
	}
	
	return rv;
}

uint8_t LXSAMD21DMX::sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 1 byte parameter is 25 (+cksum =27 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN+len, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_SET_COMMAND, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_SET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
				}
			}
		}
	}
	
	return rv;
}
