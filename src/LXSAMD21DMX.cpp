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

//**************************************************************************************

LXSAMD21DMX SAMD21DMX;

// **************************** SERCOMn_Handler  ***************

void SERCOM4_Handler()
{
  SAMD21DMX.IrqHandler();
}


// ***************** define registers, flags and interrupts  ****************

	//***** baud rate defines
    #define DMX_DATA_BAUD		250000
    #define DMX_BREAK_BAUD 	90000
    //99900

    //***** states indicate current position in DMX stream
    #define DMX_STATE_BREAK 0
    #define DMX_STATE_START 1
    #define DMX_STATE_DATA 2
    #define DMX_STATE_IDLE 3

	//***** status is if interrupts are enabled and IO is active
    #define ISR_DISABLED 			0
    #define ISR_OUTPUT_ENABLED 	1
    #define ISR_INPUT_ENABLED 	2

// **************************** global data (can be accessed in ISR)  ***************

Uart SerialDMX (&DMX_sercom, PIN_DMX_RX, PIN_DMX_TX, PAD_DMX_RX, PAD_DMX_TX);

uint8_t*  _shared_dmx_data;
uint8_t   _shared_dmx_state;
uint16_t  _shared_dmx_slot;
uint16_t  _shared_max_slots = DMX_MIN_SLOTS;
LXRecvCallback _shared_receive_callback = NULL;


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
	_shared_max_slots = DMX_MAX_SLOTS;
	_interrupt_status = ISR_DISABLED;
	
	//zero buffer including _dmxData[0] which is start code
    memset(_dmxData, 0, DMX_MAX_SLOTS+1);
}
    
    

LXSAMD21DMX::~LXSAMD21DMX ( void ) {
    stop();
    _shared_dmx_data = NULL;
    _shared_receive_callback = NULL;
}


void LXSAMD21DMX::startOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}

	if ( _interrupt_status == ISR_INPUT_ENABLED ) {
		stop();
	}
	
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
	  SerialDMX.begin(DMX_BREAK_BAUD, (uint8_t)SERIAL_8N2);
  
	  // Assign pin mux to SERCOM functionality (must come after SerialDMX.begin)
	  pinPeripheral(PIN_DMX_RX, PIO_SERCOM_ALT);
	  pinPeripheral(PIN_DMX_TX, PIO_SERCOM_ALT);
	  
	  _interrupt_status = ISR_OUTPUT_ENABLED;
	  _shared_dmx_data = dmxData();
	  _shared_dmx_slot = 0;              
	  _shared_dmx_state = DMX_STATE_START;

	  DMX_SERCOM->USART.INTENSET.reg =  SERCOM_USART_INTENSET_TXC | SERCOM_USART_INTENSET_ERROR;
	  DMX_SERCOM->USART.DATA.reg = 0;

	}
}

void LXSAMD21DMX::startInput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		SerialDMX.begin(DMX_DATA_BAUD, (uint8_t)SERIAL_8N2);
  
	   // Assign pin mux to SERCOM functionality (must come after SerialDMX.begin)
	   pinPeripheral(PIN_DMX_RX, PIO_SERCOM_ALT);
	   pinPeripheral(PIN_DMX_TX, PIO_SERCOM_ALT);
	   
		_shared_dmx_data = dmxData();
		_shared_dmx_slot = 0;              
		_shared_dmx_state = DMX_STATE_IDLE;

		_interrupt_status = ISR_INPUT_ENABLED;
	}
}

void LXSAMD21DMX::stop ( void ) {
   SerialDMX.end();
	_interrupt_status = ISR_DISABLED;
}

void LXSAMD21DMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

void LXSAMD21DMX::setMaxSlots (int slots) {
	if ( slots > DMX_MIN_SLOTS ) {
		_shared_max_slots = slots;
	} else {
		_shared_max_slots = DMX_MIN_SLOTS;
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

void LXSAMD21DMX::setDataReceivedCallback(LXRecvCallback callback) {
	_shared_receive_callback = callback;
}


void LXSAMD21DMX::IrqHandler(void) {

  if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {

    if ( DMX_SERCOM->USART.INTFLAG.bit.TXC ) {
      if ( DMX_SERCOM->USART.INTFLAG.bit.DRE ) {

        if ( _shared_dmx_state == DMX_STATE_DATA ) {
          DMX_SERCOM->USART.DATA.reg = _shared_dmx_data[_shared_dmx_slot++];	//send next slot;
          if ( _shared_dmx_slot > _shared_max_slots ) {
            _shared_dmx_state = DMX_STATE_BREAK;
          }
        } else if ( _shared_dmx_state == DMX_STATE_BREAK ) {
          setBaudRate(DMX_BREAK_BAUD);
          _shared_dmx_state = DMX_STATE_START;
          _shared_dmx_slot = 0;
          DMX_SERCOM->USART.DATA.reg = 0;	//break
        } else if ( _shared_dmx_state == DMX_STATE_START ) {
          setBaudRate(DMX_DATA_BAUD);
          _shared_dmx_state = DMX_STATE_DATA;
          DMX_SERCOM->USART.DATA.reg = _shared_dmx_data[_shared_dmx_slot++];
        }
      
      }  //DRE
    }    //TXC
    
  } else if ( _interrupt_status == ISR_INPUT_ENABLED ) {
	
		if ( DMX_SERCOM->USART.INTFLAG.bit.ERROR ) {
		   DMX_SERCOM->USART.INTFLAG.bit.ERROR = 1;		//acknowledge error, clear interrupt
		   
			if ( DMX_SERCOM->USART.STATUS.bit.FERR ) {	//framing error happens when break is sent
				_shared_dmx_state = DMX_STATE_BREAK;
				if ( _shared_dmx_slot > 0 ) {
					if ( _shared_receive_callback != NULL ) {
						_shared_receive_callback(_shared_dmx_slot);
					}
				}
				_shared_dmx_slot = 0;
			  return;
			}
			// other error flags?
			//return;?
		}	//ERR
	
		if ( DMX_SERCOM->USART.INTFLAG.bit.RXC ) {
		uint8_t incoming_byte = DMX_SERCOM->USART.DATA.reg;				// read buffer to clear interrupt flag
			switch ( _shared_dmx_state ) {
				case DMX_STATE_BREAK:
					if ( incoming_byte == 0 ) {									// start code == zero (DMX)
						_shared_dmx_data[_shared_dmx_slot] = incoming_byte;
						_shared_dmx_state = DMX_STATE_DATA;
					} else {
						_shared_dmx_state = DMX_STATE_IDLE;
					}
					break;
			
				case DMX_STATE_DATA:
					_shared_dmx_data[_shared_dmx_slot++] = incoming_byte;	// increments BEFORE assignment
					if ( _shared_dmx_slot > DMX_MAX_SLOTS ) {
						_shared_dmx_state = DMX_STATE_IDLE;						// go to idle, wait for next break
					}
					break;
			}
		} // RXC
    
  }	  // input enabled
	
}		  //--IrqHandler(void)
