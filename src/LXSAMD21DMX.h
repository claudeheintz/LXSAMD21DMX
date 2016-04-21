/* LXSAMD21DMX.h
   Copyright 2016 by Claude Heintz Design
   All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of LXSAMD21DMX nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------------------

   The LXSAMD21DMX library supports output and input of DMX using
   sercom4 of a MKR1000 microcontroller in UART mode. (pins 7 & 8)
   
   This is the circuit for a simple unisolated DMX Shield
   that could be used with LXSAMD21DMX.  It uses a line driver IC
   to convert the output from the Teensy to DMX:

 MKR1000 Pin
 |                         SN 75176 A or MAX 481CPA
 V                            _______________
       |                      | 1      Vcc 8 |------(+5v)
RX (5) |----------------------|              |                 DMX Output
       |                 +----| 2        B 7 |---------------- Pin 2
       |                 |    |              |
   (3) |----------------------| 3 DE     A 6 |---------------- Pin 3
       |                      |              |
TX (4) |----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
       |                                         |
       |                                       (GND)

       Data Enable (DE) and Inverted Read Enable (!RE) can be wired to +5v for output or Gnd for input
       if direction switching is not needed.
*/

#ifndef LXSAM21_DMX_H
#define LXSAM21_DMX_H

#include <inttypes.h>
#include "SERCOM.h"

#define DMX_MIN_SLOTS 24
#define DMX_MAX_SLOTS 512

#define DIRECTION_PIN_NOT_USED 255

typedef void (*LXRecvCallback)(int);

/*!   
@class LXSAMD21DMX
@abstract
   LXTeensyDMX is a driver for sending or receiving DMX using a Teensy 3.1/3.2's
   UART0 RX pin 0, TX pin 1
   
   LXTeensyDMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXTeensyDMX is used with a single instance called Teensy3DMX	.
*/

class LXSAMD21DMX  {

  public:
  
	LXSAMD21DMX  ( void );
   ~LXSAMD21DMX ( void );
    
   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission (TE) and tx interrupts (TIE/TCIE).
   */
   void startOutput( void );
   
   /*!
    * @brief starts interrupt that continuously reads DMX data
    * @discussion sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables receive (RE) and rx interrupt (RIE)
   */
   void startInput( void );
   
   /*!
    * @brief disables tx, rx and interrupts.
   */
	void stop( void );
	
	/*!
	 * @brief optional utility sets the pin used to control driver chip's
	 *        DE (data enable) line, HIGH for output, LOW for input.     
    * @param pin to be automatically set for input/output direction
    */
   void setDirectionPin( uint8_t pin );
	
	/*!
	 * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
	 * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.  
	 *             The DMX standard specifies min break to break time no less than 1024 usecs.  
	 *             At 44 usecs per slot ~= 24
	 * @param slot the highest slot number (~24 to 512)
	*/
	void setMaxSlots (int slot);
	
	/*!
    * @brief reads the value of a slot/address/channel
    * @discussion NOTE: Data is not double buffered.  
    *                   So a complete single frame is not guaranteed.  
    *                   The ISR continuously reads the next frame into the buffer
    * @return level (0-255)
   */
   uint8_t getSlot (int slot);
   
	/*!
	 * @brief Sets the output value of a slot
	 * @param slot number of the slot/address/channel (1-512)
	 * @param value level (0-255)
	*/
   void setSlot (int slot, uint8_t value);
   
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx array
   */
   uint8_t* dmxData(void);
   
   /*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on the break after a DMX frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
   */
   void setDataReceivedCallback(LXRecvCallback callback);
   
   /*!
    * @brief interrupt handler function
   */
   void IrqHandler();
    
  private:
   /*!
    * @brief Indicates mode ISR_OUTPUT_ENABLED or ISR_INPUT_ENABLED or ISR_DISABLED
   */
  	uint8_t  _interrupt_status;
  	
  	/*!
   * @brief pin used to control direction of output driver chip
   */
  	uint8_t _direction_pin;
  	
  	/*!
    * @brief Array of dmx data including start code
   */
  	uint8_t  _dmxData[DMX_MAX_SLOTS+1];
  	
};

extern LXSAMD21DMX SAMD21DMX;

/**************************
   Change the following lines to use other than SERCOM4.
   The following table shows the MKR1000 default uses of the SERCOMs
   and their associated pins:
   
   SERCOM0	Wire		11	12
   SERCOM1  SPI		8	9	10
   SERCOM2	ATWINC1501B SPI
   SERCOM3	
   SERCOM4
   SERCOM5	Serial1	13	14
   
   Changing the SERCOM also requires changing the pin MUX.
   The following are MUX options for SERCOM3 and SERCOM4
   Mkr1000 pins

	Sercom 3
	00 S/0
	01 S/1
	06 SA/2
	07 SA/3
	08 SA/0
	09 SA/1
	10 SA/3

	Sercom 4
	04 SA/2
	05 SA/3

	see datasheet pg 33 for sercom4 mux options
	Need to be PB10 PB11, pad 2, pad 3  pad3=rx=3, pad 2=tx=1
	which correspond to MKR10000 pins 4 & 5.
	
	In general, tx is either pad 0 = CRegA_TXPO 0x0 or pad 2 = CRegA_TXPO 0x1
	see datasheet pg 481 for control register A

	NOTE:  You MUST change the name of the SERCOMn_Handler() function in LXSAMD21DMX.cpp
	       if you use a different SERCOM
*/

#define PIN_DMX_RX (5ul)
#define PIN_DMX_TX (4ul)
#define PAD_DMX_RX SERCOM_RX_PAD_3
#define PAD_DMX_TX UART_TX_PAD_2

// SERCOMn is pointer to memory address where SERCOM registers are located.
#define DMX_SERCOM SERCOM4
// sercomN is C++ wrapper for SERCOMn (passed to UART constructor)
#define DMX_sercom sercom4

#endif // ifndef LXSAM21_DMX_H
