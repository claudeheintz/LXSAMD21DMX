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
   to convert the output from the SAM D-21 to DMX:

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
#include <rdm/UID.h>

#define DMX_MIN_SLOTS 24
#define RDM_MAX_FRAME 257
#define DMX_MAX_SLOTS 512
#define DMX_MAX_FRAME 513

#define DIRECTION_PIN_NOT_USED 255

//***** baud rate defines
#define DMX_DATA_BAUD	250000
#define DMX_BREAK_BAUD 	90000
//99900

//***** states indicate current position in DMX stream
#define DMX_STATE_BREAK 0
#define DMX_STATE_START 1
#define DMX_STATE_DATA 2
#define DMX_STATE_IDLE 3

//***** status is if interrupts are enabled and IO is active
#define ISR_DISABLED 		0
#define ISR_OUTPUT_ENABLED 	1
#define ISR_INPUT_ENABLED 	2
#define ISR_RDM_ENABLED 	3

//***** states indicate current position in DMX stream
#define DMX_READ_STATE_IDLE 0
#define DMX_READ_STATE_RECEIVING 1
#define DMX_READ_STATE_START	 2

#define DMX_TASK_RECEIVE		0   
#define DMX_TASK_SEND			1
#define DMX_TASK_SEND_RDM		2
#define DMX_TASK_SET_SEND		3
#define DMX_TASK_SET_SEND_RDM	4

#define RDM_NO_DISCOVERY		0
#define RDM_PARTIAL_DISCOVERY	1
#define RDM_DID_DISCOVER		2

#define RDM_DIRECTION_INPUT		0
#define RDM_DIRECTION_OUTPUT	1

typedef void (*LXRecvCallback)(int);

/*!   
@class LXSAMD21DMX
@abstract
   LXSAMD21DMX is a driver for sending or receiving DMX using one of a SAM D-21's five serial peripheral interfaces (SERCOMs).
   
   LXSAMD21DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXSAMD21DMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXSAMD21DMX is used with a single instance called SAMD21DMX	.
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
    * @brief starts interrupt that continuously sends DMX output
    * @discussion  direction pin is required, calls startOutput
   */
   
   void startRDM( uint8_t pin, uint8_t direction=1);
   
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
      
	uint8_t* rdmData( void );

	uint8_t* receivedData( void );

	uint8_t* receivedRDMData( void );
	
	/*!
    * @brief called when last data byte and break are completely sent
   */
	void transmissionComplete( void );
	
	/*!
    * @brief called when data register is empty and ready for the next byte
   */
	void dataRegisterEmpty( void );
   
   /*!
    * @brief utility for debugging prints received data
   */
   void printReceivedData( void );
   
   /*!
    * @brief called when a packet is finished being received either through start of next packet or size reached
   */
   void packetComplete( void );
   
   /*!
    * @brief sets read state to wait for break
   */
   void resetFrame( void );
   
   /*!
    * @brief called when a break is detected
   */
  	void breakReceived( void );
  	
  	/*!
    * @brief called from isr when a byte is read from register
   */
  	void byteReceived(uint8_t c);
   
   /*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on the break after a DMX frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
   */
   void setDataReceivedCallback(LXRecvCallback callback);
   
   /************************************ RDM Methods ***********************************/
   
   /*!
    * @brief Function called when RDM frame has been read
    * @discussion Sets a pointer to a function that is called
    *             after an RDM frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
    */
   void setRDMReceivedCallback(LXRecvCallback callback);
   
   /*!
    * @brief interrupt handler functions
   */
   void outputIRQHandler();
   void inputIRQHandler();
   void rdmIRQHandler();
   
      	/*!
    * @brief indicate if dmx frame should be sent by bi-directional task loop
    * @discussion should only be called by task loop
    * @return 1 if dmx frame should be sent
    *  return 2 if RDM should be sent
    *  return 3 if RDM should be sent and set mode to 1 after first frame finished
    */
	uint8_t rdmTaskMode( void );
	
	
	/*!
    * @brief sets rdm task to send mode and the direction pin to HIGH
	*/
	void setTaskSendDMX( void );
	
	/*!
    * @brief sets rdm task to send mode after task mode loops.
    *        Sent after sending RDM message so DMX is resumed.
    *        Blocks until task loop sets mode to send.
	*/
	void restoreTaskSendDMX( void );
	
	/*!
    * @brief sets rdm task to receive mode
    *        Prepares variables to receive starting with next break.
    *        Sets the direction pin to LOW.
	*/
	void setTaskReceive( void );
	
	/*!
    * @brief length of the rdm packet awaiting being sent
	*/
	uint8_t rdmPacketLength( void );
	
	/*!
    * @brief sends packet using bytes from _rdmPacket ( rdmData() )
    * @discussion sets rdm task mode to DMX_TASK_SEND_RDM which causes
    *             _rdmPacket to be sent on next opportunity from task loop.
    *             after _rdmPacket is sent, task mode switches to listen for response.
    *
    *             set _rdm_read_handled flag prior to calling sendRawRDMPacket
    *             _rdm_read_handled = 1 if reading is handled by calling function
    *             _rdm_read_handled = 0 if desired to resume passive listening for next break
    */
	void sendRawRDMPacket( uint8_t len );
	
	/*!
    * @brief convenience method for setting fields in the top 20 bytes of an RDM message
    *        that will be sent.
    *        Destination UID needs to be set outside this method.
    *        Source UID is set to constant THIS_DEVICE_ID below.
	*/
	void setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice);
	
	/*!
    * @brief convenience method for setting fields in the top bytes 20-23 of an RDM message
    *        that will be sent.
	*/
	void setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl);
	
	/*!
    * @brief send discovery packet using upper and lower bounds
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if discovered, 2 if valid packet (UID stored in uldata[12-17])
    */
    uint8_t sendRDMDiscoveryPacket(UID lower, UID upper, UID* single);
    
    /*!
    * @brief send discovery mute/un-mute packet to target UID
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack response is received.
    */
    uint8_t sendRDMDiscoveryMute(UID target, uint8_t cmd);
    
    /*!
    * @brief send previously built packet in _rdmPacket and validate response
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( void );
    
    /*!
    * @brief copies len of bytes into _rdmPacket and sends it
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( uint8_t* bytes, uint8_t len );
    
    /*!
    * @brief send RDM_GET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
    /*!
    * @brief send RDM_SET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
    static UID THIS_DEVICE_ID;

    
  private:
  	
  	/*!
   * @brief pin used to control direction of output driver chip
   */
  	uint8_t _direction_pin;
  	
  	/*!
	 * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
	 */
  	uint8_t  _dmx_send_state;
  	
	/*!
	 * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
	 */
  	uint8_t  _dmx_read_state;
  	
  		/*!
	 * @brief flag indicating RDM task should send dmx slots
	 */
  	uint8_t  _rdm_task_mode;
  	
	/*!
	 * @brief flag indicating RDM task should send dmx slots
	 */
  	uint8_t  _rdm_read_handled;
  	
  	/*!
	 * @brief transaction number
	 */
  	uint8_t _transaction;
  	
  	/*!
	 * @brief maximum expected length of packet
	 */
  	uint16_t  _packet_length;
  	
	/*!
	 * @brief slot index indicating position of byte to be sent
	 */
  	uint16_t  _next_send_slot;
  	
	/*!
	 * @brief slot index indicating position of last byte received
	 */
  	uint16_t  _next_read_slot;
  	
	/*!
	 * @brief number of dmx slots ~24 to 512
	 */
  	uint16_t  _slots;
  	
	/*!
	 * @brief outgoing rdm packet length
	 */
	uint16_t  _rdm_len;
  	
	/*!
	 * @brief Array of dmx data including start code
	 */
  	uint8_t  _dmxData[DMX_MAX_FRAME];
  	
	/*!
	 * @brief Array of received bytes first byte is start code
	 */
  	uint8_t  _receivedData[DMX_MAX_FRAME];
  	
  	/*!
	 * @brief Array representing an rdm packet to be sent
	 */
	uint8_t  _rdmPacket[RDM_MAX_FRAME];
	
	/*!
	 * @brief Array representing a received rdm packet
	 */
	uint8_t  _rdmData[RDM_MAX_FRAME];
  	
   /*!
    * @brief Pointer to receive callback function
	*/
  	LXRecvCallback _receive_callback;
  	
   /*!
    * @brief Pointer to receive callback function
    */
  	LXRecvCallback _rdm_receive_callback;
  	
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
   
	sercom 2
	03 SA/1 SERCOM_RX_PAD_1
	04 SA/0	UART_TX_PAD_0

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


/******************* alternate SERCOM and pins *******************
 *
 *  uncomment line 521 and define use_optional_sercom_macros
 *  to use alternate SERCOM and pins.
 *
 *  This table shows the possible configurations
 *
 *  use_optional_sercom_macros  TX Pin  RX Pin    SERCOM
 *  --------------------------------------------------
 *     		undefined          |   4   |   5   |  SERCOM4
 *     		    1              |   4   |   3   |  SERCOM2
 *     		    2              |   10  |   11  |  SERCOM1
 *
 *****************************************************************/

// #define use_optional_sercom_macros 3

#if defined( use_optional_sercom_macros )

#if ( use_optional_sercom_macros == 1 )
	//********************** optional sercom macros 1 ********************** 
	// --might be used for Arduino Zero sercom2 pins 3 and 4

	#define PIN_DMX_RX (3ul)
	#define PIN_DMX_TX (4ul)
	#define PAD_DMX_RX SERCOM_RX_PAD_1
	#define PAD_DMX_TX UART_TX_PAD_0

	// Set to PIO_SERCOM or PIO_SERCOM_ALT
	#define MUX_DMX_RX PIO_SERCOM_ALT
	#define MUX_DMX_TX PIO_SERCOM_ALT

	// SERCOMn is pointer to memory address where SERCOM registers are located.
	#define DMX_SERCOM SERCOM2

	// sercomN is C++ wrapper for SERCOMn (passed to UART constructor)
	#define DMX_sercom sercom2

	// sercom handler function
	#define DMX_SERCOM_HANDLER_FUNC SERCOM2_Handler
#elif ( use_optional_sercom_macros == 2 )
        //********************** optional sercom macros 2 ********************** 
	// --might be used for Adafruit M0 Feather sercom1 pins 10 and 11

#define PIN_DMX_RX (11ul)
	#define PIN_DMX_TX (10ul)
	#define PAD_DMX_RX SERCOM_RX_PAD_0
	#define PAD_DMX_TX UART_TX_PAD_2

	// Set to PIO_SERCOM or PIO_SERCOM_ALT
	#define MUX_DMX_RX PIO_SERCOM
	#define MUX_DMX_TX PIO_SERCOM

	// SERCOMn is pointer to memory address where SERCOM registers are located.
	#define DMX_SERCOM SERCOM1

	// sercomN is C++ wrapper for SERCOMn (passed to UART constructor)
	#define DMX_sercom sercom1

	// sercom handler function
	#define DMX_SERCOM_HANDLER_FUNC SERCOM1_Handler

#elif ( use_optional_sercom_macros == 3 )
	//********************** optional sercom macros 3 ********************** 
	// --might be used for Feather M0 pins D0 and D1

	#define PIN_DMX_RX (0ul)
	#define PIN_DMX_TX (1ul)
	#define PAD_DMX_RX SERCOM_RX_PAD_3
	#define PAD_DMX_TX UART_TX_PAD_2

	// Set to PIO_SERCOM or PIO_SERCOM_ALT
	#define MUX_DMX_RX PIO_SERCOM_ALT
	#define MUX_DMX_TX PIO_SERCOM_ALT

	// SERCOMn is pointer to memory address where SERCOM registers are located.
	#define DMX_SERCOM SERCOM2

	// sercomN is C++ wrapper for SERCOMn (passed to UART constructor)
	#define DMX_sercom sercom2

	// sercom handler function
	#define DMX_SERCOM_HANDLER_FUNC SERCOM2_Handler


#endif
	
#else
	//********************** default sercom macros ********************** 
        #define PIN_DMX_RX (5ul)
	#define PIN_DMX_TX (4ul)
	#define PAD_DMX_RX SERCOM_RX_PAD_3
	#define PAD_DMX_TX UART_TX_PAD_2

	// Set to PIO_SERCOM or PIO_SERCOM_ALT
	#define MUX_DMX_RX PIO_SERCOM_ALT
	#define MUX_DMX_TX PIO_SERCOM_ALT

	// SERCOMn is pointer to memory address where SERCOM registers are located.
	#define DMX_SERCOM SERCOM4

	// sercomN is C++ wrapper for SERCOMn (passed to UART constructor)
	#define DMX_sercom sercom4

	// sercom handler function
	#define DMX_SERCOM_HANDLER_FUNC SERCOM4_Handler

#endif

#endif // ifndef LXSAM21_DMX_H
