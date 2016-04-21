/* LXENTTECSerial.h
   Copyright 2015 by Claude Heintz Design
   This code is in the public domain
*/

#ifndef LXENTTECSerial_H
#define LXENTTECSerial_H

#include <Arduino.h>
#include <inttypes.h>

#define DMX_MIN 25
#define DMX_MAX 513

#define ENTTEC_LABEL_NONE         0
#define ENTTEC_LABEL_GET_INFO     3
#define ENTTEC_LABEL_RECEIVED_DMX 5
#define ENTTEC_LABEL_SEND_DMX     6
#define ENTTEC_LABEL_RECEIVE_DMX  8
#define ENTTEC_LABEL_GET_SERIAL   10

class LXENTTECSerial {

  public:
	LXENTTECSerial  ( void );
   ~LXENTTECSerial ( void );
   
   int  numberOfSlots    ( void );
   void setNumberOfSlots ( int n );
   uint8_t  getSlot      ( int slot );
   void     setSlot      ( int slot, uint8_t value );
   uint8_t  startCode    ( void );
   void  setStartCode    ( uint8_t value );
   uint8_t* dmxData      ( void );
   
   uint8_t readPacket     ( void );
   void    writeDMXPacket ( void );
   void    writeDMXPacket ( uint8_t *buffer, int length );
   void    writeInfo      ( uint16_t length );
   void    writeSerialNumber ( uint32_t sn );
    
  private:
  	uint8_t  _dmx_data[DMX_MAX];
  	int      _dmx_slots;
  	    
  	uint8_t  getNextByte( void );
};

#endif // ifndef LXENTTECSerial_H