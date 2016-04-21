/**************************************************************************/
/*!
    @file     DMXUSBSerial.ino
    @author   Claude Heintz
    @license  BSD (see SAMD21DMX.h or http://lx.claudeheintzdesign.com/opensource.html)
    @copyright 2016 by Claude Heintz

    This sketch allows a MKR1000 board to emulate parts of an ENTTEC DMX USB Pro's functions.
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/

#include <LXSAMD21DMX.h>
#include "LXENTTECSerial.h"

// ********************** defines **********************

// Pin 6 has an LED connected on MKR1000
#define RED_LED_PIN 7
#define GRN_LED_PIN 6
#define BLU_LED_PIN 8

#define RED_LED_BIT 1
#define GRN_LED_BIT 2
#define BLU_LED_BIT 4

#define RXTX_PIN 3

#define MODE_OUTPUT_DMX 0
#define MODE_INPUT_DMX 1

#define PACKET_LABEL_DMX 6
#define PACKET_LABEL_RECEIVE 8
#define PACKET_LABEL_GET_INFO 3
#define PACKET_LABEL_GET_SERIAL 10

// ********************** globals **********************

uint8_t green_pin = 0;
uint8_t mode = MODE_OUTPUT_DMX;
int got_dmx = 0;
uint8_t buffer[513];
LXENTTECSerial eSerial = LXENTTECSerial();

// ***************** setup() runs once  ****************

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GRN_LED_PIN, OUTPUT);           
  pinMode(BLU_LED_PIN, OUTPUT);
  SAMD21DMX.setDirectionPin(RXTX_PIN); 
  Serial.begin(57600);//115200, etc.  probably doesn't matter because it changes to USB speed
}

// ***************** utility functions  ****************

/*  setLED(uint8_t flags)
 *  sets color of RGB LED
 */

void setLED(uint8_t flags) {
  digitalWrite(RED_LED_PIN, 1 & flags);
  if ( 1 & (flags>>1) ) {
  		green_pin = (~green_pin) & 0x1;
  		digitalWrite(GRN_LED_PIN, green_pin);
  }
  digitalWrite(BLU_LED_PIN, 1 & (flags>>2));
}


// ***************** input/output functions *************

void gotDMXCallback(int slots) {
  got_dmx = slots;
}

void doOutputMode() {
  uint8_t label = eSerial.readPacket();
  if ( label == ENTTEC_LABEL_SEND_DMX ) {
    int s = eSerial.numberOfSlots() + 1;		//add start code
    for(int i=0; i<s; i++) {
      SAMD21DMX.setSlot(i,eSerial.getSlot(i));
    }
    SAMD21DMX.startOutput();  //ignored if already started
    setLED(GRN_LED_BIT);		//toggles green on and off
  } else if ( label == ENTTEC_LABEL_RECEIVE_DMX ) {
	SAMD21DMX.setDataReceivedCallback(&gotDMXCallback);
	mode = MODE_INPUT_DMX;
	SAMD21DMX.startInput();
	setLED(BLU_LED_BIT);
  } else if ( label == ENTTEC_LABEL_NONE ) {
    setLED(RED_LED_BIT);
  }
}

void doInputMode() {
  if ( Serial.available()) {  //writing anything to the USB switched to output mode
	SAMD21DMX.setDataReceivedCallback(0);
	mode = MODE_OUTPUT_DMX;
	setLED(0);
    return;
  }
  if ( got_dmx ) {
    int msg_size = got_dmx;
	 eSerial.writeDMXPacket(SAMD21DMX.dmxData(), msg_size);
    got_dmx = 0;
  }
  delay(50);         // wait to allow serial to keep up
}

// ***************** main loop  ****************

void loop() {
  if ( mode == MODE_OUTPUT_DMX ) {
    doOutputMode();
  } else {
    doInputMode();
  }
}        //main loop


