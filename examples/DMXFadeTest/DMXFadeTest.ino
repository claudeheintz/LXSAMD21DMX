/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see SAMD21DMX.h or http://lx.claudeheintzdesign.com/opensource.html)
    @copyright 2016 by Claude Heintz

    Simple Fade test of SAMD21 DMX Driver
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/

// The general default for the LXSAMD21DMX is to use SERCOM 4 with pins 5 and 4 for DMX RX/TX
// Some SERCOM & Pin setups are defined for specific boards.
// Read the LXSAMD21DMX.h file for other options and uncomment/edit the following line to select them:
//#define use_optional_sercom_macros 4

#include <LXSAMD21DMX.h>


uint8_t level = 0;

void setup() {
  SAMD21DMX.setDirectionPin(3);  // Or, wire pins 2 & 3 of MAX485 to v+ for testing
  SAMD21DMX.startOutput();
}

/************************************************************************

  The main loop fades the levels of addresses 1 and 505 and 512 to full
  
*************************************************************************/

void loop() {
 SAMD21DMX.setSlot(1,level);
 SAMD21DMX.setSlot(505,level);
 SAMD21DMX.setSlot(512,level);
 delay(50);
 level++;
}