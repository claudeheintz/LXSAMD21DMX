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
#include <LXSAMD21DMX.h>


uint8_t level = 0;

void setup() {
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