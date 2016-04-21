/**************************************************************************/
/*!
    @file     DMXInputTest.ino
    @author   Claude Heintz
    @license BSD (see SAMD21DMX.h or http://lx.claudeheintzdesign.com/opensource.html)
    @copyright 2016 by Claude Heintz

    Control brightness of LED on PWM_PIN with DMX address 1
    @section  HISTORY

    v1.00 - First release
*/
/**************************************************************************/
#include <LXSAMD21DMX.h>

#define PWM_PIN 2
int got_dmx = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);

  SAMD21DMX.setDataReceivedCallback(&gotDMXCallback);
  SAMD21DMX.startInput();
  
  Serial.begin(115200);
}


// ***************** input callback function *************

void gotDMXCallback(int slots) {
  got_dmx = slots;
}

/************************************************************************

  The main loop checks to see if dmx input is available (got_dmx>0)
  And then reads the level of dimmer 1 to set PWM level of LED
  
*************************************************************************/

void loop() {
  if ( got_dmx ) {
    analogWrite(PWM_PIN,SAMD21DMX.getSlot(1));
    Serial.println("---");
    Serial.println(SAMD21DMX.getSlot(1));
    Serial.println(SAMD21DMX.getSlot(2));
    Serial.println(got_dmx);
    Serial.println("___");
  }
}
