# LXSAMD21DMX
DMX Driver for AVR SAM-D21 microcontrollers (Arduino MKR1000 & Zero, Adafruit Feather)

   LXSAMD21DMX is a driver for sending or receiving DMX using one of the SAM D-21's five SERCOM serial periperhial interfaces
   
   LXSAMD21DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LXSAMD21DMX input mode continuously receives DMX once its interrupts have been enabled using startInput()
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LXSAMD21DMX is used with a single instance called SAMD21DMX	