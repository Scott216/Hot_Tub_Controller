Hot Tub Controller

Project uses two Arduino's to control hot tub.
An Arduino pro-mini (3.3v) is attached to an LCD display and is part of the display
panel. Connected to it are 3 illuminated pushbuttons and a rotary encoder.
This arduino communicates with the main Arduino (a panStamp) via I2C.  The 
main Arduino controls pump, heater, bubbler and monitors water temp, pressure, amps.

