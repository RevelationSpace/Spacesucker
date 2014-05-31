Spacesucker
===========

Fresh air controller.


Reads out two out of three states of a CO2 measuring device. The states are: "high, but acceptable" and "just too high".
It also checks the spacestate, "open" or "closed". When closed the fan stops for about a minute, then sucks at maximum for about half an hour. After that, the fan is shut down for about 55 minutes every hour. The 5 minutes that it's on is for demoisting the space.

There is a manual override function to control the speed manually. Could be handy for movie nights, or when the air is foul.

The output is a low frequency PWM signal. At 0% the fan should stay off (without drawing current) and at 100% the fan
runs at maximum speed. These setpoints can be adjusted on the dimmer PCB. Be careful, there is live voltage on almost
the entire PCB!

There is an IRED on the controller PCB but it's disconnected. The nRF24L01+ module needed that pin. Now all I/O pins are used. 

The nRF module sends the fan speed and operating mode to the receiver that is connected to the jukebox cubieboard. The cubie makes it available for everybody via mosquitto (revspace/spacesucker/#).

The 7-segment display shows status and errors:

A is auto mode, unit responds to CO2 levels
O is override (manual) mode.
H is high CO2 level detected
C is critical CO2 level detected
S is starting up fan
. only is space closed, fan off
d is space closed, fan running to get rid of moisture

all others, see c code.
