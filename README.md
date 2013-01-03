Spacesucker
===========

Fresh air controller (code not functional yet).


Reads out two out of three states of a CO2 measuring device. The states are: "high, but acceptable" and "just too high".
It also checks the spacestate, "open" or "closed". When closed the fan is shut down for 55 minutes every hour.
There is a manual override function and a serial 9600 baud interface for external control and parametrisation.

The output is a low frequency PWM signal. At 0% the fan should stay off (without drawing current) and at 100% the fan
runs at maximum speed. These setpoints can be adjusted on the dimmer PCB. Be careful, there is live voltage on almost
the entire PCB!

There is an IRED on the controller PCB for future options (like controlling an airconditioning unit).

The 7-segment display shows status and errors. More about that later...
