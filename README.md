# Arduino_heliostat
Model alt/az heliostat using Arduino

This is a complete, functioning model of a heliostat, typically used to position one mirror of a solar concentrator. The model uses a home made alt/az mount, with geared stepper motor driver for the azimuth setting, and a high resolution, 2000 step per revolution five phase motor as direct drive for the mirror altitude.

The Arduino portion of the project consists of a serial link command intepreter to align the device, the Arduino SolarPosition library to calculate the sun position at my geographic location, and TimeLib.h to keep track of the UTC and local time.

Construction:
1. The azimuth ring is a mixture of custom and home made parts. The basic mechanism is a modified pan platform from Servocity: https://www.servocity.com/gear-drive-pan-kit-for-37mm-spur-gear-motor/  However, instead of the D.C. motor drive, I fitted a 28BYJ-48 stepper with a spur gear chosen for 5.25:1 reduction, so the azimuth resolution is 10698.9 steps/revolution

2. Mirror normal positioning (altitude): This is direct drive, using a 1000 step/revolution five phase Vexta PX33M-A-C6 0.21A, 33 Ohms per phase, 0.36 degrees/step, (ten wires) with a custom made mirror mount. Driven in half-step mode 2000 steps/revolution can be achieved. Azimuth positioning of the mirror normal is thus accurate to (+/- 0.18)/2 degrees. For information on construction of the five phase driver and corresponding code, see https://github.com/jremington/Five-phase-stepper-driver

3. Power supplies: Arduino Pro Mini is USB powered from laptop, using an FTDI serial-USB converter. Motors are powered by a 5V 2A power brick.

4. Timing: Currently the program uses the Arduino crystal for timing, which is not particularly accurate. For long term tracking, you will want to add a DS3231 RTC or other source to synchronize TimeLib.h 

5. Command interpreter:

This part of the code accepts serial port commands from a laptop to initialize the stepper motors, set the zero position, drive the steppers to desired altitude/azimuth coordinates, and can be used for two different continuous tracking options. The suntracking option is similar to telescope positioning, as it automatically aligns the platform to track the sun position across the sky. The heliostat option allows one to position the mirror so that sunlight is reflected in a desired altitude/azimuth direction, then automatically positions the mirror to keep sunlight focused on that spot as the sun moves across the sky.

For the above two options to work properly, **the platform must be accurately leveled and aligned so that azimuth zero corresponds to true North, altitude zero is perfectly horizontal**.

Serial commands: Sent to the Arduino from a laptop running Teraterm version 105, which also logs performance data.

General format CC NNNN where CC is one or two lower case ASCII characters, NNNN is an character string, integer or float constant.

Commands:
t hh:mm   input local time in hours and minutes (colon required) Conversion from time zone to UTC done internally
d dd/mm   set current **UTC date** (slash required)
ja NNNN   jog altitude by NNNN steps. NNNN may be preceded by a minus sign
jz NNNN   jog azimuth  by NNNN steps. NNNN may be preceded by a minus sign
sa NNNN   (set) reset altitude origin and define current altitude position to be NNNN (degrees, floating point)
sz NNNN   (set) reset azimuth origin and define current azimuth position to be NNNN (degrees, floating point)
ma NNNN   (move) orient altitude of platform to be NNNN (degrees floating point)
mz NNNN   (move) orient azimuth of platform to be NNNN (degrees floating point)
a         (autotrack) track sun in heliostat mode from this point forward. 

Note Telescope tracking is possible, by activating two lines of code and disabling the heliostat code
              // sun tracking
              //      move_to_altitude(home.getSolarElevation(t_now));
              //      move_to_azimuth(home.getSolarAzimuth(t_now));


![IMG_1333](https://user-images.githubusercontent.com/5509037/142272758-720977f1-b27b-418c-a9b5-af44a58f26bb.JPG)
