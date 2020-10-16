# StandAloneServoSolution

[Best viewed in RAW for the time being]

A free standing Arduino based solution for using Servo motors to operate model railway points (model railroad switches)

Put simply, this code is offered as a simple yet flexible way for people modelling railways to deploy Servo Motor based
point motors using a free standing Arduino hardware with minimal additional hardware.

The Arduino (when loaded with the firmware) is configured using the "Serial Monitor" of the Arduino IDE, or any other
Serial Terminal emulation (e.g. minicom).  The configuration can be written to the EEPROM of the Arduino so that, when
powered on in isolation, it always loads, verifys and implements the last configuration saved.

The Servo motors are operated using any of the PWM supporting pins on the Arduino.  The state of the Servo is controlled
using either a Toggle switch or Momentary switch connected between a specified digital input pin and ground.

The "sweep" angle of the servo can be specified between 0 and 180 degrees, but is always based on a start angle of 0.

Optionally two additional digital pin can be specified and "feedback" output pins, and can be used to light up one of
two LEDs indicating which state the servo is current in.

The firmware supports as many Servos as the Arduino has PWM capable pins (though the Main console TX/RX pin and the on
board LED are explicitly excluded).

The following is the output of the 'H'elp command:

------------------------------------------------------------

Command Summary:  Upper case letter are commands, Lowercase
letters represent numeric values.

H       Display this help text.
L       List Servo configuration.
Ls      Display a specific servo definition.
P       List Pin definitions and assignments.
Pp      Display a specific pin definition and assignment.
Ns,p,i  Create new servo definition 's' using pin
        'p' to drive the servo and 'i' as the control
        input.
As,a    Set servo 's' to sweep angle 'a' (0-180).
Ds,s    Delete servo definition 's'.  's' required twice
        to reduce chance of accidental use.
Ts      Set servo 's' to use Toggle (on/off) switching.
Ms      Set servo 's' to use a momentary switch.
Fs,a,b  Set servo 's' to provide feedback on pins 'a' and 'b'.
Qs      Set servo 's' to quiet operation - no feedback.
W       Write configuration to EEPROM.

This Firmware dated: Oct 16 2020 for Arduino Mega2560.

------------------------------------------------------------

All commands are in UPPER case, all arguments are decimal numeric values separated by commas as appropaite.

The following shows creation of a single servo:

------------------------------------------------------------


N0,2,14
Done.
A0,90
Done.
T0
Done.
F0,15,16
Done.
L0
s[0]: motor=2,angle=90,input=14/toggle,feedback,a=15,b=16

------------------------------------------------------------

This creates a Servo called '0', where the motor is connected to PIN 2, the servo sweep has been set to 90 degrees,
the controlling switch (a toggle) is on pin 14, and finally, feedback will be provided on pins 15 and 16.

The follwoing out shows all Servos defined (command 'L') and all pin associations (command 'P').

------------------------------------------------------------

L
s[0]: motor=2,angle=90,input=14/toggle,feedback,a=15,b=16
s[1]: Undefined
s[2]: Undefined
s[3]: Undefined
s[4]: Undefined
s[5]: Undefined
s[6]: Undefined
s[7]: Undefined
s[8]: Undefined
s[9]: Undefined
s[10]: Undefined
s[11]: Undefined
s[12]: Undefined
s[13]: Undefined
P
p[0] Unavailable
p[1] Unavailable
p[2] IO PWM -> s[0]
p[3] IO PWM Unassigned
p[4] IO PWM Unassigned
p[5] IO PWM Unassigned
p[6] IO PWM Unassigned
p[7] IO PWM Unassigned
p[8] IO PWM Unassigned
p[9] IO PWM Unassigned
p[10] IO PWM Unassigned
p[11] IO PWM Unassigned
p[12] IO PWM Unassigned
p[13] Unavailable
p[14] IO -> s[0]
p[15] IO -> s[0]
p[16] IO -> s[0]
p[17] IO Unassigned
p[18] IO Unassigned
p[19] IO Unassigned
p[20] IO Unassigned
p[21] IO Unassigned
p[22] IO Unassigned
p[23] IO Unassigned
p[24] IO Unassigned
p[25] IO Unassigned
p[26] IO Unassigned
p[27] IO Unassigned
p[28] IO Unassigned
p[29] IO Unassigned
p[30] IO Unassigned
p[31] IO Unassigned
p[32] IO Unassigned
p[33] IO Unassigned
p[34] IO Unassigned
p[35] IO Unassigned
p[36] IO Unassigned
p[37] IO Unassigned
p[38] IO Unassigned
p[39] IO Unassigned
p[40] IO Unassigned
p[41] IO Unassigned
p[42] IO Unassigned
p[43] IO Unassigned
p[44] IO PWM Unassigned
p[45] IO PWM Unassigned
p[46] IO PWM Unassigned
p[47] IO Unassigned
p[48] IO Unassigned
p[49] IO Unassigned
p[50] IO Unassigned
p[51] IO Unassigned
p[52] IO Unassigned
p[53] IO Unassigned
p[54] IO Unassigned
p[55] IO Unassigned
p[56] IO Unassigned
p[57] IO Unassigned
p[58] IO Unassigned
p[59] IO Unassigned
p[60] IO Unassigned
p[61] IO Unassigned
p[62] IO Unassigned
p[63] IO Unassigned
p[64] IO Unassigned
p[65] IO Unassigned
p[66] IO Unassigned
p[67] IO Unassigned
p[68] IO Unassigned
p[69] IO Unassigned

------------------------------------------------------------

This read me is hardly display as I would like; my fault no doubt.

Jeff.
