# StandAloneServoSolution

A free standing Arduino based solution for using Servo motors to operate model railway points (model railroad switches)

Put simply, this code is offered as a simple yet flexible way for people modelling railways to deploy Servo Motor based
point motors using a free standing Arduino hardware with minimal additional hardware.

The Arduino (when loaded with the firmware) is configured using the "Serial Monitor" of the Arduino IDE, or any other
Serial Terminal emulation (e.g. minicom) at 9600 baud.  The configuration can be written to the EEPROM of the Arduino so that, when
powered on in isolation, it always loads, verifys and implements the last configuration saved.

The Servo motors are operated using any of the PWM supporting pins on the Arduino.  The state of the Servo is controlled
using either a Toggle switch or Momentary switch connected between a specified digital input pin and ground.

The "sweep" angle of the servo can be specified between 0 and 180 degrees, but is always based on a start angle of 0.

Optionally two additional digital pin can be specified as "feedback" output pins, and can be used to light up one of
two LEDs indicating which state the servo is current in.

The firmware supports as many Servos as the Arduino has PWM capable pins (though the Main console TX/RX pin and the on
board LED are explicitly excluded).

The following is the output of the 'H'elp command:

```
Stand Alone Servo Driver
------------------------

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
AN,s    Set servo to normal sweep mode (0->OFF).
AI,s    Set servo to inverted sweep mode (0->ON).
Ds,s    Delete servo definition 's'.  's' required twice
        to reduce chance of accidental use.
STs     Set servo 's' to use Toggle (on/off) switching.
SMs     Set servo 's' to use a Momentary switch.
FEs,a,b Enable feedback for servo 's' on pins 'a' and 'b'.
FDs     Disable feedback for servo 's'.
RDs     Disable realism mode on servo 's'.
RPs     Enable Point Realism on servo 's'.
RSs     Enable Signal Realism on servo 's'.
CPs,p   Configure Point Realism pause on servo 's' to
        'p'ms between steps.
CDs,d   Configure Signal Realism decay on servo 's' to 'd'.
CFs,f   Configure Signal Realism friction on servo 's' to 'f'.
CLs,l   Configure Signal Realism slack on servo 's' to 'l'.
CTs,t   Configure Signal Realism stretch on servo 's' to 't'.
CSs,e   Configure Signal Realism speed on servo 's' to 'e'.
CGs,g   Configure Signal Realism gravity mode on servo 's' to
        'g' (0: Linear, 1: Upper, 2: Lower, 3: Full arc).
W       Write configuration to EEPROM.

This Firmware dated: Nov  6 2020 for Arduino Uno.

```

All commands are in UPPER case, all arguments are decimal numeric values separated by commas as appropriate.

The following shows creation of a single servo:

```
N0,2,14
Done.
A0,90
Done.
T0
Done.
FE0,15,16
Done.
L0
s[0]: motor=2,angle=90,input=14/toggle,feedback,a=15,b=16
```

This creates a Servo called '0', where the motor is connected to PIN 2, the servo sweep has been set to 90 degrees,
the controlling switch (a toggle) is on pin 14, and finally, feedback will be provided on pins 15 and 16.

There has been the addition of "realism" modes: Disabled, Point and Signal.

Disabled simply implements the direct motion of the servo arm between two points as fast as the servo can.

Point does the same, but implments a pause (in ms) between each step, so the speed (and noise!) of the servo can be controlled.

Signal tries to recreate the jiggles and bouncing of a semaphore signal in operation.  There are six parameters for this:

Decay, Friction and Gravity affect the "downward" movement of the servo.
Slack, Stretch and Speed affect the "upward" movement of the servo.

The following output shows all Servos defined (command 'L') and all pin associations (command 'P').

Here another servo definition is shown defined with realism "signal" and a set of parameters associated with mode.

```
L
s[0]: motor=3,angle=60,input=2/toggle,quiet,signal,decay=70,friction=7,slack=7,stretch=10,speed=90,gravity=0/linear
s[1]: Undefined
s[2]: Undefined
s[3]: Undefined
s[4]: Undefined
s[5]: Undefined
P
p[0] Unavailable
p[1] Unavailable
p[2] IO -> s[0]
p[3] IO PWM -> s[0]
p[4] IO Unassigned
p[5] IO PWM Unassigned
p[6] IO PWM Unassigned
p[7] IO Unassigned
p[8] IO Unassigned
p[9] IO PWM Unassigned
p[10] IO PWM Unassigned
p[11] IO PWM Unassigned
p[12] IO Unassigned
p[13] Unavailable
p[14] IO Unassigned
p[15] IO Unassigned
p[16] IO Unassigned
p[17] IO Unassigned
p[18] IO Unassigned
p[19] IO Unassigned
p[20] IO Unassigned
p[21] IO Unassigned

```
There are still some "corner cases" where the servo doesn't settle, typically when the decay number is small and the integer maths gets stuck in a cycle.  To be looked at at some point.

Readme now formatted *better* but can still be improved.

Jeff.
