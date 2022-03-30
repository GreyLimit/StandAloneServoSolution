//
//	StandAloneServoSolution (Version 2)
//
// 	Version 1/
//
//	An Arduino Sketch providing a "free standing" solution
//	to using and controlling simple Servo motors for the
//	operation of model railway points/switches.
//
//	Version 2/
//
//	Version 1 extended to offer "realistic" operation of signals
//	in addition operation of points/switches.  The "realism" is
//	the addition of simulated signal bounce and cable slack/stretch.
//
//	Copyright (C) 2020, Jeff Penfold, jeff.penfold@googlemail.com
//	
//	This program is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

//
//	Some standard library includes.
//
#include <ctype.h>
#include <stdlib.h>

//
//	Include access to program memory area.
//
#include <avr/pgmspace.h>

//
//	Include the Arduino provided servo control library.
//
//	This does the work of actually making the servos
//	move as desired.
//
#include <Servo.h>

//
//	Bring in the pre-generated gravity tables:
//
//		linear_gravity[ LINEAR_GRAVITY ] values from 0 to LINEAR_GRAVITY_SCALE
//		upper_gravity[ UPPER_GRAVITY ] values from 0 to UPPER_GRAVITY_SCALE
//		lower_gravity[ LOWER_GRAVITY ] values from 0 to LOWER_GRAVITY_SCALE
//		arc_gravity[ ARC_GRAVITY ] values from 0 to ARC_GRAVITY_SCALE
//
//	These map to the time gaps between steps in three different cases:
//
//		Linear		A vertical straight line drop.
//		Upper		From the top of a circle to horizontal.
//		Lower		From horizontal to the bottom of a circle.
//		Arc		From vertical up to vertical down.
//
//	Use the following defines to turn on/off specific tables:
//
//		DEFINE_LINEAR_GRAVITY
//		DEFINE_UPPER_GRAVITY
//		DEFINE_LOWER_GRAVITY
//		DEFINE_ARC_GRAVITY
//
#define DEFINE_LINEAR_GRAVITY
#define DEFINE_UPPER_GRAVITY
#define DEFINE_LOWER_GRAVITY
#define DEFINE_ARC_GRAVITY
//
//	Include it!
//
#include "Gravity.h"

//
//	Include the "mul_div()" routine for higher speed and
//	reliable multiplication followed by division without
//	intermediate overflow (hopefully).
//
//	Pertinent for AVR or similar hardware architectures.
//
#include "mul_div.h"

//
//	Define an thin wrapper layer on the Servo library to
//	enable "sub degree" angular positioning.
//
//	The following can be determined for the Servo library
//	source code.
//
//	Servo::write( int value )
//
//		When 'value < MIN_PULSE_WIDTH' then value is
//		assumed to be in degrees, and is range cropped
//		between (inclusive) 0 and 180.
//
//		When 'value >= MIN_PULSE_WIDTH' then value is
//		the number of microseconds (us) for the pulse
//		width.
//
//	MIN_PULSE_WIDTH
//
//		This is the shortest valid pulse width that will
//		control a servo motor.
//
//	MAX_PULSE_WIDTH
//
//		This is the longest valid pulse with that will
//		control a servo motor.
//
//	For this wrapper layer the following macros are defined:
//
//	ARC_GRANULARITY		How many steps are there from 
//				MIN_PULSE_WIDTH ("0") to MAX_PULSE_WIDTH
//				("180")
//
//	ARC_TO_DEGREES(v)	Convert an ARC value to the nearest
//				DEGREE value.
//
//	DEGREES_TO_ARC(v)	Convert a Degree value to the
//				corresponding ARC value.
//
//	ARC_TO_PULSE(v)		Convert an ARC value to its matching
//				pulse width value.
//
#define ARC_GRANULARITY		(180*GRAVITY_SUB_STEPS)
#define ARC_TO_DEGREES(v)	mul_div<unsigned int>((v),180,ARC_GRANULARITY)
#define DEGREES_TO_ARC(v)	mul_div<unsigned int>((v),ARC_GRANULARITY,180)
#define ARC_TO_PULSE(v)		(MIN_PULSE_WIDTH+mul_div<unsigned int>((v),(MAX_PULSE_WIDTH-MIN_PULSE_WIDTH),ARC_GRANULARITY))

//
//	Include the Arduino provided EEPROM access library.
//
//	This is used to keep the current configuration of the
//	Arduino so it always "turns on" with the last saved
//	configuration.
//
//	It *will not* use the EEPROM to remember which position
//	any servo was last set to.
//
#include <EEPROM.h>

//
//	Define some constant values used across the sketch.
//
#define ERROR		(-1)
#define EOS		'\0'
#define SPACE		' '
#define BS		'\b'

//
//	Syntactic sugar for pointer to functions
//
#define FUNC(a)		(*(a))
#define UNUSED(x)	__attribute__((unused)) x

//
//	Arbitrary debounce count size.
//
#define DEBOUNCE_TIME	100

//
//	Set LED flash interval (500 = 1/2 second)
//
#define LED_DELAY	500

//
//	Define the macro REALISM_UPDATES if you wish to
//	see how the realism code is driving the servo
//
//	Warning:  Lots of output that will impact the
//	realism and performance of the system.
//
//#define REALISM_UPDATES

//
//	Define bit set type used to capture the hardware
//	facilities of a pin.  Symbols can be combined with
//	"|" to fully describe available pin function.
//
#define PIN_FUNC enum pin_func
PIN_FUNC {
	FUNC_NONE	= 0000,
	FUNC_DIGITAL	= 0001,
	FUNC_PWM	= 0002
};

//
//	Define (based on the type of the Arduino this is being
//	compiled for) the number and  type of pins available
//	for the program to use.
//
//	For the moment only the following devices explicitly
//	recognised:
//
//		AVR_NANO
//		AVR_UNO
//		AVR_MEGA2560
//
//	The following definitions and tables are created to
//	reflection the hardware available:
//
//	AVAILBLE_PINS			The number entries in
//					the pin table
//
//	pin_table[0..AVAILBLE_PINS-1]	The definition of each pin
//					available on the machine.
//
//	AVAILABLE_SERVOS		The number of servos that
//					the machine can support.
//
//	CONSOLE				Define the name of the serial
//					console port.
//

#ifdef ARDUINO_AVR_NANO
//
//	ARDUINO NANO
//	------------
//
#define AVAILBLE_PINS		22
#define AVAILABLE_SERVOS	6
#define LED_PIN			13
#define CONSOLE			Serial
#define BOARD_IDENTIFIED	"Arduino Nano"
static int pin_table[ AVAILBLE_PINS ] = {
	FUNC_NONE,			// 0 -> RX
	FUNC_NONE,			// 1 -> TX
	FUNC_DIGITAL,			// 2
	FUNC_DIGITAL|FUNC_PWM,		// 3
	FUNC_DIGITAL,			// 4
	FUNC_DIGITAL|FUNC_PWM,		// 5
	FUNC_DIGITAL|FUNC_PWM,		// 6
	FUNC_DIGITAL,			// 7
	FUNC_DIGITAL,			// 8
	FUNC_DIGITAL|FUNC_PWM,		// 9
	FUNC_DIGITAL|FUNC_PWM,		// 10
	FUNC_DIGITAL|FUNC_PWM,		// 11
	FUNC_DIGITAL,			// 12
	FUNC_NONE,			// 13 -> LED
	FUNC_DIGITAL,			// 14
	FUNC_DIGITAL,			// 15
	FUNC_DIGITAL,			// 16
	FUNC_DIGITAL,			// 17
	FUNC_DIGITAL,			// 18
	FUNC_DIGITAL,			// 19
	FUNC_DIGITAL,			// 20
	FUNC_DIGITAL			// 21
};
#endif

#ifdef ARDUINO_AVR_UNO
//
//	ARDUINO UNO
//	------------
//
#define AVAILBLE_PINS		22
#define AVAILABLE_SERVOS	6
#define LED_PIN			13
#define CONSOLE			Serial
#define BOARD_IDENTIFIED	"Arduino Uno"
static int pin_table[ AVAILBLE_PINS ] = {
	FUNC_NONE,			// 0 -> RX
	FUNC_NONE,			// 1 -> TX
	FUNC_DIGITAL,			// 2
	FUNC_DIGITAL|FUNC_PWM,		// 3
	FUNC_DIGITAL,			// 4
	FUNC_DIGITAL|FUNC_PWM,		// 5
	FUNC_DIGITAL|FUNC_PWM,		// 6
	FUNC_DIGITAL,			// 7
	FUNC_DIGITAL,			// 8
	FUNC_DIGITAL|FUNC_PWM,		// 9
	FUNC_DIGITAL|FUNC_PWM,		// 10
	FUNC_DIGITAL|FUNC_PWM,		// 11
	FUNC_DIGITAL,			// 12
	FUNC_NONE,			// 13 -> LED
	FUNC_DIGITAL,			// 14
	FUNC_DIGITAL,			// 15
	FUNC_DIGITAL,			// 16
	FUNC_DIGITAL,			// 17
	FUNC_DIGITAL,			// 18
	FUNC_DIGITAL,			// 19
	FUNC_DIGITAL,			// 20
	FUNC_DIGITAL			// 21
};
#endif

#ifdef ARDUINO_AVR_MEGA2560
//
//	ARDUINO MEGA 2560
//	-----------------
//
#define AVAILBLE_PINS		70
#define AVAILABLE_SERVOS	14
#define LED_PIN			13
#define CONSOLE			Serial
#define BOARD_IDENTIFIED	"Arduino Mega2560"
static int pin_table[ AVAILBLE_PINS ] = {
	FUNC_NONE,			// 0 -> RX
	FUNC_NONE,			// 1 -> TX
	FUNC_DIGITAL|FUNC_PWM,		// 2
	FUNC_DIGITAL|FUNC_PWM,		// 3
	FUNC_DIGITAL|FUNC_PWM,		// 4
	FUNC_DIGITAL|FUNC_PWM,		// 5
	FUNC_DIGITAL|FUNC_PWM,		// 6
	FUNC_DIGITAL|FUNC_PWM,		// 7
	FUNC_DIGITAL|FUNC_PWM,		// 8
	FUNC_DIGITAL|FUNC_PWM,		// 9

	FUNC_DIGITAL|FUNC_PWM,		// 10
	FUNC_DIGITAL|FUNC_PWM,		// 11
	FUNC_DIGITAL|FUNC_PWM,		// 12
	FUNC_NONE,			// 13 -> LED
	FUNC_DIGITAL,			// 14
	FUNC_DIGITAL,			// 15
	FUNC_DIGITAL,			// 16
	FUNC_DIGITAL,			// 17
	FUNC_DIGITAL,			// 18
	FUNC_DIGITAL,			// 19

	FUNC_DIGITAL,			// 20
	FUNC_DIGITAL,			// 21
	FUNC_DIGITAL,			// 22
	FUNC_DIGITAL,			// 23
	FUNC_DIGITAL,			// 24
	FUNC_DIGITAL,			// 25
	FUNC_DIGITAL,			// 26
	FUNC_DIGITAL,			// 27
	FUNC_DIGITAL,			// 28
	FUNC_DIGITAL,			// 29

	FUNC_DIGITAL,			// 30
	FUNC_DIGITAL,			// 31
	FUNC_DIGITAL,			// 32
	FUNC_DIGITAL,			// 33
	FUNC_DIGITAL,			// 34
	FUNC_DIGITAL,			// 35
	FUNC_DIGITAL,			// 36
	FUNC_DIGITAL,			// 37
	FUNC_DIGITAL,			// 38
	FUNC_DIGITAL,			// 39

	FUNC_DIGITAL,			// 40
	FUNC_DIGITAL,			// 41
	FUNC_DIGITAL,			// 42
	FUNC_DIGITAL,			// 43
	FUNC_DIGITAL|FUNC_PWM,		// 44
	FUNC_DIGITAL|FUNC_PWM,		// 45
	FUNC_DIGITAL|FUNC_PWM,		// 46
	FUNC_DIGITAL,			// 47
	FUNC_DIGITAL,			// 48
	FUNC_DIGITAL,			// 49

	FUNC_DIGITAL,			// 50
	FUNC_DIGITAL,			// 51
	FUNC_DIGITAL,			// 52
	FUNC_DIGITAL,			// 53
	FUNC_DIGITAL,			// 54
	FUNC_DIGITAL,			// 55
	FUNC_DIGITAL,			// 56
	FUNC_DIGITAL,			// 57
	FUNC_DIGITAL,			// 58
	FUNC_DIGITAL,			// 59

	FUNC_DIGITAL,			// 60
	FUNC_DIGITAL,			// 61
	FUNC_DIGITAL,			// 62
	FUNC_DIGITAL,			// 63
	FUNC_DIGITAL,			// 64
	FUNC_DIGITAL,			// 65
	FUNC_DIGITAL,			// 66
	FUNC_DIGITAL,			// 67
	FUNC_DIGITAL,			// 68
	FUNC_DIGITAL			// 69

};
#endif

//
//	Test configuration has been identified.
//
#ifndef BOARD_IDENTIFIED
#error "This board has not been identified."
#endif

//
//	Define valid ranges (and default values) for a number of the
//	parametric values used through the firmware.
//
#define PAUSE_MIN		0
#define PAUSE_MAX		1000
#define PAUSE_DEFAULT		50

#define DECAY_MIN		0
#define DECAY_MAX		100
#define DECAY_DEFAULT		50

#define FRICTION_MIN		0
#define FRICTION_MAX		100
#define FRICTION_DEFAULT	50

#define GRAVITY_START		500

#define SLACK_MIN		0
#define SLACK_MAX		100
#define SLACK_DEFAULT		10
#define SLACK_DELAY		250

#define STRETCH_MIN		0
#define STRETCH_MAX		100
#define STRETCH_DEFAULT		10

#define SPEED_MIN		0
#define SPEED_MAX		100
#define SPEED_DEFAULT		10

//
//	Define the various "models" of realism which can be implemented
//	by the software
//
#define REALISM enum realism
REALISM {
	NO_REALISM,		// The default, Version 1, solution.
	POINT_REALISM,		// Apply realism appropriate to a point.
	SIGNAL_REALISM		// Apply realism appropriate to a signal.
};

//
//	Define the various types of gravity table which can be applied
//	to a signal arm
//
#define ACCEL enum accel
ACCEL {
	LINEAR_ACCEL,	// Default acceleration curve: a line!
	UPPER_ACCEL,	// Top to horizontal
	LOWER_ACCEL,	// Horizontal to bottom
	ARC_ACCEL	// Top to bottom
};

//
//	Define the data used to handle the configuration of a single
//	servo.
//
#define SERVO_CONF struct servo_conf
SERVO_CONF {
	//
	//	Base configuration details:  These are the minimum
	//	details required to operate the servo with optional
	//	feedback.
	//
	//	Note on "inverted".  The following tables aligns the
	//	servo position with the "logical" position.
	//
	//		Servo
	//		position	0	sweep
	//	Inverted
	//
	//	True			ON	OFF
	//
	//	False			OFF	ON
	//
	int	active,			// Defined and in use?
		sweep,			// Angle to move the servo through.
		inverted,		// inverted servo operation?
		toggle,			// toggle switch (rather than momentary)?
		feedback,		// sending feedback signals?

		servo,			// Pin used for this servo.
		input,			// Control input pin.
		output_off,		// Feedback servo is "On"
		output_on;		// ... ... ... or "Off".
	//
	//	These are the additional details required to implement the
	//	optional realism elements of the system for either points
	//	or signals.
	//
	REALISM	mode;			// What level of realism is being applied.
	//
	//	Each applied mode of realism has its specific data
	//	in a different (but over lapping) element
	union {
		//
		//	Point/Switch realism parameters:
		//
		struct {
			int	pause;		// This is the milliseconds count between
						// each adjustment of the servo.
		} point;
		//
		//	Signal realism parameters:
		//
		struct {
			//
			//	Dropping parameters
			//
			int	decay,		// Indication of bounciness of the signal
				friction;	// Indication of resistance to dropping
			ACCEL	acc;		// How the arm drops.
			//
			//	Lifting parameters.
			//
			int	slack,		// Indication of slack in control wire
				stretch,	// and the stretch in it.
				speed;		// Finally how fast it is lifted.
		} signal;
	} var;
};

//
//	Because the whole of the configuration data will be copied
//	into the EEPROM and subsequently read out of it on power up
//	the following data structure is used to gather the data into
//	a single unit.
//
#define CONFIGURATION struct configuration
CONFIGURATION {
	SERVO_CONF	servo[ AVAILABLE_SERVOS ];
};

//
//	Define the number of chunks required to capture the whole
//	of the configuration data (a chunk is an int).
//
#define CHUNK_COUNT (( (int)sizeof( CONFIGURATION ) + (int)sizeof( int ) -1 ) / (int)sizeof( int ))

//
//	The above structure is how all the data is stored, but the
//	following structure is the actual unit written/read to the
//	EEPROM
//
#define EEPROM_DATA struct eeprom_data
EEPROM_DATA {
	int			checksum;
	union {
		CONFIGURATION	data;
		int		chunk[ CHUNK_COUNT ];
	} var;
};

//
//	Define two routines for checksum calculation and checking.
//
//	Nothing flash, just an integer based parity check.
//
static void set_checksum( EEPROM_DATA *ptr ) {
	ptr->checksum = ~0;
	for( int i = 0; i < CHUNK_COUNT; ptr->checksum ^= ptr->var.chunk[ i++ ]);
}
static int valid_eeprom( EEPROM_DATA *ptr ) {
	int	c;

	c = ~0;
	for( int i = 0; i < CHUNK_COUNT; c ^= ptr->var.chunk[ i++ ]);
	return( c == ptr->checksum );
}

//
//	Front end routines for EEPROM IO.
//
static int write_eeprom( byte *data, unsigned int count ) {
	if( count > EEPROM.length()) return( false );
	for( unsigned int a = 0; a < count ; a++ ) EEPROM.write( a, data[ a ]);
	return( true );
}
static int read_eeprom( byte *data, unsigned int count ) {
	if( count > EEPROM.length()) return( false );
	for( unsigned int a = 0; a < count ; a++ ) data[ a ] = EEPROM.read( a );
	return( true );
}

//
//	Output text from program memory to CONSOLE.
//
static void display_progmem( const char *s ) {
	char	c;

	while(( c = pgm_read_byte( s++ )) != EOS ) CONSOLE.print( c );
}

//
//	Define the variable used to hold the configuration data.
//
static EEPROM_DATA configuration;

//
//	Define a "reverse lookup array" to go from a pin number
//	to a servo configuration number. The ERROR value indicates
//	pin not assigned.
// 
static int assigned_pin[ AVAILBLE_PINS ];

//
//	Define the set of states which a servo can be "in".
//
#define SERVO_STATE enum servo_state
SERVO_STATE {
	SERVO_OFF,		// Servo at "Off" position
	SERVO_MOVING_ON,	// Servo moving towards "On" position
	SERVO_MOVING_OFF,	// Servo moving towards "Off" position
	SERVO_ON		// Servo at "On" position
};

//
//	For each servo there is a "runtime" record which keeps
//	track of the current state of the servo.  This is not
//	kept in the EEPROM as the information is essentially
//	volatile and easily re-established on power on.
//
#define RUN_TIME struct run_time
RUN_TIME {
	//
	//	This is the DRIVER object for this servo.
	//
	Servo		driver;
	//
	//	Pointers to the support routines for this
	//	servo.  These align to the type of realism
	//	which the servo is configured to emulate.
	//
	void	FUNC( set_on )( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
	void	FUNC( set_off )( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
	void	FUNC( set_flip )( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
	void	FUNC( run_state )( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
	//
	//	keep tabs on acceleration table being used.
	//
	int	*acc_table,		// Pointer to the table in use
		acc_size,		// Number of entries in the table
		acc_scale;		// Scale factor used to create the table
	//
	//	Note the state of the servo.
	//
	SERVO_STATE	state;
	//
	//	When is the next event for this servo?
	//
	unsigned long	next_event;
	//
	//	Note the servo position and the speed at which
	//	its position is changing.
	//
	int		position,
			speed;
	//
	//	For realism modes which are implemented via a finite
	//	state machine the following variable is provided
	//	as the "state index" used to drive the actions of
	//	the code.
	//
	//	Note: It is assumed in through this firware that an
	//	index of 0 is a stable "do nothing" position in the
	//	code.
	//
	int		index;
	//
	//	The following variables are used by the finite state machine
	//	while simulating signal realism.
	//
	int		fsm_speed,
			fsm_count,
			fsm_dir,
			fsm_target,
			fsm_limit;
	//
	//	Define the variables which are used to debounce the
	//	input signals from the switch.
	//
	int		button_state,
			button_count,
			human_input;
};

//
//	The following array maintains the run time state for
//	each of the servos.
//
static RUN_TIME run_state[ AVAILABLE_SERVOS ];

//
//	Routines used to reset the content of the configuration
//
static void reset_servo( SERVO_CONF *ptr ) {
	//
	//	Version 1 parameters
	//
	ptr->active = false;
	ptr->sweep = 0;
	ptr->inverted = false;
	ptr->toggle = false;
	ptr->feedback = false;
	ptr->servo = ERROR;
	ptr->input = ERROR;
	ptr->output_off = ERROR;
	ptr->output_on = ERROR;
	//
	//	Version 2 extensions.
	//
	ptr->mode = NO_REALISM;
}

static void reset_config( EEPROM_DATA *ptr ) {
	for( int i = 0; i < AVAILABLE_SERVOS; reset_servo( &( ptr->var.data.servo[ i++ ])));
	set_checksum( ptr );
}

//
//	The support routines for the various realism modes are outlined
//	as this point (they are referenced in the "apply_config()" routine)
//	but they will be defined later in the program.
//
//	Routines for the default no realism mode.
//
static void no_realism_on( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void no_realism_off( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void no_realism_flip( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void no_realism_run( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );

//
//	Routines for the POINT realism mode.
//
static void point_realism_on( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void point_realism_off( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void point_realism_flip( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void point_realism_run( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );

//
//	Routines for the SIGNAL realism mode.
//
static void signal_realism_on( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void signal_realism_off( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void signal_realism_flip( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
static void signal_realism_run( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf );
		
//
//	The "apply_config()" routine does a fair bit of range checking, so
//	this macros visually cleans up that code (a little).
//
//		v	Variable
//		x	minimum value
//		y	maximum value
//
#define SET_RANGE(v,x,y) {int z,*za;z=*(za=&(v));if(z<(x))z=(x);if(z>(y))z=(y);*za=z;}

//
//	Sanitise the configuration and set everything up.  If
//	something in the configuration does not add up, then it
//	dropped.  Only a valid working configuration should be
//	left at the end of this.
//
//	Only call by the setup() routine, once.
//
static int apply_config( CONFIGURATION *ptr ) {
	int	i, r;

	r = 0;
	//
	//	Clear pin table so we can insert data from
	//	the configuration provided.
	//
	for( i = 0; i < AVAILBLE_PINS; assigned_pin[ i++ ] = ERROR );
	//
	//	Clear the run time state information, we will fill in
	//	the correct values as we scan the data loaded in from
	//	EEPROM.
	//
	for( i = 0; i < AVAILABLE_SERVOS; i++ ) {
		run_state[ i ].driver.detach();
		
		run_state[ i ].set_on = no_realism_on;
		run_state[ i ].set_off = no_realism_off;
		run_state[ i ].set_flip = no_realism_flip;
		run_state[ i ].run_state = no_realism_run;
		
		run_state[ i ].acc_table = (int *)linear_gravity;
		run_state[ i ].acc_size = LINEAR_GRAVITY;
		run_state[ i ].acc_scale = LINEAR_GRAVITY_SCALE;
		
		run_state[ i ].button_count = 0;
		run_state[ i ].button_state = false;
		run_state[ i ].human_input = false;
		
		run_state[ i ].state = SERVO_OFF;
		run_state[ i ].next_event = 0;
		run_state[ i ].speed = SPEED_MIN;
		run_state[ i ].position = 0;
		run_state[ i ].index = 0;
	}
	//
	//	Do this one servo at a time.
	//
	for( i = 0; i < AVAILABLE_SERVOS; i++ ) {
		//
		//	Check this servo not already allocated.
		//
		if( ptr->servo[ i ].active ) {
			int	p, q;
			
			//
			//	Validate that all of the pin numbers used are
			//	in range first.
			//
			q = 0;
			p = ptr->servo[ i ].input;
			if(( p < 0 )||( p >= AVAILBLE_PINS )||( assigned_pin[ p ] != ERROR )||(( pin_table[ p ] & FUNC_DIGITAL ) == 0 )) q++;
			p = ptr->servo[ i ].servo;
			if(( p < 0 )||( p >= AVAILBLE_PINS )||( assigned_pin[ p ] != ERROR )||(( pin_table[ p ] & FUNC_PWM ) == 0 )) q++;
			if( ptr->servo[ i ].feedback ) {
				//
				//	Check feedback pins, if used.
				//
				p = ptr->servo[ i ].output_off;
				if(( p < 0 )||( p >= AVAILBLE_PINS )||( assigned_pin[ p ] != ERROR )||(( pin_table[ p ] & FUNC_DIGITAL ) == 0 )) q++;
				p = ptr->servo[ i ].output_on;
				if(( p < 0 )||( p >= AVAILBLE_PINS )||( assigned_pin[ p ] != ERROR )||(( pin_table[ p ] & FUNC_DIGITAL ) == 0 )) q++;
			}
			else {
				//
				//	Should be ERROR
				//
				if(( ptr->servo[ i ].output_off != ERROR )||( ptr->servo[ i ].output_on != ERROR )) q++;
			}
			//
			//	Any problems found?
			//
			if( q ) {
				//
				//	One of the pin numbers is invalid.
				//
				reset_servo( &( ptr->servo[ i ]));
				r++;
			}
			else {
				//
				//	We know that the pin numbers are valid, that each pin is capable
				//	of the role it is expected to do so now only need to check that
				//	the pins are unique.  To do this we update the assigned_pin array
				//	but checking if we suddenly find and entry not set to ERROR.
				//
				p = ptr->servo[ i ].input;
				assigned_pin[ p ] = i;			// no need to check first pin.
				p = ptr->servo[ i ].servo;
				if( assigned_pin[ p ] != ERROR ) q++;
				assigned_pin[ p ] = i;
				if(( p = ptr->servo[ i ].output_off ) != ERROR ) {
					if( assigned_pin[ p ] != ERROR ) q++;
					assigned_pin[ p ] = i;
				}
				if(( p = ptr->servo[ i ].output_on ) != ERROR ) {
					if( assigned_pin[ p ] != ERROR ) q++;
					assigned_pin[ p ] = i;
				}
				//
				//	Any problems found?
				//
				if( q ) {
					//
					//	Roll back pin assignments before clearing out the
					//	servo record.
					//
					p = ptr->servo[ i ].input;
					assigned_pin[ p ] = ERROR;
					p = ptr->servo[ i ].servo;
					assigned_pin[ p ] = ERROR;
					if(( p = ptr->servo[ i ].output_off ) != ERROR ) {
						assigned_pin[ p ] = ERROR;
					}
					if(( p = ptr->servo[ i ].output_on ) != ERROR ) {
						assigned_pin[ p ] = ERROR;
					}
					//
					//	We have duplicated pin numbers, remove the
					//	pin assignments and remove the servo definition.
					//
					reset_servo( &( ptr->servo[ i ]));
					r++;
				}
				else {
					//
					//	At this point we are happy that the *basic* configuration
					//	of the servo is acceptable: pin numbers valid and unique.
					//
					//	We now set up the run time record for this servo to enable
					//	the software to operate the servo as required.
					//
					
					//
					//	INPUT pin from the operator.
					//
					pinMode( ptr->servo[ i ].input, INPUT_PULLUP );
					//
					//	Allocate the SERVO PWM pin to the driver.
					//
					run_state[ i ].driver.attach( ptr->servo[ i ].servo );
					run_state[ i ].driver.write( 0 );
					//
					//	Quick sanity check the sweep angle.
					//
					SET_RANGE( ptr->servo[ i ].sweep, 0, ARC_GRANULARITY );
					//
					//	Align state with inverted flag.
					//
					if( ptr->servo[ i ].inverted ) run_state[ i ].state = SERVO_ON;
					//
					//	Feedback OFF
					//
					if(( p = ptr->servo[ i ].output_off ) != ERROR ) {
						pinMode( p, OUTPUT );
						digitalWrite( p, (( run_state[ i ].state == SERVO_OFF )? HIGH: LOW ));
					}
					//
					//	Feedback ON
					//
					if(( p = ptr->servo[ i ].output_on ) != ERROR ) {
						pinMode( p, OUTPUT );
						digitalWrite( p, (( run_state[ i ].state == SERVO_ON )? HIGH: LOW ));
					}
					//
					//	With all elements of the Version 1 data
					//	confirmed and validated, the version 2
					//	extensions can be validated.
					//
					switch( ptr->servo[ i ].mode ) {
						case NO_REALISM: {
							//
							//	Nothing to do her as the default assignments have
							//	already covered this off.
							//
							break;
						}
						case POINT_REALISM: {
							//
							//	Assign the POINT realism functions.
							//
							run_state[ i ].set_on = point_realism_on;
							run_state[ i ].set_off = point_realism_off;
							run_state[ i ].set_flip = point_realism_flip;
							run_state[ i ].run_state = point_realism_run;
							//
							//	Validate the point realism
							//	parameters resetting to
							//	defaults where not appropriate.
							//
							//	ptr->servo[ i ].var.point.*
							//
							SET_RANGE( ptr->servo[ i ].var.point.pause, PAUSE_MIN, PAUSE_MAX );
							break;
						}
						case SIGNAL_REALISM: {
							//
							//	Assign the SIGNAL realism functions.
							//
							run_state[ i ].set_on = signal_realism_on;
							run_state[ i ].set_off = signal_realism_off;
							run_state[ i ].set_flip = signal_realism_flip;
							run_state[ i ].run_state = signal_realism_run;
							//
							//	Validate the signal realism
							//	parameters resetting to
							//	defaults where not appropriate.
							//
							//	ptr->servo[ i ].var.signal.*
							//
							SET_RANGE( ptr->servo[ i ].var.signal.decay, DECAY_MIN, DECAY_MAX );
							SET_RANGE( ptr->servo[ i ].var.signal.friction, FRICTION_MIN, FRICTION_MAX );
							SET_RANGE( ptr->servo[ i ].var.signal.slack, SLACK_MIN, SLACK_MAX );
							SET_RANGE( ptr->servo[ i ].var.signal.stretch, STRETCH_MIN, STRETCH_MAX );
							SET_RANGE( ptr->servo[ i ].var.signal.speed, SPEED_MIN, SPEED_MAX );
							break;
						}
						default: {
							//
							//	Bad mode value .. say nothing and
							//	reset to NO_REALISM.
							//
							ptr->servo[ i ].mode = NO_REALISM;
							r++;
							break;
						}
					}
					//
					//	Gravity acceleration mode
					//
					switch( ptr->servo[ i ].var.signal.acc ) {
						case LINEAR_ACCEL: {
							//
							//	This is the default mode .. already set up in the run time.
							//
							break;
						}
						case UPPER_ACCEL: {
							//
							//	Upper quadrant gravity acceleration
							//
							run_state[ i ].acc_table = (int *)upper_gravity;
							run_state[ i ].acc_size = UPPER_GRAVITY;
							run_state[ i ].acc_scale = UPPER_GRAVITY_SCALE;
							break;
						}
						case LOWER_ACCEL: {
							//
							//	Lower quadrant gravity acceleration
							//
							run_state[ i ].acc_table = (int *)lower_gravity;
							run_state[ i ].acc_size = LOWER_GRAVITY;
							run_state[ i ].acc_scale = LOWER_GRAVITY_SCALE;
							break;
						}
						case ARC_ACCEL: {
							//
							//	Full arc gravity acceleration
							//
							run_state[ i ].acc_table = (int *)arc_gravity;
							run_state[ i ].acc_size = ARC_GRAVITY;
							run_state[ i ].acc_scale = ARC_GRAVITY_SCALE;
							break;
						}
						default: {
							//
							//	Anything we don't recognise gets converted to linear.
							//
							ptr->servo[ i ].var.signal.acc = LINEAR_ACCEL;
							r++;
							break;
						}
					}
				}
			}
		}
	}
	return( r );
}

//
//----------------------------------------------------------------------
//

//
//	Define all of the Textual comments and fractional replies
//	the program makes and store them in the program space so
//	that the dynamic memory space is not used to store what are
//	effectively static strings.
//
static const char str_invalid_servo_number[] PROGMEM = "Invalid servo number.\r\n";
static const char str_invalid_pin_number[] PROGMEM = "Invalid pin number.\n";
static const char str_invalid_servo_pin_number[] PROGMEM = "Invalid servo pin number.\r\n";
static const char str_invalid_input_pin_number[] PROGMEM = "Invalid input pin number.\r\n";
static const char str_invalid_feedback_pin_number[] PROGMEM = "Invalid feedback pin number.\r\n";
static const char str_invalid_feedback_pin[] PROGMEM = "Invalid feedback pin.\r\n";
static const char str_invalid_duplicate_pins[] PROGMEM = "Invalid duplicate pin numbers.\r\n";
static const char str_servo_already_defined[] PROGMEM = "Servo already defined.\r\n";
static const char str_invalid_servo_pin[] PROGMEM = "Invalid servo pin.\r\n";
static const char str_invalid_input_pin[] PROGMEM = "Invalid input pin.\r\n";
static const char str_servo_not_defined[] PROGMEM = "Servo not defined.\r\n";
static const char str_feedback_not_defined[] PROGMEM = "Feedback not defined.\r\n";
static const char str_feedback_already_defined[] PROGMEM = "Feedback already defined.\r\n";
static const char str_realism_not_defined[] PROGMEM = "Realism not defined.\r\n";
static const char str_realism_already_defined[] PROGMEM = "Realism already defined.\r\n";
static const char str_invalid_angle[] PROGMEM = "Invalid sweep angle.\r\n";
static const char str_invalid_realism_mode[] PROGMEM = "Not supported by defined realism mode.\r\n";
static const char str_invalid_pause[] PROGMEM = "Invalid pause interval.\r\n";
static const char str_invalid_gravity[] PROGMEM = "Invalid gravity mode.\r\n";
static const char str_missmatched_servos[] PROGMEM = "Miss-matched servo numbers.\r\n";
static const char str_write_failed[] PROGMEM = "Write to EEPROM failed.\r\n";
static const char str_done[] PROGMEM = "Done.\r\n";
static const char str_killed[] PROGMEM = "(Killed)\r\n";
static const char str_unavailable[] PROGMEM = "Unavailable";
static const char str_undefined[] PROGMEM = "Undefined";
static const char str_unassigned[] PROGMEM = "Unassigned";
static const char str_io[] PROGMEM = "IO ";
static const char str_pwm[] PROGMEM = "PWM ";
static const char str_cr_lf[] PROGMEM = "\r\n";


//
//	Text items specific to display_pin() function.
//
static const char str_ds_1[] PROGMEM = "s[";
static const char str_ds_2[] PROGMEM = "]: ";
static const char str_ds_3[] PROGMEM = "motor=";
static const char str_ds_4a[] PROGMEM = ",angle=";
static const char str_ds_4b[] PROGMEM = "/inverted";
static const char str_ds_5[] PROGMEM = ",input=";
static const char str_ds_6[] PROGMEM = "/toggle";
static const char str_ds_7[] PROGMEM = "/momentary";
static const char str_ds_8[] PROGMEM = ",feedback,a=";
static const char str_ds_9[] PROGMEM = ",b=";
static const char str_ds_10[] PROGMEM = ",quiet";
static const char str_ds_11[] PROGMEM = ",point,pause=";
static const char str_ds_12[] PROGMEM = ",signal,decay=";
static const char str_ds_13[] PROGMEM = ",friction=";
static const char str_ds_14[] PROGMEM = ",slack=";
static const char str_ds_15[] PROGMEM = ",stretch=";
static const char str_ds_16[] PROGMEM = ",speed=";
static const char str_ds_17[] PROGMEM = ",no realism";
static const char str_ds_18[] PROGMEM = ",gravity=";
static const char str_ds_18a[] PROGMEM = "0/linear";
static const char str_ds_18b[] PROGMEM = "1/upper";
static const char str_ds_18c[] PROGMEM = "2/lower";
static const char str_ds_18d[] PROGMEM = "3/arc";
static const char str_ds_18e[] PROGMEM = "Eh?";

//
//	Display a specific servo definition.
//
static void display_servo( CONFIGURATION *c, int s ) {
	SERVO_CONF	*p;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	p = &( c->servo[ s ]);
	display_progmem( str_ds_1 );
	CONSOLE.print( s );
	display_progmem( str_ds_2 );
	if( p->active ) {
		display_progmem( str_ds_3 );
		CONSOLE.print( p->servo );
		display_progmem( str_ds_4a );
		CONSOLE.print( ARC_TO_DEGREES( p->sweep ));
		if( p->inverted ) display_progmem( str_ds_4b );
		display_progmem( str_ds_5 );
		CONSOLE.print( p->input );
		display_progmem( p->toggle? str_ds_6: str_ds_7 );
		if( p->feedback ) {
			display_progmem( str_ds_8 );
			CONSOLE.print( p->output_off );
			display_progmem( str_ds_9 );
			CONSOLE.print( p->output_on );
		}
		else {
			display_progmem( str_ds_10 );
		}
		switch( p->mode ) {
			case POINT_REALISM: {
				display_progmem( str_ds_11 );
				CONSOLE.print( p->var.point.pause );
				break;
			}
			case SIGNAL_REALISM: {
				display_progmem( str_ds_12 );
				CONSOLE.print( p->var.signal.decay );
				display_progmem( str_ds_13 );
				CONSOLE.print( p->var.signal.friction );
				display_progmem( str_ds_14 );
				CONSOLE.print( p->var.signal.slack );
				display_progmem( str_ds_15 );
				CONSOLE.print( p->var.signal.stretch );
				display_progmem( str_ds_16 );
				CONSOLE.print( p->var.signal.speed );
				display_progmem( str_ds_18 );
				switch( p->var.signal.acc ) {
					case LINEAR_ACCEL: {
						display_progmem( str_ds_18a );
						break;
					}
					case UPPER_ACCEL: {
						display_progmem( str_ds_18b );
						break;
					}
					case LOWER_ACCEL: {
						display_progmem( str_ds_18c );
						break;
					}
					case ARC_ACCEL: {
						display_progmem( str_ds_18d );
						break;
					}
					default: {
						display_progmem( str_ds_18e );
						break;
					}
				}
				break;
			}
			default: {
				display_progmem( str_ds_17 );
				break;
			}
		}
	}
	else {
		display_progmem( str_undefined );
	}

	display_progmem( str_cr_lf );
}

//
//	Text items specific to display_pin() function.
//
static const char str_dp_1[] PROGMEM = "p[";
static const char str_dp_2[] PROGMEM = "] ";
static const char str_dp_3[] PROGMEM = "-> s[";
static const char str_dp_4[] PROGMEM = "]";

//
//	Display a specific pin definition.
//
static void display_pin( UNUSED( CONFIGURATION *c ), int p ) {
	if(( p < 0 )||( p >= AVAILBLE_PINS )) {
		display_progmem( str_invalid_pin_number );
		return;
	}
	display_progmem( str_dp_1 );
	CONSOLE.print( p );
	display_progmem( str_dp_2 );
	if( pin_table[ p ] == FUNC_NONE ) {
		display_progmem( str_unavailable );
	}
	else {
		if( pin_table[ p ] & FUNC_DIGITAL ) display_progmem( str_io );
		if( pin_table[ p ] & FUNC_PWM ) display_progmem( str_pwm );
		if( assigned_pin[ p ] == ERROR ) {
			display_progmem( str_unassigned );
		}
		else {
			display_progmem( str_dp_3 );
			CONSOLE.print( assigned_pin[ p ]);
			display_progmem( str_dp_4 );
		}
	}

	display_progmem( str_cr_lf );
}

//
//	Create a new servo device.
//
static void create_servo( CONFIGURATION *c, int s, int p, int i ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	if(( p < 0 )||( p >= AVAILBLE_PINS )) {
		display_progmem( str_invalid_servo_pin_number );
		return;
	}
	if(( i < 0 )||( i >= AVAILBLE_PINS )) {
		display_progmem( str_invalid_input_pin_number );
		return;
	}
	if( p == i ) {
		display_progmem( str_invalid_duplicate_pins );
		return;
	}
	n = &( c->servo[ s ]);
	if( n->active ) {
		display_progmem( str_servo_already_defined );
		return;
	}
	if(( assigned_pin[ p ] != ERROR )||(( pin_table[ p ] & FUNC_PWM ) == 0 )) {
		display_progmem( str_invalid_servo_pin );
		return;
	}		
	if(( assigned_pin[ i ] != ERROR )||(( pin_table[ i ] & FUNC_DIGITAL ) == 0 )) {
		display_progmem( str_invalid_input_pin );
		return;
	}
	//
	//	Set up the servo.
	//
	r = &( run_state[ s ]);
	
	n->active = true;
	n->sweep = 0;		// Fail safe: Set initial sweep to 0.
	n->inverted = false;	// Start with normal servo orientation.
	
	n->servo = p;
	r->driver.attach( p );
	r->driver.write( 0 );
	assigned_pin[ p ] = s;
	
	n->toggle = false;
	n->input = i;
	pinMode( i, INPUT_PULLUP );
	assigned_pin[ i ] = s;
	
	n->feedback = false;
	n->output_off = ERROR;
	n->output_on = ERROR;
	
	n->mode = NO_REALISM;	// Start with no applied realism.
	
	r->acc_table = (int *)linear_gravity;
	r->acc_size = LINEAR_GRAVITY;
	r->acc_scale = LINEAR_GRAVITY_SCALE;
	
	r->button_count = 0;
	r->button_state = false;
	r->human_input = false;
	
	display_progmem( str_done );
}

//
//	Set the sweep angle of a defined servo.
//
static void adjust_sweep( CONFIGURATION *c, int s, int a ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	if(( a < 0 )||( a > 180 )) {
		display_progmem( str_invalid_angle );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	//
	//	Adjust servo sweep angle.
	//
	n->sweep = DEGREES_TO_ARC( a );

	display_progmem( str_done );
}

//
//	Set the inverted flag.
//
static void adjust_invert( CONFIGURATION *c, int s, int i ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	//
	//	Adjust servo inverted flag.
	//
	n->inverted = i;

	display_progmem( str_done );
}

//
//	Delete a servo device.
//
static void delete_servo( CONFIGURATION *c, int s, int x ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if( s != x ) {
		display_progmem( str_missmatched_servos );
		return;
	}	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	//
	//	Delete the servo.
	//
	r = &( run_state[ s ]);

	n->active = false;
	n->sweep = 0;
	n->inverted = false;
	
	r->driver.detach();
	assigned_pin[ n->servo ] = ERROR;
	n->servo = ERROR;
	
	n->toggle = false;
	assigned_pin[ n->input ] = ERROR;
	n->input = ERROR;

	if( n->feedback ) {
		assigned_pin[ n->output_off ] = ERROR;
		assigned_pin[ n->output_on ] = ERROR;
	}
	n->feedback = false;
	n->output_off = ERROR;
	n->output_on = ERROR;
	n->mode = NO_REALISM;

	display_progmem( str_done );
}

//
//	Set servo to toggle switch.
//
static void toggle_switch( CONFIGURATION *c, int s ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	//
	//	Enable Toggle option.
	//
	r = &( run_state[ s ]);

	n->toggle = true;
	r->button_count = 0;
	r->button_state = false;
	r->human_input = false;

	display_progmem( str_done );
}

//
//	Set servo to momentary switch.
//
static void momentary_switch( CONFIGURATION *c, int s ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	//
	//	Disable toggle option.
	//
	r = &( run_state[ s ]);

	n->toggle = false;
	r->button_count = 0;
	r->button_state = false;
	r->human_input = false;

	display_progmem( str_done );
}

//
//	Assign feedback to the servo.
//
static void assign_feedback( CONFIGURATION *c, int s, int a, int b ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	if(( a < 0 )||( a >= AVAILBLE_PINS )) {
		display_progmem( str_invalid_feedback_pin_number );
		return;
	}
	if(( b < 0 )||( b >= AVAILBLE_PINS )) {
		display_progmem( str_invalid_feedback_pin_number );
		return;
	}
	if(( assigned_pin[ a ] != ERROR )||(( pin_table[ a ] & FUNC_DIGITAL ) == 0 )) {
		display_progmem( str_invalid_feedback_pin );
		return;
	}
	if(( assigned_pin[ b ] != ERROR )||(( pin_table[ b ] & FUNC_DIGITAL ) == 0 )) {
		display_progmem( str_invalid_feedback_pin );
		return;
	}
	if( a == b ) {
		display_progmem( str_invalid_duplicate_pins );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->feedback ) {
		display_progmem( str_feedback_already_defined );
		return;
	}
	//
	//	Enable feedback on the servo
	//
	n->feedback = true;
	n->output_off = a;
	pinMode( a, OUTPUT );
	digitalWrite( a, LOW );
	assigned_pin[ a ] = s;
	n->output_on = b;
	pinMode( b, OUTPUT );
	digitalWrite( b, LOW );
	assigned_pin[ b ] = s;

	display_progmem( str_done );
}

//
//	Set servo to quiet feedback.
//
static void quiet_feedback( CONFIGURATION *c, int s ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( !n->feedback ) {
		display_progmem( str_feedback_not_defined );
		return;
	}
	//
	//	Remove any feedback.
	//
	assigned_pin[ n->output_off ] = ERROR;
	assigned_pin[ n->output_on ] = ERROR;
	n->feedback = false;
	n->output_off = ERROR;
	n->output_on = ERROR;

	display_progmem( str_done );
}

//
//	Disable/reset the realism mode for a specified servo.
//
static void disable_realism( CONFIGURATION *c, int s ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode == NO_REALISM ) {
		display_progmem( str_realism_not_defined );
		return;
	}
	//
	//	Clear the realism flag, and remember
	//	to change the pointers to realism
	//	support routines.
	//
	r = &( run_state[ s ]);

	n->mode = NO_REALISM;
	
	r->set_on = no_realism_on;
	r->set_off = no_realism_off;
	r->set_flip = no_realism_flip;
	r->run_state = no_realism_run;

	display_progmem( str_done );
}

//
//	Enable the Point realism mode for a specified servo.
//
static void enable_point_realism( CONFIGURATION *c, int s ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode == POINT_REALISM ) {
		display_progmem( str_realism_already_defined );
		return;
	}
	//
	//	Set the realism flag, and remember
	//	to change the pointers to realism
	//	support routines.
	//
	r = &( run_state[ s ]);

	n->mode = POINT_REALISM;
	n->var.point.pause = PAUSE_DEFAULT;
	
	r->set_on = point_realism_on;
	r->set_off = point_realism_off;
	r->set_flip = point_realism_flip;
	r->run_state = point_realism_run;

	display_progmem( str_done );
}

//
//	Enable the Signal realism mode for a specified servo.
//
static void enable_signal_realism( CONFIGURATION *c, int s ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode == SIGNAL_REALISM ) {
		display_progmem( str_realism_already_defined );
		return;
	}
	//
	//	Set the realism flag, and remember
	//	to change the pointers to realism
	//	support routines.
	//
	r = &( run_state[ s ]);

	n->mode = SIGNAL_REALISM;
	n->var.signal.decay = DECAY_DEFAULT;
	n->var.signal.friction = FRICTION_DEFAULT;
	n->var.signal.acc = LINEAR_ACCEL;
	n->var.signal.slack = SLACK_DEFAULT;
	n->var.signal.stretch = STRETCH_DEFAULT;
	n->var.signal.speed = SPEED_DEFAULT;
	
	r->set_on = signal_realism_on;
	r->set_off = signal_realism_off;
	r->set_flip = signal_realism_flip;
	r->run_state = signal_realism_run;

	r->acc_table = (int *)linear_gravity;
	r->acc_size = LINEAR_GRAVITY;
	r->acc_scale = LINEAR_GRAVITY_SCALE;
	
	//
	//	Signal realism uses a state machine.  Whatever
	//	the value was, reset to 0 as a safe starting
	//	point.
	//
	r->index = 0;

	display_progmem( str_done );
}


//
//	Set the pause value to p for servo s (iif running in
//	point realism mode).
//
static void set_point_pause( CONFIGURATION *c, int s, int p ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode != POINT_REALISM ) {
		display_progmem( str_invalid_realism_mode );
		return;
	}
	if(( p < PAUSE_MIN )||( p > PAUSE_MAX )) {
		display_progmem( str_invalid_pause );
		return;
	}
	//
	//	Update pause time.
	//
	n->var.point.pause = p;
	
	display_progmem( str_done );
}

//
//	Set the decay value to d for servo s (iif running in
//	signal realism mode).
//
static void set_signal_decay( CONFIGURATION *c, int s, int d ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode != SIGNAL_REALISM ) {
		display_progmem( str_invalid_realism_mode );
		return;
	}
	if(( d < DECAY_MIN )||( d > DECAY_MAX )) {
		display_progmem( str_invalid_pause );
		return;
	}
	//
	//	Update decay time.
	//
	n->var.signal.decay = d;
	
	display_progmem( str_done );
}

//
//	Set the friction value to f for servo s (iif running in
//	signal realism mode).
//
static void set_signal_friction( CONFIGURATION *c, int s, int f ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode != SIGNAL_REALISM ) {
		display_progmem( str_invalid_realism_mode );
		return;
	}
	if(( f < FRICTION_MIN )||( f > FRICTION_MAX )) {
		display_progmem( str_invalid_pause );
		return;
	}
	//
	//	Update friction time.
	//
	n->var.signal.friction = f;
	
	display_progmem( str_done );
}

//
//	Set the slack value to k for servo s (iif running in
//	signal realism mode).
//
static void set_signal_slack( CONFIGURATION *c, int s, int k ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode != SIGNAL_REALISM ) {
		display_progmem( str_invalid_realism_mode );
		return;
	}
	if(( k < SLACK_MIN )||( k > SLACK_MAX )) {
		display_progmem( str_invalid_pause );
		return;
	}
	//
	//	Update slack time.
	//
	n->var.signal.slack = k;
	
	display_progmem( str_done );
}

//
//	Set the stretch value to t for servo s (iif running in
//	signal realism mode).
//
static void set_signal_stretch( CONFIGURATION *c, int s, int t ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode != SIGNAL_REALISM ) {
		display_progmem( str_invalid_realism_mode );
		return;
	}
	if(( t < STRETCH_MIN )||( t > STRETCH_MAX )) {
		display_progmem( str_invalid_pause );
		return;
	}
	//
	//	Update stretch time.
	//
	n->var.signal.stretch = t;
	
	display_progmem( str_done );
}

//
//	Set the speed value to e for servo s (iif running in
//	signal realism mode).
//
static void set_signal_speed( CONFIGURATION *c, int s, int e ) {
	SERVO_CONF	*n;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode != SIGNAL_REALISM ) {
		display_progmem( str_invalid_realism_mode );
		return;
	}
	if(( e < SPEED_MIN )||( e > SPEED_MAX )) {
		display_progmem( str_invalid_pause );
		return;
	}
	//
	//	Update speed time.
	//
	n->var.signal.speed = e;
	
	display_progmem( str_done );
}

//
//	Set the gravity value to g for servo s (iif running in
//	signal realism mode).
//
static void set_signal_gravity( CONFIGURATION *c, int s, int g ) {
	SERVO_CONF	*n;
	RUN_TIME	*r;
	
	if(( s < 0 )||( s >= AVAILABLE_SERVOS )) {
		display_progmem( str_invalid_servo_number );
		return;
	}
	n = &( c->servo[ s ]);
	if( !n->active ) {
		display_progmem( str_servo_not_defined );
		return;
	}
	if( n->mode != SIGNAL_REALISM ) {
		display_progmem( str_invalid_realism_mode );
		return;
	}
	if(( g < 0 )||( g > 3 )) {
		display_progmem( str_invalid_gravity );
		return;
	}
	//
	//	Update speed time.
	//
	r = &( run_state[ s ]);

	switch( g ) {
		case 1: {
			//
			//	Anything else defaults to the default.
			//
			n->var.signal.acc = UPPER_ACCEL;
			r->acc_table = (int *)upper_gravity;
			r->acc_size = UPPER_GRAVITY;
			r->acc_scale = UPPER_GRAVITY_SCALE;
			break;
		}
		case 2: {
			//
			//	Anything else defaults to the default.
			//
			n->var.signal.acc = LOWER_ACCEL;
			r->acc_table = (int *)lower_gravity;
			r->acc_size = LOWER_GRAVITY;
			r->acc_scale = LOWER_GRAVITY_SCALE;
			break;
		}
		case 3: {
			//
			//	Anything else defaults to the default.
			//
			n->var.signal.acc = ARC_ACCEL;
			r->acc_table = (int *)arc_gravity;
			r->acc_size = ARC_GRAVITY;
			r->acc_scale = ARC_GRAVITY_SCALE;
			break;
		}
		default: {
			//
			//	Anything else defaults to the default.
			//
			n->var.signal.acc = LINEAR_ACCEL;
			r->acc_table = (int *)linear_gravity;
			r->acc_size = LINEAR_GRAVITY;
			r->acc_scale = LINEAR_GRAVITY_SCALE;
			break;
		}
	}
	display_progmem( str_done );
}

//
//	Save the configuration into the EEPROM
//
static void save_configuration( EEPROM_DATA *p ) {
	set_checksum( p );
	if( write_eeprom( (byte *)p, sizeof( EEPROM_DATA ))) {
		display_progmem( str_done );
	}
	else {
		display_progmem( str_write_failed );
	}
}

//
//	Strings specific to the setup() routine.
//
static const char str_setup_1[] PROGMEM = "Arduino Servo Driver.\r\n";
static const char str_setup_2[] PROGMEM = "EEPROM read fails.\r\n";
static const char str_setup_3[] PROGMEM = "EEPROM data invalid.\r\n";
static const char str_setup_4[] PROGMEM = " configuration error(s) detected.\r\n";

//
//	The SETUP Routine
//	=================
//
void setup() {
	int	r;

	//
	//	Start by getting the system console configured
	//	and wait for it to become active.
	//
	CONSOLE.begin( 9600 );
	while( !CONSOLE );
	//
	//	Say Hello!
	//
	display_progmem( str_setup_1 );
	//
	//	Try to load the configuration from the EEPROM. If
	//	this fails or the checksum is invalid then reset
	//	the configuration explicitly.
	//
	if( !read_eeprom( (byte *)&configuration, sizeof( configuration ))) {
		//
		//	Cannot extract data from EEPROM, nothing we can do.
		//
		display_progmem( str_setup_2 );
		reset_config( &configuration );
	}
	else {
		if( !valid_eeprom( &configuration )) {
			//
			//	Config extracted, but not valid.
			//
			display_progmem( str_setup_3 );
			reset_config( &configuration );
		}
	}
	//
	//	At this point we now need to take the configuration
	//	and apply it to the hardware (while checking for
	//	possible errors).
	//
	if(( r = apply_config( &configuration.var.data )) > 0 ) {
		//
		//	There have been errors, so confirm this.
		//
		CONSOLE.print( r );
		display_progmem( str_setup_4 );
	}
	//
	//	Finally, enable the LED
	//
	pinMode( LED_PIN, OUTPUT );
	//
	//	Now we can allow the loop() function to
	//	take over.
	//
}

//
//-------------------------------------------------------------------------------
//

//
//	Define the maximum number of arguments a command can support.
//
#define MAX_ARGS 5

//
//	Convert command line into command character and arguments.
//
//	The arguments are either +ve numbers or "key letters" which are
//	converted into specific -ve numbers (-ve ASCII value). 
//
static int breakup_command( char *p, char *cmd, int arg[ MAX_ARGS ]) {
	int	a;

	//
	//	This routine effectively defines what is acceptable as
	//	valid input syntax from the console.  It is fiddly just
	//	to make the input syntax a little more forgiving.
	//
	while( isblank( *p )) p++;
	if( !isupper( *p )) return( ERROR );
	*cmd = *p++;
	while( isblank( *p )) p++;
	a = 0;
	while(( a < MAX_ARGS )&&( *p != EOS )) {
		if( isdigit( *p )) {
			//
			//	Numeric argument.
			//
			arg[ a++ ] = atoi( p );
			while( isdigit( *p )) p++;
		}
		else {
			if( isupper( *p )) {
				//
				//	Key letter argument, just take the ASCII value
				//	and assign its negative value to the argument.
				//
				arg[ a++ ] = -( *p++ );
			}
			else {
				//
				//	Not what we were expecting.
				//
				return( ERROR );
			}
		}
		while( isblank( *p )) p++;
		if( *p == ',' ) p++;
		while( isblank( *p )) p++;
	}
	return( a );
}

//
//	Data for the help data.
//
static const char help_text[] PROGMEM =
	"Stand Alone Servo Driver\r\n"
	"------------------------\r\n\n"
	"Command Summary:  Upper case letter are commands, Lowercase\r\n"
	"letters represent numeric values.\r\n\n"
	"H       Display this help text.\r\n"
	"L       List Servo configuration.\r\n"
	"Ls      Display a specific servo definition.\r\n"
	"P       List Pin definitions and assignments.\r\n"
	"Pp      Display a specific pin definition and assignment.\r\n"
	"Ns,p,i  Create new servo definition 's' using pin\r\n"
	"        'p' to drive the servo and 'i' as the control\r\n"
	"        input.\r\n"
	"As,a    Set servo 's' to sweep angle 'a' (0-180).\r\n"
	"AN,s    Set servo to normal sweep mode (0->OFF).\r\n"
	"AI,s    Set servo to inverted sweep mode (0->ON).\r\n"
	"Ds,s    Delete servo definition 's'.  's' required twice\r\n"
	"        to reduce chance of accidental use.\r\n"
	"STs     Set servo 's' to use Toggle (on/off) switching.\r\n"
	"SMs     Set servo 's' to use a Momentary switch.\r\n"
	"FEs,a,b Enable feedback for servo 's' on pins 'a' and 'b'.\r\n"
	"FDs     Disable feedback for servo 's'.\r\n"
	"RDs     Disable realism mode on servo 's'.\r\n"
	"RPs     Enable Point Realism on servo 's'.\r\n"
	"RSs     Enable Signal Realism on servo 's'.\r\n"
	"CPs,p   Configure Point Realism pause on servo 's' to\r\n"
	"        'p'ms between steps.\r\n"
	"CDs,d   Configure Signal Realism decay on servo 's' to 'd'.\r\n"
	"CFs,f   Configure Signal Realism friction on servo 's' to 'f'.\r\n"
	"CLs,l   Configure Signal Realism slack on servo 's' to 'l'.\r\n"
	"CTs,t   Configure Signal Realism stretch on servo 's' to 't'.\r\n"
	"CSs,e   Configure Signal Realism speed on servo 's' to 'e'.\r\n"
	"CGs,g   Configure Signal Realism gravity mode on servo 's' to\r\n"
	"        'g' (0: Linear, 1: Upper, 2: Lower, 3: Full arc).\r\n"
	"W       Write configuration to EEPROM.\r\n\n"
	"This Firmware dated: " __DATE__ " for " BOARD_IDENTIFIED ".\r\n\n";

static const char help_prompt[] PROGMEM = "Eh? Try 'H' for help.\r\n";

//
//	Process and entered line.
//
static void process_command( EEPROM_DATA *conf, char *input ) {
	char	cmd;
	int	args,
		arg[ MAX_ARGS ];
	
	//
	//	Now interpret what we have been asked to do.
	//
	if(( args = breakup_command( input, &cmd, arg )) == ERROR ) goto ehh;
	switch( cmd ) {
		case 'H': {
			//
			//	H	Display Help.
			//
			display_progmem( help_text );
			break;
		}
		case 'L': {
			//
			//	L	List Servo configuration.
			//	Ls	Display a specific servo definition.
			//
			switch( args ) {
				case 0: {
					for( int s = 0; s < AVAILABLE_SERVOS; display_servo( &( conf->var.data ), s++ ));
					break;
				}
				case 1: {
					display_servo( &( conf->var.data ), arg[ 0 ]);
					break;
				}
				default: {
					goto ehh;
				}
			}
			break;
		}
		case 'P': {
			//
			//	P	List Pin definitions and assignments.
			//	Pp	Display a specific pin definition and assignment.
			//
			switch( args ) {
				case 0: {
					for( int p = 0; p < AVAILBLE_PINS; display_pin( &( conf->var.data ), p++ ));
					break;
				}
				case 1: {
					display_pin( &( conf->var.data ), arg[ 0 ]);
					break;
				}
				default: {
					goto ehh;
				}
			}
			break;
		}
		case 'N': {
			//
			//	Ns,p,i	Create new servo definition 'a' using pin
			//		'p' to drive the servo and 'i' as the control
			//		input.
			//
			if( args != 3 ) goto ehh;
			create_servo( &( conf->var.data ), arg[ 0 ], arg[ 1 ], arg[ 2 ]);
			break;
		}
		case 'A': {
			//
			//	As,a	Set servo 's' to sweep angle 'a' (0-180).
			//
			if( args != 2 ) goto ehh;
			//
			//	(Un)Setting Inverted flag?
			//
			switch( -arg[ 0 ]) {
				case 'N': {
					adjust_invert( &( conf->var.data ), arg[ 1 ], false );
					break;
				}
				case 'I': {
					adjust_invert( &( conf->var.data ), arg[ 1 ], true );
					break;
				}
				default: {
					adjust_sweep( &( conf->var.data ), arg[ 0 ], arg[ 1 ]);
					break;
				}
			}
			break;
		}
		case 'D': {
			//
			//	Ds,s	Delete servo definition 's'.  's' required twice
			//		to reduce chance of accidental use.
			//
			if( args != 2 ) goto ehh;
			delete_servo( &( conf->var.data ), arg[ 0 ], arg[ 1 ]);
			break;
		}
		case 'S': {
			//
			//	STs	Set servo 's' to use Toggle (on/off) switching.
			//
			//	SMs	Set servo 's' to use Momentary switching.
			//
			if( args != 2 ) goto ehh;
			switch( -arg[ 0 ]) {
				case 'T': {
					toggle_switch( &( conf->var.data ), arg[ 1 ]);
					break;
				}
				case 'M': {
					momentary_switch( &( conf->var.data ), arg[ 1 ]);
					break;
				}
				default: {
					goto ehh;
				}
			}
			break;
		}
		case 'R': {
			//
			//	RDs	Disable realism on servo 's'.
			//
			//	RPs	Enable Point realism on servo 's'.
			//
			//	RSs	Enable Signal realism on servo 's'.
			//
			if( args != 2 ) goto ehh;
			switch( -arg[ 0 ]) {
				case 'D': {
					disable_realism( &( conf->var.data ), arg[ 1 ]);
					break;
				}
				case 'P': {
					enable_point_realism( &( conf->var.data ), arg[ 1 ]);
					break;
				}
				case 'S': {
					enable_signal_realism( &( conf->var.data ), arg[ 1 ]);
					break;
				}
				default: {
					goto ehh;
				}
			}
			break;
		}
		case 'F': {
			//
			//	FEs,a,b	Enable feedback for servo 's' on pins 'a' and 'b'.
			//
			if(( args == 4 )&&( -arg[ 0 ] == 'E' )) {
				assign_feedback( &( conf->var.data ), arg[ 1 ], arg[ 2 ], arg[ 3 ]);
				break;
			}
			//
			//	FDs	Disable feedback for servo 's'.
			//
			if(( args == 2 )&&( -arg[ 0 ] == 'D' )) {
				quiet_feedback( &( conf->var.data ), arg[ 1 ]);
				break;
			}
			goto ehh;
		}
		case 'C': {
			//
			//	CPs,p	Configure Point Realism pause on servo 's' to 'p'
			//		milliseconds between each step.
			//
			//	CDs,d	Configure Signal Realism decay on servo 's' to 'd'.
			//	CFs,f	Configure Signal Realism friction on servo 's' to 'f'.
			//	CLs,l	Configure Signal Realism slack on servo 's' to 'l'.
			//	CTs,t	Configure Signal Realism stretch on servo 's' to 't'.
			//	CSs,e	Configure Signal Realism speed on servo 's' to 'e'.
			//	CGs,g	Configure Signal Realism gravity mode on servo 's' to 'g'
			//
			if( args != 3 ) goto ehh;
			switch( -arg[ 0 ]) {
				case 'P': {
					set_point_pause( &( conf->var.data ), arg[ 1 ], arg[ 2 ]);
					break;
				}
				case 'D': {
					set_signal_decay( &( conf->var.data ), arg[ 1 ], arg[ 2 ]);
					break;
				}
				case 'F': {
					set_signal_friction( &( conf->var.data ), arg[ 1 ], arg[ 2 ]);
					break;
				}
				case 'L': {
					set_signal_slack( &( conf->var.data ), arg[ 1 ], arg[ 2 ]);
					break;
				}
				case 'T': {
					set_signal_stretch( &( conf->var.data ), arg[ 1 ], arg[ 2 ]);
					break;
				}
				case 'S': {
					set_signal_speed( &( conf->var.data ), arg[ 1 ], arg[ 2 ]);
					break;
				}
				case 'G': {
					set_signal_gravity( &( conf->var.data ), arg[ 1 ], arg[ 2 ]);
					break;
				}
				default: {
					goto ehh;
				}
			}
			break;
		}
		case 'W': {
			//
			//	W	Write configuration to EEPROM.
			//
			if( args != 0 ) goto ehh;
			save_configuration( conf );
			break;
		}
		default: {
			//
			//	Unrecognised command.
			//
			goto ehh;
		}
	}
	return;

ehh:	display_progmem( help_prompt );
	return;
}
//
//	Define the CONSOLE buffer space when the typed commands
//	are consolidated.
//
#define BUFFER 32
static char buffer[ BUFFER+1 ];
static int buffer_used = 0;

//
//	The console processing routine.
//
static void read_console( int i ) {
	while( i-- ) {
		char	c;
		
		//
		//	Read each character (that is ready) and
		//	process it in turn.
		//
		switch(( c = CONSOLE.read())) {
			case 0x08: {
				//
				//	'\b', ^H, Backspace.
				//
				if( buffer_used > 0 ) {
					CONSOLE.write( 0x08 );
					CONSOLE.write( 0x20 );
					CONSOLE.write( 0x08 );
					buffer_used--;
				}
				break;
			}
			case 0x17: {
				//
				//	^W, Word/Line Erase.
				//
				while( buffer_used > 0 ) {
					CONSOLE.write( 0x08 );
					CONSOLE.write( 0x20 );
					CONSOLE.write( 0x08 );
					buffer_used--;
				}
				break;
			}
			case 0x03: {
				//
				//	^C, Kill Line.
				//
				display_progmem( str_killed );
				CONSOLE.write( 0x0D );
				CONSOLE.write( 0x0A );
				buffer_used = 0;
				break;
			}
			case 0x0D:
			case 0x0A: {
				//
				//	^M or ^J: Process line.
				//
				CONSOLE.write( 0x0D );
				CONSOLE.write( 0x0A );
				buffer[ buffer_used ] = EOS;
				process_command( &configuration, buffer );
				buffer_used = 0;
				break;
			}
			default: {
				//
				//	Text to add to line?
				//
				if( buffer_used < BUFFER ) {
					//
					//	Only do anything is there is space in the buffer.
					//
					if(( c >= SPACE )&&( c < 127 )) {
						CONSOLE.write( c );
						buffer[ buffer_used++ ] = c;
					}
				}
				break;
			}
		}
	}
}

//
//============================================================================
//
//	The support routines for the various realism modes are implemented here.
//
//

//
//	To simplify the insertion of the output code required when REALISM_UPDATES
//	has been defined the following set of macros are defined/used.
//
//#define REALISM_UPDATES
#ifdef REALISM_UPDATES

#define REALISM_OUT(s,v)	{static const char _s[] PROGMEM = s;display_progmem(_s);CONSOLE.print(v);display_progmem(str_cr_lf);}

#else

#define REALISM_OUT(s,v)

#endif

//
//============================================================================
//

//
//	Basic routines implement "set to 0" or "set to sweep".
//
static void default_set_on( RUN_TIME *servo, SERVO_CONF *conf ) {
	//
	//	Immediate change of servo position if not already
	//	at that position.
	//
	servo->state = SERVO_ON;
	servo->driver.write( ARC_TO_PULSE( conf->inverted? 0: conf->sweep ));
	if( conf->feedback ) {
		digitalWrite( conf->output_off, LOW );
		digitalWrite( conf->output_on, HIGH );
	}
}

static void default_set_off( RUN_TIME *servo, SERVO_CONF *conf ) {
	//
	//	Immediate change of servo position if not already
	//	at that position.
	//
	servo->state = SERVO_OFF;
	servo->driver.write( ARC_TO_PULSE( conf->inverted? conf->sweep: 0 ));
	if( conf->feedback ) {
		digitalWrite( conf->output_on, LOW );
		digitalWrite( conf->output_off, HIGH );
	}
}

//
//	Routines for the default no realism mode.
//
static void no_realism_on( UNUSED( unsigned long now ), RUN_TIME *servo, SERVO_CONF *conf ) {
	if( servo->state != SERVO_ON ) default_set_on( servo, conf );
}

static void no_realism_off( UNUSED( unsigned long now ), RUN_TIME *servo, SERVO_CONF *conf ) {
	if( servo->state != SERVO_OFF ) default_set_off( servo, conf );
}

static void no_realism_flip( UNUSED( unsigned long now ), RUN_TIME *servo, SERVO_CONF *conf ) {
	if( servo->state == SERVO_OFF ) {
		default_set_on( servo, conf );
	}
	else {
		default_set_off( servo, conf );
	}
}

static void no_realism_run( UNUSED( unsigned long now ), UNUSED( RUN_TIME *servo ), UNUSED( SERVO_CONF *conf )) {
	//
	//	Nothing to be done here.
	//
}

//
//============================================================================
//

//
//	These four routines form the core action routines that the call back routines
//	use to implement the point realism effect.
//
static void start_point_on( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	//
	//	We have been asked to start moving the point from
	//	where ever it is to ON
	//
	if( conf->feedback ) digitalWrite( conf->output_off, LOW );
	servo->state = SERVO_MOVING_ON;
	servo->next_event = now + conf->var.point.pause;
}

static void continue_point_on( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	//
	//	We are moving the point towards ON, is it time
	//	to do anything, have we got there yet?
	//
	if( now > servo->next_event ) {
		if( conf->inverted ) {
			if( servo->position > 0 ) {
				servo->driver.write( ARC_TO_PULSE( servo->position -= 1 ));
			}
			else {
				if( conf->feedback ) digitalWrite( conf->output_on, HIGH );
				servo->state = SERVO_ON;
			}
		}
		else {
			if( servo->position < conf->sweep ) {
				servo->driver.write( ARC_TO_PULSE( servo->position += 1 ));
			}
			else {
				if( conf->feedback ) digitalWrite( conf->output_on, HIGH );
				servo->state = SERVO_ON;
			}
		}
		servo->next_event = now + conf->var.point.pause;
	}
}

static void start_point_off( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	//
	//	We have been asked to start moving the point from
	//	where ever it is to OFF
	//	
	if( conf->feedback ) digitalWrite( conf->output_on, LOW );
	servo->state = SERVO_MOVING_OFF;
	servo->next_event = now + conf->var.point.pause;
}

static void continue_point_off( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	//
	//	We are moving the point towards its base, is it time
	//	to do anything, have we got there yet?
	//
	if( now > servo->next_event ) {
		if( conf->inverted ) {
			if( servo->position < conf->sweep ) {
				servo->driver.write( ARC_TO_PULSE( servo->position += 1 ));
			}
			else {
				if( conf->feedback ) digitalWrite( conf->output_off, HIGH );
				servo->state = SERVO_OFF;
			}
		}
		else {
			if( servo->position > 0 ) {
				servo->driver.write( ARC_TO_PULSE( servo->position -= 1 ));
			}
			else {
				if( conf->feedback ) digitalWrite( conf->output_off, HIGH );
				servo->state = SERVO_OFF;
			}
		}
		servo->next_event = now + conf->var.point.pause;
	}
}

//
//	Routines for the POINT realism mode.
//	------------------------------------
//
static void point_realism_on( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	switch( servo->state ) {
		case SERVO_MOVING_OFF:
		case SERVO_OFF: {
			start_point_on( now, servo, conf );
			break;
		}
		case SERVO_MOVING_ON: {
			continue_point_on( now, servo, conf );
			break;
		}
		default: {
			//
			//	Do nothing, we are there!
			//
			break;
		}
	}
}

static void point_realism_off( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	switch( servo->state ) {
		case SERVO_MOVING_ON:
		case SERVO_ON: {
			start_point_off( now, servo, conf );
			break;
		}
		case SERVO_MOVING_OFF: {
			continue_point_off( now, servo, conf );
			break;
		}
		default: {
			//
			//	Nothing to do, we are there.
			//
			break;
		}
	}
}

static void point_realism_flip( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	if(( servo->state == SERVO_OFF )||( servo->state == SERVO_MOVING_OFF )) {
		start_point_on( now, servo, conf );
	}
	else {
		start_point_off( now, servo, conf );
	}
}

static void point_realism_run( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	switch( servo->state ) {
		case SERVO_MOVING_ON: {
			continue_point_on( now, servo, conf );
			break;
		}
		case SERVO_MOVING_OFF: {
			continue_point_off( now, servo, conf );
			break;
		}
		default: {
			//
			//	Any other state means are where the point needs to be.
			//
			break;
		}
	}
}

//
//============================================================================
//

//
//	Support routines which drive the signal state machine forwards.
//
//	Note:
//
//		The "moving up" and "dropping down" realism functions are
//		different, thus:-
//
//		Up	A wire driven "pulled" action needs to reflect snatch
//			as slack is removed and cable stretch.
//
//		Down	A gravity driven "descent" with gravity acceleration,
//			friction and bounce.
//
//	These actions/processes can be captured using the following steps:
//
//		Up	a/	Small jump indicating tension/slack being removed
//				from the system
//			b/	Upward sweep, accelerating (a little) as the cable is
//				completely pulled
//			c/	Small bouncing at top of stroke
//
//		Down	a/	Gravity based acceleration (allowing for friction)
//				to the bottom of the stroke
//			b/	Larger bouncing at bottom of stroke
//
//

//
//	The routine which drives the Signal state machine forwards step-by-step.
//
static void signal_fsm( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	//
	//	If we haven't reached the time for the next event then
	//	just bail out of this routine.  Keeps following code cleaner.
	//
	if( now < servo->next_event ) return;
	//
	//	All of the state machine actions are in this routine.
	//
	switch( servo->index ) {
		//
		//	This is the "ground" state:  Nothing is done and there
		//	is no state change.
		//
		case 0: {
			REALISM_OUT( "Ground State: ", 0 );
			break;
		}
		//
		//	Generic move signal to ON code.
		//
#define SIGNAL_ON 1
		case 1: {
			REALISM_OUT( "Initialise ON: ", 1 );
			
			servo->state = SERVO_MOVING_ON;
			if( conf->feedback ) digitalWrite( conf->output_off, LOW );
			servo->fsm_speed = SPEED_MAX - conf->var.signal.speed;
			servo->next_event = now + SLACK_DELAY;
			servo->index = 2;
			break;
		}
		case 2: {
			REALISM_OUT( "Moving to ON: ", 2 );
			
			if( servo->position == servo->fsm_target ) {
				servo->fsm_speed >>= 1;
				servo->index = 3;
			}
			else {
				servo->driver.write( ARC_TO_PULSE( servo->position += servo->fsm_dir ));
			}
			servo->next_event = now + servo->fsm_speed;
			break;
		}
		case 3: {
			REALISM_OUT( "Bouncing ON: ", 3 );
			
			if( servo->position == servo->fsm_target ) {
				if( servo->fsm_limit == servo->position ) {
					servo->state = SERVO_ON;
					if( conf->feedback ) digitalWrite( conf->output_on, HIGH );
					servo->index = 0;
				}
				else {
					if(( servo->fsm_limit += servo->fsm_dir ) != servo->fsm_target ) {
						servo->fsm_dir = -servo->fsm_dir;
						servo->position += servo->fsm_dir;
					}
				}
			}
			else {
				if( servo->position == servo->fsm_limit ) servo->fsm_dir = -servo->fsm_dir;
				servo->position += servo->fsm_dir;
			}
			servo->driver.write( ARC_TO_PULSE( servo->position ));
			servo->next_event = now + servo->fsm_speed;
			break;
		}
		//
		//	Generic OFF code.
		//
#define SIGNAL_OFF 4
		case 4: {
			REALISM_OUT( "Swing OFF: ", 4 );

			servo->position += servo->fsm_dir;
			servo->driver.write( ARC_TO_PULSE( servo->position ));
			servo->next_event = now + mul_div<unsigned int>( pgm_read_word( servo->acc_table + servo->fsm_speed++ ), conf->var.signal.friction, FRICTION_MAX );
			if( servo->fsm_speed >= servo->acc_size ) servo->fsm_speed = servo->acc_size-1;
			if( servo->position == servo->fsm_target ) servo->index = 5;
			break;
		}
		case 5: {
			REALISM_OUT( "Rebound: ", 5 );

			REALISM_OUT( "speed IN: ", servo->fsm_speed );
			servo->fsm_speed = mul_div<unsigned int>( servo->fsm_speed, DECAY_MAX - conf->var.signal.decay, DECAY_MAX );
			REALISM_OUT( "speed OUT: ", servo->fsm_speed );
			if( servo->fsm_speed <= 1 ) {
				servo->position = servo->fsm_target;
				servo->state = SERVO_OFF;
				servo->driver.write( ARC_TO_PULSE( servo->position ));
				if( conf->feedback ) digitalWrite( conf->output_off, HIGH );
				servo->index = 0;
			}
			else {
				servo->index = 6;
			}
			break;
		}
		case 6: {
			REALISM_OUT( "Swing ON: ", 6 );
			servo->position -= servo->fsm_dir;
			servo->driver.write( ARC_TO_PULSE( servo->position ));		
			if( servo->fsm_speed == 0 ) {
				servo->index = 4;
			}
			else {
				servo->next_event = now + mul_div<unsigned int>( pgm_read_word( servo->acc_table + servo->fsm_speed ), conf->var.signal.friction, FRICTION_MAX );
				servo->fsm_speed -= 1;
			}
			break;
		}
		//
		//	Configuration and setup state for
		//	SIGNAL -> ON, posn 0 -> sweep
		//
#define		SIGNAL_N_ON	7
		case 7: {
			REALISM_OUT( "Setup Normal ON: ", SIGNAL_N_ON );

			servo->fsm_dir = 1;
			servo->fsm_target = conf->sweep;
			servo->fsm_limit = conf->sweep - conf->var.signal.stretch;
			if( servo->position < conf->var.signal.slack ) servo->driver.write( ARC_TO_PULSE( servo->position = conf->var.signal.slack ));
			servo->index = SIGNAL_ON;
			break;
		}
		//
		//	Configuration and setup state for
		//	SIGNAL -> OFF, posn sweep -> 0
		//
#define		SIGNAL_N_OFF	8
		case 8: {
			REALISM_OUT( "Setup Normal OFF: ", SIGNAL_N_OFF );
			
			servo->fsm_dir = -1;
			servo->fsm_target = 0;
			servo->fsm_speed = 0;
			if( conf->feedback ) digitalWrite( conf->output_on, LOW );
			servo->index = SIGNAL_OFF;
			break;
		}
		//
		//	Configuration and setup state for
		//	SIGNAL -> ON, posn sweep -> 0
		//
#define		SIGNAL_I_ON	9
		case 9: {
			int p;
				
			REALISM_OUT( "Setup Inverted ON: ", SIGNAL_I_ON );
			
			servo->fsm_dir = -1;
			servo->fsm_target = 0;
			servo->fsm_limit = conf->var.signal.stretch;
			p = conf->sweep - conf->var.signal.slack;
			if( servo->position > p ) servo->driver.write( ARC_TO_PULSE( servo->position = p ));
			servo->index = SIGNAL_ON;
			break;
		}
		//
		//	Configuration and setup state for
		//	SIGNAL -> OFF, posn 0 -> sweep
		//
#define		SIGNAL_I_OFF	10
		case 10: {
			REALISM_OUT( "Setup Inverted OFF: ", SIGNAL_I_OFF );
			
			servo->fsm_dir = 1;
			servo->fsm_target = conf->sweep;
			servo->fsm_speed = 0;
			if( conf->feedback ) digitalWrite( conf->output_on, LOW );
			servo->index = SIGNAL_OFF;
			break;
		}
		default: {
			//
			//	In the event of something odd happening, this just
			//	directs the state machine to the ground state.
			//
			servo->index = 0;
			break;
		}
	}
}

//
//	Routines for the SIGNAL realism mode.
//	-------------------------------------
//
static void signal_realism_on( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	REALISM_OUT( "signal_realism_on: ", servo->state );
	
	switch( servo->state ) {
		case SERVO_MOVING_OFF:
		case SERVO_OFF: {
			servo->index = conf->inverted? SIGNAL_I_ON: SIGNAL_N_ON;
			servo->state = SERVO_MOVING_ON;
			break;
		}
		case SERVO_MOVING_ON: {
			signal_fsm( now, servo, conf );
			break;
		}
		default: {
			break;
		}
	}
}

static void signal_realism_off( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	REALISM_OUT( "signal_realism_off: ", servo->state );
	
	switch( servo->state ) {
		case SERVO_MOVING_ON:
		case SERVO_ON: {
			servo->index = conf->inverted? SIGNAL_I_OFF: SIGNAL_N_OFF;
			servo->state = SERVO_MOVING_OFF;
			break;
		}
		case SERVO_MOVING_OFF: {
			signal_fsm( now, servo, conf );
			break;
		}
		default: {
			break;
		}
	}
}

static void signal_realism_flip( UNUSED( unsigned long now ), RUN_TIME *servo, SERVO_CONF *conf ) {
	REALISM_OUT( "signal_realism_flip: ", servo->state );
	
	if(( servo->state == SERVO_OFF )||( servo->state == SERVO_MOVING_OFF )) {
		servo->index = conf->inverted? SIGNAL_I_ON: SIGNAL_N_ON;
		servo->state = SERVO_MOVING_ON;
	}
	else {
		servo->index = conf->inverted? SIGNAL_I_OFF: SIGNAL_N_OFF;
		servo->state = SERVO_MOVING_OFF;
	}
}

static void signal_realism_run( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	REALISM_OUT( "signal_realism_run: ", servo->state );
	
	if(( servo->state == SERVO_MOVING_ON )||( servo->state == SERVO_MOVING_OFF )) signal_fsm( now, servo, conf );
}



//
//============================================================================
//

//
//	The Servo servicing routine
//
static void service_servo( unsigned long now, RUN_TIME *servo, SERVO_CONF *conf ) {
	int	enabled;
	
	//
	//	we are working on servo whose run state is at "servo" and
	//	configuration is at "conf"..
	//
	//	Only need to do anything if this is defined.
	//
	if( !conf->active ) return;
	//
	//	First action is to read the input pin and debounce the
	//	reply to establish effective value of the switch.
	//
	//	Switch "on/enabled"	->	pin == LOW
	//	Switch "off/disabled"	->	pin == HIGH
	//
	//	This is because the switch pulls the pin to ground while
	//	the internal pull-up resistor will keep the pin high if
	//	the switch is not applied.
	//
	//	Start by capturing the state of the switch as it
	//	appears right now.
	//
	enabled = ( digitalRead( conf->input ) == LOW );
	//
	//	If this is a different state from the last time we
	//	checked the input pin?
	//
	if( enabled != servo->button_state ) {
		//
		//	Yes!  We start a new debounce countdown for
		//	the new state.
		//
		servo->button_state = enabled;
		servo->button_count = DEBOUNCE_TIME;
		return;
	}
	//
	//	So we continue to be in a consistent state.  What we do
	//	is based on the value of the count down:
	//
	//	If non-zero, reduce and return (we assume that it is a
	//	glitch).
	//
	//	If zero then we have confirmed a solid input and handle
	//	it in the following code.
	//
	if( servo->button_count ) {
		//
		//	We are still counting down, take one off the
		//	counter and return.
		//
		servo->button_count -= 1;
		return;
	}
	//
	//	We have got here because either:
	//
	//	1/	The switch has been "enabled" for a consistent duration
	//
	//	or
	//
	//	2/	The switch has been "disabled" for a consistent duration
	//
	//	What we do now depends on the switch type.
	//
	if( conf->toggle ) {
		//
		//	For a toggle switch we simply adjust the servo to reflect
		//	the position of the switch:
		//
		if( enabled ) {
			FUNC( servo->set_on )( now, servo, conf );
		}
		else {
			FUNC( servo->set_off )( now, servo, conf );
		}
	}
	else {
		//
		//	For a momentary switch there are two actions..
		//
		if( enabled ) {
			//
			//	On being enabled we set the human_input flag.
			//
			servo->human_input = true;
		}
		else {
			//
			//	On being disabled we flip the servo if the human_flag was set.
			//	If not then we simply call the state machine.
			//
			if( servo->human_input ) {
				FUNC( servo->set_flip )( now, servo, conf );
				servo->human_input = false;
			}
			else {
				FUNC( servo->run_state )( now, servo, conf );
			}
		}
	}
}

//
//	Define the index variable used to cycle endlessly through the
//	table of servos, servicing each as and when.
//
static int		this_servo = 0;

//
//	Not strictly essential, but include a blinking LED to indicate
//	that the Arduino is working.
//
static unsigned long	led_next = 0;
static int		led_state = false;

//
//	The LOOP Routine.
//	=================
//
void loop() {
	int		i;
	unsigned long	now;

	//
	//	Make a note of the time now.
	//
	now = millis();
	//
	//	Before getting into the nitty-gritty servo stuff, is there
	//	some console input?
	//
	if(( i = CONSOLE.available()) > 0 ) read_console( i );

	//
	//	Now we give this servo a piece of CPU time.
	//
	service_servo( now, &( run_state[ this_servo ]), &( configuration.var.data.servo[ this_servo ]));

	//
	//	Last step, move this_servo on...
	//
	if(( this_servo += 1 ) == AVAILABLE_SERVOS ) this_servo = 0;

	//
	//	Before we loop round again, do we blink the LED?
	//
	if( now > led_next ) {
		led_next = now + LED_DELAY;
		digitalWrite( LED_PIN, (( led_state = !led_state )? HIGH: LOW ));
	}
}

//
//	EOF
//
