//
//	StandAloneServoSolution.
//
// 	An Arduino Sketch providing a "free standing" solution
//	to using and controlling simple Servo motors for the
//	operation of model railway points/switches.
//
//	Copyright (C) 2019, Jeff Penfold, jeff.penfold@googlemail.com
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
#define ERROR	(-1)
#define EOS	'\0'
#define SPACE	' '
#define BS	'\b'

//
//	Arbitary debounce count size.
//
#define DEBOUNCE_TIME	100

//
//	Set LED flash interval (500 = 1/2 second)
//
#define LED_DELAY	500


//
//	Define the value SWITCHING_UPDATES if you wish to
//	see servo updates (switching) displayed on the console.
//
#define SWITCHING_UPDATES

//
//	Define bit set type used to capture the hardware
//	facilities of a pin.
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
//	For the moment only the following devices explicity
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
//	Define the data used to handle the configuration of a single
//	servo.
//
#define SERVO_CONF struct servo_conf
SERVO_CONF {
	int	active,			// Defined and in use?
		sweep,			// Angle to move the servo through.
		toggle,			// toggle switch (rather than momentary)?
		feedback,		// sending feedback signals?

		servo,			// Pin used for this servo.
		input,			// Control input pin.
		output_a,		// Feedback pin a
		output_b;		// ... and b.
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
//	Define the array of Servo variables as used by the Servo
//	library.
//
static Servo driver[ AVAILABLE_SERVOS ];

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
//	Define the array which keeps track of the current state
//	of each of the defined servos.
//
static int servo_state[ AVAILABLE_SERVOS ];

//
//	Define a set of arrays which are used to debounce the
//	input signals from the switch (these are aligned
//	with the servo number as there is only one input pin
//	per servo).
//
static int button_state[ AVAILABLE_SERVOS ];
static int button_count[ AVAILABLE_SERVOS ];
static int human_input[ AVAILABLE_SERVOS ];
//
//	Routine used to reset the content of the configuration
//
static void reset_servo( SERVO_CONF *ptr ) {
	ptr->active = false;
	ptr->sweep = 0;
	ptr->toggle = false;
	ptr->feedback = false;
	ptr->servo = ERROR;
	ptr->input = ERROR;
	ptr->output_a = ERROR;
	ptr->output_b = ERROR;
}
static void reset_config( EEPROM_DATA *ptr ) {
	for( int i = 0; i < AVAILABLE_SERVOS; reset_servo( &( ptr->var.data.servo[ i++ ])));
	set_checksum( ptr );
}

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
	//	Clear tables so we can insert data from
	//	the configuration provided.
	//
	for( i = 0; i < AVAILBLE_PINS; assigned_pin[ i++ ] = ERROR );
	for( i = 0; i < AVAILABLE_SERVOS; button_count[ i++ ] = 0 );
	for( i = 0; i < AVAILABLE_SERVOS; button_state[ i++ ] = false );
	for( i = 0; i < AVAILABLE_SERVOS; human_input[ i++ ] = false );
	for( i = 0; i < AVAILABLE_SERVOS; servo_state[ i++ ] = false );
	//
	//	Do this one servo at a time.
	//
	for( i = 0; i < AVAILABLE_SERVOS; i++ ) {
		//
		//	Check this servo not already allocated.
		//
		if( ptr->servo[ i ].active ) {
			int	p, q, s;
			
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
				p = ptr->servo[ i ].output_a;
				if(( p < 0 )||( p >= AVAILBLE_PINS )||( assigned_pin[ p ] != ERROR )||(( pin_table[ p ] & FUNC_DIGITAL ) == 0 )) q++;
				p = ptr->servo[ i ].output_b;
				if(( p < 0 )||( p >= AVAILBLE_PINS )||( assigned_pin[ p ] != ERROR )||(( pin_table[ p ] & FUNC_DIGITAL ) == 0 )) q++;
			}
			else {
				//
				//	Should be ERROR
				//
				if(( ptr->servo[ i ].output_a != ERROR )||( ptr->servo[ i ].output_b != ERROR )) q++;
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
				assigned_pin[ ptr->servo[ i ].input ] = i;	// no need to check first pin.
				p = ptr->servo[ i ].servo;
				if( assigned_pin[ p ] != ERROR ) q++;
				assigned_pin[ p ] = i;
				if(( p = ptr->servo[ i ].output_a ) != ERROR ) {
					if( assigned_pin[ p ] != ERROR ) q++;
					assigned_pin[ p ] = i;
				}
				if(( p = ptr->servo[ i ].output_b ) != ERROR ) {
					if( assigned_pin[ p ] != ERROR ) q++;
					assigned_pin[ p ] = i;
				}
				//
				//	Any problems found?
				//
				if( q ) {
					p = ptr->servo[ i ].input;
					assigned_pin[ ptr->servo[ i ].input ] = ERROR;
					p = ptr->servo[ i ].servo;
					assigned_pin[ p ] = ERROR;
					if(( p = ptr->servo[ i ].output_a ) != ERROR ) {
						assigned_pin[ p ] = ERROR;
					}
					if(( p = ptr->servo[ i ].output_b ) != ERROR ) {
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
					//	Create the servo, assign the pins!
					//
					//	INPUT pin from the operator.
					//
					p = ptr->servo[ i ].input;
					pinMode( p, INPUT_PULLUP );
					//
					//	SERVO PWM pin to the motor.
					//
					p = ptr->servo[ i ].servo;
					driver[ i ].attach( p );
					driver[ i ].write( 0 );
					//
					//	Quick sanity check on sweep angle.
					//
					s = ptr->servo[ i ].sweep;
					if( s < 0 ) s = 0;
					if( s > 180 ) s = 180;
					ptr->servo[ i ].sweep = s;
					//
					//	Feedback A
					//
					if(( p = ptr->servo[ i ].output_a ) != ERROR ) {
						pinMode( p, OUTPUT );
						digitalWrite( p, LOW );
					}
					//
					//	Feedback B
					//
					if(( p = ptr->servo[ i ].output_b ) != ERROR ) {
						pinMode( p, OUTPUT );
						digitalWrite( p, LOW );
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
static const char str_invalid_angle[] PROGMEM = "Invalid sweep angle.\r\n";
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
static const char str_ds_4[] PROGMEM = ",angle=";
static const char str_ds_5[] PROGMEM = ",input=";
static const char str_ds_6[] PROGMEM = "/toggle";
static const char str_ds_7[] PROGMEM = "/momentary";
static const char str_ds_8[] PROGMEM = ",feedback,a=";
static const char str_ds_9[] PROGMEM = ",b=";
static const char str_ds_10[] PROGMEM = ",quiet";

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
		display_progmem( str_ds_4 );
		CONSOLE.print( p->sweep );
		display_progmem( str_ds_5 );
		CONSOLE.print( p->input );
		display_progmem( p->toggle? str_ds_6: str_ds_7 );
		if( p->feedback ) {
			display_progmem( str_ds_8 );
			CONSOLE.print( p->output_a );
			display_progmem( str_ds_9 );
			CONSOLE.print( p->output_b );
		}
		else {
			display_progmem( str_ds_10 );
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
static void display_pin( CONFIGURATION *c, int p ) {
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
	n->active = true;
	n->sweep = 0;		// fail safe - set initial sweep to 0.
	
	n->servo = p;
	driver[ s ].attach( p );
	driver[ s ].write( 0 );
	assigned_pin[ p ] = s;
	
	n->toggle = false;
	n->input = i;
	pinMode( i, INPUT_PULLUP );
	assigned_pin[ i ] = s;
	
	n->feedback = false;
	n->output_a = ERROR;
	n->output_b = ERROR;
	
	button_count[ s ] = 0;
	button_state[ s ] = false;
	human_input[ s ] = false;
	
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
	n->sweep = a;
	display_progmem( str_done );
}

//
//	Create a new servo device.
//
static void delete_servo( CONFIGURATION *c, int s, int x ) {
	SERVO_CONF	*n;
	
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
	n->active = false;
	n->sweep = 0;
	
	driver[ s ].detach();
	assigned_pin[ n->servo ] = ERROR;
	n->servo = ERROR;
	
	n->toggle = false;
	assigned_pin[ n->input ] = ERROR;
	n->input = ERROR;

	if( n->feedback ) {
		assigned_pin[ n->output_a ] = ERROR;
		assigned_pin[ n->output_b ] = ERROR;
	}
	n->feedback = false;
	n->output_a = ERROR;
	n->output_b = ERROR;
	display_progmem( str_done );
}

//
//	Set servo to toggle switch.
//
static void toggle_switch( CONFIGURATION *c, int s ) {
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
	//	Enable Toggle option.
	//
	n->toggle = true;
	display_progmem( str_done );
}

//
//	Set servo to momentary switch.
//
static void momentary_switch( CONFIGURATION *c, int s ) {
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
	//	Disable toggle option.
	//
	n->toggle = false;
	button_count[ s ] = 0;
	button_state[ s ] = false;
	human_input[ s ] = false;
	display_progmem( str_done );
}

//
//	Set servo to quiet feedback.
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
	n->output_a = a;
	pinMode( a, OUTPUT );
	digitalWrite( a, LOW );
	assigned_pin[ a ] = s;
	n->output_b = b;
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
	assigned_pin[ n->output_a ] = ERROR;
	assigned_pin[ n->output_b ] = ERROR;
	n->feedback = false;
	n->output_a = ERROR;
	n->output_b = ERROR;
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
static const char str_setup_4[] PROGMEM = "Reduced configuration applied.\r\n";

//
//	The SETUP Routine
//	=================
//
void setup() {
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
	if( apply_config( &configuration.var.data ) > 0 ) {
		//
		//	There have been errors, so confirm this.
		//
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
//	Define the maximum number of arguments a command can suport.
//
#define MAX_ARGS 3

//
//	Convert command line into command character and arguments.
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
		if( !isdigit( *p )) return( ERROR );
		arg[ a++ ] = atoi( p );
		while( isdigit( *p )) p++;
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
	"Ns,p,i  Create new servo definition 'a' using pin\r\n"
	"        'p' to drive the servo and 'i' as the control\r\n"
	"        input.\r\n"
	"As,a    Set servo 's' to sweep angle 'a' (0-180).\r\n"
	"Ds,s    Delete servo definition 's'.  's' required twice\r\n"
	"        to reduce chance of accidental use.\r\n"
	"Ts      Set servo 's' to use Toggle (on/off) switching.\r\n"
	"Ms      Set servo 's' to use a momentary switch.\r\n"
	"Fs,a,b  Set servo 's' to provide feedback on pins 'a' and 'b'.\r\n"
	"Qs      Set servo 's' to quiet operation - no feedback.\r\n"
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
			adjust_sweep( &( conf->var.data ), arg[ 0 ], arg[ 1 ]);
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
		case 'T': {
			//
			//	Ts	Set servo 's' to use Toggle (on/off) switching.
			//
			if( args != 1 ) goto ehh;
			toggle_switch( &( conf->var.data ), arg[ 0 ]);
			break;
		}
		case 'M': {
			//
			//	Ms	Set servo 's' to use a momentary switch.
			//
			if( args != 1 ) goto ehh;
			momentary_switch( &( conf->var.data ), arg[ 0 ]);
			break;
		}
		case 'F': {
			//
			//	Fs,a,b	Set servo 's' to provide feedback on pins 'a' and 'b'
			//
			if( args != 3 ) goto ehh;
			assign_feedback( &( conf->var.data ), arg[ 0 ], arg[ 1 ], arg[ 2 ]);
			break;
		}
		case 'Q': {
			//
			//	Qs	Set servo 's' to quiet operation - no feedback.
			//
			if( args != 1 ) goto ehh;
			quiet_feedback( &( conf->var.data ), arg[ 0 ]);
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
//	Set Servo ON
//
static void set_servo_on( int s, SERVO_CONF *p ) {
	
#	ifdef SWITCHING_UPDATES
		CONSOLE.print( "s[" );
		CONSOLE.print( s );
		CONSOLE.print( "] -> ON\r\n" );
#	endif

	//
	//	Make Servo "ON".
	//
	driver[ s ].write( p->sweep );
	if( p->feedback ) {
		digitalWrite( p->output_a, LOW );
		digitalWrite( p->output_b, HIGH );
	}
}

//
//	Set Servo OFF
//
static void set_servo_off( int s, SERVO_CONF *p ) {
	
#	ifdef SWITCHING_UPDATES
		CONSOLE.print( "s[" );
		CONSOLE.print( s );
		CONSOLE.print( "] -> OFF\r\n" );
#	endif

	//
	//	Make Servo "OFF".
	//
	driver[ s ].write( 0 );
	if( p->feedback ) {
		digitalWrite( p->output_a, HIGH );
		digitalWrite( p->output_b, LOW );
	}
}

//
//	The Servo servicing routine
//
static void service_servo( int s, SERVO_CONF *p ) {
	int	enabled;
	
	//
	//	we are working on servo number 's' whose
	//	configuration is at 'p'.
	//
	//	Only need to do anything if this is defined.
	//
	if( !p->active ) return;
	//
	//	First action is to debounce the button and establish
	//	the effective value of the switch.
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
	enabled = ( digitalRead( p->input ) == LOW );
	//
	//	If this is a different state from the last time we
	//	checked the input pin?
	//
	if( enabled != button_state[ s ]) {
		//
		//	Yes!  We start a new debounce countdown for
		//	the new state.
		//
		button_state[ s ] = enabled;
		button_count[ s ] = DEBOUNCE_TIME;
		return;
	}
	//
	//	So we continue to be in a consistent state.  What we do
	//	is based on the  value of the count down.
	//
	switch( button_count[ s ]) {
		case 0: {
			//
			//	If we have got this far then we have already
			//	handled any activity requred, we are now in a
			//	spin loop waiting for a change to the opposite
			//	switch state.
			//
			return;
		}
		case 1: {
			//
			//	The countdown has reached the "It is a real input"
			//	level, so we reduce the count to 0, but cause the
			//	code to fall through to the action code.
			button_count[ s ] = 0;
			break;
		}
		default: {
			//
			//	We are still counting down, take one off the
			//	counter and return.
			//
			button_count[ s ] -= 1;
			return;
		}
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
	if( p->toggle ) {
		//
		//	For a toggle switch we simply adjust the servo to reflect
		//	the position of the switch.
		//
		if( enabled ) {
			//
			//	Turn servo on if servo is off.
			//
			if( !servo_state[ s ]) {
				set_servo_on( s, p );
				servo_state[ s ] = true;
			}
		}
		else {
			//
			//	Turn servo off if servo is on.
			//
			if( servo_state[ s ]) {
				set_servo_off( s, p );
				servo_state[ s ] = false;
			}
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
			human_input[ s ] = true;
		}
		else {
			//
			//	On being disabled we flip the servo if the human_flag was set.
			//
			if( human_input[ s ]) {
				//
				//	Flip the servo.
				//
				if( servo_state[ s ]) {
					set_servo_off( s, p );
					servo_state[ s ] = false;
				}
				else {
					set_servo_on( s, p );
					servo_state[ s ] = true;
				}
			}
			human_input[ s ] = false;
		}
	}
}

//
//	Define the index variable used to cycle endlessly through the
//	table of servos, servicing each as and when.
//
static int	this_servo = 0;

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
	unsigned long	l;

	//
	//	Before getting into the nitty gritty servo stuff, is there
	//	some console input?
	//
	if(( i = CONSOLE.available()) > 0 ) read_console( i );

	//
	//	Now we give this servo a piece of CPU time.
	//
	service_servo( this_servo, &( configuration.var.data.servo[ this_servo ]));

	//
	//	Last step, move this_servo on...
	//
	if(( this_servo += 1 ) == AVAILABLE_SERVOS ) this_servo = 0;

	//
	//	Before we start again, do we blink the LED?
	//
	if(( l = millis()) > led_next ) {
		led_next = l + LED_DELAY;
		if(( led_state = !led_state )) {
			digitalWrite( LED_PIN, HIGH );
		}
		else {
			digitalWrite( LED_PIN, LOW );
		}
	}
}

//
//	EOF
//
