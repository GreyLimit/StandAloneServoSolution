//    mul_div: Unsigned software multiplication then division
//    Copyright (C) 2022  Jeff Penfold
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.
//


//
//	The Mul_Div template function.
//

#ifndef _MUL_DIV_H_
#define _MUL_DIV_H_

template<class T> T mul_div( T a, T b, T c ) {
	T	st, sb,		// product sum top and bottom
		d, e,		// division result
		j;		// overflow check
		
	const int bits_in_T = sizeof( T ) << 3;
	const T top_bit_in_T = (T)1 << ( bits_in_T - 1 );

	st = 0;
	sb = 0;

	d = 0;
	e = 0;

	for( int i = 0; i < bits_in_T; i++ ) {
		//
		//	Shift sum left to make space
		//	for next partial sum
		//
		st <<= 1;
		if( sb & top_bit_in_T ) st |= 1;
		sb <<= 1;
		//
		//	Add a to s if top bit on b
		//	is set.
		//
		if( b & top_bit_in_T ) {
			j = sb;
			sb += a;
			if( sb < j ) st++;
		}
		//
		//	Division.
		//
		d <<= 1;
		if( st >= c ) {
			d |= 1;
			st -= c;
			e++;
		}
		else {
			if( e ) e++;
		}
		//
		//	Shift b up by one bit.
		//
		b <<= 1;
	}
	//
	//	Roll in missing bits.
	//
	for( int i = e; i < bits_in_T; i++ ) {
		//
		//	Shift across product sum
		//
		st <<= 1;
		if( sb & top_bit_in_T ) st |= 1;
		sb <<= 1;
		//
		//	Division, continued.
		//
		d <<= 1;
		if( st >= c ) {
			d |= 1;
			st -= c;
		}
	}
	return( d );
}

#endif

//
//	EOF
//
