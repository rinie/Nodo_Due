/*
 * Transfer point From Nodo_Due code into AvrLirc code
 *
 *	RawSignal[Ook.iTime] contains start of Nodo measured timings
 *	Ook.iTime == 0: original nodo code
 *	              != 0: measure repetitions before decoding...
 *
 * AvrLirc uses 2 byte, little endian format for LIRC UDP code in 1/16384 seconds...
 * This enables processing by LIRC and
 *
  * Copyright 2002 Karl Bongers (karl@turbobit.com)
  * Copyright 2007 Paul Fox (pgf@foxharp.boston.ma.us)
  *
  * This program is free software; you can redistribute it and/or
  * modify it under the terms of the GNU General Public License as
  * published by the Free Software Foundation; either version 2 of
  * the License, or (at your option) any later version.
  *
  * This program is distributed in the hope that it will be
  * useful, but WITHOUT ANY WARRANTY; without even the implied
  * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  * PURPOSE.  See the GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public
  * License along with this program; if not, write to the Free
  * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
  *
 */

/*
 * tx_char - send a serial character
 */
static void
tx_char(byte t)
{
#ifdef AVR_LIRC_BINARY
	  Serial.write(t); //arduino...
#else // test ASCII first
	PrintNumHex(t, 0, 2);
#endif
}

// start avrlic.c copy

#if DO_RECEIVE
/*
 * tx_char_hex - send a serial nibble, in ascii hex
 */
static void
tx_char_hex(byte t)
{
    if (t <= 9)
	tx_char(t + '0');
    else
	tx_char(t + 'a' - 10);

}

/*
 *  tx_hexword - send up to a word as hex
 */
static void
tx_hexword(word v)
{
    tx_char_hex((v >> 12) & 0xf);
    tx_char_hex((v >>  8) & 0xf);
    tx_char_hex((v >>  4) & 0xf);
    tx_char_hex((v >>  0) & 0xf);
}
#endif

#if DO_RECEIVE
static void
tx_str_p(const prog_char *s)
{
    char c;

    while ( (c = pgm_read_byte(s++)) )
	tx_char(c);
}
#endif

/*
 * tx_word - send 16 bits, little-endian, optionally in ascii
 */
static void
tx_word(word t)
{
#if DO_RECEIVE
    if (ascii) {
	tx_hexword(t);
	tx_str_p(crnl_s);
	return;
    }
#endif
    tx_char(t & 0xff);
    tx_char((t >> 8) & 0xff);
}

void
emit_pulse_data(uint xStart, uint xEnd)
{
    word len;
    byte high = 1;
    byte overflow = 0;
	byte had_overflow = 1;

	for(int x=xStart;x<=xEnd;x++, high = !high) {
		word pulse_length = RawSignal[x];
//		high = (x - xStart) %2 != 1;
//    while (pulse_length) {
	//	cli();
		len = pulse_length;
		//high = pulse_is_high;
		overflow = had_overflow;

		pulse_length = had_overflow = 0;

	//	sei();

	//	Led1_On();
		if (overflow) {
			// if we had an overflow, then the current pulse_length
			// is meaningless -- it's just the last remnant of a
			// long gap.  just send the previously recorded
			// overflow value to indicate that gap.  this is
			// effectively the start of a "packet".
			tx_word((overflow << 8) | 0xff);
		} else {
			uint32_t l;

			/* do long arithmetic.  expensive, but we have time. */

			l = ((uint32_t)len * 16384)/1000000; // millis() to 1/16384 sec base

			if (l > 0x7fff)	// limit range.
				len = 0x7fff;
			else
				len = l;

			if (len == 0)	// pulse length never zero.
				len++;

			if (!high)	// report the state we transitioned out of
				len |= 0x8000;

			tx_word(len);
		}
    }
}

// end avrlirc.c copy
