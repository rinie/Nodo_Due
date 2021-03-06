/*
 * Based on RawSignal_2_32bit you can derive a lot of signal properties by looking at
 * Min/Max times of pulses/spaces and Pulse + Space times
 *
 * Time variations within a timerange are typically less than 10%
 * Time variantions between different ranges are much larger
 *
 * E.g. KAKU: Pulse [296-326/998-1012], Space [207-226/899-929], Pulse+Space: [1219-1229],
 *            no sync, total time 29731, 50 #P/S, 7+ repetitions
 * ..NewKAKU: Pulse  [179-198], Space [162-199/1174-1207], Pulse+Space: [356-386/1367-1395],
 *            Sync pulse + space 2750, total time 58748, 132 #P/S 3+ repetitions
 *
 *..X10 RF remote 0546:
 *            Pulse  [467-479], Space [465-477/1575-1583], Pulse+Space: [935-952/2046-2054],
 *            Sync pulse + space 4320, total time 31457, 44 #P/S,  5+ repetitions
 *
 * AGC preamble, sync, data
 * Sync, preamble?, data, checksum
 * preamble, data, checksum...
 */

/*
// Oregon V2 decoder added - Dominique Pierre
// Oregon V3 decoder revisited - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $
 *
 * Jeelabs OOK decoders
 *
 *     byte total_bits, bits, flip, state, pos, data[25];
 * no distinction between pulse/mark and spaces
 * pos = nrBytes
 * w = isLong
 *
 *	OregonDecoderV2:
 *		200..1200, split on 700
 *			Preamble: Upto 32 long PS.
 *			Short/Short starts data
 *			TO + second Short: Manchester 0, to OK.
 *			OK + Long: Manchester 1
 *			OK + Short: TO: wait second short.
 *			? Only add data on totalbits even?,         pos = total_bits >> 4;
 *			totalbits 160 at end.
 *
 *	OregonDecoderV3:
 *			Same as V2 but
 *			pos = total_bits >> 3;
 *			totalbits 80 at end
 *
 *	CrestaDecoder:
 *		200..1300, split on 750
 *		Preamble: Long 2..10
 *			First short starts data
 *			Short/Short: 0
 *			Long: 1?
 *			End 2500 and 7 bytes or more
 *
 * KakuDecoder
 *		180 .. 450 or  950 .. 1250, split on 700
 *			4 State decoder
 *		End 2500 && 8 * pos + bits == 12
 *		Nodo: (12*4 + 2): 50: Nodo checks valid 4 time patterns: keep.
 *
 * X10
 *		350..1800, split on 720
 *		Preamble: 4000, start with short
 *		Long: 1, Short 0.
 *		Timeout: 5000
 * 		End: 2000 and at least 4 bytes
 *
 * FS20 (868)
 *		300..775, split on 500
 *		Start with 20 short
 *		0: Short/Short
 *		1: Long/Long
 *
 *		End 1500 and 5 bytes
 *
 * NibbleStop bit
 * ByteStop bit
 */

typedef enum IXPS {
	ixPulse, ixSpace, ixPulseSpace
} IXPS;

struct OokProperties {
	uint iTime; // RawIndexStart: index in RawSignal array for pulse/space times
	uint iTimeEnd; //  xEnd / RawSignal[RawIndexStart] + RawIndexStart
	uint iTimeRange[3];// pulse, space, pulse + space
} Ook;

#define Ook_NrTimeRanges(x)  (RawSignal[Ook.iTimeRange[x]] / 2)

void PrintDash(void)
  {
  PrintChar('-');
  }

#undef RKRMINMAX_VERBOSE
#define RKRRANGEANALYSE_VERBOSE
void PrintComma(void)
  {
  Serial.print(", ");
  }

void PrintNum(uint x, char c, uint digits) {
     // Rinie add space for small digits
     if(c) {
     	PrintChar(c);
 	}
	for (uint i=1, val=10; i < digits; i++, val *= 10) {
		if (x < val) {
			PrintChar(' ');
		}
	}

    Serial.print(x,DEC);
}

void PrintNumHex(uint x, char c, uint digits) {
	// Rinie add space for small digits
	if(c) {
		PrintChar(c);
	}
	for (uint i=1, val=16; i < digits; i++, val *= 16) {
		if (x < val) {
			PrintChar('0');
		}
	}

	Serial.print(x,HEX);
}

#undef RKRDOUBLE_SHORT
// todo: add byte/nibble length, stopBit
void RkrPreAmbleAnalyse(boolean fBitWise, boolean fSkipEven) {
	uint iTimeSplitPulse = RawSignal[Ook.iTimeRange[ixPulse] + 2]; //max Short
	uint iTimeSplitSpace = RawSignal[Ook.iTimeRange[ixSpace] + 2]; //max Short
	uint xEnd = Ook.iTimeEnd;
	boolean isLongPreamblePulse = RawSignal[Ook.iTime + 3] > iTimeSplitPulse;
	boolean isLongPreambleSpace = RawSignal[Ook.iTime + 4] > iTimeSplitSpace;
	boolean isLongPrev;
	uint iPreamble = 0;
	uint iBits = 0;
	byte	byteVal= 0;
	byte prevBit;
	//uint	byteValMSB= 0;
	uint x;

	// 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e puls
	// preamble: long until short or short until long...
	for (x = 5 + Ook.iTime; x <= xEnd-1; x++) {
		// pulse
		if (isLongPreamblePulse != (RawSignal[x] > iTimeSplitPulse)) {
			break;
		}
		iPreamble++;
		x++;
		// space
		if (isLongPreambleSpace != (RawSignal[x] > iTimeSplitSpace)) {
			break;
		}
		iPreamble++;
 	}
	uint iSync = 0;
	uint iTimeSplitSync = max(RawSignal[Ook.iTimeRange[ixPulse] + 4], RawSignal[Ook.iTimeRange[ixSpace] + 4]); // longer than max long
	for (;x <= xEnd-1; x++) {
		if (RawSignal[x] <= iTimeSplitSync) {
			break;
		}
		iSync++;
	}
	x++;
	// old
	boolean isLongPreamble = isLongPreamblePulse && isLongPreambleSpace;
	uint iTimeSplit = max(iTimeSplitPulse, iTimeSplitSpace);
	Serial.print("PreAmble ");
	Serial.print((iSync > 0) ? "Sync " : ((isLongPreamble) ? "Long ": "Short "));
	PrintNum(iPreamble, 0, 1);
	PrintNum(RawSignal[Ook.iTime] - iPreamble - iSync, ' ', 1);
	prevBit = (isLongPreamble) ? 1: 0; // last long was a 1
	isLongPrev = (isLongPreamble) ? false : true; // stop on short that we skipped

	// assume rest is manchester encoded
	for (;x <= xEnd-1; x++) {
		boolean isLong = RawSignal[x] > iTimeSplit;
		if ((!isLong && !isLongPrev) || isLong) {  // double short or long
			if (isLong) {
				prevBit = (prevBit) ? 0 : 1;
			}
			if ((!fSkipEven) || ((iBits%2) == 1)) {
				if (fSkipEven) {
					byteVal = (byteVal >> 1) | (prevBit ? 0x80 : 0);
				}
				else {
					byteVal = (byteVal << 1) | (prevBit ? 1 : 0);
				}
				if (fBitWise) {
					PrintNumHex(prevBit, ((fSkipEven && (iBits == 1)) || (iBits == 0)) ? ' ' : 0, 0);
				}
			}
			iBits++;
			if ((!fSkipEven && (iBits >=8)) || (iBits >= 16)) {
				iBits = 0;
				if (!fBitWise) {
					if (fSkipEven) { // swap nibbles
						PrintNumHex(((byteVal) & 0x0F), ' ', 1);
						PrintNumHex(((byteVal >> 4) & 0x0F), ' ', 1);
					}
					else {
						PrintNumHex(byteVal, ' ', 2);
					}
				}
				byteVal = 0;
			}
		}
		isLongPrev = isLong;
	}
	if ((!fBitWise) && (byteVal != 0)) {
		if (fSkipEven) { // swap nibbles
			PrintNumHex(((byteVal) & 0x0F), ' ', 1);
			PrintNumHex(((byteVal >> 4) & 0x0F), ' ', 1);
		}
		else {
			PrintNumHex(byteVal, ' ', 2);
		}
	}
	PrintTerm();
}

// 938-952/2043-2053
void RkrSyncPulseSpaceAnalyse() {
	uint iTimeSplit = RawSignal[Ook.iTimeRange[ixPulseSpace] + 2];
	uint xEnd = Ook.iTimeEnd;
	uint iBits = 0;
	byte byteValLSB= 0;
	uint x;

	Serial.print("SyncP+S ");
	// 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e puls
	for (x = 3 + Ook.iTime; x <= xEnd-1; x+=2) {
		boolean isLong = (RawSignal[x] + RawSignal[x + 1]) > iTimeSplit;
		byteValLSB = (byteValLSB << 1) | ((isLong) ? 0x01 : 0);
		iBits++;
		if (iBits >=8) {
			iBits = 0;
			PrintNumHex(byteValLSB, ' ', 2);
			byteValLSB = 0;
		}
	}
	PrintTerm();
}

byte RkrTimeRangeAnalyse() {
	uint iTimeRangePulse = RAW_BUFFER_TIMERANGE_START;
	uint iTimeRangeSpace = iTimeRangePulse + RawSignal[iTimeRangePulse] + 1;
	uint iTimeRangePulseSpace = iTimeRangePulseSpace = iTimeRangeSpace + RawSignal[iTimeRangeSpace] + 1;
	uint cPulseRanges = RawSignal[iTimeRangePulse]/2;
	uint cSpaceRanges = RawSignal[iTimeRangeSpace]/2;
	uint cPulseSpaceRanges = RawSignal[iTimeRangePulseSpace]/2;
	uint iMatchRanges = 0;
#ifdef RKRRANGEANALYSE_VERBOSE
	PrintNum(cPulseRanges, 0, 1);
	PrintNum(cSpaceRanges, ',', 1);
	PrintNum(cPulseSpaceRanges, ',', 1);
#endif
	if (cPulseRanges <= 1 || cSpaceRanges <= 1) {
			return 2; // Pulse Or Space single range: don't bother just use pulse+space length
	}
	for (uint iPulse = iTimeRangePulse + 1; iPulse <= iTimeRangePulse + RawSignal[iTimeRangePulse]; iPulse += 2) {
		int MinPulse = RawSignal[iPulse];
		int MaxPulse = RawSignal[iPulse + 1];
		for (uint iSpace = iTimeRangeSpace + 1; iSpace <= iTimeRangeSpace + RawSignal[iTimeRangeSpace]; iSpace += 2) {
			int MinSpace = RawSignal[iSpace];
			int MaxSpace = RawSignal[iSpace + 1];
#if 0
			PrintNum(MinSpace, ',', 1);
			PrintNum(abs((MinPulse - MinSpace)), ',', 1);
			PrintNum(abs((MaxPulse - MaxSpace)), ',', 1);
#endif
			// pulse length same range as space length?
			if (((abs((MinPulse - MinSpace))) < 100) && (abs(((MaxPulse - MaxSpace))) <100)) {
				RawSignal[iPulse] = min(MinPulse, MinSpace);
				RawSignal[iSpace] = min(MinPulse, MinSpace);
				RawSignal[iPulse+1] = max(MaxPulse, MaxSpace);
				RawSignal[iSpace+1] = max(MaxPulse, MaxSpace);
				iMatchRanges++;
			}
		}
	}
#ifdef RKRRANGEANALYSE_VERBOSE
	PrintNum(iMatchRanges, ',', 1);
#endif
	// ? P+S constant: don't print p+s
	return (cPulseSpaceRanges <= 1) ? 3: 1;
}

void RkrTimeRangeReplaceMedian(uint Min, uint Max, uint Median, int What) {
//	int x = 5 + Ook.iTime;
	uint xEnd = Ook.iTimeEnd;
	if (What > 1) {
		return;
	}
	// 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e puls
	for (uint x = 1 + Ook.iTime; x <= xEnd-1; x+=2) {
		uint value = RawSignal[x + What];
		if (value >= Min && value <= Max) {
				RawSignal[x + What] = Median;
		}
	}
}

#define RKR_MEDIANROUNDING
#define RKR_ROUGHSYNC // clear sync or assume same range as first pulse/space

#ifdef 	RKR_MEDIANROUNDING	// Try Median rounding
uint RkrRoundTime(uint Median) {
	if (Median > 2000) { // round 100
		Median = ((Median + 50) / 100) * 100;
	}
	else if (Median > 100) { // round 10
		Median = ((Median + 5) / 10) * 10;
	}
	return Median;
}
#endif

void RkrTimeRangePsReplaceMedian() {
	uint iTimeRange = RAW_BUFFER_TIMERANGE_START;
	for (int What = 0; What < 2; What++) {
		int i=1;
		int iEnd=RawSignal[iTimeRange];
		for (i = 1; i < iEnd; i+=2) {
				uint Min = RawSignal[iTimeRange + i];
				uint Max = RawSignal[iTimeRange + i + 1];
				uint Median = Min + (Max-Min) / 2;
#ifdef 	RKR_MEDIANROUNDING	// Try Median rounding
				Median = RkrRoundTime(Median);
#endif
				RkrTimeRangeReplaceMedian(Min, Max, Median, What);
		}
#ifdef 	RKR_ROUGHSYNC
		if (RawSignal[Ook.iTime + 1 + What] * 2 <= RawSignal[Ook.iTime + 3 + What] * 3) { // 1.5 sync
			RawSignal[Ook.iTime + 1 + What] = RawSignal[Ook.iTime + 3 + What];
		}
#endif
#ifdef 	RKR_MEDIANROUNDING	// Rounding sync/preamble as well
		RawSignal[Ook.iTime + 1 + What] = RkrRoundTime(RawSignal[Ook.iTime + 1 + What]);
		RawSignal[Ook.iTime + 3 + What] = RkrRoundTime(RawSignal[Ook.iTime + 3 + What]);
#endif
		iTimeRange += RawSignal[iTimeRange] + 1;
	}
}

// called from RawSignal_2_32bit, that is called by PrintRawSignal...
//int RkrTimeRange(uint iTime, uint iTimeRange, int What) {
int RkrTimeRange(uint MinTime, uint MaxTime, int What) {
	uint i=1;
	uint iEnd = 2;
	uint iTimeRange = (What == 0) ? RAW_BUFFER_TIMERANGE_START : Ook.iTimeRange[What - 1] + RawSignal[Ook.iTimeRange[What - 1]] + 1;
	Ook.iTimeRange[What] = iTimeRange;
	RawSignal[iTimeRange] = iEnd;
	RawSignal[iTimeRange+1] = MinTime;
	RawSignal[iTimeRange+2] = MaxTime;

	if (RawSignal[Ook.iTime] < 10) {
		return 0;
	}
	if (Ook.iTime + RawSignal[Ook.iTime] >= iTimeRange) {
		PrintNum(Ook.iTime,0, 3);
		PrintNum(Ook.iTime + RawSignal[Ook.iTime],',', 3);
		PrintNum(iTimeRange,',', 3);
		Serial.print("RkrTimeRange Overflow\n");
		return 0;
	}

#if 0
	PrintTerm();
	Serial.print(iTimeRange,DEC);
	PrintChar('*');
	Serial.print(iEnd,DEC);
	PrintChar('*');
	Serial.print(What,DEC);
	PrintTerm();
#endif
#if 0
	return iEnd;
#endif
#if 0
	for (int j=3;j<15 && ((iTimeRange + j) < (RAW_BUFFER_TIMERANGE_SIZE - 2));j++){
				RawSignal[iTimeRange + j] = RawSignal[iTimeRange + 2];
	}
#endif
#if 0
	return iEnd;
#endif
	while ((i < iEnd) && (iEnd < (RAW_BUFFER_TIMERANGE_SIZE - 2))) {
		uint Min=RawSignal[iTimeRange + i + 1];
		uint Max=RawSignal[iTimeRange + i + 0]; //RKR

#ifdef RKRMINMAX_VERBOSE
		//PrintChar(' ');
		PrintTerm();
		Serial.print(i,DEC);
		PrintChar(':');
		Serial.print(iEnd,DEC);
		PrintChar(':');
		Serial.print(Max,DEC);
		PrintChar('-');
		Serial.print(Min,DEC);
		for(int j = 0; j <= iEnd; j++) {
			PrintChar('#');
			Serial.print(RawSignal[iTimeRange+j],DEC);
		}
#endif
#if 1
		if (i > 16) {
			return iEnd;
		}
#endif
		if ((Min > Max) && ((Min-Max) > 200)) {
			uint Median=Max + (((Min-Max) > 400) ? 200 : ((Min-Max)/2)); // currently inverted min/max
			uint x = 5 + Ook.iTime;
			uint xEnd = Ook.iTimeEnd;

#ifdef RKRMINMAX_VERBOSE
			PrintChar('-');
			Serial.print(Median,DEC);
#endif
#if 1
			if (i > 12 || (xEnd >= RAW_BUFFER_SIZE+2)) {
				break;
			}
#endif
		//	uint MinCount =0;
		//	uint MaxCount =0;

			// Kleinste, groter dan mid
			// Grootste, kleinder dan mid
			// diff > x?
			// dan replace by median
			// zoek de kortste tijd (PULSE en SPACE)
			// 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e puls
			// RKR try 3 instead of 5 for starters
			for (x = 3 + Ook.iTime; x <= xEnd-4; x+=2) {
#if 1
				uint value = ((What == 0) ? RawSignal[x] // pulse
									   : ((What == 1) ? RawSignal[x + 1] //space
													:  (RawSignal[x] + RawSignal[x + 1]))); // pulse + space
#else
				uint value;
				switch (What) {
				case 0:
					value = RawSignal[x]; // pulse
					break;
				case 1:
					value = RawSignal[x+1]; // pulse
					break;
				default:
					value =  RawSignal[x] + RawSignal[x + 1]; // pulse + space
					break;
				}
#endif
				if (value < Min && value >= Median) {
					Min=value; // Zoek naar de kortste pulstijd.
				}
				if (value > Max  && value <= Median) {
					Max=value; // Zoek naar de langste pulstijd.
				}
			}
#ifdef RKRMINMAX_VERBOSE
			PrintChar('!');
			Serial.print(Max,DEC);
			PrintChar('-');
			Serial.print(Min,DEC);
#endif
			// at least one new value found: extend with 2
			if (/* (Min != Max) && */ !((Min==RawSignal[iTimeRange + i + 1]) && (Max==RawSignal[iTimeRange + i + 0]))) {
				//return iEnd;
				// extend array with 2
				iEnd += 2;
				RawSignal[iTimeRange] = iEnd;
				for (int j = iEnd; j > i+2; j--) {
					RawSignal[iTimeRange + j] = RawSignal[iTimeRange + j - 2];
				}
				RawSignal[iTimeRange + i + 1] = Max;
				RawSignal[iTimeRange + i + 2] = Min;
			}
			else {
				i++;
			}
		}
		else {
			i++;
		}
#if 0
		return iEnd;
#endif
	}
#if 0
	PrintTerm();
	Serial.print(iTimeRange,DEC);
	PrintChar('*');
	Serial.print(iEnd,DEC);
#else
	//PrintChar(' ');
#ifdef RKRMINMAX_VERBOSE
	PrintTerm();
#else
	PrintChar(' ');
#endif
	PrintChar('[');
	for(uint j = 1; j <= iEnd; j++) {
		if (j > 1) {
			if (j%2==0) {
				PrintChar('-');
			}
			else {
				PrintChar('/');
			}

		}
		Serial.print(RawSignal[iTimeRange+j],DEC);
	}
	PrintChar(']');
#ifdef RKRMINMAX_VERBOSE
	PrintTerm();
#endif
#endif

	return iEnd;
}




/*
 * Transfer point From Nodo_Due code into OokTimeRangeCode
 *
 *	RawSignal[Ook.iTime] contains start of Nodo measured timings
 *	Ook.iTime == 0: original nodo code
 *	              != 0: measure repetitions before decoding...
 *
 */
void PrintRawSignal(uint iTime) {
	uint x, xEnd;
	uint i;
	byte iPrintPulseAndSpace = 3;
	Ook.iTime = iTime;
	Ook.iTimeEnd = RawSignal[iTime] + iTime;
	xEnd = Ook.iTimeEnd;
#ifdef RAW_BUFFER_TIMERANGE_START
	int iTimeRange = RAW_BUFFER_TIMERANGE_START;
	RawSignal[iTimeRange] = 0;
	Ook.iTimeRange[ixPulse] = iTimeRange;
	Ook.iTimeRange[ixSpace] = iTimeRange;
	Ook.iTimeRange[ixPulseSpace] = iTimeRange;
#endif
	//total time
	if ((Ook.iTime > RAW_BUFFER_SIZE+2) || (Ook.iTimeEnd > RAW_BUFFER_SIZE+2)) {
		PrintNum(Ook.iTime,0, 3);
		PrintNum(Ook.iTimeEnd,',', 3);
		Serial.print("PrintRawSignal Overflow\n");
		return;
	}
	i = 0;
	for(int x=1+Ook.iTime;x<=xEnd;x++) {
		i += RawSignal[x];
	}

	if (i <= 0) {
		return;
	}
	if (Ook.iTime <= 0) { // first signal/no repetition yet
		//PrintEventCode(AnalyzeRawSignal(0));
		//PrintTerm();
		  ClockRead();
		PrintDateTime();
		PrintTerm();
		Serial.print("* ");
		// inter message time
		PrintNum(RawStartSignalTime - RawStartSignalTimeLast,0, 5);
		RawStartSignalTimeLast = RawStartSignalTime;
	}
	else {
		Serial.print("! ");
		// intra message
		PrintNum(RawSignal[xEnd+1],0, 5);
	}
//	PrintComma();
	//total time
	i = 0;
	for(int x=1+Ook.iTime;x<=xEnd;x++) {
		i += RawSignal[x];
	}
	PrintNum(i,',', 5);
//	PrintComma();

	// count
	PrintNum(RawSignal[Ook.iTime],',', 2);
//	PrintComma();

	// net count min spikes
	i = 0;
	for(int x=1+Ook.iTime;x<=xEnd;x++) {
		if (RawSignal[x] < 100) {
			i++;
		}
	}

	PrintNum(RawSignal[Ook.iTime]-i,',', 0);
//	PrintComma();

	PrintNum(i,',', 0);
	PrintComma();

	PrintEventCode(AnalyzeRawSignal(Ook.iTime));
	// todo print min/max and minButOne/maxButOne
	RawSignal_2_32bit(Ook.iTime, true);
//	PrintTerm();
	for (i=0; i < 2; i++) {
		//  PrintText(Text_07,false);
		for(int x=1+Ook.iTime;x<=xEnd;x++) {
			if ((x - (1+Ook.iTime))%16==0) {
					PrintTerm();
					PrintNum(x - (1+Ook.iTime), 0, 4);
					PrintChar(':');
					if (iPrintPulseAndSpace & 1) { // mark, space
						PrintNum(RawSignal[x], 0, 4);
					}
			}
			else {
				if (iPrintPulseAndSpace & 1) { // mark, space
					PrintNum(RawSignal[x], ',', 4);
				}
			}
			if (iPrintPulseAndSpace & 2) { // mark + space
				if ((x - (1+Ook.iTime))%2==1) {
					Serial.print(" [");
					PrintNum(RawSignal[x] + RawSignal[x-1], 0, 4);
					PrintChar(']');
				}
			}

		}
		PrintTerm();
		if (i == 0) {
			if (RawSignal[iTimeRange] != 0) {
				Serial.print("!Rounded ");
				iPrintPulseAndSpace = RkrTimeRangeAnalyse();
				RkrTimeRangePsReplaceMedian();
				if (iPrintPulseAndSpace == 2) {
					break;
				}
			}
			else {
				i = 2;
			}
		}
	}
	if (iPrintPulseAndSpace == 2) {
		for(int x=1+Ook.iTime;x<=xEnd;x++) {
			if ((x - (1+Ook.iTime))%16==0) {
					PrintTerm();
					PrintNum((x - (1+Ook.iTime))/2, 0, 4);
					PrintChar(':');
			}
			if ((x - (1+Ook.iTime))%2==1) {
				Serial.print(" [");
				PrintNum(RawSignal[x] + RawSignal[x-1], 0, 4);
				PrintChar(']');
			}
		}
		PrintTerm();
	}
#if 1 // Preamble experiment 2,2,3
	if ((Ook_NrTimeRanges(ixPulseSpace)>=3) && ((Ook_NrTimeRanges(ixPulse)>=2) && (Ook_NrTimeRanges(ixSpace)>=2))) { // preamble/manchester like
		if (RawSignal[iTime] <= 180) {
			RkrPreAmbleAnalyse(false, false);
			RkrPreAmbleAnalyse(true, false);
		}
		else {// oregon scientific v2 protocol?
			RkrPreAmbleAnalyse(false, true);
			//RkrPreAmbleAnalyse(true, true);
		}
	}
	// 1,2,2 or 2,1,2 signals: double min/max pairs...
	if ((Ook_NrTimeRanges(ixPulseSpace)==2) && ((Ook_NrTimeRanges(ixPulse)==1) || (Ook_NrTimeRanges(ixSpace)==1))) { // x10 like
		RkrSyncPulseSpaceAnalyse(); // seems ok for 44 X10...
	}
#endif
}

