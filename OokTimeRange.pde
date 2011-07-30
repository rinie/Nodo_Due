/*
 * Based on RawSignal_2_32bit you can derive a lot of signal properties by looking at
 * Min/Max times of pulses/spaces and Pulse + Space times
 *
 * Time variations within a timerange are typically less than 10%
 * Time variantions between different ranges are much larger
 *
 *
 */

void PrintDash(void)
  {
  PrintChar('-');
  }

#undef RKRMINMAX_VERBOSE
void RkrTimeRangeReplaceMedian(uint RawIndexStart, uint Min, uint Max, uint Median, int What) {
//	int x = 5 + RawIndexStart;
	uint xEnd = RawSignal[RawIndexStart] + RawIndexStart;
	if (What > 1) {
		return;
	}
	// 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e puls
	for (uint x = 1 + RawIndexStart; x <= xEnd-1; x+=2) {
		uint value = RawSignal[x + What];
		if (value >= Min && value <= Max) {
				RawSignal[x + What] = Median;
		}
	}
}

#define RKR_MEDIANROUNDING

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

void RkrTimeRangePsReplaceMedian(uint RawIndexStart) {
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
				RkrTimeRangeReplaceMedian(RawIndexStart, Min, Max, Median, What);
		}
#ifdef 	RKR_MEDIANROUNDING	// Rounding sync/preamble as well
		RawSignal[RawIndexStart + 1 + What] = RkrRoundTime(RawSignal[RawIndexStart + 1 + What]);
		RawSignal[RawIndexStart + 3 + What] = RkrRoundTime(RawSignal[RawIndexStart + 3 + What]);
#endif
		iTimeRange += RawSignal[iTimeRange] + 1;
	}
}


int RkrTimeRange(uint RawIndexStart, uint iTimeRange, int What) {
	uint i=1;
	uint iEnd=RawSignal[iTimeRange];
	if (RawSignal[RawIndexStart] < 10) {
		return 0;
	}
	if (RawIndexStart + RawSignal[RawIndexStart] >= iTimeRange) {
		PrintNum(RawIndexStart,false, 3);
		PrintNum(RawIndexStart + RawSignal[RawIndexStart],true, 3);
		PrintNum(iTimeRange,true, 3);
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
			uint x = 5 + RawIndexStart;
			uint xEnd = RawSignal[RawIndexStart] + RawIndexStart;

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
			for (x = 3 + RawIndexStart; x <= xEnd-4; x+=2) {
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


void PrintComma(void)
  {
  Serial.print(", ");
  }

void PrintNum(uint x, bool fPrintComma, uint digits) {
     // Rinie add space for small digits
     if(fPrintComma) {
     	PrintComma();
 	}
	for (uint i=0, val=10; i < digits; i++, val *= 10) {
		if (x < val) {
			PrintChar(' ');
		}
	}

    Serial.print(x,DEC);
}

void PrintRawSignal(uint RawIndexStart) {
	uint x;
	uint i;
	bool fPrintPulseAndSpace = true;
	uint  xEnd = RawSignal[RawIndexStart] + RawIndexStart;
#ifdef RAW_BUFFER_TIMERANGE_START
	int iTimeRange = RAW_BUFFER_TIMERANGE_START;
	RawSignal[iTimeRange] = 0;
#endif
	//total time
	if ((RawIndexStart > RAW_BUFFER_SIZE+2) || (xEnd > RAW_BUFFER_SIZE+2)) {
		PrintNum(RawIndexStart,false, 3);
		PrintNum(xEnd,true, 3);
		Serial.print("PrintRawSignal Overflow\n");
		return;
	}
	i = 0;
	for(int x=1+RawIndexStart;x<=xEnd;x++) {
		i += RawSignal[x];
	}

	if (i <= 0) {
		return;
	}
	if (RawIndexStart <= 0) {
		//PrintEventCode(AnalyzeRawSignal(0));
		//PrintTerm();
		  ClockRead();
		PrintDateTime();
		PrintTerm();
		Serial.print("* ");
		// inter message time
		PrintNum(RawStartSignalTime - RawStartSignalTimeLast,false, 5);
		RawStartSignalTimeLast = RawStartSignalTime;
	}
	else {
		Serial.print("! ");
		// intra message
		PrintNum(RawSignal[xEnd+1],false, 5);
	}
//	PrintComma();
	//total time
	i = 0;
	for(int x=1+RawIndexStart;x<=xEnd;x++) {
		i += RawSignal[x];
	}
	PrintNum(i,true, 5);
//	PrintComma();

	// count
	PrintNum(RawSignal[RawIndexStart],true, 2);
//	PrintComma();

	// net count min spikes
	i = 0;
	for(int x=1+RawIndexStart;x<=xEnd;x++) {
		if (RawSignal[x] < 100) {
			i++;
		}
	}

	PrintNum(RawSignal[RawIndexStart]-i,true, 0);
//	PrintComma();

	PrintNum(i,true, 0);
	PrintComma();

	PrintEventCode(AnalyzeRawSignal(RawIndexStart));
	// todo print min/max and minButOne/maxButOne
	RawSignal_2_32bit(RawIndexStart, true);
//	PrintTerm();
	for (i=0; i < 2; i++) {
		//  PrintText(Text_07,false);
		for(int x=1+RawIndexStart;x<=xEnd;x++) {
			if ((x - (1+RawIndexStart))%16==0) {
					PrintTerm();
					PrintNum(x - (1+RawIndexStart), false, 4);
					PrintChar(':');
					PrintNum(RawSignal[x], false, 4);
			}
			else {
				PrintNum(RawSignal[x], true, 4);
			}
			if (fPrintPulseAndSpace) { // mark + space
				if ((x - (1+RawIndexStart))%2==1) {
					Serial.print(" [");
					PrintNum(RawSignal[x] + RawSignal[x-1], false, 4);
					PrintChar(']');
				}
			}

		}
		PrintTerm();
		if (i == 0) {
			if (RawSignal[iTimeRange] != 0) {
				RkrTimeRangePsReplaceMedian(RawIndexStart);
				Serial.print("!Rounded");
			}
			else {
				i = 2;
			}
		}
	}
}

