/**************************************************************************\

    This file is part of Nodo Due, Â© Copyright Paul Tonkes

    Nodo Due is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Nodo Due is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Nodo Due.  If not, see <http://www.gnu.org/licenses/>.

\**************************************************************************/


 /******************************welcome***************************************************************\
 * Print een event volgens formaat:  'EVENT/ACTION: <port>, <type>, <content>
 \*********************************************************************************************/
void PrintEvent(unsigned long Content, byte Port, byte Direction)
  {
  byte x,first=0;

  // als ingesteld staat dat seriële input niet weergegeven moet worden en de poort was serieel, dan direct terug
  if(!(S.Display&DISPLAY_SERIAL) && Port==VALUE_SOURCE_SERIAL && Direction==VALUE_DIRECTION_INPUT)
    return;

  // datum en tijd weergeven
  if(S.Display&DISPLAY_TIMESTAMP && Time.Day) // Time.Day=true want dan is er een RTC aanwezig.
    {
    if(S.Display&DISPLAY_TAG)
      PrintText(Text_10);
    PrintDateTime();
    first++;
    }

  // geef aan of de simulatie mode aan stond
  if(Simulate)
    {
    if(first++)
      {
      PrintChar(',');
      PrintChar(' ');
      }
    Serial.print(cmd2str(CMD_SIMULATE));
    PrintChar('=');
    Serial.print(cmd2str(VALUE_ON));
    }

  // Geef de richting van de communicatie weer
  if(S.Display&DISPLAY_DIRECTION)
    {
    if(first++)
      {
      PrintChar(',');
      PrintChar(' ');
      }
    if(S.Display&DISPLAY_TAG)
      PrintText(Text_11);
    Serial.print(cmd2str(Direction));
    if(Direction==VALUE_SOURCE_QUEUE)
      {
      PrintChar('-');
      PrintValue(QueuePos+1);
      }
    }

  // geef de source van het event weer
  if(S.Display&DISPLAY_SOURCE && Port)
    {
    if(first++)
      {
      PrintChar(',');
      PrintChar(' ');
      }
    if(S.Display&DISPLAY_TAG)
      PrintText(Text_12);
    Serial.print(cmd2str(Port));
    }

  // geef unit nummer weer
  if(S.Display&DISPLAY_UNIT)
    {
    // Voor NIET-nodo signalen is het afdrukken van het unitnummer zinloos
    if(((Content>>28)&0xf)==SIGNAL_TYPE_NODO && ((Content>>16)&0xff)!=CMD_KAKU_NEW && ((Content>>16)&0xff)!=CMD_KAKU)
      {
      if(first++)
        {
        PrintChar(',');
        PrintChar(' ');
        }
      if(S.Display&DISPLAY_TAG)
          {
          Serial.print(cmd2str(CMD_UNIT));
          PrintChar('=');
          }
      PrintValue((Content>>24)&0xf);
      }
    }

  if(first++)
    {
    PrintChar(',');
    PrintChar(' ');
    }

  if(S.Display&DISPLAY_TAG)
    PrintText(Text_14);
  PrintEventCode(Content);
  PrintTerm();
  }


 /*********************************************************************************************\
 * print een lijst met de inhoud van de RawSignal buffer.
 *
 * RKR support multiple repeated signals
 * Print statistics before timing data
 \*********************************************************************************************/
#if 0
void PrintRawSignal(void)
  {
  PrintText(Text_07);
  for(byte x=1;x<=RawSignal[0];x++)
     {
     if(x>1)PrintChar(',');
     PrintValue(RawSignal[x]);
     }
  PrintTerm();
  }
#else
void PrintComma(void)
  {
  Serial.print(", ");
  }

void PrintNum(unsigned int x, bool fPrintComma, unsigned int digits) {
     // Rinie add space for small digits
     if(fPrintComma) {
     	PrintComma();
 	}
	for (unsigned int i=0, val=10; i < digits; i++, val *= 10) {
		if (x < val) {
			PrintChar(' ');
		}
	}

    Serial.print(x,DEC);
}

void PrintRawSignal(int RawIndexStart) {
	unsigned int x;
	unsigned int i;
	bool fPrintPulseAndSpace = true;
	unsigned int  xEnd = RawSignal[RawIndexStart] + RawIndexStart;
#ifdef RAW_BUFFER_PULSELEN_START
	int iPulse = RAW_BUFFER_PULSELEN_START;
	RawSignal[iPulse] = 0;
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
			if (RawSignal[iPulse] != 0) {
				RkrMinMaxPsReplaceMedian(RawIndexStart);
				Serial.print("!Rounded");
			}
			else {
				i = 2;
			}
		}
	}
}

#endif
 /*********************************************************************************************\
 * Print een decimaal getal
 * Serial.Print neemt veel progmem in beslag.
 \*********************************************************************************************/
void PrintValue(unsigned long x)
  {
  if(x<=255)
    Serial.print(x,DEC);
  else
    {
    Serial.print("0x");
    Serial.print(x,HEX);
    }
  }

 /*********************************************************************************************\
 * Print een lijn met het teken '*'
 \*********************************************************************************************/
void PrintLine(void)
  {
  for(byte x=1;x<=60;x++)PrintChar('*');
  PrintTerm();
  }


 /*********************************************************************************************\
 * Serial.Print neemt veel progmem in beslag.
 * Print het teken '0'
 \*********************************************************************************************/
void PrintChar(byte S)
  {
  Serial.print(S, BYTE);
  }


 /*********************************************************************************************\
 * Print een EventCode. de MMI van de Nodo
 * Een Evencode wordt weergegeven als "(<commando> <parameter-1>,<parameter-2>)"
 \*********************************************************************************************/
#define P_NOT   0
#define P_TEXT  1
#define P_KAKU  2
#define P_VALUE 3
#define P_DIM   4

void PrintEventCode(unsigned long Code)
  {
  byte P1,P2,Par2_b;
  boolean P2Z=true;     // vlag: true=Par2 als nul waarde afdrukken false=nulwaarde weglaten

  byte Type     = (Code>>28)&0xf;
  byte Command  = (Code>>16)&0xff;
  byte Par1     = (Code>>8)&0xff;
  byte Par2     = (Code)&0xff;

  PrintChar('(');

  if(Type==SIGNAL_TYPE_NEWKAKU)
    {
    // Aan/Uit zit in bit 5
    Serial.print(cmd2str(CMD_KAKU_NEW));
    PrintChar(' ');
    PrintValue(Code&0x0FFFFFEF);
    PrintChar(',');
    Serial.print(cmd2str(((Code>>4)&0x1)?VALUE_ON:VALUE_OFF));
    }

  else if(Type==SIGNAL_TYPE_NODO || Type==SIGNAL_TYPE_KAKU)
    {
    Serial.print(cmd2str(Command));
    switch(Command)
      {
      // Par1 als KAKU adres [A0..P16] en Par2 als [On,Off]
      case CMD_KAKU:
      case CMD_SEND_KAKU:
        P1=P_KAKU;
        P2=P_TEXT;
        Par2_b=Par2;
        break;

      case CMD_KAKU_NEW:
      case CMD_SEND_KAKU_NEW:
        P1=P_VALUE;
        P2=P_DIM;
        break;

      // Par1 als waarde en par2 als tekst
      case CMD_DELAY:
      case CMD_WIRED_PULLUP:
      case CMD_WIRED_OUT:
      case CMD_WIRED_IN_EVENT:
        P1=P_VALUE;
        P2=P_TEXT;
        break;

      // Par1 als tekst en par2 als tekst
      case CMD_COMMAND_WILDCARD:
        P1=P_TEXT;
        P2=P_TEXT;
        break;

      // Par1 als tekst en par2 als getal
      case CMD_ERROR:
      case CMD_COPYSIGNAL:
      case CMD_TRANSMIT_SETTINGS:
      case CMD_STATUS:
        P1=P_TEXT;
        P2=P_VALUE;
        break;

      // Par1 als tekst en par2 niet
      case CMD_DLS_EVENT:
      case CMD_BUSY:
      case CMD_SENDBUSY:
      case CMD_WAITBUSY:
      case CMD_SIMULATE:
        P1=P_TEXT;
        P2=P_NOT;
        break;

      // Par1 als waarde en par2 niet
      case CMD_UNIT:
      case CMD_DIVERT:
      case CMD_SIMULATE_DAY:
      case CMD_CLOCK_DOW:
        P1=P_VALUE;
        P2=P_NOT;
        break;

      // Geen parameters
      case CMD_SEND_SIGNAL:
      case CMD_BOOT_EVENT:
        P1=P_NOT;
        P2=P_NOT;
        break;

      // Par1 als waarde en par2 als waarde
      default:
        P1=P_VALUE;
        P2=P_VALUE;
      }

    // Print Par1
    if(P1!=P_NOT)
      {
      PrintChar(' ');
      switch(P1)
        {
        case P_TEXT:
          Serial.print(cmd2str(Par1));
          break;
        case P_VALUE:
          PrintValue(Par1);
          break;
        case P_KAKU:
          Serial.print('A'+((Par1&0xf0)>>4),BYTE); //  A..P printen.
          PrintValue(Par2_b&0x2?0:(Par1&0xF)+1);// als 2e bit in Par2 staat, dan is het een groep commando en moet adres '0' zijn.
          Par2=(Par2&0x1)?VALUE_ON:VALUE_OFF;
          break;
        }
      }// P1

    // Print Par2
    if(P2!=P_NOT)
      {
      PrintChar(',');
      switch(P2)
        {
        case P_TEXT:
          Serial.print(cmd2str(Par2));
          break;
        case P_VALUE:
          PrintValue(Par2);
          break;
        case P_DIM:
          {
          if(Par2==VALUE_OFF || Par2==VALUE_ON)
            Serial.print(cmd2str(Par2)); // Print 'On' of 'Off'
          else
            PrintValue(Par2);
          break;
          }
        }
      }// P2
    }//   if(Type==SIGNAL_TYPE_NODO || Type==SIGNAL_TYPE_OTHERUNIT)

  else // wat over blijft is het type UNKNOWN.
    PrintValue(Code);

  PrintChar(')');
  }


 /**********************************************************************************************\
 * Verzend teken(s) naar de seriele poort die een regel afsluiten.
 \*********************************************************************************************/
void PrintTerm()
  {
  // FreeMemory();//??? t.b.v. debugging
  if(SERIAL_TERMINATOR_1)Serial.print(SERIAL_TERMINATOR_1,BYTE);
  if(SERIAL_TERMINATOR_2)Serial.print(SERIAL_TERMINATOR_2,BYTE);
  }


 /**********************************************************************************************\
 * Print een regel uit de Eventlist.
 \*********************************************************************************************/
void PrintEventlistEntry(int entry, byte d)
  {
  unsigned long Event, Action;

  Eventlist_Read(entry,&Event,&Action); // leesregel uit de Eventlist.

  // Geef de entry van de eventlist weer
  if(S.Display&DISPLAY_TAG)
    PrintText(Text_03);
  if(d>1)
    {
    PrintValue(d);
    PrintChar('.');
    }
  PrintValue(entry);

  // geef het event weer
  PrintChar(',');
  PrintChar(' ');
  if(S.Display&DISPLAY_TAG)
    PrintText(Text_14);
  PrintEventCode(Event);

  // geef het action weer
  PrintChar(',');
  PrintChar(' ');
  if(S.Display&DISPLAY_TAG)
    PrintText(Text_16);
  PrintEventCode(Action);
  }

 /**********************************************************************************************\
 * Print actuele dag, datum, tijd.
 \*********************************************************************************************/
void PrintDateTime(void)
    {
    // Print de dag. 1=zondag, 0=geen RTC aanwezig
    for(byte x=0;x<=2;x++)Serial.print(*(Text(Text_04)+(Time.Day-1)*3+x),BYTE);
    PrintChar(' ');

    // print year.
    Serial.print(Time.Year,DEC);
    PrintChar('-');

    // print maand.
    if(Time.Month<10)PrintChar('0');
    PrintValue(Time.Month);
    PrintChar('-');

    // print datum.
    if(Time.Date<10)PrintChar('0');
    PrintValue(Time.Date);
    PrintChar(' ');

    // print uren.
    if(Time.Hour<10)PrintChar('0');
    PrintValue(Time.Hour);
    PrintChar(':');

    // print minuten.
    if(Time.Minutes<10)PrintChar('0');
    PrintValue(Time.Minutes);
    // RKR print seconds
    PrintChar(':');
    if(Time.Seconds<10)PrintChar('0');
    PrintValue(Time.Seconds);
    }

 /**********************************************************************************************\
 * Print de welkomsttekst van de Nodo.
 \*********************************************************************************************/
void PrintWelcome(void)
  {
  // Print Welkomsttekst
  PrintTerm();
  PrintLine();

  PrintText(Text_01);
  PrintTerm();

  PrintText(Text_02);
  PrintTerm();

  PrintText(Text_15);
  PrintValue(S.Version/100);
  PrintChar('.');
  PrintValue((S.Version%100)/10);
  PrintChar('.');
  PrintValue(S.Version%10);
  PrintChar(',');
  PrintChar(' ');
  PrintText(Text_13);
  PrintValue(S.Unit);
  PrintTerm();

  if(Time.Day)
    {
    PrintText(Text_10);
    PrintDateTime();
    if(Time.DaylightSaving)
      {
      PrintChar(',');
      PrintChar(' ');
      Serial.print(cmd2str(CMD_DLS_EVENT));
      }
    PrintTerm();
    }
#ifdef RAWSIGNAL_TOGGLE
	Serial.print("rawsignalget; on");
    PrintTerm();
#endif
  PrintLine();
  }

 /**********************************************************************************************\
 * Print een string uit PROGMEM
 \*********************************************************************************************/
void PrintText(prog_char* text)
  {
  byte x=0;
  char buffer[60];

  do
    {
    buffer[x]=pgm_read_byte_near(text+x);
    }while(x < 60 && (buffer[x++]!=0)); // RKR prevent overflow
  buffer[59] = 0;
  Serial.print(buffer);
  }

