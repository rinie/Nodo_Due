  /**************************************************************************\
    This file is part of Nodo Due, (c) Copyright Paul Tonkes

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

// timings NODO signalen
#define NODO_PULSE_0                    500   // PWM: Tijdsduur van de puls bij verzenden van een '0' in uSec.
#define NODO_PULSE_MID                 1000   // PWM: Pulsen langer zijn '1'
#define NODO_PULSE_1                   1500   // PWM: Tijdsduur van de puls bij verzenden van een '1' in uSec. (3x NODO_PULSE_0)
#define NODO_SPACE                      500   // PWM: Tijdsduur van de space tussen de bitspuls bij verzenden van een '1' in uSec.

#define NewKAKU_RawSignalLength      132
#define NewKAKUdim_RawSignalLength   148
#define KAKU_CodeLength    12

ulong AnalyzeRawSignal(uint RawIndexStart)
  {
  ulong Code=0L;

  if(RawSignal[RawIndexStart]>=RAW_BUFFER_SIZE)return 0L;     // Als het signaal een volle buffer beslaat is het zeer waarschijnlijk ruis.

	switch (RawSignal[RawIndexStart]) {
	case 66:
  		Code=RawSignal_2_Nodo(RawIndexStart);
  		break;
  	case KAKU_CodeLength*4+2:
    	Code=RawSignal_2_KAKU(RawIndexStart);
    	break;
    case NewKAKU_RawSignalLength:
    case NewKAKUdim_RawSignalLength:
      	Code=RawSignal_2_NewKAKU(RawIndexStart);
      	break;
 	}
  if (!Code) { // Geen Nodo, KAKU of NewKAKU code. Genereer uit het onbekende signaal een (vrijwel) unieke 32-bit waarde uit.
     Code=RawSignal_2_32bit(RawIndexStart, false);
  }
  return Code;
  }

/*********************************************************************************************\
* Deze routine berekent de uit een RawSignal een NODO code
* Geeft een false retour als geen geldig NODO signaal
\*********************************************************************************************/
ulong RawSignal_2_Nodo(uint RawIndexStart)
  {
  ulong bitstream=0L;
  int x,y,z;
  int xEnd = RawSignal[RawIndexStart] + RawIndexStart;
  // nieuwe NODO signaal bestaat altijd uit start bit + 32 bits. Ongelijk aan 66, dan geen Nodo signaal
  if (RawSignal[RawIndexStart]!=66)return 0L;

  x=3 + RawIndexStart; // 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e pulslengte
  do{
    if(RawSignal[x]>NODO_PULSE_MID)
      bitstream|=(ulong)(1L<<z); //LSB in signaal wordt  als eerste verzonden
    x+=2;
    z++;
    }while(x<xEnd);

  if(((bitstream>>28)&0xf) == SIGNAL_TYPE_NODO)// is het type-nibble uit het signaal gevuld met de aanduiding NODO ?
    return bitstream;
  else
    return 0L;
  }




 /**********************************************************************************************\
 * Deze functie genereert uit een willekeurig gevulde RawSignal afkomstig van de meeste
 * afstandsbedieningen een (vrijwel) unieke bit code.
 * Zowel breedte van de pulsen als de afstand tussen de pulsen worden in de berekening
 * meegenomen zodat deze functie geschikt is voor PWM, PDM en Bi-Pase modulatie.
 * LET OP: Het betreft een unieke hash-waarde zonder betekenis van waarde.
 \*********************************************************************************************/
ulong RawSignal_2_32bit(uint RawIndexStart, bool fPrint) {
	int x,y,z;
	int Counter_pulse=0,Counter_space=0;
	int MinPulse=0xffff;
	int MinSpace=0xffff;
	int MaxPulse=0x0;//RKR
	int MaxSpace=0x0;
	int MinPulseSpace=0xffff;
	int MaxPulseSpace=0x0;//RKR
	int MinPulseP;
	int MinSpaceP;
	ulong CodeP=0L;
	ulong CodeS=0L;
	int xEnd = RawSignal[RawIndexStart] + RawIndexStart;

	// Kleinste, groter dan mid
	// Grootste, kleinder dan mid
	// diff > x?
	// dan replace by median
	// zoek de kortste tijd (PULSE en SPACE)
	x = 5 + RawIndexStart; // 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e puls
	while (x <= xEnd-4) {
		if (RawSignal[x] < MinPulse) {
			MinPulse=RawSignal[x]; // Zoek naar de kortste pulstijd.
		}
		if (RawSignal[x] > MaxPulse) {
			MaxPulse=RawSignal[x]; // Zoek naar de langste pulstijd.
		}

		x++;

		if (RawSignal[x] < MinSpace && RawSignal[x] > 10) {
			MinSpace=RawSignal[x]; // Zoek naar de kortste spacetijd.
		}
		if (RawSignal[x] > MaxSpace) {
			MaxSpace=RawSignal[x]; // Zoek naar de langste spacetijd.
		}

		if (RawSignal[x]+RawSignal[x-1] < MinPulseSpace && RawSignal[x] > 10) {
				MinPulseSpace=RawSignal[x]+RawSignal[x-1]; // Zoek naar de kortste pulse + spacetijd.
		}

		if (RawSignal[x]+RawSignal[x-1] > MaxPulseSpace) {
			MaxPulseSpace = RawSignal[x] + RawSignal[x-1]; // Zoek naar de langste pulse + spacetijd.
		}
		x++;
	}

	MinPulseP = MinPulse; // RKR print without rounding
	MinSpaceP = MinSpace;
	if (MaxPulse - MinPulse < 200) {
		// RKR Original
		MinPulse+=(MinPulse*S.AnalyseSharpness)/100;
	}
	else {
		// try half way
		MinPulse+= (MaxPulse-MinPulse) / 2;
	}

	if (MaxSpace - MinSpace < 200) {
		MinSpace+=(MinSpace*S.AnalyseSharpness)/100;
	}
	else {
		// try half way
		MinSpace+= (MaxSpace-MinSpace) / 2;
	}

	x=3 + RawIndexStart; // 0=aantal, 1=startpuls, 2=space na startpuls, 3=1e pulslengte
	z=0; // bit in de Code die geset moet worden
	do {
		if (z>31) {
			CodeP=CodeP>>1;
			CodeS=CodeS>>1;
		}

		if (RawSignal[x]>MinPulse) {
			if (z <= 31) {// de eerste 32 bits vullen in de 32-bit variabele
				CodeP |= (long)(1L<<z); //LSB in signaal wordt  als eerste verzonden
			}
			else { // daarna de resterende doorschuiven
				CodeP |= 0x80000000L;
			}
			Counter_pulse++;
		}
		x++;

		if (RawSignal[x]>MinSpace) {
			if (z<=31) {// de eerste 32 bits vullen in de 32-bit variabele
				CodeS |= (long)(1L<<z); //LSB in signaal wordt  als eerste verzonden
			}
			else { // daarna de resterende doorschuiven
				CodeS |= 0x80000000L;
			}
			Counter_space++;
		}
		x++;

		z++;
	}
	while (x<xEnd);

	if (fPrint) {
		PrintTerm();
		Serial.print("RAW P ");
		PrintNum(RawSignal[RawIndexStart+1], ' ', 4); // start pulse/preamble
		PrintNum(MinPulseP, ',', 4);
		PrintNum(MaxPulse, ',', 4);
		PrintNum(MaxPulse-MinPulseP, ',', 4);
		PrintNum(Counter_pulse, ',', 4);
		PrintComma();
		PrintValue(CodeP);
#if 1
		RkrTimeRange(MinPulseP, MaxPulse, ixPulse); // Pulse
#else
		PrintNum(iTimeRange, ',', 4);
#endif
		PrintTerm();
		Serial.print("RAW S ");
		PrintNum(RawSignal[RawIndexStart+2], ' ', 4); // start space/preamble
		PrintNum(MinSpaceP, ',', 4);
		PrintNum(MaxSpace, ',', 4);
		PrintNum(MaxSpace-MinSpaceP, ',', 4);
		PrintNum(Counter_space, ',', 4);
		PrintComma();
		PrintValue(CodeS);
#if 1
		RkrTimeRange(MinSpaceP, MaxSpace, ixSpace); // Space
#else
		PrintComma();
		PrintNum(iTimeRange, ',', 4);
#endif
		PrintTerm();
		Serial.print("RAW PS");

		PrintNum(RawSignal[RawIndexStart+1] + RawSignal[RawIndexStart+2], ' ', 4); // start space/preamble
		PrintNum(MinPulseSpace, ',', 4);
		PrintNum(MaxPulseSpace, ',', 4);
		PrintNum(MaxPulseSpace-MinPulseSpace, ',', 4);
		PrintNum(Counter_pulse + Counter_space, ',', 4);
		PrintComma();
		PrintValue(CodeS^CodeP);
#if 1
		RkrTimeRange(MinPulseSpace, MaxPulseSpace, ixPulseSpace); // // Pulse + Space
#else
		PrintNum(iTimeRange, ',', 4);
#endif
	}

	if(Counter_pulse>=1 && Counter_space<=1) {
		return CodeP; // data zat in de pulsbreedte
	}
	if(Counter_pulse<=1 && Counter_space>=1) {
		return CodeS; // data zat in de pulse afstand
	}
	return (CodeS^CodeP); // data zat in beide = bi-phase, maak er een leuke mix van.
}


 /**********************************************************************************************\
 * Opwekken draaggolf van 38Khz voor verzenden IR.
 * Deze code is gesloopt uit de library FrequencyTimer2.h omdat deze library niet meer door de compiler versie 0015 kwam.
 * Tevens volledig uitgekleed.
 \*********************************************************************************************/
static uint8_t enabled = 0;
static void IR38Khz_set()
  {
  uint8_t pre, top;
  ulong period=208; // IR_TransmitCarrier=26 want pulsen van de IR-led op een draaggolf van 38Khz. (1000000/38000=26uSec.) Vervolgens period=IR_TransmitCarrier*clockCyclesPerMicrosecond())/2;  // period =208 bij 38Khz
  pre=1;
  top=period-1;
  TCCR2B=0;
  TCCR2A=0;
  TCNT2=0;
  ASSR&=~_BV(AS2);    // use clock, not T2 pin
  OCR2A=top;
  TCCR2A=(_BV(WGM21)|(enabled?_BV(COM2A0):0));
  TCCR2B=pre;
  }


 /**********************************************************************************************\
 * Deze functie wacht totdat de 433 band vrij is of er een timeout heeft plaats gevonden
 * Window en delay tijd in milliseconden
 \*********************************************************************************************/
# define WAITFREERF_TIMEOUT             30000 // tijd in ms. waarna het wachten wordt afgebroken als er geen ruimte in de vrije ether komt

void WaitFreeRF(int Delay, int Window)
  {
  ulong Timer, TimeOutTimer;

  // WaitFreeRF is zinloos in de simulatie mode
  if(Simulate)return;

  // eerst de 'dode' wachttijd
  Timer=millis()+Delay; // set de timer.
  while(Timer>millis())
    digitalWrite(MonitorLedPin,(millis()>>7)&0x01);

  // dan kijken of de ether vrij is.
  Timer=millis()+Window; // reset de timer.
  TimeOutTimer=millis()+WAITFREERF_TIMEOUT; // tijd waarna de routine wordt afgebroken in milliseconden

  while(Timer>millis() && TimeOutTimer>millis())
    {
    if((*portInputRegister(RFport)&RFbit)==RFbit)// Kijk if er iets op de RF poort binnenkomt. (Pin=HOOG als signaal in de ether).
      {
      if(FetchSignal(RF_ReceiveDataPin,HIGH,SIGNAL_TIMEOUT_RF, 0))// Als het een duidelijk signaal was
        Timer=millis()+Window; // reset de timer weer.
      }
    digitalWrite(MonitorLedPin,(millis()>>7)&0x01);
    }
  }


 /*********************************************************************************************\
 * Deze routine zendt een RAW code via RF.
 * De inhoud van de buffer RawSignal moet de pulstijden bevatten.
 * RawSignal[0] het aantal pulsen*2
 \*********************************************************************************************/

void RawSendRF(void)
  {
  int x;

  if(Simulate)return;
  digitalWrite(RF_ReceivePowerPin,LOW);   // Spanning naar de RF ontvanger uit om interferentie met de zender te voorkomen.
  digitalWrite(RF_TransmitPowerPin,HIGH); // zet de 433Mhz zender aan
  delay(5);// kleine pause om de zender de tijd te geven om stabiel te worden

  for(byte y=0; y<S.TransmitRepeat; y++) // herhaal verzenden RF code
    {
    x=1;
    while(x<=RawSignal[0])
      {
      digitalWrite(RF_TransmitDataPin,HIGH); // 1
      delayMicroseconds(RawSignal[x++]);
      digitalWrite(RF_TransmitDataPin,LOW); // 0
      delayMicroseconds(RawSignal[x++]);
      }
    }
  digitalWrite(RF_TransmitPowerPin,LOW); // zet de 433Mhz zender weer uit
  digitalWrite(RF_ReceivePowerPin,HIGH); // Spanning naar de RF ontvanger weer aan.
  }


 /*********************************************************************************************\
 * Deze routine zendt een 32-bits code via IR.
 * De inhoud van de buffer RawSignal moet de pulstijden bevatten.
 * RawSignal[0] het aantal pulsen*2
 * Pulsen worden verzonden op en draaggolf van 38Khz.
 \*********************************************************************************************/

void RawSendIR(void)
  {
  int x,y;

  if(Simulate)return;

  for(y=0; y<S.TransmitRepeat; y++) // herhaal verzenden IR code
    {
    x=1;
    while(x<=RawSignal[0])
      {
      TCCR2A|=_BV(COM2A0); // zet IR-modulatie AAN
      delayMicroseconds(RawSignal[x++]);
      TCCR2A&=~_BV(COM2A0); // zet IR-modulatie UIT
      delayMicroseconds(RawSignal[x++]);
      }
    }
  }

 /*********************************************************************************************\
 * Deze routine berekend de RAW pulsen van een 32-bit Nodo-code en plaatst deze in de buffer RawSignal
 * RawSignal.Bits het aantal pulsen*2+startbit*2 ==> 66
 *
 \*********************************************************************************************/
void Nodo_2_RawSignal(ulong Code)
  {
  byte BitCounter,y=1;

  // begin met een startbit.
  RawSignal[y++]=NODO_PULSE_1*2;
  RawSignal[y++]=NODO_SPACE*4;

  // de rest van de bits LSB als eerste de lucht in
  for(BitCounter=0; BitCounter<=31; BitCounter++)
    {
    if(Code>>BitCounter&1)
      RawSignal[y++]=NODO_PULSE_1;
    else
      RawSignal[y++]=NODO_PULSE_0;
    RawSignal[y++]=NODO_SPACE;
    }
  RawSignal[y-1]=NODO_PULSE_1*10; // pauze tussen de pulsreeksen
  RawSignal[0]=66; //  1 startbit bestaande uit een pulse/space + 32-bits is 64 pulse/space = totaal 66
  }

 /**********************************************************************************************\
 * Wacht totdat de pin verandert naar status state. Geeft de tijd in uSec. terug.
 * Als geen verandering, dan wordt na timeout teruggekeerd met de waarde 0L
 \*********************************************************************************************/
ulong WaitForChangeState(uint8_t pin, uint8_t state, ulong timeout)
	{
        uint8_t bit = digitalPinToBitMask(pin);
        uint8_t port = digitalPinToPort(pin);
	uint8_t stateMask = (state ? bit : 0);
	ulong numloops = 0; // keep initialization out of time critical area
	ulong maxloops = microsecondsToClockCycles(timeout) / 19;

	// wait for the pulse to stop. One loop takes 19 clock-cycles
	while((*portInputRegister(port) & bit) == stateMask)
		if (numloops++ == maxloops)
			return 0;//timeout opgetreden
	return clockCyclesToMicroseconds(numloops * 19 + 16);
	}



 /**********************************************************************************************\
 * Haal de pulsen en plaats in buffer. Op het moment hier aangekomen is de startbit actief.
 * bij de TSOP1738 is in rust is de uitgang hoog. StateSignal moet LOW zijn
 * bij de 433RX is in rust is de uitgang laag. StateSignal moet HIGH zijn
 *
 * RKR: Added uint RawIndexStart: receive repeated signals in one go
 \*********************************************************************************************/

int FetchSignal(byte DataPin, boolean StateSignal, int TimeOut, uint RawIndexStart) {
	int RawCodeLength=RawIndexStart+1;
	ulong PulseLength;
	if (RawCodeLength>=RAW_BUFFER_SIZE-4) {
		return 0;
	}

	// support for long preamble
	PulseLength=WaitForChangeState(DataPin, StateSignal, 2*TimeOut); // meet hoe lang signaal LOW (= PULSE van IR signaal)
	if (PulseLength<MIN_PULSE_LENGTH) {
			return 0;
	}
	RawSignal[RawCodeLength++]=PulseLength;
	PulseLength=WaitForChangeState(DataPin, !StateSignal, 2*TimeOut); // meet hoe lang signaal HIGH (= SPACE van IR signaal)
	if(PulseLength + RawSignal[RawCodeLength-1] > TimeOut) {
		if(PulseLength + RawSignal[RawCodeLength-1] > TimeOut + 1000) {
			TimeOut = (PulseLength + RawSignal[RawCodeLength-1]);
		}
		else {
			TimeOut = 2*(PulseLength + RawSignal[RawCodeLength-1] + 1000);
		}
	}
	RawSignal[RawCodeLength++]=PulseLength;

	// Original code
	do { // lees de pulsen in microseconden en plaats deze in een tijdelijke buffer
		PulseLength=WaitForChangeState(DataPin, StateSignal, TimeOut); // meet hoe lang signaal LOW (= PULSE van IR signaal)
		if (PulseLength<MIN_PULSE_LENGTH) {
			return 0;
		}
		RawSignal[RawCodeLength++]=PulseLength;
		PulseLength=WaitForChangeState(DataPin, !StateSignal, TimeOut); // meet hoe lang signaal HIGH (= SPACE van IR signaal)
		RawSignal[RawCodeLength++]=PulseLength;
	} while (RawCodeLength<RAW_BUFFER_SIZE && PulseLength!=0);// Zolang nog niet alle bits ontvangen en er niet vroegtijdig een timeout plaats vindt

	if (RawCodeLength-RawIndexStart>=MIN_RAW_PULSES && RawCodeLength<RAW_BUFFER_SIZE) {
		RawSignal[RawIndexStart]=(RawCodeLength-RawIndexStart)-1; // RKR store signal length
		return (RawCodeLength-RawIndexStart)-1;
	}
	RawSignal[RawIndexStart]=0;
	return 0;
}



/**********************************************************************************************\
* Deze functie zendt gedurende Window seconden de IR ontvangst direct door naar RF
* Window tijd in seconden.
\*********************************************************************************************/
void CopySignalIR2RF(byte Window)
  {
  ulong Timer=millis()+((ulong)Window)*1000; // reset de timer.

  digitalWrite(RF_ReceivePowerPin,LOW);   // Spanning naar de RF ontvanger uit om interferentie met de zender te voorkomen.
  digitalWrite(RF_TransmitPowerPin,HIGH); // zet de 433Mhz zender aan
  while(Timer>millis())
    {
    digitalWrite(RF_TransmitDataPin,(*portInputRegister(IRport)&IRbit)==0);// Kijk if er iets op de IR poort binnenkomt. (Pin=LAAG als signaal in de ether).
    digitalWrite(MonitorLedPin,(millis()>>7)&0x01);
    }
  digitalWrite(RF_TransmitPowerPin,LOW); // zet de 433Mhz zender weer uit
  digitalWrite(RF_ReceivePowerPin,HIGH); // Spanning naar de RF ontvanger weer aan.
  }

/**********************************************************************************************\
* Deze functie zendt gedurende Window seconden de RF ontvangst direct door naar IR
* Window tijd in seconden.
\*********************************************************************************************/
#define MAXPULSETIME 50 // maximale zendtijd van de IR-LED in mSec. Ter voorkoming van overbelasting
void CopySignalRF2IR(byte Window)
  {
  ulong Timer=millis()+((ulong)Window)*1000; // reset de timer.
  ulong PulseTimer;

  while(Timer>millis())// voor de duur van het opgegeven tijdframe
    {
    while((*portInputRegister(RFport)&RFbit)==RFbit)// Zolang de RF-pulse duurt. (Pin=HOOG bij puls, laag bij SPACE).
      {
      if(PulseTimer>millis())// als de maximale zendtijd van IR nog niet verstreken
        {
        digitalWrite(MonitorLedPin,HIGH);
        TCCR2A|=_BV(COM2A0);  // zet IR-modulatie AAN
        }
      else // zendtijd IR voorbij, zet IR uit.
        {
        digitalWrite(MonitorLedPin,LOW);
        TCCR2A&=~_BV(COM2A0); // zet IR-modulatie UIT
        }
      }
    PulseTimer=millis()+MAXPULSETIME;
    }
  digitalWrite(MonitorLedPin,LOW);
  TCCR2A&=~_BV(COM2A0);
  }

/**********************************************************************************************\
* verzendt een event en geeft dit tevens weer op SERIAL
* als het Event gelijk is aan 0L dan wordt alleen de huidige inhoud van de buffer als RAW
* verzonden.
\**********************************************************************************************/

boolean TransmitCode(ulong Event,byte SignalType)
  {

  if(SignalType!=SIGNAL_TYPE_UNKNOWN)
    if((S.WaitFreeRF_Window + S.WaitFreeRF_Delay)>=0)
        WaitFreeRF(S.WaitFreeRF_Delay*100, S.WaitFreeRF_Window*100); // alleen WaitFreeRF als type bekend is, anders gaat SendSignal niet goed a.g.v. overschrijven buffer

  switch(SignalType)
    {
    case SIGNAL_TYPE_KAKU:
      KAKU_2_RawSignal(Event);
      break;

    case SIGNAL_TYPE_NEWKAKU:
      NewKAKU_2_RawSignal(Event);
      break;

    case SIGNAL_TYPE_NODO:
      Nodo_2_RawSignal(Event);
      break;

    case SIGNAL_TYPE_UNKNOWN:
      break;

    default:
      return false;
    }

  if(S.TransmitPort==VALUE_SOURCE_RF || S.TransmitPort==VALUE_SOURCE_IR_RF)
    {
    PrintEvent(Event,VALUE_SOURCE_RF,VALUE_DIRECTION_OUTPUT);
    RawSendRF();
    }

  if(S.TransmitPort==VALUE_SOURCE_IR || S.TransmitPort==VALUE_SOURCE_IR_RF)
    {
    PrintEvent(Event,VALUE_SOURCE_IR,VALUE_DIRECTION_OUTPUT);
    RawSendIR();
    }
  }


