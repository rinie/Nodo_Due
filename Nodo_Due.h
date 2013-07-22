#define RAWSIGNAL_TOGGLE	// RKR instead of just one rawsignal, repeat until you send RawSignalGet; again...
#define RAWSIGNAL_MULTI		// RKR Rawsignal can be multiple signals: <count> <timing data> <count> <timing data> etc...

#undef USERVAR // RKR make optional to save space
#undef WIRED // RKR make optional to save space
#undef CLOCK // RKR make optional to save space

#undef USERTIMER
//#define RAW_BUFFER_SIZE            472 // Maximaal aantal te ontvangen bits*2
#define RAW_BUFFER_SIZE            236 // Maximaal aantal te ontvangen bits*2
#define RAW_BUFFER_TIMERANGE_SIZE	36 // RKR pulse time range calculations
#define RAW_BUFFER_TIMERANGE_START	(RAW_BUFFER_SIZE + 4) // RKR borrow same array
typedef unsigned long ulong; //RKR U N S I G N E D is so verbose
typedef unsigned int uint;
#define AVR_LIRC 1
#define AVR_LIRC_BINARY 1