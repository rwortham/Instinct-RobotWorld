// the arduino code uses snprintf_P and sscanf_P because these AVR library functions read the format string from program memory, thus saving valiable RAM.
// the format strings are defined as const static char PROGMEM szFmt[] = {"<fmt>"} so we also need to #define PROGMEM as blank for VC++
#ifndef  _INSTINCT_VCPP_
#define _INSTINCT_VCPP_

#define PROGMEM

int snprintf_P(char * 	_s, size_t 	_n, const char * 	_fmt, ...);
int sscanf_P(const char * _s, const char * _fmt, ...);

#endif // _INSTINCT_VCPP_
