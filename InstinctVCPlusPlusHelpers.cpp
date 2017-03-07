// contains help functions to allow Instinct to compile on VC++ platform
#include <stdafx.h>
#include "InstinctVCPlusPlusHelpers.h"


int snprintf_P(char * _s, size_t _n, const char * _fmt, ...)
{
	int nRtn;
	va_list args;
	va_start(args, _fmt);

	*(_s + _n - 1) = 0; // force a zero term at end of buffer
	nRtn = vsnprintf(_s, _n - 1, _fmt, args);
	va_end(args);
	
	if (nRtn < 0)
	{
		// ideally we should return number of chars that would have been written had the buffer been long enough
		// we don't know this easily, so just return buffer length
		nRtn = _n;
	}
	return nRtn;
}

int sscanf_P(const char * _s, const char * _fmt, ...)
{
	int nRtn;
	va_list args;
	va_start(args, _fmt);

	nRtn = vsscanf(_s, _fmt, args);
	va_end(args);

	return nRtn;
}