// Part of Kinematic, a utility for GPS positioning
//
// Copyright (C) 2006  John Morris    www.precision-gps.org
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation, version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.


#include "Stream.h"
#include <stdarg.h>


int StreamValidBaud[] = {38400, 19200, 9600, 4800, 56000, 
		  115200, 2400, 1200, 57600, 256000, 128000, 300, 110, 0};


// Fixed length read
bool Stream::Read(byte* buf, size_t len)
{
    size_t actual;
    for (size_t remaining=len; remaining > 0; remaining-=actual, buf+=actual) {
        if (Read(buf, remaining, actual) != OK) return Error();
        if (actual == 0) return Error("Stream::fixed length read timed out\n");
    }

    return OK;
}



bool Stream::ReadLine(char *buf, size_t len)
{
	size_t actual;
	
	// Repeat until buffer is full or C/R received
	for (actual = 0;  actual < len-1; )
	{
        // Read a byte; if Newline, done
		byte c;
		if (Read(c) != OK) return Error();
		if (c == '\n') 
			break;

		// If not a CR, save it in buffer
		if (c != '\r')
			buf[actual++] = c;
	}

	// Terminate the string
	buf[actual] = 0;

	debug(5,"Stream::ReadLine: len=%d actual=%d buf=%s\n", len,actual,buf);

	return OK;
}


bool Stream::WriteLine(const char* buf)
{
	debug(8,"Port::WriteLine: buf=%s\n", buf);
	return Write(buf)  ||  Write("\r\n");
}

bool Stream::WriteNmea(const char* buf)
{
	// Calculate the checksum, XOR of all bytes except $ and \r\n
	int len = strlen(buf);
	byte cksum = 0;
	for (int i=1; i<len; i++)
		cksum ^= buf[i];

	// Output the string
	if (Printf("%s*%02x\r\n", buf, cksum) != OK) return Error();

	return OK;
}



bool Stream::Printf(const char* format, ...)
{
    // Format a buffer 
	va_list arglist;
	va_start(arglist, format);
	bool ret = VPrintf(format, arglist);
	va_end(arglist);
	return ret;
}

bool Stream::VPrintf(const char* format, va_list arglist)
{
	// Write into a local buffer. We may be invoked in debug code,
	//    and we don't want to be doing malloc's which could interfere.
	char buffer[256];
	buffer[sizeof(buffer)-1] = '\0';
	int actual = vsnprintf(buffer, sizeof(buffer)-1, format, arglist);
	buffer[sizeof(buffer)-1] = '\0';

	// Write out the buffer
	if (Write(buffer) != OK) return Error();
	if (actual == -1) return Error("Stream::Printf buffer overflowed\n");
	
	return OK;
}



bool Stream::QueryResponse(const char *query, const char* response, int TooMany)
{
	debug(8,"QueryResponse  query=%s  response=%s\n", query, response);

	// Send the query
	if (Write(query) != OK)  return Error();

	// Await the response
	if (AwaitString(response, TooMany) != OK)
		return Error("Did not get expected response to query\n");

	return OK;
}

bool Stream::SkipLine()
{
	return AwaitString("\n");
}

bool Stream::AwaitString(const char* response, int TooMany)
{
	debug(8,"AwaitString: response=%s  TooMany=%d\n", response, TooMany);
	// Make sure the expected response size is within range
	int len = strlen(response);
	if (len >= 500) return Error("Expected response is too long");

	// Read the response
	byte buf[500];
	if (Read(buf, len) != OK) return Error();
	buf[len] = '\0';

	// Repeat until we timeout or have a match
	//   (Note: this is quick+dirty. There are far more efficient ways to do it)
	for (int i=0; TooMany == 0 || i<TooMany; i++) {

		// If we have a match, then done
	    if (strcmp((const char*)buf, response) == 0)
			return OK;

		// shift the buffer down a byte.
		for (int i=1; i<len; i++)
			buf[i-1] = buf[i];

		// read a new byte to end of buffer
		if (Read(buf[len-1]) != OK) return Error();
	}

	return Error("Didn't receive expected response");
}


bool Stream::PrintSvid(int s)
{
	int svid = SatToSvid(s);
	if (svid <= 32)
		return Printf("G%02d", svid);
	else if (120 <= svid && svid <= 152)
		return Printf("S%02d", svid-100);
	else
		return Printf("###");
}

bool Stream::PrintTime(Time time)
{
	int32 year, month, day, hour, min, sec, nsec;
	TimeToDate(time, year,month, day);
	TimeToTod(time, hour, min, sec, nsec);
	return Printf("%02d/%02d/%04d-%02d:%02d:%02d", month,day,year,hour,min,sec);
}

