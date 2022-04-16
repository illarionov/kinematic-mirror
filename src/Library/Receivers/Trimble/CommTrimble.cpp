// CommReadTrimble communicates with a Trimble Lassen IQ receiver
//    Part of Kinetic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    kinetic@precision-gps.org
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

#include "CommTrimble.h"


CommTrimble::CommTrimble(Stream& s)
: Comm(s)
{
	ErrCode = Open();
}

CommTrimble::~CommTrimble(void)
{
}

static int TrimbleBaud[] = {57600, 56000, 38400, 19200, 9600, 4800, 0};

bool CommTrimble::Open()
{
	// open the rs-232 port and set it to 8 data bits, odd parity, 1 stop bit
	if (com.SetFraming(8, 1, 1) != OK) return Error();

	// Set the timeout so we know if we lose connection
	if (com.SetTimeout(2000) != OK) return Error();
	com.SetBaud(TrimbleBaud[0]);

	// Find the matching baud rate by listening for binary mode (DLE ETX)
	int baud = com.FindBaudRate("", "\x10\x03", TrimbleBaud);
	return OK;
}

bool CommTrimble::GetBlock(Block& b)
//////////////////////////////////////////////////////////////////////
// GetBlock reads a block of binary data from a trimble gps receiver
//////////////////////////////////////////////////////////////////////
{
restart:

	// read a DLE 
	byte c;
	if (com.Read(c) != OK) 
		return Error();
	if (c != DLE) 
		goto restart;
		
	// read the Id (not ETX and not DLE)
	byte id;
	if (com.Read(id) != OK) 
		return Error();
	if (id == ETX || id == DLE) 
		goto restart;
	b.Id = id;

	// Read data until DLE ETX is encountered
	byte prev = 0;
	for (b.Length=0; b.Length<b.Max;) {
		if (com.Read(c) != OK) 
			return Error();

		// Process the DLE escape sequence if present
		if (prev == DLE && c == ETX)
			break;
		else if (prev == DLE && c == DLE) {
		    b.Data[b.Length++] = c;
			c = 0;  // It is a data byte, don't use it for control
		}
		else if (prev != DLE && c != DLE)
			b.Data[b.Length++] = c;
		else if (prev != DLE && c == DLE)
			;
		else {
			ClearError();
			Error("Trimble protocol error\n");
			goto restart;
		}

		prev = c;
	}

	if (b.Length >= b.Max)
		return Error("block too large\n");

	return OK;
}

		

bool CommTrimble::PutBlock(Block& b)
//////////////////////////////////////////////////////////////////////
// PutBlock writes a block of binary data to a Trimble gps receiver
//////////////////////////////////////////////////////////////////////
{
	b.Display("CommTrimble - Output Block");
	com.Write(DLE);
	com.Write(b.Id);
	for (int i=0; i<b.Length; i++){
		com.Write(b.Data[i]);
		if (b.Data[i] == DLE)
			com.Write(DLE);
	}
	com.Write(DLE);
	com.Write(ETX);

	return OK;
}



