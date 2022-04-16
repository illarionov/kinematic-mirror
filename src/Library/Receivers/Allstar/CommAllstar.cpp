
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


#include "CommAllstar.h"


CommAllstar::CommAllstar(Stream& s)
: Comm(s)
{
	ErrCode = Open();
}

CommAllstar::~CommAllstar()
{
	// Put the device back into NMEA mode
}


static int AllstarBaudRate[] = {19200, 38400, 9600, 4800, 0};

bool CommAllstar::Open()
{
	if (com.GetError() != OK) return Error();
	if (ReadOnly()) return OK;

	if (com.SetTimeout(2000) != OK) return Error();

	// Put into binary mode and look for SOH (or issue command and wait for ack)
	int baud = com.FindBaudRate("$Pxxxxxx*99\r\n", "\x01", AllstarBaudRate);
	if (baud == 0) return Error("Can't communicate with Allstar.\n");

	return OK;
}


bool CommAllstar::GetBlock(Block& b)
{
// Note: We rely on the fact SOH (1) and ~SOH (254) are not valid message id's

start_read:
	// read the first header byte, hopefully an SOH
	byte c, c2;
	if (com.Read(c) != OK) return Error();

	// We may have NMEA messages interspersed, so check for one. // verify this happens
	if (c == '$') {
		if (com.SkipLine() != OK) return Error();
		goto start_read;
	}

	// We should have read an SOH
	if (c != SOH)         goto start_read;
GotSOH:

	// Now, read the id byte
	if (com.Read(c) != OK) return Error();
	if (c == SOH) goto GotSOH;

	// read the complement of the ID byte
	if (com.Read(c2) != OK) return Error();
	if (c2 == SOH) goto GotSOH;
	if (c != ~c2) goto start_read;

	// We now have a block header up to the Id
	b.Id = c;

	// Get the length
    if (com.Read(c) != OK) return Error();
	b.Length = c;

	// Make sure this packet type has the correct length
	//  (TODO - use knowledge about packet types)

	// read the packet contents
	if (com.Read(b.Data, b.Length) != OK) return Error();

	// Calculate the expected checksum
    byte ck_a, ck_b;
    CheckSum(b, ck_a, ck_b);

    // Read the real checksum and make sure it matches
	if (com.Read(c) != OK)  return Error();
	if (com.Read(c2) != OK) return Error();
	if (c != ck_a && c2 != ck_b)  goto start_read;

	b.Display("Read Allstar Block");

	// done
	return OK;
}


bool CommAllstar::PutBlock(Block& b)
{
	b.Display("Writing Allstar block");
	if (b.Length > 255) return Error("Can't write blocks larger than 255\n");
	// Send the header characters
	if (com.Write(SOH) != OK) return Error();
	if (com.Write(b.Id) != OK) return Error();
	if (com.Write(~b.Id) != OK) return Error();

	// Send the length (little endian)
	if (com.Write(b.Length) != OK) return Error();

	// Send the data
	if (com.Write(b.Data, b.Length) != OK) return Error();

	// Send the checksum
    byte ck_a, ck_b;
	CheckSum(b, ck_a, ck_b);
	if (com.Write(ck_a) != OK) return Error();
	if (com.Write(ck_b) != OK) return Error();

	// done
	return OK;
}


void CommAllstar::CheckSum(Block& b, byte& ck_a, byte& ck_b)
{
	uint16 sum = 0;
	for (int i=0; i<b.Length; i++)
		sum += b.Data[i];

	ck_a = sum;
	ck_b = sum>>8;
}

