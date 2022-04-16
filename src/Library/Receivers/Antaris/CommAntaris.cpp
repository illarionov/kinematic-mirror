
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


#include "CommAntaris.h"


CommAntaris::CommAntaris(Stream& s)
: Comm(s)
{
	ErrCode = Open();
}

CommAntaris::~CommAntaris()
{
	// Put the device back into NMEA mode
}


static int AntarisBaudRate[] = {19200, 57600, 9600, 113400, 0};

bool CommAntaris::Open()
{
	if (com.GetError() != OK) return Error();
	if (ReadOnly()) return OK;

	// When Antaris starts, it sends a whole bunch of messages.
	//   Let them go by before trying to talk to the antaris
	//   Otherwise, we may not see the response to the commands we sent
	com.Purge();

	// Find Nmea messages at some baud rate. (Query and get response)
	int baud = com.FindBaudRate("$EIGPQ,RMC*3A\r\n", "$GPRMC", AntarisBaudRate);
	if (baud == 0) return Error("Not communicating with Antaris at any baud rate\n");

	return OK;
}


bool CommAntaris::GetBlock(Block& b)
{
start_read:
	// read the first header byte
	byte c, c2;
	if (com.Read(c) != OK) return Error();

	// We may have NMEA messages interspersed, so check for one.
	if (c == '$') {
		if (com.SkipLine() != OK) return Error();
		goto start_read;
	}

restart:
	if (c != 0xB5)         goto start_read;

    // read the second header byte
	if (com.Read(c) != OK) return Error();
	if (c != 0x62)         goto restart;

	// read the packet class and type
	if (com.Read(c) != OK) return Error();
      if (com.Read(c2) != OK) return Error();
	b.Id = (((int)c)<<8)+c2;

	// read the length (little endian)
	if (com.Read(c) != OK) return Error();
	if (com.Read(c2) != OK) return Error();
	b.Length = (((int)c2)<<8)+c;

	// Make sure this packet type has the correct length
	//  (TODO - use knowledge about packet types)
	if (b.Length > sizeof(b.Data))
		goto restart;

	// read the packet contents
	if (com.Read(b.Data, b.Length) != OK) return Error();

	// Calculate the expected checksum
    byte ck_a, ck_b;
    CheckSum(b, ck_a, ck_b);

    // Read the real checksum and make sure it matches
	if (com.Read(c) != OK)  return Error();
	if (c != ck_a)           goto restart;
	if (com.Read(c) != OK) return Error();
	if (c != ck_b)           goto restart;

	b.Display("Read Antaris Block");

	// done
	return OK;
}


bool CommAntaris::PutBlock(Block& b)
{
	b.Display("Writing Antaris block");
	// Send the header characters
	if (com.Write(0xB5) != OK) return Error();
	if (com.Write(0x62) != OK) return Error();

	// Send the Id (class followed by id)
    if (com.Write(b.Id>>8) != OK) return Error();
	if (com.Write(b.Id) != OK)  return Error();

	// Send the length (little endian)
	if (com.Write(b.Length) != OK) return Error();
	if (com.Write(b.Length>>8) != OK) return Error();

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


inline void AddByte(byte b, byte& ck_a, byte& ck_b)
{
    ck_a += b;
    ck_b += ck_a;
}

void CommAntaris::CheckSum(Block& b, byte& ck_a, byte& ck_b)
{
	ck_a = 0;
    ck_b = 0;

      AddByte(b.Id>>8, ck_a, ck_b);
      AddByte(b.Id, ck_a, ck_b);
      AddByte(b.Length, ck_a, ck_b);
	  AddByte(b.Length>>8, ck_a, ck_b);


	for (int i=0; i<b.Length; i++)
		AddByte(b.Data[i], ck_a, ck_b);
}

