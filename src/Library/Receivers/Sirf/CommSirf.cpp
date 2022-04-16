
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


#include "CommSirf.h"

 
CommSirf::CommSirf(Stream& s)
:Comm(s)
{
	ErrCode = Open();
	if (ErrCode != OK) return;

	// Select a high baud rate
	//  We are assuming bluetooth, so this is a simulated baud rate)
	//  Slome bluetooth drivers slow down the data rate.
	//  DO NOT CHANGE THE DEVICE BAUD RATE FOR BLUETOOTH DEVICES.
	//  THE BLUETOOTH CONTROLLER WILL NO NO LONGER COMMUNICATE WITH DEVICE!!

	// Request the device's id and firmware level
	// (later)
}

CommSirf::~CommSirf()
{

	// Put the device back into NMEA mode
}

static int SirfBaud[] = {38400, 9600, 57600, 19200, 4800, 115200, 0};

bool CommSirf::Open()
{
	if (com.GetError() != OK) return Error();
	if (com.ReadOnly()) return OK;


	if (com.SetTimeout(2000) != OK) return Error();

	// Scan the baud rates looking for a match.(Put in binary + look for msg)
	int baud = com.FindBaudRate("$PSRF100,0*3A\r\n", "\xB0\xB3", SirfBaud);
	if (baud == 0)
		return Error("Unable to communicate with Sirf at any baud rate");
	debug("CommSirf: Talking at baud %d\n", baud);

	return OK;
}


bool CommSirf::GetBlock(Block& b)
{
	debug("CommSirf::GetBlock - starts\n");
start_read:
	// read the first header byte
	byte c, c2;
	if (com.Read(c) != OK) return Error();
restart:
	if (c != 0xA0)         goto start_read;

    // read the second header byte
	if (com.Read(c) != OK) return Error();
	if (c != 0xA2)         goto restart;

	// read the length
	if (com.Read(c) != OK) return Error();
	if (com.Read(c2) != OK) return Error();
	b.Length = (((int)c)<<8)+c2-1;

	// read the packet type
	if (com.Read(c) != OK) return Error();
	b.Id = c;

	// Make sure this packet type has the correct length
	//  (later)
	if (b.Length > sizeof(b.Data)) {
		ClearError();
		Error("CommSirf: Bad packet length (%d)\n", b.Length);
		goto restart;
	}

	// read the packet contents
	if (com.Read(b.Data, b.Length) != OK) return Error();

	// read the checksum
	if (com.Read(c) != OK)  return Error();
	if (c > 0x7f)           goto restart;
	if (com.Read(c2) != OK) return Error();
	int checksum =  ((int)c<<8) + c2;

	// Make sure the checksums match
	if (checksum != CheckSum(b)) goto restart;

	// Read the trailer
	if (com.Read(c) != OK) return Error();
	if (c != 0xB0)         goto restart;
	if (com.Read(c) != OK) return Error();
	if (c != 0xB3)         goto restart;

	b.Display("Read Sirf Block");
	// done
	return OK;
}


bool CommSirf::PutBlock(Block& b)
{
	b.Display("Writing Sirf block");
	// Send the header characters
	if (com.Write(0xA0) != OK) return Error();
	if (com.Write(0xA2) != OK) return Error();

	// Send the length
	if (com.Write( (b.Length+1)>>8 ) != OK) return Error();
	if (com.Write(b.Length+1) != OK) return Error();

	// Send the Id
	if (com.Write(b.Id) != OK)  return Error();

	// Send the data
	if (com.Write(b.Data, b.Length) != OK) return Error();

	// Send the checksum
	int checksum = CheckSum(b);
	if (com.Write(checksum>>8) != OK) return Error();
	if (com.Write(checksum) != OK) return Error();

	// Send the trailer
	if (com.Write(0xB0) != OK) return Error();
	if (com.Write(0xB3) != OK) return Error();

	// done
	return OK;
}



int CommSirf::CheckSum(Block& b)
{
	int checksum = b.Id;
	for (int i=0; i<b.Length; i++)
		checksum += b.Data[i];
	return checksum & 0x7fff;
}

