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


#include "CommFuruno.h"
#include "Rs232.h"


CommFuruno::CommFuruno(Stream& s)
: Comm(s)
{
	ErrCode = Open();
}

CommFuruno::~CommFuruno()
{

}



bool CommFuruno::Open()
{
	if (com.GetError() != OK) return Error();
	if (ReadOnly()) return OK;

    // If this is not an RS232 port, then assume it is a file
    Rs232* port = dynamic_cast<Rs232*>(&com);
    if (port == NULL) return OK;
    
    // The Furuno always talks at 38400 baud
    if ( port->SetBaud(38400) != OK) return Error();
    
    // Occasionally, the Furano hesitates. increase the timeout a bit
    if ( port->SetTimeout(5000) != OK) return Error();
    
    // Read the first block to verify a receiver is out there
    Block dummy;
    if (GetBlock(dummy) != OK) return Error("Can't read from Furuno\n");
    
    debug(3, "CommFuruno -- initialized\n");
	return OK;
}


bool CommFuruno::GetBlock(Block& b)
{
	debug(3, "CommFuruno::GetBlock - Starting\n");
start_read:
	// read the first byte of message
	byte c;
	if (com.Read(c) != OK) return Error();
	debug(3, "CommFuruno - read first byte: 0x%02x(%c)\n", c,c);

restart:	

	// If a "$", then we have an NMEA message. Skip it.
	if (c == '$') {
		if (com.SkipLine() != OK) return Error();
		goto start_read;
	}
	
	// Binary messages begin with the sync character. Start over if not a binary msg.
	if (c != 0x8b)         goto start_read;

    // read the the message type (2nd byte)
	if (com.Read(c) != OK) return Error();
	b.Id = c;
	debug(3, "CommFuruno - message type 0x%02x(%c)\n", c, c);
	
	// Calculate the length based on message type.
	if      (c == 0x50)  b.Length = 266-4;
	else if (c == 0x52)  b.Length = 35-4;
	else                 goto restart;
	
	// read the packet contents
	if (com.Read(b.Data, b.Length) != OK) return Error();

	// Calculate the expected checksum
    byte ck_a, ck_b;
    CheckSum(b, ck_a, ck_b);
    debug(3, "CommFuruno -- expecting checksum characters 0x%02x(%c)  0x%02x(%c)\n",ck_a, ck_a, ck_b, ck_b);

    // Read the real checksum and make sure it matches
	if (com.Read(c) != OK)  return Error();
	debug(3, "CommFuruno - c=0x%02x(%c)\n", c, c);
	if (c != ck_a)           goto restart;
	if (com.Read(c) != OK) return Error();
	debug(3, "CommFuruno - c=0x%02x(%c)\n", c, c);
	if (c != ck_b)           goto restart;

	b.Display("Read Furuno Block");

	// done
	return OK;
}


bool CommFuruno::PutBlock(Block& b)
{
    return Error("CommFuruno::PutBlock - Not implemented\n");
}



void CommFuruno::CheckSum(Block& b, byte& ck_a, byte& ck_b)
{
	unsigned short checksum = b.Id + 0x8b;

    for (int i=0; i<b.Length; i++)
        checksum += b.Data[i];
        
    ck_b = (byte)checksum;  
    ck_a = (byte)(checksum>>8);
    debug("CommFuruno::Checksum - ck_a=0x%x ck_b=0x%x\n", ck_a, ck_b);    
}

