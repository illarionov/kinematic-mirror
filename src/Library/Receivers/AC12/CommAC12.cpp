
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



#include "CommAC12.h"
static int AC12Baud[] = {57600, 56000, 4800, 9600, 19200, 38400, 1200, 0};

CommAC12::CommAC12(Stream& s)
    : Comm(s)
{
	ErrCode = Init();
}

bool CommAC12::Init()
{
	if (com.GetError() != OK) return Error();
	if (ReadOnly()) return OK;

	// Set a 2 second timeout on the port
	if (com.SetTimeout(2000) != OK) return Error();

	// Some AC12 have auto baud rate.
	//   Set the desired baud and try to get the unit talking
	if (com.SetBaud(57600) != OK) return Error();
	if (com.Write("\r\n\r\n\r\n") != OK) return Error();
	com.Purge();

	// Listen at various baud rates and see if the device is out there
	int baud = com.FindBaudRate("$PASHS,NME,ALL,A,OFF\r\n", "$PASHR,ACK", AC12Baud);
	if (baud == 0) return Error("Can't communicate with AC12 at any baud rate\n");
	debug("CommAC12: found device at %d baud\n", baud);

	// Change it to high speed serial
	if (baud != 57600) {
		if (com.Write("$PASHS,SPD,A,8\r\n") != OK) return Error();
		if (com.SetBaud(57600) != OK) return Error();
		if (com.QueryResponse("$PASHS,NME,ALL,A,OFF\r\n", "$PASHR,ACK", 200) != OK) return Error();
	}
   
	return OK;
}


bool CommAC12::PutBlock(Block& b)
{
	if (b.Id != 0)
		return Error("PutBlock only does NMEA (Id=0)");

	b.Display("CommAC12::PutBlock");
	return com.Write(b.Data, b.Length);
}


bool CommAC12::GetBlock(Block& b)
{
	debug(4,"CommAC12::GetBlock (begin)\n");
	// Wait for a $
	if (com.AwaitString("$", 500) != OK) return Error();
	b.Data[0] = '$';
	b.Length = 1;
	b.Id = 0;

	// Get the NMEA message type
	if (ReadField(b) != OK) return Error();

	// if this is an Ashtec proprietary response
	if (Match(b, 0, "$PASHR")) {

		// then read the subtype as an ID 
		if (ReadId(b) != OK) return Error();

	    // Process according to the block type
		b.Length = 0;
	    if      (b.Id == 'PBN')  ReadBinaryData(b, 56);
	    else if (b.Id == 'MCA')  ReadBinaryData(b, 37);
	    else if (b.Id == 'SNV')  ReadBinaryData(b, 132);
	    else                     ReadNmeaData(b);
	}

	// Otherwise, this is a regular NMEA message
	else {
	    ReadNmeaData(b);
          if (Match(b, 0, "$GPRRE")) b.Id = 'RRE';
          else                       b.Id = 'NMEA';
      }

	b.Display("Get Block");
	return ErrCode;
}





bool CommAC12::ReadBinaryData(Block& b, int size)
{
	debug(4, "CommAC12::ReadBinaryData: size=%d\n", size);
	b.Length = size;
	if (com.Read(b.Data, b.Length) != OK) 
		return Error("AC12: Unable to read binary message\n");

	return OK;
}


bool CommAC12::ReadId(Block& blk)
{
	debug(4, "CommAC12::ReadId\n");
	blk.Id = 0;

	for (;;) {
		byte b;

		ErrCode = com.Read(b);
		if (ErrCode) return Error("Can't read message ID");

		if      (b == '\r') break;
		else if (b == ',')  break;
		else if (b == '*')  break;
		else if (b == '\n') ;
		else if (b == ' ')  ;
		else   
			blk.Id = (blk.Id<<8) | b;
	}

	debug(4, "CommAC12::ReadId -- got 0x%x  %c%c%c\n", blk.Id, blk.Id>>16,blk.Id>>8,blk.Id);
	return OK;
}



bool CommAC12::ReadNmeaData(Block& blk)
{
	// Read data into block, until a 'CR' is encourntered.
	for (;;) {
		byte b;

		if (com.Read(b) != OK) return Error("Can't Read NMEA");

		if      (b == '\n')  ;
		else if (b == '\r')  break;
		else if (b == '*')   break;  // checksum at end of line
		else {
			if (blk.Length >= blk.Max-1) return Error("NMEA sentence too long");
			blk.Data[blk.Length] = b;
			blk.Length++;
		}

	}

	blk.Data[blk.Length] = '\0';

	return OK;
}


bool CommAC12::ReadField(Block& blk)
{
	debug(4, "CommAC12::ReadField - begin\n");
	// Repeat until the field is read (or block overflows)
	for (;;)
	{
		// Read the next character
		byte b;
		if (com.Read(b) != OK) return Error();

		// If end of line, then done.
		if (b == '\r') break;
		if (b == '\n') break;
		if (b == '*')  break;

		// Save the character
		if (blk.Length >= blk.Max-1) 
			return Error("Field exceeds block size");
		blk.Data[blk.Length] = b;
		blk.Length++;

		// If field separator, then done
		if (b == ',') break;
	}
	
	// terminate with extra EOL to make printing easier
	blk.Data[blk.Length] = '\0';
	debug(4, "CommAC12::ReadField: Data=%s\n", blk.Data);

	return OK;
}
		


bool CommAC12::Match(Block& b, int start, const char* str)
{
	const char* p = (char*)&b.Data[start];
	while (*str != '\0')
		if (*p++ != *str++)
			return false;
	return true;
}



bool CommAC12::Command(const char* cmd)
{
	debug("CommAC1::Command %s\n", cmd);
	if (GetError() != OK) return Error();
    ErrCode = com.Write("$PASHS,")
	          || com.Write(cmd)
	          || com.Write("\r\n")
	          || com.AwaitString("$PASHR,ACK", 500);
	return ErrCode;
}



	

CommAC12::~CommAC12()
{
}


