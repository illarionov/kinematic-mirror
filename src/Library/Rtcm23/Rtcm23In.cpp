
//    Part of Kinematic, a utility for GPS positioning
//
// Copyright (C) 2005  John Morris    www.precision-gps.org
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



#include "Rtcm23In.h"
#include "util.h"

// reverse the bits of a 6 bit byte
static byte Reverse[] = {0x00, 0x20, 0x10, 0x30, 0x08, 0x28, 0x18, 0x38,
                         0x04, 0x24, 0x14, 0x34, 0x0c, 0x2c, 0x1c, 0x3c,
						 0x02, 0x22, 0x12, 0x32, 0x0a, 0x2a, 0x1a, 0x3a,
						 0x06, 0x26, 0x16, 0x36, 0x0e, 0x2e, 0x1e, 0x3e,
						 0x01, 0x21, 0x11, 0x31, 0x09, 0x29, 0x19, 0x39,
						 0x05, 0x25, 0x15, 0x35, 0x0d, 0x2d, 0x1d, 0x3d,
						 0x03, 0x23, 0x13, 0x33, 0x0b, 0x2b, 0x1b, 0x3b,
						 0x07, 0x27, 0x17, 0x37, 0x0f, 0x2f, 0x1f, 0x3f};


Rtcm23In::Rtcm23In(Stream& s)
:In(s)
{
	RawWord = 0;
	BitShift = 0;
	ErrCode = In.GetError();
}


bool Rtcm23In::ReadFrame(Frame& f, bool& slip)
{
	// Start with an empty frame and expect just the header
	f.Init();
	int Length = 2;
	slip = false;

	// read words until done
	while (f.NrWords < Length) {
		uint32 word; bool wordslip;
		if (ReadWord(word, wordslip) != OK) 
			return Error();

		// if any problems, reset the frame
		slip |= wordslip;
		if (wordslip)
			f.Init();

		// Add the word to the frame
		f.AddWord(word);

		// CASE: word 1, make sure there is a preamble.
		if (f.NrWords == 1 && f.GetField(1, 1, 8) != 0x66){
			f.Init();  
			slip=true;
		}

		// CASE: word 2, get the length
		else if (f.NrWords == 2)
			Length = f.GetField(2, 17, 21);
	}

	if (slip) debug("Frame slipped!\n");
	f.Display("ReadFrame");

	return OK;
}



bool Rtcm23In::ReadWord(uint32& word, bool& slip)
{
	// read 5 bytes and shift them into the raw data 
	for (int i=0; i<5; i++) {
		byte b;
		if (In.Read(b) != OK) return Error();
		b = Reverse[b&0x3f];
		RawWord = (RawWord<<6) | b;
	}

	// slip the appropriate number of bits
	word = (uint32)(RawWord>>BitShift);
	
	// If bad parity, then get synchronized
	slip = (CheckParity(word) != OK);
	if (slip && Synchronize(word) != OK) return Error();

	// done
	word = RemoveParity(word);

	return OK;
}


bool Rtcm23In::Synchronize(uint32& word)
{
	// Repeat until we are synchronized  (parity match, sync bits)
	do {
		// Move to the next bit in the input stream
		BitShift--;

		// If the high order 6-bit byte ran out of bits, ...
		if (BitShift < 0) {

            // ... Read a new 6 bit byte
	        byte b;
	        if (In.Read(b) != OK) return Error();
	        b = Reverse[b&0x3f];

	       // ... shift the new byte onto the word
	       RawWord = (RawWord << 6) | b;
		   BitShift = 5;
		}

		// Extract the word from the bit stream
		word = (uint32)(RawWord>>BitShift);

    // End "Repeat until we are synchronized"
	} while (CheckParity(word) != OK);

	return OK;
}


Rtcm23In::~Rtcm23In(void)
{
}

