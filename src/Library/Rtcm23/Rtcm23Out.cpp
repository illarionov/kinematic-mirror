
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


#include "Rtcm23Out.h"

// reverse the bits of a 6 bit byte
static byte Reverse[] = {0x00, 0x20, 0x10, 0x30, 0x08, 0x28, 0x18, 0x38,
                         0x04, 0x24, 0x14, 0x34, 0x0c, 0x2c, 0x1c, 0x3c,
						 0x02, 0x22, 0x12, 0x32, 0x0a, 0x2a, 0x1a, 0x3a,
						 0x06, 0x26, 0x16, 0x36, 0x0e, 0x2e, 0x1e, 0x3e,
						 0x01, 0x21, 0x11, 0x31, 0x09, 0x29, 0x19, 0x39,
						 0x05, 0x25, 0x15, 0x35, 0x0d, 0x2d, 0x1d, 0x3d,
						 0x03, 0x23, 0x13, 0x33, 0x0b, 0x2b, 0x1b, 0x3b,
						 0x07, 0x27, 0x17, 0x37, 0x0f, 0x2f, 0x1f, 0x3f};

Rtcm23Out::Rtcm23Out(Stream& out)
: Out(out)
{
	ErrCode = Out.GetError();
	PreviousWord = 0;
}

bool Rtcm23Out::WriteFrame(Frame &f)
{
	f.Display("WriteFrame");

	// Write out each word of the frame
	for (int i=0; i<f.NrWords; i++)
		if (WriteWord(f.Data[i]) != OK)
			return Error();
	return OK;
}

bool Rtcm23Out::WriteWord(uint32 w)
{
		// Add parity to the word
	    uint32 PreviousD29 = (PreviousWord>>1)&1;
		uint32 PreviousD30 = (PreviousWord)&1;
		uint32 word = AddParity(w, PreviousD29, PreviousD30);
		PreviousWord = word;

		// do for each 6 bit byte in the word
		for (int shift=24; shift>=0; shift-=6) {
			byte b = (word>>shift) & 0x3f;

			// reverse the bits, set high bits and output it.
			if (Out.Write(Reverse[b]|0xc0) != OK)
				return Error();
		}

	return OK;
}


Rtcm23Out::~Rtcm23Out(void)
{
}

