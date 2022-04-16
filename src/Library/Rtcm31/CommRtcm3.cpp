
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


#include "CommRtcm3.h"
#include "Crc.h"

static const byte preamble = 0xd3;


CommRtcm3::CommRtcm3(Stream& com)
: Comm(com)
{
}

bool CommRtcm3::PutBlock(Block& blk)
{
    blk.Display("Writing RTCM 3.1 Block");

    byte LenHi = (blk.Length>>8) & 0x3;
    byte LenLo = blk.Length;

    // Send the header and body
    if (com.Write(preamble) != OK
     || com.Write(LenHi) != OK
     || com.Write(LenLo) != OK
     || com.Write(blk.Data, blk.Length) != OK) return Error();

    // Send the crc
    Crc24 crc;
    crc.Add(preamble); crc.Add(LenHi); crc.Add(LenLo);
    crc.Add(blk.Data, blk.Length);
    if (com.Write(crc.AsBytes(), 3) != OK) return Error();

    return OK;
}


bool CommRtcm3::GetBlock(Block& b)
{
restart:
    // read the preamble byte
    byte preamble;
    if (com.Read(preamble) != OK) return Error();
    if (preamble != 0xD3)         goto restart;

    // read the length
    byte LenHi, LenLo;
    if (com.Read(LenHi) != OK) return Error();
    if (com.Read(LenLo) != OK) return Error();
    b.Length = ((((int)LenHi)<<8)+LenLo);
    if (b.Length > 1023) goto restart;

    // read the packet contents
    if (com.Read(b.Data, b.Length) != OK) return Error();

    // Calculate the expected checksum
    Crc24 crc;
    crc.Add(preamble); crc.Add(LenHi); crc.Add(LenLo);
    crc.Add(b.Data, b.Length);
    byte *bytes = crc.AsBytes();
  
    // Read the checksum and make sure it matches
    byte c;
    if (com.Read(c) != OK)  return Error();
    if (c != bytes[0])           goto restart;
    if (com.Read(c) != OK) return Error();
    if (c != bytes[1])           goto restart;
    if (com.Read(c) != OK) return Error();
    if (c != bytes[2])           goto restart;

    // Get the message id
    b.Id = (b.Data[0]<<4) | (b.Data[1]>>4);
    b.Display("Read Rtcm 3.1 Block");

    // done
    return OK;
}



CommRtcm3::~CommRtcm3()
{
}

