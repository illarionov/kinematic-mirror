
// Comm - an abstract class for storing and communicating binary records
//   Part of the Kinetic GPS utilities
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
//////////////////////////////////////////////////////////////////////////////

#include "Comm.h"
#include <stdarg.h>
#include <ctype.h>


Comm::Comm(Stream &s)
: com(s)
{
	ErrCode = com.GetError();
}

// This is a TEMPORARY hack to support Garmin usb
DummyStream dummystream;
Comm::Comm()
: com(dummystream)
{
	ErrCode = Error();
}

bool Comm::PutBlock(int Id, ...)
{
	Block block(Id);
	LittleEndian b(block);

	va_list args;
	va_start(args, Id);

	int Data;
	while( (Data=va_arg(args,int)) >= 0)
		b.Put(Data);

	va_end(args);
	return PutBlock(block);
}

bool Comm::PutString(int id, const char* str)
{
	Block blk(id);
	LittleEndian b(blk);
	while (*str != '\0')
		b.Put(*str++);
	return PutBlock(blk);
}

bool Comm::AwaitBlock(int32 id, Block& b)
{
	debug("Comm::AwaitBlock - Id=%d(0x%x)\n", id, id);
	do {
		if (GetBlock(b) != OK) return Error();
		debug("Waiting for %d.  Got %d\n", id, b.Id);
	} while (b.Id != id);
	
	return OK;
}

bool Comm::AwaitBlock(int32 Id)
{
	Block b;
	return AwaitBlock(Id, b);
}




void Block::Display(int level, const char* s)
{
	if (level > DebugLevel) return;
	debug(level, "%s - Id=%d(0x%x)  Length=%d\n", s, Id, Id, Length);
      for (int j=0; j < Length; j += 16) {
          for (int i=j; i<j+16; i++)
              if (i < Length)   debug(level, "%02x ", Data[i]);
              else              debug(level, "   ");
          debug(level, "    ");
          for (int i=j; i<j+16; i++)
              if (i < Length)   debug(level, "%c", isprint(Data[i])?Data[i]:'.');
              else              debug(level, " ");
          debug(level, "\n");
      }
}



void NmeaBlock::GetField(char *buf, int size)
{
    int i;
    for (i=0; i<size-1; i++) {
        buf[i] = (char)Get();
        if (buf[i] == ',' || buf[i] == '\0') break;
    }

    buf[i] = '\0';
}

void NmeaBlock::PutField(char *buf)
{
}

int NmeaBlock::GetInt()
{
    char buf[sizeof(NmeaBlock)];
    GetField(buf, sizeof(buf));
    return atoi(buf);
}

double NmeaBlock::GetFloat()
{
    char buf[sizeof(NmeaBlock)];
    GetField(buf, sizeof(buf));
    return atof(buf);
}

void NmeaBlock::PutInt(int value)
{
    Error("NmeaBlock::PutInt not implemented\n");
}

void NmeaBlock::PutFloat(double value)
{
}


void Bits::PutBits(int64 value, int bits)
{
    debug(9, "PutBits: value=0x%llx  bits=%d ExtraBits=%d\n", value, bits,ExtraBits);
    // Align the value to the far left
    uint64 word =  value<<(64-bits);
    
    // If there are leftover bits, shift right and add them in
    if (ExtraBits > 0)
        word = (word >> ExtraBits) | ((uint64)UnPut() << 56);

    // Starting from left, store each full byte
    int nbits = bits + ExtraBits;
    for (; nbits >= 8; nbits -= 8) {
        Put(word>>56);
        word <<= 8;
    }

   // If there are new leftover bits, store them as well
   ExtraBits = nbits;
   if (ExtraBits > 0)
       Put(word>>56);
}


uint64 Bits::GetBits(int nbits)
{

    int bytes = (nbits+ExtraBits) / 8;
    int bits  = (nbits+ExtraBits) % 8;

    // Fetch the data bytes
    uint64 word = 0;
    for (int i=0; i<bytes; i++)
        word = (word << 8) | Get();

    // If there are extra bits, add them in and shift right
    if (bits > 0)
        word = ((word << 8) | Get()) >> (8-bits);
    
    // Extract the desired value
    word = word << (64 - nbits);
    word = word >> (64 - nbits);

    // If there are still extra bits, put them back
    if (bits > 0)  UnGet();

    debug(9, "GetBits nbits=%d  word=0x%llx  ExtraBits=%d  bits=%d\n",
                      nbits,    word,        ExtraBits,    bits);

    ExtraBits = bits;
    return word;
}


int64 Bits::GetSignedBits(int bits)
{
    int64 word = (int64)GetBits(bits);
    return ((word << (64 - bits)) >> (64 - bits));
}



Comm::~Comm()
{
}

