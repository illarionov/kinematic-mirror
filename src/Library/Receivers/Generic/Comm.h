#ifndef COMM_INCLUDED
#define COMM_INCLUDED
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


#include "util.h"
#include "Stream.h"

class BigEndian;
class LittleEndian;

// A Block is a generic structure for holding binary data
//   Each block has an "id" and a set of data bytes.
struct Block
{
public: // Muddled. Some code looks at Id and length directly
	static const int Max = 512;
	int Length;
	int Id;
	byte Data[Max];	

public:
	Block(int id=0) {Length=0; Id=id;}
	void Display(const char* s = "Display") {Display(2, s);}
	void Display(int level, const char* s = "Display");
};


// A BlockPacker stores and retrieves data sequentially in a Block
class BlockPacker
{
protected:
	Block& b;
	int Reader;
public:
	BlockPacker(Block& blk) :b(blk) {Reader=0;}
	byte Get() {return b.Data[Reader++];}
      void Put(byte i) {b.Data[b.Length++] = i;}
	void Put(const char* s) {for (; *s!='\0'; s++) Put(*s);}
      byte UnPut() {return b.Data[--b.Length];}
      void UnGet() {Reader--;}
};

// A BigEndian block has numeric fields represented MSB first
class BigEndian: public BlockPacker
{
public:
	BigEndian(Block& blk): BlockPacker(blk) {};

	void Put2(uint16 i) {Put(i>>8); Put(i);}
	void Put4(uint32 i) {Put2(i>>16); Put2(i);}
	void Put8(uint64 i) {Put4(i>>32); Put4(i);}
	void PutFloat(float f)	{Put4(*(uint32*)&f);}
	void PutDouble(double d) {Put8(*(uint64*)&d);}
	
	uint16 Get2() {return ((uint16)Get()<<8)   + (uint16)Get();}
	uint32 Get4() {return ((uint32)Get2()<<16) + (uint32)Get2();}
	uint64 Get8() {return ((uint64)Get4()<<32) + (uint64)Get4();}
	float GetFloat()
	{
		uint32 f=Get4(); 
		//printf("GetFloat: f=0x%08x (%g)\n", f, *(float*)&f); 
		return *(float*)&f;
	}
	double GetDouble()
	{
		uint64 d=Get8(); 
		//debug("GetDouble: d=0x%016llx (%g)\n", d, *(double*)&d);
		return *(double*)&d;
	}
};

// A LittleEndian block has numeric fields represented as LSB first
class LittleEndian: public BlockPacker
{
public:
	LittleEndian(Block& blk) : BlockPacker(blk){}
	void Put2(uint16 i) {Put(i); Put(i>>8);}
	void Put4(uint32 i) {Put2(i); Put2(i>>16);}
	void Put8(uint64 i) {Put4(i); Put4(i>>32);}
	void PutFloat(float f)	{Put4(*(uint32*)&f);}
	void PutDouble(double d) {Put8(*(uint64*)&d);}

	uint16 Get2() {return (uint16)Get() + ((uint16)Get()<<8);}
	uint32 Get4() {return (uint32)Get2() + ((uint32)Get2()<<16);}
	uint64 Get8() {return (uint64)Get4() + ((uint64)Get4()<<32);}
	float GetFloat()
	{
		uint32 f = Get4(); 
		//debug("GetFloat: f=0x%08x (%g)\n", f, *(float*)&f); 
		return *(float*)&f;
	}

	double GetDouble()
	{
		uint64 d=Get8(); 
		//debug("GetDouble: d=0x%016llx (%g)\n", d, *(double*)&d);
		return *(double*)&d;
	}
};

// Arm is a bigendian machine, but it stores 64bit values
//  as 32bits(low) followed by 32bit(high).
class ArmEndian :public BigEndian
{
public:
	ArmEndian(Block& b): BigEndian(b) {}

	inline
	uint64 Get8() {return (uint64)Get4() + ((uint64)Get4()<<32);}

	inline
	double GetDouble()
	{
		uint64 i=Get8();
		double d = *(double*)&i;
		//debug("GetDouble: i=0x%016llx d=%g\n", i, d);
		return d;
	}

	void Put8(uint64 i) {Put4(i); Put4(i>>32);}
	void PutDouble(double d) {Put8(*(uint64*)&d);}
};


// XXX is a little endian machine, but it stores 64bit values
//  as 32bits(high) followed by 32bit(low).
class XXXEndian :public BigEndian
{
public:
	XXXEndian(Block& b): BigEndian(b) {}

	inline
	uint64 Get8() {return + ((uint64)Get4()<<32) + (uint64)Get4();}

	inline
	double GetDouble()
	{
		uint64 i=Get8();
		double d = *(double*)&i;
		//debug("GetDouble: i=0x%016llx d=%g\n", i, d);
		return d;
	}

	void Put8(uint64 i) {Put4(i); Put4(i>>32);}
	void PutDouble(double d) {Put8(*(uint64*)&d);}
};


class NmeaBlock: public BlockPacker {
public:
    NmeaBlock(Block& b): BlockPacker(b) {}
    void GetField(char *buf, int size);
    int GetInt();
    double GetFloat();
    void PutField(char *buf);
    void PutInt(int value);
    void PutFloat(double value);
};


class Bits: public BlockPacker {
public:
    Bits(Block& b): BlockPacker(b), ExtraBits(0) {}
    void PutBits(int64 value, int bits);
    uint64 GetBits(int bits);
    int64 GetSignedBits(int bits);
protected:
    int ExtraBits;
};




// Comm is a communication interface which reads and writes binary blocks
class Comm
{
protected:
	Stream& com;
	bool ErrCode;

public:
	Comm(Stream& s);
	Comm();  // Temporary - to support Garmin until rewritten

	virtual bool GetBlock(Block& b) = 0;
	virtual bool PutBlock(Block& b)  = 0;
	virtual bool ReadOnly() {return com.ReadOnly();}
	virtual ~Comm();

	bool PutBlock(int id, ...);
	bool PutString(int id, const char* str);

	bool GetError() {return ErrCode;}
	bool AwaitBlock(int32 Id, Block& b);
	bool AwaitBlock(int32 Id);
};


#endif // COMM_INCLUDED

