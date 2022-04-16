#ifndef FRAME_INCLUDED
#define FRAME_INCLUDED
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
#include "EphemerisXmitRaw.h"


class Frame
{
public:
	static const int MaxWords = 256;
    int32 NrWords;
	uint32 Data[MaxWords];

public:
	Frame(int nrwords=0);
	void Init(int nrwords=0);
	void PutWord(int wordnr, uint32 value);
	void AddWord(uint32 value = 0);
	void PutField(int wordnr, int first, int last, uint32 value);
	void Put32(int wordnr, uint32 value);
	int32 Get32(int wordnr);
	uint32 GetField(int wordnr, int first, int last);
	int32 GetSigned(int wordnr, int first, int last);
	void Display(const char *str);

        bool ToRaw(EphemerisXmitRaw& r);
        bool FromRaw(EphemerisXmitRaw& r);
};

uint32 AddParity(uint32 w, uint32 PrevD29, uint32 PrevD30);
bool CheckParity(uint32 w);
bool CheckParity(uint32 w, uint32 PrevD29, uint32 PrevD30);
uint32 RemoveParity(uint32 w);
uint32 RemoveParity(uint32 w, uint32 PrevD29, uint32 PrevD30);

#endif // FRAME_INCLUDED

