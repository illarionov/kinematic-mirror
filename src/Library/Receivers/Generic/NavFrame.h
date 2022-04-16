
#ifndef __NavFrame_included
#define __NavFrame_included
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

static const int NavWordsPerFrame = 50;
static const int NavWordsPerSubframe = 10;
static const int NavBitsPerWord = 30;

// Navigation words are 30 bits each. Bits 0 and 1 may contain parity info
//   from the previous word.
typedef uint32 NavWord;


class NavFrame
{
public:
    NavFrame();
    void Display(const char *str);
    bool Valid();
    bool ToRaw(EphemerisXmitRaw& raw);
    bool FromRaw(EphemerisXmitRaw& raw);

    uint32 GetField(int32 FrameNr, int32 BitNr, int32 NrBits);
    int32 GetSigned(int32 FrameNr, int32 BitNr, int32 NrBits);
  

//protected:
    NavWord Data[NavWordsPerFrame];
};



class NavFrameBuilder: public NavFrame
{
public:
	NavFrameBuilder();
	bool AddWord(NavWord word);
	bool AddWord(NavWord word, int32 bitnr);
	bool Complete();

private:
	int32 NextWord;
	uint32 D29, D30;
};


#endif

