#ifndef Rtcm23OUT_INCLUDED
#define Rtcm23OUT_INCLUDED
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


#include "Stream.h"
#include "Frame.h"


class Rtcm23Out
{
protected:
	Stream& Out;
	bool ErrCode;
	uint32 PreviousWord;

public:
	Rtcm23Out(Stream& out);
	bool GetError() {return ErrCode;}
	virtual bool WriteFrame(Frame& f);
	virtual ~Rtcm23Out(void);

private:
	bool WriteWord(uint32 word);
};


#endif // Rtcm23OUT_INCLUDED

