#ifndef RAWFURUNO_CPP_
#define RAWFURUNO_CPP_
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


#include "RawReceiver.h"
#include "CommFuruno.h"
#include "EphemerisXmit.h"


class RawFuruno :public RawReceiver
{
protected:
    CommFuruno comm;
    int PrevSlip[MaxSats];

public:
	RawFuruno(Stream& s);
	virtual bool NextEpoch();
	virtual ~RawFuruno();

private:
	bool Initialize();

	// Process messages
	bool ProcessPosition(Block& b);
	bool ProcessNav(Block& b);

};



#endif /*RAWFURUNO_CPP_*/
