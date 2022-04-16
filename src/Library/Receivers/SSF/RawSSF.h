#ifndef RAWSSF_INCLUDED
#define RAWSSF_INCLUDED
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
#include "EphemerisXmit.h"
#include "CommSSF.h"

class RawSSF : public RawReceiver
{
protected:
	CommSSF comm;
      int Count;  // The number of satellites remaining in epoch
      double PhaseAdjust;  // adjust phase for clock rollovers
      double ClockError;    // The reported clock error. Adjusted to [-.5 ... +.5]

public:
	RawSSF(Stream& s);
	virtual bool NextEpoch();
	virtual ~RawSSF();

private:
	bool ProcessStartEpoch(Block& b);
      bool ProcessHeader(Block& b);
      bool ProcessRaw(Block& b);
      bool ProcessClock(Block& b);
      bool ProcessPosition(Block& b);
      bool ProcessRoverStartEpoch(Block& b);
      bool ProcessRoverRaw(Block& b);
};

#endif // RAWSSF_INCLUDED


