#ifndef RAWRTCM_INCLUDED
#define RAWRTCM_INCLUDED
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


#include "CommRtcm3.h"
#include "RawReceiver.h"

class RawRtcm3: public RawReceiver
{
protected:
	CommRtcm3 In;

	// To keep track of slips and overflows between epochs
        int32 PhaseAdjust[MaxSats];
        int32 OldPhase[MaxSats];

        // Guess we are using current time until we get ephemeris
        bool GuessTime;

public:
	RawRtcm3(Stream& in);
	virtual bool NextEpoch();
	virtual ~RawRtcm3(void);

private:
	bool ProcessStationRef(Block& b);
	bool ProcessAntennaRef(Block& b);
	bool ProcessObservations(Block& b);
        bool ProcessEphemeris(Block& b);

        double PreviousPhaseRange[MaxSats];
        int PreviousLockTime[MaxSats];
};

#endif // RAWRTCM_INCLUDED

