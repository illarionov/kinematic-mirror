
#ifndef RAWRECEIVER_INCLUDED
#define RAWRECEIVER_INCLUDED
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



#include "GpsReceiver.h"
#include "Ephemeris.h"
#include "RawObservation.h"


//////////////////////////////////////////////////////////////////////////
// 
// How to store the observations?
//     Struggled over 1)only storing actual observations,
//     or 2)reserving an entry for each satellite. The first is 
//     compact and seems to match the data formats of most receiver.
//     The second means scanning empty entries all the time, but
//     makes it easier to assemble all the data for a satellite 
//     in one place.  Having gone back and forth a couple times,
//     I'm choosing 2) reserving an entry for each satellite.
//     Although it appears wasteful, in the long run it reduces
//     complexity and eliminates copying of data.
//
// What is an observation?
//   - For now, it is a psuedorange and carrier phase measurement
//     for a single satellite.
//     I'd like to generalize it to multiple frequencies,
//     to phase-only measurements, and landbased transmissions. Later, if needed.
//     The quality representation is another issue. The deviation of the measurement
//     is most useful to the math, but SNR is what gets actually measured.
//     I'm avoiding the issue and not relying on either for the moment.
//   - Phase is in Hz. I was considering meters and normalizing to match
//     pseudorange. Hz is less likely to have round off errors, since the 
//     the raw measurements are usually binary fractions.
//
// Ephemerides
//     Another decision was whether a raw receiver *contains* an ephemerides,
//     or whether the receiver *implements* an ephemerides.
//     *implements* seems more elegant, so raw receiver became a subclass.
//
/////////////////////////////////////////////////////////////////////////////////

class RawReceiver : public Ephemerides, public GpsReceiver
{
public:
	// Related to Epoch
	static int HZ;
	RawObservation obs[MaxSats];
	Time RawTime;

	double Adjust;  // defunct
	double PreviousTow;  // defunct

	Time PreviousTime;
	Time PreviousRaw;
	double PreviousPhase[MaxSats];
        double PreviousPR[MaxSats];

	// Related to receiver
	double AntennaHeight;

public:
	RawReceiver();
	virtual bool NextEpoch() = 0;
	virtual ~RawReceiver();
protected:
	bool AdjustToHz(bool IncludeDoppler=true);
	bool AdjustToTime(Time t, bool IncludeDoppler=true);

	Time AdjustToHz(Time t, double tow);  // defunct
	void AdjustToTime(Time t, double tow);          // defunct
};

#endif // RAWRECEIVER_INCLUDED

