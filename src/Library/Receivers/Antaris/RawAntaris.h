#ifndef RAWAntaris_INCLUDED
#define RAWAntaris_INCLUDED
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
#include "CommAntaris.h"
#include "EphemerisXmit.h"


class RawAntaris :public RawReceiver
{
protected:
    CommAntaris comm;
	Time NavTime, NavEpoch;   // Last known NAV-SOL time
	Position NavPos, NavVel;  // Last known position and velocity
      double NavPacc;            // Last known position accuracy
	Time RawTime, RawEpoch;   // Time of the RAW-RAW record
	Time PrevNavTime, PrevNavEpoch; // Time of the previous NAV-SOL record
	double ClockDrift;        // time(nsec) per second
	Time GpsEpoch;            // Epoch of GpsTime

public:
	RawAntaris(Stream& s);
	virtual bool NextEpoch();
	virtual ~RawAntaris();

private:
	bool Initialize();

	// Send commands
	bool EnableMessage(int id);
	bool DisableMessage(int id);
	bool SetMessageRate(int HZ);

	// Process messages
	bool ProcessSolution(Block& b);
	bool ProcessEphemeris(Block& b);
	bool ProcessRawMeasurement(Block& b);

	bool Ack();
};




#endif // RAWAntaris_INCLUDED

