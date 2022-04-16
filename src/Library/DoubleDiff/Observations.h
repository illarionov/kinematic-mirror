
//    Part of Kinematic, a utility for GPS positioning
//
// Copyright (C) 2005  John Morris    www.precision-gps.org
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
#ifndef OBSERVATIONS_INCLUDED
#define OBSERVATIONS_INCLUDED

#include "RawReceiver.h"
#include "Ephemeris.h"

struct Observation
{
	// Satellite info
	Position SatPos;
	bool ValidCode;
	bool ValidPhase;
	bool Slip;

	// Reliability of measurements
	double CodeWeight;
	double PhaseWeight;

	// Measured differences
	double Phase;
	double PR;

	int Sat;  // more for debugging than any computational need
};

class Observations
{
public:
	bool ErrCode;
	Observation obs[MaxSats];
	Position BasePos;
	Position RoverPos;
	Time GpsTime;

public:
	Observations();
	Observations(RawReceiver& base, RawReceiver& rover, Ephemerides& eph);
	void Init(RawReceiver&  base, RawReceiver& rover, Ephemerides& eph);
	inline Observation& operator[](int sat) {return obs[sat];}
	inline bool GetError(){return ErrCode;}
	Observations& operator=(Observations& obs);
	virtual ~Observations();
};




#endif // OBSERVATIONS_INCLUDED

