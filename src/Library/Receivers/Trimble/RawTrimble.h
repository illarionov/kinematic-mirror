#ifndef RAWTRIMBLE_INCLUDED
#define RAWTRIMBLE_INCLUDED
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
#include "CommTrimble.h"
#include "Util.h"
#include "EphemerisXmit.h"

class RawTrimble :public RawReceiver
{
protected:
	CommTrimble comm;
	double ClockBias;
	double IntegratedDoppler[MaxSats];

	// Update the ephemerides
	Time UpdateEphemerisAfter[MaxSats];
	int LastEphemeris;

	Time MeasurementTime;
	Time ClockTime;

public:
	RawTrimble(Stream& s);
	virtual bool NextEpoch();
	virtual ~RawTrimble(void);

private:
	bool Initialize();

	// Here are the received packets we're most interested in
	static const byte PositionId = 0x83;
	static const byte RawDataId = 0x5a;
	static const byte GpsTimeId = 0x41;
	static const byte DgpsModeId = 0x82;
	static const byte SatDataId = 0x58;

	bool ProcessPosition(Block& b);
	bool ProcessRawData(Block& b);
	bool ProcessGpsTime(Block& b);
	bool ProcessSatData(Block& b);

	bool InitEphemerides();
	bool RequestEphemeris(int s);
	bool GotEphemeris(int s);
	bool UpdateEphemerides();
};


#endif

