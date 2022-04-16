#ifndef RAWAC12_INCLUDED
#define RAWAC12_INCLUDED
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
#include "CommAC12.h"

class RawAC12 : public RawReceiver
{
protected:
	CommAC12 comm;
	int PositionTag;
	int MeasurementTag;

public:
	RawAC12(Stream& s);
	virtual bool NextEpoch();
	virtual ~RawAC12();

private:
	bool ProcessPosition(Block& b);
	bool ProcessMeasurement(Block& b);
	bool ProcessEphemeris(Block& b);
      bool ProcessResiduals(Block& b);
	bool Command(const char* cmd);
	bool AckOrNak();
};

#endif // RAWAC12_INCLUDED

