//    Part of Kinematic, a utility for GPS positioning
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
#ifndef RAWGARMIN_INCLUDED
#define RAWGARMIN_INCLUDED


#include "RawReceiver.h"
#include "Comm.h"
#include "CommGarmin.h"
#include "EphemerisXmit.h"
#include "CommBufferedRead.h"


class RawGarmin : public RawReceiver  
{
protected:
	NavFrameBuilder Frame[MaxSats];
	Comm& comm;
	Time PositionTime;
	Time MeasurementTime;

public:
	RawGarmin(Comm& com);
	virtual ~RawGarmin();
	virtual bool NextEpoch();

private:
	bool Initialize();

	bool PutCommand(int16 command);
	bool PutCommand(int16 command, uint16 data);

	bool ProcessProductData(Block& b);
	bool ProcessProtocolArray(Block& b);
	bool ProcessPositionRecord(Block& b);
	bool ProcessReceiverMeasurement(Block& b);
	bool ProcessNavigationData(Block& b);
};



#endif // RAWGARMIN_INCLUDED

