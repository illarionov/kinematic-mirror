#ifndef RAWRtcm23_INCLUDED
#define RAWRtcm23_INCLUDED
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


#include "Rtcm23In.h"
#include "RawReceiver.h"

class RawRtcm23: public RawReceiver
{
protected:
	Rtcm23In In;

	// To keep track of slips and overflows between epochs
	int32 CarrierLossCount[MaxSats];
	int64 PreviousPhase[MaxSats];
	int SequenceNr;

	// For keeping time
	int Week;
	int HourOfWeek;
	double PreviousReceiverTime;

	// For processing an epoch
	bool MoreToCome;
	double EpochTow;
	bool HasPhase[MaxSats];

public:
	RawRtcm23(Stream& in);
	virtual bool NextEpoch();
	virtual ~RawRtcm23(void);

private:
	bool ProcessTimeTag(Frame& f);
	bool ProcessAntennaRef(Frame& f);
	bool ProcessAntennaTypeDef(Frame& f);
	bool ProcessPseudorange(Frame& f);
	bool ProcessCarrierPhase(Frame& f);
	bool ProcessEphemeris(Frame& f);
	void Header(Frame& f, Time& t, int& type);
	bool GetMeasurementTime(Frame& f);
};

#endif // RAWRtcm23_INCLUDED

