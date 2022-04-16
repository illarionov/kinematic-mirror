#ifndef RINEX_INCLUDED
#define RINEX_INCLUDED
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
#include "Stream.h"

class Rinex  
{
protected:
	Stream& Out;
	RawReceiver& Gps;
	bool ErrCode;

	bool PreviouslyValid[MaxSats];
	double PhaseAdjust[MaxSats];
	bool FirstEpoch;

public:
	Rinex(Stream& out, RawReceiver& gps);
	bool OutputEpoch();
	virtual ~Rinex();
	inline bool GetError() {return ErrCode;}

private:
	bool WriteObservationHeader();
	bool WriteObservation();
	bool Initialize();
	bool PrintSat(int sat);
};

#endif // !defined(AFX_RINEX_H__23352C41_436A_4F19_9899_5A3AAC42DD55__INCLUDED_)

