#ifndef RAWSIRF_INCLUDED
#define RAWSIRF_INCLUDED
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
#include "CommSirf.h"
#include "EphemerisXmit.h"


class RawSirf :public RawReceiver
{
protected:
   CommSirf comm;
   double PreviousTow;

   bool SolutionFound;
   double MeasurementTow;
   double IntegratedDoppler[MaxSats];
   int PreviousPhaseErrorCount[MaxSats];
   NavFrameBuilder Frame[MaxSats];

public:
	RawSirf(Stream& s);
	virtual bool NextEpoch();
	virtual ~RawSirf();

private:
	bool Initialize();

	// Send commands
	bool WarmReset();
	bool EnableSBAS();
	bool RequestVersion();
	bool EnableMsg(int id);
	bool DisableMsg(int id);

	// Process messages
	bool ProcessNavigation(Block& b);
	bool ProcessVersion(Block& b);
	bool Process50Bps(Block& b);
	bool ProcessClock(Block& b);
	bool ProcessNavlibMeasurement(Block& b);
	bool AckOrNak(int Id);
};




#endif // RAWSIRF_INCLUDED

