// L1Rinex.h: interface for the L1Rinex class.
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

#ifndef RAWRINEX_INCLUDED
#define RAWRINEX_INCLUDED

#include "RawReceiver.h"
#include "Stream.h"

class RawRinex : public RawReceiver  
{
protected:
	Stream& In;
	// Header Information we need to keep
	int L1Index;
	int C1Index;
	int D1Index;
	int S1Index;
	int NrMeasurements;
	int StartYear;
public:
	RawRinex(Stream& s);
	virtual ~RawRinex();
	virtual bool NextEpoch();
private:
	bool Initialize(int baud=9600);
	bool ReadHeader();
	bool ReadLine(char* line, int len);

	bool ProcessObservations(char* line);
	bool ProcessFixups(char* line);
	bool ProcessEvent(char* line);

	bool ParseRinexTime(char* line, Time& time);
	bool ParseObservation(char *line, int index, double& r, double& snr);
	bool ParsePhaseObservation(char* line, int index, double& p, bool& slp, double& snr);
};


// Oops.  This rinex software has phase going the wrong direction
class RawReverseRinex : public RawRinex
{
public:
	RawReverseRinex(Stream& s):RawRinex(s) {}
	~RawReverseRinex(){}
	bool NextEpoch()
	{
		bool ret = RawRinex::NextEpoch();
		for (int s=0; s<MaxSats; s++)
			obs[s].Phase = -obs[s].Phase;
		return ret;
	}
};

#endif // RAWRINEX_INCLUDED

