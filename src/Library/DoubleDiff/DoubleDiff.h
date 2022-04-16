#ifndef DOUBLEDIFF_INCLUDED
#define DOUBLEDIFF_INCLUDED
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


#include "Util.h"
#include "Ephemeris.h"
#include "RawReceiver.h"
#include "Observations.h"
#include "Policy.h"
#include "Solution.h"


//////////////////////////////////////////////////////////////////
//
// DoubleDiff - Does double differencing between two GPS receivers.
//   
///////////////////////////////////////////////////////////////////

class DoubleDiff
{
public:
	Time GpsTime;

protected: // Data
    // GPS receivers and ephemerides
	Ephemerides& Eph;
	RawReceiver& Base;
	RawReceiver& Rover;
	bool Kinematic;

	Observations *Obs, *PreviousObs;

	// Current estimated positions
	Position LastComputedPosition;

	// Consistency checker
	Policy Check;
	
	// The cumulative solution
	Solution solution;

public:
	DoubleDiff(Ephemerides& e, RawReceiver& s, RawReceiver& r);
	bool NextPosition(Time& time, Position& pos, double& cep, double& fit);
	void BeginStatic();
	void BeginKinematic();
	virtual ~DoubleDiff(void);

	void LogResiduals();

	// Get information about the solution
	Time GetTime()                   {return GpsTime;}
	Position GetPosition()           {return solution.GetPosition();}
	double GetCep()                  {return solution.GetCep();}   
	bool ValidCode(int sat)          {return (*Obs)[sat].ValidCode;}
	bool ValidPhase(int sat)         {return (*Obs)[sat].ValidPhase;}
	double GetCodeResidual(int sat)  {return solution.GetCodeResidual(sat);} 
	double GetPhaseResidual(int sat) {return solution.GetPhaseResidual(sat);}

private: // Procedures
    bool DoubleNextEpoch(RawReceiver& base, RawReceiver& rover);
	void NewPosition(Position& pos);
	bool UpdateObservations(Position& pos, double& cep, double& fit);
	void Reset();
	bool FindBestSolution(Observations &Obs, Position& pos, double& cep, double& fit);
        bool DropWorst(Observations& Obs, int& sat);
        bool Drop2Worst(Observations& obs, int& Worst1, int& Worst2);
};

#endif // DOUBLEDIFF_INCLUDED

