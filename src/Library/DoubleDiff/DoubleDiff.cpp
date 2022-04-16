
// DoubleDiff generates positions from two GPS receivers
//    Part of Kinematic, a utility for GPS positioning
//
// Copyright (C) 2005  John Morris    kinematic@precision-gps.org
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

///////////////////////////////////////////////////////////////////////////////
//
// This file solves the double difference code and phase equations.
//   Given a stationary receiver and a roving receiver, it calculates
//   the position of the roving receiver at each epoch. 
//
// This code uses "carrier phase" to calculate a position which is more
//   precise than pseudorange alone. The solution is based on estimating the 
//   number of radio waves between the two receivers, updating the estimate
//   as the roving receiver and the satellites move about.
//
//
// TO DO:
//   o Statistically valid error estimates. 
//   o (eventually) Integer solution of phase ambiguity variables.
//   o Save data necessary to move backward in time. This allows us to
//     use our final values of the phase variables to update earlier estimates 
//     of positions. (smoothing).
//   o Allow roving receiver to take up a known station and improve the location
//     of the stationary receiver.  (eg. stationary is on car in parking lot,
//     rover stops at a known benchmark for a short time.)
//
////////////////////////////////////////////////////////////////////////////////

#include "DoubleDiff.h"



DoubleDiff::DoubleDiff(Ephemerides& e, RawReceiver& s, RawReceiver& r)
: Eph(e), Base(s), Rover(r), solution(Base.Pos, Rover.Pos)
{
	// Start with this position estimate
	LastComputedPosition = Rover.Pos;

	// Assume Kinematic by default
	Kinematic = true;

	// Current and previous observations. Pointers so we can swap easily.
	Obs = new Observations;
	PreviousObs = new Observations;

	Reset();
}




bool DoubleDiff::NextPosition(Time& t, Position& pos, double& cep, double& fit)
{
    if (Kinematic)
		solution.NewPosition(LastComputedPosition);

    // Advance the two receivers until they have an epoch in common
	if (DoubleNextEpoch(Base, Rover) != OK)
		return Error();

	// Save the old observations and get new ones
	Swap(Obs, PreviousObs);
	Obs->Init(Base, Rover, Eph);
	
	// Decide which satellites we are going to use
	Check.SelectSatellites(*Obs, *PreviousObs);

	// Find the best solution, dropping satellites if necesary
	if (FindBestSolution(*Obs, pos, cep, fit) != OK) return Error();

	// Keep track of our last position
	if (cep < 0)     pos = LastComputedPosition;
	else             LastComputedPosition = pos;

	t = GpsTime;

	//LogResiduals();     // done at lower level for now.
	//SatelliteEvents();  // need previous state, which we don't have.

	return OK;
}



bool DoubleDiff::FindBestSolution(Observations& Obs, Position& pos, double& cep, double& fit)
{
	// Save our current solution so we can roll back if necessary
	Solution backup = solution;

	// Using all the satellites, solve position
	if (solution.Update(Obs, pos, cep, fit) != OK) return Error();

	// If the solution is acceptable (or none found), then done
	if (Check.Acceptable(Obs, solution)) return OK;

      // Drop the worst of the satellites.
      solution = backup;
      double oldfit = fit;
      int Worst1 = -1; int Worst2 = -1;
      if (DropWorst(Obs, Worst1) != OK) return Error();

      // If didn't work, drop the two worst satellites
      if (Worst1 == -1)
          if (Drop2Worst(Obs, Worst1, Worst2) != OK) return Error();

      // If no acceptable solution found, start all over.
      if (Worst1 == -1) {
          Event("Unable get solution. Starting all over.  fit=%.1f\n", oldfit);
          solution.Reset();
          cep = -1;
          return OK;
      }

      // Calculate the new solution
	if (solution.Update(Obs, pos, cep, fit)) return Error();  
      Event("Residuals out of bounds, dropping satellites %d and %d.  fit(%.1f-->%.1f)\n", 
                                                       Worst1, Worst2, oldfit, fit);
      return OK;
}

     

bool DoubleDiff::DropWorst(Observations& Obs, int& WorstSat)
{
	Solution backup = solution;

	// do for each valid satellite
	WorstSat = -1; double WorstFit = 999999;
	for (int s=0; s<MaxSats; s++) {
		if (!Obs[s].ValidPhase && !Obs[s].ValidCode) continue;

	    // do a temporary reconfiguration without the satellite
		bool oldphase = Obs[s].ValidPhase; Obs[s].ValidPhase = false;
		bool oldcode  = Obs[s].ValidCode;  Obs[s].ValidCode = false;
		debug("DoubledDiff::Update - experimentally droppinng %d\n", s);

		// keep track of the most acceptable configuration (ie worst satellite)
          Position pos; double cep; double fit;
	    if (solution.Update(Obs, pos, cep, fit) != OK) return Error();
		if (Check.Acceptable(Obs, solution) && fit < WorstFit)
			{WorstSat = s; WorstFit=fit;}

		// Undo the temporary reconfiguration
		solution = backup;
		Obs.obs[s].ValidPhase = oldphase;
		Obs.obs[s].ValidCode = oldcode;
	}
	debug("DoubleDiff::Update - WorstSat=%d\n", WorstSat);

	// Drop the worst satellite if any.
      if (WorstSat != -1)
	     Obs[WorstSat].ValidPhase = Obs[WorstSat].ValidCode = false;
	return OK;
}



bool DoubleDiff::Drop2Worst(Observations& Obs, int& Worst1, int& Worst2)
{
	Solution backup = solution;

	// do for each valid satellite
	Worst1 = Worst2 = -1; double WorstFit = 999999;
	for (int s=0; s<MaxSats; s++) for (int t=s+1; t<MaxSats; t++) {
		if (!Obs[s].ValidPhase && !Obs[s].ValidCode) continue;
            if (!Obs[t].ValidPhase && !Obs[t].ValidCode) continue;

	    // do a temporary reconfiguration without the satellite
		bool oldphases = Obs[s].ValidPhase; Obs[s].ValidPhase = false;
		bool oldcodes  = Obs[s].ValidCode;  Obs[s].ValidCode = false;
            bool oldphaset = Obs[t].ValidPhase; Obs[t].ValidPhase = false;
            bool oldcodet  = Obs[t].ValidCode;  Obs[t].ValidCode = false;
		debug("DoubledDiff::Update - experimentally droppinng %d and %d\n", s, t);

		// keep track of the most acceptable configuration (ie worst satellite)
          Position pos; double cep; double fit;
	    if (solution.Update(Obs, pos, cep, fit) != OK) return Error();
		if (Check.Acceptable(Obs, solution) && fit < WorstFit)
			{Worst1 = s; Worst2 = t; WorstFit=fit;}

		// Undo the temporary reconfiguration
		solution = backup;
		Obs[s].ValidPhase = oldphases;
		Obs[s].ValidCode = oldcodes;
            Obs[t].ValidPhase = oldphaset;
            Obs[t].ValidCode = oldcodet;
	}
	debug("DoubleDiff::Update - Worst1=%d  Worst2=%d\n", Worst1, Worst2);

	// Drop the worst satellite if any.
      if (Worst1 != -1)
	     Obs[Worst1].ValidPhase=Obs[Worst1].ValidCode=Obs[Worst2].ValidPhase=Obs[Worst2].ValidCode = false;
	return OK;
}




void DoubleDiff::BeginKinematic()
{
	Event("Begin Kinematic - Rover may move\n");
	Kinematic = true;
}

void DoubleDiff::BeginStatic()
{
	Event("Begin Static - Rover is stationary\n");
	Kinematic = false;
}





DoubleDiff::~DoubleDiff(void)
{
}


//
// Private methods.
//

bool DoubleDiff::DoubleNextEpoch(RawReceiver& base, RawReceiver& rover)
{
	// Assume the satellites haven't slipped until proven otherwise
	bool Slip[MaxSats];
	for (int s=0; s<MaxSats; s++)
		Slip[s] = false;

	// repeat ... 
	do {
		// Decide which receiver is lagging behind
		RawReceiver& lagging = (base.GpsTime < rover.GpsTime)? 
                                   base: rover;
		// Advance the lagging receiver
		if (lagging.NextEpoch() != OK) return Error();

		// Update the slip status
		for (int s=0; s<MaxSats; s++)
			Slip[s] |= !lagging.obs[s].Valid 
			        || lagging.obs[s].Slip 
					|| lagging.obs[s].Phase == 0;

    // ... until the epochs match
	} while (base.GpsTime != rover.GpsTime);

	// Make note of our time
	GpsTime = base.GpsTime;

	// Mark the receivers as slipped
	for (int s=0; s<MaxSats; s++)
		base.obs[s].Slip = rover.obs[s].Slip = Slip[s];
	
	return OK;
}


void DoubleDiff::Reset()
{
	solution.Reset();
}


