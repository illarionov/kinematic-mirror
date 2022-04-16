
#include "Policy.h"
bool CodeOnly = false;



Policy::Policy()
{
	// Set the default filtering parameters
	FitThreshhold = 4;
	CodeResidualThreshhold = 8;
	PhaseResidualThreshhold = .1;
	SinMinElev = sin(DegToRad(15));  // Elevation mask
	Delay = 10*NsecPerSec;           // How long to wait before trying again
	PhaseErrorTolerance = 5;         // Allowable noise (m) pseudo-range VS phase

	for (int s=0; s<MaxSats; s++)
		SelectTime[s] = -1;

	CodeOnly = ::CodeOnly;
}

bool Policy::SelectSatellites(Observations& obs, Observations& prev)
{
	GpsTime = obs.GpsTime;

	// do for each satellite
	for (int s=0; s<MaxSats; s++) {
		Observation& o = obs[s];
		Observation& p = prev[s];

		// Ignore satellites with no data
		if (!o.ValidCode && !o.ValidPhase) continue;

		// Must be 15 degrees above horizon. 
		//    This assumes a spherical earth, but good enough.
		Position& pos = obs.BasePos;
		Position satpos = o.SatPos - pos;
		double SinElev = (pos * satpos) / ( Range(pos) * Range(satpos));
	        debug("Select: s=%d SinElev=%.3f  SinMinElev=%.3f  ValidCode=%d ValidPhase=%d\n", 
			s,SinElev, SinMinElev, o.ValidCode, o.ValidPhase);
		if (SinElev < SinMinElev)
			o.ValidCode = o.ValidPhase = false;

		// Don't use phase if a CodeOnly solution
		if (CodeOnly)
			o.ValidPhase = false;
	}

	// Make sure the satellites have consistent code and phase
	CheckCodePhase(obs, prev);

	// Don't use any satellites that have been marked as bad
	for (int s=0; s<MaxSats; s++) {
		Observation&o = obs[s];
		if (SelectTime[s] >= obs.GpsTime)
			o.ValidCode = o.ValidPhase = false;
	}

      // We want at least 4 satellites
      int PhaseCount = 0; int CodeCount = 0;
      for (int s=0; s<MaxSats; s++) {
          if (obs[s].ValidPhase) PhaseCount++;
          if (obs[s].ValidCode)  CodeCount++;
      }
      for (int s=0; s<MaxSats; s++) {
          if (PhaseCount < 4 || CodeCount < 4)  obs[s].ValidPhase = false;
          if (CodeCount < 4)                    obs[s].ValidCode = false;
      }

	return OK;
}


bool Policy::Acceptable(Observations& obs, Solution& sol)
{
	// If we didn't find any solution, then done.
	if (sol.GetCep() == -1)  return false;

    // If overall residuals jumped significantly, then not acceptable
	if (sol.GetFit() > FitThreshhold) return false;

	// If individual residuals are out of bounds, then not acceptable.
	for (int s=0; s<MaxSats; s++) {
		if (obs[s].ValidCode)
			if (sol.GetCodeResidual(s) > CodeResidualThreshhold) return false;
		if (obs[s].ValidPhase)
			if (sol.GetPhaseResidual(s) > PhaseResidualThreshhold) return false;
	}

	return true;
}

bool Policy::EliminateWorst(Observations& obs, Solution& sol)
{
	return OK;
}






void Policy::MarkBad(Observations& obs, int sat)
{
	// TODO: we want to use current time, not previous time.
	debug("Checker::MarkBad: sat=%d  Delay=%.1f\n", sat, S(Delay));
	//SelectTime[sat] = GpsTime + Delay;
	obs[sat].ValidCode = obs[sat].ValidPhase = false;
}




void Policy::CheckCodePhase(Observations& obs, Observations& prev)
{
	double Threshhold = PhaseErrorTolerance;

	// Repeat until all satellites fit within the threshhold
	for (;;) {

		// Find the worst of the satellites
		int WorstSat; double WorstDelta;
		FindWorst(obs, prev, WorstSat, WorstDelta);

		// If the worst is still OK, then done
		if (WorstDelta <= Threshhold || WorstSat == -1) break;

		// Remove the worst satellite
		//MarkBad(obs, WorstSat);
            obs[WorstSat].ValidPhase = false;
		Event("Inconsistent code/phase on s=%d  delta=%.3f\n", WorstSat, WorstDelta);
	}
}




void Policy::FindWorst(Observations& obs, Observations& prev, 
			   int& WorstSat, double& WorstDelta)
{

	// Keep track of the average, min, and max code/phase error
	int MinSat; double MinDelta = INFINITY;
	int MaxSat; double MaxDelta = -INFINITY;
	int count=0; double TotalDelta=0;

    // Do for each selected satellite
	for (int s=0; s<MaxSats; s++) {
		Observation& p = prev[s];
		Observation& o = obs[s];

		// Skip satellites which aren't candidates
		if (!o.ValidCode || !o.ValidPhase || !p.ValidCode || !p.ValidPhase || o.Slip)
			continue;

		// Calculate the relative change in pseudorange and phase
		double Delta =  (o.PR - p.PR) - (o.Phase - p.Phase)*L1WaveLength;

		debug(2, "Checker s=%d  Delta=%.3f b.pr=%.3f  b.ph=%.3f  r.pr=%.3f  r.ph=%.3f\n", 
			s, Delta, o.PR, o.Phase, p.PR, p.Phase);

		// Keep track of the min, max and average
		if (Delta < MinDelta) 
		    {MinDelta = Delta; MinSat = s;}
		if (Delta > MaxDelta) 
		    {MaxDelta = Delta; MaxSat = s;}

		TotalDelta += Delta;
		count++;
	}

	if (count == 0) {WorstSat = -1; WorstDelta = 0; return;}

	// figure out whether the min or the max is furthest from the average
	double Average = TotalDelta/count;
	MinDelta = abs(MinDelta - Average);
	MaxDelta = abs(MaxDelta - Average);
	if (MaxDelta > MinDelta)  {WorstDelta = MaxDelta; WorstSat = MaxSat;}
	else                      {WorstDelta = MinDelta; WorstSat = MinSat;}
	
	debug("FindWorstSat: count=%d sat=%d  delta=%g\n", count, WorstSat, WorstDelta);
}




Policy::~Policy()
{
}

