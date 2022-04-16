#include "Solution.h"



Solution::Solution(Position& basepos, Position& roverpos)
: RoverPos(roverpos), BasePos(basepos)
{
	ReferenceSat = -1;
}



bool Solution::Update(Observations& obs, Position& pos, double& cep, double& fit)
{
	debug("Solution::Update BasePos=(%.3f, %.3f, %.3f)  RoverPos=(%.3f, %.3f, %.3f)\n",
		BasePos.x, BasePos.y, BasePos.z, RoverPos.x, RoverPos.y, RoverPos.z);

	// We are starting a new epoch and need new clock error variables
	eqn.NewEpoch();

	// Figure which satellies we are now tracking
	if (UpdateSatellites(obs) != OK) return Error();

    // Append the current epoch to the equations
	AppendDoubleDifference(obs);

	// Get the solution if any 
	if (Solve(pos, cep, fit) != OK)
		return Error();

	return OK;
}



bool Solution::UpdateSatellites(Observations& obs)
{
	debug("UpdateSatellites: ReferenceSat=%d\n", ReferenceSat);
	// Drop lost or slipped satellite
	//  If we aren't currently tracking, losing it again is a NOP
	for (int s=0; s<MaxSats; s++)
		if ((!obs[s].ValidPhase || obs[s].Slip) && s != ReferenceSat)
			eqn.DropPhase(s);

	// Switch reference satellites if it was lost
	if (ReferenceSat != -1 && (!obs[ReferenceSat].ValidPhase || obs[ReferenceSat].Slip))
	    DropReference(obs);

	// Gain the new and slipped satellites
	//   If we are already tracking, gaining it again is a NOP
	for (int s=0; s<MaxSats; s++)
		if (obs[s].ValidPhase && s != ReferenceSat)
			eqn.AddPhase(s);

	// If we are starting fresh, need to pick a new reference
	if (ReferenceSat == -1)
		PickNewReference(obs);

	return OK;
}


bool Solution::AppendDoubleDifference(Observations& obs)
{
	// See if we have enough satellite with valid measurements
	int MCode = 0; int MPhase = 0;
	for (int s=0; s<MaxSats; s++) {
		if (obs[s].ValidCode) MCode++;
		if (obs[s].ValidPhase) MPhase++;
	}
	if (MCode < 4 && MPhase < 4)
	    {Reset(); return OK;}

	// Do for each satellite with data
	for (int s=0; s<MaxSats; s++) {
		Observation& o = obs[s];
		if (!o.ValidCode && !o.ValidPhase) continue;

		// Calculate the single difference code and phase equations
		//    tcode + x*e[s] = bcode, 
		//    tphase + x*e[s] + L1WaveLength*ambig = bphase
		// Remember them for later so we can calculate residuals
		SingleDifference(o, e[s], CodeB[s], PhaseB[s]);

		// Add in the code and phase equations
		if (o.ValidCode)
			eqn.AppendCode(e[s], CodeB[s], o.CodeWeight);
		if (o.ValidPhase)
			eqn.AppendPhase(e[s], PhaseB[s], s, L1WaveLength, 0, o.PhaseWeight);
	}

	// Append dummy equations if there are no code or phase equations
	//   These assign zero to the clock variables and keeps the equations solvable
	//   Alternatively, we could delete and renumber the columns.
	// if (MCode == 0)   eqn.AppendCode(Triple(0), 0, 1);
	// if (MPhase == 0)  eqn.AppendPhase(Triple(0), 0, 0, 0, 0, 1);
	Triple GnuTemp(0);
	if (MCode == 0)   eqn.AppendCode(GnuTemp, 0, 1);
	if (MPhase == 0)  eqn.AppendPhase(GnuTemp, 0, 0, 0, 0, 1);


	// debug - show the double difference phase values
	debug("Solution::Append  - Double Difference phase. ReferenceSat=%d\n", ReferenceSat);
	for (int s=0; s<MaxSats; s++)
		if (obs[s].ValidPhase && s != ReferenceSat && ReferenceSat != -1)
			debug("   s=%d  PhaseDiff=%.3f  Phase=%.3f\n", 
			s, obs[s].Phase-obs[ReferenceSat].Phase, 
			(obs[s].Phase-obs[ReferenceSat].Phase)-round(obs[s].Phase-obs[ReferenceSat].Phase)  );
	return OK;
}



bool Solution::Solve(Position& pos, double& cep, double& fit)
{
    // If not enough satellites being tracked, then the equations were reset earlier
	if (eqn.LastRow < 0) {
		pos = RoverPos;
		cep = -1;
		fit = -1;
		return OK;
	}

    // Calculate the new position
	Position offset;
	if (eqn.SolvePosition(offset, cep, fit) != OK) return Error();
	pos = RoverPos + offset;

	return OK;
}



bool Solution::DropReference(Observations& obs)
{
	debug("DropReference:  ReferenceSat=%d\n", ReferenceSat);
	int OldRef = ReferenceSat;

	// Pick a new reference satellite.
	ReferenceSat = BestReference(obs);
	if (ReferenceSat == -1) return OK;

	// Switch to the new reference satellite.
	if (eqn.ChangeReference(OldRef, ReferenceSat) != OK) return Error();

	// Drop the old reference satellite
	if (eqn.DropPhase(OldRef) != OK) return Error();

	return OK;
}

bool Solution::PickNewReference(Observations& obs)
{
	// Pick any satellite. Use the one in the last column since it is easiest to delete.
	ReferenceSat = BestReference(obs);
	debug("PickNewReference: ReferenceSat=%d\n", ReferenceSat);
	if (ReferenceSat == -1) return OK;

	// Simply get rid of this satellite's phase column. (TODO: review)
	return eqn.DeletePhase(ReferenceSat);
}

int Solution::BestReference(Observations& obs)
{
	int Best = -1;
	double BestValue = 0;

	// Do for each satellite with valid phase
	for (int s=0; s<MaxSats; s++) {

		// A reference satellite must be tracked now and be part of previous solution
		if (!obs[s].ValidPhase) continue;
		if (!eqn.PhaseDefined(s)) continue;

		// Pick the one with the highest elevation
		Position& pos = obs.BasePos;
		Position satpos = obs[s].SatPos - pos;
		double SinElev = (pos * satpos) / ( Range(pos) * Range(satpos));
		if (SinElev > BestValue)
			Best = s;
	}

	return Best;
}





bool Solution::SingleDifference(Observation& o, Triple& e, double& CodeB, double& PhaseB)
{
    // Calculate the single difference values
	Position R = RoverPos - o.SatPos;
    Position B = BasePos -  o.SatPos;
	double RangeR = Range(R);
	double RangeB = Range(B);
	e = (R + B) / (RangeR + RangeB);
	CodeB = o.PR - (RangeR - RangeB);
	PhaseB = o.Phase*L1WaveLength - (RangeR - RangeB);
	debug(2, "SingleDifference: sat=%d  e=(%.3f, %.3f, %.3f)  b=%.3f  p=%.3f\n",
		    o.Sat, e[0], e[1], e[2], CodeB, PhaseB);
	return OK;
}



bool Solution::NewPosition(Position& pos)
{
	// We are introducing 3 new position variables.
	//    The matrix should be upper diagonal, so we
	//    eliminate the old position variables by
	//    simply removing the top 3 equations.
	debug("Solution::NewPosition: pos=(%.3f, %.3f, %.3f)\n", pos.x, pos.y, pos.z);
	RoverPos = pos;
	return eqn.NewPosition();
}


Position Solution::GetPosition()
{
	return RoverPos + eqn.GetOffset();
}

double Solution::GetCep()
{
	return 0;
}

double Solution::GetCodeResidual(int sat)
{
	return eqn.GetTc() + e[sat]*eqn.GetOffset() - CodeB[sat];
}

double Solution::GetPhaseResidual(int sat)
{
	return eqn.GetTp() + e[sat]*eqn.GetOffset() + eqn.GetAmbiguity(sat)*L1WaveLength
		     - PhaseB[sat];
}

bool Solution::Reset()
{
	eqn.Reset(); 
	ReferenceSat = -1;
	return OK;
}

Solution::~Solution(){}


