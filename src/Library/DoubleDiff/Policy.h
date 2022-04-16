#ifndef POLICY_INCLUDED
#define POLICY_INCLUDED

#include "RawReceiver.h"
#include "Observations.h"
#include "Solution.h"

class Policy {
protected:

	// Filter parameters
	double FitThreshhold;           // ratio of current residual to average residual
	double CodeResidualThreshhold;  // largest allowable code residual (meters)
	double PhaseResidualThreshhold; // largest allowable phase residual (meters0

	double SinMinElev;           // Sin of the minimum elevation
	Time   Delay;                // How long to wait before reconsidering a dropped sat
	double PhaseErrorTolerance;  // How much discrepency to allow between code+phase (m)
	bool CodeOnly;

	// When to reconsider bad satellites
	Time SelectTime[MaxSats];
	Time GpsTime;

public:
	Policy();
	bool SelectSatellites(Observations& obs, Observations& previous);
	bool Acceptable(Observations& obs, Solution& sol);
	bool EliminateWorst(Observations& obs, Solution& sol);
	void MarkBad(Observations& obs, int sat);
	virtual ~Policy();

private:
	void CheckCodePhase(Observations& obs, Observations& prev);
	void FindWorst(Observations& obs, Observations& prev, 
			   int& WorstSat, double& WorstDelta);
};


#endif

