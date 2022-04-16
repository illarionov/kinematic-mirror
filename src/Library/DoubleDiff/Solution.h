#ifndef SOLUTION_INCLUDED
#define SOLUTION_INCLUDED

#include "GpsEquations.h"
#include "Observations.h"

class Solution
{
protected:
	// Basic geometry
	Position RoverPos;
	Position BasePos;
	double CodeClock;
	double PhaseClock;

	// Single difference equations
	Triple e[MaxSats];
	double CodeB[MaxSats];
	double PhaseB[MaxSats];

	// Reference satellite used for double differencing
	int ReferenceSat;

	// The resulting linear gps equations
	GpsEquations eqn;


public:
	Solution(Position& basepos, Position& roverpos);
	bool NewPosition(Position& pos);
	bool Update(Observations& obs, Position& pos, double& cep, double& fit);
	bool Reset();

	Position GetPosition();
	double GetCep();
	double GetFit() {return eqn.GetFit();}
	double GetCodeResidual(int sat);
	double GetPhaseResidual(int sat);

	virtual ~Solution();

private:
	bool AppendDoubleDifference(Observations& obs);
	bool AppendDoublePhase(Observations& obs);
	bool AppendDoubleCode(Observations& obs);
	bool UpdateSatellites(Observations& obs);
	bool Solve(Position& pos, double& cep, double& fit);
	bool SingleDifference(Observation& o, Triple& e, double& CodeB, double& PhaseB);

	bool DropReference(Observations& obs);
	bool PickNewReference(Observations& obs);
	int  BestReference(Observations& obs);
};


#endif // SOLUTION_INCLUDED

