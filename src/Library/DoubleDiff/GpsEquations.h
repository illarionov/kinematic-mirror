#ifndef GPSEQUATIONS_INCLUDED
#define GPSEQUATIONS_INCLUDED
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


#include "LinearEquation.h"
#include "Observations.h"


class GpsEquations: public LinearEquations
{
protected:

	// How the columns are assigned
	int SatelliteToColumn[MaxSats];

public:
	static const int TcCol=0, TpCol=1, XCol=2, YCol=3, ZCol=4, FirstPhase=5;
	GpsEquations();
	bool AppendCode(Triple& e, double b, double weight);
	bool AppendPhase(Triple& e, double p, int sat, double SatVal, double NonsatVal, 
		double weight);
	int LastSatellite();

	bool SolvePosition(Position& pos, double& cep, double& fit);
	bool NewPosition();
	bool NewEpoch();
	void Reset();

	bool ChangeReference(int OldRef, int NewRef);
	bool DropPhase(int sat);
	bool AddPhase(int sat);
	bool DeletePhase(int sat);
	bool PhaseDefined(int sat);

	// Get solution values
	Position GetOffset();           // The position offset
	double GetCep();
	double GetTc();
	double GetTp();
	double GetAmbiguity(int sat);

	GpsEquations(GpsEquations& src);
	GpsEquations& operator=(GpsEquations& src);
};

#endif

