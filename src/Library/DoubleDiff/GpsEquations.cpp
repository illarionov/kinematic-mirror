
//    Part of Kinetic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    kinetic@precision-gps.org
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
//
//
////////////////////////////////////////////////////////////////////////////
//
// The code implements a "float" solution. The GPS hardware tells us
//   the fraction of the wave and ideally we would calculate the integer number of 
//   whole waves. In this case, we do a least squares estimate and let
//   the resultss fall on fractional numbers. We should get a good solution,
//   but the resulting positions are not as precise as if we figured out the 
//   exact integers. I hope to add integer "fixed" solutions at a later date.
//
// All of the mathematics is based on Householder transformations. These
//   linear transformations take the place of conventional "elimination" 
//   for solving systems of equations. Householder transforms are orthagonal,
//   so they can be used freely without messing up the error vectors or disrupting
//   the least squares solution.
//
// This code is based on single differencing. It is equivalent to double
//   differencing, but it gets there by a slightly different path.
//
// The steps are:
//   o Form N single difference code and phase equations for the N satellites.
//   o The phase equations are singular, (N+1 variables, N equations)
//   o Pick an arbitrary reference satellite and pick an arbitrary 
//     value for its phase variable.  We choose zero. 
//   o We now have N phase variables and N phase equations,
//     along with 4 code variables and N code equations. It can be solved.
//   o Use your favorite method to solve the equations.
//     We use Golub's method to calculate a least squares solution.
//
// The ordering of the variables is fairly important.
//   Tcode, Tphase change every epoch. Place them first so they can be eliminated.
//   For Kinematic positioning, X,Y,Z also change every epoch. They come next.
//   Later, we may want to update X,Y,Z based on better estimates of the phase
//   variables. Since the phase variables come after X,Y,X, it is a simple 
//   matter of backsubstitution.
//
// Householder transforms are used as follows:
//    o Least squares solution. Eliminate each variable in turn, leaving 
//      an upper diagonal matrix which can be solved by backsubstitution.
//      (This is called ... )
//    o When a satellite is lost, it's phase variable is eliminated using a
//      Householder transform. This way, we only have to deal with a small number
//      of phase variables at any one time.
//    o The current least squares solution is updated every epoch by
//      appending the new observation equations to the previous NxN matrix and
//      applying Householder transforms until we again have an NxN upper diagonal
//      matrix.
//
// Summarizing the mathematics, Householder transforms are everything!
//    They allow us to rearrange and eliminate variables to our heart's content
//    while maintaining the conditions necessary for a good least-squares solution.
//
// To keep memory management and matrix operations simple, we allocate a single
//   fixed size matrix.  The first five columns hold Tphase, Tcode, X, Y and Z,
//   and the subsequent columns contain the ambiguity variables.
//
// At the end of every epoch, the matrix is NxN upper diagonal and contains the 
//   least squares solution.
//  
//////////////////////////////////////////////////////////////////////////////////////

#include "GpsEquations.h"


GpsEquations::GpsEquations()
{
	Reset();
}




bool GpsEquations::SolvePosition(Position& offset, double& cep, double& fit)
{
	Debug("GpsEquations::SolvePosition  (start)");

	// Get a solution for the current equations
	if (Solve() != OK)
		return Error();

	fit = GetFit();
	
	// Estimate the cep  (needs to be worked on. this is a hack)
	double XYZTrace = sqrt(A[XCol][XCol]*A[XCol][XCol] + A[YCol][YCol]*A[YCol][YCol] 
                                 + A[ZCol][ZCol]*A[ZCol][ZCol]);
      double trace = Trace2();
	cep = sqrt(TotalR2)*100/TotalCount/XYZTrace; 
	offset = Position(X[XCol], X[YCol], X[ZCol]);
	debug("TotalR2=%g  TotalCount=%d  XYZTrace=%g trace=%g cep=%g\n",
		TotalR2, TotalCount, XYZTrace, trace, cep);

	debug("Offset=(%.3f %.3f %.3f) cep=%.3f fit=%.1f\n", offset.x, offset.y, offset.z,cep,fit);


	return OK;
}

bool GpsEquations::AppendCode(Triple& e, double b, double weight)
{
	debug(2, "GpsEquations::AppendCode e=(%g,%g,%g) b=%g weight=%g\n",e[0],e[1],e[2],b,weight);
	int row = AddRow();
	A[row][TcCol] = 1   * weight;
	A[row][XCol] = e[0] * weight;
	A[row][YCol] = e[1] * weight;
	A[row][ZCol] = e[2] * weight;
	B[row] = b * weight;

	return OK;
}


bool GpsEquations::AppendPhase(Triple& e, double p, int sat, double v, double nonv, double weight)
{
	int col = SatelliteToColumn[sat];
	debug(2, "GpsEquations::AppendPhase e=(%g,%g,%g) p=%g weight=%g v=%g nonv=%g sat=%d col=%d\n",
		e[0],e[1],e[2], p, weight, v, nonv, sat, col);
	int row = AddRow();
	if (row == -1) return Error();
	A[row][TpCol] = 1   * weight;
	A[row][XCol] = e[0] * weight;
	A[row][YCol] = e[1] * weight;
	A[row][ZCol] = e[2] * weight;
	B[row] = p * weight;
	for (int c = FirstPhase; c<=LastCol; c++)
	    if (c == col) A[row][c] = v * weight;
		else          A[row][c] = nonv * weight;

	return OK;
}


bool GpsEquations::NewPosition()
{
	debug("NewPosition  LastRow=%d  LastCol=%d\n", LastRow, LastCol);
	if (LastRow == -1)
		;
	else if (LastRow < ZCol)
		Reset();
	else {
		DeleteRow(ZCol);
		DeleteRow(YCol);
		DeleteRow(XCol);
	}

	return OK;
}

bool GpsEquations::NewEpoch()
{
	debug("GpsEquations::NewEpoch  LastRow=%d \n", LastRow);
	if (LastRow == -1)
		;
	else if (LastRow < TpCol)
		Reset();
	else {
		DeleteRow(TpCol);
		DeleteRow(TcCol);
	}
	return OK;
}


bool GpsEquations::AddPhase(int sat)
{
	// If we are already tracking the satellite, then NOP
	if (SatelliteToColumn[sat] != -1) return OK;

	// Add a new phase variable for the satellite
	int col = AddCol();
	debug("AddPhase  sat=%d  col=%d\n", sat, col);
	if (col == -1) return Error();
	SatelliteToColumn[sat] = col;

	return OK;
}


bool GpsEquations::DropPhase(int sat)
{
	// if already dropped, then NOP
	int col = SatelliteToColumn[sat];
	if (col == -1) return OK;
	debug("DropPhase  sat=%d  col=%d\n", sat, col);

	// Eliminate the phase variable from all but the last row and delete the last row
	if (Eliminate(LastRow, col) != OK) return Error();
	DeleteRow(LastRow);

	// Delete the phase variable
	DeletePhase(sat);

	return OK;
}

bool GpsEquations::DeletePhase(int sat)
{
	// Drop the Satellite phase variable.
	//   Redo the mapping to match the DeleteCol operation.

	int col = SatelliteToColumn[sat];
	if (col == -1) return Error("DeletePhase - sat %d already deleted!\n", sat);
	int LastSat = LastSatellite();
	debug("DeletePhase: sat=%d  col=%d LastSat=%d LastCol=%d\n", sat, col, LastSat, LastCol);
	SatelliteToColumn[LastSat] = col;
	SatelliteToColumn[sat] = -1;
	DeleteCol(col);

	return OK;
}


bool GpsEquations::ChangeReference(int OldRef, int NewRef)
{
	// Make sure we are changing to a valid satellite
	int RefCol = SatelliteToColumn[NewRef];
	if (RefCol == -1) 
		return Error("Error - changing to bad reference satellite: old=%d new=%d\n", OldRef, NewRef);
	
	// Reuse the new reference satellite's column to hold the former reference satellite's data
	debug("ChangeReference OldRef=%d  NewRef=%d  RefCol=%d \n", OldRef, NewRef, RefCol);
	SatelliteToColumn[OldRef] = RefCol;
	SatelliteToColumn[NewRef] = -1;

	// We need to redefine the Phase variables. As it turns out, all the old cooeficients
	//   stay the same, but we have to define a column for the previous reference sat.
	debug(4, "       LastRow=%d  LastCol=%d   FirstPhase=%d\n", LastRow, LastCol, FirstPhase);
	for (int r=0; r<=LastRow; r++) {
		double sum = 0;
		for (int c=FirstPhase; c<=LastCol; c++)
			sum += A[r][c];
		A[r][RefCol] = -sum;
		debug(4, "    A[%d][%d]=%.3f\n", r, RefCol, A[r][RefCol]);
	}
	return OK;
}



int GpsEquations::LastSatellite()
// Note: we could keep track of ColumnToSatellite instead of scanning.
{
	for (int s=0; s<MaxSats; s++)
		if (SatelliteToColumn[s] == LastCol)
			return s;
	return -1;
}



double GpsEquations::GetAmbiguity(int sat)
{
	int col = SatelliteToColumn[sat];
	if (col == -1) return 0;
	else           return X[col];
}

Position GpsEquations::GetOffset()
{
	return Position(X[XCol], X[YCol], X[ZCol]);
}

double GpsEquations::GetTc()
{
	return X[TcCol];
}

double GpsEquations::GetTp()
{
	return X[TpCol];
}


bool GpsEquations::PhaseDefined(int sat)
{
	return SatelliteToColumn[sat] != -1;
}


void GpsEquations::Reset()
{
	debug("GpsEquations::Reset\n");
	LinearEquations::Reset();
	LastCol = FirstPhase-1;
	for (int s=0; s<MaxSats; s++)
		SatelliteToColumn[s] = -1;

}





GpsEquations& GpsEquations::operator=(GpsEquations& src)
{
	*(LinearEquations*)this = src;
	for (int s=0; s<MaxSats; s++)
		SatelliteToColumn[s] = src.SatelliteToColumn[s];

	return *this;
}

GpsEquations::GpsEquations(GpsEquations& src)
{
	*this = src;
}

