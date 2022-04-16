
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

#include "LinearEquation.h"


LinearEquations::LinearEquations(void)
{
	Reset();
}

LinearEquations::~LinearEquations(void)
{
}


void LinearEquations::Reset()
{
	debug(3,"LinearEquations:: Reset\n");
	LastCol = -1;
	LastRow = -1;
	TotalR2 = R2 = 0;
	TotalCount = Count = 0;
}

int LinearEquations::AddRow()
{
	debug(2, "LinearEquations::Addrow  LastRow=%d\n", LastRow);
	LastRow++;
	assert(LastRow < MaxRows);

	for (int c=0; c<=LastCol; c++)
		A[LastRow][c] = 0;

	return LastRow;
}

int LinearEquations::AddCol()
{
	debug(2, "LinearEquations::AddCol  LastCol=%d\n", LastCol);
	LastCol++;
	assert(LastCol < MaxCols);

	for (int r=0; r<=LastRow; r++)
		A[r][LastCol] = 0;

	return LastCol;
}

int LinearEquations::DeleteRow(int row)
{
	debug(2, "LinearEquations::DeleteRow  row=%d  LastRow=%d\n", row, LastRow);
	assert(row >= 0 && row <= LastRow);
	if (row < LastRow) {
		for (int c=0; c<=LastCol; c++)
			A[row][c] = A[LastRow][c];
		B[row] = B[LastRow];
	}

	LastRow--;
	return LastRow;
}


int LinearEquations::DeleteCol(int col)
{
	debug(2, "LinearEquations::DeleteCol  col=%d  LastCol=%d\n", col, LastCol);
	assert (col >= 0 && col <= LastCol);
	if (col < LastCol)
		for (int r=0; r<=LastRow; r++)
			A[r][col] = A[r][LastCol];

	LastCol--;
	return LastCol;
}


bool LinearEquations::Eliminate(int row, int col)
{
	debug(2, "LinearEquations::Eliminate row=%d  col=%d LastRow=%d  LastCol=%d\n",
		                                 row,    col,   LastRow,    LastCol);
	if (LastRow < 0) return OK;

	return ApplyHouseholder(A, 0, LastRow, 0, LastCol, B, row, col);
}




bool LinearEquations::Solve()
{
	Debug("LinearEquations::Solve\n");
	if (LastRow < LastCol) 
		return Error("Can't solve underdetermined linear equation\n");

	// Add the last solutions residuals to the totals
	TotalCount += Count;  TotalR2 += R2;

	// Do a QR factorization, putting the matrix into upper diagonal form
	for (int c=0; c<=LastCol; c++)
		if (ApplyHouseholder(A, c, LastRow, c, LastCol, B, c, c) != OK)
			return Error("Can't solve equations");

	// use backsubstitution to solve
	if (BackSubstitute(A, LastCol, LastCol, B, X) != OK)
		return Error("Can't solve equations");

	// Calculate the residuals from the current equations
	Count = LastRow-LastCol;
	R2 = 0;
	for (int r=LastCol+1; r<=LastRow; r++)
		R2 += B[r]*B[r];

	// Drop the residuals from the matrix
	LastRow = LastCol;

	debug(2, "LinearEquation::Solve LastCol=%d resid**2=%.3f\n", LastCol,R2);
	debug(3,"   ");
	for (int c=0; c<=LastCol; c++)
		debug(3, "%.3f(%.3f) ", X[c], A[c][c]);
	debug(3,"\n");

	return OK;
}

double LinearEquations::GetFit()
{
	// See how the current residuals compare against the previous ones
	//   This ratio is useful for detecting errors in the data.
	if (Count == 0 || TotalCount == 0 || TotalR2 == 0)
		return 1;

	return (R2/Count) / (TotalR2/TotalCount);
	}

double LinearEquations::Trace2()
// Takes the trace of the covariance matrix.
{
	if (LastRow < LastCol)
		return 1;
	double sum = 0;
	for (int c=0; c<=LastCol; c++)
		sum += A[c][c]*A[c][c];

	return sum;
}


void LinearEquations::Debug(const char* s)
{
	debug(2,"Linear Equations%s\n", s);
	DebugArray(A, 0, LastRow, 0, LastCol, B, s);
}

LinearEquations& LinearEquations::operator=(LinearEquations& src)
{
	LastRow = src.LastRow;
	LastCol = src.LastCol;
	for (int r=0; r<=LastRow; r++) {
		B[r] = src.B[r];
		for (int c=0; c<=LastCol; c++)
			A[r][c] = src.A[r][c];
	}

	TotalR2 = src.TotalR2;
	TotalCount = src.TotalCount;
	R2 = src.R2;
	Count = src.Count;
	return *this;
}

LinearEquations::LinearEquations(LinearEquations& src)
{
	*this = src;
}


