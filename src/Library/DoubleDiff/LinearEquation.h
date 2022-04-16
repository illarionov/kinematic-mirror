#ifndef LINEAREQUATIONS_INCLUDED
#define LINEAREQUATIONS_INCLUDED
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
#include "Householder.h"


static const int MaxRows = 3*MaxChannels;
static const int MaxCols = 3+MaxChannels;

class LinearEquations  // probably should be a template
{
public:
	double A[MaxRows][MaxCols], B[MaxRows], X[MaxCols];
	double R2, TotalR2;
	int Count, TotalCount;

	int LastRow;
	int LastCol;

public:
	LinearEquations(void);
	~LinearEquations(void);

	void Reset();
    int AddRow();
	int AddCol();
	int DeleteRow(int row);
	int DeleteCol(int col);
	double Trace2();
	void Debug(const char* s);

	bool Eliminate(int col, int row);
	bool Solve();
	double GetFit();

	LinearEquations& operator=(LinearEquations& src);
	LinearEquations(LinearEquations& src);
};

#endif

