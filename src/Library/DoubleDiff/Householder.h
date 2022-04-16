// Householder implements the Householder tranformation for solving inear equations
//    Part of kinematic, a collection of utilities for GPS positioning
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


#ifndef HOUSEHOLDER_INCLUDED
#define HOUSEHOLDER_INCLUDED


#include "util.h"


// Templatized functions for working with Householder tranformations.
//   Wanted to use <int M, int N> as the parameters, but had problems 
//   getting Visual C++ to work. Worked around by using type parameters 
//   instead, which places an onus on the caller to use the proper types.

// Working on submatrix A[MinRow..MaxRow][MinCol..MaxCol],
//   eliminate the variable at the given Row and Col.
//   When completed, A[Col][Row] will contain the only non-zero entry in the column,
//   allowing us to remove Row and Column from the set of equations.
static const double eps = .0001;

template <typename Ta, typename Tb>
bool ApplyHouseholder(Ta& A, int32 MinRow, int32 MaxRow, int32 MinCol,int32 MaxCol,
					  Tb& B, int32 Row, int32 Col)
	{
		debug(3,"ApplyHouseholder:  A(%d-%d, %d-%d)  row=%d col=%d\n",
	      	MinRow, MaxRow, MinCol, MaxCol, Row, Col);

		// Calculate S as +/- 
		double S2 = 0;
		for (int32 i=MinRow; i<=MaxRow; i++)
			S2 += A[i][Col] * A[i][Col];
		double S = sqrt(S2);
		if (A[Row][Col] < 0)
			S = -S;

		if (S2 < eps*eps) 
			return Error("Invalid Householder Transform\n");

		// Calculate w[k],
		double wk = A[Row][Col] + S;
		double Beta = 1 / (wk * S);

		// Do for each column i except the reference column
		for (int32 j=MinCol; j<=MaxCol; j++) {
			if (j == Col) continue;

			// Calculate (W * Vi) / (W * W)
			double d = wk * A[Row][j];
			for (int32 i=MinRow; i<=MaxRow; i++)
				if (i != Row)
					d = d + A[i][Col] * A[i][j];
			double f = Beta * d;

			// Transform column i
			for (int32 i=MinRow; i<=MaxRow; i++)
				if (i != Row)
					A[i][j]= A[i][j] - f * A[i][Col];
			A[Row][j] = A[Row][j] - f * wk;
		}

		// Calculate (W * B) / (W * W)
		double d = wk * B[Row];
		for (int32 i=MinRow; i<=MaxRow; i++)
			if (i != Row)
				d = d + A[i][Col] * B[i];
		double f = Beta * d;

        // Transform B
		for (int32 i=MinRow; i<=MaxRow; i++)
			if (i != Row)
				B[i] = B[i] - f * A[i][Col];
		B[Row] = B[Row] - f * wk;

		// Transform the k'th column of A
		for (int32 i=MinRow; i<=MaxRow; i++)
			if (i != Row)
				A[i][Col] = 0;
       	A[Row][Col] = -S;

        DebugArray(A, MinRow, MaxRow, MinCol, MaxCol, B, "After Householder");
		return OK;
	}


template<typename Ta, typename Tb, typename Tx>
bool BackSubstitute(Ta& A, int LastRow, int LastCol, Tb& B, Tx& X)
	{
		// Assumes A is upper triangular for first m variables,
		//   m<=n, and X[m-1..n-1] have already been solved
		//debug_array(A, 0, m-1, 0, n-1, B, "BackSubstitute");

		// Solve for each remaining variable
		for (int r=LastRow; r >= 0; r--) {
			X[r] = B[r];
			for (int c=r+1; c<=LastCol; c++)
				X[r] = X[r] - A[r][c] * X[c];
			X[r] = X[r] / A[r][r];
		}

		//for (int32 i=0; i<=LastCol; i++) 
		//	debug("X[%d]=%.3f\n", i, X[i]);
			
		return OK;
	}


template <typename T>
T SpecialHouseholder(int M, T ref, T sum)
{
	double RootM = sqrt((double)M);
	if (M < 2) return 0;
	else       return (ref*RootM - sum) / (M - RootM);
}

#endif // !defined(AFX_HOUSEHOLDER_H__DB5D0222_F3F0_40E4_B2D5_87C38D9E0600__INCLUDED_)

