// Interpolator does polynomial interpretation 
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


// NOTE: This should change to only keep a small number of reference points
//   in memory. The SP3 file which uses it should read new data as needed.

#include "util.h"
#include "Interpolator.h"
#include <algorithm>


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

template<typename Xt, typename Yt>
Interpolator<Xt,Yt>::Interpolator()
{
}

template<typename Xt, typename Yt>
Interpolator<Xt,Yt>::~Interpolator()
{
}



/////////////////////////////////////////////////////////////////
// Interpolate using a polynomial of degree n-1
//
// This code is based on Neville's algorithm as described in
//    "Numerical Recipes in C: The Art of Scientific Computing".
//    The code has been rearranged so it computes a succession
//    of interpolated values based on increasing degrees
//    of polynomials, adding one datapoint at a time. If the data values 
//    are sorted by their proximity to X,
//    then the c[0]'s at each step should give an indication of the error.
//

template<typename Tx, typename Ty>
void Interpolator<Tx,Ty>::Interpolate(int32 n, Tx* x, Ty* y, Tx X, Ty& Y)
{
    Ty c[50], d[50];   // n must be <= 50, or dynamically allocate.

	// Make an initial guess for the interpolated value (zero)
	Y = Ty(0);

	// do for increasing order of polynomials
	for (int order = 0; order < n; order++){

		// Add the next data point to interpolation
		d[order] = c[order] = y[order];

		// update the diagonal row of polynomial calculations
		for (int i = order-1; i>=0; i--) {

			Ty m = (c[i+1] - d[i]) / (x[i] - x[order]);
			c[i] = (x[i]     - X) * m;
			d[i] = (x[order] - X) * m;
		}

		// update the interpolated value 
		Y = Y + c[0];
	}
}





template<typename Xt, typename Yt>
bool Interpolator<Xt,Yt>::GetY(Xt x, Yt& y)
{
	// Choose points
	int32 i, n;
	if (Choose(x, i, n))
		return Error("Interpolator::GetY - X value out of range");

	// Do the interpolation
	Interpolate(n, &Xv[i], &Yv[i], x, y);

	return OK;
}



template<typename Xt, typename Yt>
bool Interpolator<Xt,Yt>::Choose(Xt x, int32& i, int32& n)
{
	size_t size = Xv.size();

	// Find closest points -- Good enough for now, but may want to fix.
	// (direct lookup since intervals are equal)
	i = (x - Xv[0]) / (Xv[1] - Xv[0]);

    // Choose points around the nearest point
	n = 10;
	i = i-n/2;

    // Adjust if any points are out of bounds
	if (i < 0)       i = 0;
	if (n > size)    n = size;
	if (i+n > size)  i = size - n;

	debug(4, "Interpolator::Choose: x=%g Xv[%d]=%g Xv[%d]=%g  size=%d  \n", 
		(double)x, i, (double)Xv[i], i+n-1, (double)Xv[i+n-1], size);

	return n == 0 || Xv[0] > x || Xv[size-1] < x;
}
    

template<typename Xt, typename Yt>
bool Interpolator<Xt,Yt>::SetY(Xt x, Yt y)
{
	// Must be increasing
	int32 size = Xv.size();
	if (size > 0 && x <= Xv[size-1])
		return Error("Interpolator::SetY - Must have increasing values\n");

	// Must have equal increments between X value
	if (size >= 2 && (Xv[1]-Xv[0]) != (x-Xv[size-1])) 
		return Error("Interpolator::SetY - Must have equal spaced values\n");

    // Add the new point to the vectors
	Xv.push_back(x); Yv.push_back(y);

	return false;
}


// Instantiate for the cases we know we're going to use
template class Interpolator<Time, Position>;
template class Interpolator<Time, Time>;
template class Interpolator<Time, double>;

