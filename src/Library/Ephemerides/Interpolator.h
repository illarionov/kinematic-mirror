
#ifndef INTERPOLATOR_INCLUDED
#define INTERPOLATOR_INCLUDED
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



#include <vector>
using namespace std;


template<typename Tx, typename Ty>
class Interpolator
{
public:
	Interpolator();
	virtual ~Interpolator();
	bool GetY(Tx x, Ty& y);
	bool SetY(Tx x, Ty y);

private:
	vector<Tx> Xv;
	vector<Ty> Yv;

	bool Choose(Tx x, int32& i, int32& n);
    void Interpolate(int32 n, Tx* x, Ty* y, Tx X, Ty& Y);
};



#endif // !defined(AFX_INTERPOLATOR_H__2A049B3F_3393_4BBD_9565_7B4E8E0C984D__INCLUDED_)

