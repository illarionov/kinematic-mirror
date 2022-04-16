
#ifndef EPHEMERIS_INCLUDED
#define EPHEMERIS_INCLUDED

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

Position RotateEarth(Position p, double secs);


class Ephemeris  
{
public:
	Ephemeris(int Sat, const char* description = "");
	virtual ~Ephemeris();
	virtual bool SatPos(Time t, Position& XmitPos, double& Adjust) = 0;
	virtual double Accuracy(Time time) = 0; 
	virtual bool Valid(Time time) = 0;
	
	bool GetError() {return ErrCode;}

	virtual void Display(const char* str) 
	    {debug("Generic Ephemeris [%d] %s  %w\n", SatIndex, str, Description);}

//protected:
	bool ErrCode;
        Time MinTime;
        Time MaxTime;

public:
	int32 SatIndex;
	const char* Description;
};

class EphemerisDummy: public Ephemeris
{
public:
	EphemerisDummy(int Sat, const char* description = "EphemerisDummy")
		:Ephemeris(Sat, description) {}
	virtual ~EphemerisDummy()
	    {}
	virtual bool SatPos(Time t, Position& XmitPos, double& Adjust)
	    {return Error("Dummy Ephemeris");}
	virtual double Accuracy(Time time)
	    {return INFINITY;}
	virtual bool Valid(Time time)
	    {return false;}
};




class Ephemerides
{
public:
	Ephemerides();
	Ephemeris& operator[](int s) {return *eph[s];}
	virtual ~Ephemerides();
//protected: needed for simulation. Maybe make it a friend?
	Ephemeris* eph[MaxSats];
};


#endif // !defined(AFX_EPHEMERIS_H__4213776D_15A6_4221_A8A6_F3D5481D714F__INCLUDED_)

