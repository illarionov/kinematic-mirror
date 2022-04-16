
#ifndef SP3_INCLUDED
#define SP3_INCLUDED
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



#include "Ephemeris.h"
#include "Interpolator.h"
#include "util.h"
#include "Parse.h"  // GetLine
#include "InputFile.h"



class EphemerisInterpolated: public Ephemeris
{
public:
	EphemerisInterpolated(int Sat, const char* description = "SP3 Ephemeris");
    virtual bool Valid(Time t);
    virtual double Accuracy(Time t);
	virtual bool SatPos(Time t, Position& XmitPos, double& Adjust);

	bool AddSatPos(Time t, Position& XmitPos, double Adjust);
	virtual ~EphemerisInterpolated();

private:
	Interpolator<Time,double>   xTime;
	Interpolator<Time,Position> xPos;
	Time MinTime, MaxTime;
	double acc;
};


class SP3: public Ephemerides
{
public:
	SP3(const char* name);
	virtual ~SP3();
	bool Open(const char* name);
	bool GetError() { return ErrCode;}

private:
	bool ReadPos(InputFile& in, Time& t, int32& sat, Position& p, double& Adjust);
	Time GpsTime;
	bool ErrCode;
};
#endif // !defined(AFX_SP3_H__F4246127_53FE_4572_BA20_2DD65F24ECAD__INCLUDED_)

