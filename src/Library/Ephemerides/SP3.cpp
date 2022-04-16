// SP3 is a precise ephemeris as described in an "sp3" file
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

#include "util.h"
#include "SP3.h"
#include "RinexParse.h"

bool SkipLine(FILE* f);


//////////////////////////////////////////////////////////////////////
// Read a precise ephemeris file in the "sp3" file format.
//    sp3 files contain satellite position and clock adjustment data for 
//    each gps satellite, updated every 15 minutes.
//
// Note: This code reads the entire ephemeris file and stores it in memory
//   In practice, the data is used in time order, so it would make sense
//   to read the sp3 data only as needed. Get rid on the unnecesary array.
//////////////////////////////////////////////////////////////////////



SP3::SP3(const char* name)
{
	for (int s=0; s<MaxSats; s++)
		eph[s] = new EphemerisInterpolated(s);
	ErrCode = Open(name);
}


SP3::~SP3()
{
}

bool SP3::Open(const char* name)
{
	InputFile in(name);
	if (in.GetError())
		return Error("Can't open Sp3 Ephemeris file %s", name);

	// Do for each satellite position record
	Time time; double Adjust;  Position pos; int32 s;
	while (ReadPos(in, time, s, pos, Adjust) == OK) {
		//debug("sp3;  sat=%d  time=%.0f\n", s, S(time));

		// Add information to interpolator
		((EphemerisInterpolated*)eph[s])->AddSatPos(time, pos, Adjust);
	}

	//for (int32 s=0; s<MaxSats; s++)
	//	debug(4, "sp3: s=%d  MinTime=%.0f MaxTime=%.0f\n", s,eph[s]->MinTime, eph[s]->MaxTime);

	return OK;
}

bool SP3::ReadPos(InputFile& in, Time& t, int32& sat, Position& p, double &Adjust)
{
	// Repeat until a position record was read
	char line[256];
	while (!in.ReadLine(line)) {


		// Case: Position. Return position data.
		if (match(line, 0, "P")) {

			// Read the satellite data
			p.x = GetDouble(line, 4, 14) * 1000;
			p.y = GetDouble(line, 18, 14) * 1000;
			p.z = GetDouble(line, 32, 14) * 1000;
			Adjust = GetDouble(line, 46, 14) / 1000000;
			int svid = ParseSvid(line, 1);
			sat = SvidToSat(svid);
			debug(5, "svid=%d s=%d  p=(%.3f, %.3f, %.3f)  a=%.9f\n",svid,sat,p.x,p.y,p.z,Adjust);
			if (sat == -1)
				return Error("Invalid satellite id in SP3 file\n");

			
			// Set the time and return
			t = GpsTime;
			debug(5,"time=%.0f\n", S(t));
            return OK;
		}

		// Case: Time. Update GpsTime
		else if (match(line, 0, "*")) {

			// Read the time information
			int32 year, month, day, hour, min, sec, nsec;
			year = GetInt(line, 3, 4);
			month = GetInt(line, 8, 2);
			day = GetInt(line, 11, 2);
			hour = GetInt(line, 14, 2);
			min = GetInt(line, 17, 2);
			sec = GetInt(line, 20, 2);
			nsec = GetInt(line, 23, 8);

			// Convert to Time
			GpsTime = DateToTime(year, month, day) 
				          + TodToTime(hour, min, sec, nsec);
			debug(4,"A:date=%d/%d/%d time=%d:%d:%d.%09d GpsTime=%.9f\n", month, day, year, hour, min, sec, nsec, S(GpsTime));
			TimeToDate(GpsTime, year, month, day); TimeToTod(GpsTime, hour, min, sec, nsec);
			debug(4,"B:date=%d/%d/%d time=%d:%d:%d.%09d GpsTime=%.9f\n", month, day, year, hour, min, sec, nsec, S(GpsTime));
		}

		// Otherwise, ignore the line
		else 
			;
	}


	return Error();  // end of file.
}






EphemerisInterpolated::EphemerisInterpolated(int Sat, const char* description)
:Ephemeris(Sat, description)
{
	acc = 1.0;
    MinTime = -1;
    MaxTime = -1;
	Description = "SP3 Ephemeris";
}

bool EphemerisInterpolated::Valid(Time t)
{
	// Don't push too close to the boundary for now.
	//  We really should read more data points instead
	return t >= MinTime+5*NsecPerSec && t <= MaxTime-5*NsecPerSec;
}

double EphemerisInterpolated::Accuracy(Time t)
{
	return acc;
}
bool EphemerisInterpolated::SatPos(Time t, Position &XmitPos, double& Adjust)
{
	if (t < MinTime || t > MaxTime)
		return Error("SP3::SatPos - time out of range\n");

	if (xTime.GetY(t, Adjust))
		return Error("SP3::GetXmit - couldn't interpolate clock error\n");

	if (xPos.GetY(t, XmitPos))  // Question: Do we adjust for clock error?
		return Error("SP3::GetXmit - couldn't interpolate position\n");

	debug(5, "EphemerisInterpolated: s=%d  Adjust=%g  pos=(%.3f, %.3f,%.3f)\n",
		SatIndex, Adjust, XmitPos.x, XmitPos.y, XmitPos.z);

	return OK;
}

bool EphemerisInterpolated::AddSatPos(Time t, Position &XmitPos, double Adjust)
{
	if (xTime.SetY(t, Adjust))
		return Error("Ephemeris Interpolated - couldn't set clock adjustment\n");
	if (xPos.SetY(t, XmitPos))
		return Error("Ephemeris Interpolated -couldn't set position\n");

	if (MaxTime <= 0 || t > MaxTime)  MaxTime = t;
	if (MinTime <= 0 || t < MinTime)  MinTime = t;
	debug(4, "AddSatPos:  t=%.0f  MinTime=%.0f  MaxTime=%.0f\n",S(t),S(MinTime),S(MaxTime));

	return OK;
}


EphemerisInterpolated::~EphemerisInterpolated()
{
}

