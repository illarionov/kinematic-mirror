#ifndef TIME__INCLUDED
#define TIME__INCLUDED// Part of Kinematic, a utility for GPS positioning
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




#include "util.h"

// Represent time as nanoseconds from 1981.
//    We could have used 2001, but we might want to process old data.
//    Start dates of 4n+1 make leap year calculations simpler.
typedef int64 Time;   // Used for delta time, so needs to be signed

static const Time NsecPerSec = 1000000000;
static const Time NsecPerMinute = 60*NsecPerSec;
static const Time NsecPerHour = 60*NsecPerMinute;
static const Time NsecPerDay = 24*NsecPerHour;
static const Time NsecPerWeek = 7*NsecPerDay;
static const Time NsecPerYear = (Time)365.25*NsecPerDay;
static const int32 SecPerWeek = 7*24*60*60;

// Various time/date conversion routines
void TimeToDoy(Time time, int32& year, int32& day);
void TimeToTod(Time time, int32& hours, int32& minutes, int32& seconds, int32& nsec);
void TimeToDate(Time time, int32& year, int32& month, int32& day);

void DoyToDate(int32 year, int32 doy, int32& month, int32& day);
int32 DateToDoy(int32 year, int32 month, int32 day);

Time DateToTime(int32 year, int32 month, int32 day);
Time TodToTime(int32 hour, int32 min, int32 sec, int32 nsec);
Time TodToTime(int32 hour, int32 min, double sec);
Time DoyToTime(int32 year, int32 Doy);

static const Time MinTime = 0x8000000000000000ll;
static const Time MaxTime = 0x7fffffffffffffffll;


inline double S(Time t) {return t / (double)NsecPerSec;}
inline Time T(double s) { return (Time)(s * NsecPerSec); }

// The GPS system represents time as:
//   gps weeks  -  nr of weeks since Jan 6, 1980, modulo 1024.)
//   gps time of week - nr of seconds since saturday midnight
// Since gps weeks may be modulo 1024, it is necessary to give
//   an approximate year when converting. 
Time ConvertGpsTime(int32 GpsWeeks, double TOW, int32 ApproxYear=2010);
Time UpdateGpsTime(Time GpsTime, double Tow);
int ClosestWeek(Time, double tow);
double GpsTow(Time t);
int32  GpsWeek(Time t);

// Garmin GPS systems represent time as:
//    garmin days - days since Dec 31, 1989
//    garmin tow  - seconds since starting of week.
// (This should be moved to Garmin specific code later on)
Time ConvertGarminTime(int32 GarminDays, double TOW);


inline Time NearestHz(Time t, int Hz = 1)
{
	Time NsecPerHz = NsecPerSec / Hz;
	t += NsecPerHz/2;
	return t - (t%NsecPerHz);
}

inline Time NearestSecond(Time t)  // defunct
{
	// round up and truncate to even second
	t += NsecPerSec/2;
	return t - (t%NsecPerSec);
}


Time GetCurrentTime();

extern const char *MonthName[];

#endif // !defined(AFX_TIME_H__A50736A9_6AED_45EB_99CC_80CEC4D6AE91__INCLUDED_)

