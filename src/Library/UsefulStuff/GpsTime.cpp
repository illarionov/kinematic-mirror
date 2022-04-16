// Time contains a set of utilities for working with time
//    Internally, we represent time as int64 nanoseconds.
//    Part of Kinematic, a utility for GPS positioning
//
// Copyright (C) 2005  John Morris    kinematic@precision-gps.org
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


// Time.cpp: implementation of the Time class.
//
//////////////////////////////////////////////////////////////////////
#include "util.h"
#include <math.h>
#include <sys/time.h>


Time ConvertGarminTime(int32 GarminDays, double TOW)
{
	// Convert Dec 31, 1989 to time since Jan 1, 1981.
	Time GarminOrigin = (9*365+3) * NsecPerDay ;

	// Get time of day
	Time TOD = (Time)(TOW * NsecPerSec) % NsecPerDay;

	// Calculate the time
	Time result = GarminOrigin + GarminDays*NsecPerDay + TOD;
	//debug("ConvertGarmin GarminDays=%d TOW=%.0f  result=%.0f\n",GarminDays,TOW,S(result));
    return result;
}


// GPS time starts Jan 6, 1980. Our time starts Jan 1, 1981.
Time GpsOrigin = -361 * NsecPerDay;

Time ConvertGpsTime(int32 GpsWeeks, double TOW, int32 ApproxYear)
{

    // If GpsWeeks is modulo 1024, fit it to closest 1024 week cycle.
	if (GpsWeeks < 1024) {

		// Figure out approximately how many 1024 week cycles have elapsed
        int32 ApproxWeeks = (ApproxYear - 1980) * 52;
	    int32 Cycles = (int32)floor( (ApproxWeeks - GpsWeeks) / 1024.0 + .5);
        debug(3,"ApproxWeeks=%d Cycles=%d  GpsWeeks=%d\n",ApproxWeeks,Cycles,GpsWeeks);
		// Adjust gps weeks so it isn't modulo 1024.
	    GpsWeeks = GpsWeeks + 1024 * Cycles;
	}


	Time result = GpsOrigin + GpsWeeks*NsecPerWeek + (Time)(TOW*NsecPerSec);
	debug(3,"ConvertGpsTime: GpsWeeks=%d  TOW=%.9f  result=%.9f\n",GpsWeeks,TOW,S(result));
    debug(3,"   GpsWeeks=%d  TOW=%.0f\n", GpsWeek(result), GpsTow(result));
	return result;
}

double GpsTow(Time t)
{
	return  S((t-GpsOrigin) % NsecPerWeek);
}

int32 GpsWeek(Time t)
{
	return (t-GpsOrigin) / NsecPerWeek;
}
	

int ClosestWeek(Time time, double tow)
{
	int WN = GpsWeek(time);
	double OldTow = GpsTow(time);
	if      (tow > OldTow + 7*24*60*60/2)  WN--;
	else if (tow < OldTow - 7*24*60*60/2)  WN++;
	else                                          ;

	return WN;
}



Time UpdateGpsTime(Time time, double NewTow)
///////////////////////////////////////////////////////////////////////
// UpdateGpsTime updates an existing time from a new Time of Week.
//   (Assumes the new time is reasonably close to the old time)
///////////////////////////////////////////////////////////////////////
{
	int WN = ClosestWeek(time, NewTow);
	Time NewTime = ConvertGpsTime(WN, NewTow);
	debug(2, "UpdateGpsTime - time=%.0f NewTow=%.3f WN=%d  newtime=%.0f\n",
		S(time), NewTow, WN, S(NewTime) );
	return NewTime;
}




void TimeToDoy(Time time, int32& year, int32& day)
{
	// Get number of days since Jan 1,1981
	day = (int32)(time / NsecPerDay);
			
	// Get Leap years from 1981 and days from previous leap year
	int32 LeapYears = day/(365*4+1); day = day%(365*4+1);

	// Get years and days from previous leap year
	year = day/365; day = day%365;

	// Get calendar year
	year = 1981 + LeapYears*4 + year;

	// Jan 1 is day 1, not day 0
	day++;
}

void TimeToTod(Time time, int32& hours, int32& minutes, int32& seconds, int32& nsec)
{
	// Get time of day in nanoseconds
	Time nanosec = time % NsecPerDay;

	// Get time of day in seconds and nanoseconds
	seconds = (int32)(nanosec/NsecPerSec); 
	nsec = (int32)(nanosec%NsecPerSec);

	// Get time of day in minutes, seconds and ...
	minutes = seconds / 60;  seconds = seconds % 60;

	// Get time of day in hours, minutes, ...
	hours = minutes / 60;  minutes = minutes % 60;
	//debug("TimeToTOD: time=%.9f hours=%d minutes=%d seconds=%d nsec=%d\n",
	//	             S(time),  hours,   minutes,   seconds,   nsec);
}


void TimeToDate(Time time, int32& year, int32& month, int32& day)
{
	// Convert time to year and day of year
	int32 DOY;
	TimeToDoy(time, year, DOY);

	// Convert day of year to date
	DoyToDate(year, DOY, month, day);
	debug(3,"TimeToDate: time=%.9f year=%d month=%d day=%d\n",S(time),year,month,day);
}



const char *MonthName[] = {"", "Jan","Feb","Mar","Apr", "May","Jun","Jul","Aug", 
"Sep", "Oct", "Dec"};



Time DateToTime(int32 year, int32 month, int32 day)
{
	int32 doy = DateToDoy(year, month, day);
	Time result = DoyToTime(year, doy);
	//debug("DateToTime: year=%d month=%d day=%d result=%.0f\n",year,month,day,S(result));
	return result;
}

Time TodToTime(int32 hour, int32 min, double dsec)
{
	double sec = floor(dsec);
	return TodToTime(hour, min, (int32)sec, (int32)( (dsec-sec)*NsecPerSec));
}

	
Time TodToTime(int32 hour, int32 min, int32 sec, int32 nsec)
{
	return (((hour * 60) + min) * 60 + sec) * NsecPerSec + nsec;
}


static int DaysPerMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

void DoyToDate(int32 year, int32 doy, int32& month, int32& day)
{
	// Figure out whether this is a leap year or not
	if ((year%4) == 0)  DaysPerMonth[2] = 29;
	else                DaysPerMonth[2] = 28;

	// Get the Calendar date
	day = doy;
	for (month=1; day > DaysPerMonth[month] && month < 12; month++)
		day -= DaysPerMonth[month];
}

int32 DateToDoy(int32 year, int32 month, int32 day)
{
	// Figure out whether this is a leap year ornot
	if ((year%4) == 0) DaysPerMonth[2] = 29;
	else               DaysPerMonth[2] = 28;

	// Figure out which day and month
	int32 doy = day;
	for (int i = 1; i < month; i++)   // can be preadded
		doy += DaysPerMonth[i];
   
	return doy;
}


Time DoyToTime(int32 year, int32 doy)
{
	int32 leapyears = (year - 1981) / 4;
	int32 years = (year - 1981) % 4;
	return (leapyears*(4*365+1) + years*365 + doy - 1) * NsecPerDay;
}



Time GetCurrentTime()
{
    // Get time since Jan 1, 1970
    struct timeval tv;
    if (gettimeofday(&tv, 0) == -1)  return 0;

    // time is since Jan 1, 170. Convert to time since Jan 1, 1981
    tv.tv_sec -= (11*365+3)*24*60*60;

    // Convert to nanoseconds
    return tv.tv_sec * NsecPerSec + tv.tv_usec * 1000ll;
}
