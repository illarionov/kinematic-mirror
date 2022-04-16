#ifndef UTIL_INCLUDED
#define UTIL_INCLUDED
#define DEBUG
#define VERSION "Beta Snapshot 6"
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

// Use the overloaded templates for strcpy
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <algorithm>  // for "min" and "max"


namespace std {};
using namespace std;

// some looping conditions to use in place of "while(xxx)"
#define until(condition) while (!(condition))
#define forever while(true)

// Standard integer types
typedef char int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef long int32;
typedef unsigned long uint32;
typedef int64_t int64;
typedef uint64_t uint64;

#include "Position.h"
#include "GpsTime.h"

// Special integer types
typedef uint8 byte;

extern int DebugLevel;
#ifndef DEBUG
inline static void debug(const char* fmt, ...){}
inline static void debug(int level, const char* fmt, ...){}
inline static void debug_buf (int level, const byte* buf, size_t size) {}
inline static void vdebug(int level, const char* fmt, va_list args){}
template<typename Ta, typename Tb> inline static
void DebugArray(Ta& A, int32 MinRow, int32 MaxRow, int32 MinCol, 
					   int32 MaxCol, Tb& B, const char* s="")  {}


#else
void debug(const char* fmt, ...);
void debug(int level, const char* fmt, ...);
void debug_buf(int level, const byte* buf, size_t size);
void vdebug(int level, const char* fmt, va_list args);
template<typename Ta, typename Tb>
static void DebugArray(Ta& A, int32 MinRow, int32 MaxRow, int32 MinCol, 
						 int32 MaxCol, Tb& B, const char* s="")
{
	debug("Array[%d..%d][%d..%d]  %s\n", MinRow, MaxRow, MinCol, MaxCol,s);
	for (int i=MinRow; i<=MaxRow; i++) {
		for (int j=MinCol; j<=MaxCol; j++)
			debug(3,"%10.6f ", A[i][j]);
		debug(3,"    %10.6f\n", B[i]);
	}

}

#endif


// Some useful constants
//static const double INFINITY = 99e99;  //TODO get the ieee value
static const double PI = 3.1415926535898;
static const double EPS = 1.0e-20;
static const double C = 2.99792458e+08; // speed of light in m/sec
static const double L1Freq = 1575420000;
static const double L1WaveLength = C / L1Freq;
static const int32 MaxSats = 66; // SV 1-32 GPS, SV 120-152 EGNOS, 0 unused
static const int32 MaxChannels = 24;

static const double wgs84_f = 1.0/298.2572235630;  // flattening of earth
static const double wgs84_a = 6378137.0;  // major axis of earth
static const double mu = 3.986005e+14; // earth gravity constant
static const double OmegaEDot = 7.2921151467e-5; // earth rotation


template <typename T>
T abs(T val)
{
	if (val > 0) return val;
	else         return -val;
}


// powers of two for scaling.
inline double p2(int32 n) {return ((uint64)1) << n;}


int SvidToSat(int svid);
int SatToSvid(int s);

// Convert between rinex signal level and snr
double LevelToSnr(int level);
int SnrToLevel(double snr);


inline double DegToRad(double degrees)
{
	return degrees/180*PI;
}

inline double RadToDeg(double radians)
{
	return radians*180/PI;
}

Position Wgs84ToPosition(double lat, double lon, double alt);

void Wgs84ToXYZ(double lat, double lon, double alt,
				double& x, double& y, double& z);

void GeodToXYZ(double a, double finv, 
			   double phi, double lambda, double h,
			   double& x, double& y, double& z);

void PositionToWgs84(Position p, double& lat, double& lon, double& alt);
void XYZToWgs84(double x, double y, double z,
				double& lat, double& lon, double& alt);
void XYZToGeod(double a, double finv,
			   double x, double y, double z,
			   double& lat, double& lon, double& alt);

bool Event(const char *fmt, ...);
bool Error(const char *fmt, ...);
bool SysError(const char *fmt, ...);
bool Verror(const char* fmt, va_list args);
inline bool Error() {return true;}
void ClearError();
int ShowErrors();
static const bool OK = false;


template<typename SRC, typename DST> void copy(const SRC &src, DST &dst)
{
	const byte* s = (const byte*)&src;
	byte* d = (byte*)&dst;
	size_t len = min(sizeof(src), sizeof(dst));
	for (size_t i = 0; i<len; i++)
		d[i] = s[i];
}

template<typename DST> void copy(const char* src, DST &dst)
{
	strcpy((char*)&dst, src);
}

inline void copy(const char* src, char* dst)
{
	strcpy(dst, src);
}




#ifdef NOT_NEEDED_ANY_MORE_X
#undef min
#undef max 

template <typename T> T min(T a, T b)
{
	return (a < b)?a: b;
}

template <typename T> T max(T a, T b)
{
	return (a > b)? a: b;
}
#endif


inline double round(double value)
{
	return floor(value + .5);
}

template <typename T> inline
void Swap(T& a, T& b)
{
	T temp = a;
	a = b;
	b = temp;
}


inline double round(double value, double rnd)
{
	return floor(value/rnd+.5) * rnd;
}



template <typename T>
T RoundDown(T value, T base=1)
{
	return (value/base)*base;
}

inline double RoundDown(double value, double base=1)
{
	return floor(value/base)*base;
}


// Random numbers
double Uniform();
double Normal();
double Normal(double mean, double dev);

// strings
inline
bool Same(const char* a, const char* b) {return strcmp(a,b) == 0;}

inline
bool Match(const char* str, const char* pat, const char*& nxt)
//////////////////////////////////////////////////////////////
// Match is true if the pattern matches the head of the string
///////////////////////////////////////////////////////////////
{
	// See if the pattern matches the head of the string
	for (; *pat != '\0'; str++,pat++)
		if (*str != *pat) break;

	// If it matches, "next" points to the rest of the string
	if (*pat == '\0')
		nxt = str;

	return *pat == '\0';
}

inline
bool IsEmpty(const char* str)
{
    return str[0] == 0;
}


void Encode(char* buf, const char* user, const char* pwd);


#ifndef Windows
void Sleep(int msec);
const char* GetLastError();
#endif

#endif // !defined(AFX_UTIL_H__DAB7AFBC_C1AF_44A4_9619_5D69508077F9__INCLUDED_)

