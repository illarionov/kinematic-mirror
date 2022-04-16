// Triple represents XYZ Triples and does basic math on them
//    Part of Kinetic, a collection of utilities for GPS Tripleing
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

#ifndef POSITION_included
#define POSITION_included

#include <math.h>

class Triple {
public:
	double a[3];

	inline Triple(double a0=0, double a1=0, double a2=0){a[0]=a0; a[1]=a1; a[2]=a2;}
	inline const double& operator[] (int i) const {return a[i];}
	inline double& operator[] (int i) {return a[i];}
};


class Position
{
public:
	double x, y, z;
	inline Position(){}
	inline Position(double X, double Y, double Z): x(X),y(Y),z(Z) {}
	inline Position(void* zero): x(0),y(0),z(0) {}
	inline bool operator==(const Position& p) const
	    {return (x==p.x && y==p.y && z==p.z);}

    // Conversions with Triple to enable math operators
	inline operator Triple&() {return *(Triple*)this;}
	inline Position(const Triple& t): x(t[0]), y(t[1]), z(t[2]) {}
};

class enu
{
public:
	double e, n, u;
	inline enu(){}
	inline enu(double E, double N, double U=0) :e(E),n(N),u(U) {}

	// Conversions with Triple to enable math operators
	inline operator Triple&() {return *(Triple*)this;}
	inline enu(const Triple& t): e(t[0]), n(t[1]), u(t[2]) {}
};

class lla
{
public:
	double Lat, Lon, Alt;
	inline lla(){}
	inline lla(double lat, double lon, double alt=0){Lat = lat; Lon = lon; Alt = alt;}

	inline operator Triple&() {return *(Triple*)this;}
	inline lla(const Triple& t): Lat(t[0]), Lon(t[1]), Alt(t[2]) {}
};


class LocalEnu
{
public:
	LocalEnu(Position& center);
	enu ToEnu(Position& point);
	Position FromEnu(enu& point);
	Position ToXyz(Position& point);

private:
	double a[3][3];
	Position TopoCenter;
};


Position Wgs84ToPosition(const lla& wgs);
lla PositionToWgs84(const Position& pos);


inline
Triple operator* (const Triple& p, double scaler)
{
	return Triple(p[0]*scaler, p[1]*scaler, p[2]*scaler);
}

inline
Triple operator*(double scaler, const Triple& p)
{
	return p * scaler;
}

inline 
double operator*(const Triple& a, const Triple& b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}


inline
Triple operator/(const Triple& p, double scaler)
{
	return Triple(p[0]/scaler, p[1]/scaler, p[2]/scaler);
}

inline
Triple operator-(const Triple& a, const Triple& b)
{
	return Triple(a[0]-b[0], a[1]-b[1], a[2]-b[2]);
}

inline
Triple operator+(const Triple& a, const Triple& b)
{
	return Triple(a[0]+b[0], a[1]+b[1], a[2]+b[2]);
}

inline Triple operator+=(Triple& a, const Triple& b)
{
	a = a + b;
	return a;
}

inline Triple operator-=(Triple& a, const Triple& b)
{
	a = a - b;
	return a;
}



inline
double Range(const Triple& Pos)
{
	return sqrt(Pos*Pos);
}

inline
Triple Unit(const Triple& Pos)
{
	return Pos / Range(Pos);
}




#endif // Triple_included

