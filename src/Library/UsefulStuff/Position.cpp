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
#include "Position.h"

Position Wgs84ToPosition(const lla& wgs)
{
	Position pos;
	GeodToXYZ(wgs84_a, wgs84_f,
		DegToRad(wgs.Lat), DegToRad(wgs.Lon), wgs.Alt, pos.x, pos.y, pos.z);
	return pos;
}

lla PositionToWgs84(const Position& pos)
{
	double phi, lambda, alt;
	XYZToGeod(wgs84_a, wgs84_f, pos.x, pos.y, pos.z, phi, lambda, alt);
	return lla(RadToDeg(phi), RadToDeg(lambda), alt);
}


void GeodToXYZ(double a, double f, 
			   double phi, double lambda, double h,
			   double& x, double& y, double& z)
/////////////////////////////////////////////////////////////////////
// Given an elipsoid and polar coordinates, calculate X,Y,Z
//    (From Chpt 14, "Linear Algebra, Geodesy, and GPS", Strang and Borre, 1997.
////////////////////////////////////////////////////////////////////////////
{
	debug("GeodToXYZ: a=%.3f f=%.6f phi=%.3f lambda=%.3f  h=%.3f\n",
		              a, f,   RadToDeg(phi),  RadToDeg(lambda), h);
	double cosphi = cos(phi); 
	double sinphi = sin(phi);

	// Calculate the vector normal to surface of ellipsoid
	double N = a / sqrt(1 - f*(2-f)*sinphi*sinphi);

	// Now, can calculate x,y and x
	x = (N+h) * cosphi * cos(lambda);
	y = (N+h) * cosphi * sin(lambda);
	z = ((1-f)*(1-f)*N + h) * sinphi;

	debug("     N=%.3f x=%.3f  y=%.3f  z=%.3f\n", N, x,y,z);
}



void XYZToGeod(double a, double f,
			   double x, double y, double z,
			   double& phi, double& lambda, double& h)
{
	debug("XYZToGeod a=%.3f  f=%.6f  x=%.3f y=%.3f z=%.3f\n", a,f,x,y,z);

    // set up some constants
	double r = sqrt(x*x + y*y);

	// Iterate. Converges quickly. Do a fixed number of iterations for now.
	h = 0;
	double sinphi = 0;
	for (int i=0; i<10; i++) {
		double N = a / sqrt(1 - f*(2-f)*sinphi*sinphi);
		phi = atan2(z, r*(1 - f*(2-f)*N/(N+h)));
		h = r / cos(phi) - N;
		sinphi = sin(phi);
		debug("  N=%.3f  h=%.3f phi=%.6f\n", N, h, RadToDeg(phi));
	}

	// calculate longitude
	lambda = atan2(y,x);

	debug("   lambda=%.3f  phi=%.3f  h=%.3f\n",RadToDeg(lambda),RadToDeg(phi),h);
}
	





LocalEnu::LocalEnu(Position& center)
{
    TopoCenter = center;

	lla temp = PositionToWgs84(TopoCenter);
	double phi = DegToRad(temp.Lat);
	double lambda = DegToRad(temp.Lon);
	debug("LocalEnu  (%.3f, %.3f, %.3f)  phi=%.6f lambda=%.6f\n",
       TopoCenter.x, TopoCenter.y, TopoCenter.z, RadToDeg(phi),RadToDeg(lambda));

	// easting unit vector
	a[0][0] = -sin(lambda);
	a[0][1] = cos(lambda);
	a[0][2] = 0;

	// northing unit vector
	a[1][0] = -sin(phi) * cos(lambda);
	a[1][1] = -sin(phi) * sin(lambda);
	a[1][2] = cos(phi);

	// up unit vector
	a[2][0] = cos(phi) * cos(lambda);
	a[2][1] = cos(phi) * sin(lambda);
	a[2][2] = sin(phi);
}

enu LocalEnu::ToEnu(Position& p)
{
	Position d = p - TopoCenter;
	debug("ToEnu: p=(%.3f, %.3f, %.3f) d=(%.3f, %.3f, %.3f)\n",
		p.x,p.y,p.z,d.x,d.y,d.z);
	enu result;
	result.e = a[0][0]*d.x + a[0][1]*d.y + a[0][2]*d.z;
	result.n = a[1][0]*d.x + a[1][1]*d.y + a[1][2]*d.z;
	result.u = a[2][0]*d.x + a[2][1]*d.y + a[2][2]*d.z;
	debug("ToEnu: result=(%.3f, %.3f, %.3f)\n", result.e, result.n,result.u);
	
	return result;
}

Position LocalEnu::FromEnu(enu& p)
{
	Position result;
	result.x = a[0][0]*p.e + a[1][0]*p.n + a[2][0]*p.u + TopoCenter.x;
	result.y = a[0][1]*p.e + a[1][1]*p.n + a[2][1]*p.u + TopoCenter.y;
	result.z = a[0][2]*p.e + a[1][2]*p.n + a[2][2]*p.u + TopoCenter.z;
	return result;
}

Position LocalEnu::ToXyz(Position& point)
{
	return point - TopoCenter;
}


