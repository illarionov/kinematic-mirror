// Ephemeris is an abstract class describing satellite positions
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

#include "Ephemeris.h"



Position RotateEarth(Position pos, double secs)
{
	// Calculate the earth's rotation angle in the elapsed time
	double omega = secs * OmegaEDot;

	// Rotate by the given angle
	return Position( pos.x*cos(omega) - pos.y*sin(omega),
					 pos.x*sin(omega) + pos.y*cos(omega),
					 pos.z );
}


// TODO: constructor with SatIndex as parameter. Should ALWAYS be set.
Ephemeris::Ephemeris(int Sat, const char* description)
{
    ErrCode = Error();
    SatIndex = Sat;
    Description = description;
}

Ephemeris::~Ephemeris()
{
	Description = "";
}


Ephemerides::Ephemerides()
{
    for (int s = 0; s<MaxSats; s++)
        eph[s] = NULL;
}

Ephemerides::~Ephemerides()
{
    for (int s=0; s<MaxSats; s++) {
        if (eph[s] != NULL)
            delete eph[s];
        eph[s] = NULL;
	}
}

