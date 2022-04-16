
//    Part of Kinetic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    www.precision-gps.org
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



#include "Observations.h"

Observations::Observations()
{
	GpsTime = -1;
	RoverPos = 0;
	BasePos = 0;
	for (int s=0; s<MaxSats; s++) {
		obs[s].ValidCode = false;
		obs[s].ValidPhase = false;
		obs[s].Slip = true;
		obs[s].SatPos = 0;
	}

}

Observations::Observations(RawReceiver& base, RawReceiver& rover, Ephemerides& eph)
{
	Init(base, rover, eph);
}

void Observations::Init(RawReceiver& base, RawReceiver& rover, Ephemerides& eph)
{
	GpsTime = base.GpsTime;
	RoverPos = rover.Pos;   // These are approximations and are not used in the solution
	BasePos = base.Pos;

	// Do for each satellite
	for (int s=0; s<MaxSats; s++) {

		// Assume the observation is invalid until shown otherwise
		Observation& o = obs[s];
		o.Sat = s;
		o.ValidCode = false;
		o.ValidPhase = false;
		o.Slip = true;

		// If either of the receivers aren't tracking, then we are done with this sat
		if (!base.obs[s].Valid || !rover.obs[s].Valid || !eph[s].Valid(GpsTime)) 
			continue;

		// Get the satellite's position
		double Adjust;
		ErrCode = eph[s].SatPos(GpsTime, o.SatPos, Adjust);
		if (ErrCode != OK) return;
		debug(2, "Observations  s=%d  SatPos=(%.3f, %.3f, %.3f)\n",s, o.SatPos.x, o.SatPos.y, o.SatPos.z);

		// Adjust the satellite's position to compensate for the earth's rotation
		//  during the signal's transit
		double TransitTime = Range(o.SatPos-RoverPos) / C;
		o.SatPos = RotateEarth(o.SatPos, -TransitTime);
		
		// Make sure we have valid measurements.
		o.ValidCode   = base.obs[s].PR != 0    && rover.obs[s].PR != 0;
		o.ValidPhase  = base.obs[s].Phase != 0 && rover.obs[s].Phase != 0;
		o.Slip        = base.obs[s].Slip       || rover.obs[s].Slip;


#ifdef NotNow
            // solve integer mseconds, but we seem to know if it is 0-8.
            double range = Range(BasePos - o.SatPos);
            double adjust = round((range - base.obs[s].PR), C/1000);
            base.obs[s].PR += adjust;
            debug("  s=%d  range=%.3f  adjust=%.3f  PR=%.3f\n", s, range, adjust, base.obs[s].PR);
#endif
		// Keep track of pseudorange and phase differences for generating equations
		o.PR = rover.obs[s].PR - base.obs[s].PR;
		o.Phase = rover.obs[s].Phase - base.obs[s].Phase;
		debug("Observations: s=%d  b.PR=%.3f  b.range=%.3f  r.PR=%.3f  r.range=%.3f\n",
                s, base.obs[s].PR, Range(o.SatPos-BasePos), 
                   rover.obs[s].PR, Range(o.SatPos-RoverPos));

		// Calculate the default weights to use.
		o.CodeWeight = .01;
		o.PhaseWeight = 1;
	}

#ifndef TESTING
	double bsum = 0;
	double rsum = 0;
	int count = 0;
	for (int s=0; s<MaxSats; s++) {
		if (!obs[s].ValidCode) continue;
		bsum += base.obs[s].PR;
		rsum += rover.obs[s].PR;
		count++;
	}
	double bavg = bsum / count;
	double ravg = rsum / count;

	for (int s=0; s<MaxSats; s++) {
		if (!obs[s].ValidCode) continue;

	    debug("Observations s=%d  base=%.3f  rover=%.3f  dPR=%.3f\n",
			s, base.obs[s].PR-bavg, rover.obs[s].PR-ravg, 
			   (base.obs[s].PR-bavg) - (rover.obs[s].PR-ravg) );
	}
#endif

}

		
Observations::~Observations()
{
}

