
//    Part of Kinematic, a utility for GPS positioning
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

#include "RawReceiver.h"

int RawReceiver::HZ = 1;

RawReceiver::RawReceiver()
{
	RawTime=-1;
	PreviousTime = -1;

	Adjust=0;
	PreviousTow=0;
	GpsTime=0;
	for (int s=0; s<MaxSats; s++) {
		obs[s].Sat = s;
		obs[s].Valid = false;
		PreviousPhase[s] = 0;
	}
}



bool RawReceiver::AdjustToHz(bool IncludeDoppler)
{
	debug("AdjustToHz: HZ=%d  IncludeDoppler=%d\n", HZ, IncludeDoppler);
	return AdjustToTime(NearestHz(GpsTime, HZ), IncludeDoppler);
}
 
 
bool RawReceiver::AdjustToTime(Time t, bool IncludeDoppler)
{
    // Update the clock
	RawTime = GpsTime;
	GpsTime = t;
	Adjust = S(GpsTime - RawTime);	
      debug("RawReceiver::AdjustTotime: GpsTime=%lld Adjust=%.9f\n",GpsTime,Adjust);

      Time Drift = ((RawTime - PreviousRaw) - (GpsTime - PreviousTime));
      debug("  Drift(nsec)=%lld  Drift(m)=%.3f \n", Drift, S(Drift)*C);


	// Do for each satellite
	for (int s=0; s<MaxSats; s++) {
		RawObservation& o=obs[s];
		if (!o.Valid) continue;

            double Doppler = (o.Phase - PreviousPhase[s]) / S(GpsTime - PreviousTime);
            double Doppler2 = (o.Phase - PreviousPhase[s]) / S(RawTime - PreviousRaw);
            if (PreviousPhase[s] != 0 && o.Phase != 0)
                debug ("  sat=%d  o.doppler=%.3f Doppler=%.3f Doppler2=%.3f\n", s, o.Doppler, Doppler, Doppler2);
            if (obs[s].Valid && obs[s].Phase != 0)   PreviousPhase[s] = obs[s].Phase;
		else                                     PreviousPhase[s] = 0;

            // Interpolate PR
            double factor = S(GpsTime-PreviousTime) / S(RawTime - PreviousRaw);
            double PR = PreviousPR[s] * (1-factor) + o.PR * factor;
            double PR2 = PreviousPR[s] * (1-1/factor) + o.PR * 1/factor;
            if (PreviousPhase[s] != 0 && o.Phase != 0)
                debug ("  sat=%d  o.PR=%.3f PR'=%.3f PR2'=%.3f\n", s, o.PR, PR, PR2);
            if (obs[s].Valid && obs[s].PR != 0)   PreviousPR[s] = obs[s].PR;
		else                                     PreviousPR[s] = 0;

		// Adjust the satellite's range and phase to the start of epoch
		double freq = L1Freq;
		if (IncludeDoppler) freq -= o.Doppler;

		//if (o.PR != 0)    o.PR += Adjust*freq*L1WaveLength;
            if (o.PR != 0)    o.PR += Adjust*C;
		if (o.Phase != 0) o.Phase += Adjust*freq;

		debug("    Sat=%d  PR(hz)=%.3f  Phase=%.3f o.Doppler=%.3f\n",
			o.Sat, o.PR/L1WaveLength, o.Phase, o.Doppler);

	}   

	// Save observations for future drift calculation
	PreviousTime = GpsTime;
      PreviousRaw = RawTime;
	//for (int s=0; s<MaxSats; s++)
	//	if (obs[s].Valid && obs[s].Phase != 0)   PreviousPhase[s] = obs[s].Phase;
	//	else                                     PreviousPhase[s] = 0;

    return OK;
}




// DEFUNCT - called by sirf code.
Time RawReceiver::AdjustToHz(Time t, double tow)
{ 
    // Find a valid satellite. 
	int s;
	for (s=0; s<MaxSats; s++)
		if (obs[s].Valid && obs[s].PR != 0) break;
	if (s == MaxSats) return NearestSecond(UpdateGpsTime(t,tow));

	// Adjust drifting clock so the pseudorange is 
	//   within the interval [0..C/4]. Some units, Garmin in particular,
	//   let the clocks drift beyond reasonable amounts.
	double NewPR = fmod(obs[s].PR, C/4);
	double Adjust = (NewPR - obs[s].PR) / C;
    debug(2,"AdjustToHz: s=%d  PR=%.3f  NewPR=%.3f  Adjust=%g\n",
				          s, obs[s].PR,  NewPR,      Adjust);

	// Find the epoch closest to the adjusted time;
	Time Epoch = NearestHz( UpdateGpsTime(t, tow+Adjust), HZ );

	// Adjust the measurements to the epoch.
	AdjustToTime(Epoch);

	return Epoch;
}





void RawReceiver::AdjustToTime(Time t, double tow)
{
    // How much adjustment is needed to get to desired time
	int Week = ClosestWeek(t, tow);
	Adjust = (GpsWeek(t)-Week) * 7*24*60*60 + GpsTow(t)-tow;

	// Get the interval since the last measurement
	double DeltaTime = tow - PreviousTow;
	if (DeltaTime < 0) DeltaTime += 7*24*60*60;
	PreviousTow = tow;

	debug("RawReceiver::AdjustTotime: Week=%d tow=%.6f PreviousTow=%.6f DeltaTime=%6f  Adjust=%.6f\n", 
		Week, tow, PreviousTow, DeltaTime, Adjust);

	// Do for each satellite
	for (int s=0; s<MaxSats; s++) {
		RawObservation& o=obs[s];
		if (!o.Valid) continue;

		// Calculate Doppler as -DeltaPhase/DeltaTime
		//  (or use given Doppler if it can't be calculated)
		//  (Need to decide which takes priority.)
		double Doppler = o.Doppler;
		if (!o.Slip && PreviousPhase[s]!=0 && o.Phase!=0)  
			Doppler = -(o.Phase - PreviousPhase[s]) / DeltaTime;
		PreviousPhase[s] = o.Phase;

		debug(2,"RawReceiver::AdjustToTime: Doppler=%.3f o.Doppler=%.3f\n", Doppler, o.Doppler);

		// Adjust the satellite's range and phase to the start of epoch
		if (o.PR != 0)    o.PR += Adjust*C;
		if (o.Phase != 0) o.Phase += Adjust*L1Freq;   // Consider adjusting by doppler too
		if (o.Doppler == 0) o.Doppler = Doppler;      //   at least for clock drift portion

		debug(2,"RawReceiver::AdjustToTime: Sat=%d  PR=%.3f  Phase(m)=%.3f o.Doppler(m)=%.3f\n",
			o.Sat, o.PR, o.Phase*L1WaveLength, o.Doppler*L1WaveLength);
	}
}

RawReceiver::~RawReceiver()
{
}

