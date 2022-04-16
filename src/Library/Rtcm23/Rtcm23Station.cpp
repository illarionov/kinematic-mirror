
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


#include "Rtcm23Station.h"
#include "EphemerisXmit.h"




Rtcm23Station::Rtcm23Station(Stream& out, RawReceiver& gps, int id, int health)
: Gps(gps), Out(out)
{
	debug("Rtcm23Station: id=%d  health=%d\n", id, health);
	Health = health;
	StationId = id;
	SequenceNr=0;
	ErrCode = Out.GetError();
	StationPos = Position(0,0,0);
	FirstEpoch = true;

	// Setup times so we output all records at the beginning
	TimeTagTime = -1;
	AntennaRefTime = -1;
	for (int s=0; s<MaxSats; s++)
		EphemerisTime[s] = -1;

	// We also need to keep track of slips
	for (int s=0; s<MaxSats; s++) {
		PreviouslyValid[s] = false;
		CumulativeLossOfLock[s] = 0;
	}
}


bool Rtcm23Station::OutputEpoch()
{
	ErrCode = OK;

	// if the time is right, output the various records
	if (TimeTagTime <= Gps.GpsTime)
		ErrCode |= OutputTimeTag(TimeTagTime);
	if (AntennaRefTime <= Gps.GpsTime) {
		ErrCode |= OutputAntennaTypeDef(AntennaRefTime);
		ErrCode |= OutputAntennaRef(AntennaRefTime);
	}

	// Send the measurements for this epoch
	Time dummy;
	ErrCode |= OutputPseudorange(dummy);
	ErrCode |= OutputCarrierPhase(dummy);

	// if the time is right, send the ephemerides
	//  TODO: check if iode has changed as well.
	for (int s=0; s<MaxSats; s++)
		if (EphemerisTime[s] <= Gps.GpsTime)
			ErrCode |= OutputEphemeris(s, EphemerisTime[s]);

	return ErrCode;
}




bool Rtcm23Station::OutputTimeTag(Time& NextTime)
{
    // Schedule the TimeTag every two minutes on the hour
	NextTime = Gps.GpsTime - Gps.GpsTime%(2*NsecPerMinute) + 2*NsecPerMinute;

    // get the values used in the record
	int Week = GpsWeek(Gps.GpsTime) % 1024;
	int HourOfWeek = (Gps.GpsTime % NsecPerWeek) / NsecPerHour;
	int LeapSec = Gps.LeapSec;
	debug("OutputTimeTag: week=%d  hour=%d  leap=%d\n", 
		Week, HourOfWeek, LeapSec);

	// Create the frame
	Frame Rtcm23(3);
	Header(Rtcm23, Gps.GpsTime, 14);
	Rtcm23.PutWord(3, (Week<<20)|(HourOfWeek<<12)|(LeapSec<<6));

	return Out.WriteFrame(Rtcm23);
}

bool Rtcm23Station::OutputAntennaTypeDef(Time& NextTime)
{
	NextTime = Gps.GpsTime + 2*NsecPerMinute;
	return OK;
}
	
bool Rtcm23Station::OutputAntennaRef(Time& NextTime)
{
	// Schedule the record every two minutes, one minute after the hour
	NextTime = Gps.GpsTime - Gps.GpsTime%(2*NsecPerMinute) + 3*NsecPerMinute;

	// If we don't have an assigned station position, use gps position
	if (StationPos == Position(0,0,0))
		StationPos = Gps.Pos;

	// Get the information about the antenna reference(38 bit integers)
	int64 x = StationPos.x * 10000;
	int64 y = StationPos.y * 10000;
	int64 z = StationPos.z * 10000;

	debug("WriteAntennaRef:  Pos=(%.3f, %.3f, %.3f) \n",
		StationPos.x, StationPos.y, StationPos.z);

	// Break up the values for insertion into the records
	uint32 xhi  = (x >> 14) & 0xffffff;
	uint32 xlow = x & 0x3fff;
	uint32 yhi  = (y >> 30) & 0xff;
	uint32 ymid = (y >> 6) & 0xffffff;
	uint32 ylow = y & 0x3f;
	uint32 zhi  = (z >> 22) & 0xffff;
	uint32 zlow = z & 0x3fffff;

	// Build up the frame, no antenna height for now.
	Frame Rtcm23(7);
	Header(Rtcm23, Gps.GpsTime, 24);
	Rtcm23.PutField(3, 1, 24, x>>14);
	Rtcm23.PutField(4, 1, 14, x);
	Rtcm23.PutField(4, 17, 24, y>>30);
	Rtcm23.PutField(5, 1, 24, y>>8);
	Rtcm23.PutField(6, 1, 6, y);
	Rtcm23.PutField(6, 9, 24, z>>22);
	Rtcm23.PutField(7, 1, 22, z);
	return Out.WriteFrame(Rtcm23);
}


bool Rtcm23Station::OutputPseudorange(Time& NextTime)
{
	debug("OutputPseudorange\n");
	// Schedule the next epoch
	NextTime = Gps.GpsTime + 1;

	// Create a frame, reserving space for header
	Frame Rtcm23(3);

	// Do for each valid satellite
	for (int s=0; s<MaxSats; s++) {
		if (!Gps.obs[s].Valid) continue;

		// get the satid
		uint32 satid = SatToSvid(s);
		if (satid > 32) continue;
		if (satid == 32) satid = 0;

		// get the other values
		uint32 quality = 15;
		uint32 multipath = 15;
		uint32 Pr = Gps.obs[s].PR * 50;
		uint32 PrHi = (Pr >> 24) & 0xff;
		uint32 PrLow = Pr & 0xffffff;
		debug("Rtcm23Station::OutputPseudorange: Sat=%d PR=%.3f  pr=0x%08x\n", 
			s, Pr/50.0, Pr);

		// build up the satellite's portion of the record (More to come)
		Rtcm23.AddWord((1<<29)|(satid<<22)|(quality<<18)|(multipath<<14)|(PrHi<<6));
		Rtcm23.AddWord(PrLow<<6);
	}

	// Do the Rtcm23 header
	Header(Rtcm23, Gps.GpsTime, 19);

	// figure out how much time since the zcount
	int zcount = Rtcm23.GetField(2, 1, 13);
	Time nsec = (Gps.GpsTime%NsecPerHour) - zcount*NsecPerSec*6/10;
	assert(S(nsec) < .6 && S(nsec) >= 0);

	// Do the pseudorange header. 
	uint32 GnssTime = nsec / 1000;
	uint32 smoothing = 0;  // 0-1 minute
	Rtcm23.PutWord(3, (smoothing<<27) | (GnssTime<<6));
	debug("OutputPseudorange: GnssTime=%d\n", GnssTime);

	return Out.WriteFrame(Rtcm23);
}


bool Rtcm23Station::OutputCarrierPhase(Time& NextTime)
{
	debug("OutputCarrierPhase\n");
	// Schedule for the next epoch
	NextTime = Gps.GpsTime + 1;

	// Create a CarrierPhase header. 
	Frame Rtcm23(3);

	// Do for each satellite
	for (int s=0; s<MaxSats; s++) {
		if (!Gps.obs[s].Valid) continue;

		// Check for loss of lock or gain of new satellite
		bool Slip = Gps.obs[s].Slip || !PreviouslyValid[s];

		// If slipped, keep track of the cumulative and reset the close to zero
		if (Slip) {
			CumulativeLossOfLock[s]++;
			PhaseAdjust[s] = round(-Gps.obs[s].Phase);
		}

		// get the satid
		uint32 satid = SatToSvid(s);
		if (satid > 32) continue;
		if (satid == 32) satid = 0;

		// get the other values
		uint32 quality = 7;
		uint32 lockloss = CumulativeLossOfLock[s]&0x1f;
		int64 Phase = (Gps.obs[s].Phase + PhaseAdjust[s]) * -256;
		uint32 PhaseHi = (Phase >> 24) & 0xff;
		uint32 PhaseLow = Phase & 0xffffff;
		debug("OutputCarrierPhase:  s=%d  Phase(m)=%.3f lockloss=%d\n", 
			s, Gps.obs[s].Phase*L1WaveLength, lockloss);

		// build up the satellite's portion of the record (no more to come)
		Rtcm23.AddWord((satid<<22)|(quality<<19)|(lockloss<<14)|(PhaseHi<<6));
		Rtcm23.AddWord(PhaseLow<<6);
	}

	// Keep track of which sats were valid
	for (int s=0; s<MaxSats; s++)
		PreviouslyValid[s] = Gps.obs[s].Valid;
	
	// create Rtcm23 header
	Header(Rtcm23, Gps.GpsTime, 18);

	// Figure out how much time since last zcount
	int zcount = Rtcm23.GetField(2, 1, 13);
	Time nsec = (Gps.GpsTime%NsecPerHour) - zcount*NsecPerSec*6/10;
	assert(nsec >= 0 && S(nsec) < .6);

	// Do the pseudorange header. 
	uint32 GnssTime = nsec/1000;
	Rtcm23.PutWord(3,GnssTime<<6);
	debug("OutputPseudorange: Gnsstime=%d\n", GnssTime);

	return Out.WriteFrame(Rtcm23);
}



bool Rtcm23Station::OutputEphemeris(int s, Time& NextTime)
{
	// if not a broadcast ephemeris, don't schedule again
	EphemerisXmit& e = *dynamic_cast<EphemerisXmit*>(&Gps[s]);
	if (&e == 0) {
		NextTime = MaxTime;
		return OK;
	}
	
	// Schedule every two minutes, spread out on odd seconds
	NextTime = Gps.GpsTime - Gps.GpsTime%(2*NsecPerMinute) + 2*NsecPerMinute
		  + s*2*NsecPerSec + NsecPerSec;

	// If the ephemeris isn't valid, then done
	if (!e.Valid(Gps.GpsTime))
		return OK;

	// Get the ephemeris frame
	Frame f;  EphemerisXmitRaw r;
	e.ToRaw(r);   f.FromRaw(r);
        
	Header(f, Gps.GpsTime, 17);

	// Output it
	return Out.WriteFrame(f);
}



Rtcm23Station::~Rtcm23Station(void)
{
}



void Rtcm23Station::Header(Frame& f, Time time, int type)
{
	uint32 zcount = (time % NsecPerHour) * 10 / 6 / NsecPerSec;
	if (type == 64) type = 0;
	debug("Rtcm23Frame  type=%d  zcount=%d seqnr=%d\n", type, zcount,SequenceNr);

	f.PutWord(1, (0x66<<22) | (type<<16) | (StationId<<6));
	f.PutWord(2, (zcount<<17) | (SequenceNr<<14) | (f.NrWords<<9) | (Health<<6));

	SequenceNr = (SequenceNr + 1) & 0x7;
}

