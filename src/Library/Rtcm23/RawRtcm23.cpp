
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


#include "RawRtcm23.h"
#include "EphemerisXmit.h"

RawRtcm23::RawRtcm23(Stream& in)
: In(in)
{
    strcpy(Description, "Rtcm23 3.1");
    ErrCode = In.GetError();
    for (int s=0; s<MaxSats; s++) {
        CarrierLossCount[s] = -1;
        PreviousPhase[s] = 0;
        eph[s] = new EphemerisXmit(s, "Rtcm23 3.1");
    }

    PreviousReceiverTime = -1;
    Week = -1;
    HourOfWeek = -1;
}


bool RawRtcm23::NextEpoch()
{
	// Clear out our observations
	for (int s=0; s<MaxSats; s++)
		obs[s].Valid = HasPhase[s] = false;
	MoreToCome = true;
	EpochTow = -1;

	// repeat until we get measurements without the "more" bits
	while (MoreToCome) {

		// Read a frame
		Frame f; bool slip;
		if (In.ReadFrame(f, slip) != OK)
			return Error();

		// Get the header information
		int type = f.GetField(1, 9, 14);
		if (type == 0) type = 64;
		int StationId = f.GetField(1, 15, 24);
		int zcount = f.GetField(2, 1, 13);
		int seqnr = f.GetField(2, 14, 16);
		int StationHealth = f.GetField(2, 22, 24);
		debug("Rtcm23::NextEpoch - type=%d  zcount=%d  StationId=%d  Health=%d  seqnr=%d\n",
			                     type,    zcount,    StationId,    StationHealth, seqnr);
		
        // Update the sequence number
		slip |= (SequenceNr != seqnr);
		SequenceNr = (seqnr + 1) & 0x7;

		// if our communications slipped, ...
		//   TODO.  What should we do? We may be missing frames.
		//   don't need to slip the phases since the slip counts handle it.
		//   We should make sure the current epoch is consistent.
		//   Perhaps keep epoch time for each code+phase.

		// Process the frame according to its type
		if (type == 14)
			ProcessTimeTag(f);
		else if (type == 24)
			ProcessAntennaRef(f);
		else if (type == 23)
			ProcessAntennaTypeDef(f);
		else if (type == 19)
			ProcessPseudorange(f);
		else if (type == 18)
			ProcessCarrierPhase(f);
		else if (type == 17)
			ProcessEphemeris(f);
	}

	// valid observations have both code and phase. (NOTE: CHANGE!!!)
	for (int s=0; s<MaxSats; s++)
		obs[s].Valid &= HasPhase[s];

	// Normalize the results
	GpsTime = ConvertGpsTime(Week, EpochTow);
	AdjustToHz();

	return OK;
}

bool RawRtcm23::ProcessTimeTag(Frame& f)
{
	Week = f.GetField(3, 1, 10);
	HourOfWeek = f.GetField(3, 11, 18);
	int Leapsec = f.GetField(3, 19, 24);
	debug("ProcessTimeTag: Week=%d HourOfWeek=%d LeapSec=%d\n", 
		                   Week,   HourOfWeek,   LeapSec);
    return OK;
}


bool RawRtcm23::ProcessAntennaRef(Frame& f)
{
	int64 x = ((int64)f.GetSigned(3, 1, 24)<<14) | f.GetField(4, 1, 14);
	int64 y = ((int64)f.GetSigned(4, 17, 24)<<30) | (f.GetField(5, 1, 24)<<6) | f.GetField(6,1,6);
	int64 z = ((int64)f.GetSigned(6, 9, 24)<<22) | f.GetField(7, 1, 22);
	int64 height = 0;
	if (f.GetField(7, 24, 24) != 0)
		height = f.GetField(8, 1, 18);

	Pos = Position(x/10000.0, y/10000.0, z/10000.0);
	AntennaHeight = height/10000.0;
	debug("ProcessAntennaRef: x=%.3f y=%.3f z=%.3f h=%.3f\n",
		Pos.x, Pos.y, Pos.z, AntennaHeight);
	return OK;
}

bool RawRtcm23::ProcessAntennaTypeDef(Frame& f)
{
	return OK;
}

bool RawRtcm23::ProcessCarrierPhase(Frame& f)
{
	// Get the time of measurement
	if (GetMeasurementTime(f) != OK)
		return OK;

	// We are only interested in L1 for now
	if (f.GetField(3, 1, 2) != 0)
		return OK;

	// Decide how many observables
	int n = (f.NrWords-3)/2;

	// Get the data for each observation
	for (int i=0; i<n; i++) {

		// We want C/A GPS measurements only (for now)
		if (f.GetField(4+2*i, 2, 3) != 0) continue;

		// which satellite?
		int svid = f.GetField(4+2*i, 4, 8);
		int Sat = SvidToSat(svid);
		if (Sat == -1) return Error("RawRtcm23: didn't recognize svid=%d\n", svid);
		HasPhase[Sat] = true;

		// Phase
		int64 phase = (f.GetSigned(4+2*i, 17, 24)<<24) | f.GetField(5+2*i, 1, 24);
		obs[Sat].Phase = phase / -256.0;

		obs[Sat].Doppler = 0;

		// TODO: Handle phase rollover.
		//  for now, it will be caught as an outlier

		// Quality
		uint32 DataQual = f.GetField(4+2*i, 9, 11);

		// Slipped?
		int32 LossCount = f.GetField(4+2*i, 12, 16);
		obs[Sat].Slip = (LossCount == CarrierLossCount[Sat]);
		if (obs[Sat].Slip)
			CarrierLossCount[Sat] = LossCount;

		// More?
		MoreToCome = (f.GetField(4+2*i, 1, 1) != 0);
		debug("RawRtcm23: Sat=%d  Phase=%.3f  MoreToComm=%d\n", Sat,obs[Sat].Phase, MoreToCome);
	}

	return OK;
}

bool RawRtcm23::ProcessPseudorange(Frame& f)
{
	// Get the time of measurement
	if (GetMeasurementTime(f) != OK)
		return Error();

	// We are only interested in L1 for now
	if (f.GetField(3, 1, 2) != 00)
		return OK;

	// Decide how many observables
	int n = (f.NrWords-3)/2;

	// Get the data for each observation
	for (int i=0; i<n; i++) {

		// We want C/A GPS measurements only (for now)
		if (f.GetField(4+2*i, 2, 3) != 0) continue;

		// which satellite?
		int svid = f.GetField(4+2*i, 4, 8);
		if (svid == 0) svid = 32;
		int Sat = SvidToSat(svid);
		obs[Sat].Valid = true;

		// Pseudorange
		uint32 pr = (f.GetField(4+2*i, 17, 24)<<24) | f.GetField(5+2*i, 1, 24);
		obs[Sat].PR = pr / 50.0;
		debug("RawRtcm23::ProcessPseudorange  s=%d  PR=%.3f  pr=0x%08x\n",
			                                Sat, obs[Sat].PR, pr);

		// Quality
		int DataQual = f.GetField(4+2*i, 9, 12);
		int Multipath = f.GetField(4+2*i, 13, 16);
		obs[Sat].SNR = DataQual;  // For now. We need to do better.

		// More?
		MoreToCome = (f.GetField(4+2*i, 1, 1) != 0);
		debug("RawRtcm23: Sat=%d  PR=%.3f  MoreToComm=%d\n", Sat,obs[Sat].PR, MoreToCome);
	}

	return OK;
}


bool RawRtcm23::ProcessEphemeris(Frame& f)
{
	// Figure out which satellite
	int svid = f.GetField(31, 17, 21);
	if (svid == 0) svid = 32;
	int s = SvidToSat(svid);

	// Make sure we have an xmitted ephemeris (we do, but check anyway)
	EphemerisXmit& e = *dynamic_cast<EphemerisXmit*>( &(*this)[s] );
	if (&e == NULL) return OK;

	// update the ephemeris with the new frame
        EphemerisXmitRaw r;
        f.ToRaw(r);
	return e.FromRaw(r);
}

bool RawRtcm23::GetMeasurementTime(Frame& f)
{
	if (HourOfWeek < 0 || Week < 0)
		return Error("Haven't received a time tag yet\n");

	// Get the time from the record
	int zcount = f.GetField(2, 1, 13);
	int GNSST = f.GetField(3, 5, 24);
	double ETOM = GNSST/1000000.0 + zcount*.6;
	debug("RawRtcm23::GetMeasurementTime zcount=%d  GNSST=%d  ETOM=%.6f\n",zcount,GNSST,ETOM);
	double ReceiverTime = round(ETOM, .01); // Epochs are at most 100 hz

	// If we moved backwards, then we missed a clock record
	if (ReceiverTime < PreviousReceiverTime) {
		debug("RawRtcm23 - Clock moved backwards prev=%.2f\n", PreviousReceiverTime);
		HourOfWeek++;
		if (HourOfWeek >= 7*24) {
			HourOfWeek = 0;
			Week++;
		}
	}
	PreviousReceiverTime = ReceiverTime;

	// calculate the time of week
	double Tow = HourOfWeek*3600.0 + ReceiverTime;
	if (EpochTow == -1)
		EpochTow = Tow;
	else if (EpochTow != Tow)
		return Error("Measurement time changed within an epoch");

	return OK;
}


RawRtcm23::~RawRtcm23(void)
{
}

