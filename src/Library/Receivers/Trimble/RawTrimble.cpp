
// RawTrimble reads raw data from a Trimble Lassen IQ gps receiver
//    Part of Kinematic, a collection of utilities for GPS positioning
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

#include "RawTrimble.h"


RawTrimble::RawTrimble(Stream& s)
: comm(s)
{
	ErrCode = Initialize();
}

RawTrimble::~RawTrimble(void)
{
}


bool RawTrimble::Initialize()
{
	strcpy(Description, "LassenIQ");
	if (comm.GetError() != OK) return Error();

	// We need a valid Gps Week, which we get from the clock record
	ClockTime = -1;
	MeasurementTime = -2;

	// Do for each satellite
	for (int32 s=0; s<MaxSats; s++) {

		// Create a transmitted ephemeris
		eph[s] = new EphemerisXmit(s, "Trimble Lassen IQ");

	    // In the beginning, the integrated doppler is zero.
		//  (zero means "invalid phase", so make it very close to zero)
		IntegratedDoppler[s] = .0001;
	}

	if (comm.GetError() != OK) return Error();
	if (comm.ReadOnly()) return OK;

	// Enable the raw measurements
	if (comm.PutBlock(0x35, 0x11, 0, 0, 0x01, -1))
		return Error();

	// Wait for the beginning of a new epoch
	//if (comm.AwaitBlock(DgpsModeId)) 
	if (comm.AwaitBlock(0x6d)) 
		return Error("Trimble unit not talking to us.\n");

	// Start collecting satellite ephemerides as they become available
	InitEphemerides();
	
	debug("Trimble init: Pos = (%.3f,  %.3f,  %.3f)\n",Pos.x,Pos.y,Pos.z);
	return OK;
}

bool RawTrimble::NextEpoch()
{

	MeasurementTime = -2;
 
    // repeat until we have consistent measurement and position data
	while (MeasurementTime != GpsTime) { 

		// Read the next data block from the receiver
		Block b;
		if (comm.GetBlock(b)) return Error();
       
		// Process the block
		if (b.Id == PositionId)       ProcessPosition(b);
		else if (b.Id == GpsTimeId)   ProcessGpsTime(b);
		else if (b.Id == RawDataId)   ProcessRawData(b);
		else if (b.Id == SatDataId)   ProcessSatData(b);
		else                          debug("Trimble Record: Id=%d  Len=%d\n",
			                                 b.Id, b.Length);
	}

	// Request ephemeris data for one needy satellite
	UpdateEphemerides();

	// Reset the integrated doppler for invalid satellites
	for (int s=0; s<MaxSats; s++)
		if (!obs[s].Valid) IntegratedDoppler[s] = .0001;

	return OK;
}


bool RawTrimble::ProcessPosition(Block& blk)   
{
    // If we haven't received a valid Clock record yet, then done.
	if (ClockTime == -1)
		return OK;

	// Unpack the record
	BigEndian b(blk);
	double X = b.GetDouble();
	double Y = b.GetDouble();
	double Z = b.GetDouble();
	ClockBias = b.GetDouble() / C;  // Convert to seconds.
	double Tow = b.GetFloat();

	debug("ProcessPosition (%.3f,%.3f,%.3f) Tow=%.6f\n",
		X,Y,Z,Tow);

	// Save our estimated position
	Pos = Position(X,Y,Z);

	// Update the GPS time
	GpsTime = UpdateGpsTime(ClockTime, Tow);

	if (GpsTime%NsecPerSec != 0) debug("ERROR: Position not aligned to Epoch");

    return OK;
}

bool RawTrimble::ProcessGpsTime(Block& blk)
{
	BigEndian b(blk);
	double TOW = b.GetFloat();
	int Week = b.Get2();
	int UtcOffset = b.GetFloat();

	ClockTime = ConvertGpsTime(Week, TOW);

    debug("ProcessGpsTime: Week=%d TOW=%.9f UtcOff=%d ClockTime=%lld\n", 
		Week, TOW, UtcOffset, ClockTime);
    return OK;
}

bool RawTrimble::ProcessRawData(Block& blk)
{
	// If we don't know the Gps Week yet, skip the measurement
	if (ClockTime == -1) return OK;

	// Unpack the record
	BigEndian b(blk);
	byte svid = b.Get();
	float SampleLength = b.GetFloat();
	float SignalLevel = b.GetFloat();
	float CaPhase = b.GetFloat() / (16*1023); 
	float Doppler = b.GetFloat();
	double Tow = b.GetDouble();

	// Get our current time
	Time CurrentTime = UpdateGpsTime(ClockTime, Tow);
	int Sat = SvidToSat(svid);

	debug("ProcessRawData s=%d Time=%lld\n",Sat,CurrentTime);

	// If we are in a different epoch, then start with clean observations
	if (CurrentTime != MeasurementTime)
		for (int s = 0; s<MaxSats; s++)
			obs[s].Valid = false;
	MeasurementTime = CurrentTime;
	if (MeasurementTime % NsecPerSec != 0) debug("measurement time not on epoch\n");

	// Request an ephemeris if we don't have one already.
	if (!eph[Sat]->Valid(CurrentTime)) 
		return RequestEphemeris(Sat);

	// Lookup the satellite's position
	Position SatPos; double Adjust;
	if (eph[Sat]->SatPos(MeasurementTime, SatPos, Adjust) != OK)
		return Error(); 

	// Estimate how many "chips" are between satellite and receiver
	//   (We only need to 1 msec, so we can ignore atmosphere?)
	double Ca = Range(SatPos - Pos) / C * 1000;

	// Calculate the pseudorange based on the chip estimate and chip phase
	double CaInt = round(Ca-CaPhase, 1);
	double PR = (CaInt + CaPhase) * (C / 1000);

	// For approximate phase information, we need to integrate the doppler shift
	IntegratedDoppler[Sat] += Doppler;
	double Phase = -IntegratedDoppler[Sat];

	// TODO: On pseudo range calculation, check for clock adjustments
	//   which might mean adding or subtracting a msec.
	//   (maybe clock bias should be part of it?)
	obs[Sat].PR = PR;
	obs[Sat].Valid = true;
	obs[Sat].Phase = -IntegratedDoppler[Sat];  // set to zero instead?
	obs[Sat].Doppler = Doppler;
	obs[Sat].Slip = false;
	obs[Sat].SNR = SignalLevel * 6;

	debug("  Sat=%d  PR=%.3f  CaInt=%.0f  CaPhase=%.6f Phase=%.3f  SNR=%g\n",
		     Sat, obs[Sat].PR, CaInt, CaPhase, obs[Sat].Phase, obs[Sat].SNR);

	return OK;
}



bool RawTrimble::InitEphemerides()
{
	for (int s=0; s<MaxSats; s++)
		UpdateEphemerisAfter[s] = 0x7fffffffffffffffll;
	LastEphemeris = 0;
	return OK;
}

bool RawTrimble::RequestEphemeris(int sat)
{
	debug("RequestEphemeris: sat=%d\n", sat);
	UpdateEphemerisAfter[sat] = 0;
	return OK;
}

bool RawTrimble::GotEphemeris(int sat)
{
	debug("GotEphemeris: sat=%d\n", sat);
	UpdateEphemerisAfter[sat] = GpsTime + 30*NsecPerMinute;
	return OK;
}

bool RawTrimble::UpdateEphemerides()
{
	// find the next satellite which needs updating
	for (int s=0; s<MaxSats; s++) {
		LastEphemeris = (LastEphemeris+1)%MaxSats;
		if (UpdateEphemerisAfter[LastEphemeris]>GpsTime) continue;

		// request the ephemeris
		debug("UpdateEphemeris: Requesting %d\n", LastEphemeris);
	    if (comm.PutBlock( 0x38, 1, 6, SatToSvid(LastEphemeris), -1) != OK) return Error();
        break;
	}

	return OK;
}


bool RawTrimble::ProcessSatData(Block& blk)
{
	BigEndian b(blk);
	byte Operation = b.Get();
	byte TypeOfData = b.Get();
	byte PRN = b.Get();
	byte Length = b.Get();

	if (Operation != 2 || TypeOfData != 6 || PRN == 0)
		return Error("Unexpected Satellite Data\n");

	// Update the transmitted ephemeris for the given satellite
	int32 Svid = b.Get();
	int32 s = SvidToSat(Svid);  // gps satellite
	if (s == -1)
		return Error("Trimble Ephemeris has wrong svid");
	EphemerisXmit& e = *dynamic_cast<EphemerisXmit*>(eph[s]);
	float t_ephem = b.GetFloat();
	int16 weeknum = b.Get2();

	byte codeL2 = b.Get();
	byte L2pdata = b.Get();
	byte SVacc_raw = b.Get();
	e.health = b.Get();
	e.iodc = b.Get2();

	e.t_gd = b.GetFloat();
	float tow_oc = b.GetFloat();
	e.t_oc = ConvertGpsTime(weeknum, tow_oc);
	e.a_f2 = b.GetFloat();
	e.a_f1 = b.GetFloat();
	e.a_f0 = b.GetFloat();
	e.acc = b.GetFloat();
	e.iode = b.Get();
	byte fitinterval = b.Get();
	e.c_rs = b.GetFloat();
	e.delta_n = b.GetFloat();
	e.m_0 = b.GetDouble();
	e.c_uc = b.GetFloat();
	e.e = b.GetDouble();
	e.c_us = b.GetFloat();
	e.sqrt_a = b.GetDouble();
	double tow_oe = b.GetFloat();
	e.t_oe = ConvertGpsTime(weeknum, tow_oe);
	e.c_ic = b.GetFloat();
	e.omega_0 = b.GetDouble();
	e.c_is = b.GetFloat();
	e.i_0 = b.GetDouble();
	e.c_rc = b.GetFloat();
	e.omega = b.GetDouble();
	e.omegadot = b.GetFloat();
	e.idot = b.GetFloat();
	double Axis = b.GetDouble();
	double n = b.GetDouble();
	double r1me2 = b.GetDouble();
	double omega_n = b.GetDouble();
	double odot_n = b.GetDouble();

	// LATER. For now, assume valid for two hours
	e.MinTime = e.t_oe - 2*NsecPerHour;
	e.MaxTime = e.t_oe + 2*NsecPerHour;
	e.Display();

	// Check for validity. Trimble may not have data yet.
	if (e.sqrt_a < 1) {
		e.MinTime = -1;
		e.MaxTime = -2;
		return OK;
	}

	// Schedule a later update
	GotEphemeris(s);

	debug("ProcessSatData: op=%d type=%d s=%d\n", Operation,TypeOfData,s);
	debug("  t_oe = %.3f \n", tow_oe);

	return OK;
}

