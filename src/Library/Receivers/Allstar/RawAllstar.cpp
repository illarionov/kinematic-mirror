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


#include "RawAllstar.h"
#include "NavFrame.h"


// Message types
static const int CONTINUOUS = 128;
static const int MEASUREMENTS = 23;   // Raw measurements
static const int EPHEMERIS = 22;   // ephemeris
static const int NAVIGATION = 21;   // Navigation solution

static const int ACKACK = 126;   
static const int INITIATE = 63;  
static const int SOFTWAREID = 45;   



RawAllstar::RawAllstar(Stream& s)
   :comm(s)
{
	strcpy(Description, "Allstar");
	ErrCode = comm.GetError();
	if (ErrCode == OK)
       ErrCode = Initialize();
}

RawAllstar::~RawAllstar()
{
}


bool RawAllstar::NextEpoch()
{
	NavTime = -1;
	RawTime = -2;

	// Epoch ends when we have matching raw and solution records 
	while (RawTime != NavTime) {

		// Read a message from the GPS
		Block b;
		if (comm.GetBlock(b) != OK) return Error();

		// Process according to the type of message
		if      (b.Id == NAVIGATION)     ProcessSolution(b);
		else if (b.Id == EPHEMERIS)      ProcessEphemeris(b);
		else if (b.Id == MEASUREMENTS)   ProcessRawMeasurement(b);
		else                        debug("Got Message id 0x%04x\n", b.Id);
	}

	debug("RawAllstar::NextEpoch (done) GpsTime=%.3f\n", S(GpsTime));
	return OK;
}


bool RawAllstar::ProcessSolution(Block& block)
{
	LittleEndian b(block);
	double Tow = b.GetDouble();
	uint16 Week = b.Get2();
	double XPos = b.GetDouble();
	double YPos = b.GetDouble();
	double ZPos = b.GetDouble();
	float XVel = b.GetFloat();
	float YVel = b.GetFloat();
	float ZVel = b.GetFloat();
	float ClockBias = b.GetFloat();
	double ClockDrift = b.GetDouble();
	float HFOM = b.GetFloat();
	float VFOM = b.GetFloat();
	byte NavMode = b.Get();
	byte NrSv = b.Get();
	


    // Make sure this is a valid measurement
	//  (Must be 3D fix with week and TOW known)
    if ( (NrSv&15) < 4)
		return OK;

	// Get the position and time information
	Pos = Position(XPos,YPos,ZPos);
	GpsTime = ConvertGpsTime(Week, Tow);

	debug("      week=%d Tow=%g x=%.3f  y=%.3f  z=%.3f Pacc=%g\n", 
		Week, Tow, Pos.x, Pos.y, Pos.z, HFOM);

	return OK;
}



bool RawAllstar::ProcessEphemeris(Block& block)
{

	// Get satellite number and check the HOW for validity
	LittleEndian b(block);
	byte SVID = b.Get();

	// Point to the satellite and its ephemeris
	int Sat = SvidToSat(SVID&31);
	debug("ProcessEphemeris: Sat=%d  length=%d\n", Sat, block.Length);  

	// We're going to copy the data into one of our NavFrames
	NavFrame f; int fsize=0;

	// Do for each word in the ephemeris message
	for (int i=0; i<24; i++) {

		// If start of frame, reserve room for TLM and HOW words
		if ((i%8) == 0)
			fsize += 2;

		// Copy the data to the Nav frame, (Verify this is the order)
		f.Data[fsize++] = (b.Get()<<16) | (b.Get()<<8) | b.Get();
	}

	// Update the ephemeris
	EphemerisXmit& e = *(EphemerisXmit*)eph[Sat];
        EphemerisXmitRaw r;
	f.ToRaw(r);
        e.FromRaw(r);

	return OK;
}

bool RawAllstar::ProcessRawMeasurement(Block& block)
{
	// Assume nothing is valid to start
	for (int s=0; s<MaxSats; s++)
		obs[s].Valid = false;

	// Parse the header
	LittleEndian b(block);
	uint16 reserved = b.Get2();
	byte NSV = b.Get();
	double Tow = b.GetDouble();

	debug("RawAllstar::ProcessRawmeasurement Tow=%.6f, NSV=%d\n",Tow,NSV);

	// Do for each satellite being tracked
	for (int i=0; i<NSV; i++) {

		// Get the measurements
		byte SV = b.Get();
		byte SNR = b.Get();
		uint32 CodePhase = b.Get4();
		uint32 CarrierPhase = b.Get4();
		byte SlipCount = b.Get();

		// Get the satellite		
		int Sat = SvidToSat( (SV&0x1f));

		// Calculate pseudorange from code phase
		double PR = (Tow-floor(Tow)) - CodePhase/(102300*2048.0);
		if (PR < 0) PR += 1;
		PR = PR * C;

		// Calculate Phase, could wrap around.
		double RawPhase = (CarrierPhase>>2) / 1024.0;
		double DeltaPhase = RawPhase - OldRawPhase[Sat];
		if (abs(DeltaPhase) > 512*1024)
			if ((RawPhase - OldRawPhase[Sat]) > 0)  Adjust[Sat] += 1024*1024;
			else                                    Adjust[Sat] -= 1024*1024;

		// Save the information
		obs[Sat].PR = (Tow-floor(Tow)) - CodePhase/(102300*2048);
		if ( (CarrierPhase&2) != 0) obs[Sat].Phase = 0;
		else                        obs[Sat].Phase = (CarrierPhase&~3) / 4096.0;
		obs[Sat].Slip = SlipCount != OldSlipCount[Sat] || (CarrierPhase&3) != 0;
		obs[Sat].SNR = SNR / 6.0;
		obs[Sat].Valid = true;
		obs[Sat].Doppler = 0;
		debug("      Sat=%d  PR=%.3f  Phase=%.3f  Slip=%d snr=%.1f \n",
			Sat, obs[Sat].PR, obs[Sat].Phase, obs[Sat].Slip, obs[Sat].SNR);

		OldSlipCount[Sat] = SlipCount;
		OldRawPhase[Sat] = RawPhase;
	}

	RawTime = UpdateGpsTime(GpsTime, Tow);

	return OK;
}




bool RawAllstar::Initialize()
{
	strcpy(Description, "Allstar");

	// Set up the ephemerides
	for (int s=0; s<MaxSats; s++)
		eph[s] = new EphemerisXmit(s, "Allstar");

	for (int s=0; s<MaxSats; s++)
		OldSlipCount[s] = -1;

	if (comm.ReadOnly()) return OK;

	// Reset the link
 	if (comm.PutBlock(INITIATE, 'U','G','P','S','-','0','0','0', -1) != OK) return Error();

	// Request software id
	if (comm.PutBlock(SOFTWAREID, -1) != 0) return Error();

	// Configure to send the desired messages
	if (comm.PutBlock(CONTINUOUS|NAVIGATION, -1) != OK) return Error();
	if (comm.PutBlock(CONTINUOUS|EPHEMERIS, -1) != OK) return Error();
	if (comm.PutBlock(CONTINUOUS|MEASUREMENTS, 0, -1) != OK) return Error();

	return OK;
}

