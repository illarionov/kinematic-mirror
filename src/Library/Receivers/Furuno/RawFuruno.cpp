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


#include "RawFuruno.h"



RawFuruno::RawFuruno(Stream& s)
   :comm(s)
{
	strcpy(Description, "Furuno");
	ErrCode = comm.GetError();
	if (ErrCode == OK)
       ErrCode = Initialize();
}

RawFuruno::~RawFuruno()
{
}

/////////////////////////////////////////////////////////////////////////////
//
// About the Furuno. 
//
// We expect two types of records from the Furuno receiver
//   o PositionFix  - contains calculated position, velocity and raw measurements
//   o Nav - contains navigation frame for one satellite
//
///////////////////////////////////////////////////////////////////////////////

bool RawFuruno::NextEpoch()
{
	GpsTime = -1;  

	// Epoch ends when raw data processing sets GpsTime
	while (GpsTime < 0) {

		// Read a message from the GPS
		Block b;
		if (comm.GetBlock(b) != OK) return Error();

		// Process according to the type of message
		if      (b.Id == 0x50)   ProcessPosition(b);
		else if (b.Id == 0x52)   ProcessNav(b);
		else                     return Error("Furuno got Bad Message. Id=0x%04x %3c\n", b.Id);
	}

	// Round to the nearest epoch and adjust the measurements (updating by doppler)
      AdjustToHz();

	debug("RawFuruno::NextEpoch (done) GpsTime=%.3f\n", S(GpsTime));
	return OK;
}


bool RawFuruno::ProcessPosition(Block& block)
///////////////////////////////////////////////////////////////////
// ProcessSolution processes a Position record
//    If valid fix, will set GpsTime
///////////////////////////////////////////////////////////////////
{
    // Clear out the raw measurements
    for (int s=0; s<MaxSats; s++)
        obs[s].Valid = false;
	
	// Read in the header
	BigEndian b(block);
    uint32 Secs = b.Get4();   // seconds time of week
    uint16 Msecs = b.Get2();  //  milliseconds
	int8  NrSats = b.Get();   // number sats used in position
	int8 Status = b.Get();    // positioning status
	debug(2, "RawFuruno::ProcessPosition  Secs=%d Msecs=%d NrSats=%d Status=%d\n",Secs,Msecs,NrSats,Status);
	
	int32 ECEF_X = b.Get4();  // ECEF X coordinate (cm)
	int32 ECEF_Y = b.Get4();  // ECEF Y coordinate (cm)
	int32 ECEF_Z = b.Get4();  // ECEF Z coordinate (cm)
	debug(2, "RawFuruno::ProcessPosition  X=%d Y=%d Z=%d\n", ECEF_X, ECEF_Y, ECEF_Z);

	int32 ECEFV_X = b.Get4();  // ECEF X Velocity (cm/s)
	int32 ECEFV_Y = b.Get4();  // ECEF Y Velocity (cm/s)
	int32 ECEFV_Z = b.Get4();  // ECEF Z Velocity (cm/s)
	int8 HDOP = b.Get();
	int8 VDOP = b.Get();
	uint32 Offset = b.Get4();
	debug(2, "RawFuruno::ProcessPosition  Offset=%d\n", Offset);
	
	static const int SecsPerWeek = 7*24*60*60;
	int Week = Secs / SecsPerWeek;
	double Tow = (Secs % SecsPerWeek) + Msecs/1000.0;
	
	// If we aren't getting 3D fixes, then done
	if (Status!=3 && Status!=5 && Status!=7) goto done;
	
	// Get the time and position
	GpsTime = ConvertGpsTime(Week, Tow);
	Pos = Position(ECEF_X/32.0, ECEF_Y/32.0, ECEF_Z/32.0);
       
    // Do for each channel
    for (int i=0; i<14; i++) {
    	
    	// Get the channel data
    	int8 Svid = b.Get();
    	int8 Mode = b.Get();
    	int8 SNR = b.Get();
    	int8 SlipCnt = b.Get();
    	int32 ADR = b.Get4();
    	int32 Doppler = b.Get4();
    	int32 PR = b.Get4();
    	debug("Furuno: Svid=%d Mode=%d SNR=%d ADR=%d Doppler=%d  PR=%d  SlipCnt=%d\n",
    	               Svid,   Mode,   SNR,   ADR,     Doppler,      PR,      SlipCnt);
    	
    	// if not tracking, done with this channel
    	if (Mode != 8) continue;
    	
    	// Get the satellite number
        int s = SvidToSat(Svid);
    	if (s == -1) return Error("Furuno - Bad satellite number\n");
    		
    	// Get the raw measurements
    	obs[s].Valid = true;
    	obs[s].PR = PR / 32.0;
    	obs[s].Doppler = Doppler / 32768.0;
    	obs[s].Phase = ADR / 256.0;
    	obs[s].SNR = SNR;
    	
    	// if slip count changed, then slipped.
    	obs[s].Slip = (SlipCnt != PrevSlip[s]);
    	PrevSlip[s] = SlipCnt;
    }
    
 done:   
    // reset the slip count for satellites we aren't tracking
    for (int s=0; s<MaxSats; s++)
       if (!obs[s].Valid || obs[s].Phase == 0)
           PrevSlip[s] = -1;
       
	return OK;
}



bool RawFuruno::ProcessNav(Block& block)
{
#ifdef NOTYET
	// Get satellite number and check the HOW for validity
	LittleEndian b(block);
	int32 SVID = b.Get4();
	uint32 HOW = b.Get4();

	// Point to the satellite and its ephemeris
	int Sat = SvidToSat(SVID);
	debug("Furuno ProcessEphemeris: Sat=%d  HOW=0x%x  length=%d\n", Sat, HOW, block.Length); 
	
	// Check for empty ephemeris
	if (HOW == 0 || block.Length <= 8) return OK;

	// We're going to copy the data into one of our NavFrames
	NavFrame f; int fsize=0;

	// Do for each word in the ephemeris message
	for (int i=0; i<24; i++) {

		// If start of frame, reserve room for TLM and HOW words
		if ((i%8) == 0)
			fsize += 2;

		// Copy the word to the Nav frame, restoring to original bit position
		f.Data[fsize++] = b.Get4()&0xffffff;
		//debug("   Data[%d]=%06x\n", fsize-1, f.Data[fsize-1]>>6);
	}

	// Update the ephemeris
	EphemerisXmit& e = *(EphemerisXmit*)eph[Sat];
	e.AddFrame(f);

#endif
	return OK;
}




bool RawFuruno::Initialize()
{
	strcpy(Description, "Furuno");

	// Set up the ephemerides
	for (int s=0; s<MaxSats; s++)
		eph[s] = new EphemerisXmit(s, "Furuno");
		
	// Clear the slip counts
	for (int s=0; s<MaxSats; s++)
	    PrevSlip[s] = -1;	

	return OK;
}






