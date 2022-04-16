
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


#include "RawSSF.h"

RawSSF::RawSSF(Stream& s)
: comm(s)
{
	ErrCode = comm.GetError();
	if (ErrCode != OK) return;

	strcpy(Description, "Trimble SSF");

	// Set up the ephemerides
	for (int s=0; s<MaxSats; s++)
		eph[s] = new EphemerisXmit(s, "SSF Broadcast Ephemeris");

      // Clear the clock until we find out the week
      GpsTime = -1;

      // Clear the position until we read the header
      Pos = 0;

      // Start out with no phase adjustments. 
      PhaseAdjust = 0;
      ClockError = 0;
}



bool RawSSF::NextEpoch()
{
	Block b;
      bool Err;

	// Repeat until we have a complete epoch
      Count = -1;
	do {
		// Read a block of data
		if (comm.GetBlock(b) != OK) return Error();

		// Process according to type
            if      (b.Id == 255) Err = ProcessHeader(b);
            else if (b.Id == 21)  Err = ProcessPosition(b);
            else if (b.Id == 16)  Err = ProcessClock(b);
            else if (b.Id == 70)  Err = ProcessStartEpoch(b);
            else if (b.Id == 71)  Err = ProcessRaw(b);
            else if (b.Id == 80)  Err = ProcessRoverStartEpoch(b);
            else if (b.Id == 81)  Err = ProcessRoverRaw(b);
            else                  {b.Display(1, "SSF - Unknown block"), Err = OK;}

	} while (Count != 0 && Err == OK);

	debug("SSF NextEpoch: GpsTime=%.1f\n", S(GpsTime));

	return Err;
}


bool RawSSF::ProcessHeader(Block& blk)
{
    blk.Display(1, "ProcessHeader");
    LittleEndian b(blk);
    float x1 = b.GetFloat();
   
    char info[81];
    for (int i=0; i<80; i++)
        info[i] = b.Get();
    info[80] = 0;
    debug("ProcessHeader: x1=%g  info=%s\n", x1, info);

    float x2 = b.GetFloat();
    double tow = b.GetDouble();
    uint16 wn = b.Get2();
    uint32 x3 = b.Get4();
    float x4 = b.GetFloat();
    debug("ProcessHeader: x2=%g tow=%.6f wn=%d  x3=%d  x4=%g\n", x2, tow, wn, x3, x4);

    double lat = b.GetDouble();
    double lon = b.GetDouble();
    double alt = b.GetDouble();
    debug("ProcessHeader: lat=%.6f lon=%.6f alt=%.3f\n", 
                      RadToDeg(lat), RadToDeg(lon), alt);

    // Get the position.
    Pos = Wgs84ToPosition( lla(RadToDeg(lat), RadToDeg(lon), alt) );
    debug("ProcessHeader: Pos=(%.3f, %.3f, %.3f)\n", Pos.x, Pos.y, Pos.z);

    // Save the time, esp week number
    GpsTime = ConvertGpsTime(wn, tow);

    return OK;
}

bool RawSSF::ProcessPosition(Block&  blk)
{
    LittleEndian b(blk);
    double lat = b.GetDouble();
    double lon = b.GetDouble();
    double alt = b.GetDouble();
    double x1 = b.GetDouble();
    uint16 x2 = b.Get2();
    debug("ProcessPosition: lat=%.6f lon=%.6f alt=%.3f x1=%g  x2=%d\n", 
                      RadToDeg(lat), RadToDeg(lon), alt, x1, x2);

    // Get the position.
    Pos = Wgs84ToPosition( lla(RadToDeg(lat), RadToDeg(lon), alt) );
    debug("ProcessHeader: Pos=(%.3f, %.3f, %.3f)\n", Pos.x, Pos.y, Pos.z);

    return OK;
}

bool RawSSF::ProcessStartEpoch(Block& blk)
{
    // Unpack the fields
    LittleEndian b(blk);
    double tow = b.GetDouble();
    double err = b.GetDouble();
    uint8 nrsats  = b.Get();
    byte x1 = b.Get();
    debug("RawSSF::ProcessClock - tow=%.6f  err=%.6f  nrsats=%d x1=%d\n", tow, err, nrsats, x1);

    // if we don't know the week number, don't start collecting data yet
    debug("    GpsTime=%lld\n", GpsTime);
    if (GpsTime == -1) return OK;

    // We are starting a new epoch
    for (int s=0; s<MaxSats; s++)
        obs[s].Valid = false;
    
    // Update the time
    GpsTime = UpdateGpsTime(GpsTime, tow/1000);

    // Make note of how many satellites we are about to process.
    Count = nrsats;

    // If the clock has rolled over, add a msec to phase measurements.
    //   Don't know if this is a receiver bug, but it makes things add up
    if (fabs(ClockError - err) > .5)
        PhaseAdjust += L1Freq/1000;  // don't need array if same across all sats
    ClockError = err;

    return OK;
}

bool RawSSF::ProcessClock(Block& blk)
{

    LittleEndian b(blk);
    float tow = b.GetFloat();
    uint16 wn = b.Get2();
    uint16 x1 = b.Get2();
    uint16 x2 = b.Get2();
    debug("ProcessWeek: tow=%.6f wn=%d x1=%d x2=%d\n", tow, wn, x1, x2);

    // Wait until we know our position before starting. (Must read a header record)
    if (Pos == 0) return OK;

    // Get the time
    GpsTime = ConvertGpsTime(wn, tow);
    return OK;
}


bool RawSSF::ProcessRaw(Block& blk)
{
    blk.Display(1, "ProcessRaw");
    // Unpack the fields
    LittleEndian b(blk);
    byte svid = b.Get();
    byte status = b.Get();
    uint16 x1 = b.Get2();
    uint16 x2 = b.Get2();
    uint16 x3 = b.Get2();
    uint16 x4 = b.Get2();
    debug("ProcessRaw: svid=%d  status=0x%02x x1=%d x2=%d x3=%d x4=%d\n", 
                  svid, status, x1, x2, x3, x4);
    double pr = b.GetDouble();
    double phase = b.GetDouble();
    float  doppler = b.GetFloat();
    debug("ProcessRaw: pr=%.3f phase=%.3f  doppler=%.3f  Count=%d\n", pr, phase, doppler, Count);

    // If we haven't started an epoch, then we can't process records
    if (Count <= 0) return OK;

    int Sat = SvidToSat(svid);
    if (Sat < 0) return Error("RawSSF::ProcessRaw - Invalid svid (%d)\n", svid);

    // Copy the values into the data for the current epoch
    obs[Sat].PR = pr;
    obs[Sat].SNR = x2;
    if ( (status&0xd0) == 0xd0)  obs[Sat].Doppler = -doppler/L1WaveLength;
    else                         obs[Sat].Doppler = 0;
    obs[Sat].Phase = -phase - PhaseAdjust;
    obs[Sat].Valid = true;

    // We have processed a measurement. Decrement the remaining count.
    Count--;

    debug("ProcessRaw: svid=%d  PR=%.3f phase=%.3f  doppler=%.3f\n", 
             svid, pr, phase, doppler);
    return OK;
}


bool RawSSF::ProcessRoverStartEpoch(Block& blk)
{
    blk.Display(1,"ProcessRoverStartEpoch");
    LittleEndian b(blk);
    int16 x1 = b.Get2();
    byte x2 = b.Get();
    int16 nrsvs = b.Get2();
    byte hrs = b.Get();
    float secs = b.GetFloat();

    // Get the time of week
    double TOW = hrs*3600 + secs;
    debug("ProcessRoverStartEpoch: TOW=%.3f  nrsvs=%d hrs=%d secs=%.3f x1=%d x2=%d \n",
                                     TOW, nrsvs, hrs, secs, x1, x2);
    
    // If we don't know the week number, then done
    if (GpsTime <= 0) return OK;

    // Update the time and make note of how many satellites are about to follow
    GpsTime = UpdateGpsTime(GpsTime, TOW);
    Count = nrsvs;

    return OK;
}

bool RawSSF::ProcessRoverRaw(Block& blk)
{
    blk.Display(1,"ProcessRoverRaw");
    LittleEndian b(blk);
    int16 svid = b.Get2();
    uint16 x1 = b.Get2();
    uint16 x2 = b.Get2();
    uint16 x3 = b.Get2();
    float time = b.GetFloat();
    float pr = b.GetFloat();
    uint32 x4 = b.Get4();

    debug("ProcessRoverRaw: svid=%d  time=%.6f pr=%.6f  x1=%d x2=%d x3=%d x4=%d\n",
                            svid,time, pr,x1,x2,x3,44);

    
    int Sat = SvidToSat(svid);
    if (Sat < 0) return Error("ProcessRoverRaw - bad svid (%s)\n", svid);
    
    // If we aren't expecting satellite data, then done
    if (Count <= 0) return OK;

    // Save the data
    obs[Sat].PR = 20000000.0 - pr * (C / 1000);
    obs[Sat].Phase = 0;
    obs[Sat].SNR = 0;
    obs[Sat].Doppler = 0;
    obs[Sat].Valid = true;
  
    // Make note we have a new satellite
    Count--;
    
    debug("ProcessRoverRaw: Sat=%d  PR=%.3f\n", Sat, obs[Sat].PR);

    return OK;
}

RawSSF::~RawSSF()
{
}




