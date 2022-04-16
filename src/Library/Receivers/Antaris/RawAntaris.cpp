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


#include "RawAntaris.h"
#include "NavFrame.h"

// Message types
static const int RXM_RAW = 0x0210;   // Raw measurements
static const int RXM_EPH = 0x0231;   // ephemeris
static const int NAV_SOL = 0x0106;   // Navigation solution
static const int NAV_CLOCK = 0x0122; // Clock drift

static const int CFG_MSG = 0x0601;   // Set message rate
static const int CFG_RATE = 0x0608;  // Set measurement rate (msec)
static const int CFG_RST = 0x0604;   // Reset

static const int ACK_ACK = 0x0501;   // Message acknowledged
static const int ACK_NAK = 0x0500;   // Message not acknowledged

static const int NMEA = 0xf000;      // Generic NMEA message - add type 0-10
static const int PUBX = 0xf100;      // Generic Proprietary NMEA message - types 0-4



RawAntaris::RawAntaris(Stream& s)
   :comm(s)
{
    strcpy(Description, "Antaris");
    ErrCode = comm.GetError();
    if (ErrCode == OK)
        ErrCode = Initialize();
}

RawAntaris::~RawAntaris()
{
}

/////////////////////////////////////////////////////////////////////////////
//
// About the Antaris. This code was written for the LEA-4T, but it should
//   work on the LEA-6T as well as the original Antaris.
//
// We expect three types of records from the Antaris receiver
//   o NAV_SOL  - contains calculated position, velocity, and clock bias
//   o RAW_RAW  - contains raw pseudorange and phase measurements
//   o RXM_EPH  - contains ephemerides
//
// Each record has a time field. The NAV_SOL time is accurate to the 
//   nearest nanosecond, and the RAW_RAW time is approximate.  We assume both
//   times are within 50 milliseconds of the transmit time or "epoch".
//
// Both calculated and raw records occur at regular intervals, but the intervals
//   may be different. It is quite likely raw records will arrive without a
//   matching calculated position.  In that case, the previous position and 
//   clock bias are extrapolated forward to match the raw data.
//   The Antaris LEA-4T allows up to 10hz raw data and 5hz calculated data.
//
// When aligning measurements to the epoch boundary, it is necessary
//   to adjust pseudoranges and phase by doppler*clock_error.
//
// At 10hz raw measurements, the LEA-4T starts dropping messages.
//   This could be a limitation of the receiver's processing capabilities, or
//   perhaps the host computer isn't reading data fast enough. To reduce workload 
//   and minimize bandwidth, we configure the Antaris to:
//      o turn off NMEA output
//      o calculate positions at most once per second.
// 
// At 10hz, the calculated position lags an epoch or two behind the corresponding
//   raw data. To be more "realtime", we don't wait for the corresponding position,
//   but go ahead and interpolate based on the two previous two calculations.
//   The positions are estimates only, so no harm there. The clock drift doesn't
//   change much from one second to the next, so again no harm done.
///////////////////////////////////////////////////////////////////////////////

/// fetches the next set of raw measurements from the Ublox receiver
bool RawAntaris::NextEpoch()
{

	// Epoch ends when raw data processing sets GpsTime
	for (GpsTime = -1; GpsTime < 0; ) {

		// Read a message from the GPS
		Block b;
		if (comm.GetBlock(b) != OK) return Error();

		// Process according to the type of message
		if      (b.Id == NAV_SOL)   ProcessSolution(b);
		else if (b.Id == RXM_RAW)   ProcessRawMeasurement(b);
		else if (b.Id == RXM_EPH)   ProcessEphemeris(b);
		else                        debug("Antaris Got Message id 0x%04x %3c\n", b.Id);
	}

	// Round to the nearest epoch and adjust the measurements (updating by doppler)
	AdjustToHz();

	debug("RawAntaris::NextEpoch (done) GpsTime=%.3f\n", S(GpsTime));
	return OK;
}


bool RawAntaris::ProcessSolution(Block& block)
///////////////////////////////////////////////////////////////////
/// ProcessSolution processes a solutions message.
//   When things are good, NavEpoch will have a positive value
///////////////////////////////////////////////////////////////////
{
    // Parse the NAVSOL message
    LittleEndian b(block);
    uint32 ITOW = b.Get4();   // GPS Mllisecond Time of Week
    int32  Frac = b.Get4();   // Nanoseconds remainder of rounded ms
    int16 week = b.Get2();    // GPS week
    uint8 GPSfix = b.Get();   // GPS fix Type, (3="3D-Fix")
    uint8 Flags = b.Get();    // Flags, want 0xD set.

    int32 ECEF_X = b.Get4();  // ECEF X coordinate (cm)
    int32 ECEF_Y = b.Get4();  // ECEF Y coordinate (cm)
    int32 ECEF_Z = b.Get4();  // ECEF Z coordinate (cm)
    uint32 Pacc = b.Get4();   // ED Position Accuracy Estimate (cm)

    int32 ECEFVX = b.Get4();  // ECEF X Velocity (cm/s)
    int32 ECEFVY = b.Get4();  // ECEF Y Velocity (cm/s)
    int32 ECEFVZ = b.Get4();  // ECEF Z Velocity (cm/s)
    uint32 SAcc  = b.Get4();  // Speed Accuracy Estimate (cm/s)
    
    uint16 pdop =  b.Get2();  // Position DOP
    uint8 res1 = b.Get();     // reserved
    uint8 numSV = b.Get();    // Number of SVs used in Nav Solution
    int32 res2 = b.Get4();   // reserved

    debug("Antaris::ProcessSolution - fix=%d Flags=0x%x ITOW=%d  nsec=%d\n", 
		GPSfix, Flags, ITOW, Frac);

    // Make sure this is a valid measurement
    //  (Must be 3D fix with week and TOW known)
    if (GPSfix != 3 || (Flags&0xD) != 0xD) {
        PrevNavTime = -11;
        NavTime = -12;
        return OK;
    }

    // Save the old clock data for calculating clock drift
    PrevNavTime = NavTime;
    PrevNavEpoch = NavEpoch;
    debug("   PrevNavtime=%lld  PrevNavEpoch=%lld\n", PrevNavTime, PrevNavEpoch);

    // Get the position and time information
    NavPos = Position(ECEF_X/100.0, ECEF_Y/100.0, ECEF_Z/100.0);
    NavVel = Position(ECEFVX/100.0, ECEFVY/100.0, ECEFVZ/100.0);
    NavPacc = Pacc/100.0;
    NavTime = ConvertGpsTime(week, ITOW/1000.0) + Frac;
    NavEpoch = NearestHz(NavTime);
    debug("     x=%.3f  y=%.3f  z=%.3f Pacc=%g\n", 
		        NavPos.x, NavPos.y, NavPos.z, Pacc/100.0);

    // If we have two epochs of data, ...
    if (PrevNavTime > 0) {

        // calculate the clock drift 
        ClockDrift = ((NavTime-PrevNavTime)-(NavEpoch-PrevNavEpoch)) / S(NavEpoch-PrevNavEpoch);
        debug("RawAntaris: NavEpoch=%lld ClockDrift=%.0f\n", NavEpoch, ClockDrift);
    }

    return OK;
}



/// processes a raw measurement record
bool RawAntaris::ProcessRawMeasurement(Block& block)
{
    // Assume no satellite has valid observations to start
    for (int s=0; s<MaxSats; s++)
        obs[s].Valid = false;

    // Parse the raw observation header
    LittleEndian b(block);
    int32 ITOW = b.Get4();  // measurement time of week (msec)
    int16 Week = b.Get2();  // gps measurement week nr
    uint8 NSV = b.Get();    // # of satellites
    uint8 RES1 = b.Get();   // Reserved
    debug("RawAntaris::ProcessRawmeasurement  ITOW=%d Week=%d NSV=%d\n", ITOW,Week,NSV);
    
    // Do for each satellite being tracked
    for (int i=0; i<NSV; i++) {
    
    // Get the measurements
    double CPMes = b.GetDouble();  // Carrier phase (cycles)
    double PRMes = b.GetDouble();  // Pseudorange measurement (m)
    float DOMes = b.GetFloat();    // Doppler Measurement
    uint8 SV = b.Get();            // Space Vehicle Number
    int8 MesQI = b.Get();          // Nav Meas Quality Indicator
    int8 CNO = b.Get();            // Signal strength 
    uint8 LLI = b.Get();           // Loss of lock (ala rinex)
    
    // Save the information
    int Sat = SvidToSat(SV);
    obs[Sat].PR = PRMes;
    obs[Sat].Phase = CPMes;
    obs[Sat].Doppler = DOMes;
    obs[Sat].Slip = (LLI&1) != 0;
    obs[Sat].SNR = CNO;
    obs[Sat].Valid = true;
    debug("      Sat=%d  PR(m)=%.3f  Phase(m)=%.3f Doppler(m)=%3f Slip=%d CNO=%d \n",
    	Sat, PRMes, CPMes*L1WaveLength, DOMes*L1WaveLength, LLI, CNO);
    }
    
    RawTime = ConvertGpsTime(Week, ITOW/1000.0);
    RawEpoch = NearestHz(RawTime);  // Assumes clock bias < 50msec
    debug("RawAntaris: Week=%d ITOW=%d Sec=%.3f RawTime=%lld RawEpoch=%lld\n",
                       Week, ITOW, ITOW/1000.0, RawTime, RawEpoch);
    
    // If we have navigation and clock drift information
    if (PrevNavTime > 0) {
    
        // ... Figure out how much we have to extrapolate
        Adjust = S(RawEpoch-NavEpoch);
        
        // ... Extrapolate the navigation information to match the raw data
        GpsTime = NavTime + (RawEpoch-NavEpoch) + (Time)(Adjust*ClockDrift);
        Pos = NavPos + Adjust*NavVel;
        Vel = NavVel;
        Cep = NavPacc;  // TODO:  Conversion??? 
        debug("RawAntaris (interpolating):  RawTime=%lld  RawEpoch=%lld\n",RawTime,RawEpoch);
        debug("    NavTime=%lld  NavEpoch=%lld\n",NavTime,NavEpoch);
        debug("    Adjust=%.9f ClockDrift=%.0f\n",Adjust,ClockDrift);
        debug("    GpsTime=%lld  GpsEpoch=%lld\n", GpsTime, GpsEpoch);
    }
    
    return OK;
}
    


/// processes an ephemeris message
/// Extracts the satellite navigation frame from the ublox message, then converts to ephemeris
bool RawAntaris::ProcessEphemeris(Block& block)
{
    // Get satellite number and check the HOW for validity
    LittleEndian b(block);
    int32 SVID = b.Get4();
    uint32 HOW = b.Get4();
    
    // Point to the satellite and its ephemeris
    int Sat = SvidToSat(SVID);
    debug("Antaris ProcessEphemeris: Sat=%d  HOW=0x%x  length=%d\n", Sat, HOW, block.Length); 
    	
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
    
    // Update the ephemeris. We convert the frame to raw ephemeris, and then to usable ephemeris
    EphemerisXmit& e = *(EphemerisXmit*)eph[Sat];
    EphemerisXmitRaw r;
    f.ToRaw(r);
    e.FromRaw(r);
    
    return OK;
    }



/// configures the ublox receiver to send raw data messages
bool RawAntaris::Initialize()
{
    strcpy(Description, "Antaris");
    
    PrevNavEpoch = -1;
    NavEpoch = -2;
    ClockDrift = 0;
    
    // Set up the ephemerides
    for (int s=0; s<MaxSats; s++)
        eph[s] = new EphemerisXmit(s, "Antaris");
    
    if (comm.ReadOnly()) return OK;
    
    // Disable the NMEA messages to conserve bandwidth
    for (int i=0; i<10; i++)
        if (DisableMessage(NMEA+i) != OK) return Error();
    for (int i=0; i<4; i++)
        if (DisableMessage(PUBX+i) != OK) return Error();
    
    // Take measurements HZ times a second.
    //   Raw measurements are up to 10hz, and nav measurements are up to 5hz
    if (SetMessageRate(HZ) != OK) return Error();
    
    // Configure to send the desired messages
    // Note: we may want to switch to subframes for non-interactive data logging
    if (EnableMessage(NAV_SOL) != OK) return Error();
    if (EnableMessage(RXM_EPH) != OK) return Error();
    if (EnableMessage(RXM_RAW) != OK) return Error();
    
    // Request an initial set of ephemerides. We should receive updates automatically.
    if (comm.PutBlock(RXM_EPH, -1) != OK) return Error();
    
    return OK;
}

/// Tells the ublox receiver to start sending a particlar message type
bool RawAntaris::EnableMessage(int id)
{
    // Turn on the specified message every cycle
    if (comm.PutBlock(CFG_MSG, id>>8, (byte)id, 1, -1) != OK) return Error();
    if (Ack() != OK) return Error("Can't enable message 0x%4x\n", id);
  
    return OK;
}

/// Tells the ublox receiver to stop a particular message type
bool RawAntaris::DisableMessage(int id)
{
    if (comm.PutBlock(CFG_MSG, id>>8, (byte)id, 0, -1) != OK) return Error();
    if (Ack() != OK) return Error("Can't disable message 0x%4x\n", id);
    return OK;
}

/// Tells the receiver how often to send raw observations
bool RawAntaris::SetMessageRate(int HZ)
{
    // Antaris supports up 10hz raw and 5hz computed, but we never do more than 1hz computed
    uint16 RxmMsec = 1000 / min(10, HZ);
    uint16 NavCycles = max(HZ, 1);   // No more than 1 per second
    	                                 //  or the receiver may get too busy
    
    // Request the message rate and get confirmation.
    if (comm.PutBlock(CFG_RATE, (byte)RxmMsec, RxmMsec>>8, NavCycles, 0,
    		              1, 0, -1) != OK) return Error();
    if (Ack() != OK) return Error("Can't set message rate to %dhz\n", HZ);
    return OK;
}




/// Waits for the ublox receiver to send an "ACK" message
bool RawAntaris::Ack()
{
    // Read a limited number of messages before giving up
    for (int i=0; i<20; i++) {
        Block b;
        if (comm.GetBlock(b) != OK) return Error("Antaris failed to find Ack\n");

        // if an Ack block, then done
        if (b.Id == ACK_ACK) return OK;
        if (b.Id == ACK_NAK) return Error("Antaris command rejected.\n");
    }

    return Error("No response (Ack or Nak) to Antaris command.\n");
}

