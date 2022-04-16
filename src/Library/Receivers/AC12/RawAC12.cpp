
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


#include "RawAC12.h"

RawAC12::RawAC12(Stream& s)
: comm(s)
{
    ErrCode = comm.GetError();
    if (ErrCode != OK) return;
    
    strcpy(Description, "ThalesAC12");
    
    // Set up the ephemerides
    for (int s=0; s<MaxSats; s++)
        eph[s] = new EphemerisXmit(s, "AC12 Broadcast Ephemeris");
    
    // We don't have a valid GPS time until we read the first Ephemeris record
    //  (it is the only record with Gps Week)
    GpsTime = -1;
    
    if (comm.ReadOnly()) return;
    
    // Turn off NMEA messages
    ErrCode = Command("$PASHS,NME,ALL,A,OFF"); if (ErrCode != OK) return;
    
    // Disable WAAS so we can get 12 channels of raw measurements
    ErrCode = Command("$PASHS,WAS,OFF");  if (ErrCode != OK) return;
    
    //ErrCode = Command("$PASHS,SMI,0");   if (ErrCode != OK) return;
    Command("$PASHS,SMI,0");  // Older firmware doesn't recognize it
    
    // Enable the Position, raw measurements, ephemeris and error
    ErrCode = Command("$PASHS,NME,PBN,A,ON,1");  if (ErrCode != OK) return;
    ErrCode = Command("$PASHS,NME,MCA,A,ON,1");  if (ErrCode != OK) return;
    ErrCode = Command("$PASHS,NME,RRE,A,ON,1");  if (ErrCode != OK) return;
    ErrCode = Command("$PASHS,NME,SNV,A,ON,1");  if (ErrCode != OK) return;

    debug("RawAc12::RawAc12 - Success!\n");
}


/// reads the next set of raw observations from the AC12 receiver
bool RawAC12::NextEpoch()
{
Block b;

MeasurementTag = -1; 
PositionTag = -2;

    // Assume no valid measurements until proven otherwise
    for (int s=0; s<MaxSats; s++)
        obs[s].Valid = false;

    // Repeat until we have a complete epoch
    //  An epoch is when we receive some raw measurements followed by
    //  a matching position. 
    //  For now, session ends with position
    do {
        // Read a block of data
        if (comm.GetBlock(b) != OK) return Error();
        
        // Process according to type
        if      (b.Id == 'PBN')   ProcessPosition(b);
        else if (b.Id == 'MCA')   ProcessMeasurement(b);
        else if (b.Id == 'SNV')   ProcessEphemeris(b);
        else if (b.Id == 'RRE')   ProcessResiduals(b);
        else                      b.Display("AC12: Unknown block");
        
    } until (b.Id == 'PBN' && GpsTime != -1);  // while (MeasurementTag != PositionTag);
    
    debug("AC12 NextEpoch: GpsTime=%.1f\n", S(GpsTime));
    
    return OK;
}


/// Processes an AC12 calculated position record
bool RawAC12::ProcessPosition(Block& blk)
{
    // Validate the checksum
    // (later)

    // Extract the data
    BigEndian b(blk);
    int32 rcvtime = b.Get4();
    char sitename[5];
    for (int i=0; i<4; i++)
        sitename[i] = b.Get();
    sitename[4] = '\0';
    
    double x = b.GetDouble();
    double y = b.GetDouble();
    double z = b.GetDouble();
    float t = b.GetFloat();
    
    float xdot = b.GetFloat();
    float ydot = b.GetFloat();
    float zdot = b.GetFloat();
    float tdot = b.GetFloat();
    
    int pdop = b.Get2();
    
    debug("AC12 received position: rcvtime=%d  pos=(%.3f %.3f %.3f) t=%.9f\n",
    rcvtime, x, y, z, t);
    debug("   vel=(%.3f %.3f %.3f)  tdrift=%.9f pdop=%d\n", 
             xdot, ydot, zdot, tdot, pdop);
    
    // if we don't have a fix or we don't know the Gps week, then we're done
    debug("RawAC12(position) GpsTime=%.3f Tow=%.3f\n", S(GpsTime),rcvtime/1000.0);
    if (pdop == 0 || GpsTime == -1) 
    return OK;
    
    // update the gps position and time
    debug("  GpsTow(old)=%.3f  rcvtime=%.3f  t=%.9f  tdot=%.9f\n", 
    GpsTow(GpsTime), rcvtime/1000.0, (double)t, (double)tdot);
    Pos = Position(x, y, z);
    Vel = Position(xdot, ydot, zdot);
    GpsTime = UpdateGpsTime(GpsTime, rcvtime / 1000.0);
    
    // Create a position tag akin to the raw measurement tag
    //   (50 msec units, modulo 30 minutes)
    PositionTag = (rcvtime/50) % (30*60*20);
    debug("AC12ProcessPosition: Pos=(%.3f %.3f %.3f)  PositionTag=%d\n",
                          Pos.x, Pos.y, Pos.z, PositionTag);
    
    return OK;
}
     

/// processes an AC12 raw measurement record
bool RawAC12::ProcessMeasurement(Block& block)
{
    BigEndian b(block);
    int Tag = b.Get2();
    int left = b.Get();
    int svid = b.Get();
    int elev = b.Get();
    int azim= b.Get();
    int channel = b.Get();
    int warning = b.Get();
    int GoodBad = b.Get();
    int PolarityKnown = b.Get();
    int SNR = b.Get();
    int qa_phase = b.Get();
    double phase = b.GetDouble();
    double range = b.GetDouble() * C;
    double doppler = (int32)b.Get4() / -10000.0;
    uint32 smoothing = b.Get4();
    int SmoothCount = smoothing>>24;
    double SmoothCorrection = (smoothing&0x7fffff) / 100.0;
    if ( (smoothing&0x800000) != 0)  SmoothCorrection = -SmoothCorrection;
    
    debug("AC12 svid=%d  range=%.3f  phase=%.3f doppler=%.3f SNR=%d  warn=0x%x left=%d \n",
       svid,    range,   phase,    doppler,     SNR,   warning,  left);
    debug("         Tag=%d  Smooth count=%d  Correction=%.3f\n", 
              Tag, SmoothCount,SmoothCorrection);
    
    // If the beginning of a new epoch, then clear the observations
    if (MeasurementTag != Tag)
    for (int s=0; s<MaxSats; s++)
    obs[s].Valid = false;
    MeasurementTag = Tag;
    
    // Add in the current observation
    int s = SvidToSat(svid);
    obs[s].Valid = true;
    obs[s].PR = range - SmoothCorrection;
    obs[s].Phase = phase;
    obs[s].Doppler = doppler;
    obs[s].SNR = SNR;
    obs[s].Sat = s;
    obs[s].Slip = (GoodBad&0xc0) != 0;
    
    return OK;
}

bool RawAC12::ProcessEphemeris(Block& blk)
{
    blk.Display("Received Epemeris\n");
    
    // Validate the checksum 
    //  (later)
    
    // Point to the appropriate ephemeris
    //   The satellite prn is at the end of the record, so we peek ahead
    //   rather than unpacking it sequentially.
    int Sat = SvidToSat(blk.Data[128]+1);
    if (Sat == -1) return Error("Invalid Ephemeris record");
    EphemerisXmit& e = *(EphemerisXmit*)eph[Sat];
    
    // Extract the time the navigation record was originally sent. (??)
    BigEndian b(blk);
    int WN = b.Get2();             // Week nr
    int Tow = b.Get4();            // Seconds into week (Garbage!!!)
    Time NavTime = ConvertGpsTime(WN, Tow);
    debug("AC12 Ephemeris:  Sat=%d  WN=%d  Tow=%d  NavTime=%.0f\n", 
    Sat, WN, Tow, S(NavTime));
    
    // get the relativistic value
    e.t_gd = b.GetFloat();      // group delay
    
    // Get the clock information
    e.iodc = b.Get4();           // clock issue data
    double tow_oc = b.Get4();    // reference time of clock corrections
    e.t_oc = ConvertGpsTime(WN, tow_oc); 
    e.a_f2 = b.GetFloat();      // clock correction (sec/sec/sec)
    e.a_f1 = b.GetFloat();      // clock correction (sec/sec)
    e.a_f0 = b.GetFloat();      // clock correction (sec)
    
    // Get the ephemeris information
    e.iode = b.Get4();               // ephemeris issue of data
    e.delta_n = b.GetFloat() * PI;   // mean anomaly correction (radians)
    e.m_0 = b.GetDouble() * PI;      // mean anomaly at reference time (radians)
    e.e = b.GetDouble();             // Eccentricity
    e.sqrt_a = b.GetDouble();        // square root of semi-major axis
    double tow_oe = b.Get4();        // Reference time for ephemeris (sec)
    e.t_oe = ConvertGpsTime(WN, tow_oe);  
    
    // Harmonic corrections
    e.c_ic = b.GetFloat();      // harmonic correction term (radians)
    e.c_rc = b.GetFloat();      // harmonic correction term (meters)
    e.c_is = b.GetFloat();      // harmonic correction term (radians)
    e.c_rs = b.GetFloat();      // harmonic correction term(meters)
    e.c_uc = b.GetFloat();      // harmonic correction term (radians)
    e.c_us = b.GetFloat();      // harmonic correction term (radians)
    
    // Orbit description
    e.omega_0 = b.GetDouble() * PI; // longitude of ascending node (radians)
    e.omega = b.GetDouble() * PI;   // argument of perigee (radians)
    e.i_0 = b.GetDouble() * PI;     // inclination angle (radians)
    e.omegadot = b.GetFloat() * PI; // rate of right ascension (radians/sec)
    e.idot = b.GetFloat() * PI;     // rate of inclination (radians/sec)
    
    // Satellite state
    e.acc = b.Get2();     // user range accuracy TODO - what units?
    e.health = b.Get2();           // satellite health
    int fit = b.Get2();            // curve fit interval
    byte prnnum = b.Get();         // svid - 1
    e.MinTime = e.t_oe - 2*NsecPerHour;  // TODO: should be based on fit
    e.MaxTime = e.t_oe + 2*NsecPerHour;
    
    e.Display("AC12 Ephemeris Processed");
    
    // If the AC12 hasn't seen a valid time yet, use the time from the ephemeris.
    //   This is where we get the initial WN.
    //   The position records will keep GpsTime up-to-date from here on.
    if (GpsTime == -1) 
        GpsTime = e.t_oe;  // Would use Navtime, but it is garbage
    
    return OK;
}


/// Process the AC12 residuals message for position error
bool RawAC12::ProcessResiduals(Block& blk)
{
    NmeaBlock b(blk);

    // Get the nmea message type
    char type[sizeof(b)];
    b.GetField(type, sizeof(type));

    // Get the number of satellites
    int nsat = b.GetInt();
    
    // Skip the satellite residuals
    for (int i=0; i<nsat; i++) {
        int sat = b.GetInt();
        double resid = b.GetFloat();
        debug("%d(%.2f) ",sat,resid);
    }

    // Get the position error
    double hpos = b.GetFloat();  // RMS horizontal
    double vpos = b.GetFloat();  // RMS vertical
    debug("hpos=%.2f  vpos=%.2f\n",hpos,vpos);

    Cep = hpos;  // Conversion???

    return OK;
}


/// Send a command to the AC12
bool RawAC12::Command(const char* cmd)
{
    Block blk(0);
    BlockPacker b(blk);
    
    // Copy the command into the block
    b.Put(cmd);
    b.Put("\r\n");
    
    // Send the block and wait for ACK
    if (comm.PutBlock(blk) != OK) return Error();
    if (AckOrNak() != OK) return Error("AC12: failed to respond to command %s\n", cmd);
    
    return OK;
}


/// waits for the AC12 to send an Ack or a Nak message
bool RawAC12::AckOrNak()
{
    for (int i=0; i<20; i++) {
        Block b;
        if (comm.GetBlock(b) != OK)  return Error();
        else if (b.Id == 'ACK')      return OK;
        else if (b.Id == 'NAK')      return Error("AC12: received NAK message\n");
    }
    
    return Error("AC12: failed to Ack or Nak\n");
}



RawAC12::~RawAC12()
{
}



