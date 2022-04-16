
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


#include "Rtcm3Station.h"
#include "Util.h"
#include "EphemerisXmit.h"




Rtcm3Station::Rtcm3Station(Stream& com, RawReceiver& gps, Attributes& attr)
: Gps(gps), comm(com), Station(attr)
{
    ErrCode = comm.GetError();

    // Setup times so we output all records at the beginning
    StationRefTime = -1;
    AntennaRefTime = -1;
    AuxiliaryTime = -1; 
    for (int s=0; s<MaxSats; s++)
        EphemerisTime[s] = -1;
        

    // Keep count of the number of each record type
    StationRefCount = 0;
    AntennaRefCount = 0;
    AuxiliaryCount = 0;
    ObservationsCount = 0;
    EphemerisCount = 0;

    // We need to keep track of slips
    for (int s=0; s<MaxSats; s++) {
        TrackingTime[s] = 0;
        PreviouslyValid[s] = false;
        PhaseAdjust[s] = 0;
    }		
}


bool Rtcm3Station::OutputEpoch()
{
    // Use the gps position if we haven't set it already
    if (Station.ARP == Position(0,0,0))
        Station.ARP = Gps.Pos;

    // if the time is right, output the various records
    if (StationRefTime <= Gps.GpsTime)
        if (OutputStationRef(StationRefTime) != OK) return Error();
    if (AntennaRefTime <= Gps.GpsTime)
        if (OutputAntennaRef(AntennaRefTime) != OK) return Error();
    if (AuxiliaryTime <= Gps.GpsTime) 
        if (OutputAuxiliary(AuxiliaryTime) != OK) return Error();

    // Send the epemerides if appropriate
    for (int s=0; s<MaxSats; s++) {
       if (EphemerisTime[s] <= Gps.GpsTime 
               && Gps.obs[s].Valid 
               && Gps[s].Valid(Gps.GpsTime))
           if (OutputEphemeris(s, EphemerisTime[s]) != OK) return Error();
    }

    // Send the measurements for this epoch
    //   Note we send the observations last. The earlier records may be
    //   needed to process the observations. (Ephemeride, antenna position ...)
    if (OutputObservations() != OK) return Error();
    

    return OK;
}

// MaxDelta is the largest phase-pseudo range which can be stored
static const double MaxDelta = 0x7ffff;

// ExtremeDelta means the phase has gone crazy and "slipped"
static const double ExtremeDelta = MaxDelta + 700*L1WaveLength/.02;

bool Rtcm3Station::OutputObservations()
{

    // Find how many valid satellites
    int nrsats = 0;
    for (int s=0; s<MaxSats; s++)
        if (Gps.obs[s].Valid)
            nrsats++;

    // Create the RTCM Observation header
    Block blk(1002);
    Bits b(blk);
    b.PutBits(1002, 12);         // Message Number 1002
    b.PutBits(Station.Id, 12);   // Station Id
    b.PutBits(round(GpsTow(Gps.GpsTime)*1000), 30); 
    b.PutBits(0, 1);             // Synchronous GNSS flag
    b.PutBits(nrsats, 5);        // No. of GPS Satellites Processed
    b.PutBits(0, 1);             // Smoothing (none)
    b.PutBits(0, 3);             // Smoothing interval (none)

    // Do for each valid satellite observation
    for (int s=0; s<MaxSats; s++) {
        RawObservation& o = Gps.obs[s];
        if (!o.Valid) {PreviouslyValid[s] = false;  continue;}

        // Calculate PR value
        uint32 Modulus = floor(o.PR / (C/1000));
        uint32 iPR = round((o.PR - Modulus*(C/1000)) / .02);
        double pseudorange = Modulus*(C/1000) + iPR*.02;

        // Calculate phaserange value
        int32 oldadjust = PhaseAdjust[s];
        double phaserange = (o.Phase-PhaseAdjust[s]) * L1WaveLength;
        double iDelta = round( (phaserange-pseudorange)/0.0005  );

        // Case: there was a slip. Create a new phase adjustment
        if (o.Slip || !PreviouslyValid[s] || abs(iDelta) > ExtremeDelta) {
            PhaseAdjust[s] = round(o.Phase - o.PR/L1WaveLength);
            TrackingTime[s] = 0;
        } 
    
        // Case: phase has drifted too high to store in field. adjust
        else if (iDelta > MaxDelta) 
            PhaseAdjust[s] += 1500;

        // Case: phase has drifted too low to store in field. adjust
        else if (iDelta < -MaxDelta) 
             PhaseAdjust[s] -= 1500;

        // Recompute phase range if adjustments were made
        if (PhaseAdjust[s] != oldadjust) {
            phaserange = (o.Phase-PhaseAdjust[s]) * L1WaveLength;
            iDelta = round(  (phaserange-pseudorange)/.0005  );
        }

        // If there was no phase measurement, then throw away all of the above
        if (o.Phase == 0) iDelta = 0x40000;

        // Add the satellite's data to the observation record
        b.PutBits(SatToSvid(s), 6);     // GPS Satellite ID
        b.PutBits(0, 1);               // GPS L1 Code Indicater (C/A)
        b.PutBits(iPR, 24);             // GPS L1 Pseudorange
        b.PutBits((uint32)iDelta, 20);   // GPS L1 PhaseRange - L1 Pseudorange
        b.PutBits(Modulus, 8);         // GPS L1 Pseudorange Modulus Ambiguity
        b.PutBits(LockTime(TrackingTime[s]), 7);  // GPS L1 Lock time indicator;
        b.PutBits(SnrToLevel(o.SNR), 8); // GPS L1 CNR
    
        TrackingTime[s]++;
        PreviouslyValid[s] = o.Valid;
    }

    // Output the observations record
    return comm.PutBlock(blk);
    } 




bool Rtcm3Station::OutputStationRef(Time& time)
{
    Block  blk(1005);
    Bits   b(blk);

    // Assemble the Station Reference record
    b.PutBits(1005, 12);  
    b.PutBits(Station.Id, 12);
    b.PutBits(0,6);
    b.PutBits(1,1);   // GPS measurements
    b.PutBits(0, 3);
    b.PutBits(round(Station.ARP.x/.0005), 38);
    b.PutBits(0,2);
    b.PutBits(round(Station.ARP.y/.0005), 38);
    b.PutBits(0,2);
    b.PutBits(round(Station.ARP.z/.0005), 38);

    // Reschedule in a minute
    time = Gps.GpsTime + 60*NsecPerSec;

    // Output it
    return  comm.PutBlock(blk);
}





bool Rtcm3Station::OutputAntennaRef(Time& time)
{
    return OK;
}


bool Rtcm3Station::OutputAuxiliary(Time& time)
{
    return OK;
}


bool Rtcm3Station::OutputEphemeris(int s, Time& NextTime)
{
    debug("OutputEphemeris: s=%d\n", s);


    // if not a broadcast ephemeris, don't schedule again
    EphemerisXmit& e = *dynamic_cast<EphemerisXmit*>(&Gps[s]);
    if (&e == NULL) {
        NextTime = MaxTime;
        return Error("Rtcm31: No xmit ephemeris\n");
    }
	

    // Schedule every two minutes, spread out on odd seconds
    NextTime = Gps.GpsTime - Gps.GpsTime%(2*NsecPerMinute) + 2*NsecPerMinute
     	  + s*2*NsecPerSec + NsecPerSec;
	
    // Convert broadcast ephemeris to "raw" form
    EphemerisXmitRaw r;
    e.ToRaw(r);
    debug("Rtcm3Station::OutputEphemeris svid=%d wn=%d t_oe=%d iode=%d\n",
           r.svid, r.wn, r.t_oe, r.iode);

    // Build up the RTCM message
    Block  blk(1019);
    Bits   b(blk);

    // Assemble the Ephemeris record
    b.PutBits(1019, 12);                   // Message id 1019
    b.PutBits(SatToSvid(s), 6);            // Satelite number
    b.PutBits(GpsWeek(Gps.GpsTime), 10);   // Week number
    b.PutBits(r.acc, 4);                   // Accuracy
    b.PutBits(2, 2);                       // C/A on L2  TODO: is this right?
    b.PutBits(r.idot, 14);
    b.PutBits(r.iode, 8);
    b.PutBits(r.t_oc, 16);
    b.PutBits(r.a_f2, 8);
    b.PutBits(r.a_f2, 16);
    b.PutBits(r.a_f0, 22);
    b.PutBits(r.iodc, 10);
    b.PutBits(r.c_rs, 16);
    b.PutBits(r.delta_n, 16);
    b.PutBits(r.m_0, 32);
    b.PutBits(r.c_uc, 16);
    b.PutBits(r.e, 32);
    b.PutBits(r.c_us, 16);
    b.PutBits(r.sqrt_a, 32);
    b.PutBits(r.t_oe, 16);
    b.PutBits(r.c_ic, 16);
    b.PutBits(r.omega_0, 32);
    b.PutBits(r.c_is, 16);
    b.PutBits(r.i_0, 32);
    b.PutBits(r.c_rc, 16);
    b.PutBits(r.omega, 32);
    b.PutBits(r.omegadot, 24);
    b.PutBits(r.t_gd, 8);
    b.PutBits(r.health, 6);
    b.PutBits(1, 1);   // L2 P data is OFF
    b.PutBits(0, 1);   // Fit interval is OFF  TODO: Is this right?

    return comm.PutBlock(blk);
}



uint32 Rtcm3Station::LockTime(uint32 i)
////////////////////////////////////////////////////////////
// Locktime converts Lock Time Indicator according table 3.4-2 in RTCM 3.0
////////////////////////////////////////////////////////////////////////
{
    if (i < 24)   return i;
    if (i < 72)   return (i + 24) / 2;
    if (i < 168)  return (i + 120) / 4; 
    if (i < 360)  return (i + 408) / 8;
    if (i < 744)  return (i + 1172) / 16;
    if (i < 937)  return (i + 3096) / 32;
    return 127;
}



Rtcm3Station::~Rtcm3Station(void)
{
}

