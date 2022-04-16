
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


#include "RawRtcm3.h"
#include "EphemerisXmit.h"

RawRtcm3::RawRtcm3(Stream& in)
: In(in)
{
    strcpy(Description, "Rtcm3.1");
    ErrCode = In.GetError();
    for (int s=0; s<MaxSats; s++) {
        PreviousPhase[s] = 0;
        PhaseAdjust[s] = 0;
        eph[s] = new EphemerisXmit(s, "RTCM 3.1");
    }

    // We don't know the time until we receive an ephemeris message
    //  Some streams may not have epemerides, so assume they are
    //  running realtime and get week number from current date.
    GuessTime = true;
    GpsTime = GetCurrentTime();
}


bool RawRtcm3::NextEpoch()
{
    // repeat until we get an observation record or an error
    bool errcode;
    Block b;
    do {

        // Read a frame
        if (In.GetBlock(b) != OK) return Error();

        // Process according to type of frame
        if      (b.Id == 1002)   errcode = ProcessObservations(b);
        else if (b.Id == 1005)   errcode = ProcessStationRef(b);
        else if (b.Id == 1019)   errcode = ProcessEphemeris(b);
        else                     errcode = OK;

    } until ( (b.Id == 1002 && GpsTime != -1) || errcode != OK);

    return errcode;
}



bool RawRtcm3::ProcessObservations(Block& blk)
{

    static const double MaxDelta = 0x7ffff;
    static const double BigDelta = MaxDelta - 800*L1WaveLength/.0005;

    // Assume no observations until shown otherwise
    for (int s=0; s<MaxSats; s++)
        obs[s].Valid = false;

    // Get the header info from the record
    Bits b(blk);
    int MessageId = b.GetBits(12);
    int StationId = b.GetBits(12);
    double Tow = b.GetBits(30) / 1000.0;
    int Synch = b.GetBits(1);
    int NrSats = b.GetBits(5);
    int Smoothing = b.GetBits(1);
    int Interval = b.GetBits(3);
    debug("RawRtcm3::ProcessObservations \n");
    debug(" Tow=%.3f Synch=%d NrSats=%d Smoothing=%d Interval=%d\n",
            Tow,   Synch,   NrSats,   Smoothing,   Interval);

    // Upate the time. 
    GpsTime = UpdateGpsTime(GpsTime, Tow);

    // Do for each measurement
    debug("   Svid Code   iPR  iDelta  Modulus  Locktime Snr\n");
    for (int i=0; i<NrSats; i++) {

        // Extract the measurement's fields
        int Svid = b.GetBits(6);
        int Code = b.GetBits(1);
        int32 iPR  = b.GetBits(24);
        int32 iDelta = b.GetSignedBits(20);
        int32 Modulus = b.GetBits(8);
        int32 LockTime = b.GetBits(7);
        int32 Snr = b.GetBits(8);
        debug("   %2d %d %8d %8d %3d %3d %3d\n",
               Svid,Code,iPR,iDelta,Modulus,LockTime,Snr);

        // Figure out which satellite
        int s = SvidToSat(Svid);
        RawObservation& o = obs[s];

        double PseudoRange = Modulus*(C/1000) + iPR*.02;
        double PhaseRange = PseudoRange + iDelta*.0005 
                              + PhaseAdjust[s]*L1WaveLength;

        // How do we determine if phase was adjusted by 1500 cycles?
        //  If we PhaseRange shrank, but it is at higher boundary
        double Doppler = PhaseRange - PreviousPhaseRange[s];
        if (Doppler < 0 && iDelta > BigDelta)       
           {PhaseAdjust[s] += 1500; PhaseRange += 1500*L1WaveLength;}
       else if (Doppler > 0 && iDelta < -BigDelta)  PhaseAdjust[s] += 1500;
           {PhaseAdjust[s] -= 1500; PhaseRange -= 1500*L1WaveLength;}

        o.PR = PseudoRange;
        o.Phase = PhaseRange / L1WaveLength;
        o.SNR = LevelToSnr(Snr);
        o.Slip = (LockTime < PreviousLockTime[s] || PreviousPhaseRange[s] == 0);
        o.Doppler = -(PhaseRange - PreviousPhaseRange[s]) / L1WaveLength;
        o.Valid = true;

        if (iDelta == 0x80000) 
            PhaseRange = o.Doppler = o.Phase = 0;
 
        if (PreviousPhaseRange[s] == 0)
            o.Doppler = 0;

        PreviousPhaseRange[s] = PhaseRange;
        PreviousLockTime[s] = LockTime;
        if (o.Slip)
            PhaseAdjust[s] = 0;

     }
    
    return OK;
}

bool RawRtcm3::ProcessStationRef(Block& blk)
{
    Bits b(blk);
    int MessageId = b.GetBits(12);
    int StationId = b.GetBits(12);
    int rsvd1 = b.GetBits(6);
    int gps = b.GetBits(1);
    int rdvd2 = b.GetBits(3);
    double x = b.GetSignedBits(38) * .0005;
    int rsvd3 = b.GetBits(2);
    double y = b.GetSignedBits(38) * .0005;
    int rsvd4 = b.GetBits(2);
    double z = b.GetSignedBits(38) * .0005;

    Pos = Position(x, y, z);


    return OK;
}


bool RawRtcm3::ProcessEphemeris(Block& blk)
{
    // Extract the raw ephemeris parameters
    EphemerisXmitRaw r;
    Bits b(blk);
    int msgid = b.GetBits(12);
    r.svid = b.GetBits(6);
    if (r.svid == 0) r.svid = 32;
    r.wn = b.GetBits(10);
    r.acc = b.GetBits(4);
    int code_on_l2 = b.GetBits(2);
    r.idot = b.GetSignedBits(14);
    r.iode = b.GetBits(8);
    r.t_oc = b.GetBits(16);
    r.a_f2 = b.GetSignedBits(8);
    r.a_f1 = b.GetSignedBits(16);
    r.a_f2 = b.GetSignedBits(22);
    r.iodc = b.GetBits(10);
    r.c_rs = b.GetSignedBits(16);
    r.delta_n = b.GetSignedBits(16);
    r.m_0 = b.GetSignedBits(32);
    r.c_uc = b.GetSignedBits(16);
    r.e = b.GetBits(32);
    r.c_us = b.GetSignedBits(16);
    r.sqrt_a = b.GetBits(32);
    r.t_oe = b.GetBits(16);
    r.c_ic = b.GetSignedBits(16);
    r.omega_0 = b.GetSignedBits(32);
    r.c_is = b.GetSignedBits(16);
    r.i_0 = b.GetSignedBits(32);
    r.c_rc = b.GetSignedBits(16);
    r.omega = b.GetSignedBits(32);
    r.omegadot = b.GetSignedBits(24);
    r.t_gd = b.GetSignedBits(8);
    r.health = b.GetBits(6);
    int l2pflag = b.GetBits(1);
    int fit_interval = b.GetBits(1);
    debug("RawRtcm3::ProcessEphemeris  svid=%d iode=%d wn=%d\n",
                                       r.svid, r.iode, r.wn);

    // Get a reference to the current ephemeris for this satellite
    int sat = SvidToSat(r.svid);
    EphemerisXmit& e = *dynamic_cast<EphemerisXmit*>(eph[sat]);
    if (&e == 0) return Error("rtcm ephemeris isn't correct\n");

    // If ephemeris changed, then update it
    if (e.iode != r.iode || e.iodc != r.iodc)
        if (e.FromRaw(r) != OK) return Error(); 

    // If we guessed the time, now update it with the ephemeris time
    if (GuessTime) {
        GpsTime = e.t_oe;
        GuessTime = false;
        debug("ProcessEphemeris - wn=%d  tow=%.2f\n", 
                          GpsWeek(GpsTime), GpsTow(GpsTime));
    }

    return OK;
}


RawRtcm3::~RawRtcm3()
{
}
