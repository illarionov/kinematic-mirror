#ifndef RTCM3STATION_INCLUDED
#define RTCM3STATION_INCLUDED
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


#include "CommRtcm3.h"
#include "RawReceiver.h"
#include "Util.h"

   


class Rtcm3Station
{
public:
    struct Attributes {
        int Id;
        Position ARP;
        char AntennaDesc[41];
        int AntennaSetupId;
    };

protected:
	RawReceiver& Gps;
	CommRtcm3 comm;
	Attributes Station;
	bool ErrCode;

	// When to output the next rtcm frames
	Time StationRefTime;
	Time AntennaRefTime;
	Time AuxiliaryTime;
        Time EphemerisTime[MaxSats];

        int StationRefCount;
        int AntennaRefCount;
        int AuxiliaryCount;
        int ObservationsCount;
        int EphemerisCount;
    

	// keep track of whether we are locked to each satellite
        uint32 TrackingTime[MaxSats];
	int32 PhaseAdjust[MaxSats];
        bool PreviouslyValid[MaxSats];

public:	
	Rtcm3Station(Stream& com, RawReceiver& gps, Attributes& attr);
        bool OutputEpoch();
	bool GetError() {return ErrCode;}
	virtual ~Rtcm3Station(void);

private:
	bool OutputStationRef(Time& NextTime);
        bool OutputAntennaRef(Time& NextTime);
	bool OutputAuxiliary(Time& NextTime);
        bool OutputObservations();
        bool OutputEphemeris(int s, Time& NextTime);

        uint32 LockTime(uint32 TrackingTime);
        int32 RtcmPhase(uint32 PR, uint32 Modulus, double phase, int32& adjust);

        
};

#endif // RTCMSTATION_INCLUDED
