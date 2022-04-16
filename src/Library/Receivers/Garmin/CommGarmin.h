// CommGarmin.h: interface for the CommGarmin class.
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

#ifndef COMMGARMIN_INCLUDED
#define COMMGARMIN_INCLUDED

#include "util.h"
#include "comm.h"


// Garmin Commands
	enum {
		PositionRecordOn = 49,
		PositionRecordOff = 50,
		ReceiverMeasurementOn = 110,
		ReceiverMeasurementOff = 111,
		AsyncRecordsOn = 0x1C
	};

	// Garmin Record types
enum {
	    PidCommandData = 10,      // Sent to Garmin
		PidProtocolArray = 253,
		PidProductRequest = 254,
		PidProductData = 255,
		PidPositionRecord = 51,
		PidReceiverMeasurement = 52,
		PidSatelliteData = 114,
		PidNavigationData = 0x36
	};


	// Garmin Data records
	#pragma pack(push, 1)

	struct  ProductData
	{
		int16 ProductId;
		int16 SoftwareVersion;
		char  Description[256];
	};

	typedef struct 
	{
		byte  Tag;
		int16 Value;
	} ProtocolArray[1];

	struct PositionRecord {
		float	alt;
		float	epe;
		float	eph;
		float	epv;
		int16	fix;
		double	gps_tow;
		double	lat;
		double	lon;
		float	lon_vel;
		float	lat_vel;
		float	alt_vel;
		float	msl_hght;
		int16	leap_sec;
		int32	grmn_days;
	};

	struct ReceiverMeasurement{
		double	rcvr_tow;
		uint16	rcvr_wn;
		struct {
			int32	cycles;
			double	pr;
			int16	phase;
			byte	slp_dtct;
			byte	snr_dbhz;
			byte	svid;
			byte	valid;
		} Sat[12];
	};

	typedef struct {
		uint8	svid;
		uint16	snr;
		uint8	elev;
		uint16	azmth;
		uint8	status;
	} SatelliteData[12];

#pragma pack(pop)

#endif // COMMGARMIN_INCLUDED


