
// RawRinex reads raw gps data from a RINEX file
//    Part of kinematic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    kinematic@precision-gps.org
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


#include "RawRinex.h"
#include "RinexParse.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

RawRinex::RawRinex(Stream& in)
: In(in)
{
	ErrCode = Initialize();
}

bool RawRinex::Initialize(int baud)
{
	strcpy(Description, "RinexFile");
	if (In.GetError() != OK)
		return Error("Unable to open Rinex input file\n");

	// Create dummy ephemerides
	for (int s=0; s<MaxSats; s++)
		eph[s] = new EphemerisDummy(s, "Rinex Dummy Ephemeris");

	return ReadHeader();
}



bool RawRinex::ReadHeader()
{
	// Repeat for each line in the header
	char line[128];
	while (ReadLine(line, sizeof(line)) == OK) {
		debug(2,"Rinex::ReadHeader: line=%s\n", line);

		// Process according to the description field

		// Case: Approximate position 
		if (match(line, 60, "APPROX POSITION XYZ")) {
			Pos.x = GetDouble(line, 0, 14);
			Pos.y = GetDouble(line, 14, 14);
			Pos.z = GetDouble(line, 28, 14);
			debug(2,"Rinex::ReadHeader; Pos=(%.4f,%.4f,%.4f)\n",Pos.x,Pos.y,Pos.z);
		}

		// Case: types of observations
		else if (match(line, 60, "# / TYPES OF OBSERV")) {

			// Get the number of observations. (we can handle up to 9)
			NrMeasurements = GetInt(line, 0, 6);
			if (NrMeasurements < 0 || NrMeasurements > 9) 
				return Error("Rinex error: too many types of observations (%d)\n",NrMeasurements);

			// Look for the L1 observation types
			C1Index = L1Index = D1Index = S1Index = -1;
			for (int i=0; i<NrMeasurements; i++)
				if (match(line, 10+i*6, "L1"))
					L1Index = i;
				else if (match(line, 10+i*6,"C1"))
					C1Index = i;
				else if (match(line, 10+i*6,"D1"))
					D1Index = i;
				else if (match(line, 10+i*6,"S1"))
					S1Index = i;

			debug(2,"L1Index=%d  C1Index=%d  D1Index=%d S1Index=%d\n", 
				     L1Index, C1Index, D1Index, S1Index);

			// At a minimum, we need to have pseudoranges
			if (C1Index == -1)
				return Error("Rinex error: Doesn't contain C1 pseudorange data");
		}

		// Case: Starting time
		else if (match(line, 60, "TIME OF FIRST OBS")) {
			StartYear = GetInt(line, 0, 6);
		}

		// Case: end of header
		else if (match(line, 60, "END OF HEADER"))
			return OK;
	}
	
	// Reached end of file without completing header
	return Error("Rinex error: couldn't find 'end of header'");
}





bool RawRinex::NextEpoch()
{
    for (int s=0; s<MaxSats; s++)
		obs[s].Valid = false;

	// Repeat until we get an epoch flag indicating data
	int EpochFlag;
	do {
	    // Get a line from the rinexfile
		char line[128];
	    if (ReadLine(line, sizeof(line))) return Error();

		// Get the type of data, and date if present
		EpochFlag = GetInt(line, 28, 1);
		ParseRinexTime(line, GpsTime);
		debug("Rinex::NextEpoch GpsTime=%.3f EpochFlag=%d line=%s\n", 
			S(GpsTime), EpochFlag, line);

	    // Process according to the type of header
		if (EpochFlag == 0 || EpochFlag == 1)  {      
			if (ProcessObservations(line) != OK) return Error();
		} else if (EpochFlag == 6) {
			if (ProcessFixups(line) != OK) return Error();
		} else {
			if (ProcessEvent(line) != OK) return Error();
		}
	} while (EpochFlag > 1);

	AdjustToHz();

	return OK;
}

int MaxSatsPerLine = 12;  // Make it a variable cause we have to read some old files
                          //  where it was done incorrectly.

bool RawRinex::ProcessObservations(char* line)
{
	// Get the number of satellites
	int NrSats = GetInt(line, 30, 3);

	// Get the id of the first 12 satellites
	byte sats[MaxSats];
	for (int i=0; i<NrSats && i<MaxSatsPerLine; i++) {
		int svid = ParseSvid(line, 32+3*i);
		if (svid == -1) 
			return Error("Bad Rinex SVID: i=%d line=%s\n", i, line);
		sats[i] = (byte)SvidToSat(svid);
	}

	// Get the remaining ids, reading continuation lines as needed
	//  Note: merge this with previous block of code
	for (int i=MaxSatsPerLine; i<NrSats; i++) {
		char line[128];
		if ( (i%MaxSatsPerLine) == 0)
		    if (ReadLine(line, sizeof(line)) != OK) return Error();
		int svid = ParseSvid(line, 32+3*(i%MaxSatsPerLine));
		if (svid == -1)
		    return Error("Bad Rinex SVID: i=$d line=%s\n", i, line);
		sats[i] = SvidToSat(svid);
	}

	// Do for each satellite in view
	for (int i=0; i<NrSats; i++) {
		int Sat = sats[i];
		RawObservation& o=obs[Sat];

		// Set defaults in case we don't have measurement
		o.PR = o.Phase = o.SNR = o.Doppler = 0;
		o.Valid = true;

		// Do for each observation in the RINEX file
		for (int j=0; j<NrMeasurements; j++) {
			char line[128];

			// Read an 80 char line when needed
			int col = (j*16)%80;
			if (col == 0)
				if (ReadLine(line, sizeof(line)) != OK) return Error();

			// Process according to the measurement type
			if (j == L1Index) 
				ParsePhaseObservation(line, col, o.Phase, o.Slip, o.SNR);
            else if (j == C1Index)
				ParseObservation(line, col, o.PR, o.SNR);
			else if (j == S1Index)
				ParseObservation(line, col, o.SNR, o.SNR);
			else if (j == D1Index)
				ParseObservation(line, col, o.Doppler, o.SNR);
		}
	}

	return OK;
}


bool RawRinex::ProcessFixups(char* line)
{
	return Error("RawRinex - Phase fixups are not implemented yet\n");
}

bool RawRinex::ProcessEvent(char* line)
{
	// Get the number of header records which follow
	int NrHeaders = GetInt(line, 30, 3);
	if (NrHeaders > 999) return Error("Invalid RINEX event header\n");

	// Skip the headers. (later, want to treat them as events)
	for (int i=0; i<NrHeaders; i++) {
		char line[128];
		if (ReadLine(line, sizeof(line)) != OK) return Error();
	}
	return OK;
}


bool RawRinex::ParseRinexTime(char* line, Time& time)
{
	// If the time fields are blank, then use existing time
	if (match(line, 0, "     ")) return OK;

	// Get the date and time
	int32 year, month, day, hour, min;
	double sec;
	year = GetInt(line,0, 4);
	month = GetInt(line, 4, 3);
	day = GetInt(line, 7, 3);
	hour = GetInt(line, 10,3);
	min = GetInt(line,13, 3);
	sec = GetDouble(line, 16, 11);

	// Since year is given as 2 digits, get the century from the 4 digit start time
	int32 cent = StartYear/100;
	if (year < StartYear%100) cent++;  // don't have to worry about this for a while

	// Convert Date to time
	time = DateToTime(cent*100+year, month, day) + TodToTime(hour, min, sec);

	return OK;
}




bool RawRinex::ParsePhaseObservation(char* line, int col, double& phase, bool& slip, double& snr)
{
	phase = GetDouble(line, col, 14);
	slip = phase == 0 || (GetInt(line, col+14, 1)&1) != 0;
	if (line[col+15] != ' ' && snr == 0)
		snr = LevelToSnr(GetInt(line, col+15, 1));
	debug(3, "ParsePhaseObservation: col=%d phase=%.3f slip=%d\n", col, phase, slip);
	return false;
}

bool RawRinex::ParseObservation(char* line, int col, double& value, double& snr)
{
	value = GetDouble(line, col,14);
	if (line[col+15] != ' ' && snr == 0)
		snr = LevelToSnr(GetInt(line, col+15, 1));
	debug(3, "ParseObservation: col=%d value=%.3f  snr=%.3f\n", col, value, snr);
	return OK;
}

bool RawRinex::ReadLine(char* line, int len)
{
	bool ret = In.ReadLine(line, len);
	
	// get the length of the line
	int slen = strlen(line);
	debug(4, "Rinex::ReadLine   len=%d  slen=%d  line=%s\n",len,slen,line);

	// fill in the trailing spaces
	for (int i=slen; i<len-1; i++)
		line[i] = ' ';

	// And always terminate with a null
	line[len-1] = '\0';

	debug(4, "Rinex::ReadLine - ""%s""\n", line);
	return ret;
}

RawRinex::~RawRinex()
{
}

