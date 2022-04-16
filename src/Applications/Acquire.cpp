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

#include "InputFile.h"
#include "Rinex.h"
#include "Rtcm3Station.h"
#include "SqliteLogger.h"
//#include "DgpsStation.h"
#include "NewRawReceiver.h" 
#include <stdio.h>

bool Configure(int argc, const char** argv);
void DisplayHelp();

Rinex* NewRinex(const char* FileName, RawReceiver& gps);
Rtcm3Station* NewRtcm(const char* FileName, RawReceiver& gps);
SqliteLogger* NewLogger(const char* FileName, RawReceiver& gps);
//DgpsStation* NewDgps(const char* FileName, RawReceiver& gps);

// Globals which are set up by "configure"
const char *RinexName;
const char *RawName;
const char *RtcmName;
const char *LogName;
const char *DgpsName;
const char *Model;
const char *PortName;
Position InitialPos;
extern int DebugLevel;
int HZ;

// A flag telling us when to stop
bool MoreToDo = true;

int main(int argc, const char** argv)
{
        setlinebuf(stdout);

	// Get configured according to arguments
	if (Configure(argc, argv) != OK) {
		DisplayHelp();
		return ShowErrors();
	}

	// Initialize the gps receiver
	RawReceiver::HZ = HZ;
	RawReceiver* gps = NewRawReceiver(Model, PortName, RawName);
	if (gps == NULL || gps->GetError() != OK) {
		Error("Unable to initialize the %s gps on port %s\n", Model,PortName);
		return ShowErrors();
	}

	// Create the RINEX output file
	Rinex* rinex = NewRinex(RinexName, *gps);
	if (rinex == NULL && RinexName != NULL) return ShowErrors();

	// Create the RTCM output file
	Rtcm3Station* rtcm = NewRtcm(RtcmName, *gps);
	if (rtcm == NULL && RtcmName != NULL) return ShowErrors();

	// Create the DGPS output file
	//DgpsStation* dgps= NewDgps(DgpsName, *gps);
	//if (dgps == NULL && DgpsName != NULL) return ShowErrors();

        // Create an sqlite log file
        SqliteLogger* logger = NewLogger(LogName, *gps);
        if (logger == NULL && LogName != NULL) return ShowErrors();

	// Get first epoch
	printf("Waiting for data from %s on port %s\n", Model, PortName);
	if (gps->NextEpoch() != OK) return ShowErrors();

	// if configured to override the first position, use the override
	if (InitialPos.x != 0 || InitialPos.y != 0 || InitialPos.z != 0)
		gps->Pos = InitialPos;

	// Repeat until we decide to stop
	while (MoreToDo) {

		// Display the satellites being tracked
		int32 day, month, year, hour, min, sec, nsec;
		TimeToDate(gps->GpsTime, year, month, day); 
		TimeToTod(gps->GpsTime, hour, min, sec, nsec);
		printf("%2d/%02d/%04d %02d:%02d:%02d  ", month,day,year,hour,min,sec);
		for (int s=0; s<MaxSats; s++) {
			if (gps->obs[s].Valid)
				if ((*gps)[s].Valid(gps->GpsTime)) printf("*%d ",SatToSvid(s));
				else                               printf("%d ", SatToSvid(s));
		}
		printf("\07\n");

            

		// Write it out as Rinex
		if (rinex != NULL)
		    if (rinex->OutputEpoch() != OK) return ShowErrors();

		// Write it out as RTCM
		if (rtcm != NULL)
			if (rtcm->OutputEpoch() != OK) return ShowErrors();

                if (logger != NULL)
                    if (logger->OutputEpoch() != OK) return ShowErrors();

		// Write it out as DGPS
//		if (dgps != NULL)
//			if (dgps->OutputEpoch() != OK) return ShowErrors();

		// Read next epoch of data
		if (gps->NextEpoch() != OK) return ShowErrors();
	}

	// Done
	delete rinex;
//	delete dgps;
	delete rtcm;
        delete logger;
	delete gps;

	return 0;

err:
	ShowErrors();
	return 1;
}


bool Configure(int argc, const char** argv)
{
	// Set the defaults
	InitialPos = Position(0,0,0);
	RawName = NULL;
	Model = NULL;
	PortName = NULL;
	RinexName = NULL;
	RtcmName = NULL;
        LogName = NULL;
	HZ = 1;

	// Process each option
	int i;
	const char* val;
	for (i=1; i<argc&& argv[i][0] == '-'; i++) {
		if      (Match(argv[i], "-raw=", RawName))        ;
		else if (Match(argv[i], "-rinex=", RinexName))    ;
		else if (Match(argv[i], "-rtcm=", RtcmName))      ;
                else if (Match(argv[i], "-log=", LogName))        ;
		else if (Match(argv[i], "-dgps=", DgpsName))      ;
		else if (Match(argv[i], "-x=", val))  InitialPos.x = atof(val);
		else if (Match(argv[i], "-y=", val))  InitialPos.y = atof(val);
		else if (Match(argv[i], "-z=", val))  InitialPos.z = atof(val);
		else if (Match(argv[i], "-debug=", val)) DebugLevel = atoi(val);
		else if (Match(argv[i], "-hz=", val))  HZ = atoi(val);
		else    return Error("Didn't recognize option %s\n", argv[i]);
	}
	
	if (argc-i < 2) 
		return Error("Must specify both Model and Port\n");

	Model = argv[i];
	PortName = argv[i+1];

	// Verify we have valid HZ. Must go evenly into one second.
	if ( HZ <= 0 ||  (100/HZ)*HZ != 100 )  return Error("%dHz is not valid\n", HZ);

	debug("Configure: RawName=%s Rinex=%s Rtcm=%s Receiver=%s port=%s\n",
		RawName, RinexName, RtcmName, Model, PortName);

	return OK;
}


void DisplayHelp()
{
	printf("\n");
	printf("Acquire [-raw=RawFile] [-rinex=RinexFile] [-rtcm=RtcmFile] [-hz=HZ] GpsModel  Port\n");
	printf("   Acquires Rinex data from a GPS receiver.\n");
	printf("\n");
	printf("   GpsModel - the model of the receiver\n");
	printf("              currently AC12, ANTARIS, GPS18, SIRF, ALLSTAR or LASSENIQ\n");
	printf("              along with RINEX and RTCM.\n");
	printf("   Port     - the name of the Rs-232 port to talk to the receiver\n");
	printf("               eg. \\com3, \\com16  or \\usb  or a 'raw' file \n");
	printf("   RawFile  - output file for raw gps data\n");
	printf("   RinexFile - output file for Rinex observation data\n");
	printf("   RtcmFile - output file for Rtcm data\n");
	printf("\n");
	printf("Note: the input ""port"" can actually be a data file.\n");
	printf("   Acquire can also be used to convert one data file to another\n");
	printf("   (for example, RINEX to RTCM, GarminRaw to RINEX, etc)\n");
	printf("   This is version '%s' built on %s %s\n", VERSION, __TIME__, __DATE__);

	printf("\n\n");


}




Rinex* NewRinex(const char* name, RawReceiver& gps)
{
	if (name == NULL) return NULL;
	if (gps.GetError() != OK) return NULL;
	Stream* s = NewOutputStream(name);
	if (s == NULL) return NULL;
	Rinex* r = new Rinex(*s, gps);
	if (r == NULL || r->GetError() != OK) return NULL;
	return r;
}

Rtcm3Station* NewRtcm(const char* name, RawReceiver& gps)
{
        Rtcm3Station::Attributes Station;
        Station.ARP = Position(0,0,0);
        Station.Id = 0;
	if (name == NULL) return NULL;
	if (gps.GetError() != OK) return NULL;
	Stream* s = NewOutputStream(name);
	if (s == NULL) return NULL;
	Rtcm3Station* r = new Rtcm3Station(*s, gps, Station);
	if (r == NULL || r->GetError() != OK) return NULL;
	return r;
}


SqliteLogger* NewLogger(const char* name, RawReceiver& gps)
{
    if (name == NULL) return NULL;
    if (gps.GetError() != OK) return NULL;
    SqliteLogger* logger = new SqliteLogger(name, gps, 1234);
    if (logger == NULL || logger->GetError() != OK) return NULL;
    return logger;
}


#ifdef NOTYET
DgpsStation* NewDgps(const char* name, RawReceiver& gps)
{
	if (name == NULL) return NULL;
	if (gps.GetError() != OK) return NULL;
	Stream* s = NewOutputStream(name);
	if (s == NULL) return NULL;
	DgpsStation* r = new DgpsStation(*s, gps);
	if (r == NULL || r->GetError() != OK) return NULL;
	return r;
}
#endif
	
