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

#include "NtripClient.h"
#include "RawRtcm3.h"
#include "SqliteLogger.h"
#include <stdio.h>

bool Configure(int argc, const char** argv);
void DisplayHelp();
bool LoggerSession();
void Display(RawReceiver& gps);

// Globals which are set up by "configure"
const char *User;
const char *Password;
const char *CasterName;
const char *Port;
const char *Mount;
const char *LogName;
int StationId;
extern int DebugLevel;


int main(int argc, const char** argv)
{
    // Display output immediately
    setlinebuf(stdout);

    // Get configured according to arguments
    if (Configure(argc, argv) != OK) {
        DisplayHelp();
        return ShowErrors();
    }

    // Repeat forever
    for (;;) {

        // Start a session acquiring data
        printf("Starting Logger Session\n");
        LoggerSession();

        ShowErrors();
        ClearError();

        // Sleep a bit before restarting the session
        printf("Session Failed -- Restart in 15 seconds\n");
        Sleep(15000);
    }
    
    return 0;
}




bool LoggerSession()
{
    debug("LoggerSession: starting\n");
    // Initialize the gps receiver
    NtripClient   in(CasterName, Port, Mount, User, Password);
    RawRtcm3 gps(in);
    if (gps.GetError() != OK)
        return Error("Unable to read RTCM3.1 data from %s:%s/%s:%s:%s\n", 
                      CasterName, Port, Mount, User, Password);

    // Open the logger database
    SqliteLogger log(LogName, gps, StationId);
    if (log.GetError() != OK)
        return Error("Can't initialize the NTRIP stream\n");

    // Repeat forever
    for (;;) {
        // Read next epoch of data
        if (gps.NextEpoch() != OK) return Error("Can't get gps data\n");

        Display(gps);

        // Write it to the log
        if (log.OutputEpoch() != OK) 
           return Error("Can't write gps data to log\n");
    }

    // Done
    return OK;
}



void Display(RawReceiver& gps)
{
    // Display the satellites being tracked
    int32 day, month, year, hour, min, sec, nsec;
    TimeToDate(gps.GpsTime, year, month, day); 
    TimeToTod(gps.GpsTime, hour, min, sec, nsec);
    printf("%2d/%02d/%04d %02d:%02d:%02d  ", month,day,year,hour,min,sec);
    for (int s=0; s<MaxSats; s++) {
        if (gps.obs[s].Valid)
            if (gps[s].Valid(gps.GpsTime)) printf("*%d ",SatToSvid(s));
            else                            printf("%d ", SatToSvid(s));
    }
    
    printf("\07\n");  // Ring the bell so we know things are alive
}


bool Configure(int argc, const char** argv)
{
        debug("Configure: starting out\n");
	// Set the defaults
        User="";
        Password="";
        Port = "2101";
        Mount = 0;
        CasterName = "localhost";
        LogName = "log.sqlite";
        StationId = 0;   // should come from Gps??

	// Process each option
	int i;
	const char* val;
	for (i=1; i<argc; i++) {
                debug("Configure: argv[%d]=%s\n", i, argv[i]);
		if      (Match(argv[i], "-caster=", CasterName))      ;
                else if (Match(argv[i], "-port=", Port)) ;
                else if (Match(argv[i], "-mount=", Mount)) ;
		else if (Match(argv[i], "-debug=", val)) DebugLevel = atoi(val);
                else if (Match(argv[i], "-user=", User))  ;
                else if (Match(argv[i], "-password=", Password))  ;
                else if (Match(argv[i], "-log=", LogName)) ;
		else    return Error("Didn't recognize option %s\n", argv[i]);
	}
	

        if (Mount == 0)
            return Error("Must specify at least and -mount=yy\n");

	return OK;
}


void DisplayHelp()
{
        debug("DisplayHelp:\n");
	printf("\n");
	printf("NtripAc12 <config options>\n");
	printf("   Acquires rtcm data from an AC12 GPS receiver.\n");
	printf("\n");
	printf("   -serial=SerialDevice  - name of device to access GPS\n");
	printf("               eg. /dev/ttyUSB0\n");
        printf("   -id=StationID - RTCM station id of GPS\n");
        printf("   -x=xxx, -y=-yyy, -z=zzz  ECEF antenna coordinates\n");
        printf("   -caster=CasterName - name or ip address of NTRIP caster\n");
        printf("   -port=TcpPortNr - tcp port number of NTRIP caster (2101)\n");
        printf("   -mnt=MountPoint - NTRIP mount point\n");
        printf("   -debug=n  Debug level, 0=none ... 9=lots\n");
	printf("\n");


}

