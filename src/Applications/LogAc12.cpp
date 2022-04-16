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

#include "SqliteLogger.h"
#include "Rtcm3Station.h"
#include "Rs232.h"
#include "RawAC12.h"
#include <stdio.h>

bool Configure(int argc, const char** argv);
void DisplayHelp();
bool GpsSession(const char *PortName, const char *LogName, int Port, const char* Mount);
void Display(RawReceiver& gps);

// Globals which are set up by "configure"
const char *User;
const char *Password;
const char *LogName;
int Port;
const char *Mount;
const char *SerialName;
Rtcm3Station::Attributes Station;
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

        // Start a GPS session
        printf("Starting Session\n");
        GpsSession(SerialName, LogName, Port, Mount);

        ShowErrors();
        ClearError();

        // Sleep a bit before restarting the session
        printf("Session Failed -- Restart in 15 seconds\n");
        Sleep(15000);
    }
    
    return 0;
}




bool GpsSession(const char *SerialName, const char *Logname, int Port, const char *Mount)
{
    // Initialize the gps receiver
    Rs232   in(SerialName);
    RawAC12 gps(in);
    if (gps.GetError() != OK)
        return Error("Unable to init the AC12  on port %s\n", SerialName);

    // Create the RTCM output file
    SqliteLogger  Log(LogName, gps, 1234);
    if (Log.GetError() != OK)
        return Error();

    // Repeat forever
    for (;;) {
        // Read next epoch of data
        if (gps.NextEpoch() != OK) return Error("Can't get gps data\n");

        Display(gps);

        // Write it out as RTCM
        if (Log.OutputEpoch() != OK) return Error("Can't write observations to Sqlite\n");
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
	// Set the defaults
        User="";
        Password="";
	Station.ARP = Position(0,0,0);
	SerialName = NULL;
        Port = 0;
        Mount = NULL;
        LogName = NULL;

	// Process each option
	int i;
	const char* val;
	for (i=1; i<argc; i++) {
		if      (Match(argv[i], "-caster=", LogName))      ;
                else if (Match(argv[i], "-log=", val)) LogName=val;
                else if (Match(argv[i], "-mount=", Mount)) ;
                else if (Match(argv[i], "-serial=", SerialName))      ;
		else if (Match(argv[i], "-x=", val))  Station.ARP.x = atof(val);
		else if (Match(argv[i], "-y=", val))  Station.ARP.y = atof(val);
		else if (Match(argv[i], "-z=", val))  Station.ARP.z = atof(val);
		else if (Match(argv[i], "-debug=", val)) DebugLevel = atoi(val);
                else if (Match(argv[i], "-user=", User))  ;
                else if (Match(argv[i], "-password=", Password))  ;
		else    return Error("Didn't recognize option %s\n", argv[i]);
	}
	

        if (SerialName == NULL ||  LogName == 0)
            return Error("Must specify SerialDevice, LogName\n");

	return OK;
}


void DisplayHelp()
{
	printf("\n");
	printf("NtripAc12 caster=LogName port=PortNr mount=MountPoint serial=SerialPort x=xxx y=yyyy z=zzz\n");
	printf("   Acquires rtcm data from an AC12 GPS receiver.\n");
	printf("\n");
	printf("   SerialDevice  - the name of the Rs-232 device to talk to the receiver\n");
	printf("               eg. /dev/ttyUSB0\n");
        printf("   LogName - the name or ip address of the NTRIP caster\n");
        printf("   x, y, z  are ECEF station coordinates\n");
	printf("\n");


}

