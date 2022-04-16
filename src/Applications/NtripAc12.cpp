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

#include "NtripServer.h"
#include "Rtcm3Station.h"
#include "Rs232.h"
#include "RawAC12.h"
#include <stdio.h>

bool Configure(int argc, const char** argv);
void DisplayHelp();
bool GpsSession();
void Display(RawReceiver& gps);

// Globals which are set up by "configure"
const char *User;
const char *Password;
const char *CasterName;
const char *Port;
const char *Mount;
const char *SerialName;
Rtcm3Station::Attributes attr;
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
        GpsSession();

        ShowErrors();
        ClearError();

        // Sleep a bit before restarting the session
        printf("Session Failed -- Restart in 15 seconds\n");
        Sleep(15000);
    }
    
    return 0;
}




bool GpsSession()
{
    debug("GpsSession: starting\n");
    // Initialize the gps receiver
    Rs232   in(SerialName);
    RawAC12 gps(in);
    if (gps.GetError() != OK)
        return Error("Unable to init the AC12  on port %s\n", SerialName);

    // Create the RTCM output file
    //OutputFile out(RtcmName);
    NtripServer out(CasterName, Port, Mount, User, Password);
    if (out.GetError() != OK)
        return Error("Can't initialize the NTRIP stream\n");
    Rtcm3Station rtcm(out, gps, attr);
    if (rtcm.GetError() != OK) 
        return Error();

    // Repeat forever
    for (;;) {
        // Read next epoch of data
        if (gps.NextEpoch() != OK) return Error("Can't get gps data\n");

        Display(gps);

        // Write it out as RTCM
        if (rtcm.OutputEpoch() != OK) 
           return Error("Can't send RTCM to caster\n");
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
	attr.ARP = Position(0,0,0);
        attr.Id = 0;
	SerialName = 0;
        Port = "2101";
        Mount = 0;
        CasterName = "localhost";

	// Process each option
	int i;
	const char* val;
	for (i=1; i<argc; i++) {
                debug("Configure: argv[%d]=%s\n", i, argv[i]);
		if      (Match(argv[i], "-caster=", CasterName))      ;
                else if (Match(argv[i], "-port=", Port)) ;
                else if (Match(argv[i], "-mount=", Mount)) ;
                else if (Match(argv[i], "-serial=", SerialName))      ;
		else if (Match(argv[i], "-x=", val))  attr.ARP.x = atof(val);
		else if (Match(argv[i], "-y=", val))  attr.ARP.y = atof(val);
		else if (Match(argv[i], "-z=", val))  attr.ARP.z = atof(val);
		else if (Match(argv[i], "-debug=", val)) DebugLevel = atoi(val);
                else if (Match(argv[i], "-user=", User))  ;
                else if (Match(argv[i], "-password=", Password))  ;
                else if (Match(argv[i], "-stationid=", val)) attr.Id=atoi(val);
		else    return Error("Didn't recognize option %s\n", argv[i]);
	}
	

        if (SerialName == 0 || Mount == 0)
            return Error("Must specify at least -serial=xx and -mount=yy\n");

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

