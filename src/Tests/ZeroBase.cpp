// Kinematic - a program for doing double difference, kinematic positioning
//    Part of kinematic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    kinematic@coyotebush.net
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

#include "NewRawReceiver.h"
#include "SP3.h"
#include <stdio.h>
#include <stdarg.h>

bool DoubleNextEpoch(RawReceiver& base, RawReceiver& rover);
bool ZeroDiff(RawObservation b[MaxSats], RawObservation r[MaxSats]);
bool ShortBaseDiff(RawReceiver* base, RawReceiver* roving, Ephemerides* eph);
double StdDev(int n, double sum, double sumsq);

bool ZeroBase(int argc, const char** argv);
bool Configure(int argc, const char** argv);
bool DisplayOptions();


// run string parameters
const char* BaseModel;
const char* BasePortName;
const char* RovingModel;
const char* RovingPortName;
const char* Sp3File;


struct SatInfo {
    int Slip;
    int Ambiguity;

    int CountPhase;
    double SumPhase;
    double SumSqPhase;

    int CountPr;
    double SumPr;
    double SumSqPr;

    SatInfo() {Slip=Ambiguity=CountPhase=SumPhase=SumSqPhase=
                CountPr=SumPr=SumSqPr  = 0;}

};

SatInfo Sat[MaxSats];



int main(int argc, const char** argv)
{
	ZeroBase(argc, argv);
	//ShowErrors();
	return 0;
}

bool ZeroBase(int argc, const char** argv) 
{
	// parse the command line
	if (Configure(argc, argv) != OK) {
		DisplayOptions();
		return Error();
	}

	// Open up the two sources of raw measurements
	RawReceiver* base = NewRawReceiver(BaseModel, BasePortName);
	RawReceiver* roving = NewRawReceiver(RovingModel, RovingPortName);
	if (base == NULL || roving == NULL) return Error();

	// Read the first epoch so we have initial position estimates
	if (Range(base->Pos) == 0 && base->NextEpoch() != OK) 
            return Error("Can't read first epoch from base\n");
	if (Range(roving->Pos) == 0 && roving->NextEpoch() != OK) 
            return Error("Can't read first epoch from rover\n");
            
    // Remember the initial positions         
    Position BasePos = base->Pos;
    Position RovingPos = roving->Pos;        
            
    // Read the ephemerides if appropriate
    Ephemerides *eph;
    if (Sp3File == NULL)  eph = NULL;
    else eph = new SP3(Sp3File);        

	// do for each epoch until "done"
	while (DoubleNextEpoch(*base, *roving) == OK) {
		base->Pos = BasePos; roving->Pos = RovingPos;
	    if (ShortBaseDiff(base, roving, eph))
                break;
	}
		

    printf("\nSat      Phase  Dev       PR    Dev       Count Slip\n");
    for (int s=0; s<MaxSats; s++) {
        if (Sat[s].CountPhase > 0) 
            printf("%3d    %6.3f %6.3f    %6.3f %6.3f    %6d %6d\n", s,
                 Sat[s].SumPhase/Sat[s].CountPhase * L1WaveLength,
                 StdDev(Sat[s].CountPhase, Sat[s].SumPhase, Sat[s].SumSqPhase)*L1WaveLength,
                 Sat[s].SumPr/Sat[s].CountPr,
                 StdDev(Sat[s].CountPr, Sat[s].SumPr, Sat[s].SumSqPr),
                 Sat[s].CountPhase, Sat[s].Slip);
        }

	return 0;
}



bool ShortBaseDiff(RawReceiver* Base, RawReceiver* Roving, Ephemerides* eph)
{
	RawObservation* b = Base->obs;
	RawObservation* r = Roving->obs;
	Time GpsTime = Base->GpsTime;
	Ephemerides& e = *eph;

    // Figure out which satellites have valid data
    bool Valid[MaxSats];
    for (int s=0; s<MaxSats; s++) {
        Valid[s] = b[s].Valid && r[s].Valid
                  && b[s].Phase != 0 && r[s].Phase != 0
                  && b[s].PR != 0 && r[s].PR != 0
                  && (eph == NULL || e[s].Valid(GpsTime))
                  /* && b[s].SNR >50 && r[s].SNR > 50 */;
        if (!Valid[s] || b[s].Slip || r[s].Slip)
            Sat[s].Ambiguity = 0;
    }
    
    // Get the satellite positions and check elevation
    Position SatPos[MaxSats]; double dummy;
    if (eph != NULL)
    for (int s=0; s<MaxSats; s++) {
    	if (!Valid[s]) continue;
    	if (e[s].SatPos(Base->GpsTime, SatPos[s], dummy) != OK) return Error();
        SatPos[s] = RotateEarth(SatPos[s], -Range(SatPos[s]-Base->Pos)/C);
    	double SinElev = ((SatPos[s]-Base->Pos) * Base->Pos) 
    	    / Range(SatPos[s]-Base->Pos) / Range(Base->Pos);
    	debug("s=%d  SinElev=%.3f  sin(15)=%.3f\n", s, SinElev, sin(DegToRad(15)));    
   
    	Valid[s] = SinElev > sin(DegToRad(15));
    }  
   
    // Calculate the difference in distance to the satellites
    double DiffDistance[MaxSats];
    for (int s=0; s<MaxSats; s++) {
    	if (!Valid[s] || eph == NULL)
    	    DiffDistance[s] = 0;
    	else    
    	    DiffDistance[s] = Range(SatPos[s] - Roving->Pos) - Range(SatPos[s] - Base->Pos);
    }

    // Calculate the average PR clock error (Meters)
    double SumClock = 0; int CntClock = 0;
    for (int s=0; s<MaxSats; s++) {
        if (!Valid[s]) continue;
        double ClockError = (r[s].PR - b[s].PR) - DiffDistance[s];
        SumClock += ClockError;
        CntClock++;
    }
    double PrClockError = SumClock / CntClock; // TODO: Divide by zero check

    // Using the existing ambiguity variables, calculate the average Phase Clock Error (Hz)
    SumClock = 0; CntClock = 0;
    for (int s=0; s<MaxSats; s++) {
        if (!Valid[s] || Sat[s].Ambiguity == 0) continue;
        double ClockError = (r[s].Phase - b[s].Phase - Sat[s].Ambiguity) - DiffDistance[s]/L1WaveLength;
        SumClock += ClockError;
        CntClock++;
    }
    double PhaseClockError;
    if (CntClock == 0) PhaseClockError = PrClockError;
    else               PhaseClockError = SumClock / CntClock;
    
    // Look for phase slips
    for (int s=0; s<MaxSats; s++) {
    	if (!Valid[s] || Sat[s].Ambiguity == 0) continue;
    	if (abs((r[s].Phase - b[s].Phase - Sat[s].Ambiguity) - DiffDistance[s]/L1WaveLength - PhaseClockError)  > 0.5){
            Sat[s].Ambiguity = 0;
            Sat[s].Slip++;
    	}
    }

    // Update the new ambiguity variables, if any
    for (int s=0; s<MaxSats; s++) {
        if (!Valid[s]) continue;
        if (Sat[s].Ambiguity == 0)
            Sat[s].Ambiguity = round((r[s].Phase - b[s].Phase - PhaseClockError) - DiffDistance[s]/L1WaveLength);
    }

    // Recalculate the phase clock error using new ambiguity variables (if any)
    CntClock = 0; SumClock = 0;
    for (int s=0; s<MaxSats; s++) {
        if (!Valid[s]) continue;
        double ClockError = (r[s].Phase - b[s].Phase - Sat[s].Ambiguity) - DiffDistance[s]/L1WaveLength;
        SumClock += ClockError;
        CntClock++;
    }
    PhaseClockError = SumClock / CntClock;

    // Display the measurement errors
    for (int s=0; s<MaxSats; s++) {
        if (!Valid[s]) continue;

        double Phase = (r[s].Phase - b[s].Phase - Sat[s].Ambiguity) - DiffDistance[s]/L1WaveLength - PhaseClockError;
        double Pr = r[s].PR - b[s].PR - DiffDistance[s] - PrClockError;          
    
        Sat[s].CountPr++; Sat[s].SumPr+=Pr; Sat[s].SumSqPr+=Pr*Pr;
        Sat[s].CountPhase++; Sat[s].SumPhase+=Phase; Sat[s].SumSqPhase+=Phase*Phase;

        printf("S%d:%.3f:%.3f   ", s, Phase, Pr);
    } 
    printf("\n");


    return OK;
}



 bool Configure(int argc, const char** argv)
 {
     // defaults

	 // Do for each argument
	 const char* arg;
	 int i;
	 for (i=1; i<argc && argv[i][0] == '-'; i++) {

		 if (Match(argv[i], "-debug=", arg))          DebugLevel = atoi(arg);
		 else if (Match(argv[i], "-sp3=", arg))       Sp3File = arg;
		 
		 else    return Error("Didn't recognize option %s\n", argv[i]);
	 }

	 if (argc-i < 4)
		 return Error("Need to specify: BaseModel BaseFile RovingModel RovingFile\n");

	 BaseModel = argv[i];
	 BasePortName = argv[i+1];
	 RovingModel = argv[i+2];
	 RovingPortName = argv[i+3];

	 return OK;
 }


 bool DisplayOptions()
 {
	 printf("\n");
     printf("ZeroBase [options] BaseModel BaseFile RovingModel RovingFile\n");
	 printf("     Double difference postprocessor for GPS data\n");
	 printf("\n");
	 printf("        BaseModel - the type of gps (or data) for the base receiver\n");
	 printf("        BaseFile  - the name of base receiver's data file\n");
	 printf("        RovingModel - type of gps (or data) for the rover\n");
	 printf("        RovingFile - the name of the roving receiver's data file\n");
     printf("\n");
	 printf("    The following ""models"" are supported\n");
	 printf("        RINEX      - Rinex V2.3\n");
	 printf("        XENIR      - Rinex, but with phase reversed\n");
	 printf("        RTCM       - Rtcm104 (RTK) messages xx xx xx\n");
	 printf("        <receiver> - Raw data stream from a gps receiver\n");
	 printf("                     (AC12, ANTARIS, SIRF, LASSENIQ, ALLSTAR, GPS18)\n");
	 printf("\n");    
	 printf("    Where {options} include any of the following:\n");
	 printf("        -static    - the roving receiver is standing still\n");
	 printf("        -codeonly  - do the calculation without carrier phase\n");
     printf("        -sp3=ephfile  - use precise ephemerides from ""file""\n");
	 printf("                     (otherwise, use base receiver's broadcast eph if avail)\n");
	 printf("        -enu=outputfile  - output ENU from Base\n");
	 printf("        -ecef=outputfile - output ECEF (XYZ)\n");
	 printf("        -wgs84=outputfile - output Lat/Lon/Alt (default)\n");
	 printf("        -test=outputfile  - output ENU relative to initial rover position\n");
	 printf("        -commas          - output is comma separated\n");
     printf("    This is version '%s' built on %s %s\n", VERSION, __TIME__, __DATE__);
	 printf("\n");
	 return OK;
 }

bool DoubleNextEpoch(RawReceiver& base, RawReceiver& rover)
{
	// Assume the satellites haven't slipped until proven otherwise
	bool Slip[MaxSats];
	for (int s=0; s<MaxSats; s++)
		Slip[s] = false;

	// repeat ... 
	do {
		// Decide which receiver is lagging behind
		RawReceiver& lagging = (base.GpsTime < rover.GpsTime)? 
                                   base: rover;
		// Advance the lagging receiver
		if (lagging.NextEpoch() != OK) return Error();

		// Update the slip status
		for (int s=0; s<MaxSats; s++)
			Slip[s] |= !lagging.obs[s].Valid 
			        || lagging.obs[s].Slip 
					|| lagging.obs[s].Phase == 0;

    // ... until the epochs match
	} while (base.GpsTime != rover.GpsTime);

	// Mark the receivers as slipped
	for (int s=0; s<MaxSats; s++)
		base.obs[s].Slip = rover.obs[s].Slip = Slip[s];
	
	return OK;
}



double StdDev(int n, double sum, double sumsq)
{
	return sqrt(   (n*sumsq - sum*sum) / n / (n-1)  );
}

