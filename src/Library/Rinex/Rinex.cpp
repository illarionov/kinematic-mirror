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


#include "Rinex.h"
int Snr2Level(double Snr);



Rinex::Rinex(Stream& out, RawReceiver& gps)
: Out(out), Gps(gps)
{
	ErrCode = Initialize();
}


bool Rinex::OutputEpoch()
{
	return WriteObservation();
}


Rinex::~Rinex()
{
}


bool Rinex::Initialize()
{
	debug("Rinex::Initialize \n");
	FirstEpoch = true;

	// Make sure the gps is OK;
	if (Gps.GetError() != OK || Out.GetError() != OK) return Error();

	for (int s=0; s<MaxSats; s++)
		PreviouslyValid[s] = false;

	return OK;
}


bool Rinex::WriteObservationHeader()
{
	// Get UTC time of the first epoch
	int32 year, month, day, hour, min, sec, nsec;
	TimeToDate(Gps.GpsTime, year, month, day);
	TimeToTod(Gps.GpsTime, hour, min, sec, nsec);
   
	Out.Printf("     2.10           OBSERVATION         "
		       "G (GPS)             RINEX VERSION / TYPE\n");
	Out.Printf("Kinematic           Precision-gps.org   "
		     "%02d-%3s-%04d         PGM / RUN BY / DATE\n",
				  day, MonthName[month], year);
	Out.Printf("Dummy antenna marker                    "
		       "                    MARKER NAME\n");
	Out.Printf("<name>              <agency>            " 
		       "                    OBSERVER / AGENCY\n");
	Out.Printf("                    %-20s"
		       "<version>           REC # / TYPE / VERS\n", Gps.Description);
	Out.Printf("                                        "
		       "                    ANT # / TYPE\n");
	Out.Printf("%14.4f%14.4f%14.4f"
		       "                  APPROX POSITION XYZ\n",
					Gps.Pos.x, Gps.Pos.y, Gps.Pos.z);
	Out.Printf("%14.4f%14.4f%14.4f"
		       "                  ANTENNA: DELTA H/E/N\n",
					0.0, 0.0, 0.0);
	Out.Printf("     1     0                            "
		       "                    WAVELENGTH FACT L1/2\n");
	Out.Printf("     4    C1    L1    S1    D1          "
		       "                    # / TYPES OF OBSERV\n");

	Out.Printf("%6d%6d%6d%6d%6d%5d.%06d "
		             "                 TIME OF FIRST OBS\n",
					year, month, day, hour, min, sec, nsec/1000);
	Out.Printf("                                        "
		          "                    END OF HEADER\n");

	return OK;
}


bool Rinex::WriteObservation()
{
    // If this is the first epoch, write out the header
	if (FirstEpoch)
		if (WriteObservationHeader() != OK) return Error();
	FirstEpoch = false;

	///////////////////////////////////
	// Print the DATA RECORD DESCRIPTOR
	////////////////////////////////////
	int i;

    // print the EPOCH in GPS time
	int32 year, month, day, hour, min, sec, nsec;
	TimeToDate(Gps.GpsTime, year, month, day);
	TimeToTod(Gps.GpsTime, hour, min, sec, nsec);
	Out.Printf(" %02d %02d %02d %02d %02d %2d.%07d",   
	            year%100, month, day, hour, min, sec, (nsec+50)/100);

	// print the EPOCH flag
	Out.Printf("  0");  // Assume OK for now

	// Get a list of satellites in current epoch
	int sats[MaxChannels];
    int count = 0;
	for (int s=0; s<MaxSats; s++)
		if (Gps.obs[s].Valid)
			sats[count++] = s;

	 // Print the first 12 satellites
     Out.Printf("%3d", count);
	 for (i=0; i<count && i<12; i++)
		 PrintSat(sats[i]); 
     
	 // fill in blanks for remaining satellites
	 for (i=count; i<12; i++)
		Out.Printf("   ");
     Out.Printf("            ");   // TODO - show adjustment from raw measurements

	 // if there are more than 12, use continuation lines
	 for (i=12; i<count; i++) {
		 if ( (i%12) == 0)
			 Out.Printf("\n                                ");  // 32 spaces
	     PrintSat(sats[i]);
	 }

	 // Print the final end of line.
	 Out.Printf("\n");

	 //////////////////////////////////////////////////////
	 // Print the Observation for each satellite
	 //////////////////////////////////////////////////////

	 // Do for each satellite
	 for (int s=0; s<MaxSats; s++) {
         RawObservation& o = Gps.obs[s];
		 if (!o.Valid) continue;

		 bool Slip = o.Slip || !PreviouslyValid[s];

		 // if we are going to overflow, then reset
		 if (o.Phase+PhaseAdjust[s] > 999999999.999 || 
			 o.Phase+PhaseAdjust[s] < -999999999.999)
			 Slip = true;

		 // if it slipped or wasn't valid earlier, then reset the phase
		 if (Slip)
			 if (o.Phase == 0)   PhaseAdjust[s] = 0;
			 else                PhaseAdjust[s] = round(o.PR/L1WaveLength - o.Phase);		 

		 debug("Rinex - s=%d  PR=%.3f  Phase=%.3f  PhaseAdjust=%.3f  delta(m)=%.3f Slip=%d\n",
		 s, o.PR, o.Phase, PhaseAdjust[s], o.PR-(o.Phase+PhaseAdjust[s])*L1WaveLength, Slip);

		 char snr = " 123456789"[SnrToLevel(o.SNR)];

	     // Print the L1 Coarse Code range
         Out.Printf("%14.3f %c", o.PR, snr);

		 // Print the L1 Phase information
		 if (o.Phase == 0)  Out.Printf("                ");
		 else Out.Printf("%14.3f%s%c", o.Phase+PhaseAdjust[s], Slip?"1":" ", snr);

		 // Print the S1 SNR
		 if (o.SNR == 0)  Out.Printf("                ");
		 else             Out.Printf("%14.3f %c", o.SNR, snr);

		 if (o.Doppler == 0) Out.Printf("                ");
		 else                Out.Printf("%14.3f %c", o.Doppler, snr);

		 // Print 



		 // End of line
		 Out.Printf("\n");
	 }

	 // Remember which satellites were valid
	 for (int s=0; s<MaxSats; s++)
		 PreviouslyValid[s] = Gps.obs[s].Valid;

	return OK;
}



int Snr2Level(double Snr)
{
	return (int32)( (Snr-30)/3+1, 9.0);
}



bool Rinex::PrintSat(int s)
{
	int svid = SatToSvid(s);
	if (svid <= 32)
		Out.Printf("G%02d", svid);
	else if (120 <= svid && svid <= 152)
		Out.Printf("S%02d", svid-100);
	else
		Out.Printf("###");

	return OK;
}

