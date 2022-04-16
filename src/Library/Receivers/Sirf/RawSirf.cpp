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


// Background on Sirf Messages
//
// General parsing of messages
//   Sirf messages are "big endian", with the exception of 64 bit values
//   which have the two 32-bit halves swapped. This is typical of the "ARM"
//   microprocessor used internally in Sirf receivers. There is some confusion
//   in the documentation however, and there may be versions of Sirf which 
//   do not swap the 32-bit halves.
//
// The RawSirf interface uses the following Sirf output messages
//   Navlib messages.
//     Contain pseudorange and carrier phase for one satellite,
//     The time information is uncorrrected and reflects clock drift.
//     The doppler is reported in the "freq" field. Oddly, "freq" is in 
//        meters, and you have to divide by wavelength to get Hz.
//        This value is not corrected for clock drift,
//        so you need to subtract clock drift to get the actual doppler value.
//     On the Sirf III, carrier phase and timetag are always set to zero.
//  Navigation message
//     Contain X,Y,Z position data, as well as status information
//     The position data is only used as an inital estimate, but the status 
//     information ensures we only collect data once the receiver has a 
//     good fix.
//  Clock message
//     Contains the epoch time as well as the corrected clock information.
//     In general, clocks don't have to be accurate for double-differencing,
//     but with Sirf the clock drift is needed to calculate the Doppler. 
//     Drift is only reported to the nearest nsec which limits the accuracy of 
//     the doppler values. The errors may be significant enough to prevent you
//     from using integrated doppler as an alternative to carrier phase on 
//     the Sirf III.
//  50bps messages
//     These are the raw ephemeris messages sent by the satellites. 
//     They contain the same data as Sirf's "ephemeris" message,
//     but the "epemeris" message didn't work on the Sirf III.
//
// According to documentation for the "initialize data source" command,
//    Sirf will output the above messages when Sirf is placed into "Navlib"
//    mode. With the Sirf III, we enable the messages explicitly and don't
//    bother with Navlib mode.
//
//
// With each epoch, messages are sent in the following order:
//    - Navlib messages.  On the Sirf III, the time tags are zero and the tow
//                        is the same for each satellite in the epoch.
//    - Navigation Message
//    - Clock
//
// If Sirf isn't calculating positions, the Navigation and Clock messages
//     may or may not be present.
//
// Sirf III does not send Raw measurements, nor does it reply with
//    firmware version.

#include "RawSirf.h"

// Message types - protocol
static const int CMDACK = 11;
static const int CMDNAK = 12;

// Sent message types
static const int INITIALIZE = 128;
static const int POLLVERSION = 132;
static const int SETSBAS = 170;
static const int DGPSSOURCE = 133;
static const int MSGRATE = 166;

// Received message types
static const int NAVIGATION = 2;
static const int CLOCK = 7;
static const int SUBFRAME = 8;
static const int NAVLIB = 28;


// Note: Sirf III (revision ???) Doesn't seem to support the "Poll Ephemeris"
//  command. Consequently, we enable the 50Bps navigation messages as a source
//  of ephemeris data.


RawSirf::RawSirf(Stream& s)
   :comm(s)
{
	ErrCode = Initialize();
}

RawSirf::~RawSirf()
{
}


bool RawSirf::NextEpoch()
{
	// Sequence of messages is:
	//    Navlib messages (raw measurements)
	//    if tracking,
	//         Position
	//         Clock

	// Epoch ends when we read a clock record with a valid solution
	SolutionFound = false;
	do {
		// Read a message from the GPS
		Block b;
		if (comm.GetBlock(b) != OK) return Error();

		// Process according to the type of message
		if      (b.Id == NAVIGATION) ProcessNavigation(b);
		else if (b.Id == CLOCK)      ProcessClock(b);
		else if (b.Id == SUBFRAME)   Process50Bps(b);
		else if (b.Id == NAVLIB)     ProcessNavlibMeasurement(b);
		else                       debug("Sirf Got Message id %d\n", b.Id);

	} while (!SolutionFound);

	debug("RawSirf::NextEpoch (done) GpsTime=%.3f\n", S(GpsTime));

	return OK;
}




bool RawSirf::ProcessNavigation(Block& block)
{
	ArmEndian b(block);
	int x = b.Get4();
	int y = b.Get4();
	int z = b.Get4();
	int xv = b.Get2();
	int yv = b.Get2();
	int zv = b.Get2();
	byte mode1 = b.Get();
	double HDOP = b.Get() / 5.0;
	byte mode2 = b.Get();
	int WN = b.Get2();
	int Tag = b.Get4();   // Epoch's Tow, expressed in 1/100 seconds.
	debug("Sirf Position: TimeTag=%d x=%d  y=%d  z=%d mode1=0x%x  mode2=0x%x\n", 
		Tag, x, y, z, mode1, mode2);

	// When everything is tracking, we'll see a PMODE of 4
	if ((mode1&7)!=4) return OK;

	// We expect everything to be on a 1 second boundary
	if ( (Tag%100) != 0) Event("Sirf Position: %.2f is not an even second\n", Tag/100.0);

    // Save our position and time
	Pos = Position(x, y, z);
	GpsTime = ConvertGpsTime(WN, Tag/100.0);

	return OK;
}


bool RawSirf::ProcessVersion(Block& b)
{
	b.Data[b.Length] = '\0';
	debug("Sirf ProcessVersion: %s\n", b.Data);
	return OK;
}

bool RawSirf::ProcessClock(Block& block)
{
	ArmEndian b(block);
	uint16 Week = b.Get2();
	uint32 TOW =  b.Get4();
	byte SVs = b.Get();
	int32 ClockDrift = b.Get4();   // millisec
	int32 ClockBias = b.Get4();    // nsec
	uint32 EstGpsTime = b.Get4();  // millisec

	debug("Sirf Clock: Week=%d TOW=%d SVs=%d ClockDrift=%d ClockBias=%d Time=%d\n",
		Week, TOW, SVs, ClockDrift, ClockBias, EstGpsTime);

	SolutionFound = SVs >= 4;
	if (!SolutionFound) return OK;

	// Get the raw time of week
	double Tow = TOW/100.0 + ClockBias / 1000000000.0;

	// Adjust Doppler to account for clock drift.
	// Clock drift is to the nearest nsec, so we may want to calculate
	//  clock drift ourselves using the Tow from the Navlib message.
	for (int s=0; s<MaxSats; s++)
		if (obs[s].Valid)
			obs[s].Doppler += ClockDrift;

	// Adjust the measurements to the GpsTime
	AdjustToHz(GpsTime, Tow);

	return OK;
}


bool RawSirf::Process50Bps(Block& block)
{
	ArmEndian b(block);
	int channel = b.Get();
	int svid = b.Get();

	// Point to the satellite and it's associated data structures
	int Sat = SvidToSat(svid);
	EphemerisXmit& ephemeris = *(EphemerisXmit*)eph[Sat];
	NavFrameBuilder& frame = Frame[Sat];

	// The data buffer contains 10 words of NavFrame data.
	//    The upper two bits of each word
	//    contain the previous D29 and D30 parity bits.

	// Peek ahead into the buffer and get the subframe number.
	int SubFrameNr = block.Data[8]&7;
	if ( (block.Data[6]&0x40) != 0)	 SubFrameNr = 7 - SubFrameNr;

	debug("Sirf Processing subframe %d for sat=%d\n", SubFrameNr, Sat);
	block.Display();
	if (SubFrameNr == 0 || SubFrameNr > 5) return OK;  // Not quite tracking yet

	// Do for each word of the subframe
	for (int i=0; i<10; i++) {
		uint32 word = b.Get4();

		// figure out the starting bit number of this word
		int BitNr = (SubFrameNr-1)*300 + i*30 + 1;

		// Add the word to the frame
		frame.AddWord(word, BitNr);
	}

	// If the frame is complete, update the ephemeris
	if (frame.Complete()) 
		ephemeris.AddFrame(frame);

	return OK;
}

bool RawSirf::ProcessNavlibMeasurement(Block& block)
{
	ArmEndian b(block);
	int Channel = b.Get();
	int Tag = b.Get4();   // always zero for GSW3.1.0 (sirf III)
	int svid = b.Get();
	int Sat = SvidToSat(svid);
	double Tow = b.GetDouble();
	double PR = b.GetDouble();
	float freq = b.GetFloat();  // Delta phase in nsec.
	double Phase = b.GetDouble();
	int TimeInTrack = b.Get2();
	int SyncFlags = b.Get();
	byte c[10];
	for (int i=0; i<10; i++)
		c[i] = b.Get();
	uint16 DeltaRangeInterval = b.Get2();
	uint16 DeltaRangeTime = b.Get2();
	int16 ExtrapolationTime = b.Get2();
	uint8 PhaseErrorCount = b.Get();
	uint8 LowPowerCount = b.Get();

	debug("Sirf NavlibMeasurement: svid=%d TimeTag=%d PR=%.3f Tow=%.9f Phase=%g\n",
		svid, Tag, PR, Tow, Phase);
	debug("     freq=%g TimeInTrack=%d  SyncFlags=0x%02x, DeltaRangeInterval=%d\n",
		freq, TimeInTrack, SyncFlags, DeltaRangeInterval);
	debug("     DeltaRangeTime=%d ExtrapolationTime=%d PhaseErrorCount=%d LowPowerCount=%d\n",
		        DeltaRangeTime,   ExtrapolationTime,   PhaseErrorCount,   LowPowerCount);

	// If not synced, then skip this data
	if ( (SyncFlags&1) == 0) return OK;

	// Figure out the signal to noise ratio (minimum of the C values)
	double SNR = c[0];
	for (int i=1; i<10; i++)
		if (c[i] < SNR)
			SNR = c[i];

	// If the measurement time has changed, then clear out the old measurements
	if (Tow != MeasurementTow)
		for (int s=0; s<MaxSats; s++)
			obs[s].Valid = false;
	PreviousTow = Tow;
	MeasurementTow = Tow;

	// Keep the observation
	obs[Sat].PR = PR;
	obs[Sat].Phase = Phase;
	obs[Sat].Doppler = -freq / L1WaveLength;  // convert to delta Hz
	obs[Sat].SNR = SNR;
	obs[Sat].Valid = true;
	obs[Sat].Slip = PhaseErrorCount != PreviousPhaseErrorCount[Sat];

	PreviousPhaseErrorCount[Sat] = PhaseErrorCount;

	return OK;
}


bool RawSirf::Initialize()
{
        debug("RawSirf::Initialize\n");
	strcpy(Description, "Sirf");

	// Create an ephemeris for each satellite
	for (int s=0; s<MaxSats; s++)
		eph[s] = new EphemerisXmit(s, "Sirf Star III");

	// Make sure communications are OK;
	if (comm.GetError() != OK) return Error();
	if (comm.ReadOnly()) return OK;
        debug("RawSirf::Initialize - Writable port\n");

	// Do a warm reset. 
	if (WarmReset() != OK)   return Error();

        // Request the firmware version
        if (RequestVersion() != OK) return Error();

        // Turn off messages we don't use
        if (DisableMsg(0x08) != OK) return Error();
        if (DisableMsg(0x04) != OK) return Error();
        if (DisableMsg(0x0d) != OK) return Error();
        if (DisableMsg(0x09) != OK) return Error();
        if (DisableMsg(0x29) != OK) return Error();
        if (DisableMsg(0x1b) != OK) return Error();

	// turn on the raw messages (needed for Sirf III?)
	if (EnableMsg(NAVLIB) != OK) return Error();     // Pseudoranges
	if (EnableMsg(NAVIGATION) != 0) return Error();  // PVT
	if (EnableMsg(CLOCK) != 0) return Error();       // Clock
	if (EnableMsg(SUBFRAME) != 0) return Error();    // 50 BPS 

	// Enable SBAS (why not?)
	if (EnableSBAS() != OK) return Error();


	return OK;
}

bool RawSirf::WarmReset()
{
    debug("RawSirf::WarmReset\n");
	// Warm Reset 
    return comm.PutBlock (INITIALIZE, 0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0, 
		                0,0,0,0,  0,0, 0, -1);  // X,Y,Z,Clock,Tow,Wn,Mask
}

bool RawSirf::RequestVersion()
{
	// Request the software version number
	if (comm.PutBlock(POLLVERSION,0,-1) != OK) return Error();
	if (AckOrNak(POLLVERSION) != OK) return Error();

	return OK;
}

bool RawSirf::EnableMsg(int id)
{
        debug("RawSirf::EnableMsg - id=%d\n", id);
	// Turn on the specified message every second
	if (comm.PutBlock(MSGRATE, 0, id, 1, 0,0,0,0, -1) != OK) return Error();
	if (AckOrNak(MSGRATE) != OK) return Error();
	return OK;
}
bool RawSirf::DisableMsg(int id)
{
        debug("RawSirf::EnableMsg - id=%d\n", id);
	// Turn on the specified message every second
	if (comm.PutBlock(MSGRATE, 0, id, 0, 0,0,0,0, -1) != OK) return Error();
	if (AckOrNak(MSGRATE) != OK) return Error();
	return OK;
}

bool RawSirf::EnableSBAS()
{
	// Set the SBAS mode to automatically select from good satellites
	if (comm.PutBlock(SETSBAS, 0, 1, 0, -1) != OK) return Error();
	if (AckOrNak(SETSBAS) != OK) return Error();

	// Set the DGPS to use SBAS
	if (comm.PutBlock(DGPSSOURCE, 1, 0,0,0,0, 0, -1) != OK) return Error();
	if (AckOrNak(DGPSSOURCE) != OK) return Error();

	return OK;
}

bool RawSirf::AckOrNak(int Id)
{
	return OK;

	// Repeat until we get an Ack or a Nak
	for (int i=0; i<20; i++) {
		Block b;
		if (comm.GetBlock(b) != OK) return Error();

		if (b.Id == CMDACK && b.Data[0] == Id) 
			return OK;
		if (b.Id == CMDNAK && b.Data[0] == Id) 
			return Error("Command %d was Nak'd by Sirf receiver\n", Id);
	}

	return Error("Command %d was ignored by Sirf receiver\n", Id);
}



