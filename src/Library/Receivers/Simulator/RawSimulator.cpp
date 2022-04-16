#include "RawSimulator.h"

RawSimulator::RawSimulator(RawReceiver& rcv, Ephemerides& e, bool stationary)
: gps(rcv), ephemerides(e), Static(stationary)
{
	strcpy(Description, "Simulated");
	ErrCode = gps.GetError();

	// use the same ephemerides
	for (int s=0; s<MaxSats; s++)
		eph[s] = ephemerides.eph[s];

	// Read the first epoch because we need a position
	gps.NextEpoch();
	Pos = gps.Pos;
	GpsTime = 0;

	// Round position off to nearest meter in each direction. (debugging assistance)
	Pos = Position(round(Pos.x), round(Pos.y), round(Pos.z));
	debug("RawSimulated(): GpsTime=%lld Pos=(%.3f, %.3f, %.3f)\n", 
		GpsTime, Pos.x, Pos.y, Pos.z);
}


bool RawSimulator::NextEpoch()
{
	debug("RawSimulator::NextEpoch\n");
	// Read an epoch from the GPS
	if (gps.NextEpoch() != OK)
		return Error();
	GpsTime = gps.GpsTime;

	// Advance our position by 1 meter in each direction
	if (!Static)
		Pos += Position(1,1,1);
	debug("RawSimulator: Position=(%.3f, %.3f, %.3f)\n",Pos.x,Pos.y,Pos.z);

	// Generate range and phase clock errors
	double PhaseError = Normal(0, .000001) * L1Freq;  // 1 usec of error
	double RangeError = Normal(0, .000001) * C; 
	debug("RawSimulator::NextEpoch(): GpsTime=%lld, PhaseError=%.3f, RangeError=%.3f\n",
		GpsTime, PhaseError, RangeError);

	// repeat for each valid measurement
	for (int s=0; s<MaxSats; s++) {
		obs[s] = gps.obs[s];
		if (!obs[s].Valid || !ephemerides[s].Valid(GpsTime)) continue;
		
		// Calculate the actual range to the satellite
		Position SatPos; double Clock;
		ephemerides[s].SatPos(GpsTime, SatPos, Clock);
		double range = Range(SatPos - Pos);
		debug("Simulator: Pos=(%.3f, %.3f, %.3f)  SatPos=(%.3f, %.3f, %.3f)\n",
			Pos.x,Pos.y,Pos.z,  SatPos.x,SatPos.y,SatPos.z);

		// Generate new measurement values
		obs[s] = gps.obs[s];
		if (obs[s].PR != 0)
			obs[s].PR = range; //Normal(range, 1);
		if (obs[s].Phase != 0)
			obs[s].Phase = range / L1WaveLength; //Normal(range, .01) / L1WaveLength + PhaseError;
		debug("RawSimulator: s=%d range=%.3f PR=%.3f Phase=%.3f\n", 
			                 s, range,      obs[s].PR,  obs[s].Phase);
	}

	return OK;
}

RawSimulator::~RawSimulator(void)
{
}

