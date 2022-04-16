#pragma once
#include "RawReceiver.h"
#include "Ephemeris.h"

class RawSimulator : public RawReceiver
{
protected:
	RawReceiver& gps;
	Ephemerides& ephemerides;
	bool Static;

public:
	bool NextEpoch();
	RawSimulator(RawReceiver& rcv, Ephemerides& e, bool stationary=true);
	virtual ~RawSimulator(void);
};

