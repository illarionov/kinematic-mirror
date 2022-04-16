#ifndef DGPSREFERENCE_INCLUDED
#define DGPSREFERENCE_INCLUDED

#include "util.h"
#include "Frame.h"
#include "RtcmOut.h"
#include "RawReceiver.h"
#include "EphemerisXmit.h"
#include "LinearEquation.h"

class DgpsObservation
{
public:
	Time time;
	double PR[MaxSats];

public:
	DgpsObservation();
	virtual ~DgpsObservation();
};

class DgpsHistory
{
public:
	static const int MaxHist=90;
	int count;
	DgpsObservation obs[MaxHist];

public:
	DgpsHistory();
	bool AddEpoch(RawReceiver& gps);
	virtual ~DgpsHistory();
};

class DgpsEquations  // should be subclass of LinearEquations<MaxChannels*2,MaxChannels*90>
{
protected:
	int N;
	int M;
	double A[MaxChannels*2][MaxChannels*90];  // too big? make dynamic?
	double B[MaxChannels*90];
	int ColumnToSatellite[MaxChannels*2];

	Position Pos;
	Ephemerides& eph;

public:
	DgpsEquations(bool Selected[MaxSats], Position& p, Ephemerides& e);
	bool AddEpoch(DgpsObservation& o);
	bool SolveCorrections(double range[MaxSats], double rate[MaxSats]);
	virtual ~DgpsEquations();

private:

};



class DgpsCorrector
{
protected:
	Position Pos;
	bool ErrCode;
	DgpsHistory Hist;

public:
	DgpsCorrector(Position& pos);
	bool AddEpoch(RawReceiver& gps);
	bool SelectSatellites(bool Selected[MaxSats]);
	bool GetCorrections(Time now, Ephemerides& eph, double range[MaxSats], double rate[MaxSats]);
	virtual ~DgpsCorrector();
	bool GetError(){return ErrCode;}

private:
	bool SolveCorrections(bool Selected[MaxSats], Time now, Ephemerides& eph, 
		                  double range[MaxSats], double rate[MaxSats]);
};

class DgpsStation
{
protected:
	RawReceiver& gps;
	RtcmOut rtcm;
	DgpsCorrector dgps;
	bool ErrCode;
	Time NextTime;

public:
	DgpsStation(Stream& out, RawReceiver& g);
	bool OutputEpoch();
	virtual ~DgpsStation();
	bool GetError(){return ErrCode;}

private:

};




#endif

