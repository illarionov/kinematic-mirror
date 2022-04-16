#ifndef EphemerisXmit_INCLUDED
#define EphemerisXmit_INCLUDED

#include "EphemerisXmitRaw.h"
#include "Ephemeris.h"
#include "NavFrame.h"

class EphemerisXmit : public Ephemeris 
{
public:
    double m_0;      // Mean Anomaly at reference time
    double delta_n;  // Mean Motion Difference from Computed Value
    double e;        // Eccentricity
    double sqrt_a;   // Square root of the Semi-Major Axis
    double omega_0;  // Longitude of Ascending Node of orbit Plan at Weekly Epoch
    double i_0;      // Inclination Angle at Reference Time
    double omega;       // Argument of Perigee
    double omegadot; // Rate of Right Ascension
    double idot;     // Rate of Inclination angle
    double c_uc;     // Amplitude of the Cosine Harmonic Correction to Latitude
    double c_us;     // Amplitude of the Sine Harmonic correction to Latitude
    double c_rc;     // Amplitude of the Cosine Harmonic Correction to Orbit Radius
    double c_rs;     // Amplitude of the Sine Harmonic Correction to Orbit Radius
    double c_ic;     // Amplitude of the Cosine Harmonic Correction to Angle of Inclination
    double c_is;     // Amplitude of the Sine Harmonic Correction to Angle of Inclination
    Time   t_oe;     // Reference time
    uint8  iode;     // Issue of Data (Ephemeris)

    double t_gd;
    Time   t_oc;
    double a_f0;
    double a_f1;
    double a_f2;
    uint8  iodc;

    uint8  health;
    double acc;

    bool FromRaw(EphemerisXmitRaw& r);
    bool ToRaw(EphemerisXmitRaw& r);
    bool AddFrame(NavFrame& f);

    bool SatPos(Time t, Position& satpos, double& adjust);

    void Display(const char* title="");

    EphemerisXmit(int sat, const char* desription);
    ~EphemerisXmit();

    bool Valid(Time t) {return (MinTime <= t && t <= MaxTime);}
    double Accuracy(Time t) {return acc;}


protected:
    double SvaccToAcc(int svacc);
    int AccToSvacc(double acc);

};

    static const double AccuracyIndex[16] = {2.4, 3.4, 4.85, 6.85, 9.65, 
	  13.65, 24, 48, 96, 192, 384, 768, 1536, 3072, 6144, INFINITY};

#endif
