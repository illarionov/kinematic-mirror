
// EphemerisXmit1 implements the ephemeris as transmitted by the satellites
//    Part of Kinematic, a utility for GPS positioning
//
// Copyright (C) 2005  John Morris    kinetic@precision-gps.org
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation, version 2.

//
// This program is distributed in the hope that it will be useful,
// but WITHOUT aNY WaRRaNTY; without even the implied warranty of
// MERCHaNTaBILITY or FITNESS FOR a PaRTICULaR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, Ma  02111-1307, USa.


#include "EphemerisXmit.h"

static const double RelativisticConstant = -4.442807633e-10;

EphemerisXmit::EphemerisXmit(int Sat, const char* description)
:Ephemeris(Sat, description)
{
      iode = -1;
      iodc = -1;
}


EphemerisXmit::~EphemerisXmit()
{
}

bool EphemerisXmit::SatPos(Time xmitTime, Position& XmitPos, double& adjust)
{
	if (!Valid(xmitTime))
		return Error("Ephemeris is not valid\n");

	// Calculate the satellite position, as described in ICD200.
	
	// Semi-major axis
	double a = sqrt_a * sqrt_a;

	// Computed mean motion (rad/sec)
	double n_0 = sqrt(mu)/(a*sqrt_a);

	// time from ephemeris reference epoch
	double t = S(xmitTime - t_oe);

	// corrected mean motion
	double n = n_0 + delta_n;

	// mean anomaly
	double M = m_0 + n*t;

	// Calculate E, Ecentric anomaly, using fix point of Kepler's equation
	double E = M;
	for (int i=0; i<20; i++) 
		E = M + e*sin(E);

	// True anomaly
	double S_nu = sqrt(1-e*e) * sin(E) / (1 - e * cos(E));
	double c_nu = (cos(E) - e) / (1 - e *cos(E));
	double nu = atan2(S_nu, c_nu);
	debug("  S_nu=%.6f  c_nu=%.6f  nu=%.6f\n", S_nu, c_nu, nu);

	// argument of latitude
	double phi = nu + omega;

	// Second harmonic perturbations (latitude, radius, inclination)
	double du = c_uc*cos(2*phi) + c_us*sin(2*phi);
	double dr = c_rc*cos(2*phi) + c_rs*sin(2*phi);
	double di = c_ic*cos(2*phi) + c_is*sin(2*phi);
	debug("  du=%.6f  dr=%.6f  di=%.6f\n", du, dr, di);

	// Corrected latitude, radius, inclination
	double u = phi + du;
	double r = a * (1 - e*cos(E)) + dr;
	double i = i_0 + idot*t + di;
	debug("  u=%.6f  r=%.6f  i=%.6f\n", u, r, i);

	// Positions in orbital plane
	double xdash = r * cos(u);
	double ydash = r * sin(u);
	debug("  xdash = %.3f  ydash=%.3f\n", xdash, ydash); 

	// Corrected longitude of ascending node, including transit time
	double omega_c = omega_0 - GpsTow(t_oe)*OmegaEDot 
		           + (omegadot - OmegaEDot)*t;
	debug("omega_c=%g  omega_0=%g  omegadot=%g  OmegaEDot=%g  t=%g Tow=%g\n",
		omega_c, omega_0, omegadot, OmegaEDot, t, GpsTow(t));

	debug("  ToW*OmegaEDot=%.3f  (O-Oe)*t=%.3f\n", GpsTow(t_oe)*OmegaEDot, (omegadot-OmegaEDot)*t);
	
	// Earth fixed coordinates
	XmitPos.x = xdash*cos(omega_c) - ydash*cos(i)*sin(omega_c);
	XmitPos.y = xdash*sin(omega_c) + ydash*cos(i)*cos(omega_c);
	XmitPos.z = ydash*sin(i);

	// Clock adjustment
	double tc = S(xmitTime - t_oc);
     double adjustClock = (  (a_f2 * tc) + a_f1 ) * tc + a_f0;
	double adjustRelativity = RelativisticConstant * e * sqrt_a * sin(E);
	adjust = adjustClock + adjustRelativity - t_gd;
	debug("adjustclock=%g  adjustRelativity=%g t_gd=%g adjust=%g\n",
		adjustClock,adjustRelativity,t_gd, adjust);
	
	return OK;
}




bool EphemerisXmit::FromRaw(EphemerisXmitRaw& r)
{
    // scale the raw orbit parameters
    m_0 = r.m_0  / p2(31) * PI;
    delta_n = r.delta_n / p2(43) * PI;
    e = r.e / p2(33);
    sqrt_a = r.sqrt_a / p2(19);
    omega_0 = r.omega_0 / p2(31) * PI;
    i_0 = r.i_0 / p2(31) * PI;
    omega = r.omega / p2(31) * PI;
    omegadot = r.omegadot / p2(23) * PI;
    idot = r.idot / p2(43);
    c_uc = r.c_uc / p2(29);
    c_us = r.c_us / p2(29);
    c_rc = r.c_rc / p2(5);
    c_rs = r.c_rs / p2(5);
    c_ic = r.c_ic / p2(29);
    c_is = r.c_is / p2(31);  // verify
    t_oe = ConvertGpsTime(r.wn, r.t_oe * p2(4));
    iode = r.iode;

    // Get the clock parameters
    t_gd = r.t_gd / p2(31);
    t_oc = ConvertGpsTime(r.wn, r.t_oc * p2(5));
    a_f0 = r.a_f0 / p2(55);
    a_f1 = r.a_f1 / p2(43);
    a_f2 = r.a_f2 / p2(31);

    health = r.health;
    acc = SvaccToAcc(r.acc);
    iode = r.iode;

    MinTime = t_oe - 2*NsecPerHour;
    MaxTime = t_oe + 2*NsecPerHour;

    debug("  r.omegadot=%d\n", r.omegadot);
    Display("From Raw");
    return OK;
}
    

bool EphemerisXmit::ToRaw(EphemerisXmitRaw& r)
{
    // scale the raw orbit parameters
    r.m_0 = m_0  * p2(31) / PI;
    r.delta_n = delta_n * p2(43) / PI;
    r.e = e * p2(33);
    r.sqrt_a = sqrt_a * p2(19);
    r.omega_0 = omega_0 * p2(31) / PI;
    r.i_0 = i_0 * p2(31) / PI;
    r.omega = omega * p2(31) / PI;
    r.omegadot = omegadot * p2(23) / PI;
    r.idot = idot * p2(43);
    r.c_uc = c_uc * p2(29);
    r.c_us = c_us * p2(29);
    r.c_rc = c_rc * p2(5);
    r.c_rs = c_rs * p2(5);
    r.c_ic = c_ic * p2(29);
    r.c_is = c_is * p2(31); 
    r.wn = GpsWeek(t_oe);
    r.t_oe = GpsTow(t_oe) / p2(4);
    r.iode = iode;

    // Get the clock parameters
    r.t_gd = t_gd * p2(31);
    r.t_oc = GpsTow(t_oc) / p2(5);
    r.a_f0 = a_f0 * p2(55);
    r.a_f1 = a_f1 * p2(43);
    r.a_f2 = a_f2 * p2(31);

    r.health = health;
    r.acc = AccToSvacc(acc);
    r.iode = iode;

    debug("  r.omegadot=%d\n", r.omegadot);
    Display("To Raw");
    return OK;
}
    


bool EphemerisXmit::AddFrame(NavFrame& f)
{
    EphemerisXmitRaw r;
    f.ToRaw(r);
    return FromRaw(r);
}


void EphemerisXmit::Display(const char* str)
{
	debug("Broadcast Ephemeris[%d] %s  %s  at 0x%p\n", 
		SatIndex, Description, str, this);
	if (iode == -1) {
		debug("    Not Initialized\n");
		return;
	}

	debug("   Mintime=%.0f  Maxtime=%.0f  health=%d accuracy=%.3f\n", 
		      S(MinTime), S(MaxTime), health, acc);
	debug("  iode=%d  t_oc=%.0f   t_oe=%.0f\n", iode, S(t_oc), S(t_oe));
	debug("  a_f2=%g  a_f1=%g  a_f0=%g\n", a_f2, a_f1, a_f0);

	debug("  mu=%g omega_0=%g omegadot=%g  idot=%g\n",
		     mu,     omega_0,    omegadot,      idot);

	debug("  c_rs=%g  c_rc=%g  c_is=%g  c_ic=%g\n", c_rs, c_rc, c_is, c_ic);
	debug("  c_us=%g  c_uc=%g\n", c_us, c_uc);

	debug("  sqrt_a=%g  e=%g  m_0=%g  delta_n=%g\n", sqrt_a, e, m_0, delta_n);
	debug("  i_0=%g\n", i_0);
}



int EphemerisXmit::AccToSvacc(double acc)
{
    int i;
    for (i=0; i<15; i++)
        if (AccuracyIndex[i] > acc)
            break;
    return i;
}

double EphemerisXmit::SvaccToAcc(int svacc)
{
    return AccuracyIndex[svacc];
}

