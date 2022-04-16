#ifndef OBSERVATION_INCLUDED
#define OBSERVATION_INCLUDED
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


// This is a basic raw GPS measurement with the following adjustments
//  o Pseudo-range and phase data are adjusted to the epoch
//  o Phase and PR are the same sign.
//    When one increases, the other usually does as well.
//  o The phase ambiguity is set so 
//       phase * L1 Wavelength ==(roughly) Pseudorange
//
// These adjustments make little difference to the mathematics,
//   but they give a clearer view of the data to human readers.
//
// To consider: 
//    - report phase in meters instead of cycles.
//    - report "slip count" instead of slip flag.
//    - report standard deviation of measurement
//
class RawObservation
{
public:
	int Sat;           // the satellite index,  SvidToSat(svid)
	bool Valid;        // is data valid?
	bool Slip;         // did we detect a slip?
	double PR;         // Pseudorange, meters.
	double Phase;      // Phase, Cycles.  Moves same direction as PR.
	double SNR;        // Noise in dB
	double Doppler;    // Doppler shift in hz  ie. -delta(Phase)

public:
	inline RawObservation()
		: Valid(false)
	{}
};


#endif // OBSERVATION_INCLUDED

