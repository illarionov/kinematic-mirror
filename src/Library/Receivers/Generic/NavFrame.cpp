
//    Part of Kinematic, a utility for GPS positioning
//
// Copyright (C) 2005  John Morris    www.precision-gps.org
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



#include "util.h"
#include "NavFrame.h"


bool NavFrame::ToRaw(EphemerisXmitRaw& r)
{
	// Make sure the 3 subframes are the same version
	int NewIode = GetField(2,61,8);
	int NewIode2 = GetField(3,271,8);
	int NewIode3 = GetField(1,211,8);
	if (NewIode != NewIode2 || NewIode != NewIode3)
		return Error("Inconsistent Ephemeris for iode=(%d,%d,%d)\n",
		                NewIode,NewIode2,NewIode3);

     int NewIodc = (GetField(1,83,2)<<8) + NewIode;

     // If this is the same ephemeris we already have, then done
     if (NewIode == r.iode && NewIodc == r.iodc)
         return OK;

	// Make note we have a new ephemeris
	debug("New raw xmit ephemeris  old=%d  new=%d\n", r.iode, NewIode);
	r.iode = NewIode;
        r.iodc = NewIodc;

	// Read the clock data, subframe 1
	r.wn = GetField(1,61,10);
	r.acc = GetField(1,73,4);
	r.health = GetField(1,77,6);
	r.t_gd = GetSigned(1,197,8);
	r.t_oc = GetField(1,219,16);
	r.a_f2 = GetSigned(1,241,8);
	r.a_f1 = GetSigned(1,249,16);
	r.a_f0 = GetSigned(1,271,22);

	// Read the ephemeris, subframe 2
	r.c_rs = GetSigned(2,69,16);
	r.delta_n = GetSigned(2,91,16);
	r.m_0 = (GetField(2,107,8)<<24) + GetField(2,121,24);
	r.c_uc = GetSigned(2,151,16);
	r.e = (GetField(2,167,8)<<24) + GetField(2,181,24);
	r.c_us = GetSigned(2,211,16);
	r.sqrt_a = (GetField(2,227,8)<<24) + GetField(2,241,24);
	r.t_oe = GetField(2,271,16);
	int32 FitIntervalFlag = GetField(2,286,1);
	int32 aODO = GetField(2,287,5);

	// Read the ephemeris, subframe 3
	r.c_ic = GetSigned(3,61,16);
	r.omega_0 = (GetField(3,77,8)<<24) + GetField(3,91,24);
	r.c_is = GetSigned(3,121,16);
	r.i_0 = (GetField(3,137,8)<<24)+ GetField(3,151,24);
	r.c_rc = GetSigned(3,181,16);
	r.omega = (GetField(3,197,8)<<24) + GetField(3,211,24);
	r.omegadot = GetSigned(3,241,24);
	r.idot = GetSigned(3,279,14);

	return OK;
}




inline uint32 ExtractBits(uint32 word, int32 BitNr, int32 NrBits)
///////////////////////////////////////////////////////////////
// ExtractBits extracts a bit field where bit 0 is MSB
//////////////////////////////////////////////////////////////
{
	return (word<<BitNr)>>(32-NrBits);
}

inline int32 ExtractSigned(int32 word, int32 BitNr, int32 NrBits)
/////////////////////////////////////////////////////////////////
// ExtractSigned extracts a signed bit field where bit 0 is MSB
////////////////////////////////////////////////////////////////////
{
	return (word<<BitNr)>>(32-NrBits);
}



inline uint32 Parity(uint32 w)
/////////////////////////////////////////////////////////////////
// Parity calculates the parity (0 or 1) of a word
/////////////////////////////////////////////////////////////////
{
	w ^= w>>16;
	w ^= w>>8;
	w ^= w>>4;
	w ^= w>>2;
	w ^= w>>1;
	return (w&1);
}





NavFrame::NavFrame()
/////////////////////////////////////////////////////////////////////////
// A NavFrame holds one frame of navigation data sent by a GPS satellite
/////////////////////////////////////////////////////////////////////////
{
}

uint32 NavFrame::GetField(int32 subframe, int32 bitnr, int32 nrbits)
//////////////////////////////////////////////////////////////////
// GetField extracts an unsigned data field from a navigation frame
//   The parameters match those in the IPS-200 spec
///////////////////////////////////////////////////////////////////
{
	// Note: bit 1 is the most significant of the first 30 bit word.
	int32 index = (bitnr-1) / NavBitsPerWord 
		        + (subframe-1) * NavWordsPerSubframe;
	int32 bit = ((bitnr-1) % NavBitsPerWord) + 8;
	return ExtractBits(Data[index], bit, nrbits);
}

int32 NavFrame::GetSigned(int32 subframe, int32 bitnr, int32 nrbits)
////////////////////////////////////////////////////////////////////
// GetSigned extracts a signed data field from a navigation frame
////////////////////////////////////////////////////////////////////
{
	// Note: bit 1 is the most significant of the first 30 bit word.
	int32 index = (bitnr-1) / NavBitsPerWord 
		        + (subframe-1) * NavWordsPerSubframe;
	int32 bit = ((bitnr-1) % NavBitsPerWord) + 8;
	return ExtractSigned(Data[index], bit, nrbits);
}

void NavFrame::Display(const char* str)
///////////////////////////////////////////////////////////////
// Display prints out the Navigation Frame for debug purposes
///////////////////////////////////////////////////////////////
{
	debug("2,%s\n", str);
	for (int subframe=1; subframe<=5; subframe++) {
		debug(2,"%2d: ", subframe);
		for (int bit=1; bit <= 300; bit+=30)
			debug(2," %06x", GetField(subframe, bit, 24));
		debug(2,"\n");
	}
}
	


bool ExtractData(NavWord w, uint32& PrevD29, uint32& PrevD30, uint32& data)
/////////////////////////////////////////////////////////////////
// ExtractData extracts the data bits from a raw Navagation word
//   returns OK if the data is valid
/////////////////////////////////////////////////////////////////
{
	// Extract the 24 data bits, complementing if necessary
	if (PrevD30) data = ExtractBits(~w, 2, 24);
	else         data = ExtractBits( w, 2, 24);

	// compute the desired parity bits
	uint32 D25 = PrevD29 ^ Parity(data & 0xec7cd2);
	uint32 D26 = PrevD30 ^ Parity(data & 0x763e69);
	uint32 D27 = PrevD29 ^ Parity(data & 0xbb1f34);
	uint32 D28 = PrevD30 ^ Parity(data & 0x5d8f9a);
	uint32 D29 = PrevD30 ^ Parity(data & 0xaec7cd);
	uint32 D30 = PrevD29 ^ Parity(data & 0x2dea27);
	uint32 parity = (D25<<5) + (D26<<4) + (D27<<3) + (D28<<2) + (D29<<1) + D30;

	// compare with actual parity
	if (ExtractBits(w, 26, 6) != parity) {
		debug("ExtractData(bad parity): w=0x%0x  parity=0x%02x  PrevD29=%d PrevD30=%d\n",
			w, parity, PrevD29, PrevD30);
		PrevD30 = 0;
		PrevD29 = 0;
		return Error();
	}

	// Keep track of the previous parity bits
	PrevD30 = ExtractBits(w, 31, 1);
	PrevD29 = ExtractBits(w, 30, 1);
	return OK;
}

bool ExtractData(NavWord w, uint32& data)
////////////////////////////////////////////////////////////////
// Extracts from word where two highest bits contain 
//    parity information from the previous word
////////////////////////////////////////////////////////////////
// Note: Might corrupt data in next word if D30 gets corrupted.
{
	uint32 PrevD29 = (w>>31)&1;
	uint32 PrevD30 = (w>>30)&1;
	return ExtractData(w, PrevD29, PrevD30, data);
}



NavFrameBuilder::NavFrameBuilder()
//////////////////////////////////////////////////////////////
// NavFrameBuilder builds up a navigation frame, one word at a time
///////////////////////////////////////////////////////////////////
{
	NextWord=0;
	D29 = 0;
	D30 = 0;
}


inline bool IsTlm(uint32 d) {return ExtractBits(d, 8, 8) == 0x8b;}
inline uint32 SubFrame(uint32 d) {return ExtractBits(d, 27, 3);}



bool NavFrameBuilder::AddWord(NavWord w, int32 bitnr)
{
	debug("AddWord:  w=0x%8x  bitnr=%d\n", w, bitnr);
	// Note: bitnr starts at 1 for the first word.
	// If we finished the previous frame, start a new one
	if (Complete())
		NextWord = 0;

	// Figure out which word we just received
	int32 index = ((bitnr-1)/30)%50;

	// Extract the data bits and check parity. 
    // CASE: parity error.  Start over.
	if (ExtractData(w, Data[NextWord]) != OK)
		NextWord = 0;

	// CASE: this is the wrong word. Ignore. (Some units retransmit subframes)
	else if (index != NextWord)
		;

	// Otherwise, keep the word
	else
		NextWord++;

	return OK;
}

bool NavFrameBuilder::AddWord(NavWord w)
///////////////////////////////////////////////////////////////////
// AddWord adds the next raw navigation word to a navigation frame
//////////////////////////////////////////////////////////////////
{
	// If we finished the previous frame, start a new one
	if (Complete())
		NextWord = 0;
	uint32& d = Data[NextWord];  // for convenience
	debug("AddWord: this=%p  w=0x%08x  NextWord=%d\n", this, w, NextWord);
	
	// Extract the data bits and check parity. 
    // CASE: parity error.  Start over.
	if (ExtractData(w, D29, D30, d) != OK)
		NextWord = 0;

	// CASE: TLM word at first subframe. Keep it.
	else if ( IsTlm(d) && NextWord==0 )
		NextWord++;

	// CASE: TLM for subsequent subframe. Should match first TLM. Keep it.
	else if ( IsTlm(d) && (NextWord%10)==0 && d==Data[0])
		NextWord++;

	// CASE: HOW with appropriate subframe number.  Keep it.
	else if ( (NextWord%10)==1 && SubFrame(d)==NextWord/10+1
		&& D29==0 && D30==0)
		NextWord++;

	// CASE: end of subframe. 
	else if ( (NextWord%10)==9 && D29==0 && D30==0)
		NextWord++;

	// CASE: regular data word.  Keep it.
	else if ( (NextWord%10) != 0 && (NextWord%10) != 1)
		NextWord++;

	// OTHERWISE: Not in sync. Start over
	else
		NextWord = 0;
	
	if (NextWord == 50)
		Display("Frame completed\n");
	if (NextWord == 0)
		debug("AddWord: Starting New frame\n");
	return OK;
}

bool NavFrameBuilder::Complete()
////////////////////////////////////////////////////////////////
// Complete tells us we have completed a navigation frame
////////////////////////////////////////////////////////////////
{
	return NextWord == NavWordsPerFrame;
}


