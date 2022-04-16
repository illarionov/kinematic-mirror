#ifndef COMMAC12_INCLUDED
#define COMMAC12_INCLUDED
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


#include "Rs232.h"
#include "Comm.h"


class CommAC12 : public Comm
{
public:
	CommAC12(Stream& s);
	virtual ~CommAC12();
	virtual bool GetBlock(Block& b);
	virtual bool PutBlock(Block& b);
	using Comm::PutBlock;

private:
	bool Init();
	bool Command(const char* cmd);
	bool ReadId(Block& b);
	bool ReadBinaryData(Block& b, int size);
	bool ReadNmeaData(Block& b);
	bool ReadField(Block& b);
	bool Match(Block& b, int start, const char* str);
};



#endif // COMMAC12_INCLUDED

