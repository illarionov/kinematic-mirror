#ifndef COMMBUFFEREDREAD_INCLUDED
#define COMMBUFFEREDREAD_INCLUDED
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

#include "Comm.h"
#include "Thread.h"
#include "Circular.h"



class CommBufferedRead :public Comm, public Thread
{
public:
	CommBufferedRead(Comm& c);
	virtual bool GetBlock(Block& b);
	virtual bool PutBlock(Block& b);
	virtual ~CommBufferedRead();

private:
	// The actual communications channel
	Comm& comm;

	// Read queue of blocks
	static const int32 QueSize = 64;
	Block block[QueSize];
	bool retcode[QueSize];
	int32 head;
	CircularSemaphore circ;

	// The reader thread
	void Run();
};


#endif // COMMBUFFEREDREAD_INCLUDED

