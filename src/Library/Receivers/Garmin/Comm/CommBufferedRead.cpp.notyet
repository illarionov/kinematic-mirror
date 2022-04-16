
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

#include "CommBufferedRead.h"
#include "Thread.h"
#include "Circular.h"



CommBufferedRead::CommBufferedRead(Comm& c)
    : comm(c), circ(QueSize)
{
	// Check for comm errors
	ErrCode = comm.GetError();
	if (ErrCode) return;

	// Give it high priority since we don't want to miss data
	SetPriority(2);

	// Create the thread
	ErrCode = Start();
}

bool CommBufferedRead::GetBlock(Block& b)
{
	// Wait until we have data in the queue
	int32 i = circ.Pop();
	b = block[i];
	return retcode[i];
}

bool CommBufferedRead::PutBlock(Block& b)
{
	return comm.PutBlock(b);
}



CommBufferedRead::~CommBufferedRead(void)
{
}




void CommBufferedRead::Run()
{ 
	// Repeat until exit
	while (1) {

		// Read a block from the real comm
		retcode[circ.NextPush] = comm.GetBlock(block[circ.NextPush]);

		debug("Pushing block %d. Id=%d  ErrCode=%d\n", 
			circ.NextPush, block[circ.NextPush].Id, retcode[circ.NextPush]);

		// Push the block to the consumer
		circ.Push();
	}
}

