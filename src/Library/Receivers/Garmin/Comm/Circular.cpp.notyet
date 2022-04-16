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

#include "Circular.h"

CircularSemaphore::CircularSemaphore(int32 max)
: Circular(max)
{
}

void CircularSemaphore::Push()
{
	//debug("CircularSemaphore::Push  NextPush=%d  NextPop=%d\n",NextPush,NextPop);
	// Wait for a slot to become empty
	m.Lock();
	while (Full())
		c.Wait(m);

	// Note if everything is empty. Might be a consumer sleeping
	bool empty = Empty();

	// Fill up an active slot
	Circular::Push();
	m.Unlock();

	// Wake up a possible consumer
	if (empty)
		c.Wake();

	//debug("CircularSemaphore::Push done\n");
}


int32 CircularSemaphore::Pop()
{
	//debug("CircularSemaphore::Pop   NextPop=%d  NextPush=%d\n",NextPop,NextPush);
	// Wait for a slot to become active
	m.Lock();
	while (Empty())
		c.Wait(m);

	// Make note if full - producer could be sleeping.
	bool full = Full();

	// Remove an active slot
	int index = Circular::Pop();
	m.Unlock();

	// Wake up producer, if any
	if (full)
		c.Wake();
	//debug("CircularSemaphore::Pop (done)  index=%d\n", index);
	return index;
}

CircularSemaphore::~CircularSemaphore()
{
}

