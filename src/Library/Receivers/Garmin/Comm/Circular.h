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

#ifndef CIRCULAR_INCLUDED
#define CIRCULAR_INCLUDED
#pragma once
#include "Thread.h"

/////////////////////////////////////////////////////////////
//
// Circular handles the indexing for a circular que.
//
// Circular is based on the rotating thingy that used to be 
//    used in restaurants. New orders go in on one side, 
//    old orders come out in the other.
//
//        NextPush - index where the next order will go
//        Push()   - move to the next index for the next order
//        = pop()  - remove an index, fifo order. 
//                   The slot will be unused until after the next pop.
//
////////////////////////////////////////////////////////////////////

class Circular
{
public:
	Circular(int32 nrslots) {NrSlots = nrslots; NextPush=0; NextPop=0;}
	int32 NextPush;
	int32 NextPop;
	void Push() {NextPush = Next(NextPush);}
	int32 Pop() {int old=NextPop; NextPop = Next(NextPop);return old;}
	~Circular(){}
	bool Empty() {return NextPush == NextPop;}
	bool Full()  {return Next(NextPush) == NextPop;}
private:
	int32 NrSlots;
	int32 Next(int32 i) {return (i+1 >= NrSlots)? 0: i+1;}
};


////////////////////////////////////////////////////////////
//
// CircularSemaphore is like a Circular index, but it blocks
//
////////////////////////////////////////////////////////////

class CircularSemaphore: public Circular
{
public:
	CircularSemaphore(int32 max);
	void Push();
	int32 Pop();
	~CircularSemaphore();
private:
	Condition c;
	Mutex m;
};

#endif // CIRCULAR_INCLUDED

