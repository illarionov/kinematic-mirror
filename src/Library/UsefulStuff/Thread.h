#ifndef THREAD_INCLUDED
#define THREAD_INCLUDED
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


#include "util.h"
#include <windows.h>  // MOVE ELSEWHERE!


class Mutex
{
public:
	Mutex();
	void Lock();
	void Unlock();
	~Mutex();
private:
	CRITICAL_SECTION cs[1];
};


class Semaphore
{
public:
	Semaphore();
	void Wait();
	void Wake();
	~Semaphore();
private:
	HANDLE sem;
};



class Condition
{
public:
	Condition();
	void Wait(Mutex& m);
	void Wake();
	~Condition();

private:
	Semaphore sem;
	Mutex SleepLock;
	int32 Sleepers;
};




class Thread
{
public:
	Thread();
	bool Start();
	virtual ~Thread(void);

	bool SetPriority(int32 priority);

protected:
	// Each thread type redefines this method.
	virtual void Run();

private:
	HANDLE Handle;
	static void Startup(Thread*);
	int32 Priority;
};


#endif // THREAD_INCLUDED

