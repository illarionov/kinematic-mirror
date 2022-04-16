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
#include "thread.h"

Mutex::Mutex()
{
	InitializeCriticalSection(cs);
}

void Mutex::Lock()
{
	EnterCriticalSection(cs);
}

void Mutex::Unlock()
{
	LeaveCriticalSection(cs);
}

Mutex::~Mutex()
{
	DeleteCriticalSection(cs);
}



Semaphore::Semaphore()
{
	sem = CreateSemaphore(NULL, 1, 2000000000, NULL);
}

void Semaphore::Wait()
{
	WaitForSingleObject(sem, 0);
}

void Semaphore::Wake()
{
	ReleaseSemaphore(sem, 1, NULL);
}

Semaphore::~Semaphore()
{
	CloseHandle(sem);
	sem = NULL;
}



Condition::Condition()
{
	Sleepers = 0;
}


void Condition::Wait(Mutex &mutex)
{

	// Make note we are about to sleep.
	SleepLock.Lock();
	Sleepers++;
	SleepLock.Unlock();

	// Release the application mutex
	mutex.Unlock();

	//  sleep
	sem.Wait();

	// Reacquire the mutex
	mutex.Lock();
}

void Condition::Wake()
{
	if (Sleepers == 0)
		return;

	SleepLock.Lock();
	bool Wakeup = (Sleepers > 0);
	if (Wakeup)
		Sleepers--;
	SleepLock.Unlock();

	if (Wakeup)
		sem.Wake();
}

Condition::~Condition()
{
}




Thread::Thread()
{
	Handle = NULL;
	Priority = 0;
}

bool Thread::SetPriority(int32 priority)
{
	Priority = priority;
	return OK;
}


bool Thread::Start()
{
	Handle = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)&Startup, this, 0, NULL);
	if (Handle == NULL)
		return Error("Unable to create thread");

	// Set the threads priority
	int32 p;
	if (Priority < 0) p = THREAD_PRIORITY_BELOW_NORMAL;
	else if (Priority > 0) p = THREAD_PRIORITY_ABOVE_NORMAL;
	else p = THREAD_PRIORITY_NORMAL;
	if (SetThreadPriority(Handle, p) == 0)
		Error("Unable to set priority");

	return OK;
}

void Thread::Startup(Thread *t)
////////////////////////////////////////////////////////////
// ThreadStartup is the first code executed in the new thread.
////////////////////////////////////////////////////////////////
{
	// Invoke the thread's body
	t->Run();

	// Done
	ExitThread(0);
}


void Thread::Run()
{
	Error("Thread::Run wasn't redefined by subclass.");
}

Thread::~Thread()
{
	if (Handle != NULL)
		CloseHandle(Handle);
}

