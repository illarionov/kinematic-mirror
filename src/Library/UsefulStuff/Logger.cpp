#include "Logger.h"
#include "stdio.h"

Logger::Logger(const char* name)
{
	strncpy(FileName, name, sizeof(FileName)-1);
	FileName[sizeof(FileName)-1] = '\0';
	FileOpened = false;
    ErrCode = OK;
    enabled = true;
}


bool Logger::Write(const byte* buffer, size_t len)
{
    // Open the log file if it hasn't already been opened
	if (!FileOpened)
		if (Open(FileName) != OK) 
			return Error("Can't open log file %s\n",FileName);
	FileOpened = true;

	// Write the buffer
      if (enabled)
	    if (OutputFile::Write(buffer, len) != OK) return Error();

	return OK;
}

Logger::~Logger()
{
}


TimeLogger::TimeLogger(const char* name, Time* tp)
: Logger(name),TimePtr(tp)
{
}


bool TimeLogger::SetTime(Time* tp)
{
	TimePtr = tp;
	return OK;
}

bool TimeLogger::Write(const byte* buffer, size_t len)
{
	// If the clock was set, display the time
	if (TimePtr != NULL) {
	    int32 day, month, year, hour, min, sec, nsec;
	    TimeToDate(*TimePtr, year, month, day); 
	    TimeToTod(*TimePtr, hour, min, sec, nsec);

	    // display the time. We can't call Printf without going into a loop.
	    char timebuf[30];
	    sprintf(timebuf, "%2d/%02d/%04d-%02d:%02d:%02d ", month,day,year,hour,min,sec);
	    if (Logger::Write((const byte*)timebuf, 20) != OK) return Error();
	}
	
	// Otherwise, just lead with spaces
	else
		if (Logger::Write((const byte*)"                    ", 20) != OK) return Error();        
	
	// display the original buffer
	return Logger::Write(buffer, len);
}


TimeLogger::~TimeLogger()
{
}




static TimeLogger EventLog("events.txt");

bool EventSetTime(Time *tp)
{
	return EventLog.SetTime(tp);
}

bool Event(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	debug(1, "Event: "); vdebug(1, fmt, args);
	bool ret = EventLog.VPrintf(fmt, args);
	EventLog.Flush();
	va_end(args);

    return ret;
}

