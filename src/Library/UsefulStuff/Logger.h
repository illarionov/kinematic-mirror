#include "OutputFile.h"

class Logger : public OutputFile
{
protected:
	char FileName[128];
	bool FileOpened;
public:
        bool enabled;
	Logger(const char* name);
	bool Write(const byte* buffer, size_t len);
	using OutputFile::Write;
	~Logger();
};


class TimeLogger : public Logger
{
protected:
	Time* TimePtr;

public:
	TimeLogger(const char* name, Time* tp = NULL);
	bool SetTime(Time* tp);
	bool Write(const byte* buffer, size_t len);
	using OutputFile::Write;
	~TimeLogger();
};

bool EventSetTime(Time* tp);
bool Event(const char* fmt, ...);


