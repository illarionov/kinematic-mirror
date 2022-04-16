#ifndef STREAM_INCLUDED
#define STREAM_INCLUDED

#include "Util.h"

static const byte NUL = 0;
static const byte SOH = 1;
static const byte STX = 2;
static const byte ETX = 3;
static const byte EOT = 4;
static const byte ACK = 5;
static const byte NL = 10;
static const byte CR = 13;
static const byte DLE = 16;

extern int StreamValidBaud[];

class Stream 
{
protected:
	bool ErrCode;

public:
    Stream() {ErrCode = Error();}
	virtual ~Stream(){}

    // every subclass must implement these
    virtual bool Read(byte* buf, size_t len, size_t& actual) = 0;
    virtual bool Write(const byte* buf, size_t len) = 0;
	virtual bool ReadOnly() = 0;

    // Common routines for all streams
    bool GetError() {return ErrCode;}
    bool Read(byte& b)
	    {return Read(&b, 1);}
    bool Read(byte* buf, size_t len);
    bool Write(byte b)
	    {byte buf=b; return this->Write(&buf, 1);}	
	bool Write(const char* buf)
	    {return Write((byte*)buf, strlen(buf));}


    bool ReadLine(char* line, size_t max=128);
    bool WriteLine(const char* line);
    bool ReadNmea(char* line);
    bool WriteNmea(const char* line);
	bool SkipLine();
	bool Printf(const char* format, ...);
	bool VPrintf(const char* format, va_list args);
	bool PrintSvid(int s);
	bool PrintTime(Time t);

    // Some subclasses may implement these
    virtual bool SetBaud(int baud) {return OK;}
    virtual bool GetBaud(int& baud) {return OK;}
    virtual bool SetFraming(int32 DataBits, int32 Parity, int32 StopBits){return OK;}
    virtual bool SetTimeout(int msec){return OK;}
	virtual bool Purge() {return OK;}
    virtual int FindBaudRate(const char* query, const char* response, int* BaudRates=StreamValidBaud)
	    {return 110;}
	
	bool QueryResponse(const char *query, const char* response, int TooMany=0);
    bool AwaitString(const char* response, int TooMany=0);
};


class DummyStream : public Stream
{
public:
	DummyStream() {ErrCode = Error();}
	~DummyStream() {}
	bool Read(byte* buf, size_t len, size_t& actual)
	    {return Error("DummyStream::Read - Not implemented\n");}
    bool Write(const byte* buf, size_t len)
	    {return Error("DummyStream::Write - Not implemented\n");}
	bool ReadOnly() {return true;}
};



#endif

