#ifndef STREAMCOPY_INCLUDED
#define STREAMCOPY_INCLUDED

#include "Stream.h"

class StreamCopy: public Stream
{
protected:
	Stream& In;
	Stream& Copy;
public:
	StreamCopy(Stream& in, Stream& copy) : In(in), Copy(copy)
	    {ErrCode = In.GetError() || Copy.GetError();}
	~StreamCopy(){}
	using Stream::Read;
	using Stream::Write;
	virtual bool ReadOnly() {return In.ReadOnly();}
    
	// Read copies the data to the copy stream
	bool Read(byte* buf, size_t len, size_t& actual)
	    {return In.Read(buf, len, actual) || Copy.Write(buf, len);}

	// All other operations get passed to the original stream
	bool Write(const byte* buf, size_t len) {return In.Write(buf, len);};
	virtual bool SetBaud(int baud) {return In.SetBaud(baud);}
    virtual bool GetBaud(int& baud) {return In.GetBaud(baud);}
    virtual int FindBaudRate(const char* query, const char* response, int* BaudRates)
	    {return In.FindBaudRate(query, response, BaudRates);}
    virtual bool SetFraming(int32 DataBits, int32 Parity, int32 StopBits)
	    {return In.SetFraming(DataBits, Parity, StopBits);}
    virtual bool SetTimeout(int msec){return In.SetTimeout(msec);}
	virtual bool Purge() {return In.Purge();}
};


#endif

