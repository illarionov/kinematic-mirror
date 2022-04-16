#ifndef RS232_INCLUDED
#define RS232_INCLUDED
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

#ifndef WINDOWS
typedef int FileHandle;
#endif

#include "Stream.h"
#include "Util.h"


class Rs232 : public Stream
{
protected:
	FileHandle Handle;
        char Name[40];

public:
	Rs232(const char* name);
	virtual ~Rs232(void);

	using Stream::Read;
	using Stream::Write;
	bool Read(byte* buf, size_t len, size_t& actual);
	bool Write(const byte* buf, size_t len);
	bool ReadOnly() { return false; }

	bool Purge(void);
	bool SetBaud(int baud);
	bool GetBaud(int& baud);
	bool SetFraming(int32 DataBits, int32 Parity, int32 StopBits);
	bool SetTimeout(int msec);
    int FindBaudRate(const char* query, const char* response, 
		             int* BaudRates=StreamValidBaud);

private:
	bool GetTimeout(int& msec);
	bool Open(const char* name);
	void Close(void);
	void ClearErrors(void);
        bool Flush();

};


#endif

