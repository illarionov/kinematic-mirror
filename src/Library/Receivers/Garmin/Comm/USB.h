// 
#ifndef USB_INCLUDED
#define USB_INCLUDED
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



#include <stdio.h>
//#include <tchar.h>
#include <windows.h>
#include <objbase.h>
#include <setupapi.h> // Need to explicitly link with setupapi.lib
#include <winioctl.h>

#include "Util.h"
#include "Thread.h"

// Maximum packet sizes the USB driver currently handles.
static const size_t MaxBufferSize = 2048; // largest read/write to driver
static const size_t MaxAsyncSize = 64;    // largest async buffer


class USB  
{
public:
	uint16 UsbPacketSize;                     // USB transfer size

	USB():Handle(NULL){}
	USB(GUID guid);
	virtual ~USB();
	bool GetError();

	bool Open(GUID guid);
	bool Open(const char *path);

	bool Write(byte* buf, size_t size);
	bool Read(byte* buf, size_t MaxSize, size_t& size);
	bool ReadAsync(byte* buf, size_t MaxSize, size_t& size);

private:
	bool ErrCode;
	HANDLE Handle;

	bool WriteOverlapped(byte* buf, size_t size, size_t& actual);
    bool ReadOverlapped(byte* buf, size_t size, size_t& actual);
    bool ReadAsyncOverlapped(byte* buf, size_t size, size_t& actual);
};



#endif // !defined(AFX_USB_H__B9F426BE_2FA2_41A7_9647_D3EEA11DDD60__INCLUDED_)

