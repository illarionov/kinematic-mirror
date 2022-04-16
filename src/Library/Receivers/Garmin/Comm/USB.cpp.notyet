// usb implements a usb interface as defined by the Garmin USB driver
//    Part of Kinetic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    kinetic@precision-gps.org
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

//////////////////////////////////////////////////////////////////////////
//
// Routines to interface with the Garmin USB driver.
//
// This driver has a number of quirks
//   o The data arrives as asynchronous data, which is read with I/O control.
//   o The driver is unbuffered, so it easy to lose input data.
//   o Any write operations overlapping a read will kill the read with error 31.
//
// The solution is
//   o Spawn a thread which always keeps a read pending
//   o If a read has error 31, reissue the read.
//     (This only happens during startup when we issue commands to the Garmin)
//   o Que up the completed reads so the main thread can process them when convenient
//
////////////////////////////////////////////////////////////////////////////////////





#include "USB.h"

#include <InitGuid.h>
#include <WinIoCtl.h>

#define IOCTL_ASYNC_IN        CTL_CODE (FILE_DEVICE_UNKNOWN, 0x850, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_USB_PACKET_SIZE CTL_CODE (FILE_DEVICE_UNKNOWN, 0x851, METHOD_BUFFERED, FILE_ANY_ACCESS)

// Forward references. 


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

USB::USB(GUID guid)
{
	ErrCode = Open(guid);
}

bool USB::Open(GUID guid)
{
	// Get the device class information
    HDEVINFO DevInfo = SetupDiGetClassDevs(&guid, NULL, NULL, 
		DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);

	// Get the first interface of the class
	SP_DEVICE_INTERFACE_DATA InterfaceData;
    InterfaceData.cbSize = sizeof(InterfaceData);
    if(!SetupDiEnumDeviceInterfaces(DevInfo,NULL, &guid,0, &InterfaceData))
		return Error("Couldn't find any devices of the USB class");

	// Get the size of the interface details
	DWORD NrBytes;
	SetupDiGetDeviceInterfaceDetail(DevInfo,&InterfaceData,NULL,0,&NrBytes,NULL);


	// Get the interface details
	PSP_INTERFACE_DEVICE_DETAIL_DATA DevDetailData = 
		(PSP_INTERFACE_DEVICE_DETAIL_DATA)malloc( NrBytes );
	if (DevDetailData == NULL) return Error("Out of memory");
    DevDetailData->cbSize = sizeof( SP_INTERFACE_DEVICE_DETAIL_DATA );

	SP_DEVINFO_DATA DevInfoData = { sizeof( SP_DEVINFO_DATA ) };
    SetupDiGetDeviceInterfaceDetail(DevInfo,&InterfaceData,DevDetailData,NrBytes,
		NULL, &DevInfoData);
                         
	// Open the device using the path from the details
	bool ret = Open(DevDetailData->DevicePath);

	free(DevDetailData);
	return ret;
}

bool USB::Open(const char *path)
{
	//debug("Open: path=%s\n", path);

	Handle = CreateFile(path, GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED, NULL );

	if (Handle == INVALID_HANDLE_VALUE)
		return Error("Can't open USB device %s\n", path);

	// Get the USB packet size.
	DWORD NrBytes;
	DeviceIoControl(Handle, IOCTL_USB_PACKET_SIZE, 0, 0, 
		&UsbPacketSize, sizeof(UsbPacketSize), &NrBytes, NULL);
	//debug("The USB packet size is %d  NrBytes=%d\n", UsbPacketSize, NrBytes);

	return OK;
}

USB::~USB()
{
    if (Handle != INVALID_HANDLE_VALUE)
		CloseHandle(Handle);
}

bool USB::GetError()
{
	return ErrCode;
}




bool USB::Write(byte* buf, size_t size)
{
	//debug("Write: size=%d\n", size);
	//debug_buf(buf, size);
	size_t actual;

	if (size > MaxBufferSize)
		return Error("Haven't implemented writing large USB packets");

	// Send the buffer
	if (WriteOverlapped(buf, size, actual) != OK)
	    return Error("Overlapped Write to USB failed\n");

	//debug("usb::write  size=%d  actual=%d  error=%d(0x%x)\n",size,actual,GetLastError(),GetLastError());

	if (actual != size)
		return Error("Write didn't complete the buffer");

	// If an even number of USB packets, then add a zero-length write to flush.
	if (size % UsbPacketSize == 0) 
		WriteOverlapped(buf, 0, actual);

	return OK;
}

bool USB::Read(byte* buf, size_t MaxSize, size_t& actual)
{
	if (actual >= MaxBufferSize || MaxSize >= MaxBufferSize)
		return Error("Large USB read buffers not implemented yet");

	return ReadOverlapped(buf, MaxSize, actual);
}

bool USB::ReadAsync(byte* buf, size_t MaxSize, size_t& total)
{
	size_t actual;
	//debug("ReadAsync: MaxSize=%d\n", MaxSize);

	// Read buffers until we get a partial read
	total = 0;
	do {
		if (total+MaxAsyncSize >= MaxSize)
			return Error("Read buffer might not be large enough");

		if (ReadAsyncOverlapped(buf+total, MaxSize, actual) != OK)
			return Error("Problems reading async data from USB");
	  
		total+=actual;

	} while (actual == MaxAsyncSize);

	//debug("ReadAsync (done) total=%d  actual=%d\n", total,actual);
    return OK;
}

bool USB::WriteOverlapped(byte* buf, size_t size, size_t& actual)
{
	OVERLAPPED overlapped = {0};
	int success = WriteFile(Handle, buf, size, (DWORD*)&actual, &overlapped);
	if (!success && GetLastError() == ERROR_IO_PENDING)
		success = GetOverlappedResult(Handle, &overlapped, (DWORD*)&actual, true);
	
	if (!success) return Error();
	return OK;
}

bool USB::ReadOverlapped(byte* buf, size_t size, size_t& actual)
{
	OVERLAPPED overlapped = {0};
	int success = ReadFile(Handle, buf, size, (DWORD*)&actual, &overlapped);
	if (!success && GetLastError() == ERROR_IO_PENDING)
		success = GetOverlappedResult(Handle, &overlapped, (DWORD*)&actual, true);

	//debug("ReadOverlapped: success=%d  actual=%d  errno=%d\n",success,actual,GetLastError());
	if (!success) return Error(); 
	return OK;
}


bool USB::ReadAsyncOverlapped(byte* buf, size_t size, size_t& actual)
{
	// Repeat until we read data that wasn't aborted
	int success;
	do { 		
	    OVERLAPPED overlapped = {0};
	    success = DeviceIoControl(Handle,IOCTL_ASYNC_IN,0,0,buf, 
			            size, (DWORD*)&actual, &overlapped);
		if (!success && GetLastError() == ERROR_IO_PENDING)
		    success = GetOverlappedResult(Handle, &overlapped, (DWORD*)&actual, true);
 
		//if (success)debug("ReadAsyncOverlapped actual=%d\n", actual); 
		//else        debug("ReadAsyncOverlapped (failed) error=%d\n",GetLastError());

    // end "Repeat ..."   Note that error 31 means the read was aborted
	} while (!success && GetLastError() == 31);

	if (!success) return Error();
	return OK;
}
	

