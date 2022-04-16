
// CommGarminUsb - for communicating with a garmin GPS18 over USB
//    Part of Kinetic, a collection of utilities for GPS positioning
//
// Copyright (C) 2005  John Morris    kinetic@precision-gps.org
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation, version 2
// of the License.
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
// Note.  The early Garmin USB driver had a number of problems. 
//  o It doesn't support overlapped I/O. 
//    The device will open in OVERLAPPED mode, but any overlapped I/O
//    tends to hang.
//  o The driver is unbuffered, so it is easy for slower machines to lose data.
//  o When reading and writing from separate threads, the buffers sometimes get
//    truncated.
//
// Once Garmin fixes these problems, this code can be simplified.
//
// The current workaround was move device initialization to this lower layer and
//   create a "reader" thread to ensure there is always a high priority read
//   pending.  Once initialized, the device is essentially read-only.
///////////////////////////////////////////////////////////////////////////////////////
#include "CommGarminUsb.h"


// Needed to define the GUID structures.
#include <InitGuid.h>
#include <WinIoCtl.h>

DEFINE_GUID(Gps18Guid, 0x2c9c45c2L, 0x8e7d, 0x4c08, 0xa1, 0x2d, 0x81, 0x6b, 
			0xba, 0xe7, 0x22, 0xc0);


CommGarminUsb::CommGarminUsb(const char* x)
{
	ErrCode = Open();
}

bool CommGarminUsb::Open()
{
	if (UsbPort.Open(Gps18Guid))
		return Error("Can't open Garmin USB port");

	// Send a Start Session packet
	if (SendPacket(TRANSPORT_LAYER, START_SESSION))
		return Error("Unable to send first packet");;

	// Wait for a Session Started packet
	if (AwaitPacket(TRANSPORT_LAYER, SESSION_STARTED))
		return Error("Couldn't start talking to Garmin");

	// Identify the GPS unit
	if (PutBlock(PidProductRequest) != OK)  return Error();
    if (AwaitBlock(PidProductData) != OK)
		return Error("Expecting Garmin Gps but protocol doesn't match");

	// verify the device supports the right protocols
	if (AwaitBlock(PidProtocolArray) != OK)
		return Error("Gps doesn't communicate protocol information");

	// Enable the undocumented raw Navigation messages
	//   along with Position and measurement records
	if (PutBlock(0x1c, 0x20, 0x00, -1))
		return Error();

	// Enable the raw measurements
	if (PutBlock(PidCommandData, PositionRecordOn, 0, -1) != OK)
		return Error();
	if (PutBlock(PidCommandData, ReceiverMeasurementOn, 0, -1) != OK)
		return Error();

	return OK;
}


CommGarminUsb::~CommGarminUsb()
{
}


bool CommGarminUsb::GetBlock(Block &b)
{
	debug("CommGarminUsb::GetBlock\n");
	// Read the packet
	Packet_t Packet;
	if (GetPacket(Packet, sizeof(Packet)))
		return Error();

	if (Packet.DataSize > sizeof(b.Data))
		return Error("Oops. Data buffer larger than Garmin's block size");

	// If a lower level protocol, then process it
	if (Packet.ProtocolLevel != APPLICATION_LAYER)
		return Error("Unexpected Protocol level");

	// Move data into the block
	b.Id = Packet.PacketId;
	b.Length = Packet.DataSize;
	for (int i=0; i<b.Length; i++)
		b.Data[i] = Packet.Data[i];
	
	//b.Display("Read from Garmin");
	return OK;
}


bool CommGarminUsb::PutBlock(Block &b)
{
	//b.Display("Garmin: Putblock");
	return SendPacket(b.Id, b.Data, b.Length);
}


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//
// Private methods
//
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////


bool CommGarminUsb::SendPacket(int id, byte* buf, size_t len)
{
	Packet_t Packet = {APPLICATION_LAYER, {0,0,0}, id, {0,0}, len}; // little endian
	for (int i=0; i<len; i++)
		Packet.Data[i] = buf[i];
	return SendPacket(Packet);
}

bool CommGarminUsb::SendPacket(int level, int id)
{
	Packet_t Packet = {level, {0,0,0}, id, {0,0}, 0}; // Little endian machine only
	return SendPacket(Packet);
}

bool CommGarminUsb::SendPacket(Packet_t& p)
{
	// figure out the total size
	size_t size = p.DataSize + EmptySize;
	byte* buf = (byte*)&p;

    // Send packet,
	return UsbPort.Write(buf, size);
}

bool CommGarminUsb::AwaitPacket(int level, int id)
{
	Packet_t Packet;
	do {
		if (GetPacket(Packet, sizeof(Packet)))
			return Error("Error while awaiting packet  id=%d", id);
	} while (Packet.ProtocolLevel != level || Packet.PacketId != id);

	return OK;
}
	

bool CommGarminUsb::GetPacket(Packet_t& p, size_t MaxSize)
{

	byte* buf = (byte*)&p;
	size_t actual;
	size_t total = 0;

    // Read an asynchronous packet
	do {
	     if (UsbPort.ReadAsync(buf, MaxSize, actual))
		     return Error();
	} while (actual == 0);
	if (p.DataSize != actual-EmptySize) {
		debug("packet size was %d, should be %d\n",actual-EmptySize,p.DataSize);
		return Error("Packet data size doesn't match actual size");
	}

	// If a Data Available packet, then do a regular read.
	if (p.ProtocolLevel == TRANSPORT_LAYER && p.PacketId == DATA_AVAILABLE)
		if (UsbPort.Read(buf, MaxSize, actual))
			return Error();

	return OK;
}



