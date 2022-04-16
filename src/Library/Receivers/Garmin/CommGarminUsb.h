// CommGarminUsb.h: interface for the CommGarminUsb class.
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
#ifndef CommGarminUsb_H_INCLUDED
#define CommGarminUsb_H_INCLUDED

///////////////////////////////////////////////////////////////////////////////
//
// We encountered a number of problems with the Garmin USB driver
//   o Data can be lost if a read isn't pending. The driver has no buffering.
//   o Buffers are lost if another thread writes while we are reading.
//
// To get around these problems,
//   o Created a buffered routine, where a "backend" thread does the reads
//     and sends the data to the frontend.
//   o All initialization (writing) is done by the "backend".
//
// We were using an early release of the driver. It is possible more recent 
//   release have fixed the problem.
//
/////////////////////////////////////////////////////////////////////////////
#include "Comm.h"
#include "Usb.h"
#include "CommGarmin.h"
#include "CommBufferedRead.h"
#include "Stream.h"



static const size_t EmptySize = 12;   // Size of empty packet
#pragma pack( push, 1)
typedef struct {
    byte        ProtocolLevel;
    byte        Reserved1[3];
    uint16      PacketId;
    byte        Reserved2[2];
    uint32      DataSize;
    byte        Data[MaxBufferSize-EmptySize];  // Subtract 12 to match total size
} Packet_t;
#pragma pack( pop)

enum {TRANSPORT_LAYER=0, APPLICATION_LAYER=20};
enum {DATA_AVAILABLE=2, START_SESSION=5, SESSION_STARTED=6}; 


class CommGarminUsb  : public Comm
{
public:
	CommGarminUsb(const char* port = "");
	virtual bool Open();
	virtual ~CommGarminUsb();
    virtual bool GetBlock(Block& b);
	virtual bool PutBlock(Block& b);
	using Comm::PutBlock;

private:
	USB UsbPort;

    bool SendPacket(Packet_t&);
	bool SendPacket(int type, int id);
	bool SendPacket(int id, byte* buf, size_t len);
	bool GetPacket(Packet_t&, size_t);
	bool AwaitPacket(int type, int id);
};




class CommGarminUsbBuffered : public Comm
{
public:
	CommGarminUsbBuffered(): usb(), buffered(usb){ErrCode = buffered.GetError();}
	virtual bool GetBlock(Block& b) {return buffered.GetBlock(b);}
	virtual bool PutBlock(Block& b) {return buffered.PutBlock(b);}
	virtual ~CommGarminUsbBuffered(){}
private:
	CommGarminUsb usb;
	CommBufferedRead buffered;
};

#endif // !defined(AFX_CommGarminUsb_H__F0FD1249_937E_4F58_90A7_FDF46BB0B712__INCLUDED_)

