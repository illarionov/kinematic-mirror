
// CommWriteLog copyies blocks of binary data to a file before passing them on
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



#include "CommWriteLog.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CommWriteLog::CommWriteLog(Comm &c, const char* path)
: comm(c), out(path)
{
	ErrCode = comm.GetError();
	debug("CommWriteLog()  path=%s  ErrCode=%d\n", path, ErrCode);
}

CommWriteLog::~CommWriteLog()
{
}

bool CommWriteLog::GetBlock(Block& b)
////////////////////////////////////////////////////////////////////////
// GetBlock reads a block from the underlying Comm and logs it to a file
//////////////////////////////////////////////////////////////////////////
{
	if (comm.GetBlock(b))
		return Error();
	out.Write((byte*)&b.Length, sizeof(b.Length));
	out.Write((byte*)&b.Id, sizeof(b.Id));
	out.Write(b.Data, b.Length);
	return OK;
}

bool CommWriteLog::PutBlock(Block& b)
{
	return comm.PutBlock(b);
}

