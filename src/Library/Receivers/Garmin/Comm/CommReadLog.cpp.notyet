// CommReadLog reads binary records from a disc file
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

#include "CommReadLog.h"



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CommReadLog::CommReadLog(const char* path)
: in(path)
{
    ErrCode = in.GetError();
}

bool CommReadLog::GetBlock(Block &b)
{
	size_t actual;
	if (in.Read((byte*)&b.Length, sizeof(b.Length), actual) || actual != sizeof(b.Length))
		return Error();
	if (in.Read((byte*)&b.Id, sizeof(b.Id), actual) || actual != sizeof(b.Id))
		return Error();
	if (in.Read(b.Data, b.Length, actual) || actual != b.Length)
		return Error();

	return OK;
}



bool CommReadLog::PutBlock(Block &b)
{
	return OK;
}


CommReadLog::~CommReadLog()
{

}
