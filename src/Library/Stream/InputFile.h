
#ifndef INPUTFILE__INCLUDED
#define INPUTFILE__INCLUDED

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


#include "Stream.h"

class InputFile : public Stream
{
protected:
	FILE* file;

public:
	// The essentials for a stream
	InputFile(const char* name);
	virtual ~InputFile();
	bool Read(byte* buf, size_t len, size_t& actual);
	bool Write(const byte* buf, size_t len) {return OK;}
	bool ReadOnly() {return true;}
	using Stream::Read;
	using Stream::Write;

	// Some extras, to be made private later
	InputFile();
	bool Open(const char* name);
	bool Eof();
};

#endif // !defined(AFX_INPUTFILE_H__EED899C4_235C_4C0C_A584_4F6F7704A7ED__INCLUDED_)

