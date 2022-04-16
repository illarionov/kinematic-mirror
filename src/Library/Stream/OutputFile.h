
#ifndef OUTPUTFILE__INCLUDED
#define OUTPUTFILE__INCLUDED
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

class OutputFile : public Stream
{
protected:
	FILE* file;
public:

	OutputFile(const char* name);
	virtual ~OutputFile();
	bool Write(const byte* buffer, size_t len);
	bool Read(byte* buffer, size_t len, size_t& actual) 
         {actual=0; return Error("Can't read from an Output File\n");}
	bool ReadOnly() {return false;}
	using Stream::Read;
	using Stream::Write;
	OutputFile();
	bool Open(const char* name);
	bool Flush();
};

#endif // !defined(AFX_OUTPUTFILE_H__80C2F09D_D45C_48AF_AF56_4E3F9EA94733__INCLUDED_)

