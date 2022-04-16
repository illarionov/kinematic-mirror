// InputFile reads data from a file
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

#include "InputFile.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
InputFile::InputFile()
{
	file = (FILE*)0;
	ErrCode = true;
}

InputFile::InputFile(const char* name)
{
	ErrCode = Open(name);
}


bool InputFile::Open(const char* name)
{
    file = fopen(name, "rb");
	if (file == NULL) return Error("Unable to open input file %s\n", name);
	return OK;
}

InputFile::~InputFile()
{
	if (file != NULL)
        fclose(file);
}

bool InputFile::Read(byte* buf, size_t len, size_t& actual)
{
	if (file == NULL)
		return Error();
	actual = fread(buf, 1, len, file);
	if (actual != len)
		if (Eof())      return Error("(EOF) Reached end of InputFile\n");
		else            return Error("Problems reading Input file\n");

	debug_buf(7, buf, actual);

	return OK;
}




bool InputFile::Eof()
{
	return file == NULL || feof(file) != 0;
}

