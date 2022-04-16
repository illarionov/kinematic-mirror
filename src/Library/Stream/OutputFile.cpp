// OutputFile writes data to a disk file
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

#include "OutputFile.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
OutputFile::OutputFile()
{
	ErrCode = OK;
	file = stdout;
}

OutputFile::OutputFile(const char* name)
{
	file = NULL;
	ErrCode = Open(name);
}

bool OutputFile::Open(const char* name)
{
	if (file != stdout && file != NULL)
		fclose(file);
    file = fopen(name, "wbc");
	if (file == NULL)
		return Error("Unable to open file %s for output\n", name);
	return OK;
}

OutputFile::~OutputFile()
{
    fclose(file);
}


bool OutputFile::Write(const byte* buf, size_t len)
{
	if (fwrite(buf, 1, len, file) != len)
		return Error("Problem writing output file\n");
	return OK;
}

bool OutputFile::Flush()
{
	fflush(file);
	return OK;
}

