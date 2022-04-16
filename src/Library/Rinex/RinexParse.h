#ifndef PARSE_INCLUDED
#define PARSE_INCLUDED
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



#include "util.h"

bool match(char* line, int column, char* pattern);
double GetDouble(char* line, int column, int width);
int32 GetInt(char* line, int column, int width);
int32 ParseSvid(char* line, int column);

#endif // !defined(AFX_PARSE_H__AEF62A65_4376_435C_936E_E8BAF2464707__INCLUDED_)

