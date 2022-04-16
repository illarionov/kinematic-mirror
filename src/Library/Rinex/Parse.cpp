// Parse implements text processing routines
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

#include "Parse.h"


bool match(char* line, int column, char* pattern)
{
	char* l = line+column;
	char* p = pattern;
	for (; *p != '\0'; p++,l++)
		if (*p != *l)
			break;
	

	return (*p == '\0');
}


bool IsDigit(int c)
{
	return ('0' <= c && c <= '9');
}


double GetDouble(char* line, int column, int width)
{
	double d = 0;
	double negative = 1;
	double fraction = 0;

	for (int i=column; i < column+width; i++)
		if (line[i] == '-')
			negative = -1;
		else if (line[i] == '.')
			fraction = 1;
		else if (IsDigit(line[i])) {
			fraction = fraction * 10;
			d = d*10 + (line[i] - '0');
		}

	if (fraction == 0)
		fraction = 1;

	return d / fraction * negative;
}

int32 GetInt(char* line, int column, int width)
{
    int32 n = 0;
	int32 negative = 1;

	for (int i=column; i < column+width; i++)
		if (line[i]== '-')
			negative = -1;
		else if (IsDigit(line[i]))
			n = n * 10 + (line[i] - '0');

	return negative * n;
}



int32 ParseSvid(char* line, int col)
/////////////////////////////////////////////////////////
// Convert a Satellite Id string to a satellite number
//     GPS Satellites: 1-32
//     EGNOS           120-152
/////////////////////////////////////////////////////////
{
	if (line[col] == 'S')
		return 100 + GetInt(line, col+1, 2);
	else if (line[col] == 'G' || line[col] == ' ')
		return GetInt(line, col+1,2);
	else 
		return -1;
}

