
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



#include "CommSSF.h"

CommSSF::CommSSF(Stream& s)
    : Comm(s)
{
	ErrCode = com.GetError();
}


bool CommSSF::PutBlock(Block& b)
{
    return Error("CommSSF: can only read, not write!\n");
}


bool CommSSF::GetBlock(Block& b)
{
	debug(4,"CommSSF::GetBlock (begin)\n");
	
      // Get the type (little endian)
      byte high, low;
      if (com.Read(low) != OK) return Error();
      if (com.Read(high) != OK) return Error();
      b.Id = (((uint16)high)<<8) + low;

      // Get the length (little endian)
      if (com.Read(low) != OK) return Error();
      if (com.Read(high) != OK) return Error();
      b.Length = (((uint16)high)<<8) + low - 2;

      if (b.Length < 0 || b.Length > b.Max) 
          return Error("SSF::GetBlock - bad block length (%d)\n");

      // Get the data
      if (com.Read(b.Data, b.Length) != OK) return Error();

      // Read the last two bytes. These are an offset to the previous record
      //   allowing the file to be read backwards. The offset is length+8
      if (com.Read(low) != OK) return Error();
      if (com.Read(high) != OK) return Error();
      if ( (((uint16)high)<<8)+low != b.Length+8 && high != 0 && low != 0) 
           debug("SSF::Bad backward pointer - expecting %d, got %d  low=%02x high=%02x\n",
                   b.Length+8, (((uint16)high)<<8)+low, low, high);

      b.Display("Read SSF block");
      return OK;
}
      
	

CommSSF::~CommSSF()
{
}



