#ifndef RTCMOUT_INCLUDED
#define RTCMOUT_INCLUDED
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


#include "Comm.h"


class CommRtcm3 : public Comm
{
public:
	CommRtcm3(Stream& out);
	virtual bool PutBlock(Block& blk);
        virtual bool GetBlock(Block& blk);
	virtual ~CommRtcm3(void);
};


#endif // RTCMOUT_INCLUDED

