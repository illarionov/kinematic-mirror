#ifndef EphemerisXmitRaw_included
#define EphemerisXmitRaw_included

#include "Util.h"

struct EphemerisXmitRaw
{
    int32  m_0;        // 2^-31  semicircles
    int16  delta_n;    // 2^-43  semicircles/sec
    uint32 e;          // 2^-33  n/a
    uint32 sqrt_a;     // 2^-19  meters^1/2
    int32  omega_0;    // 2^-31  semicircles
    int32  i_0;        // 2^-31  semicircles
    int32  omega;      // 2^-31  semicircles
    int32  omegadot;   // 2^-43  semicircles/sec
    int16  idot;       // 2^-43  semicircles/sec
    int16  c_uc;       // 2^-29  radians
    int16  c_us;       // 2^-23  radians
    int16  c_rc;       // 2^-5   meters
    int16  c_rs;       // 2^-5   meters
    int16  c_ic;       // 2^-29  radians
    int16  c_is;       // 2^-29  radians
    uint16 t_oe;       // 2^4    seconds from start of week
    uint8  iode;  


    uint8 iodc;
    int8   t_gd;       // 2^-31   seconds
    uint16 t_oc;       // 2^4    seconds
    int8  a_f0;        // 2^-55 
    int16 a_f1;        // 2^-43
    int32 a_f2;        // 2^-31
      
    uint8  health;
    uint8  acc;  // TODO: check these out!!!
    uint32 wn;

    int svid;
};

#endif
