

#include "Crc.h"


// Note: can be done a byte at time instead of a bit at a time.
//   Question: is this little-endian specific?
//   Rewrite this if it impacts performance
//   Algorithm from GnuPG rfc2440

Crc24::Crc24()
{
    crc = 0xb704ce;
}

void Crc24::Add(byte b)
{
    crc ^= ((uint32)b<<16);
    for (int i=0; i<8; i++) {
        crc <<= 1;
        if ((crc & 0x1000000) != 0)
            crc ^= 0x1864cfb;
    }
}



void Crc24::Add(byte* buf, size_t len)
{
    for (size_t i=0; i<len; i++)
        Add(buf[i]);
}
    

