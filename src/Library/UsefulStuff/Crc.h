#ifndef CrcIncluded
#define CrcIncluded

#include "Util.h"

class Crc24 {
protected:
    uint64 crc;
    byte bytes[3];

public:
    Crc24();
    void Add(byte b);
    void Add(byte *b, size_t length);

    byte* AsBytes()
    {
        bytes[0] = crc>>16;
        bytes[1] = crc>>8;
        bytes[2] = crc;
        return bytes;
    }

    uint32 AsInt() {return crc;}
};
    


#endif
