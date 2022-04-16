#ifdef NOTYET

#include "BufferIn.h"

BufferIn::BufferIn(Stream& in, size_t size)
    in(str), circ(size)
{
    ErrCode = Open(in, size);
}



BufferIn::Open(Stream& in, size_t size)
{
    ErrCode = in.GetError();
    if (ErrCode != OK) return Error();

    (// Initialize the input buffer
    buf = malloc(size);
    if (buf == 0) 
        return Error("malloc: Can't allocate %d bytes for buffer\n", size);

    // Spawn a child to read the actual stream
    Thread t(Child);
    if (t.GetError() != OK) return Error("Can't spawn reader thread\n");

    return OK;
}


void BufferIn::Child(void *param)
{

    // Repeat forever
    forever {

        // Read some data 
        byte buf[512];
        size_t actual;
        if (in.Read(buf, sizeof(buf), actual) != OK)
            return Error("Child failed to read\n"); // And notify parent

        for (size_t i=0; i<actual; i++)
            circ.Push(buf[i]);
    }
}



#endif
