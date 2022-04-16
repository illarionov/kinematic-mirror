#ifndef Socket_Included
#define Socket_Included

#include "Util.h"
#include "Stream.h"
#include <sys/types.h>
#include <sys/socket.h>



class Socket: public Stream {
public:
    Socket();
    Socket(const char* host, const char* port);
    virtual ~Socket();
    
    bool SetTimeout(uint32 msec);
    virtual bool Connect(const char* host, const char* port);
    virtual bool Connect(struct sockaddr& addr);
    virtual bool Close();

    bool Read(byte*, size_t size, size_t& actual);
    bool Write(const byte*, size_t size);
    bool ReadOnly() {return false;}

protected:
    int fd;
    bool Init();
 
};




#endif 
