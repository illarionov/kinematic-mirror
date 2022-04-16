#ifndef NtripServer_included
#define NtripServer_included

#include "Util.h"
#include "Socket.h"

class NtripServer: public Socket
{
public:

    NtripServer(const char* host, const char* port, const char *mount, 
                const char *user, const char *password);
    ~NtripServer();

protected:
    bool ParseHeader();
};


#endif
