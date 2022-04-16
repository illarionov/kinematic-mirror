#ifndef NtripServer_included
#define NtripServer_included

#include "Util.h"
#include "Socket.h"

class NtripClient: public Socket
{
public:

    NtripClient(const char* host, const char* port, const char *mount, 
                const char *user, const char *password);
    ~NtripClient();

protected:
    bool ParseHeader();
};


#endif
