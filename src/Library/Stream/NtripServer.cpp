#include "NtripServer.h"
#include "Parse.h"


NtripServer::NtripServer(const char* host, const char* port, const char *mount, 
                         const char *user, const char *passwd)
: Socket(host, port)
{
    debug("NtripServer::NtripServer(%s, %s, %s, %s, %s) ErrCode=%d\n",
                host, port, mount, user, passwd, ErrCode);
    if (ErrCode != OK) return;

    ErrCode = Printf("SOURCE %s/%s\r\n", passwd, mount) 
        || Printf("Source-Agent NTRIP 1.0 Precision-gps.org\r\n") 
        || Printf("\r\n") 
        || ParseHeader();

    if (ErrCode != OK)
         Error("NtripServer protocol error starting up\n");
}


bool NtripServer::ParseHeader()
{
    // repeat until we read a blank line
    char line[256];
    forever {

        if (ReadLine(line, sizeof(line)) != OK)
            return Error("Can't read Ntrip header from Caster\n");

        // parse the first token in the line
        Parse p(line);
        p.Next(" "); 

        // Look for "ICY 200". Good news. 
        if (p == "ICY") {
           if (p.Next(" ") != "200") 
               return Error("ParseHeader: Bad code - %s\n", line);
        }

        // Look for "ERROR - ...."  Bad news
        else if (p == "ERROR") 
            return Error("Ntrip Caster says: %s\n", line);

        else if (p == "")
            break;
    }

    return OK;
}



NtripServer::~NtripServer()
{
}
