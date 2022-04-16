#include "Socket.h"

#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>


Socket::Socket()
{
    ErrCode = Init();
}


Socket::Socket(const char* name, const char* port)
{
    ErrCode = Init() || Connect(name, port);
}



bool Socket::Init()
/////////////////////////////////////////////////////////////////
// Init is the real constructor for a socket. 
//   common code for the various constructors.
///////////////////////////////////////////////////////////////////
{
    debug("Socket::Init() - creating a new socket\n");
    fd = ::socket(PF_INET, SOCK_STREAM, 0);
    if (fd == -1) return SysError("Can't create a new tcp socket\n");

    // Default to a 10 second timeout
    return SetTimeout(10000);
}


// Note: Cygwin's getaddrinfo doesn't like hints, and linux needs it
#ifdef __CYGWIN__
static struct addrinfo* hint = NULL;
#else
static struct addrinfo hint[1] = {{AI_ADDRCONFIG, AF_INET, SOCK_STREAM}};
#endif

bool Socket::Connect(const char* host, const char* port)
{
    debug("Socket::Connect(%s, %s)\n", host, port);

    // Look up address of the host and port
    struct addrinfo* info;
    int err = getaddrinfo(host, port, hint, &info);
    if (err != 0)
        return Error("Unable to connect to %s:%s  - %s\n",
                         host, port, gai_strerror(err));
    freeaddrinfo(info);
 
    // Connect to the address
    return Connect(*info->ai_addr);
}


bool Socket::Connect(struct sockaddr& addr)
{
    // Connect to the address
    if (::connect(fd, &addr, sizeof(addr)) == -1)
        return SysError("Socket cannot connect\n");

    // Set socket options.
    int temp = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &temp, sizeof(temp)) == -1)
        return SysError("Can't set keepalive socket option\n");
    static const struct linger l = {1, 5};
    if (setsockopt(fd, SOL_SOCKET, SO_LINGER, &l, sizeof(l)) == -1)
        return SysError("Can't set socket linger option\n");
 
    debug("Socket::Connect - successfully connected\n");
    return OK;
}


bool Socket::Read(byte* buf, size_t size, size_t& actual)
{
    debug(3, "Socket::Read size=%d\n", size); 

    actual = ::read(fd, buf, size);
    if (actual == -1) return SysError("Reading from socket\n");

    debug_buf(3, buf, actual);
    return OK;
}


bool Socket::Write(const byte* buf, size_t size)
{
    debug(3, "Socket::Write: size=%d\n", size); 
    debug_buf(3, buf, size);

    // We don't want to receive a signal, so use "send" instead of "write"
    if (::send(fd, buf, size, MSG_NOSIGNAL) != size) 
        return SysError("Can't write to socket\n");
    return OK;
}



bool Socket::Close()
{
    if (::close(fd) == -1) 
        ErrCode = SysError("Can't close socket. fd=%d\n", fd);
    fd = -1;
    return OK;
}

Socket::~Socket()
{
    Close();
}


bool Socket::SetTimeout(uint32 msec)
{
    struct timeval to = {msec/1000, (msec%1000)*1000};
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to)) == -1 ||
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to)) == -1)
        return SysError("Can't set socket timeout\n");
    // TODO: should we set linger?
    return OK;
}




