#ifndef NEWRAWRECEIVER_INCLUDED
#define NEWRAWRECEIVER_INCLUDED

#include "Stream.h"
#include "RawReceiver.h"

RawReceiver* NewRawReceiver(const char* model, const char* port, const char* log = NULL);
Stream* NewInputStream(const char* port, const char* log = NULL);
Stream* NewOutputStream(const char* port);

#endif

