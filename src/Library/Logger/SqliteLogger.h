#ifndef SqliteLogger_included
#define SqliteLogger_included


#include "RawReceiver.h"
#include "sqlite3.h"



class SqliteLogger
{
    bool ErrCode;
    int station_id;
    RawReceiver &gps;
    const char* filename;
    sqlite3* db;

    sqlite3_stmt* begin;
    sqlite3_stmt* insert;
    sqlite3_stmt* end;

    
public:
    bool GetError() {return ErrCode;}
    SqliteLogger(const char* filename, RawReceiver& gps, int station_id);
    bool OutputEpoch();
    virtual ~SqliteLogger();

private:
    bool Initialize();
    bool Cleanup();


};


#endif
