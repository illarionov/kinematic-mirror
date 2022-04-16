
#include "SqliteLogger.h"


SqliteLogger::SqliteLogger(const char* filename, RawReceiver& gps, int station_id)
                 : filename(filename), gps(gps), station_id(station_id)
{
    debug("SqliteLogger::SqliteLogger(%s)\n", filename);

    // Open the database
    ErrCode = Initialize();
    if (ErrCode != OK) Cleanup();
}


bool SqliteLogger::OutputEpoch()
{
    debug("SqliteLogger::OutputEpoch\n");

    // Calculate the satellite positions, but outside the transaction
    Position pos[MaxSats];
    double adjust[MaxSats];
    for (int s=0; s<MaxSats; s++) {
        adjust[s] = 0;  pos[s] = Position(0);
        if (gps.obs[s].Valid && gps[s].Valid(gps.GpsTime)) 
           gps[s].SatPos(gps.GpsTime, pos[s], adjust[s]);
    }

    // Make it a transaction to improve performance
    sqlite3_step(begin);
    if (sqlite3_reset(begin) != SQLITE_OK)
        return Error("Can't cleanup for 'begin':%s\n", sqlite3_errmsg(db));

    // for each valid observation
    for (int s=0; s<MaxSats; s++) {
        if (!gps.obs[s].Valid) continue;

        // Insert observation into the database
        sqlite3_bind_int(insert, 1, station_id);
        sqlite3_bind_int64(insert, 2, (sqlite3_int64)gps.GpsTime);
        sqlite3_bind_int(insert, 3, SatToSvid(s));
        sqlite3_bind_double(insert, 4, gps.obs[s].PR); 
        sqlite3_bind_double(insert, 5, gps.obs[s].Phase);
        sqlite3_bind_double(insert, 6, gps.obs[s].Doppler);
        sqlite3_bind_double(insert, 7, gps.obs[s].SNR);
        sqlite3_bind_int(insert, 8, gps.obs[s].Slip);

        // Include the satellite information as well
        sqlite3_bind_double(insert, 9, pos[s].x);
        sqlite3_bind_double(insert, 10, pos[s].y);
        sqlite3_bind_double(insert, 11, pos[s].z);
        sqlite3_bind_double(insert, 12, adjust[s]);

        // Insert the new row into the table
        debug("About to insert row: svid=%d\n", SatToSvid(s));
        sqlite3_step(insert);
        if (sqlite3_reset(insert) != SQLITE_OK)
            return Error("Insert Observation failed:%s\n", sqlite3_errmsg(db));
    }

    // Commit the transaction
    sqlite3_step(end);
    if (sqlite3_reset(end) != SQLITE_OK)
        return Error("Can't cleanup for 'end':%s\n", sqlite3_errmsg(db));

    return OK;
}



SqliteLogger::~SqliteLogger()
{
    Cleanup();
}



bool SqliteLogger::Initialize()
{
 
    // Open the database
    if (sqlite3_open(filename, &db) != SQLITE_OK)
        return Error("Can't open database at %s: %s\n", filename, sqlite3_errmsg(db));

    // Create the observation table if not already done
    const char* sql;
    sql = "create table if not exists observation "
                  " (station_id int16, " 
                  "  time       int64, "
                  "  svid       int8, "
                  "  PR         double, "
                  "  phase      double, "
                  "  doppler    double, "
                  "  snr        double, "
                  "  slipped    boolean, "
                  "  sat_x      double, "
                  "  sat_y      double, "
                  "  sat_z      double, "
                  "  sat_t      double); "
          "create index if not exists observation_ix "
              " on observation (time, station_id); ";

    if (sqlite3_exec(db, sql, 0, 0, 0) != SQLITE_OK)
        return Error("Sqlite logger %s can't create observation table: %s\n", 
                       filename, sqlite3_errmsg(db));

    // Prepare an insert statement
    sql = "insert into observation "
                "(station_id, time, svid, PR, phase, doppler,snr, slipped, "
                    " sat_x, sat_y, sat_z, sat_t) "
                "values (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);";
    if (sqlite3_prepare_v2(db, sql, -1, &insert, 0) != SQLITE_OK)
        return Error("Unable to precompile insert stmt: %s\n", sqlite3_errmsg(db));
        
    // Prepare transaction begin and end statements
    if (sqlite3_prepare_v2(db, "BEGIN;", -1, &begin, 0) != SQLITE_OK)
       return Error("Unable to precompile 'begin': %s\n", sqlite3_errmsg(db));
    if (sqlite3_prepare_v2(db, "END;", -1, &end, 0) != SQLITE_OK)
       return Error("Unable to precompiel 'end': %s\n", sqlite3_errmsg(db));

    // Save a copy of the filename for future error messages
    char* tmp = (char*)malloc(strlen(filename)+1);
    if (tmp == 0)
        return Error("Out of memory opening Sqlite file %s\n", tmp);
    strcpy(tmp, filename);
    filename = tmp;

    return OK;
}



bool SqliteLogger::Cleanup()
{
    if (begin != 0)  sqlite3_finalize(begin);
    if (insert != 0) sqlite3_finalize(insert);
    if (end != 0)   sqlite3_finalize(end);
    if (db != 0) sqlite3_close(db);
    if (filename != 0) free((void*)filename);
    begin = 0; insert = 0; end = 0; db = 0; filename = 0;

    return OK;
}

